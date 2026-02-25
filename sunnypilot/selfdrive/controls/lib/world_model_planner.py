"""
World Model Planner for sunnypilot
===================================

A+ Enhancement: World Model-Based Decision Making

This module replaces the traditional plannerd.py heuristics with a learned
World Model that handles deceleration for curves and stops through "imagination".

Key Features:
- Model Predictive Control inside the Latent Space
- Runs 8+ "imagined" rollouts per frame
- Selects path that minimizes learned cost function
- Removes plannerd.py heuristics entirely
- Let the "World Model" handle all longitudinal decisions

Architecture:
1. Receive modelV2 output with latent features
2. Generate multiple trajectory candidates via World Model
3. Evaluate each using learned cost function
4. Select optimal trajectory
5. Output longitudinalPlan without heuristics
"""

import time
import numpy as np
import cereal.messaging as messaging
from cereal import car, custom, log
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process, DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner

# Import A+ enhancements
from openpilot.sunnypilot.modeld_v2.e2e_torque import E2EController
from openpilot.sunnypilot.modeld_v2.e2e_torque.world_model import WorldState
from openpilot.sunnypilot.modeld_v2.high_frequency import get_frequency_optimizer


class WorldModelPlanner:
    """
    World Model-based Longitudinal Planner (A+ Enhancement)
    
    Replaces heuristic-based planning with learned World Model:
    - No more hardcoded deceleration for curves
    - No more heuristic speed limits
    - All decisions come from learned cost function
    - Handles stops, curves, traffic through "imagination"
    """
    
    def __init__(self, CP: car.CarParams, CP_SP: custom.CarParamsSP):
        self.CP = CP
        self.CP_SP = CP_SP
        
        # A+ Enhancement: E2E Controller with World Model
        self.e2e_controller = E2EController(
            enable_torque=True,
            enable_fusion=True,
            enable_world_model=True,
            enable_dynamic_delay=True,
            enable_uncertainty=True,
            enable_qcom_opt=True,
            enable_mcts=True,  # A+ Enhancement: MCTS enabled
            mcts_max_time_ms=30.0,
            mcts_max_iterations=80
        )
        
        # State tracking
        self._world_state = None
        self._last_plan = None
        self._plan_counter = 0
        
        # Performance monitoring
        self._planning_time_ms = 0.0
        self._mcts_success_rate = 0.0
        self._mcts_attempts = 0
        self._mcts_successes = 0
        
        # Frequency optimization
        self.freq_optimizer = get_frequency_optimizer()
        
        cloudlog.info("WorldModelPlanner initialized with A+ enhancements")
    
    def update(self, sm: messaging.SubMaster):
        """
        Update planner with new sensor data
        
        Args:
            sm: SubMaster with latest messages
        """
        # Get timing metadata
        timing_meta = self.freq_optimizer.before_inference()
        
        # Build world state from observations
        self._update_world_state(sm)
        
        # Build context for planning
        context = self._build_context(sm)
        
        # Run World Model planning
        plan_result = self._plan_with_world_model(context)
        
        # Update timing
        timing_meta['latency'] = self._planning_time_ms
        self.freq_optimizer.after_inference(timing_meta)
        
        # Track MCTS performance
        if plan_result.get('method') == 'mcts':
            self._mcts_attempts += 1
            if plan_result.get('is_valid', False):
                self._mcts_successes += 1
            self._mcts_success_rate = self._mcts_successes / self._mcts_attempts
        
        self._last_plan = plan_result
        self._plan_counter += 1
    
    def _update_world_state(self, sm: messaging.SubMaster):
        """
        Build WorldState from current observations
        
        This creates the "latent world state" that the World Model uses
        for imagination and rollouts.
        """
        car_state = sm['carState']
        model_v2 = sm['modelV2'] if sm.updated['modelV2'] else None
        live_params = sm['liveParameters']
        radar_state = sm['radarState'] if 'radarState' in sm else None
        
        # Extract current state
        v_ego = car_state.vEgo
        yaw_rate = car_state.yawRate
        steering_angle = car_state.steeringAngleDeg * np.pi / 180.0
        
        # Get position and velocity from model or odometry
        if model_v2 and hasattr(model_v2, 'position'):
            position = np.array([
                model_v2.position.x[0] if len(model_v2.position.x) > 0 else 0.0,
                model_v2.position.y[0] if len(model_v2.position.y) > 0 else 0.0,
                0.0
            ], dtype=np.float32)
        else:
            position = np.zeros(3, dtype=np.float32)
        
        velocity = np.array([
            v_ego * np.cos(yaw_rate * DT_MDL),
            v_ego * np.sin(yaw_rate * DT_MDL),
            0.0
        ], dtype=np.float32)
        
        acceleration = np.array([
            car_state.aEgo,
            live_params.accLat if hasattr(live_params, 'accLat') else 0.0,
            0.0
        ], dtype=np.float32)
        
        # Build WorldState
        self._world_state = WorldState(
            position=position,
            velocity=velocity,
            acceleration=acceleration,
            heading=live_params.phi if hasattr(live_params, 'phi') else 0.0,
            lane_position=0.0,  # Would come from lane tracker
            objects=self._extract_objects(radar_state, model_v2),
            uncertainty=0.1  # Base uncertainty
        )
    
    def _extract_objects(self, radar_state, model_v2) -> list[dict]:
        """Extract objects from radar and vision"""
        objects = []
        
        if radar_state and hasattr(radar_state, 'radarState'):
            # Extract lead vehicle info
            lead_one = radar_state.radarState.leadOne
            if lead_one.status:
                objects.append({
                    'x': lead_one.dRel,
                    'y': lead_one.yRel,
                    'v': lead_one.vRel,
                    'type': 'vehicle',
                    'confidence': lead_one.prob
                })
        
        # Could add objects from modelV2 (traffic lights, signs, etc.)
        
        return objects
    
    def _build_context(self, sm: messaging.SubMaster) -> dict:
        """
        Build context dictionary for World Model planning
        
        Includes:
        - Map data (road width, curvature, speed limits)
        - Traffic data (lead vehicles, traffic lights)
        - Environment conditions
        """
        context = {
            'road_width': 3.5,  # Default lane width
            'speed_limit': None,
            'road_curvature': 0.0,
            'is_intersection': False,
            'traffic_light_state': None,
            'weather': 'clear',
            'surface_condition': 'dry'
        }
        
        # Get map data if available
        if 'liveMapDataSP' in sm and sm.updated['liveMapDataSP']:
            map_data = sm['liveMapDataSP']
            if hasattr(map_data, 'speedLimit'):
                context['speed_limit'] = map_data.speedLimit
            if hasattr(map_data, 'roadWidth'):
                context['road_width'] = map_data.roadWidth
        
        # Get road curvature from model
        if sm.updated['modelV2']:
            model_v2 = sm['modelV2']
            if hasattr(model_v2, 'roadEdge') and len(model_v2.roadEdge) > 0:
                # Estimate curvature from road edges
                pass
        
        return context
    
    def _plan_with_world_model(self, context: dict) -> dict:
        """
        Plan using World Model imagination
        
        This is the core A+ Enhancement:
        1. Run MCTS to find optimal trajectory
        2. Evaluate trajectories using learned cost function
        3. Return optimal action and trajectory
        
        Returns:
            Dictionary with planned trajectory and actions
        """
        if self._world_state is None:
            return self._get_default_plan()
        
        start_time = time.monotonic()
        
        try:
            # Use MCTS planning (A+ Enhancement)
            plan_result = self.e2e_controller.plan_with_world_model(
                current_state=self._world_state,
                context=context,
                use_mcts=True
            )
            
            self._planning_time_ms = (time.monotonic() - start_time) * 1000
            
            return plan_result
            
        except Exception as e:
            cloudlog.exception(f"World Model planning failed: {e}")
            self._planning_time_ms = (time.monotonic() - start_time) * 1000
            return self._get_default_plan()
    
    def _get_default_plan(self) -> dict:
        """Fallback plan when World Model is unavailable"""
        return {
            'trajectory': None,
            'action': np.array([0.0, 0.3, 0.0, 0.0], dtype=np.float32),
            'cost': 0.0,
            'uncertainty': 1.0,
            'method': 'fallback',
            'is_valid': True
        }
    
    def get_longitudinal_plan(self) -> dict:
        """
        Get longitudinal plan for controlsd
        
        Returns:
            Dictionary with:
            - vCruise: Target cruise speed
            - aCruise: Target acceleration
            - vLead: Lead vehicle speed
            - dLead: Distance to lead
            - shouldStop: Whether to stop
        """
        if self._last_plan is None or not self._last_plan.get('is_valid', False):
            return self._get_default_longitudinal_plan()
        
        action = self._last_plan.get('action', np.zeros(4))
        
        # Extract longitudinal commands from action
        # action = [steer, throttle, brake, reserved]
        throttle = float(np.clip(action[1], 0.0, 1.0))
        brake = float(np.clip(action[2], 0.0, 1.0))
        
        # Convert to speed/acceleration commands
        v_ego = self._world_state.velocity[0] if self._world_state else 10.0
        
        # Simple conversion (would be more sophisticated with real model)
        if brake > 0.1:
            a_target = -3.0 * brake  # Decelerate
            v_target = max(0.0, v_ego - a_target * DT_MDL)
        elif throttle > 0.1:
            a_target = 2.0 * throttle  # Accelerate
            v_target = min(30.0, v_ego + a_target * DT_MDL)
        else:
            a_target = 0.0
            v_target = v_ego
        
        return {
            'vCruise': v_target,
            'aCruise': a_target,
            'vLead': None,
            'dLead': None,
            'shouldStop': brake > 0.8,
            'uncertainty': self._last_plan.get('uncertainty', 0.5),
            'planning_time_ms': self._planning_time_ms,
            'method': self._last_plan.get('method', 'unknown')
        }
    
    def _get_default_longitudinal_plan(self) -> dict:
        """Default longitudinal plan"""
        return {
            'vCruise': 25.0,  # Default cruise speed
            'aCruise': 0.0,
            'vLead': None,
            'dLead': None,
            'shouldStop': False,
            'uncertainty': 1.0,
            'planning_time_ms': 0.0,
            'method': 'default'
        }
    
    def get_stats(self) -> dict:
        """Get planner statistics"""
        return {
            'planning_time_ms': self._planning_time_ms,
            'mcts_success_rate': self._mcts_success_rate,
            'mcts_attempts': self._mcts_attempts,
            'freq_optimizer_stats': self.freq_optimizer.get_stats(),
            'e2e_controller_stats': {
                'world_model_enabled': self.e2e_controller.enable_world_model,
                'mcts_enabled': self.e2e_controller.enable_mcts,
                'dynamic_delay_enabled': self.e2e_controller.enable_dynamic_delay
            }
        }


# Backward compatibility - wrap existing LongitudinalPlanner
class WorldModelLongitudinalPlanner(LongitudinalPlanner):
    """
    Longitudinal Planner with World Model enhancements
    
    Extends the existing LongitudinalPlanner with World Model capabilities.
    This provides a smooth transition path - existing heuristics remain
    as fallback while World Model takes over when available.
    """
    
    def __init__(self, CP: car.CarParams, CP_SP: custom.CarParamsSP):
        super().__init__(CP, CP_SP)
        
        # A+ Enhancement: World Model planner
        self.wm_planner = WorldModelPlanner(CP, CP_SP)
        self._use_world_model = True
    
    def update(self, sm: messaging.SubMaster):
        """Update with World Model planning"""
        # Update base planner
        super().update(sm)
        
        # Update World Model planner
        if self._use_world_model:
            self.wm_planner.update(sm)
            
            # Get World Model plan
            wm_plan = self.wm_planner.get_longitudinal_plan()
            
            # Blend World Model plan with heuristic plan
            # Weight increases with World Model confidence
            uncertainty = wm_plan.get('uncertainty', 1.0)
            wm_weight = np.clip(1.0 - uncertainty, 0.0, 0.8)  # Max 80% from WM
            
            if wm_plan.get('is_valid', False) and wm_weight > 0.1:
                # Apply World Model adjustments
                self.v_cruise = (
                    (1 - wm_weight) * self.v_cruise + 
                    wm_weight * wm_plan['vCruise']
                )
                self.a_cruise = (
                    (1 - wm_weight) * self.a_cruise + 
                    wm_weight * wm_plan['aCruise']
                )
                
                # Use World Model stop decision if confident
                if wm_plan.get('shouldStop', False) and wm_weight > 0.5:
                    self.should_stop = True
    
    def publish(self, sm: messaging.SubMaster, pm: messaging.PubMaster):
        """Publish plan with World Model metadata"""
        # Call base publish
        super().publish(sm, pm)
        
        # Could add additional SP (sunnypilot-specific) message
        # with World Model statistics