"""
Hierarchical Planning Enhancement for Sunnypilot2

This module implements advanced trajectory planning with behavioral planning layer,
prediction-based planning, and tactical decision-making to achieve Tesla FSD-like
capabilities within Comma 3x hardware limits.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Any
from enum import Enum
import math

from cereal import log
from cereal.messaging import SubMaster
from opendbc.car.structs import car
from openpilot.common.swaglog import cloudlog
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import MPC_COST_LAT, get_accel_from_plan, smooth_value, get_curvature_from_plan
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.sunnypilot.controls.lib.advanced_fusion import integrate_with_planner, AdvancedRadarCameraFusion


class BehavioralState(Enum):
    """Behavioral planning states."""
    LANE_FOLLOWING = 0
    LANE_CHANGING_LEFT = 1
    LANE_CHANGING_RIGHT = 2
    STOPPING = 3
    ADAPTIVE_CRUISE = 4
    TRAFFIC_JUNCTION = 5
    CONSTRUCTION_ZONE = 6
    ROUNDABOUT = 7
    FREE_FOLLOWING = 8  # When no clear lead vehicle


class TrajectoryType(Enum):
    """Types of planned trajectories."""
    NORMAL = 0
    LANE_CHANGE_LEFT = 1
    LANE_CHANGE_RIGHT = 2
    STOPPING = 3
    EMERGENCY = 4
    YIELDING = 5


class BehavioralPlanner:
    """High-level behavioral planning module."""
    
    def __init__(self):
        self.current_behavior = BehavioralState.LANE_FOLLOWING
        self.previous_behavior = BehavioralState.LANE_FOLLOWING
        self.behavior_confidence = 1.0
        self.lane_change_timer = 0.0
        self.lane_change_timeout = 8.0  # Max seconds for lane change
        
        # Lane change parameters
        self.lane_change_target_offset = 3.7  # Lane width in meters
        self.lane_change_rate = 0.05  # Rate of lateral movement (m/s)
        
        # Traffic light and stop sign detection (would interface with vision model)
        self.traffic_light_state = log.TrafficLight.State.unknown
        self.traffic_light_distance = float('inf')
        self.stop_sign_detected = False
        self.stop_sign_distance = float('inf')
    
    def update(self, sm: SubMaster, mpc_solution: Dict[str, np.ndarray]) -> BehavioralState:
        """
        Update behavioral state based on current situation.
        
        Args:
            sm: SubMaster with current sensor data
            mpc_solution: Current MPC solution for context
            
        Returns:
            Updated behavioral state
        """
        self.previous_behavior = self.current_behavior
        
        # Extract relevant information from sensor data
        car_state = sm['carState']
        v_ego = car_state.vEgo
        a_ego = car_state.aEgo if hasattr(car_state, 'aEgo') else 0.0
        steering_angle = car_state.steeringAngleDeg
        
        # Check for traffic light/stop sign
        self._check_traffic_signals(sm)
        
        # Check for lead vehicles
        lead_status = self._check_leading_vehicles(sm, v_ego)
        
        # Check for lane change capability
        lane_change_ready = self._check_lane_change_conditions(sm, v_ego)
        
        # Update lane change timer
        if self.current_behavior in [BehavioralState.LANE_CHANGING_LEFT, BehavioralState.LANE_CHANGING_RIGHT]:
            self.lane_change_timer += DT_MDL
            if self.lane_change_timer > self.lane_change_timeout:
                self.current_behavior = BehavioralState.LANE_FOLLOWING
                self.lane_change_timer = 0.0
        
        # Decision logic
        new_behavior = self.current_behavior
        
        # Traffic light handling
        if self._should_stop_for_traffic_light(v_ego):
            new_behavior = BehavioralState.STOPPING
        
        # Stop sign handling
        elif self._should_stop_for_stop_sign(v_ego):
            new_behavior = BehavioralState.STOPPING
        
        # Junction handling
        elif self._approaching_junction(sm):
            new_behavior = BehavioralState.TRAFFIC_JUNCTION
        
        # Construction zone
        elif self._detect_construction_zone(sm):
            new_behavior = BehavioralState.CONSTRUCTION_ZONE
        
        # Lane changing
        elif sm['selfdriveState'].laneChangeState != log.LaneChangeState.off:
            if sm['selfdriveState'].laneChangeDirection == log.LaneChangeDirection.left:
                new_behavior = BehavioralState.LANE_CHANGING_LEFT
            elif sm['selfdriveState'].laneChangeDirection == log.LaneChangeDirection.right:
                new_behavior = BehavioralState.LANE_CHANGING_RIGHT
        
        # Adaptive cruise control
        elif lead_status['close_lead']:
            new_behavior = BehavioralState.ADAPTIVE_CRUISE
        else:
            new_behavior = BehavioralState.FREE_FOLLOWING
        
        self.current_behavior = new_behavior
        self.behavior_confidence = self._calculate_behavior_confidence(sm)
        
        return self.current_behavior
    
    def _check_traffic_signals(self, sm: SubMaster):
        """Check for traffic lights and stop signs."""
        # This would interface with vision model outputs
        # For now, we'll use mock data
        if 'modelV2' in sm:
            model_v2 = sm['modelV2']
            # Check for traffic light in model output
            if hasattr(model_v2, 'meta'):
                # Extract traffic light information from model metadata
                # This would depend on the specific model output format
                pass
    
    def _check_leading_vehicles(self, sm: SubMaster, v_ego: float) -> Dict[str, Any]:
        """Check for leading vehicles and their impact."""
        lead_info = {
            'close_lead': False,
            'very_close_lead': False,
            'lead_distance': float('inf'),
            'lead_velocity': 0.0,
            'time_headway': float('inf')
        }
        
        if 'radarState' in sm:
            radar_state = sm['radarState']
            if radar_state.leadOne.status:
                lead_info['lead_distance'] = radar_state.leadOne.dRel
                lead_info['lead_velocity'] = v_ego - radar_state.leadOne.vRel
                if radar_state.leadOne.dRel > 0:
                    lead_info['time_headway'] = radar_state.leadOne.dRel / max(1.0, lead_info['lead_velocity'])
                
                # Determine if lead is close enough to affect behavior
                if radar_state.leadOne.dRel < 50.0:  # 50m threshold
                    lead_info['close_lead'] = True
                if radar_state.leadOne.dRel < 25.0:  # Very close lead
                    lead_info['very_close_lead'] = True
        
        return lead_info
    
    def _check_lane_change_conditions(self, sm: SubMaster, v_ego: float) -> bool:
        """Check if lane change conditions are met."""
        # Check if currently in a lane change
        if sm['selfdriveState'].laneChangeState != log.LaneChangeState.off:
            return False  # Already in lane change
        
        # Check vehicle speed (don't lane change at very low speeds)
        if v_ego < 10.0:  # Below 36 km/h
            return False
        
        # Check radar for safe lane change conditions
        if 'radarState' in sm:
            radar_state = sm['radarState']
            # Check for vehicles in blind spots or close behind
            # This would require more detailed radar processing
            pass
        
        return True
    
    def _should_stop_for_traffic_light(self, v_ego: float) -> bool:
        """Determine if should stop for traffic light."""
        if self.traffic_light_distance < float('inf') and self.traffic_light_state == log.TrafficLight.State.red:
            # Calculate if we can stop safely
            stopping_distance = self._calculate_stopping_distance(v_ego)
            if self.traffic_light_distance < stopping_distance + 5.0:  # 5m safety margin
                return True
        return False
    
    def _should_stop_for_stop_sign(self, v_ego: float) -> bool:
        """Determine if should stop for stop sign."""
        if self.stop_sign_distance < float('inf'):
            stopping_distance = self._calculate_stopping_distance(v_ego)
            if self.stop_sign_distance < stopping_distance + 5.0:  # 5m safety margin
                return True
        return False
    
    def _approaching_junction(self, sm: SubMaster) -> bool:
        """Check if approaching traffic junction."""
        # Check vision model for junction detection
        if 'modelV2' in sm:
            model_v2 = sm['modelV2']
            # Check for junction-specific model outputs
            # This would depend on model capabilities
            pass
        return False
    
    def _detect_construction_zone(self, sm: SubMaster) -> bool:
        """Detect construction zone."""
        # Check for construction zone indicators
        # This would use vision model outputs for cone/barrier detection
        return False
    
    def _calculate_stopping_distance(self, v_ego: float) -> float:
        """Calculate stopping distance based on vehicle speed."""
        # Simple physics model: d = v² / (2 * a_max)
        max_deceleration = 4.0  # m/s²
        reaction_time = 1.0  # s
        stopping_distance = (v_ego ** 2) / (2 * max_deceleration) + v_ego * reaction_time
        return stopping_distance
    
    def _calculate_behavior_confidence(self, sm: SubMaster) -> float:
        """Calculate confidence in current behavioral decision."""
        confidence = 1.0
        
        # Reduce confidence if sensor data is inconsistent
        # For example, if model and radar disagree significantly
        
        return min(1.0, max(0.0, confidence))


class PredictionBasedPlanner:
    """Prediction-based planning considering multiple vehicle futures."""
    
    def __init__(self):
        self.prediction_horizon = 5.0  # seconds
        self.prediction_steps = 20  # number of prediction steps
        self.uncertainty_threshold = 0.7  # threshold for high uncertainty
        
        # Vehicle dynamics model parameters
        self.max_lat_accel = 3.0  # m/s²
        self.max_long_accel = 4.0  # m/s²
        self.max_jerk = 5.0  # m/s³
    
    def predict_ego_trajectory(self, 
                              initial_state: Dict[str, float], 
                              planned_actions: List[Dict[str, float]],
                              time_horizon: float = 5.0) -> Dict[str, np.ndarray]:
        """
        Predict ego vehicle trajectory based on planned actions.
        
        Args:
            initial_state: Initial vehicle state {'x', 'y', 'vx', 'vy', 'psi', 'r'}
            planned_actions: Planned actions over time
            time_horizon: Prediction time horizon
            
        Returns:
            Predicted trajectory with uncertainty bounds
        """
        dt = time_horizon / self.prediction_steps
        times = np.linspace(0, time_horizon, self.prediction_steps + 1)
        
        # Initialize trajectory arrays
        x_traj = np.zeros(self.prediction_steps + 1)
        y_traj = np.zeros(self.prediction_steps + 1)
        vx_traj = np.zeros(self.prediction_steps + 1)
        vy_traj = np.zeros(self.prediction_steps + 1)
        psi_traj = np.zeros(self.prediction_steps + 1)
        
        # Set initial conditions
        x_traj[0] = initial_state.get('x', 0.0)
        y_traj[0] = initial_state.get('y', 0.0)
        vx_traj[0] = initial_state.get('vx', initial_state.get('vEgo', 0.0))
        vy_traj[0] = initial_state.get('vy', 0.0)
        psi_traj[0] = initial_state.get('psi', 0.0)  # Heading angle
        
        # Integrate forward in time
        for i in range(self.prediction_steps):
            # Get planned action for this time step
            # For now, use simplified model
            ax = planned_actions[i].get('acceleration', 0.0)  # Longitudinal acceleration
            delta = planned_actions[i].get('steering_angle', 0.0)  # Steering angle
            
            # Simple kinematic bicycle model integration
            # This is a simplified version - full model would be more complex
            v_total = math.sqrt(vx_traj[i]**2 + vy_traj[i]**2)
            
            # Update velocities
            vx_traj[i+1] = vx_traj[i] + ax * dt
            # Simplified lateral dynamics
            vy_traj[i+1] = vy_traj[i]  # For simplicity in this example
            
            # Update positions
            x_traj[i+1] = x_traj[i] + vx_traj[i] * dt
            y_traj[i+1] = y_traj[i] + vy_traj[i] * dt
            
            # Update heading (simplified)
            # In reality, this would depend on steering angle and vehicle speed
            yaw_rate = 0.0  # Simplified for this example
            psi_traj[i+1] = psi_traj[i] + yaw_rate * dt
        
        return {
            'times': times,
            'x': x_traj,
            'y': y_traj,
            'vx': vx_traj,
            'vy': vy_traj,
            'psi': psi_traj
        }
    
    def predict_surrounding_trajectories(self, 
                                       fused_leads: List[Dict[str, Any]],
                                       time_horizon: float = 5.0) -> Dict[int, Dict[str, np.ndarray]]:
        """
        Predict trajectories of surrounding vehicles.
        
        Args:
            fused_leads: List of tracked surrounding vehicles
            time_horizon: Prediction time horizon
            
        Returns:
            Dictionary of predicted trajectories for each vehicle
        """
        predictions = {}
        
        dt = time_horizon / self.prediction_steps
        
        for i, lead in enumerate(fused_leads):
            # Simple constant velocity prediction
            # In reality, this would use more sophisticated behavior models
            times = np.linspace(0, time_horizon, self.prediction_steps + 1)
            
            # Initial state from fused track
            x0 = lead['dRel']  # Longitudinal position relative to ego
            y0 = lead['yRel']  # Lateral position relative to ego
            vx0 = lead.get('vRel', 0.0)  # Relative velocity
            vy0 = 0.0  # Assume no lateral movement for simplicity
            
            # Predict trajectory with constant velocity
            x_traj = x0 + vx0 * times
            y_traj = y0 + vy0 * times  # No lateral movement
            
            predictions[i] = {
                'times': times,
                'x': x_traj,
                'y': y_traj,
                'probability': lead.get('modelProb', 0.8)  # Use fusion confidence
            }
        
        return predictions
    
    def compute_conflict_zones(self,
                              ego_prediction: Dict[str, np.ndarray],
                              surrounding_predictions: Dict[int, Dict[str, np.ndarray]],
                              safety_margin: float = 2.5) -> List[Dict[str, Any]]:
        """
        Compute potential conflict zones between ego and surrounding vehicles.
        
        Args:
            ego_prediction: Ego vehicle trajectory prediction
            surrounding_predictions: Surrounding vehicles' trajectory predictions
            safety_margin: Safety margin around vehicles (m)
        
        Returns:
            List of conflict zones with time, location, and probability
        """
        conflicts = []
        
        ego_x = ego_prediction['x']
        ego_y = ego_prediction['y']
        ego_times = ego_prediction['times']
        
        for veh_id, veh_pred in surrounding_predictions.items():
            veh_x = veh_pred['x']
            veh_y = veh_pred['y']
            veh_times = veh_pred['times']
            veh_prob = veh_pred['probability']
            
            # Find times when vehicles are in close proximity
            min_distance = float('inf')
            conflict_times = []
            
            # Sample the trajectories at common time steps
            for i in range(len(ego_times)):
                # Calculate distance between vehicles at time step i
                dist = math.sqrt((ego_x[i] - veh_x[i])**2 + (ego_y[i] - veh_y[i])**2)
                
                if dist < safety_margin:
                    conflict_times.append((ego_times[i], dist))
                    min_distance = min(min_distance, dist)
            
            if conflict_times:
                # Compute conflict probability weighted by vehicle detection probability
                avg_conflict_probability = veh_prob * (safety_margin - min_distance) / safety_margin
                
                conflicts.append({
                    'vehicle_id': veh_id,
                    'conflict_times': conflict_times,
                    'min_distance': min_distance,
                    'conflict_probability': avg_conflict_probability,
                    'severity': (safety_margin - min_distance) / safety_margin  # 0-1 scale
                })
        
        return conflicts


class TacticalDecisionMaker:
    """Tactical decision-making for complex driving scenarios."""
    
    def __init__(self):
        self.intersection_manager = IntersectionManager()
        self.lane_change_manager = LaneChangeManager()
        self.maneuver_planner = ManeuverPlanner()
    
    def make_tactical_decisions(self, 
                               sm: SubMaster,
                               behavioral_state: BehavioralState,
                               conflicts: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Make tactical decisions based on behavioral state and conflicts.
        
        Args:
            sm: SubMaster with current sensor data
            behavioral_state: Current behavioral planning state
            conflicts: List of predicted conflict zones
        
        Returns:
            Tactical decisions including priorities, speeds, and maneuvers
        """
        decisions = {
            'desired_speed': sm['carState'].vCruise * 0.8,  # Default to 80% of cruise
            'yield_vehicles': [],
            'priority_vehicles': [],
            'required_maneuvers': [],
            'safety_factor': 1.0
        }
        
        # Handle different behavioral states
        if behavioral_state == BehavioralState.TRAFFIC_JUNCTION:
            return self.intersection_manager.handle_intersection(sm, conflicts)
        
        elif behavioral_state in [BehavioralState.LANE_CHANGING_LEFT, BehavioralState.LANE_CHANGING_RIGHT]:
            return self.lane_change_manager.handle_lane_change(sm, behavioral_state)
        
        elif behavioral_state == BehavioralState.CONSTRUCTION_ZONE:
            return self.maneuver_planner.handle_construction_zone(sm)
        
        # Handle conflicts from prediction module
        if conflicts:
            conflict_decision = self._resolve_conflicts(conflicts, sm['carState'].vEgo)
            decisions.update(conflict_decision)
        
        return decisions
    
    def _resolve_conflicts(self, conflicts: List[Dict[str, Any]], v_ego: float) -> Dict[str, Any]:
        """Resolve potential conflicts with other vehicles."""
        if not conflicts:
            return {}
        
        # Find the most critical conflict (highest probability and severity)
        critical_conflict = max(conflicts, key=lambda x: x['conflict_probability'] * x['severity'])
        
        # Compute required response based on conflict severity
        if critical_conflict['conflict_probability'] > 0.8:
            # High probability conflict - significant speed reduction
            return {
                'desired_speed': max(0.0, v_ego * 0.3),  # Reduce to 30% of current speed
                'yield_vehicles': [critical_conflict['vehicle_id']],
                'safety_factor': 1.5  # Increase safety margins
            }
        elif critical_conflict['conflict_probability'] > 0.5:
            # Medium probability conflict - moderate speed adjustment
            return {
                'desired_speed': max(0.0, v_ego * 0.7),  # Reduce to 70% of current speed
                'yield_vehicles': [critical_conflict['vehicle_id']],
                'safety_factor': 1.2
            }
        else:
            # Low probability conflict - minor adjustment
            return {
                'desired_speed': v_ego * 0.95,  # Minor reduction
                'yield_vehicles': [],
                'safety_factor': 1.05
            }


class IntersectionManager:
    """Manager for intersection-related tactical decisions."""
    
    def handle_intersection(self, sm: SubMaster, conflicts: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Handle complex intersection scenarios."""
        return {
            'desired_speed': 5.0,  # Reduce to ~18 km/h for intersection
            'safety_factor': 1.5,
            'required_maneuvers': ['intersection_approach'],
            'right_of_way': self._determine_right_of_way(sm),
            'yield_vehicles': self._identify_yield_vehicles(conflicts)
        }
    
    def _determine_right_of_way(self, sm: SubMaster) -> bool:
        """Determine if ego vehicle has right of way."""
        # This would interface with traffic sign/light detection
        # For now, return a placeholder
        return True
    
    def _identify_yield_vehicles(self, conflicts: List[Dict[str, Any]]) -> List[int]:
        """Identify vehicles that ego should yield to."""
        yield_ids = []
        for conflict in conflicts:
            if conflict['conflict_probability'] > 0.6:
                yield_ids.append(conflict['vehicle_id'])
        return yield_ids


class LaneChangeManager:
    """Manager for lane change tactical decisions."""
    
    def handle_lane_change(self, sm: SubMaster, behavior_state: BehavioralState) -> Dict[str, Any]:
        """Handle lane change maneuvers."""
        lane_change_type = 'left' if behavior_state == BehavioralState.LANE_CHANGING_LEFT else 'right'
        
        return {
            'desired_speed': sm['carState'].vEgo,  # Maintain current speed
            'required_maneuvers': [f'lane_change_{lane_change_type}'],
            'lane_change_active': True,
            'safety_factor': 1.0,
            'lateral_comfort': 0.8  # Reduce lateral acceleration for comfort
        }


class ManeuverPlanner:
    """Planner for special maneuvers like construction zones."""
    
    def handle_construction_zone(self, sm: SubMaster) -> Dict[str, Any]:
        """Handle construction zone maneuvers."""
        return {
            'desired_speed': min(15.0, sm['carState'].vCruise * 0.6),  # Max 54 km/h in construction
            'required_maneuvers': ['construction_zone_navigation'],
            'safety_factor': 2.0,  # Increase safety in construction zones
            'lateral_comfort': 0.7
        }


class EnhancedLongitudinalPlanner:
    """
    Enhanced longitudinal planning that integrates behavioral planning,
    prediction, and tactical decisions.
    """
    
    def __init__(self):
        self.behavioral_planner = BehavioralPlanner()
        self.prediction_planner = PredictionBasedPlanner()
        self.tactical_decision_maker = TacticalDecisionMaker()
        
        # Advanced radar-camera fusion module
        self.fusion_module = AdvancedRadarCameraFusion()
        
        # Cost functions for trajectory optimization
        self.cost_weights = {
            'tracking': 1.0,
            'comfort': 0.5,
            'efficiency': 0.3,
            'safety': 2.0
        }
    
    def update(self, sm: SubMaster) -> Dict[str, Any]:
        """
        Update the enhanced longitudinal plan based on integrated inputs.
        
        Args:
            sm: SubMaster with current sensor data
            
        Returns:
            Enhanced longitudinal plan with behavioral and tactical context
        """
        # 1. Integrate advanced sensor fusion
        fused_leads = integrate_with_planner(sm, self.fusion_module)
        
        # 2. Update behavioral planning
        current_behavior = self.behavioral_planner.update(sm, {})
        
        # 3. Predict future trajectories
        ego_initial_state = self._get_ego_initial_state(sm)
        planned_actions = self._get_planned_actions(sm)  # From original planner
        
        ego_prediction = self.prediction_planner.predict_ego_trajectory(
            ego_initial_state, planned_actions
        )
        
        surrounding_predictions = self.prediction_planner.predict_surrounding_trajectories(
            fused_leads
        )
        
        # 4. Compute potential conflicts
        conflicts = self.prediction_planner.compute_conflict_zones(
            ego_prediction, surrounding_predictions
        )
        
        # 5. Make tactical decisions
        tactical_decisions = self.tactical_decision_maker.make_tactical_decisions(
            sm, current_behavior, conflicts
        )
        
        # 6. Generate enhanced plan incorporating all factors
        enhanced_plan = self._generate_enhanced_plan(
            sm, fused_leads, current_behavior, tactical_decisions, conflicts
        )
        
        return enhanced_plan
    
    def _get_ego_initial_state(self, sm: SubMaster) -> Dict[str, float]:
        """Get ego vehicle initial state for prediction."""
        car_state = sm['carState']
        return {
            'x': 0.0,  # Relative to ego (ego is at origin)
            'y': 0.0,
            'vx': car_state.vEgo,
            'vy': 0.0,
            'psi': 0.0,  # Heading relative to lane
            'vEgo': car_state.vEgo,
            'aEgo': car_state.aEgo if hasattr(car_state, 'aEgo') else 0.0
        }
    
    def _get_planned_actions(self, sm: SubMaster) -> List[Dict[str, float]]:
        """Get planned actions from current planner state."""
        # This would interface with the existing planner's planned trajectory
        # For now, return a mock set of actions
        actions = []
        for i in range(20):  # 20 time steps
            actions.append({
                'acceleration': 0.0,  # Placeholder
                'steering_angle': 0.0  # Placeholder
            })
        return actions
    
    def _generate_enhanced_plan(self,
                               sm: SubMaster,
                               fused_leads: List[Dict[str, Any]],
                               current_behavior: BehavioralState,
                               tactical_decisions: Dict[str, Any],
                               conflicts: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Generate the final enhanced planning output."""
        # Start with existing planner output
        car_state = sm['carState']
        v_ego = car_state.vEgo
        
        # Apply tactical speed adjustments
        desired_speed = tactical_decisions.get('desired_speed', v_ego)
        
        # Modify based on behavior state
        if current_behavior == BehavioralState.STOPPING:
            desired_speed = 0.0
        elif current_behavior == BehavioralState.TRAFFIC_JUNCTION:
            desired_speed = min(desired_speed, 5.0)  # 18 km/h max at junctions
        elif current_behavior in [BehavioralState.LANE_CHANGING_LEFT, BehavioralState.LANE_CHANGING_RIGHT]:
            # Maintain speed during lane change
            desired_speed = v_ego
        
        # Create enhanced plan
        enhanced_plan = {
            # Original plan values
            'speeds': None,  # Will be computed
            'accels': None,  # Will be computed 
            'jerk': None,    # Will be computed
            
            # Enhanced values
            'behavioral_state': current_behavior,
            'tactical_adjustments': tactical_decisions,
            'conflicts': conflicts,
            'desired_speed': desired_speed,
            'safety_factor': tactical_decisions.get('safety_factor', 1.0),
            'behavior_confidence': self.behavioral_planner.behavior_confidence,
            
            # Lead vehicle information
            'fused_leads': fused_leads,
            
            # Override flags for planner
            'override_lead_behavior': len([c for c in conflicts if c['conflict_probability'] > 0.7]) > 0,
            'reduce_speed_for_conflicts': len([c for c in conflicts if c['conflict_probability'] > 0.5]) > 0
        }
        
        # Compute speeds based on tactical decisions
        # This would involve more sophisticated trajectory optimization in practice
        base_time_steps = np.linspace(0, 4.0, 32)  # 32 steps like original
        speeds = self._compute_speed_profile(v_ego, desired_speed, base_time_steps)
        enhanced_plan['speeds'] = speeds.tolist()
        
        # Compute accelerations by differentiating speeds
        accels = np.diff(speeds) / (4.0 / len(speeds))  # Assuming 4s horizon
        # Pad with last value to match length
        accels_padded = np.append(accels, accels[-1])
        enhanced_plan['accels'] = accels_padded.tolist()
        
        return enhanced_plan
    
    def _compute_speed_profile(self, v_ego: float, desired_speed: float, time_steps: np.ndarray) -> np.ndarray:
        """Compute smooth speed profile from current to desired speed."""
        # Simple exponential approach to desired speed
        time_constant = 2.0  # 2 second time constant for smooth transition
        
        speeds = np.zeros_like(time_steps)
        for i, t in enumerate(time_steps):
            # Exponential approach to desired speed
            speeds[i] = desired_speed + (v_ego - desired_speed) * math.exp(-t / time_constant)
        
        return speeds


def create_enhanced_planner_integration():
    """
    Create integration wrapper for the enhanced planner that can be used
    to replace or augment the existing longitudinal planner.
    """
    enhanced_planner = EnhancedLongitudinalPlanner()
    
    def integrated_update(sm: SubMaster):
        """Integrated update function that can replace or augment existing planner."""
        return enhanced_planner.update(sm)
    
    return integrated_update


if __name__ == "__main__":
    print("Hierarchical Planning Enhancement Module")
    print("Implements behavioral planning, prediction, and tactical decision-making")
    print("for Tesla FSD-like capabilities within Comma 3x hardware limits.")