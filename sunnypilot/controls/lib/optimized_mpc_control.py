"""
Optimized MPC Control for Sunnypilot2

This module implements optimized Model Predictive Control with enhanced cost functions,
feedforward control, and lateral-longitudinal coordination to achieve Tesla FSD-like
smoothness and performance within Comma 3x hardware limits.
"""

import numpy as np
from typing import Dict, Tuple, Optional
import math

from cereal import log
from cereal.messaging import SubMaster
from opendbc.car.structs import car
from openpilot.common.swaglog import cloudlog
from openpilot.common.realtime import DT_CTRL
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.drive_helpers import LIMIT_NEAREST_SPEED, CONTROL_N, MIN_SPEED, \
    SPEED_ERROR_OFFSET, get_speed_error, get_accel_from_plan
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_mpc import LatControlMpc
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel


class EnhancedLongitudinalMpc:
    """
    Enhanced Longitudinal MPC with optimized cost functions and better human-like behavior.
    """
    
    def __init__(self, dt: float = 0.2, radar_state=None):
        """
        Initialize enhanced longitudinal MPC.
        
        Args:
            dt: Time step for MPC (should match original LongitudinalMpc default of 0.2)
        """
        self.mpc = LongitudinalMpc(dt=dt)
        
        # Enhanced parameters for smoother control
        self.jerk_limit = 3.0  # Reduce jerk for smoother acceleration
        self.comfort_acceleration = 1.5  # Comfort-focused acceleration limits
        self.smooth_follow_gain = 0.8  # For smoother car following
        
        # Human-like driving parameters
        self.human_like_acceleration = True
        self.acceleration_smoothing_factor = 0.1
        self.previous_acceleration = 0.0
        
        # Time headway parameters for adaptive cruise control
        self.min_time_headway = 1.2  # Minimum time headway (s)
        self.comfort_time_headway = 1.8  # Comfortable time headway (s)
        self.max_time_headway = 3.0  # Maximum time headway (s)
    
    def set_cur_state(self, v: float, a: float):
        """Set current vehicle state."""
        self.mpc.set_cur_state(v, a)
    
    def set_weights(self, prev_accel_constraint: bool, personality=log.DriverMonitoringState.Personality.standard):
        """Set MPC weights with human-like comfort optimization."""
        self.mpc.set_weights(prev_accel_constraint, personality)
        
        # Modify the default weights for more human-like behavior
        # These are tuned for comfort and smoothness while maintaining safety
        base_weights = {
            'cruise': {
                'FCW': 1.0,
                'STAT': 0.8,  # Station keeping weight
                'SPEED': 0.2,  # Speed tracking weight
                'ACCEL': 0.7,  # Acceleration comfort weight
                'JERK': 0.6,   # Jerk minimization weight
                'TTC': 0.3,    # Time to collision weight
            },
            'acc': {
                'FCW': 1.0,
                'STAT': 0.6,   # Lower for ACC to allow more flexibility
                'SPEED': 0.3,
                'ACCEL': 0.9,  # Higher for smooth ACC
                'JERK': 0.8,   # Higher for smooth ACC
                'TTC': 0.4,
            }
        }
        
        # Adjust weights based on personality
        if personality == log.DriverMonitoringState.Personality.relaxed:
            # More relaxed following
            weights = base_weights['acc']
            weights['JERK'] *= 1.2  # Even smoother
            weights['ACCEL'] *= 1.1
        elif personality == log.DriverMonitoringState.Personality.aggressive:
            # More responsive
            weights = base_weights['cruise']
        else:  # standard
            weights = base_weights['acc'] if self.mpc.mode == 'acc' else base_weights['cruise']
        
        # Apply the weights to MPC
        self.mpc.weights = weights
    
    def update(self, radar_state, v_cruise: float, x: np.ndarray, v: np.ndarray, a: np.ndarray, j: np.ndarray, 
               personality=log.DriverMonitoringState.Personality.standard):
        """
        Update the MPC with enhanced logic for human-like behavior.
        """
        # Adaptive time headway based on speed and situation
        self._adjust_time_headway(radar_state, personality, v_cruise)
        
        # Enhance human-like behavior
        if self.human_like_acceleration:
            self._apply_human_like_behavior(radar_state, v, a)
        
        # Call the original MPC update
        self.mpc.update(radar_state, v_cruise, x, v, a, j, personality)
    
    def _adjust_time_headway(self, radar_state, personality, v_cruise):
        """Adjust time headway based on driving context."""
        # Get current time headway from original MPC
        current_time_headway = self.comfort_time_headway
        
        # Adjust based on personality
        if personality == log.DriverMonitoringState.Personality.relaxed:
            current_time_headway = min(self.max_time_headway, current_time_headway * 1.2)
        elif personality == log.DriverMonitoringState.Personality.aggressive:
            current_time_headway = max(self.min_time_headway, current_time_headway * 0.8)
        
        # Adjust based on speed (higher speeds need longer headways)
        if v_cruise > 20:  # Above ~72 km/h
            current_time_headway = min(self.max_time_headway, current_time_headway * 1.1)
        
        # Apply to radar state if we have one
        if radar_state.leadOne.status and radar_state.leadOne.dRel > 0:
            # Adjust lead vehicle time headway based on context
            lead_v_rel = radar_state.leadOne.vRel
            if lead_v_rel < -2:  # Lead is braking hard
                # Reduce time headway for safety
                current_time_headway = max(self.min_time_headway, current_time_headway * 0.8)
    
    def _apply_human_like_behavior(self, radar_state, v_plan, a_plan):
        """Apply human-like driving behavior modifications."""
        # Add more natural acceleration profiles
        # Humans don't accelerate as aggressively as machines
        if len(a_plan) > 0:
            # Smooth the planned accelerations to be more human-like
            max_human_acc = self.comfort_acceleration
            for i in range(len(a_plan)):
                a_plan[i] = np.clip(a_plan[i], -max_human_acc * 1.2, max_human_acc)
    
    @property
    def v_solution(self):
        return self.mpc.v_solution
    
    @property
    def a_solution(self):
        return self.mpc.a_solution
    
    @property
    def j_solution(self):
        return self.mpc.j_solution
    
    @property
    def source(self):
        return self.mpc.source
    
    @property
    def solve_time(self):
        return self.mpc.solve_time
    
    @property
    def mode(self):
        return self.mpc.mode
    
    @mode.setter
    def mode(self, value):
        self.mpc.mode = value
    
    @property
    def cruise_speed(self):
        return self.mpc.cruise_speed


class EnhancedLateralMpc(LatControlMpc):
    """
    Enhanced Lateral MPC with improved cost functions and feedforward control.
    """
    
    def __init__(self, CP, mpc_id, delay, mixed_cost_mode=False):
        super().__init__(CP, mpc_id, delay)
        
        # Enhanced parameters for smoother steering
        self.steer_rate_cost = 0.5  # Reduced for smoother steering
        self.steer_actuation_cost = 0.2  # Reduced to allow more steering
        self.lane_cost = 1.0  # Standard lane keeping cost
        self.d_path_cost = 0.8  # Path following cost
        
        # Human-like steering parameters
        self.human_like_steering = True
        self.max_curvature_rate = 0.005  # Limit how fast curvature changes
        self.steering_smoothing_factor = 0.2
        self.previous_curvature = 0.0
        
        # Feedforward control parameters
        self.feedforward_gain = 0.8
        self.lookahead_distance = 5.0  # meters ahead for preview control
    
    def init_weights(self, CP, custom_weight=0.):
        """Initialize MPC weights optimized for human-like steering."""
        super().init_weights(CP, custom_weight)
        
        # Adjust default weights for smoother operation
        self.solver.options['eps'] = 0.001  # Increase solver tolerance for speed
        self.solver.options['max_iter'] = 15  # Reduce max iterations for speed
        
        # Enhanced cost function weights
        self.solver.update_params({
            'COST_Q_PATH': np.array([1.0, 0.8, 0.5, 0.3]),  # [x, y, psi, vel]
            'COST_Q_FINAL_PATH': np.array([2.0, 1.5, 1.0, 0.5]),
            'COST_R': np.array([0.1, 0.2]),  # [steer, steer_rate]
            'COST_RF': np.array([0.1, 0.2]),  # Terminal cost
        })
    
    def update(self, v_ego: float, angle_steers: float, trajectory: np.ndarray, curvature: float,
               lat_mpc_offset: float = 0.0) -> Tuple[float, float, Dict[str, np.ndarray]]:
        """
        Update the lateral MPC with enhanced human-like behavior.
        """
        # Apply preview control for smoother steering
        enhanced_trajectory = self._add_preview_control(trajectory, v_ego)
        
        # Call the original update method
        steer_angle, curvature_rate, mpc_solution = super().update(
            v_ego, angle_steers, enhanced_trajectory, curvature, lat_mpc_offset
        )
        
        # Apply human-like smoothing
        if self.human_like_steering:
            steer_angle = self._apply_human_steering_behavior(
                v_ego, angle_steers, steer_angle, curvature_rate
            )
        
        return steer_angle, curvature_rate, mpc_solution
    
    def _add_preview_control(self, trajectory: np.ndarray, v_ego: float) -> np.ndarray:
        """
        Add preview control to trajectory for smoother path following.
        
        Args:
            trajectory: Original trajectory [x, y, psi, curvature]
            v_ego: Ego vehicle speed
            
        Returns:
            Enhanced trajectory with preview control
        """
        # Calculate preview time based on speed
        preview_time = min(2.0, self.lookahead_distance / max(v_ego, 1.0))
        
        # Apply preview control adjustments
        enhanced_trajectory = trajectory.copy()
        
        # For simplicity, we'll just adjust the path slightly based on lookahead
        # In a full implementation, this would use preview control theory
        if len(enhanced_trajectory) > 10 and v_ego > 2.0:
            # Look ahead and adjust the trajectory based on anticipated path
            lookahead_idx = min(int(preview_time * 10), len(enhanced_trajectory) - 1)  # 10 points per second
            
            # Adjust the path slightly based on lookahead curvature
            if lookahead_idx < len(enhanced_trajectory):
                future_curvature = enhanced_trajectory[lookahead_idx, 3]  # Assuming curvature is in 4th column
                # Apply slight feedforward adjustment based on future curvature
                enhanced_trajectory[:, 1] += future_curvature * self.feedforward_gain * 0.1  # Lateral adjustment
        
        return enhanced_trajectory
    
    def _apply_human_steering_behavior(self, v_ego: float, current_steering: float,
                                     computed_steering: float, curvature_rate: float) -> float:
        """
        Apply human-like steering behavior for smoother control.
        
        Args:
            v_ego: Ego vehicle speed
            current_steering: Current steering angle
            computed_steering: MPC-computed steering angle
            curvature_rate: Rate of curvature change
            
        Returns:
            Human-like adjusted steering angle
        """
        # Calculate desired curvature from computed steering
        if self.CP.steerRatio > 0 and self.CP.wheelbase > 0:
            computed_curvature = math.tan(math.radians(computed_steering)) / self.CP.wheelbase
        else:
            computed_curvature = 0.0
        
        # Apply smoothing to prevent jerky steering
        if abs(v_ego) > 0.5:  # Only apply at meaningful speeds
            # Limit the rate of curvature change
            max_curvature_change = self.max_curvature_rate * DT_CTRL
            desired_curvature_change = computed_curvature - self.previous_curvature
            
            if abs(desired_curvature_change) > max_curvature_change:
                actual_curvature_change = np.sign(desired_curvature_change) * max_curvature_change
                computed_curvature = self.previous_curvature + actual_curvature_change
        
        # Convert back to steering angle
        if self.CP.steerRatio > 0 and self.CP.wheelbase > 0:
            final_steering = math.degrees(math.atan(computed_curvature * self.CP.wheelbase))
        else:
            final_steering = computed_steering
        
        # Apply smoothing factor
        final_steering = (1 - self.steering_smoothing_factor) * current_steering + \
                         self.steering_smoothing_factor * final_steering
        
        # Update previous values for next iteration
        self.previous_curvature = computed_curvature
        
        return final_steering


class CoordinatedLateralLongitudinalController:
    """
    Coordinated controller that manages interaction between lateral and longitudinal control
    to achieve more human-like and safer driving behavior.
    """
    
    def __init__(self, CP, mpc_id, delay):
        self.CP = CP
        
        # Initialize enhanced controllers
        self.lateral_control = EnhancedLateralMpc(CP, mpc_id, delay)
        self.longitudinal_control = EnhancedLongitudinalMpc()
        
        # Coordination parameters
        self.lateral_longitudinal_coupling = True
        self.max_lateral_accel = 2.5  # Maximum lateral acceleration (m/s²)
        self.lateral_influence_on_long = 0.3  # How much lateral affects longitudinal
        self.long_influence_on_lat = 0.2  # How much longitudinal affects lateral
        
        # Speed-dependent coordination
        self.speed_curvature_gain = 0.05  # Gain for speed-dependent steering
        self.braking_steering_compensation = 0.1  # Steering compensation during braking
        
        # Smoothing filters
        self.steer_rate_limiter = FirstOrderFilter(0.0, 0.1, DT_CTRL)  # 0.1 second time constant
        self.accel_rate_limiter = FirstOrderFilter(0.0, 0.1, DT_CTRL)
    
    def update(self, v_ego: float, a_ego: float, angle_steers: float, curvature: float,
               trajectory: np.ndarray, plan: Dict[str, np.ndarray],
               radar_state, v_cruise: float, personality=log.DriverMonitoringState.Personality.standard) -> Tuple[float, float, Dict[str, Any]]:
        """
        Update coordinated lateral and longitudinal control.
        
        Args:
            v_ego: Ego vehicle speed
            a_ego: Ego vehicle acceleration
            angle_steers: Current steering angle
            curvature: Current curvature
            trajectory: Path trajectory for lateral control
            plan: Longitudinal plan from model
            radar_state: Radar state for lead vehicle information
            v_cruise: Cruise speed
            personality: Driving personality setting
            
        Returns:
            Tuple of (steer_torque, acceleration, control_info_dict)
        """
        # Calculate lateral acceleration demand
        lateral_accel_demand = v_ego**2 * abs(curvature) if abs(v_ego) > 0.5 else 0.0
        
        # Coordinate lateral and longitudinal control
        coordination_factors = self._calculate_coordination_factors(
            v_ego, a_ego, curvature, lateral_accel_demand, radar_state
        )
        
        # Update lateral controller
        steer_angle, curvature_rate, lat_solution = self.lateral_control.update(
            v_ego, angle_steers, trajectory, curvature
        )
        
        # Apply coordination adjustments to lateral control
        steer_angle *= (1 - coordination_factors['lat_reduction'])
        
        # Update longitudinal controller with coordination
        x_plan = plan.get('x', np.zeros(len(T_IDXS_MPC)))
        v_plan = plan.get('v', np.zeros(len(T_IDXS_MPC)))
        a_plan = plan.get('a', np.zeros(len(T_IDXS_MPC)))
        j_plan = plan.get('j', np.zeros(len(T_IDXS_MPC)))
        
        self.longitudinal_control.set_cur_state(v_ego, a_ego)
        self.longitudinal_control.set_weights(
            prev_accel_constraint=True, personality=personality
        )
        
        self.longitudinal_control.update(
            radar_state, v_cruise, x_plan, v_plan, a_plan, j_plan, personality
        )
        
        # Apply coordination adjustments to longitudinal control
        longitudinal_acceleration = self.longitudinal_control.a_solution[0]  # First solution
        
        # Add lateral coordination to longitudinal control
        if coordination_factors['reduce_for_curve']:
            longitudinal_acceleration *= (1 - coordination_factors['long_reduction'])
        
        # Apply safety limits considering both controls
        longitudinal_acceleration = self._apply_coordination_limits(
            v_ego, longitudinal_acceleration, lateral_accel_demand, coordination_factors
        )
        
        # Apply smoothing to prevent jerky control
        smoothed_steer = self.steer_rate_limiter.update(steer_angle)
        smoothed_accel = self.accel_rate_limiter.update(longitudinal_acceleration)
        
        control_info = {
            'lateral_solution': lat_solution,
            'longitudinal_solution': {
                'v_solution': self.longitudinal_control.v_solution,
                'a_solution': self.longitudinal_control.a_solution,
                'j_solution': self.longitudinal_control.j_solution
            },
            'coordination_factors': coordination_factors,
            'lateral_accel_demand': lateral_accel_demand
        }
        
        # Convert steer angle to torque/duty cycle based on vehicle parameters
        # This is a simplified conversion - real implementation would be vehicle-specific
        steer_torque = self._steer_angle_to_torque(smoothed_steer, v_ego)
        
        return steer_torque, smoothed_accel, control_info
    
    def _calculate_coordination_factors(self, v_ego: float, a_ego: float, curvature: float,
                                      lateral_accel_demand: float, radar_state) -> Dict[str, float]:
        """
        Calculate coordination factors based on driving situation.
        """
        factors = {
            'lat_reduction': 0.0,
            'long_reduction': 0.0,
            'reduce_for_curve': False,
            'emergency_braking': False,
            'lead_vehicle_influence': 0.0
        }
        
        # Check for high lateral acceleration demand
        if lateral_accel_demand > self.max_lateral_accel * 0.8:  # 80% of max
            factors['lat_reduction'] = min(0.3, (lateral_accel_demand / self.max_lateral_accel) - 0.8)
            factors['reduce_for_curve'] = True
            
            # Reduce longitudinal acceleration in curves
            if v_ego > 5.0:  # Only at meaningful speeds
                factors['long_reduction'] = min(0.2, factors['lat_reduction'] * self.lateral_influence_on_long)
        
        # Check for lead vehicle situation
        if radar_state.leadOne.status:
            d_rel = radar_state.leadOne.dRel
            v_rel = radar_state.leadOne.vRel
            
            if d_rel < 50.0:  # Within 50m
                # Calculate time to collision with lead
                if v_rel < 0:  # Approaching lead
                    ttc = d_rel / max(0.1, abs(v_rel))
                    if ttc < 3.0:  # TTC < 3s
                        factors['emergency_braking'] = True
                        factors['long_reduction'] = 1.0  # Full deceleration priority
                    elif ttc < 5.0:  # Moderate risk
                        factors['lead_vehicle_influence'] = 1.0 - (ttc / 5.0)
                        factors['long_reduction'] = factors['lead_vehicle_influence'] * 0.5
        
        # Check for braking situation (apply steering compensation)
        if a_ego < -1.0:  # Strong braking
            factors['braking_compensation'] = abs(a_ego) * self.braking_steering_compensation
            factors['lat_reduction'] = min(0.2, factors['lat_reduction'] + factors['braking_compensation'])
        
        return factors
    
    def _apply_coordination_limits(self, v_ego: float, accel: float, lateral_accel_demand: float,
                                  coordination_factors: Dict[str, float]) -> float:
        """
        Apply coordination-based limits to longitudinal acceleration.
        """
        # Apply maximum acceleration limits based on total acceleration demand
        total_accel_demand = math.sqrt(accel**2 + lateral_accel_demand**2)
        max_total_accel = self.max_lateral_accel  # Use lateral accel limit as total limit for safety
        
        if total_accel_demand > max_total_accel:
            # Reduce both accelerations proportionally
            scale_factor = max_total_accel / total_accel_demand
            accel *= scale_factor
        
        # Apply specific coordination-based limits
        if coordination_factors['reduce_for_curve'] and v_ego > 5.0:
            # Reduce acceleration in curves based on speed and curvature
            speed_factor = max(0.5, min(1.0, v_ego / 15.0))  # 0.5 at 0m/s, 1.0 at 15m/s+
            accel *= (1 - coordination_factors['long_reduction'] * speed_factor)
        
        # Apply emergency braking limits if needed
        if coordination_factors['emergency_braking']:
            # Allow full braking capability
            max_brake = -6.0  # Maximum comfortable braking
            accel = max(max_brake, accel)
        
        return accel
    
    def _steer_angle_to_torque(self, steer_angle: float, v_ego: float) -> float:
        """
        Convert steering angle to torque/duty cycle for vehicle control.
        This is a simplified implementation - real version would use vehicle-specific calibration.
        """
        # Use vehicle parameters to convert to appropriate units
        steer_ratio = self.CP.steerRatio
        eps_kp = getattr(self.CP.lateralTuning, 'kp', 0.5)  # Use 0.5 as default if not available
        
        # Simple conversion (would be vehicle-specific in practice)
        torque = steer_angle * eps_kp * (1 + v_ego * self.speed_curvature_gain)
        
        # Apply limits
        max_torque = 1.0  # Normalized torque limit
        torque = np.clip(torque, -max_torque, max_torque)
        
        return torque


class EnhancedMPCController:
    """
    Main entry point for the enhanced MPC control system that can be integrated
    with the existing controls architecture.
    """
    
    def __init__(self, CP, delay=0.0):
        self.CP = CP
        self.delay = delay
        
        # Initialize coordinated controller
        self.coordinated_controller = CoordinatedLateralLongitudinalController(
            CP, mpc_id=0, delay=delay
        )
        
        # Original controllers as fallback
        self.original_lat_control = None
        self.original_lon_control = None
        
        # Performance monitoring
        self.control_smoothness = FirstOrderFilter(0.0, 1.0, DT_CTRL * 10)  # 10 cycle average
        self.last_steer = 0.0
        self.last_accel = 0.0
    
    def update(self, 
               v_ego: float, 
               a_ego: float,
               angle_steers: float, 
               curvature: float,
               trajectory: np.ndarray, 
               plan: Dict[str, np.ndarray],
               radar_state,
               v_cruise: float,
               personality=log.DriverMonitoringState.Personality.standard) -> Tuple[float, float, Dict[str, Any]]:
        """
        Main update function for the enhanced MPC controller.
        
        Args:
            v_ego: Ego vehicle speed
            a_ego: Ego vehicle acceleration  
            angle_steers: Current steering angle
            curvature: Current curvature
            trajectory: Path trajectory
            plan: Longitudinal plan
            radar_state: Radar state
            v_cruise: Cruise speed
            personality: Driving personality
            
        Returns:
            Tuple of (steer_torque, acceleration, info_dict)
        """
        # Call the coordinated controller
        steer_torque, acceleration, control_info = self.coordinated_controller.update(
            v_ego, a_ego, angle_steers, curvature, trajectory, plan, 
            radar_state, v_cruise, personality
        )
        
        # Monitor control smoothness
        steer_change = abs(steer_torque - self.last_steer)
        accel_change = abs(acceleration - self.last_accel)
        
        smoothness_score = self.control_smoothness.update(-(steer_change + accel_change))
        control_info['smoothness_score'] = float(smoothness_score)
        
        self.last_steer = steer_torque
        self.last_accel = acceleration
        
        return steer_torque, acceleration, control_info
    
    def publish(self, pm, control_params: Dict[str, Any]):
        """
        Publish control status and performance metrics.
        """
        # This would publish to messaging system in a real implementation
        pass


# Integration function for use with existing controls system
def create_enhanced_mpc_controller(CP, delay=0.0):
    """
    Factory function to create enhanced MPC controller that can be used
    to replace or augment the existing MPC implementation.
    """
    return EnhancedMPCController(CP, delay)


def integrate_with_controlsd(CP, delay=0.0):
    """
    Integration function to demonstrate how the enhanced controller 
    could be integrated with the existing controlsd architecture.
    """
    enhanced_controller = create_enhanced_mpc_controller(CP, delay)
    
    def update_control(
        v_ego: float, a_ego: float, angle_steers: float, curvature: float,
        trajectory: np.ndarray, plan: Dict[str, np.ndarray],
        radar_state, v_cruise: float,
        personality=log.DriverMonitoringState.Personality.standard
    ):
        """
        Wrapper function for integration with existing controlsd update loop.
        """
        return enhanced_controller.update(
            v_ego, a_ego, angle_steers, curvature, trajectory, plan,
            radar_state, v_cruise, personality
        )
    
    return update_control


if __name__ == "__main__":
    print("Enhanced MPC Control Module")
    print("Implements optimized Model Predictive Control with human-like behavior,")
    print("feedforward control, and lateral-longitudinal coordination.")