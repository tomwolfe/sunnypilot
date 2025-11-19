#!/usr/bin/env python3
"""
Advanced Planning System for Sunnypilot
Uses behavior prediction and environmental context to make intelligent driving decisions
"""
import numpy as np
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
import math
import time
from enum import Enum

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.perception.behavior_prediction import PredictionResult, EnhancedPredictionSystem


class PlanningState(Enum):
    """Planning states for the autonomous driving system"""
    LANE_FOLLOWING = "lane_following"
    LANE_CHANGING_LEFT = "lane_changing_left"
    LANE_CHANGING_RIGHT = "lane_changing_right"
    STOPPING = "stopping"
    ADAPTIVE_CRUISE = "adaptive_cruise"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class PlanningResult:
    """Result of the planning process"""
    desired_curvature: float  # Desired road curvature
    desired_speed: float      # Desired speed in m/s
    desired_acceleration: float  # Desired acceleration in m/s^2
    planning_state: PlanningState  # Current planning state
    safety_factor: float      # Safety factor (0-1, 1 is safest)
    confidence: float         # Confidence in the plan
    risk_assessment: Dict[str, float]  # Risk metrics


@dataclass
class EgoState:
    """Current state of the ego vehicle"""
    position: np.ndarray  # [x, y, z]
    velocity: float       # Speed in m/s
    acceleration: float   # Acceleration in m/s^2
    heading: float        # Heading in radians
    curvature: float      # Current road curvature
    steering_angle: float # Current steering angle in degrees


@dataclass
class EnvironmentalState:
    """Environmental state for planning"""
    lane_width: float = 3.7  # Standard lane width in meters
    speed_limit: float = 25.0  # Default speed limit in m/s (90 km/h)
    road_type: str = "highway"  # highway, city, residential
    weather_factor: float = 1.0  # 1.0 for good weather, lower for poor conditions
    time_of_day_factor: float = 1.0  # 1.0 for day, lower for night


class AdvancedPlanner:
    """
    Advanced planning system that integrates behavior prediction to make 
    intelligent driving decisions
    """
    
    def __init__(self):
        self.prediction_system = EnhancedPredictionSystem()
        self.planning_state = PlanningState.LANE_FOLLOWING
        self.last_planning_time = time.time()
        self.safety_margin_factor = 1.0  # Adjustable safety margin
        self.min_safe_distance = 50.0  # Minimum safe following distance in meters
        
        # Lane change parameters
        self.lane_change_min_distance = 50.0  # Minimum distance to target in lane
        self.lane_change_max_ego_speed = 31.0  # Max speed for lane change (112 km/h)
        self.lane_change_min_gap = 50.0  # Minimum gap required for safe lane change
        
    def plan_trajectory(self, 
                       ego_state: EgoState, 
                       tracked_objects: Dict[str, Any], 
                       env_state: EnvironmentalState) -> PlanningResult:
        """Main planning function that generates driving decisions"""
        start_time = time.time()
        
        # Predict behaviors of all tracked objects
        predictions = self.prediction_system.predict_all_behaviors(tracked_objects)
        
        # Assess risks based on predictions
        risk_assessment = self.prediction_system.get_risk_assessment(predictions, {
            'x': ego_state.position[0],
            'y': ego_state.position[1],
            'z': ego_state.position[2]
        })
        
        # Generate planning decision based on predictions and environment
        desired_curvature, desired_speed, desired_acceleration, planning_state = \
            self._generate_plan(ego_state, tracked_objects, predictions, env_state)
        
        # Calculate confidence and safety factor
        confidence = self._calculate_planning_confidence(predictions, risk_assessment)
        safety_factor = self._calculate_safety_factor(risk_assessment, ego_state, env_state)
        
        # Update planning state
        self.planning_state = planning_state
        
        # Log planning metrics
        planning_time = time.time() - start_time
        cloudlog.debug(f"Planning completed in {planning_time*1000:.1f}ms")
        
        return PlanningResult(
            desired_curvature=desired_curvature,
            desired_speed=desired_speed,
            desired_acceleration=desired_acceleration,
            planning_state=planning_state,
            safety_factor=safety_factor,
            confidence=confidence,
            risk_assessment=risk_assessment
        )
    
    def _generate_plan(self, 
                      ego_state: EgoState, 
                      tracked_objects: Dict[str, Any], 
                      predictions: Dict[str, PredictionResult], 
                      env_state: EnvironmentalState) -> Tuple[float, float, float, PlanningState]:
        """Generate the actual driving plan based on inputs"""
        # Default values
        desired_curvature = ego_state.curvature
        desired_speed = min(ego_state.velocity, env_state.speed_limit)
        desired_acceleration = 0.0
        planning_state = PlanningState.LANE_FOLLOWING
        
        # Get closest lead vehicle
        closest_lead_dist, closest_lead_obj = self._get_closest_lead_vehicle(
            ego_state, tracked_objects
        )
        
        # Get nearest vehicle in adjacent lanes
        left_lane_vehicles = self._get_vehicles_in_lane(tracked_objects, -1, ego_state)
        right_lane_vehicles = self._get_vehicles_in_lane(tracked_objects, 1, ego_state)
        
        # Collision risk assessment
        collision_risk = predictions.get('collision_risk', 0.0)
        
        # Handle adaptive cruise control and following
        if closest_lead_dist and closest_lead_dist < self.min_safe_distance * 0.8:
            lead_obj = closest_lead_obj
            lead_vel = np.linalg.norm(lead_obj.state[3:6]) if hasattr(lead_obj, 'state') else 0.0
            relative_speed = ego_state.velocity - lead_vel
            time_to_collision = closest_lead_dist / max(relative_speed, 0.1)
            
            # Adaptive cruise control: maintain safe distance
            if relative_speed > 0 and time_to_collision < 3.0:  # Less than 3 seconds
                desired_speed = max(lead_vel, 0.5)  # Don't go below 0.5 m/s
                desired_acceleration = min(-1.0, -relative_speed / max(time_to_collision, 0.1))  # Decelerate
                planning_state = PlanningState.ADAPTIVE_CRUISE
            elif 3.0 <= time_to_collision <= 6.0:  # Moderate following distance
                desired_speed = min(lead_vel * 1.1, env_state.speed_limit)  # Slightly faster than lead
                desired_acceleration = min(0.5, relative_speed / max(time_to_collision, 1.0))  # Mild acceleration
                planning_state = PlanningState.ADAPTIVE_CRUISE
        else:
            # No close lead vehicle, can follow speed limit
            desired_speed = env_state.speed_limit
            planning_state = PlanningState.LANE_FOLLOWING
        
        # Check if we should consider lane changing (for traffic optimization)
        if (ego_state.velocity < env_state.speed_limit - 5.0 and  # Traffic is slow
            len(right_lane_vehicles) == 0 and  # Right lane is clear
            ego_state.velocity < self.lane_change_max_ego_speed):  # Within speed limit for lane change
            # Consider lane change to right lane for faster traffic
            if planning_state == PlanningState.ADAPTIVE_CRUISE:
                planning_state = PlanningState.LANE_CHANGING_RIGHT
        
        # Emergency handling based on risk assessment
        if collision_risk > 0.8:
            # High collision risk, emergency stop
            desired_speed = 0.0
            desired_acceleration = -4.0  # Hard braking
            planning_state = PlanningState.EMERGENCY_STOP
        elif collision_risk > 0.5:
            # Moderate collision risk, obstacle avoidance
            desired_speed = max(ego_state.velocity * 0.5, 5.0)
            desired_acceleration = -2.0
            planning_state = PlanningState.OBSTACLE_AVOIDANCE
        
        # Apply environmental adjustments
        desired_speed *= env_state.weather_factor * env_state.time_of_day_factor
        desired_acceleration *= env_state.weather_factor * env_state.time_of_day_factor
        
        # Calculate desired curvature based on planned maneuvers
        if planning_state == PlanningState.LANE_CHANGING_RIGHT:
            desired_curvature = self._calculate_lane_change_curvature(ego_state, 1)  # Right lane
        elif planning_state == PlanningState.LANE_CHANGING_LEFT:
            desired_curvature = self._calculate_lane_change_curvature(ego_state, -1)  # Left lane
        elif planning_state in [PlanningState.EMERGENCY_STOP, PlanningState.OBSTACLE_AVOIDANCE]:
            # Smooth out curvature in emergency situations
            desired_curvature = ego_state.curvature * 0.5  # Reduce curvature for stability
        
        # Ensure values are within safe limits
        desired_speed = np.clip(desired_speed, 0.0, env_state.speed_limit * 1.1)
        desired_acceleration = np.clip(desired_acceleration, -5.0, 3.0)  # -5 to +3 m/s^2
        
        return desired_curvature, desired_speed, desired_acceleration, planning_state
    
    def _get_closest_lead_vehicle(self, ego_state: EgoState, tracked_objects: Dict[str, Any]) -> Tuple[Optional[float], Optional[Any]]:
        """Find the closest vehicle ahead in the same lane"""
        closest_dist = float('inf')
        closest_obj = None
        
        ego_pos = ego_state.position
        ego_vel = ego_state.velocity
        
        for obj_id, obj in tracked_objects.items():
            if hasattr(obj, 'state') and len(obj.state) >= 6:
                obj_pos = obj.state[:3]
                obj_vel = obj.state[3:6]
                
                # Calculate vector from ego to object
                rel_pos = obj_pos - ego_pos
                
                # Check if object is ahead (positive x direction relative to ego heading)
                heading_vec = np.array([math.cos(ego_state.heading), math.sin(ego_state.heading), 0])
                longitudinal_dist = np.dot(rel_pos, heading_vec)
                
                # Check if in same lane (lateral distance < lane width / 2)
                lateral_vec = np.array([-math.sin(ego_state.heading), math.cos(ego_state.heading), 0])
                lateral_dist = abs(np.dot(rel_pos, lateral_vec))
                
                if (0 < longitudinal_dist < 100 and  # Within 100m ahead
                    lateral_dist < 2.0 and  # Within lane width
                    obj.class_name in ['car', 'truck', 'bus']):  # Is a vehicle
                    if longitudinal_dist < closest_dist:
                        closest_dist = longitudinal_dist
                        closest_obj = obj
        
        return (closest_dist, closest_obj) if closest_obj else (None, None)
    
    def _get_vehicles_in_lane(self, tracked_objects: Dict[str, Any], lane_offset: int, ego_state: EgoState) -> List[Any]:
        """Get vehicles in adjacent lane (offset of -1 for left, +1 for right)"""
        lane_vehicles = []
        
        ego_pos = ego_state.position
        ego_heading = ego_state.heading
        
        for obj_id, obj in tracked_objects.items():
            if hasattr(obj, 'state') and len(obj.state) >= 6:
                obj_pos = obj.state[:3]
                
                # Calculate vector from ego to object
                rel_pos = obj_pos - ego_pos
                
                # Calculate longitudinal and lateral distances
                heading_vec = np.array([math.cos(ego_heading), math.sin(ego_heading), 0])
                longitudinal_dist = np.dot(rel_pos, heading_vec)
                
                lateral_vec = np.array([-math.sin(ego_heading), math.cos(ego_heading), 0])
                lateral_dist = np.dot(rel_pos, lateral_vec)
                
                # Check if in adjacent lane (lane_offset * lane_width +/- lane_width/2)
                target_lateral_pos = lane_offset * 3.7  # 3.7m per lane
                lane_threshold = 1.85  # Half lane width
                
                if (-50 < longitudinal_dist < 100 and  # Reasonable distance range
                    abs(lateral_dist - target_lateral_pos) < lane_threshold and  # In target lane
                    obj.class_name in ['car', 'truck', 'bus']):  # Is a vehicle
                    lane_vehicles.append(obj)
        
        return lane_vehicles
    
    def _calculate_lane_change_curvature(self, ego_state: EgoState, lane_direction: int) -> float:
        """Calculate appropriate curvature for lane change maneuver"""
        # Calculate target lateral offset based on lane direction
        target_lateral_offset = 3.7 * lane_direction  # 3.7m per lane

        # More sophisticated lane change model considering safety and comfort
        # Time for lane change based on vehicle speed (faster vehicles need more time for safety)
        base_time = 3.0
        speed_factor = max(1.0, min(2.0, ego_state.velocity / 15.0))  # Scale with speed (15 m/s = ~54 km/h)
        time_for_lane_change = base_time * speed_factor

        longitudinal_distance = ego_state.velocity * time_for_lane_change

        # Calculate required curvature using clothoid (Euler spiral) for smooth transition
        # This provides a more natural and comfortable lane change
        required_curvature = 4 * target_lateral_offset / (longitudinal_distance ** 2)

        # Apply smoothing to the curvature change for passenger comfort
        current_curvature = ego_state.curvature
        desired_curvature = current_curvature * 0.7 + required_curvature * 0.3

        # Apply safety limits based on vehicle dynamics and road conditions
        max_safe_curvature = self._calculate_max_safe_curvature(ego_state.velocity)
        desired_curvature = np.clip(desired_curvature, -max_safe_curvature, max_safe_curvature)

        return desired_curvature

    def _calculate_max_safe_curvature(self, velocity: float) -> float:
        """Calculate maximum safe curvature based on vehicle speed and dynamics"""
        # Maximum safe lateral acceleration is typically 0.4g to 0.5g (3.92-4.9 m/s^2)
        max_lateral_accel = 4.0  # m/s^2

        # Curvature = lateral_accel / velocity^2
        if velocity > 0.1:  # Avoid division by zero
            max_curvature = max_lateral_accel / (velocity ** 2)
        else:
            max_curvature = 0.01  # Default safe value when stopped

        # Apply additional safety margin
        return max_curvature * 0.8  # 80% of theoretical limit for safety
    
    def _calculate_planning_confidence(self, predictions: Dict[str, PredictionResult], 
                                     risk_assessment: Dict[str, float]) -> float:
        """Calculate confidence in the planning decision"""
        if not predictions:
            return 0.1  # Low confidence if no predictions
        
        # Average confidence of all predictions
        avg_prediction_conf = np.mean([pred.confidence for pred in predictions.values()])
        
        # Lower confidence if high risk is detected
        max_risk = max(risk_assessment.values()) if risk_assessment else 0.0
        risk_adjustment = max(0.5, 1.0 - max_risk)  # Reduce confidence with high risk
        
        confidence = avg_prediction_conf * risk_adjustment
        
        return max(0.1, min(1.0, confidence))
    
    def _calculate_safety_factor(self, risk_assessment: Dict[str, float], 
                                ego_state: EgoState, env_state: EnvironmentalState) -> float:
        """Calculate overall safety factor for the plan"""
        # Base safety on risk assessment
        collision_risk = risk_assessment.get('collision_risk', 0.0)
        interference_risk = risk_assessment.get('lane_change_interference', 0.0)
        stopping_risk = risk_assessment.get('stopping_distance_risk', 0.0)
        
        # Combine risk factors
        overall_risk = max(collision_risk, interference_risk, stopping_risk)
        
        # Adjust for environmental factors
        env_safety_factor = env_state.weather_factor * env_state.time_of_day_factor
        
        # Calculate safety factor (inverse of risk)
        base_safety = 1.0 - overall_risk
        
        # Apply environmental adjustments
        safety_factor = base_safety * env_safety_factor * self.safety_margin_factor
        
        return max(0.1, min(1.0, safety_factor))


class EmergencyResponsePlanner:
    """
    Emergency response component for the planning system
    Handles critical situations requiring immediate action
    """
    
    def __init__(self):
        self.emergency_threshold = 0.9  # Risk threshold for emergency mode
        self.last_emergency_time = 0.0
        self.emergency_active = False
        
    def check_emergency_conditions(self, risk_assessment: Dict[str, float], 
                                 ego_state: EgoState) -> Tuple[bool, str]:
        """Check if emergency conditions are met"""
        current_time = time.time()
        
        collision_risk = risk_assessment.get('collision_risk', 0.0)
        total_threat = risk_assessment.get('total_threat_score', 0.0)
        
        # Check for various emergency conditions
        if collision_risk > self.emergency_threshold:
            return True, f"High collision risk detected: {collision_risk:.2f}"
        elif total_threat > 0.8:
            return True, f"High threat score detected: {total_threat:.2f}"
        elif ego_state.acceleration < -5.0:  # Hard braking
            time_since_emergency = current_time - self.last_emergency_time
            if time_since_emergency > 5.0:  # Don't trigger repeatedly
                return True, "Emergency braking detected"
        
        return False, ""
    
    def generate_emergency_plan(self, ego_state: EgoState, env_state: EnvironmentalState) -> PlanningResult:
        """Generate emergency stop plan"""
        self.last_emergency_time = time.time()
        self.emergency_active = True
        
        # Emergency plan: hard stop with maximum safe braking
        max_braking = -5.0  # m/s^2 (maximum safe deceleration)
        
        return PlanningResult(
            desired_curvature=0.0,  # Straighten out for stability
            desired_speed=0.0,      # Stop
            desired_acceleration=max_braking,
            planning_state=PlanningState.EMERGENCY_STOP,
            safety_factor=1.0,      # Maximum safety in emergency
            confidence=0.9,         # High confidence in emergency stop
            risk_assessment={
                'collision_risk': 1.0,
                'total_threat_score': 1.0,
                'emergency_mode': 1.0
            }
        )


def create_advanced_planning_system() -> Tuple[AdvancedPlanner, EmergencyResponsePlanner]:
    """Factory function to create advanced planning system"""
    return AdvancedPlanner(), EmergencyResponsePlanner()


if __name__ == "__main__":
    print("Advanced Planning System for Sunnypilot - Testing")
    print("=" * 55)
    
    # Create mock objects for testing
    class MockTrackedObject:
        def __init__(self, state, class_name, confidence=0.8):
            self.state = state
            self.class_name = class_name
            self.confidence = confidence
    
    # Create planner
    planner, emergency_planner = create_advanced_planning_system()
    
    # Test ego state
    ego_state = EgoState(
        position=np.array([0.0, 0.0, 0.0]),
        velocity=25.0,  # 90 km/h
        acceleration=0.0,
        heading=0.0,  # Heading in x direction
        curvature=0.0,  # Straight road
        steering_angle=0.0
    )
    
    # Test environmental state
    env_state = EnvironmentalState(
        lane_width=3.7,
        speed_limit=25.0,
        road_type="highway",
        weather_factor=1.0,
        time_of_day_factor=1.0
    )
    
    # Test with no obstacles
    tracked_objects = {}
    result = planner.plan_trajectory(ego_state, tracked_objects, env_state)
    
    print(f"Planning state: {result.planning_state}")
    print(f"Desired speed: {result.desired_speed:.2f} m/s")
    print(f"Desired acceleration: {result.desired_acceleration:.2f} m/s^2")
    print(f"Desired curvature: {result.desired_curvature:.4f}")
    print(f"Planning confidence: {result.confidence:.2f}")
    print(f"Safety factor: {result.safety_factor:.2f}")
    
    # Test with lead vehicle
    close_lead = MockTrackedObject(
        state=np.array([50.0, 0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # 50m ahead, going 72 km/h
        class_name='car',
        confidence=0.9
    )
    
    tracked_objects = {"lead_car": close_lead}
    result = planner.plan_trajectory(ego_state, tracked_objects, env_state)
    
    print(f"\nWith lead vehicle:")
    print(f"Planning state: {result.planning_state}")
    print(f"Desired speed: {result.desired_speed:.2f} m/s")
    print(f"Desired acceleration: {result.desired_acceleration:.2f} m/s^2")
    
    # Test emergency conditions
    high_risk_assessment = {
        'collision_risk': 0.95,
        'total_threat_score': 0.9,
        'lane_change_interference': 0.3
    }
    
    is_emergency, reason = emergency_planner.check_emergency_conditions(high_risk_assessment, ego_state)
    print(f"\nEmergency check: {is_emergency}, reason: {reason}")
    
    if is_emergency:
        emergency_result = emergency_planner.generate_emergency_plan(ego_state, env_state)
        print(f"Emergency plan speed: {emergency_result.desired_speed:.2f} m/s")
        print(f"Emergency plan acceleration: {emergency_result.desired_acceleration:.2f} m/s^2")
    
    print("\nAdvanced planning system ready for integration with control system")