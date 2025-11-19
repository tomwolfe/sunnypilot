#!/usr/bin/env python3
"""
Safety Supervisor for Sunnypilot
Provides additional safety validation and oversight for autonomous driving systems
"""
import numpy as np
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
import time
import math
from enum import Enum

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.advanced_planner import PlanningResult, EgoState


class SafetyViolation(Enum):
    """Types of safety violations"""
    COLLISION_IMMINENT = "collision_imminent"
    ROAD_BOUNDARY_VIOLATION = "road_boundary_violation"
    SPEED_LIMIT_EXCEEDED = "speed_limit_exceeded"
    UNEXPECTED_MANEUVER = "unexpected_maneuver"
    TRACKING_INCONSISTENCY = "tracking_inconsistency"
    MODEL_DISAGREEMENT = "model_disagreement"
    EMERGENCY_STOP_REQUIRED = "emergency_stop_required"


@dataclass
class SafetyCheckResult:
    """Result of a safety check"""
    is_safe: bool
    violations: List[SafetyViolation]
    confidence: float
    explanation: str
    recovery_action: Optional[str] = None


@dataclass
class SafetyMetrics:
    """Safety metrics for system monitoring"""
    collision_risk: float
    tracking_accuracy: float
    model_consistency: float
    environmental_awareness: float
    overall_safety_score: float
    last_check_time: float


class SafetySupervisor:
    """
    Main safety supervisor that performs comprehensive validation of the autonomous system
    """
    
    def __init__(self):
        self.safety_thresholds = {
            'collision_risk': 0.7,  # Maximum acceptable collision risk
            'tracking_accuracy': 0.6,  # Minimum tracking accuracy
            'model_consistency': 0.7,  # Minimum model consistency
            'environmental_awareness': 0.6  # Minimum environmental awareness
        }
        
        self.last_safety_metrics = None
        self.safety_log = []  # Log of safety events
        self.max_log_size = 100
        
        # Collision prediction parameters
        self.collision_prediction_horizon = 3.0  # seconds ahead to predict
        self.min_safe_distance = 50.0  # minimum safe distance in meters
        
    def validate_control_commands(self, 
                                 ego_state: EgoState, 
                                 planning_result: PlanningResult, 
                                 model_outputs: Dict[str, Any],
                                 tracked_objects: Dict[str, Any]) -> SafetyCheckResult:
        """Validate control commands for safety before execution"""
        violations = []
        confidence = 1.0
        
        # Check 1: Collision risk assessment
        collision_check = self._assess_collision_risk(ego_state, planning_result, tracked_objects)
        if not collision_check.is_safe:
            violations.extend(collision_check.violations)
            confidence *= collision_check.confidence
        
        # Check 2: Road boundary compliance
        boundary_check = self._check_road_boundary_compliance(ego_state, planning_result)
        if not boundary_check.is_safe:
            violations.extend(boundary_check.violations)
            confidence *= boundary_check.confidence
            
        # Check 3: Speed limit compliance
        speed_check = self._check_speed_limit_compliance(ego_state, planning_result)
        if not speed_check.is_safe:
            violations.extend(speed_check.violations)
            confidence *= speed_check.confidence
            
        # Check 4: Maneuver reasonableness
        maneuver_check = self._check_maneuver_reasonableness(ego_state, planning_result)
        if not maneuver_check.is_safe:
            violations.extend(maneuver_check.violations)
            confidence *= maneuver_check.confidence
            
        # Determine overall safety
        is_safe = len(violations) == 0
        explanation = f"{'Safe' if is_safe else 'Unsafe'} - {len(violations)} violations detected"
        
        # Determine recovery action if unsafe
        recovery_action = None
        if not is_safe:
            if SafetyViolation.COLLISION_IMMINENT in violations:
                recovery_action = "EMERGENCY_BRAKE"
            elif SafetyViolation.ROAD_BOUNDARY_VIOLATION in violations:
                recovery_action = "RETURN_TO_LANE"
            else:
                recovery_action = "REDUCE_SPEED"
        
        # Update safety metrics
        metrics = self._calculate_safety_metrics(ego_state, planning_result, tracked_objects)
        self.last_safety_metrics = metrics
        
        return SafetyCheckResult(
            is_safe=is_safe,
            violations=violations,
            confidence=confidence,
            explanation=explanation,
            recovery_action=recovery_action
        )
    
    def _assess_collision_risk(self, 
                              ego_state: EgoState, 
                              planning_result: PlanningResult, 
                              tracked_objects: Dict[str, Any]) -> SafetyCheckResult:
        """Assess collision risk based on predicted trajectories"""
        violations = []
        confidence = 1.0
        
        # Predict ego vehicle trajectory based on planned acceleration and curvature
        ego_trajectory = self._predict_ego_trajectory(ego_state, planning_result)
        
        # Check for potential collisions with tracked objects
        for obj_id, obj in tracked_objects.items():
            if hasattr(obj, 'state') and len(obj.state) >= 9:  # Has position, velocity, acceleration
                obj_trajectory = self._predict_object_trajectory(obj)
                
                # Check for trajectory intersections
                min_distance = float('inf')
                for ego_pos in ego_trajectory:
                    for obj_pos in obj_trajectory:
                        dist = np.linalg.norm(ego_pos[:2] - obj_pos[:2])  # Only consider x,y plane
                        min_distance = min(min_distance, dist)
                
                # If minimum distance is too small, flag potential collision
                if min_distance < 5.0:  # Less than 5 meters is dangerous
                    violations.append(SafetyViolation.COLLISION_IMMINENT)
                    confidence *= 0.5  # Reduce confidence with collision risk
                    break  # Found one collision risk, stop checking
        
        return SafetyCheckResult(
            is_safe=len(violations) == 0,
            violations=violations,
            confidence=confidence,
            explanation=f"Collision risk check: {'OK' if not violations else 'Collision imminent'}"
        )
    
    def _predict_ego_trajectory(self, ego_state: EgoState, planning_result: PlanningResult, steps: int = 10) -> np.ndarray:
        """Predict ego vehicle trajectory based on current state and planned actions"""
        dt = self.collision_prediction_horizon / steps
        trajectory = np.zeros((steps, 3), dtype=np.float32)
        
        # Start from current position
        current_pos = ego_state.position.copy()
        current_vel = ego_state.velocity
        current_curvature = ego_state.curvature
        
        for i in range(steps):
            # Calculate position change based on velocity and curvature
            ds = current_vel * dt  # Distance traveled in this time step
            dx = ds * math.cos(current_curvature * ds / 2)  # Simplified arc movement
            dy = ds * math.sin(current_curvature * ds / 2)
            
            # Update position
            current_pos[0] += dx
            current_pos[1] += dy
            
            # Update velocity based on planned acceleration (simplified)
            current_vel += planning_result.desired_acceleration * dt
            current_vel = max(0, current_vel)  # Don't go backwards
            
            # Update curvature based on planned changes (simplified)
            current_curvature = planning_result.desired_curvature * min(1.0, i / 5)  # Gradual change
            
            trajectory[i] = current_pos.copy()
        
        return trajectory
    
    def _predict_object_trajectory(self, tracked_object: Any) -> np.ndarray:
        """Predict trajectory of a tracked object"""
        steps = 10
        dt = self.collision_prediction_horizon / steps
        
        # Extract object state (position, velocity, acceleration)
        state = tracked_object.state
        pos = state[:3].copy()
        vel = state[3:6].copy()
        acc = state[6:9].copy()
        
        trajectory = np.zeros((steps, 3), dtype=np.float32)
        
        for i in range(steps):
            # Update position: s = s0 + v*t + 0.5*a*t^2
            step_pos = pos + vel * (i+1) * dt + 0.5 * acc * ((i+1) * dt)**2
            trajectory[i] = step_pos
        
        return trajectory
    
    def _check_road_boundary_compliance(self, ego_state: EgoState, planning_result: PlanningResult) -> SafetyCheckResult:
        """Check if planned maneuver respects road boundaries"""
        violations = []
        
        # Check if desired curvature is extreme (indicating potential lane departure)
        if abs(planning_result.desired_curvature) > 0.02:  # Very high curvature
            violations.append(SafetyViolation.ROAD_BOUNDARY_VIOLATION)
        
        # Check if planned lateral movement is excessive
        # This is a simplified check - in reality would use lane line detection
        if abs(ego_state.position[1]) > 4.0:  # More than 4m from center line
            violations.append(SafetyViolation.ROAD_BOUNDARY_VIOLATION)
        
        return SafetyCheckResult(
            is_safe=len(violations) == 0,
            violations=violations,
            confidence=1.0,
            explanation=f"Road boundary check: {'OK' if not violations else 'Boundary violation'}"
        )
    
    def _check_speed_limit_compliance(self, ego_state: EgoState, planning_result: PlanningResult) -> SafetyCheckResult:
        """Check if planned speed complies with limits"""
        violations = []
        
        # Compare desired speed to current speed limit (in a real system, this would come from map data)
        speed_limit = 25.0  # Default 90 km/h if no map data
        if planning_result.desired_speed > speed_limit * 1.1:  # Allow 10% overage
            violations.append(SafetyViolation.SPEED_LIMIT_EXCEEDED)
        
        # Check acceleration/deceleration limits
        if abs(planning_result.desired_acceleration) > 4.0:  # Very high acceleration/deceleration
            violations.append(SafetyViolation.UNEXPECTED_MANEUVER)
        
        return SafetyCheckResult(
            is_safe=len(violations) == 0,
            violations=violations,
            confidence=1.0,
            explanation=f"Speed compliance check: {'OK' if not violations else 'Speed/excessive acceleration violation'}"
        )
    
    def _check_maneuver_reasonableness(self, ego_state: EgoState, planning_result: PlanningResult) -> SafetyCheckResult:
        """Check if planned maneuver is reasonable given current conditions"""
        violations = []
        
        # Check for sudden, unplanned changes in direction
        if abs(planning_result.desired_curvature - ego_state.curvature) > 0.01:  # Sudden direction change
            # Only flag if not in a lane change state
            if planning_result.planning_state.name not in ["LANE_CHANGING_LEFT", "LANE_CHANGING_RIGHT"]:
                violations.append(SafetyViolation.UNEXPECTED_MANEUVER)
        
        # Check for impossible acceleration commands relative to current speed
        if ego_state.velocity > 0 and planning_result.desired_acceleration < -6.0:  # Emergency braking threshold
            if planning_result.planning_state.name != "EMERGENCY_STOP":
                violations.append(SafetyViolation.UNEXPECTED_MANEUVER)
        
        return SafetyCheckResult(
            is_safe=len(violations) == 0,
            violations=violations,
            confidence=1.0,
            explanation=f"Maneuver reasonableness check: {'OK' if not violations else 'Unreasonable maneuver'}"
        )
    
    def _calculate_safety_metrics(self, 
                                 ego_state: EgoState, 
                                 planning_result: PlanningResult, 
                                 tracked_objects: Dict[str, Any]) -> SafetyMetrics:
        """Calculate overall safety metrics"""
        # Collision risk: based on distance to closest object and relative velocities
        collision_risk = self._calculate_collision_risk_metric(ego_state, tracked_objects)
        
        # Tracking accuracy: based on number and quality of tracked objects
        tracking_accuracy = self._calculate_tracking_accuracy_metric(tracked_objects)
        
        # Model consistency: based on confidence in planning result
        model_consistency = planning_result.confidence if hasattr(planning_result, 'confidence') else 0.8
        
        # Environmental awareness: based on how well we understand the surroundings
        environmental_awareness = self._calculate_environmental_awareness(tracked_objects)
        
        # Overall score: weighted average of all metrics
        overall_score = (
            0.3 * (1.0 - collision_risk) + 
            0.25 * tracking_accuracy + 
            0.25 * model_consistency + 
            0.2 * environmental_awareness
        )
        
        return SafetyMetrics(
            collision_risk=collision_risk,
            tracking_accuracy=tracking_accuracy,
            model_consistency=model_consistency,
            environmental_awareness=environmental_awareness,
            overall_safety_score=overall_score,
            last_check_time=time.time()
        )
    
    def _calculate_collision_risk_metric(self, ego_state: EgoState, tracked_objects: Dict[str, Any]) -> float:
        """Calculate collision risk metric (0-1, where 1 is highest risk)"""
        min_distance = float('inf')
        
        for obj_id, obj in tracked_objects.items():
            if hasattr(obj, 'state'):
                obj_pos = obj.state[:3]
                dist = np.linalg.norm(ego_state.position - obj_pos)
                min_distance = min(min_distance, dist)
        
        # Convert distance to risk metric (closer = higher risk)
        if min_distance < 1.0:
            return 1.0
        elif min_distance > 50.0:
            return 0.0
        else:
            # Linear scaling between 1-50m
            return (50.0 - min_distance) / 49.0
    
    def _calculate_tracking_accuracy_metric(self, tracked_objects: Dict[str, Any]) -> float:
        """Calculate tracking accuracy metric (0-1, where 1 is highest accuracy)"""
        if not tracked_objects:
            return 0.1  # Low if no objects tracked
        
        # Average confidence of tracked objects
        total_confidence = sum(getattr(obj, 'confidence', 0.5) for obj in tracked_objects.values())
        avg_confidence = total_confidence / len(tracked_objects) if tracked_objects else 0.5
        
        # Adjust based on number of tracked objects (more is better, up to a point)
        num_objects = len(tracked_objects)
        if num_objects > 20:  # Too many might indicate false positives
            avg_confidence *= 0.8
        elif num_objects > 10:
            avg_confidence *= 1.1  # Good amount of objects
        elif num_objects < 2:  # Too few might indicate poor tracking
            avg_confidence *= 0.7
        
        return max(0.0, min(1.0, avg_confidence))
    
    def _calculate_environmental_awareness(self, tracked_objects: Dict[str, Any]) -> float:
        """Calculate environmental awareness metric (0-1, where 1 is highest awareness)"""
        if not tracked_objects:
            return 0.2  # Basic awareness if no objects but system running
        
        # Count different object types to assess environmental diversity
        obj_types = set()
        for obj in tracked_objects.values():
            if hasattr(obj, 'class_name'):
                obj_types.add(obj.class_name)
        
        # More diverse object types = better environmental awareness
        type_score = min(1.0, len(obj_types) / 5)  # Cap at 5 different types
        
        # Also consider how well we're tracking over time
        tracking_stability_score = min(1.0, len(tracked_objects) / 10)  # Normalize by expected number
        
        return (type_score + tracking_stability_score) / 2
    
    def get_safety_recommendation(self, safety_check: SafetyCheckResult) -> Tuple[bool, str]:
        """Get safety recommendation based on safety check result"""
        if not safety_check.is_safe:
            if safety_check.recovery_action == "EMERGENCY_BRAKE":
                return False, "Immediate stop required due to collision risk"
            elif safety_check.recovery_action == "RETURN_TO_LANE":
                return False, "Return to lane required due to boundary violation"
            elif safety_check.recovery_action == "REDUCE_SPEED":
                return False, "Reduce speed required due to safety concerns"
            else:
                return False, "Safety violations detected, action required"
        else:
            return True, "System is operating safely"
    
    def update_safety_log(self, safety_check: SafetyCheckResult):
        """Update safety log with latest check result"""
        self.safety_log.append({
            'timestamp': time.time(),
            'result': safety_check,
            'metrics': self.last_safety_metrics
        })
        
        # Trim log if too long
        if len(self.safety_log) > self.max_log_size:
            self.safety_log = self.safety_log[-self.max_log_size:]


class RedundantSafetyValidator:
    """
    Redundant safety validation using multiple methods to ensure safety
    """
    
    def __init__(self):
        self.supervisor = SafetySupervisor()
        
    def validate_with_multiple_methods(self,
                                     ego_state: EgoState,
                                     planning_result: PlanningResult,
                                     model_outputs: Dict[str, Any],
                                     tracked_objects: Dict[str, Any]) -> SafetyCheckResult:
        """Validate using multiple safety methods and combine results"""
        
        # Method 1: Physics-based validation
        physics_check = self._physics_validation(ego_state, planning_result)
        
        # Method 2: Rule-based validation (using the main supervisor)
        rule_check = self.supervisor.validate_control_commands(
            ego_state, planning_result, model_outputs, tracked_objects
        )
        
        # Method 3: Model agreement validation
        model_agreement_check = self._model_agreement_validation(planning_result, model_outputs)
        
        # Combine results - if any method flags unsafe, consider unsafe
        all_violations = physics_check.violations + rule_check.violations + model_agreement_check.violations
        combined_confidence = (physics_check.confidence + rule_check.confidence + model_agreement_check.confidence) / 3
        
        is_safe = physics_check.is_safe and rule_check.is_safe and model_agreement_check.is_safe
        recovery_action = self._determine_recovery_action(all_violations, [physics_check, rule_check, model_agreement_check])
        
        return SafetyCheckResult(
            is_safe=is_safe,
            violations=all_violations,
            confidence=combined_confidence,
            explanation=f"Multi-method validation: {'SAFE' if is_safe else 'UNSAFE'}",
            recovery_action=recovery_action
        )
    
    def _physics_validation(self, ego_state: EgoState, planning_result: PlanningResult) -> SafetyCheckResult:
        """Validate based on physics constraints"""
        violations = []
        
        # Check maximum possible acceleration/deceleration
        if abs(planning_result.desired_acceleration) > 5.0:  # Beyond physical limits
            violations.append(SafetyViolation.UNEXPECTED_MANEUVER)
        
        # Check maximum possible curvature (related to speed and lateral acceleration)
        max_lateral_accel = 5.0  # m/s^2 maximum safe lateral acceleration
        if ego_state.velocity > 0 and abs(planning_result.desired_curvature) > max_lateral_accel / (ego_state.velocity ** 2):
            violations.append(SafetyViolation.UNEXPECTED_MANEUVER)
        
        return SafetyCheckResult(
            is_safe=len(violations) == 0,
            violations=violations,
            confidence=1.0,
            explanation="Physics validation check"
        )
    
    def _model_agreement_validation(self, planning_result: PlanningResult, model_outputs: Dict[str, Any]) -> SafetyCheckResult:
        """Check agreement between planning and model predictions"""
        violations = []
        
        # In a real system, this would compare planned actions with model predictions
        # For now, we'll just check if planned confidence is consistent with model confidence
        planned_confidence = getattr(planning_result, 'confidence', 0.8)
        model_confidence = model_outputs.get('overall_confidence', 0.8)
        
        # If there's a large discrepancy, flag potential model disagreement
        if abs(planned_confidence - model_confidence) > 0.3:
            violations.append(SafetyViolation.MODEL_DISAGREEMENT)
        
        return SafetyCheckResult(
            is_safe=len(violations) == 0,
            violations=violations,
            confidence=0.9 if not violations else 0.5,
            explanation="Model agreement validation"
        )
    
    def _determine_recovery_action(self, 
                                  all_violations: List[SafetyViolation], 
                                  individual_checks: List[SafetyCheckResult]) -> Optional[str]:
        """Determine the most appropriate recovery action based on violations"""
        if SafetyViolation.COLLISION_IMMINENT in all_violations:
            return "EMERGENCY_BRAKE"
        elif SafetyViolation.ROAD_BOUNDARY_VIOLATION in all_violations:
            return "RETURN_TO_LANE"
        elif SafetyViolation.SPEED_LIMIT_EXCEEDED in all_violations:
            return "REDUCE_SPEED"
        elif SafetyViolation.UNEXPECTED_MANEUVER in all_violations:
            return "STABILIZE_CONTROL"
        else:
            # Check individual check recovery actions
            for check in individual_checks:
                if check.recovery_action:
                    return check.recovery_action
            return "MONITOR_AND_ADAPT"


def create_safety_supervisor() -> Tuple[SafetySupervisor, RedundantSafetyValidator]:
    """Factory function to create safety supervisor system"""
    return SafetySupervisor(), RedundantSafetyValidator()


if __name__ == "__main__":
    print("Safety Supervisor for Sunnypilot - Testing")
    print("=" * 50)
    
    # Create mock objects for testing
    class MockTrackedObject:
        def __init__(self, state, class_name="car", confidence=0.8):
            self.state = state
            self.class_name = class_name
            self.confidence = confidence
    
    class MockPlanningResult:
        def __init__(self, desired_curvature, desired_speed, desired_acceleration, confidence=0.9):
            self.desired_curvature = desired_curvature
            self.desired_speed = desired_speed
            self.desired_acceleration = desired_acceleration
            self.planning_state = None
            self.confidence = confidence
    
    # Create safety supervisor
    supervisor, validator = create_safety_supervisor()
    
    # Test ego state
    ego_state = EgoState(
        position=np.array([0.0, 0.0, 0.0]),
        velocity=25.0,  # 90 km/h
        acceleration=0.0,
        heading=0.0,
        curvature=0.0,
        steering_angle=0.0
    )
    
    # Test with safe conditions
    safe_plan = MockPlanningResult(
        desired_curvature=0.001,  # Slight curve
        desired_speed=25.0,       # Speed limit
        desired_acceleration=0.0,  # No acceleration change
        confidence=0.9
    )
    
    tracked_objects = {
        "obj1": MockTrackedObject(
            np.array([50.0, 0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Far ahead, safe
        )
    }
    
    model_outputs = {
        'overall_confidence': 0.85,
        'lead_confidence': 0.9,
        'lane_confidence': 0.8
    }
    
    # Validate safe conditions
    safety_check = supervisor.validate_control_commands(
        ego_state, safe_plan, model_outputs, tracked_objects
    )
    
    print(f"Safe plan validation: {safety_check.is_safe}")
    print(f"Violations: {safety_check.violations}")
    print(f"Confidence: {safety_check.confidence:.2f}")
    
    # Test with hazardous conditions
    hazardous_plan = MockPlanningResult(
        desired_curvature=0.05,   # Very sharp curve
        desired_speed=30.0,       # Above speed limit
        desired_acceleration=-6.0, # Hard braking
        confidence=0.6  # Low confidence
    )
    
    close_object = MockTrackedObject(
        np.array([10.0, 0.0, 0.0, 15.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Very close ahead
    )
    
    tracked_objects["close_obj"] = close_object
    
    # Validate hazardous conditions
    hazardous_check = supervisor.validate_control_commands(
        ego_state, hazardous_plan, model_outputs, tracked_objects
    )
    
    print(f"\nHazardous plan validation: {hazardous_check.is_safe}")
    print(f"Hazardous violations: {hazardous_check.violations}")
    print(f"Recovery action: {hazardous_check.recovery_action}")
    
    # Test redundant validation
    redundant_check = validator.validate_with_multiple_methods(
        ego_state, hazardous_plan, model_outputs, tracked_objects
    )
    
    print(f"\nRedundant validation: {redundant_check.is_safe}")
    print(f"Redundant violations: {redundant_check.violations}")
    print(f"Combined confidence: {redundant_check.confidence:.2f}")
    print(f"Redundant recovery action: {redundant_check.recovery_action}")
    
    print("\nSafety supervisor system ready for integration with control validation")