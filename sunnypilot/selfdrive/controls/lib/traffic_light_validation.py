"""
Enhanced Traffic Light and Sign Validation for Sunnypilot
Provides advanced validation for traffic light and sign detection to improve safety
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
import time
from enum import Enum

from cereal import log
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.advanced_planner import PlanningResult, EgoState


class TrafficSignType(Enum):
    """Types of traffic signs and signals"""
    TRAFFIC_LIGHT_RED = "traffic_light_red"
    TRAFFIC_LIGHT_YELLOW = "traffic_light_yellow" 
    TRAFFIC_LIGHT_GREEN = "traffic_light_green"
    STOP_SIGN = "stop_sign"
    YIELD_SIGN = "yield_sign"
    SPEED_LIMIT = "speed_limit"
    PEDESTRIAN_CROSSING = "pedestrian_crossing"
    SCHOOL_ZONE = "school_zone"
    CONSTRUCTION_ZONE = "construction_zone"


@dataclass
class TrafficSignData:
    """Data structure for detected traffic sign"""
    sign_type: TrafficSignType
    distance: float  # Distance to sign in meters
    confidence: float  # Detection confidence (0-1)
    position: np.ndarray  # Position [x, y, z]
    validity_time: float  # Time until data is considered valid
    additional_data: Dict[str, Any] = None


@dataclass
class TrafficValidationResult:
    """Result of traffic validation"""
    is_valid: bool
    violations: List[str]
    confidence: float
    recommended_action: str
    next_check_time: float


class TrafficLightValidator:
    """
    Advanced traffic light and sign validation system
    """
    
    def __init__(self):
        self.traffic_signs: List[TrafficSignData] = []
        self.validation_history = []
        self.max_history = 50
        
        # Validation thresholds
        self.traffic_light_validation = {
            'min_confidence': 0.8,
            'max_response_time': 2.0,  # seconds
            'required_distance_buffer': 50.0  # meters before light
        }
        
        self.stop_sign_validation = {
            'min_confidence': 0.7,
            'required_distance_buffer': 30.0,
            'stop_duration': 2.0  # seconds to hold stop
        }
        
        self.speed_limit_validation = {
            'confidence_threshold': 0.75,
            'speed_tolerance': 5.0  # m/s tolerance
        }

    def add_traffic_sign_data(self, sign_data: TrafficSignData):
        """Add detected traffic sign data to validation"""
        self.traffic_signs.append(sign_data)
        
        # Remove expired signs
        current_time = time.time()
        self.traffic_signs = [
            sign for sign in self.traffic_signs 
            if current_time < sign.validity_time
        ]

    def validate_traffic_lights(self, 
                              ego_state: EgoState, 
                              car_state: log.CarState) -> TrafficValidationResult:
        """Validate traffic light compliance"""
        violations = []
        recommended_action = "NONE"
        
        # Find traffic lights in range
        traffic_lights = [
            sign for sign in self.traffic_signs 
            if sign.sign_type in [TrafficSignType.TRAFFIC_LIGHT_RED, 
                                TrafficSignType.TRAFFIC_LIGHT_YELLOW, 
                                TrafficSignType.TRAFFIC_LIGHT_GREEN]
        ]
        
        if not traffic_lights:
            return TrafficValidationResult(
                is_valid=True,
                violations=[],
                confidence=1.0,
                recommended_action=recommended_action,
                next_check_time=time.time() + 0.1
            )
        
        # Get closest traffic light
        closest_light = min(
            traffic_lights, 
            key=lambda x: np.linalg.norm(ego_state.position - x.position)
        )
        
        distance_to_light = np.linalg.norm(ego_state.position - closest_light.position)
        
        # Calculate stopping distance based on current speed
        current_speed = ego_state.velocity
        stopping_distance = self._calculate_stopping_distance(current_speed, ego_state.acceleration)
        
        # Check if approach is appropriate for light status
        if closest_light.sign_type == TrafficSignType.TRAFFIC_LIGHT_RED:
            if current_speed > 0.5 and distance_to_light < stopping_distance:
                violations.append("APPROACHING_RED_LIGHT_TOO_FAST")
                recommended_action = "BRAKE_IMMEDIATELY"
            elif distance_to_light < 10.0 and current_speed > 1.0:
                violations.append("TOO_CLOSE_RED_LIGHT")
                recommended_action = "STOP_IMMEDIATELY"
                
        elif closest_light.sign_type == TrafficSignType.TRAFFIC_LIGHT_YELLOW:
            if current_speed > 5.0 and distance_to_light < stopping_distance * 0.7:
                # More lenient with yellow lights - if close, may go through
                violations.append("APPROACHING_YELLOW_LIGHT_MAY_NOT_STOP")
                recommended_action = "EVALUATE_CONTINUE_OR_STOP"
                
        elif closest_light.sign_type == TrafficSignType.TRAFFIC_LIGHT_GREEN:
            # Verify we can proceed safely
            if current_speed < 0.5 and distance_to_light < 20.0:
                violations.append("UNEXPECTED_STOP_AT_GREEN_LIGHT")
                recommended_action = "PROCEED_WITH_CAUTION"
        
        # Check detection confidence
        if closest_light.confidence < self.traffic_light_validation['min_confidence']:
            violations.append("LOW_CONFIDENCE_TRAFFIC_LIGHT_DETECTION")
            recommended_action = "REDUCE_SPEED_VALIDATE"
        
        return TrafficValidationResult(
            is_valid=len(violations) == 0,
            violations=violations,
            confidence=closest_light.confidence,
            recommended_action=recommended_action,
            next_check_time=time.time() + 0.05  # Check every 50ms for traffic lights
        )

    def validate_stop_signs(self, 
                          ego_state: EgoState, 
                          car_state: log.CarState) -> TrafficValidationResult:
        """Validate stop sign compliance"""
        violations = []
        recommended_action = "NONE"
        
        # Find stop signs in range
        stop_signs = [
            sign for sign in self.traffic_signs 
            if sign.sign_type == TrafficSignType.STOP_SIGN
        ]
        
        if not stop_signs:
            return TrafficValidationResult(
                is_valid=True,
                violations=[],
                confidence=1.0,
                recommended_action=recommended_action,
                next_check_time=time.time() + 0.2
            )
        
        # Get closest stop sign
        closest_sign = min(
            stop_signs, 
            key=lambda x: np.linalg.norm(ego_state.position - x.position)
        )
        
        distance_to_sign = np.linalg.norm(ego_state.position - closest_sign.position)
        
        # Check if we're approaching stop sign appropriately
        current_speed = ego_state.velocity
        
        # Required distance to start preparing to stop
        if distance_to_sign < self.stop_sign_validation['required_distance_buffer'] and current_speed > 2.0:
            violations.append("APPROACHING_STOP_SIGN_TOO_FAST")
            recommended_action = "BRAKE_FOR_STOP"
        
        # Check if we're supposed to stop at this sign
        if distance_to_sign < 15.0:
            if current_speed > 0.5:
                violations.append("NOT_STOPPING_AT_STOP_SIGN")
                recommended_action = "STOP_IMMEDIATELY"
        
        # Check detection confidence
        if closest_sign.confidence < self.stop_sign_validation['min_confidence']:
            violations.append("LOW_CONFIDENCE_STOP_SIGN_DETECTION")
            recommended_action = "REDUCE_SPEED_VALIDATE"
        
        return TrafficValidationResult(
            is_valid=len(violations) == 0,
            violations=violations,
            confidence=closest_sign.confidence,
            recommended_action=recommended_action,
            next_check_time=time.time() + 0.1
        )

    def validate_speed_limits(self, 
                            ego_state: EgoState, 
                            car_state: log.CarState) -> TrafficValidationResult:
        """Validate speed limit compliance"""
        violations = []
        recommended_action = "NONE"
        
        # Find speed limit signs in range
        speed_limit_signs = [
            sign for sign in self.traffic_signs 
            if sign.sign_type == TrafficSignType.SPEED_LIMIT
        ]
        
        if not speed_limit_signs:
            return TrafficValidationResult(
                is_valid=True,
                violations=[],
                confidence=1.0,
                recommended_action=recommended_action,
                next_check_time=time.time() + 0.5
            )
        
        # Get most relevant speed limit sign (closest or most recent)
        relevant_sign = max(
            speed_limit_signs,
            key=lambda x: x.confidence if x.confidence else 0
        )
        
        if relevant_sign.confidence < self.speed_limit_validation['confidence_threshold']:
            return TrafficValidationResult(
                is_valid=True,  # Can't validate with low confidence
                violations=["LOW_CONFIDENCE_SPEED_LIMIT_DETECTION"],
                confidence=relevant_sign.confidence,
                recommended_action="MAINTAIN_CAUTIOUS_SPEED",
                next_check_time=time.time() + 0.3
            )
        
        # Get speed limit from sign data
        speed_limit = relevant_sign.additional_data.get('speed_limit', 25.0) if relevant_sign.additional_data else 25.0
        
        current_speed = ego_state.velocity
        
        # Check if exceeding speed limit
        if current_speed > (speed_limit + self.speed_limit_validation['speed_tolerance']):
            violations.append(f"EXCEEDING_SPEED_LIMIT_{speed_limit:.1f}_MS")
            recommended_action = "REDUCE_SPEED"
        
        return TrafficValidationResult(
            is_valid=len(violations) == 0,
            violations=violations,
            confidence=relevant_sign.confidence,
            recommended_action=recommended_action,
            next_check_time=time.time() + 0.2
        )

    def _calculate_stopping_distance(self, speed: float, deceleration: float) -> float:
        """Calculate stopping distance based on current speed and deceleration"""
        if speed <= 0:
            return 0.0
        
        # Use a conservative deceleration if not provided
        if deceleration >= 0:  # Positive acceleration, use default
            deceleration = -3.0  # m/s^2
        
        # Stopping distance = v^2 / (2 * |a|)
        return (speed ** 2) / (2 * abs(deceleration))

    def validate_traffic_compliance(self, 
                                  ego_state: EgoState,
                                  car_state: log.CarState,
                                  planning_result: PlanningResult) -> Dict[str, Any]:
        """Validate overall traffic compliance"""
        # Perform all validations
        traffic_light_result = self.validate_traffic_lights(ego_state, car_state)
        stop_sign_result = self.validate_stop_signs(ego_state, car_state) 
        speed_limit_result = self.validate_speed_limits(ego_state, car_state)
        
        # Aggregate results
        all_violations = []
        all_violations.extend(traffic_light_result.violations)
        all_violations.extend(stop_sign_result.violations)  
        all_violations.extend(speed_limit_result.violations)
        
        # Determine overall safety
        overall_safe = (traffic_light_result.is_valid and 
                       stop_sign_result.is_valid and 
                       speed_limit_result.is_valid)
        
        # Calculate confidence as average of confidences
        confidences = [r.confidence for r in [traffic_light_result, stop_sign_result, speed_limit_result] 
                      if r.confidence > 0]
        avg_confidence = np.mean(confidences) if confidences else 0.5
        
        # Determine most critical action needed
        critical_actions = [r.recommended_action for r in [traffic_light_result, stop_sign_result, speed_limit_result] 
                           if r.recommended_action != "NONE"]
        recommended_action = critical_actions[0] if critical_actions else "NONE"
        
        return {
            'is_compliant': overall_safe,
            'violations': all_violations,
            'confidence': avg_confidence,
            'recommended_action': recommended_action,
            'next_check_time': min(traffic_light_result.next_check_time,
                                 stop_sign_result.next_check_time,
                                 speed_limit_result.next_check_time),
            'traffic_light_validation': {
                'is_valid': traffic_light_result.is_valid,
                'violations': traffic_light_result.violations,
                'confidence': traffic_light_result.confidence
            },
            'stop_sign_validation': {
                'is_valid': stop_sign_result.is_valid,
                'violations': stop_sign_result.violations,
                'confidence': stop_sign_result.confidence
            },
            'speed_limit_validation': {
                'is_valid': speed_limit_result.is_valid,
                'violations': speed_limit_result.violations,
                'confidence': speed_limit_result.confidence
            }
        }

    def update_traffic_signs_with_model_output(self, model_output: Dict[str, Any]):
        """Update traffic sign data based on model outputs - standard dict format"""
        # This would be connected to the actual model output parsing
        # For now, this is a placeholder for integration
        if 'traffic_signs' in model_output:
            for sign_info in model_output['traffic_signs']:
                try:
                    sign_type = TrafficSignType(sign_info['type'])
                    sign_data = TrafficSignData(
                        sign_type=sign_type,
                        distance=sign_info.get('distance', 0.0),
                        confidence=sign_info.get('confidence', 0.5),
                        position=np.array(sign_info.get('position', [0.0, 0.0, 0.0])),
                        validity_time=time.time() + sign_info.get('validity_duration', 5.0),
                        additional_data=sign_info.get('additional_data', {})
                    )
                    self.add_traffic_sign_data(sign_data)
                except ValueError:
                    cloudlog.warning(f"Invalid traffic sign type: {sign_info.get('type')}")


class EnhancedTrafficSafetySystem:
    """
    Enhanced traffic safety system that integrates with the planning and control systems
    """

    def __init__(self):
        self.validator = TrafficLightValidator()
        self.last_validation_result = None

    def add_traffic_sign_data(self, sign_data: TrafficSignData):
        """Add traffic sign data to the validator"""
        self.validator.add_traffic_sign_data(sign_data)
        
    def validate_with_planning(self, 
                             ego_state: EgoState,
                             car_state: log.CarState, 
                             planning_result: PlanningResult,
                             model_outputs: Dict[str, Any]) -> Tuple[bool, List[str], Dict[str, Any]]:
        """
        Validate traffic safety integrated with planning outputs
        Returns: (is_safe, violations, safety_metrics)
        """
        # Update traffic signs from model output
        self.validator.update_traffic_signs_with_model_output(model_outputs)
        
        # Perform comprehensive validation
        validation_result = self.validator.validate_traffic_compliance(
            ego_state, car_state, planning_result
        )
        
        self.last_validation_result = validation_result
        
        # Check if planned actions are compliant with traffic rules
        planned_violations = self._check_planned_actions_compliance(
            planning_result, validation_result
        )
        
        # Combine model-based and planned action violations
        all_violations = validation_result['violations'] + planned_violations
        is_safe = validation_result['is_compliant'] and len(planned_violations) == 0
        
        safety_metrics = {
            'traffic_compliance_score': 1.0 - (len(all_violations) * 0.1),  # Reduce score with each violation
            'detection_confidence': validation_result['confidence'],
            'recommended_action': validation_result['recommended_action']
        }
        
        return is_safe, all_violations, safety_metrics
    
    def _check_planned_actions_compliance(self, 
                                        planning_result: PlanningResult,
                                        validation_result: Dict[str, Any]) -> List[str]:
        """Check if planned actions comply with traffic validation results"""
        violations = []
        
        # Check if planned acceleration is appropriate given traffic light status
        traffic_light_valid = validation_result['traffic_light_validation']['is_valid']
        traffic_light_violations = validation_result['traffic_light_validation']['violations']
        
        if not traffic_light_valid and 'APPROACHING_RED_LIGHT_TOO_FAST' in traffic_light_violations:
            if planning_result.desired_acceleration > 0:
                violations.append("PLANNED_ACCELERATION_CONFLICTS_RED_LIGHT")
        
        # Check if planned speed is appropriate given speed limits
        speed_limit_valid = validation_result['speed_limit_validation']['is_valid']
        if not speed_limit_valid:
            if planning_result.desired_speed > validation_result['speed_limit_validation'].get('speed_limit', planning_result.desired_speed):
                violations.append("PLANNED_SPEED_EXCEEDS_LIMIT")
        
        return violations


def create_traffic_safety_system() -> EnhancedTrafficSafetySystem:
    """Factory function to create traffic safety system"""
    return EnhancedTrafficSafetySystem()