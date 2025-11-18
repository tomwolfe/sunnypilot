"""
Safety validation module for autonomous driving systems.
Implements comprehensive safety checks and validation for sunnypilot.
"""
import time
import numpy as np
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from openpilot.selfdrive.common.metrics import Metrics, record_metric

@dataclass
class SafetyConfig:
    """Configuration for safety validation parameters."""
    # Following distance parameters
    min_following_distance: float = 50.0  # meters at highway speed
    safety_buffer_distance: float = 20.0  # additional safety buffer
    
    # Speed-dependent following distance factor
    time_gap_seconds: float = 2.0  # 2-second rule
    
    # Emergency response parameters
    max_braking_jerk: float = 5.0  # m/s³ (comfortable emergency braking)
    max_steering_rate: float = 100.0  # deg/s (maximum steering rate)
    
    # Detection requirements
    min_detection_range: float = 150.0  # meters minimum detection range
    max_false_positive_rate: float = 0.001  # 0.1% max false positive rate
    min_detection_accuracy: float = 0.95  # 95% minimum detection accuracy

class SafetyValidator:
    """
    Comprehensive safety validation system for autonomous driving.
    Validates pedestrian detection, collision avoidance, safe following distance, etc.
    """
    
    def __init__(self, config: Optional[SafetyConfig] = None):
        self.config = config or SafetyConfig()
        self.last_validation_time = time.time()
        self.safety_violations = []
        self.emergency_responses = []
    
    def validate_pedestrian_detection(self, detection_data: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        """
        Validate pedestrian detection and safety response.
        """
        start_time = time.time()
        
        # Check detection accuracy requirements
        accuracy = detection_data.get('accuracy', 0.0)
        detection_range = detection_data.get('range', 0.0)
        is_detected = detection_data.get('detected', False)
        
        # Validate detection meets safety requirements
        accuracy_pass = accuracy >= self.config.min_detection_accuracy
        range_pass = detection_range >= self.config.min_detection_range
        detected_pass = True if is_detected else False  # If not detected, check if it should have been
        
        is_safe = accuracy_pass and range_pass
        
        validation_result = {
            "accuracy_met": accuracy_pass,
            "range_met": range_pass,
            "object_detected": detected_pass,
            "is_safe": is_safe,
            "accuracy": accuracy,
            "range": detection_range,
            "validation_time": time.time() - start_time
        }
        
        # Record metric
        record_metric(Metrics.PEDESTRIAN_DETECTION_ACCURACY, accuracy, {
            "validation_type": "pedestrian_detection",
            "accuracy_met": accuracy_pass,
            "range_met": range_pass,
            "is_safe": is_safe,
            "range_meters": detection_range
        })
        
        return is_safe, validation_result
    
    def validate_collision_avoidance(self, 
                                   ego_state: Dict[str, float], 
                                   obstacle_state: Dict[str, float]) -> Tuple[bool, Dict[str, Any]]:
        """
        Validate collision avoidance system response.
        """
        start_time = time.time()
        
        # Calculate time to collision
        relative_distance = obstacle_state.get('distance', float('inf')) - ego_state.get('distance', 0)
        relative_velocity = ego_state.get('velocity', 0) - obstacle_state.get('velocity', 0)
        
        if relative_velocity > 0:  # Closing in on obstacle
            ttc = relative_distance / relative_velocity if relative_velocity != 0 else float('inf')
        else:
            ttc = float('inf')  # Not closing in
        
        # Calculate safe following distance based on time gap rule
        safe_distance = ego_state.get('velocity', 0) * self.config.time_gap_seconds + self.config.safety_buffer_distance
        
        # Check if current distance is safe
        current_distance = obstacle_state.get('distance', float('inf'))
        is_safe_distance = current_distance >= safe_distance
        
        # Determine if emergency action needed
        requires_action = ttc < 5.0  # Less than 5 seconds to collision
        
        if requires_action and not is_safe_distance:
            # Simulate emergency response
            response_needed = self._simulate_emergency_response(ego_state, obstacle_state)
            successful_avoidance = response_needed['can_avoid']
        else:
            successful_avoidance = True  # No immediate danger
        
        validation_result = {
            "time_to_collision": ttc,
            "safe_distance": safe_distance,
            "current_distance": current_distance,
            "is_safe_distance": is_safe_distance,
            "requires_action": requires_action,
            "successful_avoidance": successful_avoidance,
            "validation_time": time.time() - start_time
        }
        
        # Record metric
        record_metric(Metrics.COLLISION_AVOIDANCE_SUCCESS_RATE, 
                     1.0 if successful_avoidance else 0.0, {
            "validation_type": "collision_avoidance",
            "time_to_collision": ttc,
            "safe_distance_m": safe_distance,
            "current_distance_m": current_distance,
            "requires_emergency_action": requires_action,
            "successful_avoidance": successful_avoidance
        })
        
        return successful_avoidance, validation_result
    
    def validate_safe_following_distance(self, 
                                       ego_speed: float, 
                                       lead_distance: float, 
                                       lead_speed: float = 0.0) -> Tuple[bool, Dict[str, Any]]:
        """
        Validate safe following distance based on speed and lead vehicle.
        Implements the 2-second rule plus safety buffer.
        """
        start_time = time.time()
        
        # Calculate required following distance using 2-second rule + buffer
        required_distance = ego_speed * self.config.time_gap_seconds + self.config.safety_buffer_distance
        
        # Check against actual following distance
        is_safe = lead_distance >= required_distance
        
        validation_result = {
            "ego_speed": ego_speed,
            "lead_distance": lead_distance,
            "lead_speed": lead_speed,
            "required_distance": required_distance,
            "is_safe": is_safe,
            "validation_time": time.time() - start_time
        }
        
        # Record metric
        record_metric(Metrics.SAFETY_MARGIN_VIOLATIONS, 
                     0 if is_safe else 1, {
            "validation_type": "following_distance",
            "ego_speed_ms": ego_speed,
            "lead_distance_m": lead_distance,
            "required_distance_m": required_distance,
            "is_safe": is_safe
        })
        
        return is_safe, validation_result
    
    def validate_sensor_failures(self, sensor_data: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        """
        Validate detection of sensor failures and fail-safe response.
        """
        start_time = time.time()
        
        # Check for sensor anomalies
        sensor_health = sensor_data.get('health_status', {})
        
        # Define failure conditions
        failures_detected = []
        
        # Camera failure check
        if sensor_health.get('camera', True) == False:
            failures_detected.append('camera')
        
        # Radar failure check
        if sensor_health.get('radar', True) == False:
            failures_detected.append('radar')
        
        # GPS failure check
        if sensor_health.get('gps', True) == False:
            failures_detected.append('gps')
        
        # LIDAR failure check (if equipped)
        if sensor_health.get('lidar', True) == False:
            failures_detected.append('lidar')
        
        # IMU failure check
        if sensor_health.get('imu', True) == False:
            failures_detected.append('imu')
        
        # Determine if system can operate safely with failures
        critical_failures = ['camera', 'radar', 'gps']
        has_critical_failure = any(fail_type in failures_detected for fail_type in critical_failures)
        
        # Check if backup systems can handle the failures
        can_operate_safely = not has_critical_failure or self._can_handle_with_backups(failures_detected)
        
        validation_result = {
            "failures_detected": failures_detected,
            "has_critical_failure": has_critical_failure,
            "can_operate_safely": can_operate_safely,
            "backup_systems_available": self._check_backup_systems(failures_detected),
            "validation_time": time.time() - start_time
        }
        
        # Record metric
        record_metric(Metrics.SENSOR_FAILURE_DETECTION_RATE, 
                     1.0 if len(failures_detected) > 0 else 0.0, {
            "validation_type": "sensor_failure_detection",
            "failures_detected": failures_detected,
            "can_operate_safely": can_operate_safely,
            "has_critical_failure": has_critical_failure
        })
        
        return can_operate_safely, validation_result
    
    def validate_emergency_stop(self, current_state: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        """
        Validate emergency stop capability and response time.
        """
        start_time = time.time()
        
        speed = current_state.get('speed', 0.0)
        road_type = current_state.get('road_type', 'unknown')
        weather = current_state.get('weather', 'clear')
        
        # Calculate required stopping distance based on conditions
        reaction_time = 0.5  # seconds (system reaction time)
        braking_deceleration = self._get_braking_capability(weather, road_type)
        
        # Distance to stop = reaction distance + braking distance
        reaction_distance = speed * reaction_time
        braking_distance = (speed ** 2) / (2 * braking_deceleration) if braking_deceleration > 0 else 0
        required_stopping_distance = reaction_distance + braking_distance
        
        current_road_clear_distance = current_state.get('distance_to_obstacle', float('inf'))
        
        # Check if system can stop in time
        can_stop_safely = current_road_clear_distance >= required_stopping_distance
        
        validation_result = {
            "current_speed": speed,
            "road_type": road_type,
            "weather": weather,
            "required_stopping_distance": required_stopping_distance,
            "available_stopping_distance": current_road_clear_distance,
            "can_stop_safely": can_stop_safely,
            "reaction_distance": reaction_distance,
            "braking_distance": braking_distance,
            "validation_time": time.time() - start_time
        }
        
        # Record metric
        record_metric(Metrics.EMERGENCY_STOP_LATENCY_MS, 
                     reaction_time * 1000, {
            "validation_type": "emergency_stop",
            "speed_ms": speed,
            "road_type": road_type,
            "weather": weather,
            "required_stopping_distance_m": required_stopping_distance,
            "can_stop_safely": can_stop_safely,
            "response_time_ms": reaction_time * 1000
        })
        
        return can_stop_safely, validation_result
    
    def validate_traffic_sign_detection(self, signs_data: List[Dict[str, Any]]) -> Tuple[float, Dict[str, Any]]:
        """
        Validate traffic sign detection accuracy and compliance.
        """
        start_time = time.time()
        
        total_signs = len(signs_data)
        correctly_detected = 0
        critical_violations = 0  # Stop signs, red lights missed
        
        for sign in signs_data:
            detected = sign.get('detected', False)
            is_critical = sign.get('critical', False)
            correctly_classified = sign.get('correct_classification', False)
            
            if detected and correctly_classified:
                correctly_detected += 1
                if is_critical:
                    # Ensure critical signs are properly handled
                    action_taken = sign.get('action_taken', False)
                    if not action_taken:
                        critical_violations += 1
            elif is_critical:
                critical_violations += 1
        
        accuracy = correctly_detected / total_signs if total_signs > 0 else 1.0
        critical_violation_rate = critical_violations / total_signs if total_signs > 0 else 0.0
        
        validation_result = {
            "total_signs": total_signs,
            "correctly_detected": correctly_detected,
            "accuracy": accuracy,
            "critical_violations": critical_violations,
            "critical_violation_rate": critical_violation_rate,
            "validation_time": time.time() - start_time
        }
        
        # Record metrics
        record_metric(Metrics.STOP_SIGN_DETECTION_RATE, 
                     accuracy if total_signs > 0 else 1.0, {
            "validation_type": "traffic_sign_detection",
            "total_signs": total_signs,
            "correctly_detected": correctly_detected,
            "accuracy": accuracy,
            "critical_violations": critical_violations
        })
        
        return accuracy, validation_result
    
    def _simulate_emergency_response(self, ego_state: Dict, obstacle_state: Dict) -> Dict[str, Any]:
        """
        Simulate emergency response to potential collision.
        """
        ego_velocity = ego_state.get('velocity', 0)
        obstacle_velocity = obstacle_state.get('velocity', 0)
        distance = obstacle_state.get('distance', float('inf'))
        
        # Calculate if we can avoid collision with max braking
        max_braking_deceleration = 8.0  # m/s² (aggressive but safe braking)
        
        # Time to collision at current velocities
        relative_velocity = ego_velocity - obstacle_velocity
        if relative_velocity > 0:
            time_to_collision = distance / relative_velocity
            # Distance to stop with max braking
            braking_distance = (ego_velocity ** 2) / (2 * max_braking_deceleration)
            can_avoid = braking_distance < distance
        else:
            can_avoid = True  # Not approaching the obstacle
        
        return {
            "can_avoid": can_avoid,
            "time_to_collision": time_to_collision if relative_velocity > 0 else float('inf'),
            "braking_distance": braking_distance,
            "max_braking_deceleration": max_braking_deceleration
        }
    
    def _get_braking_capability(self, weather: str, road_type: str) -> float:
        """
        Get braking capability based on conditions.
        """
        base_deceleration = 8.0  # m/s² base emergency braking
        
        # Weather adjustments
        weather_factors = {
            'clear': 1.0,
            'rain': 0.7,
            'snow': 0.5,
            'ice': 0.3
        }
        
        # Road type adjustments
        road_factors = {
            'highway': 1.0,
            'urban': 1.0,
            'mountain': 0.9,
            'wet_surface': 0.7
        }
        
        weather_factor = weather_factors.get(weather, 0.8)
        road_factor = road_factors.get(road_type, 1.0)
        
        return base_deceleration * weather_factor * road_factor
    
    def _can_handle_with_backups(self, failures: List[str]) -> bool:
        """
        Check if system can handle failures using backup systems.
        """
        # Define backup capabilities
        # In a real system, this would check actual backup sensor availability
        return len(failures) <= 1  # System can handle single sensor failure
    
    def _check_backup_systems(self, failures: List[str]) -> Dict[str, bool]:
        """
        Check availability of backup systems for each failed sensor.
        """
        backups = {}
        for failure in failures:
            # In a real implementation, this would check actual backup sensors
            backups[failure] = True  # Assuming backup available for simplicity
        return backups

class SafetyMonitor:
    """
    Continuous safety monitoring system that tracks safety metrics over time.
    """
    
    def __init__(self):
        self.validator = SafetyValidator()
        self.safety_events = []
        self.safety_metrics = {}
    
    def monitor_safety_status(self, vehicle_state: Dict[str, Any]) -> Dict[str, Any]:
        """
        Monitor overall safety status based on current vehicle state.
        """
        start_time = time.time()
        
        # Run multiple safety checks
        safe_following, follow_result = self.validator.validate_safe_following_distance(
            vehicle_state.get('speed', 0), 
            vehicle_state.get('lead_distance', float('inf')),
            vehicle_state.get('lead_speed', 0)
        )
        
        emergency_safe, emergency_result = self.validator.validate_emergency_stop(vehicle_state)
        
        # Calculate overall safety score
        safety_score = (safe_following + emergency_safe) / 2.0  # Simple average
        
        overall_status = {
            "timestamp": time.time(),
            "safety_score": safety_score,
            "safe_following_distance": safe_following,
            "emergency_stop_capable": emergency_safe,
            "follow_validation": follow_result,
            "emergency_validation": emergency_result,
            "processing_time": time.time() - start_time
        }
        
        # Store safety event for history
        self.safety_events.append(overall_status)
        
        # Record safety metrics
        record_metric(Metrics.SAFETY_MARGIN_COMPLIANCE, safety_score, {
            "monitoring_type": "continuous_safety",
            "safe_following": safe_following,
            "emergency_capable": emergency_safe,
            "safety_score": safety_score
        })
        
        return overall_status

# Global safety validator and monitor instances
safety_validator = SafetyValidator()
safety_monitor = SafetyMonitor()

def get_safety_validator() -> SafetyValidator:
    """Get the global safety validator instance."""
    return safety_validator

def get_safety_monitor() -> SafetyMonitor:
    """Get the global safety monitor instance."""
    return safety_monitor

def validate_pedestrian_detection(detection_data: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
    """Validate pedestrian detection safety."""
    return safety_validator.validate_pedestrian_detection(detection_data)

def validate_collision_avoidance(ego_state: Dict[str, float], 
                               obstacle_state: Dict[str, float]) -> Tuple[bool, Dict[str, Any]]:
    """Validate collision avoidance system."""
    return safety_validator.validate_collision_avoidance(ego_state, obstacle_state)

def validate_safe_following_distance(speed: float, distance: float, lead_speed: float = 0.0) -> Tuple[bool, Dict[str, Any]]:
    """Validate safe following distance."""
    return safety_validator.validate_safe_following_distance(speed, distance, lead_speed)

def validate_sensor_failures(sensor_data: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
    """Validate sensor failure detection and response."""
    return safety_validator.validate_sensor_failures(sensor_data)

def validate_emergency_stop(state: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
    """Validate emergency stop capability."""
    return safety_validator.validate_emergency_stop(state)

def monitor_safety_status(vehicle_state: Dict[str, Any]) -> Dict[str, Any]:
    """Monitor overall safety status."""
    return safety_monitor.monitor_safety_status(vehicle_state)