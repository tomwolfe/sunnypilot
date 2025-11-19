"""
Redundancy and Safety Fallback Systems for Sunnypilot
Implements multiple layers of redundancy and fallback mechanisms for enhanced safety
"""

import numpy as np
from typing import Dict, List, Optional, Tuple, Any, Callable
from dataclasses import dataclass
from enum import Enum
import time
import threading
from functools import wraps
import logging


class SafetySystemStatus(Enum):
    """Status of various safety systems"""
    OPERATIONAL = "operational"
    DEGRADED = "degraded" 
    FAILED = "failed"
    BYPASSED = "bypassed"


class FallbackLevel(Enum):
    """Levels of fallback that can be triggered"""
    LANE_ASSIST_ONLY = "lane_assist_only"
    ADAPTIVE_CRUISE_ONLY = "adaptive_cruise_only"
    PASIVE_MODE = "passive_mode"
    DISENGAGE = "disengage"


@dataclass
class SystemHealth:
    """Health status of a system component"""
    name: str
    status: SafetySystemStatus
    confidence: float  # 0.0 to 1.0
    last_update: float
    error_count: int
    error_rate: float
    response_time: float
    fallback_triggers: List[str]


class RedundancyManager:
    """Manages redundant systems and provides fallback capabilities"""
    
    def __init__(self):
        self.systems: Dict[str, SystemHealth] = {}
        self.fallback_handlers: Dict[FallbackLevel, List[Callable]] = {}
        self.active_fallback: Optional[FallbackLevel] = None
        self.backup_systems: Dict[str, Any] = {}
        self.monitoring_thread = None
        self.monitoring_active = False
        self.system_monitor_lock = threading.Lock()
        
    def register_system(self, name: str, initial_status: SafetySystemStatus = SafetySystemStatus.OPERATIONAL):
        """Register a system for monitoring"""
        with self.system_monitor_lock:
            self.systems[name] = SystemHealth(
                name=name,
                status=initial_status,
                confidence=1.0,
                last_update=time.time(),
                error_count=0,
                error_rate=0.0,
                response_time=0.0,
                fallback_triggers=[]
            )
    
    def update_system_health(self, name: str, status: SafetySystemStatus, confidence: float = 1.0, 
                           response_time: float = 0.0):
        """Update the health status of a system"""
        with self.system_monitor_lock:
            if name in self.systems:
                system = self.systems[name]
                system.status = status
                system.confidence = confidence
                system.last_update = time.time()
                system.response_time = response_time
    
    def get_system_health(self, name: str) -> Optional[SystemHealth]:
        """Get the health status of a specific system"""
        with self.system_monitor_lock:
            return self.systems.get(name)
    
    def register_fallback_handler(self, level: FallbackLevel, handler: Callable):
        """Register a handler for specific fallback level"""
        if level not in self.fallback_handlers:
            self.fallback_handlers[level] = []
        self.fallback_handlers[level].append(handler)
    
    def trigger_fallback(self, level: FallbackLevel, reason: str = ""):
        """Trigger a fallback to a specific level"""
        print(f"SAFETY: Triggering fallback to {level.value} - Reason: {reason}")
        
        self.active_fallback = level
        
        # Execute registered handlers for this level
        if level in self.fallback_handlers:
            for handler in self.fallback_handlers[level]:
                try:
                    handler(level, reason)
                except Exception as e:
                    print(f"Error in fallback handler: {e}")
    
    def evaluate_system_safety(self) -> Tuple[FallbackLevel, str]:
        """Evaluate overall system safety and determine if fallback is needed"""
        critical_systems = ['perception', 'planning', 'control', 'validation']
        
        # Check for critical system failures
        for system_name in critical_systems:
            if system_name in self.systems:
                system = self.systems[system_name]
                
                # Trigger disengage if critical system failed
                if system.status == SafetySystemStatus.FAILED:
                    return FallbackLevel.DISENGAGE, f"Critical system {system_name} failed"
                
                # Trigger disengage if confidence too low
                if system.confidence < 0.3:
                    return FallbackLevel.DISENGAGE, f"Low confidence in {system_name}: {system.confidence:.2f}"
                
                # Check response time
                if system.response_time > 0.5:  # 500ms is too slow for safety-critical systems
                    return FallbackLevel.DISENGAGE, f"Slow response from {system_name}: {system.response_time:.3f}s"
        
        # Check for degraded perception
        perception_system = self.systems.get('perception')
        if perception_system and perception_system.confidence < 0.6:
            # Fall back to adaptive cruise only if perception is degraded
            return FallbackLevel.ADAPTIVE_CRUISE_ONLY, f"Perception degraded: {perception_system.confidence:.2f}"
        
        # Check for degraded planning
        planning_system = self.systems.get('planning')
        if planning_system and planning_system.confidence < 0.65:
            # Fall back to lane assist only if planning is degraded
            return FallbackLevel.LANE_ASSIST_ONLY, f"Planning degraded: {planning_system.confidence:.2f}"
        
        # If all systems are operational, return no fallback needed
        return FallbackLevel.PASIVE_MODE, "All systems nominal"  # Return passive as default safe state
    
    def start_monitoring(self):
        """Start the system monitoring thread"""
        if self.monitoring_thread is None:
            self.monitoring_active = True
            self.monitoring_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
            self.monitoring_thread.start()
    
    def stop_monitoring(self):
        """Stop the system monitoring thread"""
        self.monitoring_active = False
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=2.0)
    
    def _monitoring_loop(self):
        """Background monitoring loop"""
        while self.monitoring_active:
            try:
                fallback_level, reason = self.evaluate_system_safety()
                if fallback_level != FallbackLevel.PASIVE_MODE:  # If we need a fallback
                    self.trigger_fallback(fallback_level, reason)
                
                time.sleep(0.1)  # Check every 100ms
            except Exception as e:
                print(f"Error in monitoring loop: {e}")
                time.sleep(0.5)  # Longer sleep on error


class ValidationRedundancy:
    """Multiple validation layers for safety"""
    
    def __init__(self):
        self.validation_layers = [
            self._model_output_validation,
            self._physics_consistency_check,
            self._temporal_coherence_check,
            self._environmental_feasibility_check
        ]
        self.confidence_history = {}
    
    def _model_output_validation(self, model_output: Dict[str, Any]) -> Tuple[bool, float, str]:
        """Validate model outputs make sense"""
        issues = []
        score = 1.0
        
        # Check for NaN or infinity values
        for key, value in model_output.items():
            if isinstance(value, np.ndarray):
                if np.any(np.isnan(value)) or np.any(np.isinf(value)):
                    issues.append(f"Invalid values in {key}")
                    score *= 0.1
            elif isinstance(value, (float, int)) and (np.isnan(value) or np.isinf(value)):
                issues.append(f"Invalid value in {key}")
                score *= 0.1
        
        # Check lead vehicle validity
        if 'leads_v3' in model_output:
            for i, lead in enumerate(model_output['leads_v3']):
                if hasattr(lead, 'dRel') and lead.dRel < 0:
                    issues.append(f"Negative distance for lead {i}")
                    score *= 0.5
                if hasattr(lead, 'prob') and (lead.prob < 0 or lead.prob > 1):
                    issues.append(f"Invalid probability for lead {i}")
                    score *= 0.7
        
        status = len(issues) == 0
        message = f"Model validation: {'PASS' if status else 'FAIL'} - {', '.join(issues) if issues else 'OK'}"
        return status, score, message
    
    def _physics_consistency_check(self, model_output: Dict[str, Any], car_state: Dict[str, Any]) -> Tuple[bool, float, str]:
        """Check if outputs are physically consistent"""
        score = 1.0
        issues = []
        
        # Check acceleration limits
        if 'acceleration' in model_output:
            max_acc = car_state.get('max_acceleration', 3.0)
            min_acc = car_state.get('min_acceleration', -5.0)
            
            if hasattr(model_output['acceleration'], '__len__'):
                for acc in model_output['acceleration']:
                    if acc > max_acc or acc < min_acc:
                        issues.append(f"Acceleration out of bounds: {acc}")
                        score *= 0.3
            else:
                acc = model_output['acceleration']
                if acc > max_acc or acc < min_acc:
                    issues.append(f"Acceleration out of bounds: {acc}")
                    score *= 0.3
        
        # Check speed consistency
        current_speed = car_state.get('vEgo', 0)
        if 'desiredSpeed' in model_output:
            desired_speed = model_output['desiredSpeed']
            if abs(desired_speed - current_speed) > 15:  # More than 15 m/s change in a step
                issues.append(f"Excessive speed change requested: {desired_speed - current_speed}")
                score *= 0.5
        
        status = len(issues) == 0
        message = f"Physics validation: {'PASS' if status else 'FAIL'} - {', '.join(issues) if issues else 'OK'}"
        return status, score, message
    
    def _temporal_coherence_check(self, model_output: Dict[str, Any], previous_output: Optional[Dict[str, Any]]) -> Tuple[bool, float, str]:
        """Check temporal consistency between frames"""
        if previous_output is None:
            return True, 1.0, "Temporal validation: OK - First frame"
        
        score = 1.0
        issues = []
        
        # Check for excessive jumps in lead positions
        for attr in ['dRel', 'yRel', 'vRel']:
            if f'leads_v3' in model_output and f'leads_v3' in previous_output:
                for curr_lead, prev_lead in zip(model_output['leads_v3'], previous_output['leads_v3']):
                    if hasattr(curr_lead, attr) and hasattr(prev_lead, attr):
                        change = abs(getattr(curr_lead, attr) - getattr(prev_lead, attr))
                        if change > 5.0:  # Excessive jump in 100ms
                            issues.append(f"Excessive {attr} jump: {change:.2f}")
                            score *= 0.5
        
        status = len(issues) == 0
        message = f"Temporal validation: {'PASS' if status else 'FAIL'} - {', '.join(issues) if issues else 'OK'}"
        return status, score, message
    
    def _environmental_feasibility_check(self, model_output: Dict[str, Any], env_context: Dict[str, Any]) -> Tuple[bool, float, str]:
        """Check if plan is feasible in current environment"""
        score = 1.0
        issues = []
        
        # Check for conflicts with navigation or known obstacles
        if 'route' in env_context and 'plan' in model_output:
            # This would check if the planned path conflicts with navigation route
            pass
        
        # Check lane feasibility
        if 'lane_info' in env_context and 'lane_change' in model_output:
            # Check if a lane change is physically possible given lane width and other vehicles
            pass
        
        status = len(issues) == 0
        message = f"Environmental validation: {'PASS' if status else 'FAIL'} - {', '.join(issues) if issues else 'OK'}"
        return status, score, message
    
    def validate_comprehensive(self, model_output: Dict[str, Any], 
                             car_state: Dict[str, Any],
                             previous_output: Optional[Dict[str, Any]] = None,
                             env_context: Optional[Dict[str, Any]] = None) -> Tuple[bool, float, List[str]]:
        """Run all validation layers"""
        if env_context is None:
            env_context = {}
        
        all_passed = True
        total_score = 1.0
        all_messages = []
        
        for validation_func in self.validation_layers:
            try:
                passed, score, message = validation_func(model_output, car_state)
                all_passed = all_passed and passed
                total_score *= score
                all_messages.append(message)
            except Exception as e:
                print(f"Validation layer failed: {e}")
                all_passed = False
                total_score *= 0.1
                all_messages.append(f"Validation layer error: {e}")
        
        # Apply temporal smoothing to the score
        output_id = model_output.get('frame_id', 0)
        if output_id in self.confidence_history:
            # Use exponentially weighted average to smooth confidence
            prev_conf = self.confidence_history[output_id]
            smoothed_score = 0.7 * total_score + 0.3 * prev_conf
        else:
            smoothed_score = total_score
        
        self.confidence_history[output_id] = smoothed_score
        
        return all_passed, smoothed_score, all_messages


class SafetyMonitor:
    """Comprehensive safety monitoring system"""
    
    def __init__(self):
        self.redundancy_manager = RedundancyManager()
        self.validation_redundancy = ValidationRedundancy()
        self.safety_limits = {
            'max_lat_accel': 2.5,      # m/s^2
            'max_long_accel': 3.0,     # m/s^2  
            'max_long_decel': -5.0,    # m/s^2
            'min_safe_distance': 2.0,  # m
            'max_response_time': 0.1   # s
        }
        self.emergency_stop_triggered = False
        self.last_safety_check = 0.0
    
    def register_systems(self):
        """Register all systems for monitoring"""
        systems = [
            'perception',
            'planning', 
            'control',
            'validation',
            'communication',
            'power_management'
        ]
        
        for system in systems:
            self.redundancy_manager.register_system(system)
    
    def validate_model_output(self, model_output: Dict[str, Any], 
                            car_state: Dict[str, Any],
                            previous_output: Optional[Dict[str, Any]] = None,
                            env_context: Optional[Dict[str, Any]] = None) -> Tuple[bool, Dict[str, Any]]:
        """Validate model output with multiple redundancy layers"""
        validation_passed, confidence, messages = self.validation_redundancy.validate_comprehensive(
            model_output, car_state, previous_output, env_context
        )
        
        # Check safety limits
        if 'lateral_control' in model_output:
            lat_accel = model_output['lateral_control'].get('acceleration', 0)
            if abs(lat_accel) > self.safety_limits['max_lat_accel']:
                validation_passed = False
                confidence *= 0.1
                messages.append(f"Lateral acceleration limit exceeded: {lat_accel}")
        
        if 'longitudinal_control' in model_output:
            long_accel = model_output['longitudinal_control'].get('acceleration', 0)
            if (long_accel > self.safety_limits['max_long_accel'] or 
                long_accel < self.safety_limits['max_long_decel']):
                validation_passed = False
                confidence *= 0.1
                messages.append(f"Longitudinal acceleration limit exceeded: {long_accel}")
        
        validation_result = {
            'passed': validation_passed,
            'confidence': confidence,
            'messages': messages,
            'timestamp': time.time()
        }
        
        # Update system health
        self.redundancy_manager.update_system_health(
            'validation', 
            SafetySystemStatus.OPERATIONAL if validation_passed else SafetySystemStatus.DEGRADED,
            confidence
        )
        
        return validation_passed, validation_result
    
    def check_emergency_conditions(self, car_state: Dict[str, Any], 
                                 model_output: Dict[str, Any],
                                 env_context: Optional[Dict[str, Any]] = None) -> bool:
        """Check for emergency conditions requiring immediate action"""
        if env_context is None:
            env_context = {}
        
        current_time = time.time()
        
        # Check if too close to vehicle ahead
        safe_distance = max(
            self.safety_limits['min_safe_distance'],
            car_state.get('vEgo', 0) * 1.5  # 1.5 second rule
        )
        
        if 'leads_v3' in model_output:
            closest_lead = min(
                [lead.dRel for lead in model_output['leads_v3'] if lead.prob > 0.5] or [float('inf')]
            )
            
            if closest_lead < safe_distance * 0.5:  # Less than half safe distance
                print(f"EMERGENCY: Too close to lead vehicle: {closest_lead:.1f}m, safe: {safe_distance:.1f}m")
                return True
        
        # Check for system failures
        if self.redundancy_manager.active_fallback is not None:
            if self.redundancy_manager.active_fallback == FallbackLevel.DISENGAGE:
                print("EMERGENCY: System disengagement required")
                return True
        
        # Check for power or communication failures
        power_status = env_context.get('power_status', 'normal')
        if power_status == 'critical':
            print("EMERGENCY: Critical power failure detected")
            return True
        
        comm_status = env_context.get('communication_status', 'normal')
        if comm_status == 'failed':
            print("EMERGENCY: Communication failure detected")
            return True
        
        return False
    
    def get_safety_recommendation(self, car_state: Dict[str, Any],
                                model_output: Dict[str, Any],
                                env_context: Optional[Dict[str, Any]] = None) -> Tuple[FallbackLevel, str]:
        """Get safety recommendation based on current conditions"""
        if env_context is None:
            env_context = {}
        
        # Check for emergency conditions first
        if self.check_emergency_conditions(car_state, model_output, env_context):
            return FallbackLevel.DISENGAGE, "Emergency condition detected"
        
        # Validate the model output
        validation_passed, validation_result = self.validate_model_output(
            model_output, car_state, env_context=env_context
        )
        
        if not validation_passed or validation_result['confidence'] < 0.3:
            return FallbackLevel.DISENGAGE, f"Model validation failed: {validation_result['confidence']:.2f}"
        elif validation_result['confidence'] < 0.6:
            return FallbackLevel.ADAPTIVE_CRUISE_ONLY, f"Low validation confidence: {validation_result['confidence']:.2f}"
        elif validation_result['confidence'] < 0.8:
            return FallbackLevel.LANE_ASSIST_ONLY, f"Medium validation confidence: {validation_result['confidence']:.2f}"
        
        # Check other system health indicators
        system_health = self.redundancy_manager.systems
        for name, health in system_health.items():
            if health.confidence < 0.5 and health.status == SafetySystemStatus.DEGRADED:
                return FallbackLevel.LANE_ASSIST_ONLY, f"Degraded {name} system: {health.confidence:.2f}"
        
        return FallbackLevel.PASIVE_MODE, "All systems nominal"
    
    def initialize_monitoring(self):
        """Initialize safety monitoring"""
        self.register_systems()
        self.redundancy_manager.start_monitoring()
        
        # Register some basic fallback handlers
        def disengage_handler(fallback_level, reason):
            print(f"SAFETY: Initiating disengagement due to {reason}")
            # In real system, this would send disengage command to controls
        
        def lane_assist_handler(fallback_level, reason):
            print(f"SAFETY: Switching to lane assist mode due to {reason}")
            # In real system, this would limit functionality to lane keeping
        
        self.redundancy_manager.register_fallback_handler(FallbackLevel.DISENGAGE, disengage_handler)
        self.redundancy_manager.register_fallback_handler(FallbackLevel.LANE_ASSIST_ONLY, lane_assist_handler)


def get_safety_monitor() -> SafetyMonitor:
    """Get the global safety monitor instance"""
    if not hasattr(get_safety_monitor, 'instance'):
        get_safety_monitor.instance = SafetyMonitor()
        get_safety_monitor.instance.initialize_monitoring()
    return get_safety_monitor.instance


# Global instance for use across the system
safety_monitor = get_safety_monitor()


# Example usage and testing
if __name__ == "__main__":
    print("Testing Safety and Redundancy Systems...")
    
    monitor = get_safety_monitor()
    
    # Simulate a normal operation
    normal_model_output = {
        'frame_id': 1,
        'leads_v3': [
            type('Lead', (), {'dRel': 50.0, 'yRel': 0.0, 'vRel': 0.0, 'prob': 0.9})()
        ],
        'lateral_control': {'acceleration': 0.5},
        'longitudinal_control': {'acceleration': 1.0}
    }
    
    normal_car_state = {
        'vEgo': 15.0,
        'max_acceleration': 3.0,
        'min_acceleration': -5.0
    }
    
    validation_passed, validation_result = monitor.validate_model_output(
        normal_model_output, normal_car_state
    )
    
    print(f"Normal validation passed: {validation_passed}")
    print(f"Validation confidence: {validation_result['confidence']:.2f}")
    
    # Test with problematic output
    bad_model_output = {
        'frame_id': 2,
        'leads_v3': [
            type('Lead', (), {'dRel': -5.0, 'yRel': 0.0, 'vRel': 0.0, 'prob': 0.9})()  # Negative distance
        ],
        'lateral_control': {'acceleration': 10.0},  # Excessive acceleration
        'longitudinal_control': {'acceleration': 1.0}
    }
    
    bad_validation_passed, bad_validation_result = monitor.validate_model_output(
        bad_model_output, normal_car_state
    )
    
    print(f"Bad validation passed: {bad_validation_passed}")
    print(f"Bad validation confidence: {bad_validation_result['confidence']:.2f}")
    
    # Check safety recommendation
    fallback_level, reason = monitor.get_safety_recommendation(
        normal_car_state, normal_model_output
    )
    
    print(f"Safety recommendation: {fallback_level.value} - {reason}")
    
    # Test emergency condition
    close_lead_output = {
        'frame_id': 3,
        'leads_v3': [
            type('Lead', (), {'dRel': 1.0, 'yRel': 0.0, 'vRel': 0.0, 'prob': 0.9})()  # Very close
        ]
    }
    
    emergency_check = monitor.check_emergency_conditions(
        {'vEgo': 15.0}, close_lead_output
    )
    
    print(f"Emergency condition detected: {emergency_check}")
    
    print("Safety and redundancy systems test completed!")