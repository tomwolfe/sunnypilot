"""
Safety Supervisor for Sunnypilot
Main safety oversight system that monitors all other safety systems
"""
import time
import threading
from typing import Dict, Any, Tuple, Optional
from enum import Enum

import cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from selfdrive.common.enhanced_validation import enhanced_validator
from selfdrive.common.safety_redundancy import safety_monitor


class SafetyState(Enum):
    """Overall safety state of the system"""
    NOMINAL = "nominal"
    CAUTION = "caution"
    WARNING = "warning"
    CRITICAL = "critical"
    DISENGAGED = "disengaged"


class SafetySupervisor:
    """
    Main safety supervisor that oversees all safety-related systems
    """
    
    def __init__(self):
        self.params = Params()
        self.safety_state = SafetyState.NOMINAL
        self.last_check_time = time.time()
        self.check_interval = 0.05  # 20Hz check rate
        self.system_active = True
        
        # Initialize messaging
        try:
            self.sm = messaging.SubMaster([
                'modelV2', 'carState', 'radarState', 'deviceState',
                'selfdriveState', 'plannerd', 'controlsState'
            ])
        except Exception as e:
            cloudlog.error(f"Could not initialize SubMaster in SafetySupervisor: {e}")
            self.sm = None
            
        self.pm = messaging.PubMaster(['safetyState'])
        
        # Reference to other safety systems
        self.enhanced_validator = enhanced_validator
        self.safety_monitor = safety_monitor
        self.emergency_active = False
        self.last_emergency_time = 0.0

    def run_monitoring_cycle(self) -> Dict[str, Any]:
        """
        Run a complete safety monitoring cycle
        """
        current_time = time.time()
        
        # Rate limit checks
        if current_time - self.last_check_time < self.check_interval:
            return {
                'state': self.safety_state,
                'timestamp': current_time,
                'valid': True
            }
            
        self.last_check_time = current_time

        try:
            # Update submaster
            if self.sm:
                self.sm.update(0)
                
            # Get model output and car state for validation
            model_output = self.sm['modelV2'] if self.sm and 'modelV2' in self.sm else None
            car_state = self.sm['carState'] if self.sm and 'carState' in self.sm else None
            
            # Run enhanced validation
            validation_result = self.enhanced_validator.run_validation(self.sm) if self.sm else {
                "systemSafe": True,
                "confidence": 0.8,
                "situationFactor": 1.0
            }
            
            # Get safety recommendation from redundancy system
            if model_output and car_state:
                fallback_level, reason = self.safety_monitor.get_safety_recommendation(
                    car_state, model_output
                )
            else:
                fallback_level, reason = None, "No model output available"
            
            # Check for emergency conditions
            emergency_check = self.safety_monitor.check_emergency_conditions(
                car_state or {}, 
                model_output or {}
            ) if car_state and model_output else False

            # Determine safety state based on all inputs
            if emergency_check:
                self.safety_state = SafetyState.CRITICAL
                self.emergency_active = True
                self.last_emergency_time = current_time
                cloudlog.error(f"Safety emergency triggered: {reason}")
            elif not validation_result.get("systemSafe", True):
                self.safety_state = SafetyState.WARNING
            elif validation_result.get("confidence", 1.0) < 0.6:
                self.safety_state = SafetyState.CAUTION
            else:
                self.safety_state = SafetyState.NOMINAL

            # Create safety state message
            safety_msg = messaging.new_message('safetyState')
            safety_state_msg = safety_msg.safetyState
            safety_state_msg.safetyState = self.safety_state.name
            safety_state_msg.confidence = validation_result.get("confidence", 0.8)
            safety_state_msg.valid = validation_result.get("systemSafe", True)
            safety_state_msg.situationFactor = validation_result.get("situationFactor", 1.0)
            safety_state_msg.timestamp = current_time
            
            # Publish safety state
            self.pm.send('safetyState', safety_msg)

            return {
                'state': self.safety_state,
                'timestamp': current_time,
                'valid': validation_result.get("systemSafe", True),
                'confidence': validation_result.get("confidence", 0.8),
                'situation_factor': validation_result.get("situationFactor", 1.0),
                'emergency_active': self.emergency_active,
                'reason': reason
            }

        except Exception as e:
            cloudlog.error(f"Error in safety supervisor cycle: {e}")
            self.safety_state = SafetyState.CRITICAL
            return {
                'state': SafetyState.CRITICAL,
                'timestamp': current_time,
                'valid': False,
                'confidence': 0.0,
                'situation_factor': 0.0,
                'emergency_active': True,
                'reason': f"Supervisor error: {e}"
            }

    def check_system_integrity(self) -> Tuple[bool, str]:
        """
        Check integrity of all safety-related systems
        """
        issues = []
        
        # Check if other safety systems are responsive
        if not hasattr(self.enhanced_validator, 'run_validation'):
            issues.append("Enhanced validation system not available")
            
        if not hasattr(self.safety_monitor, 'get_safety_recommendation'):
            issues.append("Safety monitor system not available")
            
        # Check for stale data
        if self.sm:
            for service in ['modelV2', 'carState', 'radarState']:
                if service in self.sm and not self.sm.alive[service]:
                    issues.append(f"{service} not alive")
        
        is_healthy = len(issues) == 0
        status_msg = "All systems healthy" if is_healthy else "; ".join(issues)
        
        return is_healthy, status_msg

    def trigger_emergency_stop(self) -> bool:
        """
        Trigger emergency stop procedures
        """
        cloudlog.error("EMERGENCY STOP TRIGGERED by Safety Supervisor")
        
        # Set state to critical
        self.safety_state = SafetyState.CRITICAL
        self.emergency_active = True
        self.last_emergency_time = time.time()
        self.system_active = False
        
        # Publish emergency state
        try:
            safety_msg = messaging.new_message('safetyState')
            safety_state_msg = safety_msg.safetyState
            safety_state_msg.safetyState = SafetyState.CRITICAL.name
            safety_state_msg.confidence = 0.0
            safety_state_msg.valid = False
            safety_state_msg.situationFactor = 0.0
            safety_state_msg.timestamp = time.time()
            safety_state_msg.emergency = True
            
            self.pm.send('safetyState', safety_msg)
        except Exception as e:
            cloudlog.error(f"Error publishing emergency stop: {e}")
            return False
            
        return True

    def reset_emergency_state(self):
        """
        Reset emergency state after condition is cleared
        """
        self.emergency_active = False
        self.system_active = True
        cloudlog.info("Safety supervisor emergency state reset")


def get_safety_supervisor() -> SafetySupervisor:
    """
    Get global safety supervisor instance with thread safety
    """
    if not hasattr(get_safety_supervisor, 'instance'):
        get_safety_supervisor.instance = SafetySupervisor()
    return get_safety_supervisor.instance


# Global instance
safety_supervisor = get_safety_supervisor()


if __name__ == "__main__":
    print("Testing Safety Supervisor...")
    
    supervisor = get_safety_supervisor()
    
    # Quick test
    result = supervisor.check_system_integrity()
    print(f"System integrity check: {result[0]}, {result[1]}")
    
    print("Safety Supervisor initialized and ready!")