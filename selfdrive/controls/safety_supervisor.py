"""
Enhanced Safety Supervisor for Sunnypilot
Implements comprehensive safety checks for engagement with robust validation
"""
import time
from typing import Dict, Any

import cereal.messaging as messaging
from cereal import log
from openpilot.common.swaglog import cloudlog


class SafetySupervisor:
  """Enhanced safety supervisor for validating system readiness"""

  def __init__(self):
    self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'validationMetrics', 'pandaStates',
                                  'liveCalibration', 'radarState', 'gpsLocation'])
    self.pm = messaging.PubMaster(['onroadEvents'])

    # Initialize safety state
    self.safety_engaged = False
    self.last_check_time = time.time()
    self.safety_threshold = 0.7  # Increased threshold for safety
    self.min_engagement_velocity = 0.1  # Minimum speed for engagement
    self.max_engagement_velocity = 60.0  # Maximum speed for engagement (about 134 mph)

    # Safety state history for temporal consistency
    self.safety_history = []
    self.max_history = 10

  def check_system_safety(self) -> Dict[str, Any]:
    """Perform comprehensive safety checks on system state"""
    current_time = time.time()

    # Update all subscribers
    self.sm.update(0)

    safety_status = {
      'system_safe': True,
      'engagement_permitted': True,
      'reason': 'OK',
      'timestamp': current_time,
      'detailed_checks': {}
    }

    # Check for panda health (critical safety component)
    if self.sm.updated['pandaStates']:
      for pandaState in self.sm['pandaStates']:
        if hasattr(pandaState, 'safetyRxInvalid') and pandaState.safetyRxInvalid > 20:  # Lowered threshold
          safety_status['system_safe'] = False
          safety_status['engagement_permitted'] = False
          safety_status['reason'] = 'Panda safety errors detected'
          safety_status['detailed_checks']['panda_health'] = False
          return safety_status
        elif (hasattr(pandaState, 'safetyTxBlocked') and pandaState.safetyTxBlocked > 0):
          safety_status['system_safe'] = False
          safety_status['engagement_permitted'] = False
          safety_status['reason'] = 'Panda safety TX blocked'
          safety_status['detailed_checks']['panda_tx_blocked'] = False
          return safety_status

    # Check car state safety first
    if self.sm.updated['carState']:
      car_state = self.sm['carState']

      # Check for unsafe conditions
      if hasattr(car_state, 'steerError') and car_state.steerError:
        safety_status['system_safe'] = False
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = 'Steering error detected'
        safety_status['detailed_checks']['steer_error'] = False
      elif hasattr(car_state, 'steerFault') and car_state.steerFault:
        safety_status['system_safe'] = False
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = 'Steering fault detected'
        safety_status['detailed_checks']['steer_fault'] = False
      elif hasattr(car_state, 'controlsAllowed') and not car_state.controlsAllowed:
        safety_status['system_safe'] = False
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = 'Controls not allowed by car safety'
        safety_status['detailed_checks']['controls_allowed'] = False
      elif hasattr(car_state, 'vEgo') and (car_state.vEgo < self.min_engagement_velocity or
                                           car_state.vEgo > self.max_engagement_velocity):
        if safety_status['engagement_permitted']:  # Only update reason if not already set
          safety_status['engagement_permitted'] = False
          safety_status['reason'] = f'Velocity out of safe range: {car_state.vEgo:.2f} m/s'
          safety_status['detailed_checks']['velocity_check'] = False

    # Only proceed with validation checks if system is still considered safe
    if safety_status['system_safe']:
      # Check validation metrics if available
      if (self.sm.updated['validationMetrics'] and
          hasattr(self.sm['validationMetrics'], 'overallConfidence')):
        overall_conf = self.sm['validationMetrics'].overallConfidence
        is_valid = self.sm['validationMetrics'].isValid
        system_should_engage = self.sm['validationMetrics'].systemShouldEngage
        safety_score = self.sm['validationMetrics'].safetyScore

        if overall_conf < self.safety_threshold:
          safety_status['system_safe'] = False
          safety_status['engagement_permitted'] = False
          safety_status['reason'] = f'Low validation confidence: {overall_conf:.2f} (threshold: {self.safety_threshold})'
          safety_status['detailed_checks']['validation_confidence'] = False
        elif not is_valid:
          safety_status['system_safe'] = False
          safety_status['engagement_permitted'] = False
          safety_status['reason'] = 'Validation metrics indicate system not valid'
          safety_status['detailed_checks']['validation_valid'] = False
        elif not system_should_engage:
          safety_status['engagement_permitted'] = False
          safety_status['reason'] = 'Validation indicates system should not engage'
          safety_status['detailed_checks']['should_engage'] = False
        elif safety_score < 0.6:  # Additional safety check
          safety_status['system_safe'] = False
          safety_status['engagement_permitted'] = False
          safety_status['reason'] = f'Low safety score: {safety_score:.2f}'
          safety_status['detailed_checks']['safety_score'] = False

    # Check radar state for obstacle detection
    if self.sm.updated['radarState'] and hasattr(self.sm['radarState'], 'leadOne'):
      lead_one = self.sm['radarState'].leadOne
      if lead_one:
        if hasattr(lead_one, 'status') and not lead_one.status:  # Lead not valid
          # Check if lead is very close and slow (potential collision risk)
          if (hasattr(lead_one, 'dRel') and lead_one.dRel < 30.0 and  # Within 30 meters
              hasattr(lead_one, 'vRel') and lead_one.vRel < -5.0):     # Approaching rapidly
            safety_status['system_safe'] = False
            safety_status['engagement_permitted'] = False
            safety_status['reason'] = f'Closely approaching lead vehicle detected: {lead_one.dRel:.1f}m, {lead_one.vRel:.1f}m/s'
            safety_status['detailed_checks']['lead_approach'] = False

    # Check temporal consistency of safety decisions
    self.safety_history.append(safety_status.copy())
    if len(self.safety_history) > self.max_history:
      self.safety_history = self.safety_history[-self.max_history:]

    # If we have a history of unsafe states, continue to be conservative
    unsafe_count = sum(1 for s in self.safety_history if not s['system_safe'])
    if unsafe_count > len(self.safety_history) * 0.5:  # More than 50% of recent checks were unsafe
      safety_status['engagement_permitted'] = False
      safety_status['reason'] = f'Too many recent safety violations: {unsafe_count}/{len(self.safety_history)}'
      safety_status['detailed_checks']['temporal_consistency'] = False

    return safety_status

  def run_step(self) -> Dict[str, Any]:
    """Run a single safety check step with enhanced validation"""
    safety_status = self.check_system_safety()
    self.last_check_time = time.time()

    if not safety_status['engagement_permitted']:
      # Log detailed information when engagement is not permitted
      detailed_info = safety_status.get('detailed_checks', {})
      cloudlog.warning(f"Engagement not permitted: {safety_status['reason']}, details: {detailed_info}")

    return safety_status


def main():
  """Main safety supervisor loop"""
  supervisor = SafetySupervisor()

  try:
    while True:
      status = supervisor.run_step()
      time.sleep(0.05)  # 20Hz check rate
  except KeyboardInterrupt:
    cloudlog.info("Safety supervisor stopped")


if __name__ == "__main__":
  main()