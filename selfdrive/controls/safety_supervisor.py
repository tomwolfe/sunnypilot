"""
Simplified Safety Supervisor for Sunnypilot
Implements essential safety checks for engagement
"""
import time
from typing import Dict, Any

import cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from selfdrive.common.validation_config import get_validation_config


class SafetySupervisor:
  """Simplified safety supervisor for validating system readiness"""

  def __init__(self):
    self.sm = messaging.SubMaster(['carState', 'controlsState', 'validationMetrics', 'pandaStates', 'radarState'])
    self.pm = messaging.PubMaster(['onroadEvents'])

    # Get configuration
    self.config = get_validation_config()

    # Initialize safety state
    self.safety_engaged = False
    self.last_check_time = time.time()

  def check_system_safety(self) -> Dict[str, Any]:
    """Perform essential safety checks on system state"""
    current_time = time.time()

    # Update all subscribers
    self.sm.update(0)

    safety_status = {
      'system_safe': True,
      'engagement_permitted': True,
      'reason': 'OK',
      'timestamp': current_time,
    }

    # Check for panda health (critical safety component)
    if self.sm.updated['pandaStates']:
      for pandaState in self.sm['pandaStates']:
        if hasattr(pandaState, 'safetyRxInvalid') and pandaState.safetyRxInvalid > 0:
          safety_status['system_safe'] = False
          safety_status['engagement_permitted'] = False
          safety_status['reason'] = 'Panda safety errors detected'
          return safety_status
        elif (hasattr(pandaState, 'safetyTxBlocked') and pandaState.safetyTxBlocked > 0):
          safety_status['system_safe'] = False
          safety_status['engagement_permitted'] = False
          safety_status['reason'] = 'Panda safety TX blocked'
          return safety_status

    # Check car state safety
    if self.sm.updated['carState']:
      car_state = self.sm['carState']

      # Check for unsafe conditions
      if hasattr(car_state, 'steerError') and car_state.steerError:
        safety_status['system_safe'] = False
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = 'Steering error detected'
      elif hasattr(car_state, 'steerFault') and car_state.steerFault:
        safety_status['system_safe'] = False
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = 'Steering fault detected'
      elif hasattr(car_state, 'controlsAllowed') and not car_state.controlsAllowed:
        safety_status['system_safe'] = False
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = 'Controls not allowed by car safety'

    # Check validation metrics if available
    if (self.sm.updated['validationMetrics'] and
        hasattr(self.sm['validationMetrics'], 'isValid')):
      is_valid = self.sm['validationMetrics'].isValid
      system_should_engage = self.sm['validationMetrics'].systemShouldEngage

      if not is_valid or not system_should_engage:
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = 'Validation metrics indicate system should not engage'

    return safety_status

  def run_step(self) -> Dict[str, Any]:
    """Run a single safety check step"""
    safety_status = self.check_system_safety()
    self.last_check_time = time.time()

    if not safety_status['engagement_permitted']:
      cloudlog.warning(f"Engagement not permitted: {safety_status['reason']}")

    return safety_status


def main():
  """Main safety supervisor loop"""
  supervisor = SafetySupervisor()

  try:
    while True:
      status = supervisor.run_step()
      time.sleep(0.05)  # Run at 20Hz
  except KeyboardInterrupt:
    cloudlog.info("Safety supervisor stopped")


if __name__ == "__main__":
  main()