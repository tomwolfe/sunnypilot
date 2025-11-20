"""
Safety Supervisor for Sunnypilot
Implements safety checks for engagement
"""
import time
from typing import Dict, Any

import cereal.messaging as messaging
from cereal import log
from openpilot.common.swaglog import cloudlog


class SafetySupervisor:
  """Safety supervisor for validating system readiness"""

  def __init__(self):
    self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'validationMetrics', 'pandaStates'])
    self.pm = messaging.PubMaster(['onroadEvents'])

    # Initialize safety state
    self.safety_engaged = False
    self.last_check_time = time.time()
    self.safety_threshold = 0.6

  def check_system_safety(self) -> Dict[str, Any]:
    """Perform safety checks on system state"""
    current_time = time.time()

    # Update all subscribers
    self.sm.update(0)

    safety_status = {
      'system_safe': True,
      'engagement_permitted': True,
      'reason': 'OK',
      'timestamp': current_time
    }

    # Check for panda health (critical safety component)
    if self.sm.updated['pandaStates']:
      for pandaState in self.sm['pandaStates']:
        if hasattr(pandaState, 'safetyRxInvalid') and pandaState.safetyRxInvalid > 50:
          safety_status['system_safe'] = False
          safety_status['engagement_permitted'] = False
          safety_status['reason'] = 'Panda safety errors detected'
          return safety_status

    # Check validation metrics if available
    if (self.sm.updated['validationMetrics'] and
        hasattr(self.sm['validationMetrics'], 'overallConfidence')):
      overall_conf = self.sm['validationMetrics'].overallConfidence
      is_valid = self.sm['validationMetrics'].isValid
      system_should_engage = self.sm['validationMetrics'].systemShouldEngage

      if overall_conf < self.safety_threshold:
        safety_status['system_safe'] = False
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = f'Low validation confidence: {overall_conf}'
      elif not is_valid:
        safety_status['system_safe'] = False
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = 'Validation metrics indicate system not valid'
      elif not system_should_engage:
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = 'Validation indicates system should not engage'

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
      time.sleep(0.05)  # 20Hz check rate
  except KeyboardInterrupt:
    cloudlog.info("Safety supervisor stopped")


if __name__ == "__main__":
  main()