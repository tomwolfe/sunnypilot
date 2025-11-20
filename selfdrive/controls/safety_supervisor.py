"""
Safety Supervisor for Sunnypilot
Implements essential safety checks for engagement integrated with openpilot framework
"""
import time
from typing import Dict, Any

import cereal.messaging as messaging
from cereal import log
from openpilot.common.swaglog import cloudlog
from selfdrive.common.validation_config import get_validation_config


class SafetySupervisor:
  """Safety supervisor for validating system readiness before engagement"""

  def __init__(self):
    self.sm = messaging.SubMaster(['carState', 'controlsState', 'validationMetrics', 'pandaStates', 'radarState',
                                   'liveCalibration', 'modelV2', 'deviceState'])
    self.pm = messaging.PubMaster(['onroadEvents', 'safetyIssue'])

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

    # Check system-wide safety conditions
    if self.sm.updated['deviceState']:
      device_state = self.sm['deviceState']
      if hasattr(device_state, 'thermalStatus') and device_state.thermalStatus >= log.DeviceState.ThermalStatus.red:
        safety_status['system_safe'] = False
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = 'Device thermal status critical'
        return safety_status

    # Check calibration status
    if self.sm.updated['liveCalibration']:
      cal_status = self.sm['liveCalibration']
      if not len(cal_status.rpyCalib) >= 3:  # Ensure proper calibration
        safety_status['system_safe'] = False
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = 'Insufficient calibration data'
        return safety_status

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
      elif hasattr(car_state, 'vehicleMoving') and not car_state.vehicleMoving:
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = 'Vehicle not in motion (required for engagement)'

      # Check speed constraints
      if hasattr(car_state, 'vEgo') and car_state.vEgo < 1.0:  # 1 m/s minimum
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = 'Vehicle speed below minimum for engagement'

    # Check validation metrics if available
    if (self.sm.updated['validationMetrics'] and
        hasattr(self.sm['validationMetrics'], 'isValid')):
      is_valid = self.sm['validationMetrics'].isValid
      system_should_engage = self.sm['validationMetrics'].systemShouldEngage

      if not is_valid or not system_should_engage:
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = 'Validation metrics indicate system should not engage'

    return safety_status

  def publish_safety_issue(self, safety_status: Dict[str, Any]):
    """Publish safety issues when detected"""
    if not safety_status['engagement_permitted'] and safety_status['reason'] != 'OK':
      dat = messaging.new_message('safetyIssue')
      safety_issue = dat.safetyIssue
      safety_issue.text = safety_status['reason']
      safety_issue.timestamp = int(safety_status['timestamp'] * 1e9)  # Convert to nanoseconds
      # Set severity based on the type of issue
      if 'critical' in safety_status['reason'].lower() or 'thermal' in safety_status['reason'].lower() or 'error' in safety_status['reason'].lower():
        safety_issue.severity = log.SafetyIssue.SafetyIssueSeverity.critical
      elif 'calibration' in safety_status['reason'].lower() or 'minimum' in safety_status['reason'].lower():
        safety_issue.severity = log.SafetyIssue.SafetyIssueSeverity.warning
      else:
        safety_issue.severity = log.SafetyIssue.SafetyIssueSeverity.warning
      safety_issue.engaged = self.safety_engaged
      self.pm.send('safetyIssue', dat)

  def run_step(self) -> Dict[str, Any]:
    """Run a single safety check step"""
    safety_status = self.check_system_safety()
    self.last_check_time = time.time()

    # Publish safety issues if any
    self.publish_safety_issue(safety_status)

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