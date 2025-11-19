"""
Simple Safety Supervisor for Sunnypilot
Basic safety supervision with essential checks only
"""
import time
from typing import Dict, Any, List

import cereal.messaging as messaging
from cereal import log
from openpilot.common.swaglog import cloudlog


class SimpleSafetySupervisor:
  """Simple safety supervisor with essential functionality"""
  
  def __init__(self):
    self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'validationMetrics'])
    self.pm = messaging.PubMaster(['safetyEvent'])
    
    self.last_check_time = time.time()
    self.safety_threshold = 0.6
    self.engagement_permitted = True
    
    cloudlog.info("Simple safety supervisor initialized")
  
  def check_system_safety(self) -> Dict[str, Any]:
    """Perform basic safety checks"""
    current_time = time.time()
    
    # Update data
    self.sm.update(0)
    
    # Basic safety checks
    safety_status = {
      'system_safe': True,
      'engagement_permitted': True,
      'reason': 'OK',
      'timestamp': current_time
    }
    
    # Check validation metrics if available
    if (self.sm.updated['validationMetrics'] and 
        hasattr(self.sm['validationMetrics'], 'overallConfidence')):
      overall_conf = self.sm['validationMetrics'].overallConfidence
      is_valid = self.sm['validationMetrics'].isValid
      system_should_engage = self.sm['validationMetrics'].systemShouldEngage
      
      if overall_conf < self.safety_threshold or not is_valid or not system_should_engage:
        safety_status['system_safe'] = False
        safety_status['engagement_permitted'] = False
        safety_status['reason'] = f'Safety validation failed: conf={overall_conf}, valid={is_valid}, engage={system_should_engage}'
    
    # Additional basic checks could go here
    # For now, we keep it simple
    
    return safety_status
  
  def publish_safety_status(self, status: Dict[str, Any]):
    """Publish safety status if needed"""
    # This would publish safety events in a real implementation
    pass
  
  def run_step(self) -> Dict[str, Any]:
    """Run a single step of safety supervision"""
    safety_status = self.check_system_safety()
    self.last_check_time = time.time()
    
    # In a real system, we might publish safety data
    if not safety_status['system_safe']:
      cloudlog.warning(f"Safety supervisor: {safety_status['reason']}")
    
    return safety_status


def main():
  """Main safety supervisor loop"""
  supervisor = SimpleSafetySupervisor()
  
  try:
    while True:
      status = supervisor.run_step()
      time.sleep(0.1)  # 10Hz check rate
  except KeyboardInterrupt:
    cloudlog.info("Safety supervisor stopped by user")


if __name__ == "__main__":
  main()