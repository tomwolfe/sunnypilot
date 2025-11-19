#!/usr/bin/env python3
"""
Simple System Health Monitoring for Sunnypilot
Essential system health monitoring with minimal complexity
"""
import time
import psutil
from typing import Dict, Any

from openpilot.common.swaglog import cloudlog


class SimpleSystemHealthMonitor:
  """Simple system health monitor with essential metrics"""
  
  def __init__(self):
    self.cpu_threshold = 90.0  # Percent
    self.memory_threshold = 90.0  # Percent
    self.disk_threshold = 95.0  # Percent
    
  def get_system_health(self) -> Dict[str, Any]:
    """Get basic system health metrics"""
    health_report = {
      'timestamp': time.time(),
      'cpu_percent': psutil.cpu_percent(interval=0.1),
      'memory_percent': psutil.virtual_memory().percent,
      'disk_percent': psutil.disk_usage('/').percent,
      'thermal_status': self._get_basic_thermal(),
      'status': 'OK'
    }
    
    # Check for issues
    issues = []
    if health_report['cpu_percent'] > self.cpu_threshold:
      issues.append(f"High CPU: {health_report['cpu_percent']:.1f}%")
    if health_report['memory_percent'] > self.memory_threshold:
      issues.append(f"High Memory: {health_report['memory_percent']:.1f}%")
    if health_report['disk_percent'] > self.disk_threshold:
      issues.append(f"High Disk: {health_report['disk_percent']:.1f}%")
    
    if issues:
      health_report['status'] = 'WARNING'
      health_report['issues'] = issues
    
    return health_report
  
  def _get_basic_thermal(self) -> int:
    """Get basic thermal status (simplified)"""
    try:
      # Try to get thermal information if available
      if hasattr(psutil, 'sensors_temperatures'):
        temps = psutil.sensors_temperatures()
        if temps:
          # Get max temperature across all sensors
          max_temp = 0.0
          for name, entries in temps.items():
            for entry in entries:
              max_temp = max(max_temp, entry.current)
          # Return thermal status based on temperature
          if max_temp > 80.0:
            return 3  # Hot
          elif max_temp > 70.0:
            return 2  # Warm
          else:
            return 1  # Normal
    except:
      pass
    
    # Default to normal if we can't get thermal data
    return 0  # Unknown
  

def monitor_system_health():
  """Monitor system health in a simple loop"""
  monitor = SimpleSystemHealthMonitor()
  
  try:
    while True:
      health = monitor.get_system_health()
      
      if health['status'] != 'OK':
        for issue in health.get('issues', []):
          cloudlog.warning(f"System health: {issue}")
      else:
        cloudlog.debug(f"System health OK - CPU: {health['cpu_percent']:.1f}%, "
                      f"Memory: {health['memory_percent']:.1f}%, "
                      f"Disk: {health['disk_percent']:.1f}%")
      
      time.sleep(5.0)  # Check every 5 seconds
  except KeyboardInterrupt:
    cloudlog.info("System health monitoring stopped")


if __name__ == "__main__":
  monitor_system_health()