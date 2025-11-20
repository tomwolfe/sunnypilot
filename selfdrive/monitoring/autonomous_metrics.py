"""
Autonomous Driving Metrics Collection Module
Collects and processes metrics for autonomous driving improvement
"""
import time
import numpy as np
from collections import deque, defaultdict
from typing import Dict, Any, List, Optional

import cereal.messaging as messaging
from cereal import log
from openpilot.common.swaglog import cloudlog
from openpilot.common.realtime import DT_MDL


class AutonomousMetricsCollector:
  """Collects and processes metrics for autonomous driving improvements"""
  
  def __init__(self):
    # Initialize tracking variables
    self.start_time = time.time()
    self.frame_count = 0
    
    # Performance metrics
    self.lateral_jerk_buffer = deque(maxlen=200)  # ~10 seconds at 20Hz
    self.longitudinal_jerk_buffer = deque(maxlen=200)
    self.steering_angles = deque(maxlen=200)
    self.accelerations = deque(maxlen=200)
    self.velocities = deque(maxlen=200)
    
    # System resource tracking
    self.cpu_usage_buffer = deque(maxlen=100)
    self.memory_usage_buffer = deque(maxlen=100)
    self.temperature_buffer = deque(maxlen=100)
    
    # Safety metrics
    self.fcw_events = 0
    self.steer_limited_count = 0
    self.longitudinal_limited_count = 0
    
    # Driving behavior metrics
    self.driver_interventions = 0
    self.mode_switches = 0
    
    # System state
    self.sm: Optional[messaging.SubMaster] = None
    self.initialized = False
    
    cloudlog.info("Autonomous Metrics Collector initialized")
  
  def initialize_submaster(self, sm: messaging.SubMaster):
    """Initialize with messaging SubMaster"""
    self.sm = sm
    self.initialized = True
  
  def collect_metrics(self) -> 'AutonomousMetricsCollector':
    """Collect current metrics from messaging system"""
    if not self.initialized or not self.sm:
      cloudlog.warning("Metrics collector not initialized with SubMaster")
      return self

    self.frame_count += 1

    try:
      # Update from carState
      if 'carState' in self.sm and self.sm.updated['carState']:
        car_state = self.sm['carState']

        # Collect steering and acceleration data
        if hasattr(car_state, 'steeringAngleDeg'):
          self.steering_angles.append(car_state.steeringAngleDeg)

        if hasattr(car_state, 'aEgo'):
          self.accelerations.append(car_state.aEgo)

        if hasattr(car_state, 'vEgo'):
          self.velocities.append(car_state.vEgo)

        # Check for driver interventions
        if hasattr(car_state, 'steeringPressed') and car_state.steeringPressed:
          self.driver_interventions += 1
    except Exception as e:
      cloudlog.error(f"Error collecting carState metrics: {e}")

    try:
      # Update from controlsState
      if 'controlsState' in self.sm and self.sm.updated['controlsState']:
        controls_state = self.sm['controlsState']

        # Collect jerk-related metrics
        if hasattr(controls_state, 'lateralJerk'):
          self.lateral_jerk_buffer.append(abs(controls_state.lateralJerk))

        if hasattr(controls_state, 'longitudinalJerk'):
          self.longitudinal_jerk_buffer.append(abs(controls_state.longitudinalJerk))

        # Track mode switches
        if hasattr(controls_state, 'experimentalMode') and controls_state.experimentalMode:
          self.mode_switches += 1
    except Exception as e:
      cloudlog.error(f"Error collecting controlsState metrics: {e}")

    try:
      # Update from deviceState for system metrics
      if 'deviceState' in self.sm and self.sm.updated['deviceState']:
        device_state = self.sm['deviceState']

        if hasattr(device_state, 'cpuUsagePercent') and device_state.cpuUsagePercent:
          try:
            cpu_avg = sum(device_state.cpuUsagePercent) / len(device_state.cpuUsagePercent)
            self.cpu_usage_buffer.append(cpu_avg)
          except (TypeError, ZeroDivisionError):
            cloudlog.error("Error calculating CPU usage average")

        if hasattr(device_state, 'memoryUsagePercent'):
          self.memory_usage_buffer.append(device_state.memoryUsagePercent)

        if hasattr(device_state, 'cpuTempC') and device_state.cpuTempC:
          try:
            temp_avg = sum(device_state.cpuTempC) / len(device_state.cpuTempC)
            self.temperature_buffer.append(temp_avg)
          except (TypeError, ZeroDivisionError):
            cloudlog.error("Error calculating temperature average")
    except Exception as e:
      cloudlog.error(f"Error collecting deviceState metrics: {e}")

    try:
      # Update from radarState for FCW events
      if 'radarState' in self.sm and self.sm.updated['radarState']:
        radar_state = self.sm['radarState']

        if hasattr(radar_state, 'fcw') and radar_state.fcw:
          self.fcw_events += 1
    except Exception as e:
      cloudlog.error(f"Error collecting radarState metrics: {e}")

    return self
  
  def get_performance_report(self) -> Dict[str, float]:
    """Get a comprehensive performance report"""
    current_time = time.time()
    duration = current_time - self.start_time
    
    report = {
      'duration': duration,
      'frame_count': self.frame_count,
      'avg_frame_rate': self.frame_count / duration if duration > 0 else 0,
    }
    
    # Add lateral jerk metrics
    if self.lateral_jerk_buffer:
      report['avg_lateral_jerk'] = float(np.mean(self.lateral_jerk_buffer))
      report['max_lateral_jerk'] = float(np.max(self.lateral_jerk_buffer))
      report['std_lateral_jerk'] = float(np.std(self.lateral_jerk_buffer))
    
    # Add longitudinal jerk metrics
    if self.longitudinal_jerk_buffer:
      report['avg_longitudinal_jerk'] = float(np.mean(self.longitudinal_jerk_buffer))
      report['max_longitudinal_jerk'] = float(np.max(self.longitudinal_jerk_buffer))
      report['std_longitudinal_jerk'] = float(np.std(self.longitudinal_jerk_buffer))
    
    # Add system resource metrics
    if self.cpu_usage_buffer:
      report['avg_cpu_util'] = float(np.mean(self.cpu_usage_buffer))
      report['max_cpu_util'] = float(np.max(self.cpu_usage_buffer))
    
    if self.memory_usage_buffer:
      report['avg_memory_util'] = float(np.mean(self.memory_usage_buffer))
    
    if self.temperature_buffer:
      report['avg_temperature'] = float(np.mean(self.temperature_buffer))
      report['max_temperature'] = float(np.max(self.temperature_buffer))
    
    # Add safety metrics
    report['fcw_events'] = self.fcw_events
    report['driver_interventions'] = self.driver_interventions
    report['steer_limited_count'] = self.steer_limited_count
    report['long_limited_count'] = self.longitudinal_limited_count
    report['mode_switches'] = self.mode_switches
    
    return report
  
  def get_system_health(self) -> Dict[str, Any]:
    """Get system health assessment"""
    health = {
      'status': 'healthy',  # Default to healthy
      'issues': [],
      'longitudinal_smoothness': 1.0,  # 1.0 = perfectly smooth
      'lateral_smoothness': 1.0,      # 1.0 = perfectly smooth
      'system_responsiveness': 1.0    # 1.0 = fully responsive
    }
    
    # Get performance metrics
    perf_report = self.get_performance_report()
    
    # Assess longitudinal smoothness based on jerk
    avg_long_jerk = perf_report.get('avg_longitudinal_jerk', 0.0)
    if avg_long_jerk > 2.0:
      health['longitudinal_smoothness'] = 0.3
      health['status'] = 'concerning'
      health['issues'].append('High longitudinal jerk detected')
    elif avg_long_jerk > 1.0:
      health['longitudinal_smoothness'] = 0.6
      if health['status'] == 'healthy':
        health['status'] = 'caution'
    else:
      health['longitudinal_smoothness'] = 0.9
    
    # Assess lateral smoothness based on jerk
    avg_lat_jerk = perf_report.get('avg_lateral_jerk', 0.0)
    if avg_lat_jerk > 2.5:
      health['lateral_smoothness'] = 0.2
      health['status'] = 'concerning'
      health['issues'].append('High lateral jerk detected')
    elif avg_lat_jerk > 1.5:
      health['lateral_smoothness'] = 0.5
      if health['status'] == 'healthy':
        health['status'] = 'caution'
    else:
      health['lateral_smoothness'] = 0.9
    
    # Check system resources
    avg_cpu = perf_report.get('avg_cpu_util', 0.0)
    if avg_cpu > 85.0:
      health['system_responsiveness'] = 0.4
      health['status'] = 'concerning'
      health['issues'].append('High CPU utilization')
    elif avg_cpu > 75.0:
      health['system_responsiveness'] = 0.7
      if health['status'] == 'healthy':
        health['status'] = 'caution'
    
    # Check for safety issues
    if perf_report.get('fcw_events', 0) > 0:
      health['status'] = 'caution' if health['status'] == 'healthy' else health['status']
      health['issues'].append(f"{perf_report['fcw_events']} FCW events detected")
    
    if perf_report.get('driver_interventions', 0) > 5:
      health['status'] = 'caution' if health['status'] == 'healthy' else health['status']
      health['issues'].append(f"{perf_report['driver_interventions']} driver interventions detected")
    
    return health


# Global instance
_metrics_collector = AutonomousMetricsCollector()


def get_metrics_collector() -> AutonomousMetricsCollector:
  """Get the global metrics collector instance"""
  return _metrics_collector