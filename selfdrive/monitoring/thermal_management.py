"""
Thermal Management System for sunnypilot
Integrates thermal sensors to dynamically throttle or scale system performance based on temperature thresholds
"""
import time
import threading
from enum import IntEnum
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass
import numpy as np

import cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import HARDWARE, THERMAL_STATUS
from openpilot.common.params import Params
from openpilot.common.dynamic_adaptation import dynamic_adaptation, PerformanceMode


class ThermalZone(IntEnum):
  """Thermal zones for different components"""
  CPU = 0
  GPU = 1
  MEMORY = 2
  PMIC = 3
  MODEM = 4
  CASE = 5
  UNKNOWN = 99


@dataclass
class ThermalReading:
  """Thermal sensor reading"""
  zone: ThermalZone
  temperature: float  # in Celsius
  timestamp: float
  critical: bool = False


@dataclass
class ThermalPolicy:
  """Thermal management policy configuration"""
  # Temperature thresholds (in Celsius)
  cpu_normal_max: float = 65.0
  cpu_caution_max: float = 75.0
  cpu_warning_max: float = 80.0
  cpu_throttle_start: float = 85.0
  cpu_critical: float = 95.0
  
  gpu_normal_max: float = 60.0
  gpu_caution_max: float = 70.0
  gpu_warning_max: float = 75.0
  gpu_throttle_start: float = 80.0
  gpu_critical: float = 85.0
  
  memory_normal_max: float = 55.0
  memory_caution_max: float = 65.0
  memory_warning_max: float = 70.0
  memory_throttle_start: float = 75.0
  memory_critical: float = 80.0
  
  # Performance scaling factors for different thermal levels
  normal_performance: float = 1.0      # 100% performance
  caution_performance: float = 0.9     # 90% performance
  warning_performance: float = 0.7     # 70% performance
  throttle_performance: float = 0.5    # 50% performance
  critical_performance: float = 0.3    # 30% performance
  
  # Fan control thresholds (if applicable)
  fan_start_temp: float = 60.0
  fan_full_speed_temp: float = 75.0
  
  # Hysteresis to prevent rapid switching (in Celsius)
  hysteresis: float = 2.0


class ThermalManager:
  """Main thermal management class"""
  
  def __init__(self, policy: Optional[ThermalPolicy] = None):
    self.policy = policy or ThermalPolicy()
    self.current_temperatures: Dict[ThermalZone, float] = {}
    self.temperature_history: Dict[ThermalZone, List[ThermalReading]] = {}
    for zone in ThermalZone:
      self.temperature_history[zone] = []
    
    self.current_performance_scale = 1.0
    self.thermal_status = THERMAL_STATUS.green  # Start in green state
    self.fan_speed_target = 0  # 0-100%
    
    # Callbacks for thermal events
    self.thermal_callbacks: List[Callable[[ThermalZone, float, str], None]] = []
    self.performance_callbacks: List[Callable[[float], None]] = []
    
    # SubMaster for device state
    self.sm = messaging.SubMaster(['deviceState'])
    
    # Params for settings
    self.params = Params()
    
    # Threading
    self.running = True
    self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
    self.monitor_thread.start()
    
    # Performance adaptation integration
    self._register_with_performance_adaptation()
    
    cloudlog.info("Thermal management system initialized")
  
  def _register_with_performance_adaptation(self):
    """Register with the global performance adaptation system"""
    def thermal_mode_callback(old_mode: PerformanceMode, new_mode: PerformanceMode):
      self._on_performance_mode_change(old_mode, new_mode)
    
    dynamic_adaptation.add_mode_change_callback(thermal_mode_callback)
  
  def _on_performance_mode_change(self, old_mode: PerformanceMode, new_mode: PerformanceMode):
    """Callback for when performance mode changes"""
    # Adjust thermal thresholds or behavior based on performance mode
    # Create a temporary copy to avoid race conditions
    import copy
    if new_mode == PerformanceMode.MAX_PERFORMANCE:
      # In max performance mode, we might allow slightly higher temperatures
      # Use temporary values to avoid changing the base policy permanently
      pass  # For now, we'll use original thresholds for stability
    else:
      # For other modes, use standard thresholds - no changes needed
      pass
  
  def add_thermal_callback(self, callback: Callable[[ThermalZone, float, str], None]):
    """Add callback for thermal events"""
    self.thermal_callbacks.append(callback)
  
  def add_performance_callback(self, callback: Callable[[float], None]):
    """Add callback for performance scaling events"""
    self.performance_callbacks.append(callback)
  
  def get_current_temperatures(self) -> Dict[ThermalZone, float]:
    """Get current temperatures for all zones"""
    return self.current_temperatures.copy()
  
  def get_zone_temperature(self, zone: ThermalZone) -> Optional[float]:
    """Get temperature for a specific zone"""
    return self.current_temperatures.get(zone)
  
  def get_current_performance_scale(self) -> float:
    """Get the current performance scaling factor"""
    return self.current_performance_scale
  
  def _monitor_loop(self):
    """Main monitoring loop"""
    while self.running:
      try:
        # Update device state
        self.sm.update(0)
        
        # Read current temperatures
        self._read_temperatures()
        
        # Update thermal status and performance scaling
        self._update_thermal_status()
        
        # Update fan speed if needed
        self._update_fan_control()
        
        # Sleep for a reasonable interval
        time.sleep(1.0)  # Update every second
        
      except Exception as e:
        cloudlog.error(f"Thermal monitoring error: {e}")
        time.sleep(1.0)
  
  def _read_temperatures(self):
    """Read current temperatures from hardware"""
    if self.sm.valid['deviceState']:
      device_state = self.sm['deviceState']
      
      # Update CPU temperatures
      if device_state.cpuTempC:
        max_cpu_temp = max(device_state.cpuTempC) if device_state.cpuTempC else 0.0
        self.current_temperatures[ThermalZone.CPU] = max_cpu_temp
        
        # Add to history
        reading = ThermalReading(ThermalZone.CPU, max_cpu_temp, time.time(),
                               max_cpu_temp >= self.policy.cpu_critical)
        self.temperature_history[ThermalZone.CPU].append(reading)
        if len(self.temperature_history[ThermalZone.CPU]) > 100:  # Keep last 100 readings
          self.temperature_history[ThermalZone.CPU].pop(0)
      
      # Update GPU temperatures
      if device_state.gpuTempC:
        max_gpu_temp = max(device_state.gpuTempC) if device_state.gpuTempC else 0.0
        self.current_temperatures[ThermalZone.GPU] = max_gpu_temp
        
        reading = ThermalReading(ThermalZone.GPU, max_gpu_temp, time.time(),
                               max_gpu_temp >= self.policy.gpu_critical)
        self.temperature_history[ThermalZone.GPU].append(reading)
        if len(self.temperature_history[ThermalZone.GPU]) > 100:
          self.temperature_history[ThermalZone.GPU].pop(0)
      
      # Update memory temperature
      self.current_temperatures[ThermalZone.MEMORY] = device_state.memoryTempC
      reading = ThermalReading(ThermalZone.MEMORY, device_state.memoryTempC, time.time(),
                             device_state.memoryTempC >= self.policy.memory_critical)
      self.temperature_history[ThermalZone.MEMORY].append(reading)
      if len(self.temperature_history[ThermalZone.MEMORY]) > 100:
        self.temperature_history[ThermalZone.MEMORY].pop(0)
      
      # Update PMIC temperatures if available
      if device_state.pmicTempC:
        max_pmic_temp = max(device_state.pmicTempC) if device_state.pmicTempC else 0.0
        self.current_temperatures[ThermalZone.PMIC] = max_pmic_temp
        
        reading = ThermalReading(ThermalZone.PMIC, max_pmic_temp, time.time(),
                               max_pmic_temp >= 70.0)  # Assume 70C is critical for PMIC
        self.temperature_history[ThermalZone.PMIC].append(reading)
        if len(self.temperature_history[ThermalZone.PMIC]) > 100:
          self.temperature_history[ThermalZone.PMIC].pop(0)
  
  def _update_thermal_status(self):
    """Update thermal status and performance scaling based on temperatures"""
    # Determine the highest thermal level across all zones
    max_zone = ThermalZone.UNKNOWN
    max_temp = 0.0
    max_critical_temp = 0.0
    
    for zone, temp in self.current_temperatures.items():
      if zone == ThermalZone.CPU:
        critical_temp = self.policy.cpu_critical
        throttle_temp = self.policy.cpu_throttle_start
        warning_temp = self.policy.cpu_warning_max
        caution_temp = self.policy.cpu_caution_max
      elif zone == ThermalZone.GPU:
        critical_temp = self.policy.gpu_critical
        throttle_temp = self.policy.gpu_throttle_start
        warning_temp = self.policy.gpu_warning_max
        caution_temp = self.policy.gpu_caution_max
      elif zone == ThermalZone.MEMORY:
        critical_temp = self.policy.memory_critical
        throttle_temp = self.policy.memory_throttle_start
        warning_temp = self.policy.memory_warning_max
        caution_temp = self.policy.memory_caution_max
      else:
        continue  # Skip other zones for now
      
      if temp > max_temp:
        max_temp = temp
        max_zone = zone
        max_critical_temp = critical_temp
      
      # Check for thermal events and trigger callbacks
      if temp >= critical_temp:
        self._trigger_thermal_event(zone, temp, 'critical')
      elif temp >= throttle_temp:
        self._trigger_thermal_event(zone, temp, 'throttle')
      elif temp >= warning_temp:
        self._trigger_thermal_event(zone, temp, 'warning')
      elif temp >= caution_temp:
        self._trigger_thermal_event(zone, temp, 'caution')
    
    # Determine new thermal status and performance scale
    new_thermal_status = self.thermal_status
    new_performance_scale = self.current_performance_scale
    
    # Update thermal status based on max temperature
    if max_temp >= self.policy.cpu_critical or max_temp >= self.policy.gpu_critical or max_temp >= self.policy.memory_critical:
      new_thermal_status = THERMAL_STATUS.danger
      new_performance_scale = self.policy.critical_performance
    elif max_temp >= self.policy.cpu_throttle_start or max_temp >= self.policy.gpu_throttle_start or max_temp >= self.policy.memory_throttle_start:
      new_thermal_status = THERMAL_STATUS.red
      new_performance_scale = self.policy.throttle_performance
    elif max_temp >= self.policy.cpu_warning_max or max_temp >= self.policy.gpu_warning_max or max_temp >= self.policy.memory_warning_max:
      new_thermal_status = THERMAL_STATUS.red
      new_performance_scale = self.policy.warning_performance
    elif max_temp >= self.policy.cpu_caution_max or max_temp >= self.policy.gpu_caution_max or max_temp >= self.policy.memory_caution_max:
      new_thermal_status = THERMAL_STATUS.yellow
      new_performance_scale = self.policy.caution_performance
    else:
      new_thermal_status = THERMAL_STATUS.green
      new_performance_scale = self.policy.normal_performance
    
    # Apply hysteresis to prevent rapid switching
    if new_thermal_status != self.thermal_status:
      temp_diff = abs(max_temp - (self._get_threshold_for_status(self.thermal_status) - self.policy.hysteresis))
      
      # Only change status if temperature is sufficiently different
      if temp_diff > self.policy.hysteresis or new_thermal_status.value > self.thermal_status.value:
        old_status = self.thermal_status
        
        self.thermal_status = new_thermal_status
        old_scale = self.current_performance_scale
        self.current_performance_scale = new_performance_scale
        
        # Log thermal state change
        cloudlog.info(f"Thermal status changed: {old_status.name} -> {new_thermal_status.name}, "
                     f"Performance scale: {old_scale:.2f} -> {new_performance_scale:.2f}")
        
        # Trigger performance scaling callbacks
        for callback in self.performance_callbacks:
          try:
            callback(new_performance_scale)
          except (TypeError, ValueError) as e:
            cloudlog.error(f"Performance callback type/value error: {e}")
          except Exception as e:
            cloudlog.error(f"Performance callback unexpected error: {e}")
  
  def _get_threshold_for_status(self, status):
    """Get the temperature threshold for a given status"""
    if status == THERMAL_STATUS.danger:
      return self.policy.cpu_critical
    elif status == THERMAL_STATUS.red:
      return self.policy.cpu_throttle_start
    elif status == THERMAL_STATUS.yellow:
      return self.policy.cpu_caution_max
    else:  # green
      return self.policy.cpu_normal_max
  
  def _trigger_thermal_event(self, zone: ThermalZone, temp: float, level: str):
    """Trigger thermal event callbacks"""
    for callback in self.thermal_callbacks:
      try:
        callback(zone, temp, level)
      except (TypeError, ValueError) as e:
        cloudlog.error(f"Thermal callback type/value error: {e}")
      except Exception as e:
        cloudlog.error(f"Thermal callback unexpected error: {e}")
  
  def _update_fan_control(self):
    """Update fan control based on temperatures"""
    # Get max temperature across critical zones
    max_temp = 0.0
    for zone in [ThermalZone.CPU, ThermalZone.GPU, ThermalZone.MEMORY]:
      temp = self.current_temperatures.get(zone, 0.0)
      if temp > max_temp:
        max_temp = temp
    
    # Calculate target fan speed
    if max_temp >= self.policy.fan_full_speed_temp:
      target_fan_speed = 100  # Full speed
    elif max_temp >= self.policy.fan_start_temp:
      # Linear interpolation between start temp and full speed temp
      temp_range = self.policy.fan_full_speed_temp - self.policy.fan_start_temp
      if temp_range > 0:
        target_fan_speed = int(100 * (max_temp - self.policy.fan_start_temp) / temp_range)
      else:
        target_fan_speed = 50  # Default to 50% if range is invalid
    else:
      target_fan_speed = 0  # No fan needed
    
    # Only update if significantly different to avoid unnecessary changes
    if abs(target_fan_speed - self.fan_speed_target) > 5:
      self.fan_speed_target = target_fan_speed
      
      # Apply fan speed through hardware interface
      try:
        # In real implementation, this would call the appropriate hardware function
        # For now, we'll just log the change
        cloudlog.debug(f"Target fan speed updated to {target_fan_speed}%")

        # Update device state with desired fan speed
        if self.sm.valid['deviceState']:
          # In real implementation, we would set the desired fan speed
          pass
      except (TypeError, ValueError) as e:
        cloudlog.error(f"Fan control type/value error: {e}")
      except Exception as e:
        cloudlog.error(f"Failed to update fan speed: {e}")
  
  def get_thermal_recommendation(self) -> Dict[str, any]:
    """Get thermal-based recommendations for system behavior"""
    max_temp = max(self.current_temperatures.values()) if self.current_temperatures else 0
    
    recommendation = {
      'performance_scale': self.current_performance_scale,
      'thermal_status': self.thermal_status.name.lower(),
      'max_temperature_zone': max(self.current_temperatures.keys(), 
                               key=lambda k: self.current_temperatures[k]) if self.current_temperatures else ThermalZone.UNKNOWN,
      'max_temperature': max_temp,
      'action_needed': self.thermal_status in [THERMAL_STATUS.red, THERMAL_STATUS.danger],
      'reduction_needed': self.current_performance_scale < 1.0
    }
    
    return recommendation
  
  def stop(self):
    """Stop the thermal management system"""
    self.running = False


class ResourceAwareProcessor:
  """Resource-aware processing based on thermal conditions"""
  
  def __init__(self, thermal_manager: ThermalManager):
    self.thermal_manager = thermal_manager
    self.processing_queue = []
    self.max_queue_size = 100
  
  def submit_for_processing(self, task_func, *args, **kwargs):
    """Submit a task for processing with thermal awareness"""
    # Check if we should process immediately or queue based on thermal conditions
    if self._should_delay_processing():
      # Add to queue for later processing when conditions improve
      if len(self.processing_queue) < self.max_queue_size:
        self.processing_queue.append((task_func, args, kwargs, time.time()))
        return None  # Task queued, no immediate result
      else:
        # Queue is full, drop the oldest task and add this one
        self.processing_queue.pop(0)
        self.processing_queue.append((task_func, args, kwargs, time.time()))
        return None
    else:
      # Process immediately
      return self._execute_task(task_func, *args, **kwargs)
  
  def _should_delay_processing(self) -> bool:
    """Check if processing should be delayed based on thermal conditions"""
    thermal_rec = self.thermal_manager.get_thermal_recommendation()
    return thermal_rec['action_needed'] or thermal_rec['reduction_needed']
  
  def _execute_task(self, task_func, *args, **kwargs):
    """Execute a task with thermal monitoring"""
    import time
    
    start_temp = self.thermal_manager.get_zone_temperature(ThermalZone.CPU) or 0
    
    try:
      result = task_func(*args, **kwargs)
    except Exception as e:
      cloudlog.error(f"Task execution failed: {e}")
      result = None
    
    end_temp = self.thermal_manager.get_zone_temperature(ThermalZone.CPU) or 0
    
    # Log if task caused significant temperature increase
    if end_temp - start_temp > 5:  # More than 5C increase
      cloudlog.warning(f"Task caused temperature increase of {end_temp - start_temp:.1f}C")
    
    return result
  
  def process_queued_tasks(self):
    """Process queued tasks when thermal conditions improve"""
    # Process queued tasks if thermal conditions allow
    thermal_rec = self.thermal_manager.get_thermal_recommendation()
    
    if not thermal_rec['action_needed'] and not thermal_rec['reduction_needed']:
      # Process some queued tasks
      tasks_to_process = min(3, len(self.processing_queue))  # Process up to 3 at a time
      
      for _ in range(tasks_to_process):
        if self.processing_queue:
          task_func, args, kwargs, submit_time = self.processing_queue.pop(0)
          
          # Check again if conditions are still good
          if not self._should_delay_processing():
            self._execute_task(task_func, *args, **kwargs)
          else:
            # Put task back if conditions worsened
            self.processing_queue.append((task_func, args, kwargs, submit_time))


# Global thermal manager instance
thermal_manager = ThermalManager()


def thermal_event_callback(zone: ThermalZone, temp: float, level: str):
  """Default thermal event callback"""
  cloudlog.info(f"THERMAL EVENT: {zone.name} temperature {temp:.1f}°C - Level: {level}")


# Register default callbacks
thermal_manager.add_thermal_callback(thermal_event_callback)


def get_thermal_performance_scale() -> float:
  """Get the current thermal-based performance scaling factor"""
  return thermal_manager.get_current_performance_scale()


def is_thermal_throttling_active() -> bool:
  """Check if thermal throttling is currently active"""
  return thermal_manager.get_current_performance_scale() < 0.9


__all__ = [
  "ThermalZone", "ThermalReading", "ThermalPolicy", "ThermalManager",
  "ResourceAwareProcessor", "thermal_manager", "thermal_event_callback",
  "get_thermal_performance_scale", "is_thermal_throttling_active"
]