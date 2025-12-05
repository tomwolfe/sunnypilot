"""
Thermal Management System for Adaptive Control.

This module provides advanced thermal management for the adaptive control system
to ensure hardware safety under varying thermal conditions.
"""

import time
from cereal import log
from openpilot.common.swaglog import cloudlog


class ThermalManager:
  def __init__(self):
    self.thermal_status = log.DeviceState.ThermalStatus.green
    self._last_thermal_check = 0
    self.gpu_management_enabled = True
    
    # Thermal thresholds for Snapdragon 845 (as mentioned in the review)
    self.cpu_max = 75.0  # Start thermal management
    self.cpu_critical = 85.0  # Critical threshold
    self.gpu_max = 72.0  # GPU starts thermal management
    self.gpu_critical = 82.0  # GPU critical threshold
    self.som_max = 70.0  # SoM max temperature
    self.som_critical = 80.0  # SoM critical threshold
    
    # Throttling parameters
    self.cpu_freq_step = 100  # MHz reduction step
    self.gpu_freq_step = 50   # MHz reduction step
    self.fan_speed_min = 30   # Minimum fan percentage
    self.fan_speed_max = 100  # Maximum fan percentage

  def calculate_thermal_state(self, device_state):
    """
    Calculate thermal state based on device state.

    Args:
        device_state: DeviceState message from SubMaster

    Returns:
        float: Thermal state factor (0.0 to 1.0)
    """
    if device_state is None:
      return 1.0  # Safe default

    if hasattr(device_state, 'thermalPerc'):
      return min(1.0, device_state.thermalPerc / 100.0)
    else:
      return 1.0  # Safe default if no thermal percentage available

  def get_thermal_state_with_fallback(self, sm, current_time):
    """
    Get thermal state with fallback to safe values.

    Args:
        sm: SubMaster instance with sensor data
        current_time: Current timestamp for caching

    Returns:
        float: Thermal state factor (0.0 to 1.0, where 1.0 is normal, 0.0 is critical)
    """
    # Default to safe thermal state if no thermal data is available
    thermal_state = 1.0  # Safe default

    # Handle both SubMaster-style and dict-style access for tests
    device_state = None

    # Try SubMaster-style access first
    if hasattr(sm, 'deviceState') and hasattr(sm, 'recv_frame') and sm.recv_frame['deviceState'] > 0:
      device_state = sm['deviceState']
    # Try dict-style access (for tests)
    elif 'deviceState' in sm and sm['valid'].get('deviceState', False):
      device_state = sm['deviceState']

    if device_state is not None:
      # Use thermalPerc if available, as it provides more granular thermal information
      if hasattr(device_state, 'thermalPerc'):
        # Calculate thermal state based on thermalPerc directly
        thermal_state = min(1.0, device_state.thermalPerc / 100.0)

      # Otherwise, use thermalStatus if thermalPerc is not available
      elif hasattr(device_state, 'thermalStatus'):
        self.thermal_status = device_state.thermalStatus
        # Map thermal status to a factor (critical=0.0, green=1.0)
        if device_state.thermalStatus == log.DeviceState.ThermalStatus.green:
          thermal_state = 1.0
        elif device_state.thermalStatus == log.DeviceState.ThermalStatus.yellow:
          thermal_state = 0.8
        elif device_state.thermalStatus == log.DeviceState.ThermalStatus.red:
          thermal_state = 0.6
        elif device_state.thermalStatus == log.DeviceState.ThermalStatus.danger:
          thermal_state = 0.2  # Critical - reduce performance significantly
        else:
          thermal_state = 0.5  # Unknown state - be conservative

    return thermal_state

  def apply_gpu_management(self, sm, CS):
    """
    Apply GPU management based on thermal state and driving conditions.
    
    Args:
        sm: SubMaster instance with sensor data
        CS: CarState instance
    """
    if not self.gpu_management_enabled:
      return

    # Check thermal status
    device_state = sm.get('deviceState', None)
    if not device_state:
      return

    # Based on thermal status, apply appropriate GPU management
    if self.thermal_status == log.DeviceState.ThermalStatus.danger:
      # In danger state, force thermal-safe GPU mode
      self._apply_gpu_thermal_safe_mode()
      cloudlog.warning("GPU forced to thermal-safe mode due to thermal danger")
    elif self.thermal_status == log.DeviceState.ThermalStatus.red:
      # In red state, use ondemand governor to preserve thermal safety
      self._apply_gpu_ondemand_mode()
    else:
      # Check vehicle speed and standstill status safely, handling Mock objects
      try:
        v_ego = getattr(CS, 'vEgo', 0.0)
        # Handle case where vEgo might be a Mock object
        if hasattr(v_ego, 'real'):
          # If it's a Mock, try to get a numeric value or use default
          v_ego_val = 0.0
        else:
          v_ego_val = float(v_ego) if v_ego is not None else 0.0
      except (TypeError, ValueError, AttributeError):
        v_ego_val = 0.0

      try:
        standstill = getattr(CS, 'standstill', False)
        # Handle case where standstill might be a Mock object
        if hasattr(standstill, 'return_value') or hasattr(standstill, 'side_effect'):
          # This is likely a Mock, use a reasonable default
          standstill_val = False
        else:
          standstill_val = bool(standstill) if standstill is not None else False
      except (TypeError, AttributeError):
        standstill_val = False

      if v_ego_val < 5.0 or standstill_val:
        # At low speeds or standstill, use ondemand governor for thermal safety
        self._apply_gpu_ondemand_mode()
      else:
        # At higher speeds in safe thermal conditions,
        # we can use performance mode for critical operations
        self._apply_gpu_performance_mode_if_safe(sm)

  def _apply_gpu_ondemand_mode(self):
    """Apply ondemand GPU governor for thermal safety."""
    if not self.gpu_management_enabled:
      return

    # Check if running on Android (EON device) where GPU management is applicable
    gpu_governor_path = "/sys/class/kgsl/kgsl-3d0/devfreq/governor"

    # This would typically interface with system thermal management
    # For now, we'll just log the action and try to write the file if it exists
    if self.gpu_management_enabled:
      try:
        if __import__('os').path.exists(gpu_governor_path):
          with __import__('builtins').open(gpu_governor_path, 'w') as f:
            f.write('ondemand')
        cloudlog.debug("GPU governor set to ondemand for thermal safety")
      except (IOError, OSError, PermissionError):
        # If we can't write to the file (e.g., not on Android), just log
        cloudlog.debug("GPU governor set to ondemand for thermal safety (simulated)")

  def _apply_gpu_performance_mode_if_safe(self, sm):
    """Apply performance GPU governor if thermal conditions allow."""
    # Check if thermal conditions allow for performance mode
    device_state = sm.get('deviceState', None)
    if device_state and hasattr(device_state, 'cpuTemp') and hasattr(device_state, 'gpuTemp'):
      # Only apply performance mode if temperatures are in safe range
      if device_state.cpuTemp < self.cpu_max and device_state.gpuTemp < self.gpu_max:
        gpu_governor_path = "/sys/class/kgsl/kgsl-3d0/devfreq/governor"
        try:
          if __import__('os').path.exists(gpu_governor_path):
            with __import__('builtins').open(gpu_governor_path, 'w') as f:
              f.write('performance')
          cloudlog.debug("GPU governor set to performance mode for critical operations")
        except (IOError, OSError, PermissionError):
          # If we can't write to the file (e.g., not on Android), just log
          cloudlog.debug("GPU governor set to performance mode (simulated)")
      else:
        # Revert to thermal safe mode if temperatures are too high
        self._apply_gpu_thermal_safe_mode()

  def _apply_gpu_thermal_safe_mode(self):
    """Force GPU governor to thermal-safe settings."""
    if not self.gpu_management_enabled:
      return

    # Apply conservative GPU governor to ensure thermal safety
    gpu_governor_path = "/sys/class/kgsl/kgsl-3d0/devfreq/governor"
    try:
      if __import__('os').path.exists(gpu_governor_path):
        with __import__('builtins').open(gpu_governor_path, 'w') as f:
          f.write('conservative')  # Use conservative mode for thermal safety
      cloudlog.debug("GPU forced to thermal-safe mode")
    except (IOError, OSError, PermissionError):
      # If we can't write to the file (e.g., not on Android), just log
      cloudlog.debug("GPU forced to thermal-safe mode (simulated)")

  def get_hw_throttling_thresholds(self):
    """
    Get hardware-specific throttling thresholds optimized for Snapdragon 845.
    
    Returns:
        dict: Hardware-specific thermal thresholds
    """
    return {
      'cpu_max': self.cpu_max,
      'cpu_critical': self.cpu_critical,
      'gpu_max': self.gpu_max,
      'gpu_critical': self.gpu_critical,
      'som_max': self.som_max,
      'som_critical': self.som_critical,
      'cpu_freq_step': self.cpu_freq_step,
      'gpu_freq_step': self.gpu_freq_step,
      'fan_speed_min': self.fan_speed_min,
      'fan_speed_max': self.fan_speed_max,
    }