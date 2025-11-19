"""
Dynamic Performance Adaptation System for sunnypilot
Adapts computational complexity based on real-time system load and thermal conditions
"""
import time
import threading
import psutil
from enum import IntEnum
from dataclasses import dataclass
from typing import Dict, Optional, Callable, Any
from collections import deque

import cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import HARDWARE


class PerformanceMode(IntEnum):
  """Performance modes based on system conditions"""
  MAX_PERFORMANCE = 0    # Full computational resources
  BALANCED = 1           # Balanced performance and power
  POWER_SAVING = 2       # Reduced performance, power saving
  THERMAL_THROTTLE = 3   # Thermal throttling active
  CRITICAL_THROTTLE = 4  # Critical throttling


@dataclass
class SystemLoad:
  """Current system load metrics"""
  cpu_percent: float
  memory_percent: float
  gpu_percent: float
  cpu_temp: float  # in Celsius
  gpu_temp: float  # in Celsius
  thermal_status: int  # DeviceState.ThermalStatus
  timestamp: float


@dataclass
class AdaptationConfig:
  """Configuration for performance adaptation"""
  # CPU usage thresholds
  cpu_balanced_threshold: float = 60.0    # Switch to balanced mode
  cpu_power_save_threshold: float = 80.0  # Switch to power saving
  cpu_throttle_threshold: float = 90.0    # Initiate throttling
  
  # Temperature thresholds (in Celsius)
  temp_balanced_threshold: float = 60.0
  temp_power_save_threshold: float = 70.0
  temp_throttle_threshold: float = 80.0
  temp_critical_threshold: float = 90.0
  
  # Memory thresholds
  memory_balanced_threshold: float = 70.0
  memory_power_save_threshold: float = 85.0
  
  # Update interval (seconds)
  update_interval: float = 0.5


class DynamicPerformanceAdaptation:
  """
  Main class for dynamic performance adaptation

  This class monitors system conditions (CPU, memory, temperature) and dynamically
  adjusts performance modes to maintain optimal operation under varying system loads.
  """

  def __init__(self, config: Optional[AdaptationConfig] = None):
    """
    Initialize the dynamic performance adaptation system

    Args:
        config: Optional configuration for adaptation thresholds and parameters
    """
    self.config = config or AdaptationConfig()
    self.current_mode = PerformanceMode.MAX_PERFORMANCE
    self.last_load: Optional[SystemLoad] = None
    self.load_history = deque(maxlen=20)  # Keep last 10 seconds of data at 0.5s intervals
    self.adaptation_lock = threading.Lock()

    # Callbacks for when performance mode changes
    self.mode_change_callbacks: list[Callable[[PerformanceMode, PerformanceMode], None]] = []

    # Subscribe to device state for thermal information
    self.sm = messaging.SubMaster(['deviceState'])

    # Start monitoring thread
    self.monitoring_thread = threading.Thread(target=self._monitor_system, daemon=True)
    self.monitoring_thread.start()
  
  def add_mode_change_callback(self, callback: Callable[[PerformanceMode, PerformanceMode], None]):
    """Add callback for performance mode changes"""
    self.mode_change_callbacks.append(callback)
  
  def _get_system_load(self) -> SystemLoad:
    """Get current system load metrics"""
    # Get CPU usage
    try:
      cpu_percent = psutil.cpu_percent(interval=None)
      cpu_per_core = psutil.cpu_percent(percpu=True)
    except Exception as e:
      cloudlog.error(f"Error getting CPU usage: {e}")
      cpu_percent = 0.0

    # Get memory usage
    try:
      memory = psutil.virtual_memory()
      memory_percent = memory.percent
    except Exception as e:
      cloudlog.error(f"Error getting memory usage: {e}")
      memory_percent = 0.0

    # Get GPU usage (if available)
    try:
      gpu_percent = HARDWARE.get_gpu_usage_percent()
    except Exception as e:
      cloudlog.error(f"Error getting GPU usage: {e}")
      gpu_percent = 0.0

    # Get temperatures
    try:
      cpu_temps = HARDWARE.get_thermal_config().get_cpu_temps()
      gpu_temps = HARDWARE.get_thermal_config().get_gpu_temps()
    except Exception as e:
      cloudlog.error(f"Error getting thermal data: {e}")
      cpu_temps = []
      gpu_temps = []

    cpu_temp = max(cpu_temps) if cpu_temps else 0.0
    gpu_temp = max(gpu_temps) if gpu_temps else 0.0

    # Get thermal status from device state with proper error handling
    try:
      thermal_status = self.sm['deviceState'].thermalStatus if self.sm.valid['deviceState'] else 0
    except Exception as e:
      cloudlog.error(f"Error getting thermal status: {e}")
      thermal_status = 0

    return SystemLoad(
      cpu_percent=cpu_percent,
      memory_percent=memory_percent,
      gpu_percent=gpu_percent,
      cpu_temp=cpu_temp,
      gpu_temp=gpu_temp,
      thermal_status=thermal_status,
      timestamp=time.time()
    )
  
  def _determine_performance_mode(self, load: SystemLoad) -> PerformanceMode:
    """Determine appropriate performance mode based on system load"""
    # Check for critical conditions first
    if load.cpu_temp >= self.config.temp_critical_threshold or load.gpu_temp >= self.config.temp_critical_threshold:
      return PerformanceMode.CRITICAL_THROTTLE
    
    # Check thermal status from device state (higher priority)
    if load.thermal_status >= 3:  # danger status
      return PerformanceMode.CRITICAL_THROTTLE
    elif load.thermal_status >= 2:  # red status
      return PerformanceMode.THERMAL_THROTTLE
    elif load.thermal_status >= 1:  # yellow status
      return PerformanceMode.POWER_SAVING
    
    # Check temperature thresholds
    max_temp = max(load.cpu_temp, load.gpu_temp)
    if max_temp >= self.config.temp_throttle_threshold:
      return PerformanceMode.THERMAL_THROTTLE
    elif max_temp >= self.config.temp_power_save_threshold:
      return PerformanceMode.POWER_SAVING
    elif max_temp >= self.config.temp_balanced_threshold:
      return PerformanceMode.BALANCED
    
    # Check CPU usage thresholds
    if load.cpu_percent >= self.config.cpu_throttle_threshold:
      return PerformanceMode.THERMAL_THROTTLE  # Use thermal throttle for high CPU
    elif load.cpu_percent >= self.config.cpu_power_save_threshold:
      return PerformanceMode.POWER_SAVING
    elif load.cpu_percent >= self.config.cpu_balanced_threshold:
      return PerformanceMode.BALANCED
    
    # Check memory usage
    if load.memory_percent >= self.config.memory_power_save_threshold:
      return PerformanceMode.POWER_SAVING
    elif load.memory_percent >= self.config.memory_balanced_threshold:
      return PerformanceMode.BALANCED
    
    # Default to max performance if no thresholds are met
    return PerformanceMode.MAX_PERFORMANCE
  
  def _monitor_system(self):
    """Background thread to monitor system load"""
    while True:
      try:
        load = self._get_system_load()
        
        with self.adaptation_lock:
          self.last_load = load
          self.load_history.append(load)
          
          # Determine new performance mode
          new_mode = self._determine_performance_mode(load)
          
          # Trigger callbacks if mode changed
          if new_mode != self.current_mode:
            old_mode = self.current_mode
            self.current_mode = new_mode
            
            # Call all registered callbacks
            for callback in self.mode_change_callbacks:
              try:
                callback(old_mode, new_mode)
              except Exception as e:
                cloudlog.error(f"Performance mode callback error: {e}")
        
        # Update submaster
        self.sm.update(0)
        
        time.sleep(self.config.update_interval)
        
      except Exception as e:
        cloudlog.error(f"System monitoring error: {e}")
        time.sleep(1.0)  # Fallback sleep if there's an error
  
  def get_current_mode(self) -> PerformanceMode:
    """Get current performance mode"""
    with self.adaptation_lock:
      return self.current_mode
  
  def get_system_load(self) -> Optional[SystemLoad]:
    """Get current system load"""
    with self.adaptation_lock:
      return self.last_load
  
  def get_load_history(self) -> deque:
    """Get historical system load data"""
    with self.adaptation_lock:
      return self.load_history.copy()
  
  def should_reduce_computation(self) -> bool:
    """Check if computation should be reduced based on current conditions"""
    mode = self.get_current_mode()
    return mode in [PerformanceMode.POWER_SAVING, PerformanceMode.THERMAL_THROTTLE, PerformanceMode.CRITICAL_THROTTLE]
  
  def get_computation_factor(self) -> float:
    """Get factor by which computation should be adjusted (0.0 to 1.0)"""
    mode = self.get_current_mode()
    if mode == PerformanceMode.CRITICAL_THROTTLE:
      return 0.3  # Reduce to 30% of normal
    elif mode == PerformanceMode.THERMAL_THROTTLE:
      return 0.5  # Reduce to 50% of normal
    elif mode == PerformanceMode.POWER_SAVING:
      return 0.7  # Reduce to 70% of normal
    elif mode == PerformanceMode.BALANCED:
      return 0.9  # Reduce to 90% of normal
    else:  # MAX_PERFORMANCE
      return 1.0  # Full performance


# Global instance of dynamic performance adaptation
dynamic_adaptation = DynamicPerformanceAdaptation()


def adapt_model_computation(func: Callable, *args, **kwargs) -> Any:
  """Decorator to adapt model computation based on system conditions"""
  factor = dynamic_adaptation.get_computation_factor()
  
  if factor < 0.8:  # Only adapt computation if significantly throttled
    # Reduce model complexity, skip some computations, etc.
    # This is a simplified example - real implementation would depend on the specific function
    return func(*args, **kwargs)
  else:
    return func(*args, **kwargs)


class PerformanceAdaptationManager:
  """Manager for handling performance adaptation across different system components"""
  
  def __init__(self):
    self.adaptation_system = dynamic_adaptation
    self.component_configs: Dict[str, Dict[str, Any]] = {}
    self.component_adaptation_factors: Dict[str, float] = {}
  
  def register_component(self, name: str, initial_config: Dict[str, Any]):
    """Register a component for performance adaptation"""
    self.component_configs[name] = initial_config
    self.component_adaptation_factors[name] = 1.0
  
  def get_component_factor(self, component_name: str) -> float:
    """Get performance adaptation factor for a specific component"""
    factor = self.adaptation_system.get_computation_factor()
    self.component_adaptation_factors[component_name] = factor
    return factor
  
  def should_skip_component(self, component_name: str) -> bool:
    """Check if a component should be skipped based on current conditions"""
    factor = self.get_component_factor(component_name)
    return factor < 0.4  # Skip if factor is below 40%
  
  def get_component_config(self, component_name: str) -> Dict[str, Any]:
    """Get adapted configuration for a component"""
    base_config = self.component_configs.get(component_name, {})
    factor = self.get_component_factor(component_name)
    
    # Adjust configuration based on factor
    adapted_config = base_config.copy()
    
    # Example adjustments - these would be specific to each component
    if 'update_rate' in adapted_config:
      adapted_config['update_rate'] = int(base_config['update_rate'] * factor)
      adapted_config['update_rate'] = max(1, adapted_config['update_rate'])  # Don't go below 1
    
    if 'precision' in adapted_config:
      if factor < 0.7:
        adapted_config['precision'] = 'low'  # Reduce precision for high load
      elif factor < 0.9:
        adapted_config['precision'] = 'medium'
      else:
        adapted_config['precision'] = 'high'
    
    return adapted_config


# Global performance adaptation manager
performance_manager = PerformanceAdaptationManager()


__all__ = [
  "PerformanceMode", "SystemLoad", "AdaptationConfig",
  "DynamicPerformanceAdaptation", "dynamic_adaptation",
  "adapt_model_computation", "PerformanceAdaptationManager",
  "performance_manager"
]