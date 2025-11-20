#!/usr/bin/env python3
"""
Hardware-specific thermal management for sunnypilot
Handles platform-specific thermal thresholds and management
"""

import os
import platform
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
from openpilot.common.swaglog import cloudlog


@dataclass
class HardwareProfile:
    """Hardware profile with platform-specific parameters"""
    platform_name: str
    thermal_threshold: float  # Temperature threshold in Celsius
    cpu_util_threshold: float  # CPU utilization threshold (%)
    memory_threshold: float  # Memory utilization threshold (%)
    thermal_management_enabled: bool  # Whether thermal management is active
    performance_scaling_enabled: bool  # Whether performance scaling is supported


class HardwareThermalManager:
    """
    Manages hardware-specific thermal thresholds and thermal management
    """
    
    def __init__(self, platform_id: Optional[str] = None):
        self.platform_id = platform_id or self._detect_platform()
        self.hardware_profile = self._get_hardware_profile(self.platform_id)
        self.thermal_history = []
        self.max_history = 50  # Keep last 50 temperature readings
        
    def _detect_platform(self) -> str:
        """Detect the current hardware platform"""
        # Get platform information
        system_platform = platform.system().lower()
        machine = platform.machine().lower()
        
        # Try to read from system files if available
        if os.path.exists('/proc/device-tree/model'):
            try:
                with open('/proc/device-tree/model', 'r') as f:
                    model = f.read().strip().replace('\x00', '')
                    if 'raspberry' in model.lower():
                        return 'raspberry_pi'
                    elif 'jetson' in model.lower():
                        return 'nvidia_jetson'
            except:
                pass
        
        # Check for specific hardware indicators
        if 'arm' in machine or 'aarch64' in machine:
            # Likely embedded hardware
            if self._is_comma_device():
                return 'comma_device'  # openpilot hardware
            else:
                return 'embedded_arm'
        elif 'x86' in machine or 'x86_64' in machine:
            return 'x86_64'
        else:
            return 'unknown'
    
    def _is_comma_device(self) -> bool:
        """Check if running on comma.ai hardware"""
        try:
            # Check for comma-specific paths or hardware identifiers
            return os.path.exists('/data/params') or os.path.exists('/dev/kmsg')
        except:
            return False
    
    def _get_hardware_profile(self, platform_id: str) -> HardwareProfile:
        """Get hardware profile for the specified platform"""
        profiles: Dict[str, HardwareProfile] = {
            'comma_device': HardwareProfile(
                platform_name='comma_device',
                thermal_threshold=75.0,  # Standard openpilot device
                cpu_util_threshold=85.0,
                memory_threshold=85.0,
                thermal_management_enabled=True,
                performance_scaling_enabled=True
            ),
            'raspberry_pi': HardwareProfile(
                platform_name='raspberry_pi',
                thermal_threshold=70.0,  # Lower threshold for Pi's thermal characteristics
                cpu_util_threshold=80.0,
                memory_threshold=80.0,
                thermal_management_enabled=True,
                performance_scaling_enabled=False  # Pi may not support aggressive scaling
            ),
            'nvidia_jetson': HardwareProfile(
                platform_name='nvidia_jetson',
                thermal_threshold=80.0,  # Jetson can handle higher temperatures
                cpu_util_threshold=90.0,
                memory_threshold=90.0,
                thermal_management_enabled=True,
                performance_scaling_enabled=True
            ),
            'x86_64': HardwareProfile(
                platform_name='x86_64',
                thermal_threshold=85.0,  # Desktop/server hardware can handle more
                cpu_util_threshold=90.0,
                memory_threshold=90.0,
                thermal_management_enabled=True,
                performance_scaling_enabled=True
            ),
            'embedded_arm': HardwareProfile(
                platform_name='embedded_arm',
                thermal_threshold=72.0,  # Conservative for generic ARM
                cpu_util_threshold=82.0,
                memory_threshold=82.0,
                thermal_management_enabled=True,
                performance_scaling_enabled=True
            )
        }
        
        # Return specific profile if available, otherwise default to conservative values
        if platform_id in profiles:
            return profiles[platform_id]
        else:
            # Return a conservative default for unknown platforms
            return HardwareProfile(
                platform_name=platform_id,
                thermal_threshold=70.0,  # Conservative default
                cpu_util_threshold=80.0,
                memory_threshold=80.0,
                thermal_management_enabled=True,
                performance_scaling_enabled=True
            )
    
    def get_thermal_threshold(self) -> float:
        """Get the thermal threshold for this hardware platform"""
        return self.hardware_profile.thermal_threshold
    
    def get_cpu_threshold(self) -> float:
        """Get the CPU threshold for this hardware platform"""
        return self.hardware_profile.cpu_util_threshold
    
    def get_memory_threshold(self) -> float:
        """Get the memory threshold for this hardware platform"""
        return self.hardware_profile.memory_threshold
    
    def is_thermal_management_enabled(self) -> bool:
        """Check if thermal management is enabled for this platform"""
        return self.hardware_profile.thermal_management_enabled
    
    def is_performance_scaling_enabled(self) -> bool:
        """Check if performance scaling is enabled for this platform"""
        return self.hardware_profile.performance_scaling_enabled
    
    def update_temperature_reading(self, temperature: float):
        """Update with a new temperature reading"""
        self.thermal_history.append(temperature)
        if len(self.thermal_history) > self.max_history:
            self.thermal_history.pop(0)
    
    def get_temperature_trend(self) -> Tuple[float, str]:
        """
        Get the temperature trend (rate of change and direction)
        Returns: (trend_rate, trend_direction)
        """
        if len(self.thermal_history) < 5:
            return 0.0, "insufficient_data"
        
        # Calculate trend over last 5 readings
        recent_temps = self.thermal_history[-5:]
        if len(recent_temps) < 2:
            return 0.0, "stable"
        
        # Calculate average rate of change
        time_interval = 1.0  # Assuming 1 second intervals between readings
        temp_change = recent_temps[-1] - recent_temps[0]
        trend_rate = temp_change / (len(recent_temps) - 1)  # Average rate per reading
        
        if trend_rate > 0.5:
            trend_direction = "rapidly_increasing"
        elif trend_rate > 0.1:
            trend_direction = "increasing"
        elif trend_rate < -0.5:
            trend_direction = "rapidly_decreasing"
        elif trend_rate < -0.1:
            trend_direction = "decreasing"
        else:
            trend_direction = "stable"
            
        return trend_rate, trend_direction
    
    def should_reduce_performance(self, current_temp: float, 
                                current_cpu: float, 
                                current_memory: float) -> bool:
        """
        Determine if performance should be reduced based on current system state
        """
        if not self.hardware_profile.thermal_management_enabled:
            return False
            
        # Check if any threshold is exceeded
        thermal_exceeded = current_temp > self.hardware_profile.thermal_threshold
        cpu_exceeded = current_cpu > self.hardware_profile.cpu_util_threshold
        memory_exceeded = current_memory > self.hardware_profile.memory_threshold
        
        # Check temperature trend - if rapidly increasing, be more conservative
        trend_rate, trend_direction = self.get_temperature_trend()
        approaching_thermal = (trend_direction == "rapidly_increasing" and 
                              current_temp > self.hardware_profile.thermal_threshold * 0.8)
        
        return thermal_exceeded or cpu_exceeded or memory_exceeded or approaching_thermal
    
    def get_performance_reduction_factor(self, current_temp: float,
                                       current_cpu: float,
                                       current_memory: float) -> float:
        """
        Get factor by which performance should be reduced (0.0 to 1.0, where 1.0 = no reduction)
        """
        if not self.is_performance_scaling_enabled():
            return 1.0  # No scaling on this platform
            
        # Calculate how much each metric exceeds its threshold (0.0 = at threshold, > 0 = exceeded)
        thermal_factor = max(0.0, (current_temp - self.hardware_profile.thermal_threshold) / 10.0)  # Per 10 degrees over
        cpu_factor = max(0.0, (current_cpu - self.hardware_profile.cpu_util_threshold) / 10.0)  # Per 10% over
        memory_factor = max(0.0, (current_memory - self.hardware_profile.memory_threshold) / 10.0)  # Per 10% over
        
        # Calculate temperature trend factor
        trend_rate, trend_direction = self.get_temperature_trend()
        trend_factor = 0.0
        if trend_direction == "rapidly_increasing":
            trend_factor = min(0.5, abs(trend_rate) * 2.0)  # Amplify rapid increases
        elif trend_direction == "increasing":
            trend_factor = min(0.3, abs(trend_rate))
        
        # Combine all factors (max factor wins)
        max_factor = max(thermal_factor, cpu_factor, memory_factor, trend_factor)
        
        # Convert to reduction factor (1.0 - factor, but not below 0.1)
        reduction_factor = max(0.1, 1.0 - max_factor)
        
        return reduction_factor
    
    def get_platform_info(self) -> Dict[str, any]:
        """Get information about the current hardware platform"""
        return {
            'platform_id': self.platform_id,
            'platform_name': self.hardware_profile.platform_name,
            'thermal_threshold': self.hardware_profile.thermal_threshold,
            'cpu_threshold': self.hardware_profile.cpu_util_threshold,
            'memory_threshold': self.hardware_profile.memory_threshold,
            'thermal_management_enabled': self.hardware_profile.thermal_management_enabled,
            'performance_scaling_enabled': self.hardware_profile.performance_scaling_enabled
        }


def get_hardware_thermal_manager(platform_id: Optional[str] = None) -> HardwareThermalManager:
    """Convenience function to get hardware-specific thermal manager"""
    return HardwareThermalManager(platform_id)


def get_optimal_thermal_threshold(platform_id: Optional[str] = None) -> float:
    """Get the optimal thermal threshold for the current or specified platform"""
    manager = HardwareThermalManager(platform_id)
    return manager.get_thermal_threshold()


def should_reduce_performance(current_temp: float, 
                            current_cpu: float, 
                            current_memory: float,
                            platform_id: Optional[str] = None) -> bool:
    """Convenience function to check if performance should be reduced"""
    manager = HardwareThermalManager(platform_id)
    return manager.should_reduce_performance(current_temp, current_cpu, current_memory)