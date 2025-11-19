"""
Power Consumption Optimization for Sunnypilot
Provides power management and optimization for ARM processors on Comma 3x
"""

import time
import threading
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass
from enum import Enum
import psutil
import os
import subprocess
from pathlib import Path


class PowerMode(Enum):
    """Power management modes for different driving situations"""
    PERFORMANCE = "performance"
    BALANCED = "balanced" 
    POWERSAVE = "powersave"
    ULTRA_LOW_POWER = "ultra_low_power"


class PowerState(Enum):
    """Current power state of the system"""
    IDLE = "idle"
    LOW_LOAD = "low_load"
    MEDIUM_LOAD = "medium_load"
    HIGH_LOAD = "high_load"
    PEAK_LOAD = "peak_load"


@dataclass
class PowerProfile:
    """Power profile defining constraints and targets"""
    name: str
    max_cpu_percent: float = 100.0  # Maximum CPU usage
    target_power_watts: float = 8.0  # Target power consumption
    min_frequency_mhz: Optional[int] = None  # Minimum CPU frequency
    max_frequency_mhz: Optional[int] = None  # Maximum CPU frequency
    gpu_enabled: bool = True  # Whether to use GPU
    threading_level: int = 4  # Max number of threads to use
    update_frequency_hz: float = 10.0  # How often to check power state


class PowerMonitor:
    """
    Power and thermal monitoring for power consumption optimization
    """
    
    def __init__(self):
        self.current_power = 0.0  # Current power consumption in watts
        self.power_history = []
        self.max_history = 100
        self.monitoring = False
        self.monitoring_thread = None
        self.lock = threading.Lock()
        self.last_update = time.time()
        
        # Power estimation parameters
        self.base_power = 1.5  # Base system power in watts
        self.cpu_power_factor = 0.05  # Additional power per % CPU usage
        self.ram_power_factor = 0.001  # Additional power per MB RAM used
    
    def start_monitoring(self):
        """Start power monitoring"""
        if self.monitoring:
            return
        
        self.monitoring = True
        self.monitoring_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitoring_thread.start()
    
    def stop_monitoring(self):
        """Stop power monitoring"""
        self.monitoring = False
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=2.0)
    
    def _monitor_loop(self):
        """Main monitoring loop"""
        while self.monitoring:
            try:
                estimated_power = self.estimate_power_consumption()
                
                with self.lock:
                    self.current_power = estimated_power
                    self.power_history.append((time.time(), estimated_power))
                    if len(self.power_history) > self.max_history:
                        self.power_history.pop(0)
                
                time.sleep(0.5)  # Update every 500ms
            except Exception as e:
                print(f"Power monitoring error: {e}")
                time.sleep(1.0)
    
    def estimate_power_consumption(self) -> float:
        """Estimate current power consumption based on system metrics"""
        # This is a simplified estimation - real implementation would use hardware power sensors
        cpu_percent = psutil.cpu_percent(interval=0.1)
        memory_info = psutil.virtual_memory()
        memory_mb = memory_info.used / (1024 * 1024)
        
        # Simple power model: Base + CPU load + Memory usage
        estimated_power = (
            self.base_power +
            (cpu_percent * self.cpu_power_factor) +
            (memory_mb * self.ram_power_factor)
        )
        
        # Add a small factor for disk and network activity
        disk_io = psutil.disk_io_counters()
        if disk_io:
            io_factor = (disk_io.read_bytes + disk_io.write_bytes) / (1024 * 1024 * 1000)  # MB/s
            estimated_power += io_factor * 0.1
        
        return min(estimated_power, 15.0)  # Cap at reasonable maximum
    
    def get_current_power(self) -> float:
        """Get current estimated power consumption"""
        with self.lock:
            return self.current_power
    
    def get_average_power(self, minutes: int = 1) -> float:
        """Get average power consumption over specified time"""
        with self.lock:
            if not self.power_history:
                return self.current_power
            
            cutoff_time = time.time() - (minutes * 60)
            recent_measurements = [
                power for t, power in self.power_history if t >= cutoff_time
            ]
            
            return sum(recent_measurements) / len(recent_measurements) if recent_measurements else self.current_power


class PowerGovernor:
    """
    Dynamic power management that adjusts system behavior based on power consumption
    """
    
    def __init__(self, power_monitor: PowerMonitor):
        self.power_monitor = power_monitor
        self.current_mode = PowerMode.BALANCED
        self.active_profiles: Dict[str, PowerProfile] = {}
        self.profile_weights: Dict[str, float] = {}  # How much each profile contributes
        self.last_adjustment = time.time()
        self.adjustment_interval = 1.0  # Adjust every 1 second
        self.governor_active = False
        self.governor_thread = None
        
        # Register default profiles
        self._register_default_profiles()
    
    def _register_default_profiles(self):
        """Register default power profiles"""
        self.active_profiles = {
            "performance": PowerProfile(
                name="performance",
                max_cpu_percent=100.0,
                target_power_watts=10.0,
                threading_level=4,
                update_frequency_hz=20.0
            ),
            "balanced": PowerProfile(
                name="balanced", 
                max_cpu_percent=80.0,
                target_power_watts=7.0,
                threading_level=3,
                update_frequency_hz=15.0
            ),
            "powersave": PowerProfile(
                name="powersave",
                max_cpu_percent=60.0, 
                target_power_watts=5.0,
                threading_level=2,
                update_frequency_hz=10.0
            ),
            "ultra_low_power": PowerProfile(
                name="ultra_low_power",
                max_cpu_percent=40.0,
                target_power_watts=3.0,
                threading_level=1,
                update_frequency_hz=5.0
            )
        }
        
        # Set initial weights (balanced profile active by default)
        self.profile_weights = {"balanced": 1.0}
    
    def start_governor(self):
        """Start the power governor"""
        if self.governor_active:
            return
        
        self.governor_active = True
        self.governor_thread = threading.Thread(target=self._governor_loop, daemon=True)
        self.governor_thread.start()
    
    def stop_governor(self):
        """Stop the power governor"""
        self.governor_active = False
        if self.governor_thread:
            self.governor_thread.join(timeout=2.0)
    
    def _governor_loop(self):
        """Main governor loop"""
        while self.governor_active:
            try:
                self._adjust_power_settings()
                time.sleep(self.adjustment_interval)
            except Exception as e:
                print(f"Power governor error: {e}")
                time.sleep(1.0)
    
    def _adjust_power_settings(self):
        """Adjust power settings based on current consumption"""
        current_time = time.time()
        if current_time - self.last_adjustment < self.adjustment_interval:
            return
        
        self.last_adjustment = current_time
        
        current_power = self.power_monitor.get_current_power()
        target_power = self._get_target_power()
        
        # Determine how to adjust based on power vs target
        power_diff = current_power - target_power
        
        if power_diff > 1.0:  # Over target by more than 1W
            # Need to reduce power consumption
            self._reduce_power_consumption()
        elif power_diff < -0.5:  # Below target by more than 0.5W
            # Can potentially increase performance
            self._increase_performance()
    
    def _get_target_power(self) -> float:
        """Get current target power based on active profiles"""
        total_weight = sum(self.profile_weights.values())
        if total_weight == 0:
            return self.active_profiles["balanced"].target_power_watts
        
        weighted_target = 0.0
        for profile_name, weight in self.profile_weights.items():
            if profile_name in self.active_profiles:
                target = self.active_profiles[profile_name].target_power_watts
                weighted_target += target * (weight / total_weight)
        
        return weighted_target
    
    def _reduce_power_consumption(self):
        """Reduce power consumption by adjusting settings"""
        # Reduce active profiles that increase power
        if self.profile_weights.get("performance", 0) > 0.1:
            self.profile_weights["performance"] = max(0, self.profile_weights["performance"] - 0.1)
        
        # Increase power-saving profiles if they're too low
        if self.profile_weights.get("powersave", 0) < 0.5:
            self.profile_weights["powersave"] = min(1.0, self.profile_weights["powersave"] + 0.1)
    
    def _increase_performance(self):
        """Increase performance if power consumption allows"""
        # Increase performance profiles if power allows
        if self.profile_weights.get("powersave", 0) > 0.1:
            self.profile_weights["powersave"] = max(0, self.profile_weights["powersave"] - 0.05)
        
        if self.profile_weights.get("balanced", 0) < 0.8:
            self.profile_weights["balanced"] = min(1.0, self.profile_weights["balanced"] + 0.05)
    
    def set_power_mode(self, mode: PowerMode):
        """Set a specific power mode"""
        # Reset all weights
        for profile_name in self.active_profiles:
            self.profile_weights[profile_name] = 0.0
        
        # Set the specified mode
        self.profile_weights[mode.value] = 1.0
        self.current_mode = mode
    
    def get_current_profile(self) -> PowerProfile:
        """Get the most influential active power profile"""
        if not self.profile_weights:
            return self.active_profiles["balanced"]
        
        dominant_profile = max(self.profile_weights.items(), key=lambda x: x[1])
        return self.active_profiles.get(dominant_profile[0], self.active_profiles["balanced"])


class PowerOptimizer:
    """
    Main power optimization controller that combines monitoring and governance
    """
    
    def __init__(self):
        self.power_monitor = PowerMonitor()
        self.power_governor = PowerGovernor(self.power_monitor)
        self.system_state = PowerState.LOW_LOAD
        self.optimization_callbacks: List[Callable] = []
        self.active = False
        
        # Power optimization statistics
        self.stats = {
            'power_saved_wh': 0.0,  # Power saved in watt-hours
            'throttling_events': 0,
            'performance_maintained': 0,
            'efficiency_ratio': 1.0
        }
    
    def start_optimization(self):
        """Start power optimization system"""
        if self.active:
            return
        
        self.power_monitor.start_monitoring()
        self.power_governor.start_governor()
        self.active = True
        
        # Start background optimization
        self._optimization_thread = threading.Thread(target=self._optimization_loop, daemon=True)
        self._optimization_thread.start()
    
    def stop_optimization(self):
        """Stop power optimization system"""
        self.active = False
        self.power_governor.stop_governor()
        self.power_monitor.stop_monitoring()
    
    def _optimization_loop(self):
        """Main optimization loop"""
        while self.active:
            try:
                self._update_system_state()
                self._apply_optimizations()
                time.sleep(0.2)  # Update every 200ms
            except Exception as e:
                print(f"Power optimization error: {e}")
                time.sleep(0.5)
    
    def _update_system_state(self):
        """Update system power state"""
        current_power = self.power_monitor.get_current_power()
        avg_power = self.power_monitor.get_average_power(minutes=1)
        
        # Determine state based on power consumption
        if avg_power > 9.0:
            new_state = PowerState.PEAK_LOAD
        elif avg_power > 7.0:
            new_state = PowerState.HIGH_LOAD
        elif avg_power > 5.0:
            new_state = PowerState.MEDIUM_LOAD
        elif avg_power > 3.0:
            new_state = PowerState.LOW_LOAD
        else:
            new_state = PowerState.IDLE
        
        self.system_state = new_state
    
    def _apply_optimizations(self):
        """Apply power optimizations based on current state"""
        profile = self.power_governor.get_current_profile()
        
        # Apply optimizations for different system components
        self._optimize_cpu_usage(profile)
        self._optimize_memory_usage(profile)
        self._optimize_neural_network_inference(profile)
    
    def _optimize_cpu_usage(self, profile: PowerProfile):
        """Optimize CPU usage based on profile"""
        # Limit CPU usage if specified
        if profile.max_cpu_percent < 100.0:
            # This would implement CPU rate limiting in a real system
            pass
    
    def _optimize_memory_usage(self, profile: PowerProfile):
        """Optimize memory usage based on profile"""
        # Reduce memory allocation rate in power-saving modes
        if profile.max_cpu_percent < 70.0:  # Power saving mode
            # Trigger more aggressive garbage collection
            import gc
            gc.collect()
    
    def _optimize_neural_network_inference(self, profile: PowerProfile):
        """Optimize neural network inference based on power profile"""
        # Reduce neural network complexity in power saving modes
        if profile.max_cpu_percent < 70.0:  # Power saving mode
            # This would adjust neural network batch sizes or precision
            pass
    
    def register_optimization_callback(self, callback: Callable):
        """Register a callback for power optimization"""
        self.optimization_callbacks.append(callback)
    
    def update_power_mode(self, mode: Optional[PowerMode] = None, 
                         custom_weights: Optional[Dict[str, float]] = None):
        """Update power mode or weights"""
        if mode is not None:
            self.power_governor.set_power_mode(mode)
        elif custom_weights is not None:
            self.power_governor.profile_weights.update(custom_weights)
    
    def get_power_efficiency_metrics(self) -> Dict[str, Any]:
        """Get power efficiency optimization metrics"""
        return {
            'current_power_w': self.power_monitor.get_current_power(),
            'average_power_w': self.power_monitor.get_average_power(minutes=1),
            'system_state': self.system_state.value,
            'current_mode': self.power_governor.current_mode.value,
            'active_profile': self.power_governor.get_current_profile().name,
            'stats': self.stats.copy()
        }
    
    def estimate_power_savings(self, baseline_power: float = 7.5) -> float:
        """Estimate power savings compared to baseline"""
        current_avg = self.power_monitor.get_average_power(minutes=1)
        saved_power = max(0, baseline_power - current_avg)
        
        # Update stats
        self.stats['power_saved_wh'] += saved_power * (1/3600)  # Convert to Wh assuming 1-second intervals
        
        return saved_power


def get_power_optimizer() -> PowerOptimizer:
    """Get the global power optimizer instance"""
    if not hasattr(get_power_optimizer, 'instance'):
        get_power_optimizer.instance = PowerOptimizer()
        get_power_optimizer.instance.start_optimization()
    return get_power_optimizer.instance


def enable_power_optimization():
    """Enable power optimization across the system"""
    optimizer = get_power_optimizer()
    return optimizer


def get_current_power_consumption() -> float:
    """Get current estimated power consumption"""
    optimizer = get_power_optimizer()
    return optimizer.power_monitor.get_current_power()


def set_power_mode(mode: PowerMode):
    """Set the system power mode"""
    optimizer = get_power_optimizer()
    optimizer.update_power_mode(mode=mode)


def get_power_efficiency_metrics() -> Dict[str, Any]:
    """Get power efficiency metrics"""
    optimizer = get_power_optimizer()
    return optimizer.get_power_efficiency_metrics()


def estimate_power_savings(baseline_power: float = 7.5) -> float:
    """Estimate power savings compared to baseline"""
    optimizer = get_power_optimizer()
    return optimizer.estimate_power_savings(baseline_power)


# Example usage and testing
if __name__ == "__main__":
    print("Testing Power Consumption Optimization System...")
    
    # Initialize power optimizer
    power_opt = get_power_optimizer()
    
    # Test 1: Basic power monitoring
    print("Test 1: Power monitoring")
    
    initial_power = get_current_power_consumption()
    print(f"  Initial estimated power consumption: {initial_power:.2f}W")
    
    # Wait a bit and check again
    time.sleep(1)
    avg_power = power_opt.power_monitor.get_average_power(minutes=1)
    print(f"  Average power consumption: {avg_power:.2f}W")
    
    # Test 2: Power mode switching
    print("\nTest 2: Power mode switching")
    
    # Set to balanced mode
    set_power_mode(PowerMode.BALANCED)
    metrics = get_power_efficiency_metrics()
    print(f"  Set to balanced mode - Current state: {metrics['system_state']}")
    print(f"  Current profile: {metrics['active_profile']}")
    
    # Test 3: Efficiency metrics
    print("\nTest 3: Efficiency metrics")
    
    metrics = get_power_efficiency_metrics()
    print(f"  Current power: {metrics['current_power_w']:.2f}W")
    print(f"  Average power: {metrics['average_power_w']:.2f}W")
    print(f"  System state: {metrics['system_state']}")
    print(f"  Current mode: {metrics['current_mode']}")
    
    # Test 4: Power savings estimation
    print("\nTest 4: Power savings estimation")
    
    savings = estimate_power_savings(baseline_power=8.0)
    print(f"  Estimated power savings vs 8.0W baseline: {savings:.2f}W")
    
    # Test 5: Load simulation
    print("\nTest 5: Load simulation and response")
    
    # Simulate higher power usage by creating some computation load
    import numpy as np
    
    def cpu_load_task():
        # Create some CPU load to see how the governor responds
        large_array = np.random.random((1000, 1000))
        result = np.linalg.svd(large_array)
        return result[1][0]  # Return largest singular value
    
    # Run load for a bit to increase power consumption
    start_time = time.time()
    for i in range(5):  # About 5 seconds of load
        result = cpu_load_task()
        time.sleep(0.1)
    
    load_time = time.time() - start_time
    print(f"  Applied load for {load_time:.1f}s, sample result: {result:.6f}")
    
    # Check if system responded to higher load
    time.sleep(2)  # Allow governor to respond
    metrics_after_load = get_power_efficiency_metrics()
    print(f"  Power after load: {metrics_after_load['current_power_w']:.2f}W")
    print(f"  System state after load: {metrics_after_load['system_state']}")
    
    print("\nPower consumption optimization system test completed!")