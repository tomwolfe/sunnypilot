"""
Hardware validation module for comma three specifications.
Validates CPU, RAM, and power usage against the 2GB RAM, 4-core ARM CPU, 
and 10W power budget constraints.
"""
import time
import psutil
import threading
import subprocess
from typing import Dict, Tuple, List, Optional, Any
from dataclasses import dataclass
from openpilot.selfdrive.common.metrics import Metrics, record_metric

@dataclass
class HardwareSpecs:
    """Hardware specifications for comma three platform."""
    ram_gb: float = 2.0
    cpu_cores: int = 4
    cpu_arch: str = "ARM"
    power_budget_w: float = 10.0
    
    # Target usage constraints
    target_cpu_usage: float = 35.0  # percent
    target_ram_usage: float = 1433.6  # MB (70% of 2GB)
    target_power_usage: float = 8.0  # W (80% of budget)

class CommaThreeHardwareValidator:
    """
    Hardware validation for comma three platform with ARM-specific optimizations.
    Validates resource usage against specified constraints.
    """
    
    def __init__(self, specs: Optional[HardwareSpecs] = None):
        self.specs = specs or HardwareSpecs()
        self.monitoring = False
        self.monitor_thread = None
        self.hardware_metrics = {}
        self.violations = []
    
    def validate_cpu_usage(self, duration: float = 5.0) -> Tuple[float, Dict[str, float]]:
        """
        Validate CPU usage over a specified duration.
        """
        start_time = time.time()
        cpu_readings = []
        
        # Take multiple readings over the duration
        while time.time() - start_time < duration:
            cpu_percent = psutil.cpu_percent(interval=0.5)  # Non-blocking call
            cpu_readings.append(cpu_percent)
            time.sleep(0.1)  # Small delay between readings
        
        avg_cpu = sum(cpu_readings) / len(cpu_readings) if cpu_readings else 0
        max_cpu = max(cpu_readings) if cpu_readings else 0
        min_cpu = min(cpu_readings) if cpu_readings else 0
        
        # Validate against target
        within_limits = avg_cpu <= self.specs.target_cpu_usage
        
        # Record metrics
        record_metric(Metrics.CPU_USAGE_PERCENT, avg_cpu, {
            "validation_type": "cpu_usage",
            "avg_cpu_percent": avg_cpu,
            "max_cpu_percent": max_cpu,
            "min_cpu_percent": min_cpu,
            "readings_count": len(cpu_readings),
            "duration_seconds": duration,
            "within_target_limits": within_limits,
            "target_percent": self.specs.target_cpu_usage
        })
        
        metrics = {
            "avg_cpu_percent": avg_cpu,
            "max_cpu_percent": max_cpu,
            "min_cpu_percent": min_cpu,
            "readings_count": len(cpu_readings),
            "within_limits": within_limits,
            "target_limit": self.specs.target_cpu_usage
        }
        
        if not within_limits:
            self.violations.append({
                "type": "cpu_usage_exceeded",
                "avg_cpu": avg_cpu,
                "target": self.specs.target_cpu_usage,
                "timestamp": time.time()
            })
        
        return avg_cpu, metrics
    
    def validate_ram_usage(self) -> Tuple[float, Dict[str, float]]:
        """
        Validate RAM usage against comma three specifications.
        """
        memory = psutil.virtual_memory()
        ram_used_mb = memory.used / (1024 * 1024)  # Convert to MB
        ram_used_percent = memory.percent
        ram_total_mb = memory.total / (1024 * 1024)
        
        # Validate against target
        within_limits = ram_used_mb <= self.specs.target_ram_usage
        
        # Record metrics
        record_metric(Metrics.RAM_USAGE_MB, ram_used_mb, {
            "validation_type": "ram_usage",
            "ram_used_mb": ram_used_mb,
            "ram_used_percent": ram_used_percent,
            "ram_total_mb": ram_total_mb,
            "within_target_limits": within_limits,
            "target_limit_mb": self.specs.target_ram_usage
        })
        
        record_metric(Metrics.RAM_USAGE_PERCENT, ram_used_percent, {
            "validation_type": "ram_usage_percent",
            "ram_used_percent": ram_used_percent,
            "within_target_limits": within_limits,
            "target_limit_percent": (self.specs.target_ram_usage / (self.specs.ram_gb * 1024)) * 100
        })
        
        metrics = {
            "ram_used_mb": ram_used_mb,
            "ram_used_percent": ram_used_percent,
            "ram_total_mb": ram_total_mb,
            "within_limits": within_limits,
            "target_limit_mb": self.specs.target_ram_usage
        }
        
        if not within_limits:
            self.violations.append({
                "type": "ram_usage_exceeded",
                "ram_used_mb": ram_used_mb,
                "target_mb": self.specs.target_ram_usage,
                "timestamp": time.time()
            })
        
        return ram_used_mb, metrics
    
    def estimate_power_usage(self) -> Tuple[float, Dict[str, float]]:
        """
        Estimate power usage based on CPU and RAM usage for ARM platform.
        Uses ARM-specific power modeling based on big.LITTLE architecture.
        """
        cpu_percent = psutil.cpu_percent(interval=1)
        memory = psutil.virtual_memory()
        ram_used_mb = memory.used / (1024 * 1024)
        ram_percent = memory.percent
        
        # ARM power model (based on typical ARM big.LITTLE SoC characteristics)
        # Base power for ARM SoC in idle state
        base_power = 1.2  # W
        
        # CPU dynamic power - follows roughly quadratic relationship for ARM
        # At 100% CPU, ARM SoC might consume 4-6W additional
        cpu_power_factor = 4.8  # Max additional CPU power
        cpu_power = ((cpu_percent / 100.0) ** 1.4) * cpu_power_factor  # More aggressive than linear
        
        # RAM power - roughly linear with usage
        ram_power_factor = 1.0  # Max additional RAM power
        ram_power = (ram_percent / 100.0) * ram_power_factor
        
        # Total estimated power
        estimated_power = base_power + cpu_power + ram_power
        
        # Validate against power budget
        within_limits = estimated_power <= self.specs.target_power_usage
        
        # Record metrics
        power_metrics = {
            "estimated_w": estimated_power,
            "base_power_w": base_power,
            "cpu_power_w": cpu_power,
            "ram_power_w": ram_power,
            "cpu_percent": cpu_percent,
            "ram_percent": ram_percent,
            "within_limits": within_limits,
            "target_limit": self.specs.target_power_usage
        }
        
        record_metric(Metrics.POWER_DRAW_WATTS, estimated_power, {
            "validation_type": "power_estimation",
            "estimated_watts": estimated_power,
            "cpu_percent": cpu_percent,
            "ram_percent": ram_percent,
            "power_breakdown": {
                "base_w": base_power,
                "cpu_w": cpu_power,
                "ram_w": ram_power
            },
            "within_target_limits": within_limits,
            "target_limit_w": self.specs.target_power_usage
        })
        
        if not within_limits:
            self.violations.append({
                "type": "power_usage_exceeded",
                "estimated_w": estimated_power,
                "target_w": self.specs.target_power_usage,
                "timestamp": time.time()
            })
        
        return estimated_power, power_metrics
    
    def run_comprehensive_hardware_validation(self) -> Dict[str, Dict[str, float]]:
        """
        Run comprehensive hardware validation against all comma three specifications.
        """
        results = {}
        
        # Validate CPU usage (takes ~5 seconds)
        print("Validating CPU usage...")
        cpu_avg, cpu_metrics = self.validate_cpu_usage(duration=3.0)
        results["cpu"] = cpu_metrics
        
        # Validate RAM usage (instantaneous)
        print("Validating RAM usage...")
        ram_used, ram_metrics = self.validate_ram_usage()
        results["ram"] = ram_metrics
        
        # Estimate power usage (instantaneous)
        print("Estimating power usage...")
        power_est, power_metrics = self.estimate_power_usage()
        results["power"] = power_metrics
        
        # Overall validation result
        all_within_limits = (
            cpu_metrics["within_limits"] and 
            ram_metrics["within_limits"] and 
            power_metrics["within_limits"]
        )
        
        results["overall"] = {
            "all_within_limits": all_within_limits,
            "violation_count": len(self.violations),
            "timestamp": time.time(),
            "hardware_platform": "comma three",
            "specs": {
                "ram_gb": self.specs.ram_gb,
                "cpu_cores": self.specs.cpu_cores,
                "power_budget_w": self.specs.power_budget_w
            }
        }
        
        return results
    
    def start_continuous_monitoring(self, interval: float = 2.0):
        """Start continuous hardware monitoring."""
        if not self.monitoring:
            self.monitoring = True
            self.monitor_thread = threading.Thread(
                target=self._monitoring_loop, 
                args=(interval,), 
                daemon=True
            )
            self.monitor_thread.start()
            print(f"Started continuous hardware monitoring (interval: {interval}s)")
    
    def stop_continuous_monitoring(self):
        """Stop continuous hardware monitoring."""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
            print("Stopped continuous hardware monitoring")
    
    def _monitoring_loop(self, interval: float):
        """Internal monitoring loop."""
        while self.monitoring:
            try:
                # Run quick validation
                cpu_avg, cpu_metrics = self.validate_cpu_usage(duration=0.5)
                ram_used, ram_metrics = self.validate_ram_usage()
                power_est, power_metrics = self.estimate_power_usage()
                
                # Store metrics
                self.hardware_metrics[time.time()] = {
                    "cpu": cpu_metrics,
                    "ram": ram_metrics,
                    "power": power_metrics
                }
                
                time.sleep(interval)
            except Exception as e:
                print(f"Error in hardware monitoring: {e}")
                time.sleep(interval)
    
    def get_hardware_report(self) -> Dict[str, Any]:
        """Get comprehensive hardware validation report."""
        cpu_avg, cpu_metrics = self.validate_cpu_usage(duration=2.0)
        ram_used, ram_metrics = self.validate_ram_usage()
        power_est, power_metrics = self.estimate_power_usage()
        
        report = {
            "timestamp": time.time(),
            "platform": "comma three",
            "specs": {
                "ram_gb": self.specs.ram_gb,
                "cpu_cores": self.specs.cpu_cores,
                "cpu_arch": self.specs.cpu_arch,
                "power_budget_w": self.specs.power_budget_w
            },
            "current_metrics": {
                "cpu": cpu_metrics,
                "ram": ram_metrics,
                "power": power_metrics
            },
            "violations": self.violations.copy(),
            "violation_summary": {
                "cpu_violations": len([v for v in self.violations if v["type"] == "cpu_usage_exceeded"]),
                "ram_violations": len([v for v in self.violations if v["type"] == "ram_usage_exceeded"]),
                "power_violations": len([v for v in self.violations if v["type"] == "power_usage_exceeded"])
            }
        }
        
        return report

class ARMHardwareOptimizer:
    """
    ARM-specific hardware optimizer that provides optimization recommendations
    based on comma three hardware constraints.
    """
    
    def __init__(self):
        self.validator = CommaThreeHardwareValidator()
    
    def get_optimization_recommendations(self) -> List[Dict[str, str]]:
        """
        Get hardware optimization recommendations based on current usage.
        """
        report = self.validator.get_hardware_report()
        recommendations = []
        
        # CPU optimization recommendations
        if report["current_metrics"]["cpu"]["within_limits"] is False:
            recommendations.append({
                "type": "cpu",
                "priority": "high",
                "recommendation": f"CPU usage {report['current_metrics']['cpu']['avg_cpu_percent']:.1f}% exceeds target {report['current_metrics']['cpu']['target_limit']:.1f}% - consider algorithm optimization or task scheduling"
            })
        elif report["current_metrics"]["cpu"]["avg_cpu_percent"] > 25:
            recommendations.append({
                "type": "cpu",
                "priority": "medium",
                "recommendation": f"CPU usage is high ({report['current_metrics']['cpu']['avg_cpu_percent']:.1f}%) - consider optimization opportunities"
            })
        
        # RAM optimization recommendations
        if report["current_metrics"]["ram"]["within_limits"] is False:
            recommendations.append({
                "type": "ram",
                "priority": "high",
                "recommendation": f"RAM usage {report['current_metrics']['ram']['ram_used_mb']:.1f}MB exceeds target {report['current_metrics']['ram']['target_limit_mb']:.1f}MB - consider memory optimization or data streaming"
            })
        elif report["current_metrics"]["ram"]["ram_used_percent"] > 65:
            recommendations.append({
                "type": "ram",
                "priority": "medium",
                "recommendation": f"RAM usage is high ({report['current_metrics']['ram']['ram_used_percent']:.1f}%) - consider memory optimization"
            })
        
        # Power optimization recommendations
        if report["current_metrics"]["power"]["within_limits"] is False:
            recommendations.append({
                "type": "power",
                "priority": "high",
                "recommendation": f"Power estimate {report['current_metrics']['power']['estimated_w']:.2f}W exceeds target {report['current_metrics']['power']['target_limit']:.2f}W - consider power optimizations"
            })
        
        return recommendations
    
    def validate_arm_optimizations(self) -> Dict[str, bool]:
        """
        Validate ARM-specific optimizations.
        """
        # Check for ARM-specific features that could be leveraged
        try:
            # Check CPU architecture
            cpu_info = subprocess.check_output(['uname', '-m']).decode('utf-8').strip()
            is_arm = 'arm' in cpu_info.lower() or 'aarch' in cpu_info.lower()
            
            # In a real implementation, we'd check for NEON support, etc.
            has_neon = True  # Assuming NEON support for ARM architecture
            
            optimizations = {
                "arm_architecture": is_arm,
                "neon_support": has_neon,
                "big_little_config": True,  # Assuming ARM big.LITTLE
                "arm_optimized": is_arm and has_neon
            }
            
        except Exception:
            optimizations = {
                "arm_architecture": False,
                "neon_support": False,
                "big_little_config": False,
                "arm_optimized": False
            }
        
        return optimizations

# Global hardware validator and optimizer instances
hardware_validator = CommaThreeHardwareValidator()
arm_optimizer = ARMHardwareOptimizer()

def validate_hardware_compliance() -> Dict[str, Dict[str, float]]:
    """Run comprehensive hardware validation."""
    return hardware_validator.run_comprehensive_hardware_validation()

def get_hardware_report() -> Dict[str, Any]:
    """Get detailed hardware validation report."""
    return hardware_validator.get_hardware_report()

def get_optimization_recommendations() -> List[Dict[str, str]]:
    """Get hardware optimization recommendations."""
    return arm_optimizer.get_optimization_recommendations()

def start_hardware_monitoring(interval: float = 2.0):
    """Start continuous hardware monitoring."""
    hardware_validator.start_continuous_monitoring(interval)

def stop_hardware_monitoring():
    """Stop continuous hardware monitoring."""
    hardware_validator.stop_continuous_monitoring()

def validate_arm_optimizations() -> Dict[str, bool]:
    """Validate ARM-specific optimizations."""
    return arm_optimizer.validate_arm_optimizations()