"""
Hardware Constraint Validator for Sunnypilot
Ensures all implementations adhere to Comma Three hardware constraints:
- RAM usage: < 1.4 GB
- CPU usage: < 5% average, < 10% peak
- End-to-end latency: < 80 ms
"""

import time
import psutil
from typing import Dict, List, Optional
from dataclasses import dataclass
import cereal.messaging as messaging
from common.swaglog import cloudlog


@dataclass
class ConstraintResult:
    """Result of a constraint check"""
    name: str
    current_value: float
    threshold: float
    unit: str
    compliant: bool
    message: str = ""


class HardwareConstraintValidator:
    """Validates that the system stays within hardware constraints"""
    
    def __init__(self):
        self.constraints = {
            'ram_usage_gb': {
                'threshold': 1.4,  # < 1.4 GB
                'description': 'RAM usage must be less than 1.4 GB'
            },
            'cpu_avg_percent': {
                'threshold': 5.0,  # < 5% average
                'description': 'CPU usage average must be less than 5%'
            },
            'cpu_peak_percent': {
                'threshold': 10.0,  # < 10% peak
                'description': 'CPU usage peak must be less than 10%'
            },
            'latency_ms': {
                'threshold': 80.0,  # < 80 ms
                'description': 'End-to-end latency must be less than 80 ms'
            }
        }
        
        # History for calculating averages
        self.history = {
            'cpu_percent': [],
            'ram_usage_mb': [],
            'latency_ms': []
        }
        self.max_history = 100  # Keep last 100 samples
        
        # Messaging for latency measurements
        self.sm = messaging.SubMaster(['modelV2', 'controlsState'], timeout=1000)
    
    def _add_to_history(self, metric: str, value: float):
        """Add a value to the history for averaging"""
        if metric in self.history:
            self.history[metric].append(value)
            if len(self.history[metric]) > self.max_history:
                self.history[metric] = self.history[metric][-self.max_history:]
    
    def validate_ram_usage(self) -> ConstraintResult:
        """Validate RAM usage constraint"""
        try:
            memory = psutil.virtual_memory()
            ram_usage_mb = memory.used / (1024**2)  # Convert to MB
            ram_usage_gb = memory.used / (1024**3)  # Convert to GB
            
            threshold = self.constraints['ram_usage_gb']['threshold']
            compliant = ram_usage_gb < threshold
            
            message = f"RAM usage {ram_usage_gb:.2f}GB is {'within' if compliant else 'exceeding'} limit of {threshold}GB"
            
            # Add to history
            self._add_to_history('ram_usage_mb', ram_usage_mb)
            
            return ConstraintResult(
                name='ram_usage_gb',
                current_value=ram_usage_gb,
                threshold=threshold,
                unit='GB',
                compliant=compliant,
                message=message
            )
        except Exception as e:
            cloudlog.error(f"Error validating RAM usage: {e}")
            return ConstraintResult(
                name='ram_usage_gb',
                current_value=0,
                threshold=self.constraints['ram_usage_gb']['threshold'],
                unit='GB',
                compliant=False,
                message=f"Error validating RAM usage: {e}"
            )
    
    def validate_cpu_usage(self) -> List[ConstraintResult]:
        """Validate CPU usage constraints (average and peak)"""
        results = []
        
        try:
            cpu_percent = psutil.cpu_percent(interval=None)  # Non-blocking call
            
            # Add to history for average calculation
            self._add_to_history('cpu_percent', cpu_percent)
            
            # Calculate average from history
            cpu_history = self.history['cpu_percent']
            cpu_avg = sum(cpu_history) / len(cpu_history) if cpu_history else cpu_percent
            cpu_peak = max(cpu_history) if cpu_history else cpu_percent
            
            # Validate average CPU usage
            avg_threshold = self.constraints['cpu_avg_percent']['threshold']
            avg_compliant = cpu_avg < avg_threshold
            avg_message = f"CPU average {cpu_avg:.2f}% is {'within' if avg_compliant else 'exceeding'} limit of {avg_threshold}%"
            
            avg_result = ConstraintResult(
                name='cpu_avg_percent',
                current_value=cpu_avg,
                threshold=avg_threshold,
                unit='%',
                compliant=avg_compliant,
                message=avg_message
            )
            results.append(avg_result)
            
            # Validate peak CPU usage
            peak_threshold = self.constraints['cpu_peak_percent']['threshold']
            peak_compliant = cpu_peak < peak_threshold
            peak_message = f"CPU peak {cpu_peak:.2f}% is {'within' if peak_compliant else 'exceeding'} limit of {peak_threshold}%"
            
            peak_result = ConstraintResult(
                name='cpu_peak_percent',
                current_value=cpu_peak,
                threshold=peak_threshold,
                unit='%',
                compliant=peak_compliant,
                message=peak_message
            )
            results.append(peak_result)
            
        except Exception as e:
            cloudlog.error(f"Error validating CPU usage: {e}")
            # Return failure results
            results.append(ConstraintResult(
                name='cpu_avg_percent',
                current_value=0,
                threshold=self.constraints['cpu_avg_percent']['threshold'],
                unit='%',
                compliant=False,
                message=f"Error validating CPU usage: {e}"
            ))
            results.append(ConstraintResult(
                name='cpu_peak_percent',
                current_value=0,
                threshold=self.constraints['cpu_peak_percent']['threshold'],
                unit='%',
                compliant=False,
                message=f"Error validating CPU usage: {e}"
            ))
        
        return results
    
    def validate_latency(self) -> ConstraintResult:
        """Validate end-to-end latency constraint"""
        try:
            # Update messaging to get latest data
            self.sm.update(0)
            
            latency_ms = 0.0
            
            if self.sm.updated['modelV2'] and self.sm.updated['controlsState']:
                # Calculate time difference between model output and control output
                model_time = self.sm.logMonoTime['modelV2'] / 1e9  # Convert to seconds
                control_time = self.sm.logMonoTime['controlsState'] / 1e9
                calculated_latency = (control_time - model_time) * 1000  # Convert to ms
                
                # Only consider reasonable latency values
                if 0 < calculated_latency < 200:  # Reasonable range
                    latency_ms = calculated_latency
                else:
                    cloudlog.warning(f"validate_latency: Unreasonable latency {calculated_latency:.1f}ms")
            
            threshold = self.constraints['latency_ms']['threshold']
            compliant = latency_ms < threshold and latency_ms > 0  # Must measure actual latency
            
            message = f"Latency {latency_ms:.2f}ms is {'within' if compliant else 'exceeding'} limit of {threshold}ms"
            
            # Add to history
            self._add_to_history('latency_ms', latency_ms)
            
            return ConstraintResult(
                name='latency_ms',
                current_value=latency_ms,
                threshold=threshold,
                unit='ms',
                compliant=compliant,
                message=message
            )
        except Exception as e:
            cloudlog.error(f"Error validating latency: {e}")
            return ConstraintResult(
                name='latency_ms',
                current_value=0,
                threshold=self.constraints['latency_ms']['threshold'],
                unit='ms',
                compliant=False,
                message=f"Error validating latency: {e}"
            )
    
    def run_validation(self) -> List[ConstraintResult]:
        """Run all validations and return results"""
        cloudlog.info("Running hardware constraint validation")
        
        results = []
        
        # Validate RAM usage
        results.append(self.validate_ram_usage())
        
        # Validate CPU usage
        results.extend(self.validate_cpu_usage())
        
        # Validate latency
        results.append(self.validate_latency())
        
        # Log results
        compliant_count = sum(1 for r in results if r.compliant)
        total_count = len(results)
        
        cloudlog.info(f"Hardware validation: {compliant_count}/{total_count} constraints met")
        
        for result in results:
            log_level = cloudlog.info if result.compliant else cloudlog.error
            log_level(f"  {result.name}: {result.current_value}{result.unit} "
                     f"(limit: {result.threshold}{result.unit}) - {result.message}")
        
        return results
    
    def get_current_compliance_report(self) -> Dict[str, float]:
        """Get current compliance metrics"""
        report = {}
        
        for metric, values in self.history.items():
            if values:
                report[f"{metric}_avg"] = sum(values) / len(values)
                report[f"{metric}_current"] = values[-1] if values else 0
                report[f"{metric}_peak"] = max(values) if values else 0
            else:
                report[f"{metric}_avg"] = 0
                report[f"{metric}_current"] = 0
                report[f"{metric}_peak"] = 0
        
        return report


def validate_hardware_constraints() -> bool:
    """Main function to validate hardware constraints"""
    validator = HardwareConstraintValidator()
    results = validator.run_validation()
    
    # Overall compliance is that all constraints pass
    all_compliant = all(result.compliant for result in results)
    
    if all_compliant:
        cloudlog.info("✓ All hardware constraints are within acceptable limits")
    else:
        cloudlog.error("✗ Some hardware constraints are being exceeded")
        
        # Log which constraints are failing
        failing = [r for r in results if not r.compliant]
        for result in failing:
            cloudlog.error(f"  Failing constraint: {result.message}")
    
    return all_compliant


def main():
    """Run hardware constraint validation"""
    print("Sunnypilot Hardware Constraint Validator")
    print("========================================")
    print("Validating system compliance with Comma Three hardware constraints:")
    print("- RAM usage: < 1.4 GB")
    print("- CPU usage: < 5% average, < 10% peak") 
    print("- End-to-end latency: < 80 ms")
    print()
    
    # Run single validation
    success = validate_hardware_constraints()
    
    if success:
        print("\n🎉 All hardware constraints are satisfied!")
        print("The system is operating within Comma Three hardware limits.")
        return 0
    else:
        print("\n❌ Hardware constraints are being exceeded!")
        print("The system may not operate properly on Comma Three hardware.")
        return 1


if __name__ == "__main__":
    exit(main())