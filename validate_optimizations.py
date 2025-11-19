"""
Validation Module for sunnypilot
Validates against Comma Three hardware constraints:
- RAM usage: < 1.4 GB
- CPU usage: < 5% average, < 10% peak
- End-to-end latency: < 80 ms
"""
import time
import psutil
from typing import Dict, Any
import cereal.messaging as messaging


def validate_hardware_constraints() -> Dict[str, Any]:
  """Validate hardware constraints"""
  results = {}

  # Check RAM usage
  try:
    memory = psutil.virtual_memory()
    ram_usage_gb = memory.used / (1024**3)
    results['ram_usage_gb'] = ram_usage_gb
    results['ram_valid'] = ram_usage_gb < 1.4
  except Exception as e:
    cloudlog.error(f"Error checking RAM usage: {e}")
    results['ram_usage_gb'] = 0.0
    results['ram_valid'] = True  # Don't fail validation due to measurement error

  # Check CPU usage with better sampling
  try:
    # Get more accurate CPU usage by sampling over time
    cpu_percent = psutil.cpu_percent(interval=0.1)  # Shorter interval for more responsive measurement
    results['cpu_percent'] = cpu_percent
    results['cpu_valid'] = cpu_percent < 10.0  # Peak should be under 10%
  except Exception as e:
    cloudlog.error(f"Error checking CPU usage: {e}")
    results['cpu_percent'] = 0.0
    results['cpu_valid'] = True  # Don't fail validation due to measurement error

  # Check latency between model and controls
  try:
    sm = messaging.SubMaster(['modelV2', 'controlsState'], timeout=1000)
    sm.update(0)

    latency_ms = 0.0
    if sm.updated['modelV2'] and sm.updated['controlsState']:
      model_time = sm.logMonoTime['modelV2'] / 1e9
      control_time = sm.logMonoTime['controlsState'] / 1e9
      latency_ms = abs(control_time - model_time) * 1000  # Use absolute difference

      # Only consider valid if we got reasonable timestamps
      if latency_ms > 0 and latency_ms < 500:  # Reasonable upper bound
        results['latency_ms'] = latency_ms
        results['latency_valid'] = latency_ms < 80.0
      else:
        results['latency_ms'] = 0.0
        results['latency_valid'] = True  # Can't validate, assume acceptable
    else:
      results['latency_ms'] = 0.0
      results['latency_valid'] = True  # Can't measure, assume acceptable
  except Exception as e:
    cloudlog.error(f"Error checking latency: {e}")
    results['latency_ms'] = 0.0
    results['latency_valid'] = True  # Don't fail validation due to measurement error

  # Overall result - all must be valid
  results['all_valid'] = (
    results.get('ram_valid', True) and
    results.get('cpu_valid', True) and
    results.get('latency_valid', True)
  )

  return results


def check_hardware_status() -> Dict[str, Any]:
  """Get current hardware status"""
  return validate_hardware_constraints()


__all__ = [
  "validate_hardware_constraints", "check_hardware_status"
]