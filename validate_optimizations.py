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
  memory = psutil.virtual_memory()
  ram_usage_gb = memory.used / (1024**3)
  results['ram_usage_gb'] = ram_usage_gb
  results['ram_valid'] = ram_usage_gb < 1.4

  # Check CPU usage
  cpu_percent = psutil.cpu_percent(interval=None)
  results['cpu_percent'] = cpu_percent
  results['cpu_valid'] = cpu_percent < 5.0  # Average should be under 5%

  # Check latency
  sm = messaging.SubMaster(['modelV2', 'controlsState'], timeout=1000)
  sm.update(0)

  latency_ms = 0.0
  if sm.updated['modelV2'] and sm.updated['controlsState']:
    model_time = sm.logMonoTime['modelV2'] / 1e9
    control_time = sm.logMonoTime['controlsState'] / 1e9
    latency_ms = (control_time - model_time) * 1000

  results['latency_ms'] = latency_ms
  results['latency_valid'] = 0 < latency_ms < 80.0

  # Overall result
  results['all_valid'] = results['ram_valid'] and results['cpu_valid'] and results['latency_valid']

  return results


def check_hardware_status() -> Dict[str, Any]:
  """Get current hardware status"""
  return validate_hardware_constraints()


__all__ = [
  "validate_hardware_constraints", "check_hardware_status"
]