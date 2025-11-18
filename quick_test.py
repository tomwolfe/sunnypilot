#!/usr/bin/env python3
import sys
sys.path.insert(0, '.')

print("Step 1: Importing metrics module...")
from selfdrive.common.metrics import Metrics, record_metric
print("✓ Metrics module imported")

print("Step 2: Importing hardware monitor...")
from selfdrive.common.hardware_monitor import HardwareMonitor
print("✓ Hardware monitor imported")

print("Step 3: Creating hardware monitor instance...")
hw_monitor = HardwareMonitor(update_interval=0.5)
print(f"✓ Hardware monitor created with update interval: {hw_monitor.update_interval}")

print("Step 4: Getting hardware status...")
status = hw_monitor.get_hardware_status()
print(f"✓ Hardware status retrieved: CPU={status.cpu_percent}%, RAM={status.ram_used_mb}MB")

print("Step 5: All tests completed successfully!")