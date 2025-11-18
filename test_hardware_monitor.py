#!/usr/bin/env python3
"""
Test script for hardware monitor module to validate functionality.
"""
import time
from selfdrive.common.hardware_monitor import start_hardware_monitoring, stop_hardware_monitoring, get_hardware_metrics
from selfdrive.common.metrics import get_all_metric_summaries

def test_hardware_monitor():
    print("Starting hardware monitoring...")
    start_hardware_monitoring()
    
    # Let it run for a few seconds to collect metrics
    time.sleep(5)
    
    # Print some metrics
    metrics = get_all_metric_summaries()
    for name, summary in metrics.items():
        if 'hardware' in name:
            print(f"{name}: latest={summary.latest:.2f}, avg={summary.avg:.2f}")
    
    current_metrics = get_hardware_metrics()
    print(f"Current CPU: {current_metrics.get('cpu_percent', 0):.2f}%")
    print(f"Current RAM: {current_metrics.get('ram_mb', 0):.2f} MB")
    
    # Stop monitoring
    stop_hardware_monitoring()
    print("Hardware monitoring stopped.")

if __name__ == "__main__":
    test_hardware_monitor()