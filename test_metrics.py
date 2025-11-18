#!/usr/bin/env python3
"""
Test script for metrics module to validate functionality.
"""
from selfdrive.common.metrics import Metrics, record_metric, get_metric_summary, get_all_metric_summaries, export_metrics
import time
import random

def test_metrics():
    print("Testing metrics module...")
    
    # Record some test metrics
    for i in range(100):
        # Simulate various metrics
        record_metric(Metrics.PERCEPTION_LATENCY_MS, random.uniform(20, 80), {"frame_id": i})
        record_metric(Metrics.CPU_USAGE_PERCENT, random.uniform(15, 45), {"core": i % 4})
        record_metric(Metrics.RAM_USAGE_MB, random.uniform(500, 1200), {"timestamp": time.time()})
        time.sleep(0.01)  # Small delay to create different timestamps
    
    # Get summaries
    latency_summary = get_metric_summary(Metrics.PERCEPTION_LATENCY_MS)
    if latency_summary:
        print(f"Perception latency summary: count={latency_summary.count}, avg={latency_summary.avg:.2f}ms, min={latency_summary.min:.2f}ms, max={latency_summary.max:.2f}ms")
    
    cpu_summary = get_metric_summary(Metrics.CPU_USAGE_PERCENT)
    if cpu_summary:
        print(f"CPU usage summary: count={cpu_summary.count}, avg={cpu_summary.avg:.2f}%, min={cpu_summary.min:.2f}%, max={cpu_summary.max:.2f}%")
    
    all_summaries = get_all_metric_summaries()
    print(f"Total metrics tracked: {len(all_summaries)}")
    
    # Export to file
    export_metrics("/tmp/test_metrics.json")
    print("Metrics exported to /tmp/test_metrics.json")

if __name__ == "__main__":
    test_metrics()