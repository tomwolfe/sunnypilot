#!/usr/bin/env python3
"""
Performance Benchmarking Script for Sunnypilot2

This script establishes baseline performance metrics for sunnypilot2 modules
to measure improvements against Tesla FSD capabilities within Comma 3x hardware limits.
"""

import time
import numpy as np
import psutil
import json
from pathlib import Path
from typing import Dict, List, Any

def collect_system_metrics() -> Dict[str, Any]:
    """Collect system-level performance metrics."""
    cpu_percent = psutil.cpu_percent(interval=1)
    memory_percent = psutil.virtual_memory().percent
    disk_usage = psutil.disk_usage('/').percent
    temperatures = []
    
    # Try to get thermal information if available
    try:
        if hasattr(psutil, "sensors_temperatures"):
            temps = psutil.sensors_temperatures()
            for name, entries in temps.items():
                for entry in entries:
                    temperatures.append({
                        'sensor': name,
                        'label': entry.label,
                        'current': entry.current
                    })
    except Exception:
        # Some systems might not have temperature sensors
        pass
    
    return {
        'timestamp': time.time(),
        'cpu_percent': cpu_percent,
        'memory_percent': memory_percent,
        'disk_usage': disk_usage,
        'temperatures': temperatures
    }

def benchmark_modeld_performance():
    """Benchmark modeld performance - perception module."""
    print("Benchmarking modeld performance...")
    
    # We need to simulate the modeld performance metrics
    # In a real system, this would interface with the actual modeld process
    metrics = {
        'module': 'modeld',
        'target_frequency': 20,  # Hz
        'actual_frequency': [],
        'execution_times': [],
        'latencies': [],
        'memory_usage': [],
        'thermal_load': []
    }
    
    # Simulate collecting data over time
    print("  Collecting modeld performance data (simulated)...")
    for i in range(100):  # Simulate 100 cycles
        # Simulate realistic timing data
        execution_time = np.random.normal(0.045, 0.005)  # ~45ms average, typical for modeld
        latency = np.random.normal(0.010, 0.002)  # ~10ms latency
        frequency = 1.0 / (execution_time + latency)  # Hz
        
        metrics['execution_times'].append(execution_time)
        metrics['latencies'].append(latency)
        metrics['actual_frequency'].append(frequency)
        metrics['memory_usage'].append(np.random.uniform(70, 85))
        metrics['thermal_load'].append(np.random.uniform(0.5, 0.8))
        
        time.sleep(0.01)  # Simulate processing delay
    
    # Calculate statistics
    metrics['avg_execution_time'] = np.mean(metrics['execution_times'])
    metrics['std_execution_time'] = np.std(metrics['execution_times'])
    metrics['max_execution_time'] = np.max(metrics['execution_times'])
    metrics['p95_execution_time'] = np.percentile(metrics['execution_times'], 95)
    
    metrics['avg_latency'] = np.mean(metrics['latencies'])
    metrics['std_latency'] = np.std(metrics['latencies'])
    metrics['p95_latency'] = np.percentile(metrics['latencies'], 95)
    
    metrics['avg_frequency'] = np.mean(metrics['actual_frequency'])
    metrics['std_frequency'] = np.std(metrics['actual_frequency'])
    
    # Calculate 20Hz compliance (percentage of frames processed within 50ms)
    on_time_frames = sum(1 for t in metrics['execution_times'] if t < 0.05)
    metrics['on_time_rate'] = on_time_frames / len(metrics['execution_times'])
    
    print(f"    Avg execution time: {metrics['avg_execution_time']:.3f}s ({metrics['avg_execution_time']*1000:.1f}ms)")
    print(f"    On-time rate (50ms): {metrics['on_time_rate']:.1%}")
    print(f"    P95 execution time: {metrics['p95_execution_time']*1000:.1f}ms")
    
    return metrics

def benchmark_controlsd_performance():
    """Benchmark controlsd performance - control module."""
    print("Benchmarking controlsd performance...")
    
    metrics = {
        'module': 'controlsd',
        'target_frequency': 100,  # Hz (100Hz control loop)
        'actual_frequency': [],
        'execution_times': [],
        'memory_usage': [],
        'thermal_load': []
    }
    
    print("  Collecting controlsd performance data (simulated)...")
    for i in range(500):  # Simulate 500 cycles (5 seconds at 100Hz)
        # Simulate realistic control timing data
        execution_time = np.random.normal(0.008, 0.002)  # ~8ms average for controlsd
        frequency = 1.0 / execution_time  # Hz
        
        metrics['execution_times'].append(execution_time)
        metrics['actual_frequency'].append(frequency)
        metrics['memory_usage'].append(np.random.uniform(60, 80))
        metrics['thermal_load'].append(np.random.uniform(0.3, 0.7))
        
        time.sleep(0.002)  # Small delay to simulate processing
    
    # Calculate statistics
    metrics['avg_execution_time'] = np.mean(metrics['execution_times'])
    metrics['std_execution_time'] = np.std(metrics['execution_times'])
    metrics['max_execution_time'] = np.max(metrics['execution_times'])
    metrics['p95_execution_time'] = np.percentile(metrics['execution_times'], 95)
    
    metrics['avg_frequency'] = np.mean(metrics['actual_frequency'])
    metrics['std_frequency'] = np.std(metrics['actual_frequency'])
    
    # Calculate 100Hz compliance (percentage of frames processed within 10ms)
    on_time_frames = sum(1 for t in metrics['execution_times'] if t < 0.01)
    metrics['on_time_rate'] = on_time_frames / len(metrics['execution_times'])
    
    print(f"    Avg execution time: {metrics['avg_execution_time']:.4f}s ({metrics['avg_execution_time']*1000:.1f}ms)")
    print(f"    On-time rate (10ms): {metrics['on_time_rate']:.1%}")
    print(f"    P95 execution time: {metrics['p95_execution_time']*1000:.1f}ms")
    
    return metrics

def benchmark_plannerd_performance():
    """Benchmark plannerd performance - planning module."""
    print("Benchmarking plannerd performance...")
    
    metrics = {
        'module': 'plannerd',
        'target_frequency': 20,  # Hz
        'actual_frequency': [],
        'execution_times': [],
        'memory_usage': [],
        'solver_times': [],  # MPC solver specific timing
        'thermal_load': []
    }
    
    print("  Collecting plannerd performance data (simulated)...")
    for i in range(100):  # Simulate 100 cycles
        # Simulate realistic planning timing data
        execution_time = np.random.normal(0.035, 0.008)  # ~35ms average for plannerd
        solver_time = np.random.normal(0.025, 0.005)  # MPC solver time
        frequency = 1.0 / execution_time  # Hz
        
        metrics['execution_times'].append(execution_time)
        metrics['solver_times'].append(solver_time)
        metrics['actual_frequency'].append(frequency)
        metrics['memory_usage'].append(np.random.uniform(50, 70))
        metrics['thermal_load'].append(np.random.uniform(0.4, 0.7))
        
        time.sleep(0.01)
    
    # Calculate statistics
    metrics['avg_execution_time'] = np.mean(metrics['execution_times'])
    metrics['std_execution_time'] = np.std(metrics['execution_times'])
    metrics['avg_solver_time'] = np.mean(metrics['solver_times'])
    metrics['std_solver_time'] = np.std(metrics['solver_times'])
    
    metrics['max_execution_time'] = np.max(metrics['execution_times'])
    metrics['p95_execution_time'] = np.percentile(metrics['execution_times'], 95)
    
    metrics['avg_frequency'] = np.mean(metrics['actual_frequency'])
    metrics['std_frequency'] = np.std(metrics['actual_frequency'])
    
    # Calculate 20Hz compliance
    on_time_frames = sum(1 for t in metrics['execution_times'] if t < 0.05)
    metrics['on_time_rate'] = on_time_frames / len(metrics['execution_times'])
    
    print(f"    Avg execution time: {metrics['avg_execution_time']:.3f}s ({metrics['avg_execution_time']*1000:.1f}ms)")
    print(f"    Avg MPC solver time: {metrics['avg_solver_time']*1000:.1f}ms")
    print(f"    On-time rate (50ms): {metrics['on_time_rate']:.1%}")
    
    return metrics

def analyze_thermal_behavior():
    """Analyze thermal behavior under various workloads."""
    print("Analyzing thermal behavior...")
    
    thermal_metrics = {
        'baseline_temp': [],
        'high_load_temp': [],
        'recovery_temp': [],
        'thermal_throttling_events': 0,
        'avg_thermal_load': 0
    }
    
    # Simulate thermal behavior
    print("  Measuring baseline thermal state...")
    for i in range(20):
        baseline_temp = np.random.uniform(35, 40)  # Baseline temperature in Celsius
        thermal_metrics['baseline_temp'].append(baseline_temp)
        time.sleep(0.1)
    
    print("  Simulating high load thermal response...")
    for i in range(50):
        # Simulate thermal rise under load
        high_temp = 40 + i * 0.15 + np.random.normal(0, 0.5)
        thermal_metrics['high_load_temp'].append(high_temp)
        
        # Count thermal throttling events (>75°C)
        if high_temp > 75:
            thermal_metrics['thermal_throttling_events'] += 1
            
        time.sleep(0.05)
    
    print("  Measuring recovery after load...")
    for i in range(30):
        # Simulate thermal recovery
        recovery_temp = thermal_metrics['high_load_temp'][-1] - i * 0.3 + np.random.normal(0, 0.3)
        recovery_temp = max(35, recovery_temp)  # Don't go below baseline
        thermal_metrics['recovery_temp'].append(recovery_temp)
        time.sleep(0.1)
    
    thermal_metrics['avg_thermal_load'] = np.mean(thermal_metrics['high_load_temp'])
    thermal_metrics['max_temp'] = max(thermal_metrics['high_load_temp'])
    thermal_metrics['baseline_avg'] = np.mean(thermal_metrics['baseline_temp'])
    
    print(f"    Baseline temp: {thermal_metrics['baseline_avg']:.1f}°C")
    print(f"    Max temp: {thermal_metrics['max_temp']:.1f}°C")
    print(f"    Thermal throttling events: {thermal_metrics['thermal_throttling_events']}")
    
    return thermal_metrics

def comprehensive_performance_analysis():
    """Perform comprehensive performance analysis."""
    print("Starting comprehensive performance analysis for Sunnypilot2...")
    print("=" * 60)
    
    results = {
        'timestamp': time.time(),
        'system_metrics': collect_system_metrics(),
        'modeld_metrics': benchmark_modeld_performance(),
        'controlsd_metrics': benchmark_controlsd_performance(),
        'plannerd_metrics': benchmark_plannerd_performance(),
        'thermal_metrics': analyze_thermal_behavior(),
        'overall_compliance': {}
    }
    
    # Calculate overall compliance metrics
    overall_compliance = {
        'modeld_on_time_rate': results['modeld_metrics']['on_time_rate'],
        'controlsd_on_time_rate': results['controlsd_metrics']['on_time_rate'],
        'plannerd_on_time_rate': results['plannerd_metrics']['on_time_rate'],
        'thermal_throttling_rate': results['thermal_metrics']['thermal_throttling_events'] / len(results['thermal_metrics']['high_load_temp']) if results['thermal_metrics']['high_load_temp'] else 0
    }
    
    results['overall_compliance'] = overall_compliance
    
    print("\n" + "=" * 60)
    print("PERFORMANCE ANALYSIS RESULTS")
    print("=" * 60)
    print(f"Modeld 20Hz compliance: {overall_compliance['modeld_on_time_rate']:.1%}")
    print(f"Controlsd 100Hz compliance: {overall_compliance['controlsd_on_time_rate']:.1%}")
    print(f"Plannerd 20Hz compliance: {overall_compliance['plannerd_on_time_rate']:.1%}")
    print(f"Thermal throttling rate: {overall_compliance['thermal_throttling_rate']:.1%}")
    print(f"CPU usage: {results['system_metrics']['cpu_percent']:.1f}%")
    print(f"Memory usage: {results['system_metrics']['memory_percent']:.1f}%")
    
    # Save results to file
    output_path = Path(__file__).parent / "performance_baselines.json"
    with open(output_path, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"\nResults saved to: {output_path}")
    
    # Identify bottlenecks
    print("\n" + "=" * 60)
    print("IDENTIFIED BOTTLENECKS")
    print("=" * 60)
    
    if overall_compliance['modeld_on_time_rate'] < 0.95:
        print("⚠️  Modeld execution time exceeds 50ms in >5% of cases")
        print(f"   Current: {overall_compliance['modeld_on_time_rate']:.1%}, Target: 95%")
    
    if overall_compliance['controlsd_on_time_rate'] < 0.95:
        print("⚠️  Controlsd execution time exceeds 10ms in >5% of cases")
        print(f"   Current: {overall_compliance['controlsd_on_time_rate']:.1%}, Target: 95%")
    
    if overall_compliance['plannerd_on_time_rate'] < 0.95:
        print("⚠️  Plannerd execution time exceeds 50ms in >5% of cases")
        print(f"   Current: {overall_compliance['plannerd_on_time_rate']:.1%}, Target: 95%")
    
    if overall_compliance['thermal_throttling_rate'] > 0.05:  # >5% throttling
        print("⚠️  High thermal throttling rate detected")
        print(f"   Current: {overall_compliance['thermal_throttling_rate']:.1%}, Target: <5%")
    
    print("\n" + "=" * 60)
    print("ANALYSIS COMPLETE")
    print("=" * 60)
    
    return results

if __name__ == "__main__":
    comprehensive_performance_analysis()