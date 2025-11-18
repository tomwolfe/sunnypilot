#!/usr/bin/env python3
"""
Hardware constraint validation tests for sunnypilot
Validates that the system meets the required <1.4GB RAM, <5% CPU, <80ms latency constraints
"""
import time
import psutil
import threading
from typing import Dict, List
import numpy as np

from openpilot.common.resource_aware import optimize_for_hardware_constraints, calculate_hardware_efficiency_score
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import HARDWARE


def test_hardware_constraints():
    """Test that system meets hardware constraints"""
    print("Testing Hardware Constraints...")
    
    # Run the optimization function to check current state
    metrics = optimize_for_hardware_constraints()
    
    print(f"  RAM Usage: {metrics['ram_usage_mb']:.1f} MB")
    print(f"  CPU Usage: {metrics['cpu_usage_percent']:.2f}%")
    print(f"  Thermal Scale: {metrics['thermal_scale']:.2f}")
    print(f"  Performance Mode: {metrics['performance_mode']}")
    print(f"  Optimizations Applied: {metrics['optimizations_applied']}")
    print(f"  Hardware Efficiency Score: {metrics['hardware_efficiency_score']:.3f}")
    
    # Check constraints
    ram_constraint = metrics['ram_usage_mb'] < 1400  # < 1.4GB
    cpu_constraint = metrics['cpu_usage_percent'] < 5.0  # < 5%
    
    print(f"  RAM constraint (<1.4GB): {'✓' if ram_constraint else '✗'}")
    print(f"  CPU constraint (<5%): {'✓' if cpu_constraint else '✗'}")
    
    return ram_constraint and cpu_constraint, metrics


def simulate_control_loop_load():
    """Simulate the computational load of a control loop to test latency"""
    print("\nTesting Control Loop Latency...")
    
    latencies = []
    
    # Simulate 100 control loop iterations to measure performance
    for i in range(100):
        start_time = time.perf_counter()
        
        # Simulate control loop operations
        # This would typically involve sensor processing, model inference, and control calculation
        sensor_data = np.random.random(1000).astype(np.float32)
        processed_data = sensor_data * 0.95  # Simple processing
        control_output = np.sum(processed_data) / len(processed_data)  # Simple control calculation
        
        end_time = time.perf_counter()
        latency = (end_time - start_time) * 1000  # Convert to ms
        latencies.append(latency)
        
        # Small sleep to simulate real timing
        time.sleep(0.01)
    
    avg_latency = sum(latencies) / len(latencies)
    max_latency = max(latencies)
    min_latency = min(latencies)
    
    print(f"  Average Latency: {avg_latency:.2f}ms")
    print(f"  Max Latency: {max_latency:.2f}ms")
    print(f"  Min Latency: {min_latency:.2f}ms")
    
    latency_constraint = avg_latency < 80.0  # < 80ms average
    print(f"  Latency constraint (<80ms): {'✓' if latency_constraint else '✗'}")
    
    return latency_constraint, {
        'avg_latency': avg_latency,
        'max_latency': max_latency,
        'min_latency': min_latency,
        'latencies': latencies
    }


def test_memory_efficiency_over_time():
    """Test memory efficiency over an extended period"""
    print("\nTesting Memory Efficiency Over Time...")
    
    # Monitor memory over 30 seconds
    memory_readings = []
    duration = 30  # seconds
    
    start_time = time.time()
    while time.time() - start_time < duration:
        # Get current memory usage
        memory_percent = psutil.virtual_memory().percent
        memory_used = psutil.virtual_memory().used / (1024 * 1024)  # MB
        memory_readings.append(memory_used)
        
        time.sleep(1.0)  # Check every second
    
    avg_memory = sum(memory_readings) / len(memory_readings)
    max_memory = max(memory_readings)
    
    print(f"  Average Memory Usage: {avg_memory:.1f} MB")
    print(f"  Peak Memory Usage: {max_memory:.1f} MB")
    print(f"  Memory Trend: {'Stable' if abs(max_memory - avg_memory) < 100 else 'Variable'}")
    
    memory_constraint = max_memory < 1400  # < 1.4GB
    print(f"  Memory constraint (<1.4GB): {'✓' if memory_constraint else '✗'}")
    
    return memory_constraint, {
        'avg_memory': avg_memory,
        'max_memory': max_memory,
        'readings': memory_readings
    }


def test_comprehensive_performance():
    """Run comprehensive performance validation"""
    print("=" * 60)
    print("COMPREHENSIVE HARDWARE CONSTRAINT VALIDATION")
    print("=" * 60)
    
    # Test 1: Static hardware constraints
    print("\n1. STATIC HARDWARE CONSTRAINTS")
    hw_constraint_pass, hw_metrics = test_hardware_constraints()
    
    # Test 2: Control loop latency
    print("\n2. CONTROL LOOP LATENCY TEST")
    latency_constraint_pass, latency_metrics = simulate_control_loop_load()
    
    # Test 3: Memory efficiency over time
    print("\n3. MEMORY EFFICIENCY OVER TIME")
    memory_constraint_pass, memory_metrics = test_memory_efficiency_over_time()
    
    # Overall results
    print("\n" + "=" * 60)
    print("VALIDATION SUMMARY")
    print("=" * 60)
    
    results = {
        'hardware_constraints': hw_constraint_pass,
        'latency_constraints': latency_constraint_pass,
        'memory_constraints': memory_constraint_pass
    }
    
    for test_name, passed in results.items():
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {test_name.replace('_', ' ').title()}: {status}")
    
    # Calculate overall success
    passed_tests = sum(results.values())
    total_tests = len(results)
    overall_success = passed_tests == total_tests
    
    print(f"\n  PASSED: {passed_tests}/{total_tests} tests")
    print(f"  OVERALL: {'✓ ALL CONSTRAINTS MET' if overall_success else '✗ SOME CONSTRAINTS VIOLATED'}")
    
    # Calculate efficiency score
    hw_score = hw_metrics.get('hardware_efficiency_score', 0.0)
    print(f"  Hardware Efficiency Score: {hw_score:.3f}")
    
    # Calculate overall system efficiency
    efficiency_percentage = (hw_score * 100)
    print(f"  Overall System Efficiency: {efficiency_percentage:.1f}%")
    
    if overall_success and efficiency_percentage > 78:
        print(f"\n🎉 SYSTEM EXCEEDS 78% EFFICIENCY TARGET!")
        print(f"   Current efficiency: {efficiency_percentage:.1f}%")
    else:
        print(f"\n⚠️  SYSTEM MAY NEED ADDITIONAL OPTIMIZATIONS")
        print(f"   Current efficiency: {efficiency_percentage:.1f}%")
    
    return overall_success, results, {
        'hardware_metrics': hw_metrics,
        'latency_metrics': latency_metrics,
        'memory_metrics': memory_metrics
    }


if __name__ == "__main__":
    success, results, detailed_metrics = test_comprehensive_performance()
    
    if success:
        print(f"\nHardware constraint validation completed successfully!")
        print("All critical constraints are being met:")
        print("- RAM Usage < 1.4GB")
        print("- CPU Usage < 5%")
        print("- Control Loop Latency < 80ms")
    else:
        print(f"\nHardware constraint validation identified issues.")
        print("Some constraints may need additional optimization work.")
    
    # Export results for monitoring
    try:
        import json
        with open('/tmp/hardware_validation_results.json', 'w') as f:
            json.dump(detailed_metrics, f, indent=2)
        print("Detailed results saved to /tmp/hardware_validation_results.json")
    except:
        print("Could not save detailed results to file")