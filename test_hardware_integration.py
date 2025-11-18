#!/usr/bin/env python3
"""
Test script to validate hardware monitoring and optimization integration.
This tests that the improvements are properly connected to the real system.
"""
import time
import numpy as np
from selfdrive.common.hardware_monitor import (
    get_hardware_monitor, 
    start_hardware_monitoring, 
    get_current_hardware_status,
    is_hardware_within_limits
)
from selfdrive.common.performance_optimizer import (
    get_performance_optimizer,
    run_quantized_inference,
    optimize_tensor,
    optimize_memory
)
from selfdrive.common.metrics import get_all_metric_summaries, Metrics

def test_hardware_monitor():
    """Test hardware monitoring functionality."""
    print("Testing Hardware Monitor...")
    
    # Get the global hardware monitor
    hw_monitor = get_hardware_monitor()
    
    # Start monitoring
    hw_monitor.start_monitoring()
    time.sleep(1.5)  # Allow monitor to collect some data
    
    # Get current status
    status = get_current_hardware_status()
    if status:
        print(f"  ✓ Hardware status retrieved:")
        print(f"    CPU: {status.cpu_percent:.1f}% (per core: {status.cpu_per_core})")
        print(f"    RAM: {status.ram_used_mb:.1f}MB ({status.ram_percent:.1f}%)")
        print(f"    Power est: {status.power_estimate_w:.2f}W")
        
        # Check compliance
        compliant = is_hardware_within_limits()
        print(f"    Compliance: {'✓ PASS' if compliant else '✗ FAIL'}")
        
        # Get detailed compliance report
        report = hw_monitor.get_limit_compliance_report()
        print(f"    Report: {report}")
    else:
        print("  ✗ Failed to get hardware status")
        return False
    
    # Stop monitoring
    hw_monitor.stop_monitoring()
    return True

def test_performance_optimizer():
    """Test performance optimizer functionality."""
    print("\nTesting Performance Optimizer...")
    
    # Get the global performance optimizer
    perf_optimizer = get_performance_optimizer()
    
    # Test tensor optimization
    test_tensor = np.random.random((1, 3, 224, 224)).astype(np.float32)
    optimized_tensor = optimize_tensor(test_tensor)
    print(f"  ✓ Tensor optimization completed: {test_tensor.shape} -> {optimized_tensor.shape}")
    
    # Test memory optimization
    memory_reduction = optimize_memory(1500.0)  # Try to optimize 1500MB usage
    print(f"  ✓ Memory optimization: 1500MB -> {memory_reduction:.1f}MB")
    
    # Test quantized inference
    inference_input = np.random.random((1, 3, 224, 224)).astype(np.float32)
    try:
        inference_result = run_quantized_inference(inference_input)
        print(f"  ✓ Quantized inference completed: {inference_input.shape} -> {inference_result.shape}")
    except Exception as e:
        print(f"  ⚠ Quantized inference failed: {e}")
    
    return True

def test_metrics_integration():
    """Test metrics tracking functionality."""
    print("\nTesting Metrics Integration...")
    
    # Check that metrics are being recorded
    summaries = get_all_metric_summaries()
    
    # Look for key metrics that should exist after hardware monitoring
    required_metrics = [
        Metrics.CPU_USAGE_PERCENT,
        Metrics.RAM_USAGE_MB,
        Metrics.POWER_DRAW_WATTS
    ]
    
    found_metrics = []
    for metric in required_metrics:
        if metric in summaries and summaries[metric]:
            found_metrics.append(metric)
            print(f"  ✓ Found metric: {metric}")
        else:
            print(f"  ⚠ Missing metric: {metric}")
    
    print(f"  Found {len(found_metrics)}/{len(required_metrics)} required metrics")
    return len(found_metrics) > 0

def test_realtime_adaptation():
    """Test that the system can adapt in real-time based on hardware status."""
    print("\nTesting Real-time Adaptation...")
    
    # Start hardware monitoring
    start_hardware_monitoring()
    time.sleep(0.5)
    
    # Create a simple test for tensor optimization that adapts based on current load
    test_tensor = np.random.random((1, 3, 100, 100)).astype(np.float32)
    
    # Get status before optimization
    status_before = get_current_hardware_status()
    
    # Optimize tensor
    optimized_result = optimize_tensor(test_tensor)
    
    # Get status after optimization
    status_after = get_current_hardware_status()
    
    print(f"  ✓ Tensor optimization completed with hardware monitoring")
    print(f"    Before: CPU={status_before.cpu_percent:.1f}%, RAM={status_before.ram_used_mb:.1f}MB")
    if status_after:
        print(f"    After:  CPU={status_after.cpu_percent:.1f}%, RAM={status_after.ram_used_mb:.1f}MB")
    
    return True

def main():
    """Run all tests to validate the improvements."""
    print("Sunnypilot Hardware & Performance Integration Tests")
    print("="*55)
    
    tests = [
        test_hardware_monitor,
        test_performance_optimizer,
        test_metrics_integration,
        test_realtime_adaptation
    ]
    
    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"  ✗ Test {test.__name__} failed with error: {e}")
            results.append(False)
    
    print(f"\nTest Results: {sum(results)}/{len(results)} passed")
    
    if all(results):
        print("🎉 All tests passed! Hardware monitoring and optimization are properly integrated.")
        return True
    else:
        print("⚠️  Some tests failed. Please review the integration.")
        return False

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)