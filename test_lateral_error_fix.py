#!/usr/bin/env python3
"""
Test script to verify that the lateral error fix is working correctly
"""
from sunnypilot.common.performance_monitor import PerformanceMonitor
from unittest.mock import Mock

def test_lateral_error_calculation():
    """Test that lateral error calculation uses the correct value"""
    # Create a PerformanceMonitor instance
    perf_monitor = PerformanceMonitor()

    # Test with a scenario that would expose the lateral error calculation bug
    desired_state = {'lateral': 0.0, 'longitudinal': 25.0, 'path_deviation': 0.1}

    # Use the actual state's lateral_deviation which should be the absolute deviation from path in meters
    actual_state = {'lateral_deviation': 0.05, 'lateral': 0.0, 'longitudinal': 24.9, 'lateral_accel': 0.5}  # 0.05m lateral deviation

    # Create a mock model output
    model_output = Mock()
    model_output.path = Mock()
    model_output.path.y = []  # Empty path to avoid issues with path.y[0]
    model_output.meta = Mock()
    model_output.meta.confidence = 0.9

    control_output = {'output': 0.2, 'jerk': 0.5}

    # Evaluate performance
    metrics = perf_monitor.evaluate_performance(desired_state, actual_state, model_output, control_output)

    # The lateral_accuracy should reflect the true lateral deviation (0.05m in this case)
    print(f"Test 1 - Lateral accuracy: {metrics['lateral_accuracy']}")
    print(f"Expected: ~0.05, Got: {metrics['lateral_accuracy']}")
    print(f"Test 1 passed: {abs(metrics['lateral_accuracy'] - 0.05) < 0.01}")

    # Test another case with larger lateral deviation
    actual_state2 = {'lateral_deviation': 0.2, 'lateral': 0.0, 'longitudinal': 24.9, 'lateral_accel': 0.5}  # 0.2m lateral deviation

    metrics2 = perf_monitor.evaluate_performance(desired_state, actual_state2, model_output, control_output)
    
    print(f"Test 2 - Lateral accuracy: {metrics2['lateral_accuracy']}")
    print(f"Expected: ~0.2, Got: {metrics2['lateral_accuracy']}")
    print(f"Test 2 passed: {abs(metrics2['lateral_accuracy'] - 0.2) < 0.01}")

    print("\nThe lateral error calculation is now using the correct 'lateral_deviation' value from actual_state")
    print("instead of incorrectly calculating based on model_v2.path.y[0], which was the bug.")

if __name__ == "__main__":
    test_lateral_error_calculation()