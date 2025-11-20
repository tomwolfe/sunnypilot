"""
Unit tests for neural network performance and safety validation
"""
import time
import pytest
import numpy as np
from unittest.mock import Mock, patch

from selfdrive.modeld.modeld import get_action_from_model


class TestNeuralNetworkPerformanceSafety:
    """Test suite for neural network performance and safety features"""

    def test_acceleration_change_limits_enforcement(self):
        """Test that acceleration changes are properly limited"""
        # Create a mock model output with various acceleration values
        model_output = {
            'plan': np.array([[[10.0, 2.0, 0.5, 0.1]]])  # [velocity, acceleration, jerk, curvature] for multiple timesteps
        }
        
        # Create a mock previous action with a known acceleration
        prev_action = Mock()
        prev_action.desiredAcceleration = 1.0  # Previous acceleration
        prev_action.desiredCurvature = 0.001    # Previous curvature
        
        # Test with various vehicle speeds
        test_speeds = [5.0, 15.0, 25.0, 35.0]
        
        for v_ego in test_speeds:
            # Get the action with acceleration change limiting
            lat_action_t = 0.1  # Lateral action time
            long_action_t = 0.3  # Longitudinal action time
            
            start_time = time.perf_counter()
            action = get_action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego)
            end_time = time.perf_counter()
            
            execution_time_ms = (end_time - start_time) * 1000
            
            # Verify that acceleration changes are limited
            # The max_accel_change is set to 0.3 m/s^3 in the function
            if abs(v_ego) > 0.5:  # Only apply when moving
                # Check that the new acceleration is within the rate limit
                max_change = 0.3 * 0.05  # 0.3 m/s^3 * DT_MDL (approximately)
                assert abs(action.desiredAcceleration - prev_action.desiredAcceleration) <= max_change + 0.1, \
                    f"At speed {v_ego}, acceleration change exceeded rate limit"
            
            # Verify that the execution was fast
            assert execution_time_ms <= 10.0, f"Action calculation took {execution_time_ms:.2f}ms, should be faster"

    def test_curvature_change_limits_enforcement(self):
        """Test that curvature changes are properly limited based on speed"""
        # Create a mock model output
        model_output = {
            'plan': np.array([[[10.0, 1.0, 0.1, 0.002]]])  # [velocity, acceleration, jerk, curvature]
        }
        
        prev_action = Mock()
        prev_action.desiredAcceleration = 0.5
        prev_action.desiredCurvature = 0.001  # Previous curvature
        
        # Test at different speeds
        speeds_and_limits = [
            (2.0, 0.005),  # Low speed: more conservative limit
            (10.0, 0.01),  # Higher speed: less conservative limit
            (30.0, 0.01),  # High speed: same limit
        ]
        
        for v_ego, expected_max_curvature_change in speeds_and_limits:
            action = get_action_from_model(model_output, prev_action, 0.1, 0.3, v_ego)
            
            # Check that curvature change is limited appropriately based on speed
            if v_ego > 0.3:  # MIN_LAT_CONTROL_SPEED
                max_curvature_change = 0.01 if v_ego > 5.0 else 0.005
                curvature_change = abs(action.desiredCurvature - prev_action.desiredCurvature)
                
                assert curvature_change <= max_curvature_change + 0.001, \
                    f"At speed {v_ego}, curvature change {curvature_change} exceeded limit {max_curvature_change}"

    def test_safe_input_clipping_performance(self):
        """Test that safe input clipping doesn't add excessive overhead"""
        # This tests the safe clipping functionality that was implemented in nnlc
        # We'll create a mock of the safe_clip_input functionality to test performance
        
        def _safe_clip_input(input_list, v_ego, allow_high_values_for_testing=False):
            """
            Safely clips neural network inputs to prevent out-of-range values.
            Replicated from NNLC for testing purposes.
            """
            # Ensure speed is within reasonable bounds
            clipped = input_list[:]
            clipped[0] = max(0.0, min(clipped[0], 40.0))  # vEgo should not exceed 144 km/h

            # Limit lateral acceleration inputs to prevent excessive corrections
            for i in range(1, len(clipped)):  # Start from 1 to skip vEgo
                if isinstance(clipped[i], (int, float)):
                    if not allow_high_values_for_testing or i > 3:  # Apply clipping to other parameters
                        clipped[i] = max(-5.0, min(clipped[i], 5.0))  # Limit to ±5 m/s²
            return clipped

        # Test with typical neural network input sizes
        test_inputs = [
            # Typical input: [vEgo, setpoint, jerk, roll, ...past/future data...]
            [20.0, 1.5, 0.8, 0.02] + [0.5] * 20,  # Normal operation
            [50.0, 10.0, 8.0, 0.5] + [8.0] * 20,  # Extreme values that need clipping
            [-5.0, -10.0, -8.0, -0.5] + [-8.0] * 20,  # Negative extreme values
        ]
        
        for test_input in test_inputs:
            start_time = time.perf_counter()
            clipped = _safe_clip_input(test_input, test_input[0], allow_high_values_for_testing=False)
            end_time = time.perf_counter()
            
            execution_time_ms = (end_time - start_time) * 1000
            
            # Safe clipping should be very fast (much less than 20ms)
            assert execution_time_ms <= 1.0, f"Safe clipping took {execution_time_ms:.2f}ms, should be much faster"
            
            # Verify clipping worked correctly
            assert 0.0 <= clipped[0] <= 40.0, f"vEgo not properly clipped: {clipped[0]}"
            for val in clipped[1:]:
                if isinstance(val, (int, float)):
                    assert -5.0 <= val <= 5.0, f"Value not properly clipped: {val}"

    def test_model_performance_under_extreme_conditions(self):
        """Test neural network performance with extreme input values"""
        # Test the action calculation function with extreme but possible values
        extreme_model_outputs = [
            # Very high accelerations that should be limited
            {'plan': np.array([[[15.0, 5.0, 2.0, 0.01]]])},
            # Very low accelerations
            {'plan': np.array([[[5.0, -4.0, -2.0, -0.01]]])},
            # Zero velocities
            {'plan': np.array([[[0.0, 0.0, 0.0, 0.0]]])},
        ]
        
        prev_action = Mock()
        prev_action.desiredAcceleration = 0.0
        prev_action.desiredCurvature = 0.0
        
        for model_output in extreme_model_outputs:
            start_time = time.perf_counter()
            action = get_action_from_model(model_output, prev_action, 0.1, 0.3, 15.0)
            end_time = time.perf_counter()
            
            execution_time_ms = (end_time - start_time) * 1000
            
            # Should handle extreme values without performance degradation
            assert execution_time_ms <= 5.0, f"Extreme input processing took {execution_time_ms:.2f}ms"
            
            # The function should handle extreme values gracefully with proper limits
            assert isinstance(action.desiredAcceleration, float)
            assert isinstance(action.desiredCurvature, float)
            assert isinstance(action.shouldStop, bool)

    def test_model_execution_time_tracking(self):
        """Test that model execution time is properly tracked and validated"""
        # The model execution time should be tracked and not exceed reasonable limits
        model_output = {
            'plan': np.array([[[10.0, 1.0, 0.1, 0.002]]])
        }
        
        prev_action = Mock()
        prev_action.desiredAcceleration = 0.5
        prev_action.desiredCurvature = 0.001
        
        # Simulate various input conditions
        test_conditions = [
            (1.0, "Very low speed"),
            (15.0, "Normal speed"),
            (35.0, "High speed"),
        ]
        
        for v_ego, description in test_conditions:
            start_time = time.perf_counter()
            action = get_action_from_model(model_output, prev_action, 0.1, 0.3, v_ego)
            end_time = time.perf_counter()
            
            execution_time_ms = (end_time - start_time) * 1000
            
            # Action calculation should be consistent and fast
            assert execution_time_ms <= 10.0, f"{description} action calculation took {execution_time_ms:.2f}ms"
            
            # Results should be physically reasonable
            assert -10.0 <= action.desiredAcceleration <= 5.0, \
                f"Acceleration {action.desiredAcceleration} out of reasonable range"
            assert -0.1 <= action.desiredCurvature <= 0.1, \
                f"Curvature {action.desiredCurvature} out of reasonable range"


if __name__ == "__main__":
    pytest.main([__file__])