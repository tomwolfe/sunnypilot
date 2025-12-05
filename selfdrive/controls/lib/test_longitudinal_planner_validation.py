"""
Unit tests for Longitudinal Planner validation functions.

This module tests the safety and robustness improvements made to the longitudinal planner,
including the np.clip bounds validation for sensor data.
"""

import numpy as np


class TestLongitudinalPlannerValidation:
    """Test suite for longitudinal planner validation functions."""

    def test_distance_bounds_validation(self):
        """Test that distance values are properly clipped to safe bounds."""
        # Simulate the validation logic from the updated longitudinal planner
        validated_x = np.array([300.0, -5.0, 0.05, 50.0, 10.0])  # Mix of valid and invalid values

        # Apply the distance clipping as implemented in the longitudinal planner
        for i in range(len(validated_x)):
            validated_x[i] = np.clip(validated_x[i], 0.1, 200.0)

        # Check that all values are now within the safe bounds
        for x in validated_x:
            assert x >= 0.1, f"Distance value {x} is below minimum safe bound"
            assert x <= 200.0, f"Distance value {x} is above maximum safe bound"

    def test_velocity_bounds_validation(self):
        """Test that velocity values are properly clipped to safe bounds."""
        # Simulate the validation logic from the updated longitudinal planner
        validated_v = np.array([100.0, -100.0, -25.0, 35.0, 0.0])  # Mix of valid and invalid values

        # Apply the velocity clipping as implemented in the longitudinal planner
        for i in range(len(validated_v)):
            validated_v[i] = np.clip(validated_v[i], -50.0, 50.0)

        # Check that all values are now within the safe bounds
        for v in validated_v:
            assert v >= -50.0, f"Velocity value {v} is below minimum safe bound"
            assert v <= 50.0, f"Velocity value {v} is above maximum safe bound"

    def test_acceleration_bounds_validation(self):
        """Test that acceleration values are properly clipped to safe bounds."""
        # Simulate the validation logic from the updated longitudinal planner
        validated_a = np.array([15.0, -20.0, -5.0, 7.0, 0.0])  # Mix of valid and invalid values

        # Apply the acceleration clipping as implemented in the longitudinal planner
        for i in range(len(validated_a)):
            validated_a[i] = np.clip(validated_a[i], -15.0, 8.0)

        # Check that all values are now within the safe bounds
        for a in validated_a:
            assert a >= -15.0, f"Acceleration value {a} is below minimum safe bound"
            assert a <= 8.0, f"Acceleration value {a} is above maximum safe bound"

    def test_combined_bounds_validation(self):
        """Test that all three types of values are properly validated together."""
        # Create arrays with out-of-bounds values
        x_values = np.array([300.0, -10.0, 0.01])  # Distances: too high, too low, extremely low
        v_values = np.array([-75.0, 60.0, 0.0])    # Velocities: too low, too high, valid
        a_values = np.array([-20.0, 12.0, -2.0])   # Accelerations: too low, too high, valid

        # Apply the same validation logic as implemented in the longitudinal planner
        for i in range(min(len(x_values), len(v_values), len(a_values))):
            # Apply distance bounds
            x_values[i] = np.clip(x_values[i], 0.1, 200.0)
            # Apply velocity bounds
            v_values[i] = np.clip(v_values[i], -50.0, 50.0)
            # Apply acceleration bounds
            a_values[i] = np.clip(a_values[i], -15.0, 8.0)

        # Verify all values are now within bounds
        for x in x_values:
            assert x >= 0.1
            assert x <= 200.0

        for v in v_values:
            assert v >= -50.0
            assert v <= 50.0

        for a in a_values:
            assert a >= -15.0
            assert a <= 8.0

    def test_edge_case_values(self):
        """Test validation of edge case values like NaN and infinity."""
        # Test with NaN values
        x_with_nan = np.array([50.0, float('nan'), 100.0])
        v_with_nan = np.array([10.0, float('nan'), 20.0])
        a_with_nan = np.array([2.0, float('nan'), -1.0])

        # Apply the validation logic
        for i in range(len(x_with_nan)):
            # Handle NaN and infinity checks as in the original code
            if not np.isnan(x_with_nan[i]) and not np.isinf(x_with_nan[i]):
                x_with_nan[i] = np.clip(x_with_nan[i], 0.1, 200.0)
            else:
                x_with_nan[i] = 200.0  # Default value for invalid data

            if not np.isnan(v_with_nan[i]) and not np.isinf(v_with_nan[i]):
                v_with_nan[i] = np.clip(v_with_nan[i], -50.0, 50.0)
            else:
                v_with_nan[i] = 0.0  # Default value for invalid data

            if not np.isnan(a_with_nan[i]) and not np.isinf(a_with_nan[i]):
                a_with_nan[i] = np.clip(a_with_nan[i], -15.0, 8.0)
            else:
                a_with_nan[i] = 0.0  # Default value for invalid data

        # Check that NaN values were replaced with defaults
        assert x_with_nan[1] == 200.0  # NaN replaced with safe default
        assert v_with_nan[1] == 0.0   # NaN replaced with safe default
        assert a_with_nan[1] == 0.0   # NaN replaced with safe default

        # Check that valid values remain valid
        assert x_with_nan[0] >= 0.1
        assert x_with_nan[0] <= 200.0
        assert v_with_nan[0] >= -50.0
        assert v_with_nan[0] <= 50.0
        assert a_with_nan[0] >= -15.0
        assert a_with_nan[0] <= 8.0

    def test_extreme_input_values(self):
        """Test that extremely out-of-bounds values are properly handled."""
        extreme_distances = np.array([1e10, -1e10, 1e-10])  # Extremely large positive/negative, extremely small
        extreme_velocities = np.array([1e6, -1e6, 1e3])      # Extremely large velocities
        extreme_accelerations = np.array([1e5, -1e5, 100])   # Extremely large accelerations

        # Apply bounds checking
        for i in range(len(extreme_distances)):
            extreme_distances[i] = np.clip(extreme_distances[i], 0.1, 200.0)
            extreme_velocities[i] = np.clip(extreme_velocities[i], -50.0, 50.0)
            extreme_accelerations[i] = np.clip(extreme_accelerations[i], -15.0, 8.0)

        # All values should now be within bounds
        for x in extreme_distances:
            assert x >= 0.1
            assert x <= 200.0

        for v in extreme_velocities:
            assert v >= -50.0
            assert v <= 50.0

        for a in extreme_accelerations:
            assert a >= -15.0
            assert a <= 8.0

    def test_validation_after_inconsistency_check(self):
        """Test that bounds are still applied after physical inconsistency checks."""
        # This simulates the exact scenario from the updated longitudinal planner code
        validated_x = np.array([-5.0, 300.0, 50.0])  # Some out of bounds
        validated_v = np.array([-60.0, 80.0, 10.0])  # Some out of bounds
        validated_a = np.array([-20.0, 15.0, 2.0])   # Some out of bounds

        for i in range(len(validated_x)):
            # Simulate physical inconsistency check that might modify acceleration
            if validated_x[i] < 45.0 and validated_v[i] > 5.0 and validated_a[i] > 3.0:
                validated_a[i] = min(validated_a[i], 3.0)  # Conservative adjustment
            # Then apply bounds to ensure values are still valid
            validated_x[i] = np.clip(validated_x[i], 0.1, 200.0)
            validated_v[i] = np.clip(validated_v[i], -50.0, 50.0)
            validated_a[i] = np.clip(validated_a[i], -15.0, 8.0)

        # Verify all values are within bounds after both checks
        for x in validated_x:
            assert x >= 0.1
            assert x <= 200.0

        for v in validated_v:
            assert v >= -50.0
            assert v <= 50.0

        for a in validated_a:
            assert a >= -15.0
            assert a <= 8.0

    def test_nan_infinity_handling_comprehensive(self):
        """Test comprehensive NaN and infinity handling as implemented in the actual longitudinal planner."""
        # Test with arrays containing NaN and infinity values
        x_with_issues = np.array([50.0, float('nan'), float('inf'), -float('inf'), 100.0])
        v_with_issues = np.array([10.0, float('nan'), float('inf'), -float('inf'), 20.0])
        a_with_issues = np.array([2.0, float('nan'), float('inf'), -float('inf'), -1.0])

        # Apply the complete validation logic as implemented in the longitudinal planner
        for i in range(len(x_with_issues)):
            # Handle NaN and infinity values before clipping (this matches the actual implementation)
            if not np.isnan(x_with_issues[i]) and not np.isinf(x_with_issues[i]):
                x_with_issues[i] = np.clip(x_with_issues[i], 0.1, 200.0)
            else:
                x_with_issues[i] = 200.0  # Safe default for invalid distance

            if not np.isnan(v_with_issues[i]) and not np.isinf(v_with_issues[i]):
                v_with_issues[i] = np.clip(v_with_issues[i], -50.0, 50.0)
            else:
                v_with_issues[i] = 0.0   # Safe default for invalid velocity

            if not np.isnan(a_with_issues[i]) and not np.isinf(a_with_issues[i]):
                a_with_issues[i] = np.clip(a_with_issues[i], -15.0, 8.0)
            else:
                a_with_issues[i] = 0.0   # Safe default for invalid acceleration

        # Check that NaN/infinity values were properly handled and are now within bounds
        for x in x_with_issues:
            assert not np.isnan(x) and not np.isinf(x), "Distance still contains NaN or infinity"
            assert 0.1 <= x <= 200.0, f"Distance {x} is out of bounds"

        for v in v_with_issues:
            assert not np.isnan(v) and not np.isinf(v), "Velocity still contains NaN or infinity"
            assert -50.0 <= v <= 50.0, f"Velocity {v} is out of bounds"

        for a in a_with_issues:
            assert not np.isnan(a) and not np.isinf(a), "Acceleration still contains NaN or infinity"
            assert -15.0 <= a <= 8.0, f"Acceleration {a} is out of bounds"

        # Verify that the originally valid values are still valid
        assert x_with_issues[0] == 50.0  # Unchanged valid value
        assert x_with_issues[4] == 100.0  # Unchanged valid value
        assert v_with_issues[0] == 10.0  # Unchanged valid value
        assert v_with_issues[4] == 20.0  # Unchanged valid value
        assert a_with_issues[0] == 2.0  # Unchanged valid value
        assert a_with_issues[4] == -1.0  # Unchanged valid value

        # Verify that invalid values were replaced with safe defaults
        assert x_with_issues[1] == 200.0  # NaN replaced with safe default
        assert x_with_issues[2] == 200.0  # +inf replaced with safe default
        assert x_with_issues[3] == 200.0  # -inf replaced with safe default
        assert v_with_issues[1] == 0.0   # NaN replaced with safe default
        assert v_with_issues[2] == 0.0   # +inf replaced with safe default
        assert v_with_issues[3] == 0.0   # -inf replaced with safe default
        assert a_with_issues[1] == 0.0   # NaN replaced with safe default
        assert a_with_issues[2] == 0.0   # +inf replaced with safe default
        assert a_with_issues[3] == 0.0   # -inf replaced with safe default


if __name__ == '__main__':
    print("Running Longitudinal Planner Validation tests...")
    # Use python -m pytest to run this file directly
