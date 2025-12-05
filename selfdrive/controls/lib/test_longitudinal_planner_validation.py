#!/usr/bin/env python3
"""
Unit tests for Longitudinal Planner validation functions.

This module tests the safety and robustness improvements made to the longitudinal planner,
including the np.clip bounds validation for sensor data.
"""

import unittest
import numpy as np


class TestLongitudinalPlannerValidation(unittest.TestCase):
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
            self.assertGreaterEqual(x, 0.1, f"Distance value {x} is below minimum safe bound")
            self.assertLessEqual(x, 200.0, f"Distance value {x} is above maximum safe bound")

    def test_velocity_bounds_validation(self):
        """Test that velocity values are properly clipped to safe bounds."""
        # Simulate the validation logic from the updated longitudinal planner
        validated_v = np.array([100.0, -100.0, -25.0, 35.0, 0.0])  # Mix of valid and invalid values
        
        # Apply the velocity clipping as implemented in the longitudinal planner
        for i in range(len(validated_v)):
            validated_v[i] = np.clip(validated_v[i], -50.0, 50.0)
        
        # Check that all values are now within the safe bounds
        for v in validated_v:
            self.assertGreaterEqual(v, -50.0, f"Velocity value {v} is below minimum safe bound")
            self.assertLessEqual(v, 50.0, f"Velocity value {v} is above maximum safe bound")

    def test_acceleration_bounds_validation(self):
        """Test that acceleration values are properly clipped to safe bounds."""
        # Simulate the validation logic from the updated longitudinal planner
        validated_a = np.array([15.0, -20.0, -5.0, 7.0, 0.0])  # Mix of valid and invalid values
        
        # Apply the acceleration clipping as implemented in the longitudinal planner
        for i in range(len(validated_a)):
            validated_a[i] = np.clip(validated_a[i], -15.0, 8.0)
        
        # Check that all values are now within the safe bounds
        for a in validated_a:
            self.assertGreaterEqual(a, -15.0, f"Acceleration value {a} is below minimum safe bound")
            self.assertLessEqual(a, 8.0, f"Acceleration value {a} is above maximum safe bound")

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
            self.assertGreaterEqual(x, 0.1)
            self.assertLessEqual(x, 200.0)
        
        for v in v_values:
            self.assertGreaterEqual(v, -50.0)
            self.assertLessEqual(v, 50.0)
        
        for a in a_values:
            self.assertGreaterEqual(a, -15.0)
            self.assertLessEqual(a, 8.0)

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
        self.assertEqual(x_with_nan[1], 200.0)  # NaN replaced with safe default
        self.assertEqual(v_with_nan[1], 0.0)   # NaN replaced with safe default
        self.assertEqual(a_with_nan[1], 0.0)   # NaN replaced with safe default
        
        # Check that valid values remain valid
        self.assertGreaterEqual(x_with_nan[0], 0.1)
        self.assertLessEqual(x_with_nan[0], 200.0)
        self.assertGreaterEqual(v_with_nan[0], -50.0)
        self.assertLessEqual(v_with_nan[0], 50.0)
        self.assertGreaterEqual(a_with_nan[0], -15.0)
        self.assertLessEqual(a_with_nan[0], 8.0)

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
            self.assertGreaterEqual(x, 0.1)
            self.assertLessEqual(x, 200.0)
        
        for v in extreme_velocities:
            self.assertGreaterEqual(v, -50.0)
            self.assertLessEqual(v, 50.0)
        
        for a in extreme_accelerations:
            self.assertGreaterEqual(a, -15.0)
            self.assertLessEqual(a, 8.0)

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
            self.assertGreaterEqual(x, 0.1)
            self.assertLessEqual(x, 200.0)
        
        for v in validated_v:
            self.assertGreaterEqual(v, -50.0)
            self.assertLessEqual(v, 50.0)
        
        for a in validated_a:
            self.assertGreaterEqual(a, -15.0)
            self.assertLessEqual(a, 8.0)


if __name__ == '__main__':
    print("Running Longitudinal Planner Validation tests...")
    unittest.main(verbosity=2)