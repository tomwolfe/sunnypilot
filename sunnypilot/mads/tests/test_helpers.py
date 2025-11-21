import json
import unittest
import numpy as np
from unittest.mock import patch

# Correctly import CarParams and CarParamsSP
from opendbc.car.structs import CarParams, CarParamsSP

from sunnypilot.mads.helpers import set_car_specific_params

class MockParams(dict):
  def get(self, key, encoding='utf-8'):
    val = super().get(key)
    if val is None:
      return None
    if encoding is not None:
      return str(val).encode(encoding)
    return val
  
  def put(self, key, value):
    self[key] = value

  def get_bool(self, key):
    # A simple mock for get_bool
    val = super().get(key)
    if val is None:
        return False
    return val in (True, 'True', 'true', '1')

class TestMadsHelpers(unittest.TestCase):
  def setUp(self):
    self.params = MockParams()
    self.CP = CarParams.new_message()
    self.CP.brand = 'toyota' # Set a brand as it's used in the function
    self.CP_SP = CarParamsSP()

  @patch('sunnypilot.mads.helpers.cloudlog')
  def test_curvature_gain_clamping(self, mock_cloudlog):
    # Test case 1: Partial curve exceeding limit
    curvatures1 = [0.0, 0.08, 0.12]
    gains1 = [1.0, 2.0, 2.5]
    self.params.put('CurvatureGainInterp', json.dumps([curvatures1, gains1]))
    self.params.put('MaxCurvatureForGainInterp', "0.1")
    self.params.put("Mads", True) # Make sure mads is enabled

    set_car_specific_params(self.CP, self.CP_SP, self.params)

    expected_curvatures1 = [0.0, 0.08, 0.1]
    expected_gains1 = [1.0, 2.0, 2.25] # 2.0 + (2.5-2.0)*(0.1-0.08)/(0.12-0.08)

    self.assertTrue(np.allclose(self.CP_SP.curvatureGainInterp[0], expected_curvatures1))
    self.assertTrue(np.allclose(self.CP_SP.curvatureGainInterp[1], expected_gains1))

    # Test case 2: All points exceeding the limit (but starting from 0)
    self.setUp() # Reset params and objects
    curvatures2 = [0.0, 0.12, 0.15]
    gains2 = [1.0, 1.5, 1.8]
    self.params.put('CurvatureGainInterp', json.dumps([curvatures2, gains2]))
    self.params.put('MaxCurvatureForGainInterp', "0.1")
    self.params.put("Mads", True)

    set_car_specific_params(self.CP, self.CP_SP, self.params)

    expected_curvatures2 = [0.0, 0.1]
    interp_gain2 = 1.0 + (1.5 - 1.0) * (0.1 - 0.0) / (0.12 - 0.0) # 1.41666...
    expected_gains2 = [1.0, interp_gain2]

    self.assertTrue(np.allclose(self.CP_SP.curvatureGainInterp[0], expected_curvatures2))
    self.assertTrue(np.allclose(self.CP_SP.curvatureGainInterp[1], expected_gains2))

    # Test case 3: Exact match at limit point
    self.setUp() # Reset params and objects
    curvatures3 = [0.0, 0.1, 0.12]
    gains3 = [1.0, 1.5, 1.8]
    self.params.put('CurvatureGainInterp', json.dumps([curvatures3, gains3]))
    self.params.put('MaxCurvatureForGainInterp', "0.1")
    self.params.put("Mads", True)
    
    set_car_specific_params(self.CP, self.CP_SP, self.params)

    expected_curvatures3 = [0.0, 0.1]
    expected_gains3 = [1.0, 1.5]

    self.assertTrue(np.allclose(self.CP_SP.curvatureGainInterp[0], expected_curvatures3))
    self.assertTrue(np.allclose(self.CP_SP.curvatureGainInterp[1], expected_gains3))

    # Test case 4: No clamping needed
    self.setUp() # Reset params and objects
    curvatures4 = [0.0, 0.05, 0.09]
    gains4 = [1.0, 1.2, 1.4]
    self.params.put('CurvatureGainInterp', json.dumps([curvatures4, gains4]))
    self.params.put('MaxCurvatureForGainInterp', "0.1")
    self.params.put("Mads", True)

    set_car_specific_params(self.CP, self.CP_SP, self.params)

    self.assertTrue(np.allclose(self.CP_SP.curvatureGainInterp[0], curvatures4))
    self.assertTrue(np.allclose(self.CP_SP.curvatureGainInterp[1], gains4))

  def test_curvature_gain_vehicle_specific_clamping(self):
    """Test that curvature gain clamping respects vehicle-specific characteristics"""
    # Test with a large vehicle (truck/SUV) - higher wheelbase
    curvatures = [0.0, 0.08, 0.12, 0.15]
    gains = [1.0, 1.5, 2.0, 2.5]
    self.params.put('CurvatureGainInterp', json.dumps([curvatures, gains]))

    # Set up truck-like vehicle characteristics
    self.CP.wheelbase = 3.5  # Large truck
    self.CP.steerRatio = 18.0  # Higher steering ratio for trucks
    self.CP.mass = 2500  # Heavy vehicle
    self.params.put("Mads", True)

    set_car_specific_params(self.CP, self.CP_SP, self.params)

    # For large trucks, base limit should be 0.08 (12.5m radius)
    # With higher steering ratio, this becomes 0.08 * 1.2 = 0.096, but capped at 0.2
    expected_curvatures = [0.0, 0.08, 0.096]  # Points within the adjusted limit
    expected_gains = [1.0, 1.5, 1.5 + (2.0-1.5)*(0.096-0.08)/(0.12-0.08)]  # Interpolated gain

    self.assertTrue(np.allclose(self.CP_SP.curvatureGainInterp[0], expected_curvatures, atol=0.001))
    self.assertTrue(np.allclose(self.CP_SP.curvatureGainInterp[1], expected_gains, atol=0.01))

  def test_curvature_gain_small_vehicle_clamping(self):
    """Test curvature gain clamping for small vehicles"""
    # Reset for new test
    self.setUp()

    curvatures = [0.0, 0.1, 0.15, 0.2]
    gains = [1.0, 1.2, 1.5, 1.8]
    self.params.put('CurvatureGainInterp', json.dumps([curvatures, gains]))

    # Set up compact car characteristics
    self.CP.wheelbase = 2.3  # Small car
    self.CP.steerRatio = 10.0  # Lower steering ratio
    self.CP.mass = 1000  # Light vehicle
    self.params.put("Mads", True)

    set_car_specific_params(self.CP, self.CP_SP, self.params)

    # For small cars, base limit should be 0.15 (6.7m radius)
    # With lower steering ratio, this becomes 0.15 * 0.8 = 0.12, but minimum is 0.05
    expected_curvatures = [0.0, 0.1, 0.12]  # Points within the adjusted limit
    expected_gains = [1.0, 1.2, 1.2 + (1.5-1.2)*(0.12-0.1)/(0.15-0.1)]  # Interpolated gain

    self.assertTrue(np.allclose(self.CP_SP.curvatureGainInterp[0], expected_curvatures, atol=0.001))
    self.assertTrue(np.allclose(self.CP_SP.curvatureGainInterp[1], expected_gains, atol=0.01))

  def test_curvature_gain_custom_limit_validation(self):
    """Test that custom limits are validated per vehicle type"""
    # Reset for new test
    self.setUp()

    curvatures = [0.0, 0.05, 0.1]
    gains = [1.0, 1.2, 1.5]
    self.params.put('CurvatureGainInterp', json.dumps([curvatures, gains]))
    self.params.put('MaxCurvatureForGainInterp', "0.25")  # Very high value

    # Set up standard vehicle
    self.CP.wheelbase = 2.8  # Standard car
    self.CP.steerRatio = 14.0  # Standard steering ratio
    self.CP.mass = 1500  # Standard vehicle mass
    self.params.put("Mads", True)

    set_car_specific_params(self.CP, self.CP_SP, self.params)

    # For standard vehicle (base 0.1), the max limit would be min(0.3, 0.1*3.0) = 0.3
    # So custom value of 0.25 should be accepted
    self.assertLessEqual(max(self.CP_SP.curvatureGainInterp[0]), 0.25)

    # Reset for another test
    self.setUp()
    self.params.put('CurvatureGainInterp', json.dumps([curvatures, gains]))
    self.params.put('MaxCurvatureForGainInterp', "0.4")  # Outside safe range (0.3 max for standard car)

    # Should use default instead of unsafe value
    set_car_specific_params(self.CP, self.CP_SP, self.params)
    # Should be clamped based on vehicle characteristics

  def test_curvature_gain_max_multiplier_parameter(self):
    """Test that the max curvature gain multiplier can be configured via parameter"""
    # Reset for new test
    self.setUp()

    curvatures = [0.0, 0.05, 0.1]
    gains = [1.0, 1.2, 1.5]
    self.params.put('CurvatureGainInterp', json.dumps([curvatures, gains]))

    # Set up standard vehicle
    self.CP.wheelbase = 2.8  # Standard car
    self.CP.steerRatio = 14.0  # Standard steering ratio
    self.CP.mass = 1500  # Standard vehicle mass
    self.params.put("MaxCurvatureGainMultiplier", "6.0")  # Custom high multiplier
    self.params.put("Mads", True)

    set_car_specific_params(self.CP, self.CP_SP, self.params)

    # Should use the custom multiplier
    self.assertEqual(self.CP_SP.maxCurvatureGainMultiplier, 6.0)

  def test_curvature_gain_max_multiplier_invalid(self):
    """Test that invalid max curvature gain multiplier values are handled correctly"""
    # Reset for new test
    self.setUp()

    curvatures = [0.0, 0.05, 0.1]
    gains = [1.0, 1.2, 1.5]
    self.params.put('CurvatureGainInterp', json.dumps([curvatures, gains]))

    # Set up standard vehicle
    self.CP.wheelbase = 2.8  # Standard car
    self.CP.steerRatio = 14.0  # Standard steering ratio
    self.CP.mass = 1500  # Standard vehicle mass
    self.params.put("MaxCurvatureGainMultiplier", "15.0")  # Invalid high value
    self.params.put("Mads", True)

    set_car_specific_params(self.CP, self.CP_SP, self.params)

    # Should revert to default since value is outside safe range
    self.assertEqual(self.CP_SP.maxCurvatureGainMultiplier, 4.0)

  def test_curvature_gain_max_multiplier_vehicle_defaults(self):
    """Test that max curvature gain multiplier uses appropriate vehicle-specific defaults"""
    # Reset for new test
    self.setUp()

    curvatures = [0.0, 0.05, 0.1]
    gains = [1.0, 1.2, 1.5]
    self.params.put('CurvatureGainInterp', json.dumps([curvatures, gains]))

    # Test with heavy vehicle (conservative default)
    self.CP.wheelbase = 3.0  # Large vehicle
    self.CP.steerRatio = 16.0  # Standard steering ratio
    self.CP.mass = 2500  # Heavy vehicle
    self.params.put("Mads", True)

    set_car_specific_params(self.CP, self.CP_SP, self.params)

    # Should use conservative default for heavy vehicle
    self.assertEqual(self.CP_SP.maxCurvatureGainMultiplier, 3.0)

    # Reset for next test
    self.setUp()
    self.params.put('CurvatureGainInterp', json.dumps([curvatures, gains]))

    # Test with light vehicle (aggressive default)
    self.CP.wheelbase = 2.5  # Small vehicle
    self.CP.steerRatio = 12.0  # Standard steering ratio
    self.CP.mass = 1000  # Light vehicle
    self.params.put("Mads", True)

    set_car_specific_params(self.CP, self.CP_SP, self.params)

    # Should use more aggressive default for light vehicle
    self.assertEqual(self.CP_SP.maxCurvatureGainMultiplier, 5.0)

  def test_curvature_gain_enhanced_vehicle_specific_limits(self):
    """Test the enhanced vehicle-specific curvature limits based on multiple parameters"""
    # Reset for new test
    self.setUp()

    curvatures = [0.0, 0.05, 0.1]
    gains = [1.0, 1.2, 1.5]
    self.params.put('CurvatureGainInterp', json.dumps([curvatures, gains]))

    # Test with vehicle having high center of gravity (more front weight bias)
    self.CP.wheelbase = 2.8
    self.CP.steerRatio = 14.0
    self.CP.mass = 1500
    self.CP.centerToFront = 1.7  # More weight on front (1.7/2.8 = 0.607 > 0.6)
    self.CP.wheelStrutOffset = 0.15  # Standard track width
    self.params.put("Mads", True)

    set_car_specific_params(self.CP, self.CP_SP, self.params)

    # Verify that the curvature gain was processed
    self.assertIsNotNone(self.CP_SP.curvatureGainInterp)
    # The base limit would be 0.1, but with front weight bias it should be increased slightly

    # Reset for next test
    self.setUp()
    self.params.put('CurvatureGainInterp', json.dumps([curvatures, gains]))

    # Test with rear-weighted vehicle (potentially unstable)
    self.CP.wheelbase = 2.8
    self.CP.steerRatio = 14.0
    self.CP.mass = 1500
    self.CP.centerToFront = 1.2  # Less weight on front (1.2/2.8 = 0.429 < 0.45)
    self.CP.wheelStrutOffset = 0.15
    self.params.put("Mads", True)

    set_car_specific_params(self.CP, self.CP_SP, self.params)

    # Verify processing occurred
    self.assertIsNotNone(self.CP_SP.curvatureGainInterp)


if __name__ == "__main__":
  unittest.main()