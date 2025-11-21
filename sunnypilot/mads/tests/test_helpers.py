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

if __name__ == "__main__":
  unittest.main()