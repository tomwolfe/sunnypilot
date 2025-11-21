import json
import unittest
from unittest.mock import MagicMock, patch

from openpilot.common.params import Params
from opendbc.car import structs
from sunnypilot.mads.helpers import set_car_specific_params


class TestMadsHelpers(unittest.TestCase):

  def setUp(self):
    self.mock_params = MagicMock(spec=Params)
    self.CP = structs.CarParams.new_message()
    self.CP_SP = structs.CarParamsSP()
    # Reset curvatureGainInterp before each test
    self.CP_SP.curvatureGainInterp = None

  @patch('sunnypilot.mads.helpers.cloudlog')
  def test_curvature_gain_interp_valid(self, mock_cloudlog):
    valid_interp = [[0.0, 0.01, 0.02, 0.03], [1.0, 1.1, 1.2, 1.3]]
    self.mock_params.get.return_value = json.dumps(valid_interp)

    set_car_specific_params(self.CP, self.CP_SP, self.mock_params)

    self.assertEqual(list(self.CP_SP.curvatureGainInterp), valid_interp)
    mock_cloudlog.error.assert_not_called()

  @patch('sunnypilot.mads.helpers.cloudlog')
  def test_curvature_gain_interp_invalid_json(self, mock_cloudlog):
    self.mock_params.get.return_value = "invalid json"

    set_car_specific_params(self.CP, self.CP_SP, self.mock_params)

    self.assertIsNone(self.CP_SP.curvatureGainInterp)
    mock_cloudlog.error.assert_called_once_with("Failed to decode or validate CurvatureGainInterp from Params: Expecting value: line 1 column 1 (char 0)")

  @patch('sunnypilot.mads.helpers.cloudlog')
  def test_curvature_gain_interp_not_list_of_lists(self, mock_cloudlog):
    invalid_interp = [0.0, 1.0]
    self.mock_params.get.return_value = json.dumps(invalid_interp)

    set_car_specific_params(self.CP, self.CP_SP, self.mock_params)

    self.assertIsNone(self.CP_SP.curvatureGainInterp)
    mock_cloudlog.error.assert_called_once_with("Failed to decode or validate CurvatureGainInterp from Params: curvatureGainInterp must be a list of two lists of floats")

  @patch('sunnypilot.mads.helpers.cloudlog')
  def test_curvature_gain_interp_incorrect_length(self, mock_cloudlog):
    invalid_interp = [[0.0], [1.0], [2.0]]
    self.mock_params.get.return_value = json.dumps(invalid_interp)

    set_car_specific_params(self.CP, self.CP_SP, self.mock_params)

    self.assertIsNone(self.CP_SP.curvatureGainInterp)
    mock_cloudlog.error.assert_called_once_with("Failed to decode or validate CurvatureGainInterp from Params: curvatureGainInterp must be a list of two lists of floats")

  @patch('sunnypilot.mads.helpers.cloudlog')
  def test_curvature_gain_interp_non_numeric_values(self, mock_cloudlog):
    invalid_interp = [[0.0, "0.01"], [1.0, 1.1]]
    self.mock_params.get.return_value = json.dumps(invalid_interp)

    set_car_specific_params(self.CP, self.CP_SP, self.mock_params)

    self.assertIsNone(self.CP_SP.curvatureGainInterp)
    # The error message from numpy for non-numeric conversion might be complex, so just check if error is logged
    mock_cloudlog.error.assert_called_once()

  @patch('sunnypilot.mads.helpers.cloudlog')
  def test_curvature_gain_interp_non_negative_curvatures(self, mock_cloudlog):
    invalid_interp = [[-0.01, 0.01], [1.0, 1.1]]
    self.mock_params.get.return_value = json.dumps(invalid_interp)

    set_car_specific_params(self.CP, self.CP_SP, self.mock_params)

    self.assertIsNone(self.CP_SP.curvatureGainInterp)
    mock_cloudlog.error.assert_called_once_with("Failed to decode or validate CurvatureGainInterp from Params: Curvature values must be non-negative")

  @patch('sunnypilot.mads.helpers.cloudlog')
  def test_curvature_gain_interp_non_ascending_curvatures(self, mock_cloudlog):
    invalid_interp = [[0.02, 0.01], [1.1, 1.0]]
    self.mock_params.get.return_value = json.dumps(invalid_interp)

    set_car_specific_params(self.CP, self.CP_SP, self.mock_params)

    self.assertIsNone(self.CP_SP.curvatureGainInterp)
    mock_cloudlog.error.assert_called_once_with("Failed to decode or validate CurvatureGainInterp from Params: Curvature values must be in ascending order")

  @patch('sunnypilot.mads.helpers.cloudlog')
  def test_curvature_gain_interp_gain_less_than_one(self, mock_cloudlog):
    invalid_interp = [[0.0, 0.01], [1.0, 0.9]]
    self.mock_params.get.return_value = json.dumps(invalid_interp)

    set_car_specific_params(self.CP, self.CP_SP, self.mock_params)

    self.assertIsNone(self.CP_SP.curvatureGainInterp)
    mock_cloudlog.error.assert_called_once_with("Failed to decode or validate CurvatureGainInterp from Params: Gain multipliers must be at least 1.0 (reducing gain for curves is counterproductive)")

  @patch('sunnypilot.mads.helpers.cloudlog')
  def test_curvature_gain_interp_mismatched_lengths(self, mock_cloudlog):
    invalid_interp = [[0.0, 0.01], [1.0]]
    self.mock_params.get.return_value = json.dumps(invalid_interp)

    set_car_specific_params(self.CP, self.CP_SP, self.mock_params)

    self.assertIsNone(self.CP_SP.curvatureGainInterp)
    mock_cloudlog.error.assert_called_once_with("Failed to decode or validate CurvatureGainInterp from Params: Curvature values and gain multipliers must have the same length")

  @patch('sunnypilot.mads.helpers.cloudlog')
  def test_curvature_gain_interp_empty_curvature_array(self, mock_cloudlog):
    invalid_interp = [[], []]
    self.mock_params.get.return_value = json.dumps(invalid_interp)

    set_car_specific_params(self.CP, self.CP_SP, self.mock_params)

    self.assertIsNone(self.CP_SP.curvatureGainInterp)
    mock_cloudlog.error.assert_called_once_with("Failed to decode or validate CurvatureGainInterp from Params: Curvature values cannot be empty")

  @patch('sunnypilot.mads.helpers.cloudlog')
  def test_curvature_gain_interp_exceeds_physical_limit(self, mock_cloudlog):
    invalid_interp = [[0.0, 0.11], [1.0, 1.2]]
    self.mock_params.get.return_value = json.dumps(invalid_interp)

    set_car_specific_params(self.CP, self.CP_SP, self.mock_params)

    self.assertIsNone(self.CP_SP.curvatureGainInterp)
    mock_cloudlog.error.assert_called_once_with("Failed to decode or validate CurvatureGainInterp from Params: Curvature values exceed physical limits for road turns (max 0.1 m^-1)")

  @patch('sunnypilot.mads.helpers.cloudlog')
  def test_curvature_gain_interp_not_set(self, mock_cloudlog):
    self.mock_params.get.return_value = None # Param not set

    set_car_specific_params(self.CP, self.CP_SP, self.mock_params)

    self.assertIsNone(self.CP_SP.curvatureGainInterp)
    mock_cloudlog.error.assert_not_called()
