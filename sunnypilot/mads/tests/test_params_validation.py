"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import unittest
from unittest.mock import Mock
from sunnypilot.common.params_validation import ParamsValidator


class TestParamsValidator(unittest.TestCase):
  """Test cases for the ParamsValidator class."""
  
  def setUp(self):
    """Set up test fixtures."""
    self.mock_params = Mock()
    self.validator = ParamsValidator(self.mock_params)

  def test_validate_mads_steering_mode_valid(self):
    """Test validation of valid MADS steering mode values."""
    # Test all valid modes
    self.assertTrue(ParamsValidator.validate_mads_steering_mode("0"))
    self.assertTrue(ParamsValidator.validate_mads_steering_mode("1"))
    self.assertTrue(ParamsValidator.validate_mads_steering_mode("2"))

  def test_validate_mads_steering_mode_invalid(self):
    """Test validation of invalid MADS steering mode values."""
    self.assertFalse(ParamsValidator.validate_mads_steering_mode("3"))
    self.assertFalse(ParamsValidator.validate_mads_steering_mode("abc"))
    self.assertFalse(ParamsValidator.validate_mads_steering_mode("-1"))
    self.assertFalse(ParamsValidator.validate_mads_steering_mode(""))
    self.assertFalse(ParamsValidator.validate_mads_steering_mode("1.5"))

  def test_validate_mads_unified_engagement_mode(self):
    """Test validation of MADS unified engagement mode values."""
    # Valid values
    self.assertTrue(ParamsValidator.validate_mads_unified_engagement_mode("0"))
    self.assertTrue(ParamsValidator.validate_mads_unified_engagement_mode("1"))
    self.assertTrue(ParamsValidator.validate_mads_unified_engagement_mode("true"))
    self.assertTrue(ParamsValidator.validate_mads_unified_engagement_mode("false"))
    self.assertTrue(ParamsValidator.validate_mads_unified_engagement_mode("on"))
    self.assertTrue(ParamsValidator.validate_mads_unified_engagement_mode("off"))
    
    # Invalid values
    self.assertFalse(ParamsValidator.validate_mads_unified_engagement_mode("2"))
    self.assertFalse(ParamsValidator.validate_mads_unified_engagement_mode("yes"))
    self.assertFalse(ParamsValidator.validate_mads_unified_engagement_mode("no"))
    self.assertFalse(ParamsValidator.validate_mads_unified_engagement_mode("abc"))

  def test_validate_speed_limit_mode(self):
    """Test validation of speed limit mode values."""
    # Valid values
    self.assertTrue(ParamsValidator.validate_speed_limit_mode("0"))
    self.assertTrue(ParamsValidator.validate_speed_limit_mode("1"))
    self.assertTrue(ParamsValidator.validate_speed_limit_mode("2"))
    
    # Invalid values
    self.assertFalse(ParamsValidator.validate_speed_limit_mode("3"))
    self.assertFalse(ParamsValidator.validate_speed_limit_mode("abc"))
    self.assertFalse(ParamsValidator.validate_speed_limit_mode("-1"))
    self.assertFalse(ParamsValidator.validate_speed_limit_mode("1.5"))

  def test_get_validated_param_with_valid_value(self):
    """Test getting a validated parameter with a valid value."""
    self.mock_params.get.return_value = "1"
    result = self.validator.get_validated_param(
      "MadsSteeringMode", 
      ParamsValidator.validate_mads_steering_mode, 
      "1"
    )
    self.assertEqual(result, "1")

  def test_get_validated_param_with_invalid_value(self):
    """Test getting a validated parameter with an invalid value."""
    self.mock_params.get.return_value = "5"  # Invalid mode
    result = self.validator.get_validated_param(
      "MadsSteeringMode", 
      ParamsValidator.validate_mads_steering_mode, 
      "1"
    )
    self.assertEqual(result, "1")  # Should return default
    self.mock_params.put.assert_called_with("MadsSteeringMode", "1")

  def test_get_bool_param_valid_values(self):
    """Test getting boolean parameters with valid values."""
    # Test true values
    self.mock_params.get.return_value = "1"
    self.assertTrue(self.validator.get_bool_param("TestParam", False))
    
    self.mock_params.get.return_value = "true"
    self.assertTrue(self.validator.get_bool_param("TestParam", False))
    
    self.mock_params.get.return_value = "on"
    self.assertTrue(self.validator.get_bool_param("TestParam", False))
    
    # Test false values
    self.mock_params.get.return_value = "0"
    self.assertFalse(self.validator.get_bool_param("TestParam", True))
    
    self.mock_params.get.return_value = "false"
    self.assertFalse(self.validator.get_bool_param("TestParam", True))

  def test_get_bool_param_invalid_value(self):
    """Test getting boolean parameter with invalid value (should return default)."""
    self.mock_params.get.return_value = "invalid"
    self.assertTrue(self.validator.get_bool_param("TestParam", True))  # Should return default
    
    self.mock_params.get.return_value = None
    self.assertFalse(self.validator.get_bool_param("TestParam", False))  # Should return default


if __name__ == '__main__':
  unittest.main()