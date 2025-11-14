"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from typing import Any

from openpilot.common.params import Params
from opendbc.car import structs


class ParamsValidator:
  """
  A parameter validation system for sunnypilot to ensure user parameters
  are valid and provide helpful feedback when they are not.
  """

  def __init__(self, params: Params):
    self.params = params

  @staticmethod
  def validate_mads_steering_mode(value: str) -> bool:
    """Validate MADS steering mode parameter."""
    try:
      mode = int(value)
      # Valid modes are 0 (REMAIN_ACTIVE), 1 (PAUSE), 2 (DISENGAGE)
      return mode in [0, 1, 2]
    except (ValueError, TypeError):
      return False

  @staticmethod
  def validate_mads_unified_engagement_mode(value: str) -> bool:
    """Validate MADS unified engagement mode parameter."""
    # Should be a boolean string ('0' or '1', or 'true'/'false')
    return value.lower() in ['0', '1', 'true', 'false', 'on', 'off']

  @staticmethod
  def validate_speed_limit_mode(value: str) -> bool:
    """Validate Speed Limit Assist mode parameter."""
    try:
      mode = int(value)
      # Common modes are 0 (off), 1 (on), 2 (assist)
      return mode in [0, 1, 2]
    except (ValueError, TypeError):
      return False

  def get_validated_param(self, param_name: str, validator_func, default_value) -> Any:
    """
    Get a parameter and validate it, returning a default if validation fails.

    Args:
        param_name: Name of the parameter to retrieve
        validator_func: Function to validate the parameter value
        default_value: Value to return if validation fails

    Returns:
        Validated parameter value or default
    """
    try:
      value = self.params.get(param_name, encoding='utf-8')
      if value is not None and validator_func(value):
        return value
      else:
        # Parameter is invalid, store the default value
        if isinstance(default_value, bool):
          self.params.put_bool(param_name, default_value)
          return str(int(default_value))
        elif isinstance(default_value, int):
          self.params.put(param_name, str(default_value))
          return str(default_value)
        else:
          self.params.put(param_name, str(default_value))
          return str(default_value)
    except Exception:
      # Error retrieving parameter, store and return default
      if isinstance(default_value, bool):
        self.params.put_bool(param_name, default_value)
        return str(int(default_value))
      elif isinstance(default_value, int):
        self.params.put(param_name, str(default_value))
        return str(default_value)
      else:
        self.params.put(param_name, str(default_value))
        return str(default_value)

  def validate_all_mads_params(self, CP: structs.CarParams) -> None:
    """Validate all MADS-related parameters."""
    # Validate MADS steering mode
    self.get_validated_param(
      "MadsSteeringMode",
      self.validate_mads_steering_mode,
      "1"  # Default to PAUSE mode
    )

    # Validate MADS unified engagement mode
    self.get_validated_param(
      "MadsUnifiedEngagementMode",
      self.validate_mads_unified_engagement_mode,
      "1"  # Default to enabled
    )

    # Validate MADS main cruise allowed
    self.get_validated_param(
      "MadsMainCruiseAllowed",
      self.validate_mads_unified_engagement_mode,
      "1"  # Default to enabled
    )

  def get_bool_param(self, param_name: str, default: bool = False) -> bool:
    """
    Safely retrieve a boolean parameter with validation and fallback.
    
    Args:
        param_name: Name of the parameter to retrieve
        default: Default value if parameter is invalid
        
    Returns:
        Boolean value of the parameter
    """
    try:
      value = self.params.get(param_name, encoding='utf-8')
      if value is not None:
        return value.lower() in ['1', 'true', 'on', 'yes', 'enable']
      else:
        return default
    except Exception:
      return default
