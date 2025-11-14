"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.common.params import Params
from opendbc.car import structs
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, HyundaiSafetyFlagsSP
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from sunnypilot.common.params_validation import ParamsValidator


MADS_NO_ACC_MAIN_BUTTON = ("rivian", "tesla")


class MadsSteeringModeOnBrake:
  REMAIN_ACTIVE = 0
  PAUSE = 1
  DISENGAGE = 2


def get_mads_limited_brands(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> bool:
  """Determine if MADS functionality is limited for certain brands."""
  if CP.brand == 'rivian':
    return True
  if CP.brand == 'tesla':
    return not CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS

  return False


def read_steering_mode_param(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params):
  """Safely read MADS steering mode parameter with proper validation."""
  if get_mads_limited_brands(CP, CP_SP):
    return MadsSteeringModeOnBrake.DISENGAGE

  # Use the new validation system
  validator = ParamsValidator(params)
  validated_value = validator.get_validated_param(
    "MadsSteeringMode",
    ParamsValidator.validate_mads_steering_mode,
    str(MadsSteeringModeOnBrake.PAUSE)
  )

  try:
    mode_int = int(validated_value)
    # Ensure the value is within the expected range
    if mode_int in [MadsSteeringModeOnBrake.REMAIN_ACTIVE,
                    MadsSteeringModeOnBrake.PAUSE,
                    MadsSteeringModeOnBrake.DISENGAGE]:
      return mode_int
    else:
      # Invalid value despite validation, return default
      return MadsSteeringModeOnBrake.PAUSE
  except (ValueError, TypeError):
    # Error parsing validated parameter, return default
    return MadsSteeringModeOnBrake.PAUSE


def set_alternative_experience(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params):
  """Set alternative experience flags based on MADS configuration with better validation."""
  enabled = params.get_bool("Mads")
  steering_mode = read_steering_mode_param(CP, CP_SP, params)

  if enabled:
    CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ENABLE_MADS

    if steering_mode == MadsSteeringModeOnBrake.DISENGAGE:
      CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.MADS_DISENGAGE_LATERAL_ON_BRAKE
    elif steering_mode == MadsSteeringModeOnBrake.PAUSE:
      CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.MADS_PAUSE_LATERAL_ON_BRAKE


def set_car_specific_params(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params):
  """Set car-specific parameters with improved validation and error handling."""
  if CP.brand == "hyundai":
    # TODO-SP: This should be separated from MADS module for future implementations
    #          Use "HyundaiLongitudinalMainCruiseToggleable" param
    hyundai_cruise_main_toggleable = True
    if hyundai_cruise_main_toggleable:
      CP_SP.flags |= HyundaiFlagsSP.LONGITUDINAL_MAIN_CRUISE_TOGGLEABLE.value
      CP_SP.safetyParam |= HyundaiSafetyFlagsSP.LONG_MAIN_CRUISE_TOGGLEABLE

  # Use the new validation system
  validator = ParamsValidator(params)

  # Validate all MADS parameters
  validator.validate_all_mads_params(CP)

  # MADS Partial Support
  # MADS is currently partially supported for these platforms due to lack of consistent states to engage controls
  # Only MadsSteeringModeOnBrake.DISENGAGE is supported for these platforms
  # TODO-SP: To enable MADS full support for Rivian and most Tesla, identify consistent signals for MADS toggling
  mads_partial_support = get_mads_limited_brands(CP, CP_SP)
  if mads_partial_support:
    # Ensure proper mode is set for limited brands using validated parameters
    params.put("MadsSteeringMode", str(MadsSteeringModeOnBrake.DISENGAGE))
    params.put_bool("MadsUnifiedEngagementMode", True)
    # Re-validate after setting values
    validator.validate_all_mads_params(CP)

  # no ACC MAIN button for these brands
  if CP.brand in MADS_NO_ACC_MAIN_BUTTON:
    # Only remove if the param exists to avoid errors
    try:
      params.get("MadsMainCruiseAllowed")
      params.remove("MadsMainCruiseAllowed")
    except Exception:
      # Parameter doesn't exist, nothing to remove
      pass
