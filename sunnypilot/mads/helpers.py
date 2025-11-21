"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import json
import numpy as np
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
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

  # Load vehicle-specific curvature gain interpolation if available
  curvature_gain_interp_str = params.get("CurvatureGainInterp", encoding='utf8')
  if curvature_gain_interp_str:
    try:
      curvature_gain_interp = json.loads(curvature_gain_interp_str)
      # Comprehensive validation for curvatureGainInterp
      if not (isinstance(curvature_gain_interp, list) and len(curvature_gain_interp) == 2 and
              all(isinstance(l, list) and all(isinstance(f, (float, int)) for f in l) for l in curvature_gain_interp)):
        raise ValueError("curvatureGainInterp must be a list of two lists of floats")

      curvatures = np.array(curvature_gain_interp[0])
      gains = np.array(curvature_gain_interp[1])

      original_curvatures = curvatures.copy()
      original_gains = gains.copy()

      if not all(curvatures >= 0):
        raise ValueError("Curvature values must be non-negative")
      if not all(np.diff(curvatures) > 0):
        raise ValueError("Curvature values must be in ascending order")
      if not all(gains >= 1.0):
        raise ValueError("Gain multipliers must be at least 1.0 (reducing gain for curves is counterproductive)")
      if len(curvatures) != len(gains):
        raise ValueError("Curvature values and gain multipliers must have the same length")
      if len(curvatures) == 0:
        raise ValueError("Curvature values cannot be empty")

      # Check if there's a custom maximum curvature limit parameter
      # Default to 0.1 m^-1 (10m radius) for safety, but allow customization for urban driving
      max_curvature_limit = 0.1  # Default to 0.1 m^-1 (10m radius turn)
      custom_curvature_limit_str = params.get("MaxCurvatureForGainInterp", encoding='utf8')
      if custom_curvature_limit_str:
        try:
          custom_limit = float(custom_curvature_limit_str)
          # Reasonable upper limit for road driving while keeping safety
          if 0.05 <= custom_limit <= 0.2:  # Range from 20m to 5m radius turns
            max_curvature_limit = custom_limit
          else:
            cloudlog.warning(f"MaxCurvatureForGainInterp parameter outside safe range [0.05, 0.2]: {custom_limit}, using default 0.1")
        except (ValueError, TypeError):
          cloudlog.warning(f"Invalid MaxCurvatureForGainInterp parameter: {custom_curvature_limit_str}, using default 0.1")

      if max(curvatures) > max_curvature_limit:
        cloudlog.warning(f"Curvature values exceed physical limits for road turns (max {max_curvature_limit} m^-1). Clamping values.")

        # Find the index of the first curvature value that exceeds the limit
        # np.searchsorted returns the index where max_curvature_limit would be inserted to maintain order
        clamp_idx = np.searchsorted(curvatures, max_curvature_limit, side='right')

        # Truncate curvatures and gains where curvature > max_curvature_limit
        new_curvatures = curvatures[:clamp_idx].tolist()
        new_gains = gains[:clamp_idx].tolist()

        # If the truncated list's last element is less than max_curvature_limit,
        # we need to add max_curvature_limit and its interpolated gain.
        if not new_curvatures or new_curvatures[-1] < max_curvature_limit:
            # Interpolate the gain at max_curvature_limit from the ORIGINAL arrays
            clamped_gain = np.interp(max_curvature_limit, curvatures, gains)
            new_curvatures.append(max_curvature_limit)
            new_gains.append(clamped_gain)
        
        # Ensure that after clamping, the arrays are not empty. Revert to safe defaults if they are.
        if len(new_curvatures) == 0:
            cloudlog.warning("Clamping resulted in empty curvature gain points. Reverting to default.")
            new_curvatures = [0.0]
            new_gains = [1.0]

        curvature_gain_interp = [new_curvatures, new_gains]
        curvatures = np.array(new_curvatures) # Update for subsequent checks if needed
        gains = np.array(new_gains) # Update for subsequent checks if needed

      # All other validations remain as rejections, as they indicate malformed parameters
      # (e.g., non-negative, ascending order, gain multipliers >= 1.0, matching lengths, non-empty)
      # These checks are implicitly handled by the prior logic or are fundamental to the interp structure.

      CP_SP.curvatureGainInterp = curvature_gain_interp
    except (json.JSONDecodeError, ValueError) as e:
      cloudlog.error(f"Failed to decode or validate CurvatureGainInterp from Params: {e}")
      CP_SP.curvatureGainInterp = [[0.0], [1.0]] # Set safe default on error
  else:
    CP_SP.curvatureGainInterp = [[0.0], [1.0]] # Set safe default if param is not set

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
