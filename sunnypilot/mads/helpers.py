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
      if not all(np.diff(curvatures) >= 0):  # Changed from > to >= to allow non-decreasing (equal values are acceptable)
        raise ValueError("Curvature values must be in non-decreasing order to prevent interpolation issues")
      if not all(gains >= 1.0):
        raise ValueError("Gain multipliers must be at least 1.0 (reducing gain for curves is counterproductive)")
      if len(curvatures) != len(gains):
        raise ValueError("Curvature values and gain multipliers must have the same length")
      if len(curvatures) == 0:
        raise ValueError("Curvature values cannot be empty")

      # Additional validation for array size to prevent performance issues
      if len(curvatures) > 50:  # Reasonable upper limit
        cloudlog.warning(f"Curvature gain interpolation has {len(curvatures)} points, which is excessive. Limiting to 50 points.")
        # Take evenly spaced points to preserve the curve shape
        indices = np.round(np.linspace(0, len(curvatures) - 1, 50)).astype(int)
        reduced_curvatures = curvatures[indices]
        reduced_gains = gains[indices]
        curvature_gain_interp = [reduced_curvatures.tolist(), reduced_gains.tolist()]
        curvatures = reduced_curvatures
        gains = reduced_gains

      # Determine maximum curvature limit based on comprehensive vehicle characteristics for better safety
      # Default to 0.1 m^-1 (10m radius) for standard vehicles, but allow customization for urban driving
      # Adjust limit based on multiple vehicle-specific characteristics for better safety
      base_curvature_limit = 0.1  # Default: 10m radius turn

      # Adjust based on vehicle's turning capability and safety characteristics
      if CP.wheelbase > 3.0:  # Larger vehicles (trucks, SUVs) have larger turning radius
        base_curvature_limit = 0.08  # 12.5m radius
      elif CP.wheelbase < 2.4:  # Smaller vehicles (compact cars)
        base_curvature_limit = 0.15  # 6.7m radius

      # Consider vehicle's steering ratio for sharper turns
      if hasattr(CP, 'steerRatio') and CP.steerRatio is not None:
        if CP.steerRatio > 16:  # Higher steering ratio = more responsive steering
          base_curvature_limit = min(base_curvature_limit * 1.2, 0.2)
        elif CP.steerRatio < 12:  # Lower steering ratio = less responsive steering
          base_curvature_limit = max(base_curvature_limit * 0.8, 0.05)

      # Consider vehicle's center of gravity and stability characteristics
      # Higher center of gravity vehicles should have more conservative limits
      if hasattr(CP, 'centerToFront') and CP.centerToFront is not None:
        # If the center is further towards the rear (more weight on rear),
        # adjust for possible oversteering characteristics
        front_weight_ratio = CP.centerToFront / CP.wheelbase
        if front_weight_ratio > 0.6:  # More weight on front (conservative) - can handle more curvature
          base_curvature_limit = min(base_curvature_limit * 1.1, 0.2)
        elif front_weight_ratio < 0.45:  # More weight on rear (potentially unstable) - be more conservative
          base_curvature_limit = max(base_curvature_limit * 0.9, 0.03)

      # Consider vehicle track width for stability (if available)
      if hasattr(CP, 'wheelStrutOffset') and CP.wheelStrutOffset is not None:
        # Wider track = more stability = can handle higher curvatures
        if CP.wheelStrutOffset > 0.2:  # Wider track vehicles
          base_curvature_limit = min(base_curvature_limit * 1.05, 0.2)

      # Consider vehicle mass distribution and overall mass
      if CP.mass > 2200:  # Heavy vehicles (SUVs, trucks) - reduce limits for safety
        base_curvature_limit = max(base_curvature_limit * 0.9, 0.03)
      elif CP.mass < 1100:  # Light vehicles - can be more aggressive
        base_curvature_limit = min(base_curvature_limit * 1.1, 0.2)

      # Additional vehicle-specific factors could be considered in the future:
      # - Tire size and type (if available in CP)
      # - Suspension characteristics
      # - Aerodynamic properties
      # - Brake distribution

      # Allow user override but within safe bounds for the specific vehicle
      max_curvature_limit = base_curvature_limit
      custom_curvature_limit_str = params.get("MaxCurvatureForGainInterp", encoding='utf8')
      if custom_curvature_limit_str:
        try:
          custom_limit = float(custom_curvature_limit_str)
          # Set reasonable bounds that consider the vehicle's physical capabilities
          min_limit = max(0.03, base_curvature_limit * 0.5)  # At least 3.3m minimum radius
          max_limit = min(0.3, base_curvature_limit * 3.0)   # At most 3x the base limit

          if min_limit <= custom_limit <= max_limit:
            max_curvature_limit = custom_limit
          else:
            cloudlog.warning(f"MaxCurvatureForGainInterp parameter {custom_limit} outside safe range [{min_limit:.3f}, {max_limit:.3f}] for vehicle, using default {base_curvature_limit:.3f}")
        except (ValueError, TypeError):
          cloudlog.warning(f"Invalid MaxCurvatureForGainInterp parameter: {custom_curvature_limit_str}, using default {base_curvature_limit:.3f}")

      if max(curvatures) > max_curvature_limit:
        cloudlog.warning(f"Curvature values exceed physical limits for road turns (max {max_curvature_limit} m^-1). Clamping values.")

        # Find points that are within the limit
        valid_indices = np.where(curvatures <= max_curvature_limit)[0]

        new_curvatures = curvatures[valid_indices]
        new_gains = gains[valid_indices]

        # If the curve was truncated, add the limit point with an interpolated gain
        if len(new_curvatures) < len(curvatures):
          if not new_curvatures.size or new_curvatures[-1] < max_curvature_limit:
            interp_gain = np.interp(max_curvature_limit, curvatures, gains)
            new_curvatures = np.append(new_curvatures, max_curvature_limit)
            new_gains = np.append(new_gains, interp_gain)

        # Ensure that after clamping, the arrays are not empty.
        if new_curvatures.size == 0:
          cloudlog.warning("Clamping resulted in empty curvature gain points. Reverting to default.")
          new_curvatures = np.array([0.0])
          new_gains = np.array([1.0])

        curvature_gain_interp = [new_curvatures.tolist(), new_gains.tolist()]
        curvatures = new_curvatures  # Update for subsequent checks if needed
        gains = new_gains  # Update for subsequent checks if needed

      # All other validations remain as rejections, as they indicate malformed parameters
      # (e.g., non-negative, ascending order, gain multipliers >= 1.0, matching lengths, non-empty)
      # These checks are implicitly handled by the prior logic or are fundamental to the interp structure.

      CP_SP.curvatureGainInterp = curvature_gain_interp
      # Log successful configuration for monitoring
      cloudlog.info(f"Curvature gain interpolation configured successfully with {len(curvatures)} points, max curvature {max(curvatures):.3f} m^-1, max gain {max(gains):.2f}x")
    except (json.JSONDecodeError, ValueError) as e:
      cloudlog.error(f"Failed to decode or validate CurvatureGainInterp from Params: {e}")
      CP_SP.curvatureGainInterp = [[0.0], [1.0]] # Set safe default on error
      cloudlog.warning("Curvature gain interpolation reverted to safe default due to validation error.")
  else:
    CP_SP.curvatureGainInterp = [[0.0], [1.0]] # Set safe default if param is not set
    cloudlog.debug("Curvature gain interpolation using default values.")

  # Set configurable maximum curvature gain multiplier with vehicle-specific defaults
  max_curvature_gain_multiplier_str = params.get("MaxCurvatureGainMultiplier", encoding='utf8')
  if max_curvature_gain_multiplier_str:
    try:
      max_curvature_gain_multiplier = float(max_curvature_gain_multiplier_str)
      # Validate the multiplier is reasonable - minimum 1.0 (no gain limiting), maximum 10.0 (very aggressive)
      if 1.0 <= max_curvature_gain_multiplier <= 10.0:
        CP_SP.maxCurvatureGainMultiplier = max_curvature_gain_multiplier
        cloudlog.debug(f"Max curvature gain multiplier set to {max_curvature_gain_multiplier}x from parameter")
      else:
        cloudlog.warning(f"MaxCurvatureGainMultiplier parameter {max_curvature_gain_multiplier} outside safe range [1.0, 10.0], using default 4.0")
        CP_SP.maxCurvatureGainMultiplier = 4.0
    except (ValueError, TypeError):
      cloudlog.warning(f"Invalid MaxCurvatureGainMultiplier parameter: {max_curvature_gain_multiplier_str}, using default 4.0")
      CP_SP.maxCurvatureGainMultiplier = 4.0
  else:
    # Set default based on vehicle characteristics
    if CP.mass > 2000:  # Large/heavy vehicles get more conservative default
      CP_SP.maxCurvatureGainMultiplier = 3.0
      cloudlog.debug("Max curvature gain multiplier set to 3.0x (conservative default for heavy vehicles)")
    elif CP.mass < 1200:  # Lighter vehicles can be more aggressive if needed
      CP_SP.maxCurvatureGainMultiplier = 5.0
      cloudlog.debug("Max curvature gain multiplier set to 5.0x (aggressive default for light vehicles)")
    else:  # Standard vehicles
      CP_SP.maxCurvatureGainMultiplier = 4.0
      cloudlog.debug("Max curvature gain multiplier set to 4.0x (standard default)")

  # Set configurable safety limit threshold with vehicle-specific defaults
  safety_limit_threshold_str = params.get("SafetyLimitThreshold", encoding='utf8')
  if safety_limit_threshold_str:
    try:
      safety_limit_threshold = int(safety_limit_threshold_str)
      # Validate threshold is reasonable - minimum 10, maximum 500
      if 10 <= safety_limit_threshold <= 500:
        CP_SP.safetyLimitThreshold = safety_limit_threshold
        cloudlog.debug(f"Safe mode activation threshold set to {safety_limit_threshold} from parameter")
      else:
        cloudlog.warning(f"SafetyLimitThreshold parameter {safety_limit_threshold} outside safe range [10, 500], using default 100")
        CP_SP.safetyLimitThreshold = 100
    except (ValueError, TypeError):
      cloudlog.warning(f"Invalid SafetyLimitThreshold parameter: {safety_limit_threshold_str}, using default 100")
      CP_SP.safetyLimitThreshold = 100
  else:
    # Set default based on vehicle characteristics
    if CP.mass > 2000:  # Heavy vehicles - more conservative
      CP_SP.safetyLimitThreshold = 80
      cloudlog.debug("Safe mode activation threshold set to 80 (conservative for heavy vehicles)")
    elif CP.mass < 1200:  # Light vehicles - can be more sensitive
      CP_SP.safetyLimitThreshold = 120
      cloudlog.debug("Safe mode activation threshold set to 120 (sensitive for light vehicles)")
    else:  # Standard vehicles
      CP_SP.safetyLimitThreshold = 100
      cloudlog.debug("Safe mode activation threshold set to 100 (standard default)")

  # Set configurable safety limit time window with defaults
  safety_limit_time_window_str = params.get("SafetyLimitTimeWindow", encoding='utf8')
  if safety_limit_time_window_str:
    try:
      safety_limit_time_window = float(safety_limit_time_window_str)
      # Validate time window is reasonable - minimum 5.0, maximum 300.0 (5 minutes)
      if 5.0 <= safety_limit_time_window <= 300.0:
        CP_SP.safetyLimitTimeWindow = safety_limit_time_window
        cloudlog.debug(f"Safe mode time window set to {safety_limit_time_window}s from parameter")
      else:
        cloudlog.warning(f"SafetyLimitTimeWindow parameter {safety_limit_time_window} outside safe range [5.0, 300.0], using default 60.0")
        CP_SP.safetyLimitTimeWindow = 60.0
    except (ValueError, TypeError):
      cloudlog.warning(f"Invalid SafetyLimitTimeWindow parameter: {safety_limit_time_window_str}, using default 60.0")
      CP_SP.safetyLimitTimeWindow = 60.0
  else:
    CP_SP.safetyLimitTimeWindow = 60.0  # Default to 60 seconds
    cloudlog.debug("Safe mode time window set to 60.0s (standard default)")

  # Set vehicle-specific oscillation detection thresholds
  # Sign change threshold
  oscillation_sign_threshold_str = params.get("OscillationSignChangeThreshold", encoding='utf8')
  if oscillation_sign_threshold_str:
    try:
      oscillation_sign_threshold = float(oscillation_sign_threshold_str)
      # Validate threshold is reasonable - minimum 0.1, maximum 0.9
      if 0.1 <= oscillation_sign_threshold <= 0.9:
        CP_SP.oscillationSignChangeThreshold = oscillation_sign_threshold
        cloudlog.debug(f"Oscillation sign change threshold set to {oscillation_sign_threshold} from parameter")
      else:
        cloudlog.warning(f"OscillationSignChangeThreshold parameter {oscillation_sign_threshold} outside safe range [0.1, 0.9], using default 0.6")
        CP_SP.oscillationSignChangeThreshold = 0.6
    except (ValueError, TypeError):
      cloudlog.warning(f"Invalid OscillationSignChangeThreshold parameter: {oscillation_sign_threshold_str}, using default 0.6")
      CP_SP.oscillationSignChangeThreshold = 0.6
  else:
    # Set default based on vehicle characteristics
    if CP.mass > 2000:  # Heavy vehicles - higher threshold (less sensitive)
      CP_SP.oscillationSignChangeThreshold = 0.66  # 66% for heavy vehicles
      cloudlog.debug("Oscillation sign change threshold set to 0.66 (less sensitive for heavy vehicles)")
    elif CP.mass < 1200:  # Light vehicles - lower threshold (more sensitive)
      CP_SP.oscillationSignChangeThreshold = 0.54  # 54% for light vehicles
      cloudlog.debug("Oscillation sign change threshold set to 0.54 (more sensitive for light vehicles)")
    else:  # Standard vehicles
      CP_SP.oscillationSignChangeThreshold = 0.60
      cloudlog.debug("Oscillation sign change threshold set to 0.60 (standard default)")

  # Variance threshold
  oscillation_variance_threshold_str = params.get("OscillationVarianceThreshold", encoding='utf8')
  if oscillation_variance_threshold_str:
    try:
      oscillation_variance_threshold = float(oscillation_variance_threshold_str)
      # Validate threshold is reasonable - minimum 0.3, maximum 0.95
      if 0.3 <= oscillation_variance_threshold <= 0.95:
        CP_SP.oscillationVarianceThreshold = oscillation_variance_threshold
        cloudlog.debug(f"Oscillation variance threshold set to {oscillation_variance_threshold} from parameter")
      else:
        cloudlog.warning(f"OscillationVarianceThreshold parameter {oscillation_variance_threshold} outside safe range [0.3, 0.95], using default 0.8")
        CP_SP.oscillationVarianceThreshold = 0.8
    except (ValueError, TypeError):
      cloudlog.warning(f"Invalid OscillationVarianceThreshold parameter: {oscillation_variance_threshold_str}, using default 0.8")
      CP_SP.oscillationVarianceThreshold = 0.8
  else:
    # Set default based on vehicle characteristics
    if CP.mass > 2000:  # Heavy vehicles - higher threshold (less sensitive)
      CP_SP.oscillationVarianceThreshold = 0.88  # 88% for heavy vehicles
      cloudlog.debug("Oscillation variance threshold set to 0.88 (less sensitive for heavy vehicles)")
    elif CP.mass < 1200:  # Light vehicles - lower threshold (more sensitive)
      CP_SP.oscillationVarianceThreshold = 0.72  # 72% for light vehicles
      cloudlog.debug("Oscillation variance threshold set to 0.72 (more sensitive for light vehicles)")
    else:  # Standard vehicles
      CP_SP.oscillationVarianceThreshold = 0.80
      cloudlog.debug("Oscillation variance threshold set to 0.80 (standard default)")

  # Zero crossing threshold
  oscillation_zero_crossing_threshold_str = params.get("OscillationZeroCrossingThreshold", encoding='utf8')
  if oscillation_zero_crossing_threshold_str:
    try:
      oscillation_zero_crossing_threshold = float(oscillation_zero_crossing_threshold_str)
      # Validate threshold is reasonable - minimum 0.2, maximum 0.9
      if 0.2 <= oscillation_zero_crossing_threshold <= 0.9:
        CP_SP.oscillationZeroCrossingThreshold = oscillation_zero_crossing_threshold
        cloudlog.debug(f"Oscillation zero crossing threshold set to {oscillation_zero_crossing_threshold} from parameter")
      else:
        cloudlog.warning(f"OscillationZeroCrossingThreshold parameter {oscillation_zero_crossing_threshold} outside safe range [0.2, 0.9], using default 0.5")
        CP_SP.oscillationZeroCrossingThreshold = 0.5
    except (ValueError, TypeError):
      cloudlog.warning(f"Invalid OscillationZeroCrossingThreshold parameter: {oscillation_zero_crossing_threshold_str}, using default 0.5")
      CP_SP.oscillationZeroCrossingThreshold = 0.5
  else:
    # Set default based on vehicle characteristics
    if CP.mass > 2000:  # Heavy vehicles - higher threshold (less sensitive)
      CP_SP.oscillationZeroCrossingThreshold = 0.55  # 55% for heavy vehicles
      cloudlog.debug("Oscillation zero crossing threshold set to 0.55 (less sensitive for heavy vehicles)")
    elif CP.mass < 1200:  # Light vehicles - lower threshold (more sensitive)
      CP_SP.oscillationZeroCrossingThreshold = 0.45  # 45% for light vehicles
      cloudlog.debug("Oscillation zero crossing threshold set to 0.45 (more sensitive for light vehicles)")
    else:  # Standard vehicles
      CP_SP.oscillationZeroCrossingThreshold = 0.50
      cloudlog.debug("Oscillation zero crossing threshold set to 0.50 (standard default)")

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
