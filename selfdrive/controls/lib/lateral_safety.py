#!/usr/bin/env python3
"""
Enhanced lateral safety functions for autonomous driving
Provides improved lateral control safety with curve anticipation and adaptive behavior
"""
import numpy as np
import math
from openpilot.common.constants import CV, ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.modeld.constants import ModelConstants
from .autonomous_params import LATERAL_SAFETY_PARAMS, SAFETY_PARAMETERS
from .safety_state_manager import get_global_safety_state_manager
from openpilot.common.swaglog import cloudlog

# Import parameters from centralized SAFETY_PARAMETERS to ensure consistency
MAX_LATERAL_ACCEL = SAFETY_PARAMETERS['MAX_LATERAL_ACCEL']
MAX_CURVATURE_RATE = SAFETY_PARAMETERS['MAX_CURVATURE_RATE']
DEFAULT_TIME_STEP = LATERAL_SAFETY_PARAMS['DEFAULT_TIME_STEP']
MIN_TIME_STEP = LATERAL_SAFETY_PARAMS['MIN_TIME_STEP']
MAX_TIME_STEP = LATERAL_SAFETY_PARAMS['MAX_TIME_STEP']
MAX_SAFE_LATERAL_ACCEL = SAFETY_PARAMETERS['MAX_SAFE_LATERAL_ACCEL']
MIN_SAFE_LATERAL_ACCEL = SAFETY_PARAMETERS['MIN_SAFE_LATERAL_ACCEL']
CURVATURE_AHEAD_THRESHOLD = SAFETY_PARAMETERS['CURVATURE_AHEAD_THRESHOLD']
SHARP_CURVE_THRESHOLD = SAFETY_PARAMETERS['SHARP_CURVE_THRESHOLD']
MODEL_CONFIDENCE_THRESHOLD = SAFETY_PARAMETERS['MODEL_CONFIDENCE_THRESHOLD']
MODEL_CONFIDENCE_LOW_THRESHOLD = SAFETY_PARAMETERS['MODEL_CONFIDENCE_LOW_THRESHOLD']
LATERAL_JERK_LIMIT = SAFETY_PARAMETERS['LATERAL_JERK_LIMIT']

# Get thread-safe state manager
_safety_state = get_global_safety_state_manager()

def validate_time_step(dt):
    """
    Validate that the time step is within safe operating ranges
    :param dt: Time step in seconds
    :return: Validated time step
    :raises ValueError: If dt is outside the safe range
    """
    if dt < MIN_TIME_STEP or dt > MAX_TIME_STEP:
        raise ValueError(f"Time step {dt} is outside safe range [{MIN_TIME_STEP}, {MAX_TIME_STEP}]")
    return dt

def validate_lateral_acceleration(max_lat_accel):
    """
    Validate that lateral acceleration is within safe operating ranges
    :param max_lat_accel: Maximum lateral acceleration in m/s^2
    :return: Validated lateral acceleration
    :raises ValueError: If max_lat_accel is outside safe range
    """
    if max_lat_accel < MIN_SAFE_LATERAL_ACCEL or max_lat_accel > MAX_SAFE_LATERAL_ACCEL:
        raise ValueError(f"Lateral acceleration {max_lat_accel} m/s^2 is outside safe range [{MIN_SAFE_LATERAL_ACCEL}, {MAX_SAFE_LATERAL_ACCEL}]")

    return max_lat_accel


def calculate_safe_curvature_limits(v_ego, max_lat_accel=MAX_LATERAL_ACCEL, roll_compensation=0.0, CP=None):
  """
  Calculate safe curvature limits based on current speed and lateral acceleration constraints
  :param v_ego: Current vehicle speed in m/s
  :param max_lat_accel: Maximum allowed lateral acceleration
  :param roll_compensation: Compensation for road roll angle
  :param CP: CarParams for vehicle-specific parameters, if available
  :return: Tuple of (min_curvature, max_curvature) allowed
  """
  # Validate inputs to prevent dangerous calculations
  try:
      v_ego = float(v_ego)  # Ensure v_ego is a float
      max_lat_accel = float(max_lat_accel)  # Ensure max_lat_accel is a float
      roll_compensation = float(roll_compensation)  # Ensure roll_compensation is a float
  except (ValueError, TypeError):
      cloudlog.error("Invalid input types to calculate_safe_curvature_limits, using safe defaults")
      return -0.1, 0.1  # Very conservative limits for invalid inputs

  # Validate max_lat_accel is within safe ranges
  max_lat_accel = max(MIN_SAFE_LATERAL_ACCEL, min(MAX_SAFE_LATERAL_ACCEL, max_lat_accel))

  if v_ego < 0.1:  # At very low speeds, allow any curvature (essentially no limit)
    return -0.2, 0.2

  # Use vehicle-specific parameters if available, otherwise use defaults
  if CP is not None and hasattr(CP, 'lateralParams') and hasattr(CP.lateralParams, 'maxLateralAccel'):
    try:
        vehicle_max_lat_accel = float(CP.lateralParams.maxLateralAccel)
        # Use the minimum of provided max_lat_accel and vehicle capability
        max_lat_accel = min(max_lat_accel, vehicle_max_lat_accel)
    except (ValueError, TypeError, AttributeError):
        # If vehicle max_lat_accel is invalid, continue with default max_lat_accel
        pass

  v_ego_sq = v_ego ** 2
  max_lat_accel_adjusted = max_lat_accel + roll_compensation

  # Additional safety check for division by zero - ensure v_ego_sq is not too small
  # Use a more conservative threshold to avoid numerical instability
  min_v_ego_sq = 0.01  # Square of 0.1 m/s, provides more stable calculation
  if v_ego_sq < min_v_ego_sq:  # Very low speed, use conservative limits
      max_curvature = 0.2
      min_curvature = -0.2
  else:
      # Safe division with bounds checking
      try:
          max_curvature = max_lat_accel_adjusted / v_ego_sq
          min_curvature = -max_lat_accel_adjusted / v_ego_sq
      except ZeroDivisionError:
          # Shouldn't happen with our checks above, but safety net
          max_curvature = 0.2
          min_curvature = -0.2
      except OverflowError:
          # Handle potential overflow in the division
          max_curvature = 0.2
          min_curvature = -0.2

  # Clamp to reasonable bounds to ensure safety
  max_curvature = min(max_curvature, 0.2)
  min_curvature = max(min_curvature, -0.2)

  return min_curvature, max_curvature


def anticipate_curvature_ahead(model_v2, v_ego, lookahead_distance=50.0):
  """
  Analyze upcoming road curvature from model data to inform lateral control
  :param model_v2: Model output message
  :param v_ego: Current vehicle speed in m/s
  :param lookahead_distance: Distance ahead to analyze curvature (meters)
  :return: Tuple of (max_curvature_ahead, avg_curvature_ahead) in the lookahead distance
  """
  try:
    path_y = getattr(model_v2.path, 'y', [])
    path_x = getattr(model_v2.path, 'x', [])
    if not hasattr(path_y, '__len__') or not hasattr(path_x, '__len__') or len(path_y) < 10 or len(path_x) < 10:
      return 0.0, 0.0
  except (TypeError, AttributeError):
    return 0.0, 0.0

  # Calculate how many points correspond to our lookahead distance
  # Model typically outputs 0.1s spaced points, and v_ego is in m/s
  effective_v_ego = max(v_ego, 1.0)  # Avoid division by zero
  points_per_meter = 10.0 / effective_v_ego  # Approximate points per meter
  points_ahead = min(int(lookahead_distance * points_per_meter), len(path_y), len(path_x))

  if points_ahead < 2:
    return 0.0, 0.0

  # Use vectorized operations for performance optimization
  try:
    # Convert to numpy arrays for vectorized operations
    path_y_slice = np.array(path_y[:points_ahead])
    path_x_slice = np.array(path_x[:points_ahead])

    # Compute differences in one vectorized operation
    dx = np.diff(path_x_slice)
    dy = np.diff(path_y_slice)

    # Calculate headings for all points at once
    headings = np.arctan2(dy, dx)

    # Calculate heading changes
    heading_changes = np.diff(headings)

    # Normalize angles to be within [-π, π]
    heading_changes = ((heading_changes + np.pi) % (2 * np.pi)) - np.pi

    # Calculate absolute curvatures
    curvatures_ahead = np.abs(heading_changes)

    if len(curvatures_ahead) > 0:
        max_curvature = float(np.max(curvatures_ahead))
        avg_curvature = float(np.mean(curvatures_ahead))
        return max_curvature, avg_curvature
    else:
        return 0.0, 0.0
  except Exception:
    # Fallback to original method if vectorization fails (e.g., due to input data issues)
    curvatures_ahead = []
    for i in range(1, points_ahead):  # Start from 1 to compute curvature from path points
      if i < len(path_y) and i < len(path_x):
        # Simple curvature approximation from path points
        if i + 1 < len(path_y) and i + 1 < len(path_x):
          # Calculate curvature from three consecutive points (if available)
          dx1 = path_x[i] - path_x[i-1] if i > 0 else 0.0
          dy1 = path_y[i] - path_y[i-1] if i > 0 else 0.0
          dx2 = path_x[i+1] - path_x[i] if i+1 < len(path_x) else dx1
          dy2 = path_y[i+1] - path_y[i] if i+1 < len(path_y) else dy1

          if abs(dx1) > 0.001 or abs(dy1) > 0.001:
            # Approximate curvature as the rate of heading change
            heading1 = math.atan2(dy1, dx1)
            heading2 = math.atan2(dy2, dx2)
            heading_change = heading2 - heading1
            # Normalize angle to be within [-π, π]
            while heading_change > math.pi:
              heading_change -= 2 * math.pi
            while heading_change < -math.pi:
              heading_change += 2 * math.pi

            # Approximate curvature (simplified - in real implementation could use more sophisticated method)
            curvature = heading_change  # This is a simplified approximation
            curvatures_ahead.append(abs(curvature))

    if curvatures_ahead:
      max_curvature = max(curvatures_ahead)
      avg_curvature = sum(curvatures_ahead) / len(curvatures_ahead)
      return max_curvature, avg_curvature

    return 0.0, 0.0


def adjust_lateral_limits_for_conditions(v_ego, curvature_ahead, model_confidence, road_pitch=0.0, is_rainy=False, is_night=False):
  """
  Adjust lateral control parameters based on environmental and model conditions
  :param v_ego: Current vehicle speed in m/s
  :param curvature_ahead: Maximum anticipated curvature ahead
  :param model_confidence: Confidence level from neural network (0.0 to 1.0)
  :param road_pitch: Road grade/pitch angle in radians
  :param is_rainy: Whether conditions are rainy
  :param is_night: Whether it's night time
  :return: Tuple of (max_lat_accel, rate_limit_multiplier) adjusted for conditions
  """
  # Base lateral acceleration limit
  max_lat_accel = MAX_LATERAL_ACCEL

  # Reduce for low model confidence
  if model_confidence < MODEL_CONFIDENCE_THRESHOLD:
    confidence_factor = max(model_confidence / MODEL_CONFIDENCE_THRESHOLD, 0.5)  # Reduce by up to 50% if confidence is very low
    max_lat_accel *= confidence_factor

  # Reduce for high curvature ahead
  if curvature_ahead > CURVATURE_AHEAD_THRESHOLD:  # Significant curve ahead
    curve_factor = max(0.5, 1.0 - (curvature_ahead - CURVATURE_AHEAD_THRESHOLD) * 100.0)  # More significant reduction for sharp curves
    max_lat_accel *= curve_factor

  # Reduce for adverse weather conditions
  if is_rainy:
    max_lat_accel *= 0.7  # Reduce by 30% in rain

  # Reduce for night conditions
  if is_night:
    max_lat_accel *= 0.85  # Reduce by 15% at night

  # Adjust for road grade - reduce on steep hills
  if abs(road_pitch) > 0.05:  # More than 5% grade
    grade_factor = max(0.7, 1.0 - abs(road_pitch) * 2)  # Up to 30% reduction on steep hills
    max_lat_accel *= grade_factor

  # Calculate rate limit multiplier (how quickly we can change acceleration)
  rate_limit_multiplier = 1.0

  # Be more conservative when approaching curves
  if curvature_ahead > SHARP_CURVE_THRESHOLD:  # Sharp curve ahead
    rate_limit_multiplier = 0.6  # Reduce rate of change by 40%

  # Be more conservative at high speeds
  if v_ego > 25.0:  # Above ~55 mph
    rate_limit_multiplier *= 0.8

  # Be more conservative with low model confidence
  if model_confidence < MODEL_CONFIDENCE_LOW_THRESHOLD:
    rate_limit_multiplier *= 0.7

  # Further reduce rate when on hills
  if abs(road_pitch) > 0.08:  # Very steep grade
    rate_limit_multiplier *= 0.8

  return max_lat_accel, rate_limit_multiplier


def get_adaptive_lateral_curvature(v_ego, desired_curvature, prev_curvature, model_v2, params, CP=None,
                                   is_rainy=False, is_night=False, dt=0.01):
    """
    Get laterally safe and adaptive curvature considering various environmental factors
    Optimized for performance while maintaining safety
    """
    try:
        # Validate time step parameter early and return if invalid
        try:
            dt = validate_time_step(dt)
        except ValueError as e:
            cloudlog.error(f"Invalid time step in get_adaptive_lateral_curvature: {e}")
            # Use default time step if validation fails
            dt = DEFAULT_TIME_STEP

        # Get model confidence efficiently
        model_confidence = getattr(getattr(model_v2, 'meta', None), 'confidence', 1.0)

        # Get road pitch safely with minimal attribute access
        road_pitch = 0.0
        try:
            orientation_ned = getattr(model_v2, 'orientationNED', None)
            if orientation_ned and hasattr(orientation_ned, 'x') and orientation_ned.x:
                road_pitch = orientation_ned.x[0] if len(orientation_ned.x) > 0 else 0.0
        except (AttributeError, IndexError, TypeError):
            pass  # Use default road pitch

        # Anticipate upcoming curvature (this is already optimized)
        max_curv_ahead, avg_curv_ahead = anticipate_curvature_ahead(model_v2, v_ego, lookahead_distance=40.0)

        # Adjust limits based on conditions using pre-defined functions
        adjusted_max_lat_accel, rate_mult = adjust_lateral_limits_for_conditions(
          v_ego, max_curv_ahead, model_confidence, road_pitch, is_rainy, is_night
        )

        # Validate the adjusted lateral acceleration
        try:
            adjusted_max_lat_accel = validate_lateral_acceleration(adjusted_max_lat_accel)
        except ValueError as e:
            cloudlog.error(f"Invalid lateral acceleration in get_adaptive_lateral_curvature: {e}")
            # Use safe default
            adjusted_max_lat_accel = MAX_LATERAL_ACCEL * 0.5  # Conservative default

        # Calculate safe limits (optimized)
        roll_compensation = getattr(params, 'roll', 0.0) * ACCELERATION_DUE_TO_GRAVITY
        min_curv_safe, max_curv_safe = calculate_safe_curvature_limits(
            v_ego, adjusted_max_lat_accel, roll_compensation, CP
        )

        # Apply rate limiting efficiently
        v_ego_sq = max(v_ego**2, 1.0)
        max_curvature_rate = LATERAL_JERK_LIMIT / v_ego_sq
        max_delta = max_curvature_rate * dt * rate_mult

        # Apply rate limiting and safety bounds in one step
        curvature_after_rate_limit = np.clip(
            desired_curvature,
            prev_curvature - max_delta,
            prev_curvature + max_delta
        )

        final_curvature = np.clip(
            curvature_after_rate_limit,
            min_curv_safe,
            max_curv_safe
        )

        # Reset error counter on successful execution
        _safety_state.reset_errors()
        return final_curvature
    except Exception as e:
        # Comprehensive error handling with safer fallback
        cloudlog.error(f"Error in get_adaptive_lateral_curvature: {e}")

        # Update state manager with new curvature and increment error
        _safety_state.update_error(prev_curvature)

        # Get error count for tiered fallback
        error_count = _safety_state.get_error_count()

        # Implement standardized fallback as per unified error handling policy
        from .safety_state_manager import ERROR_HANDLING

        if error_count >= 8:
            # Disengage system after 8 consecutive errors
            cloudlog.error("Maximum error count reached, system should disengage")
            # For this function, return the fallback curvature
            return _safety_state.get_fallback_curvature()
        elif error_count >= 6:
            # Apply 90% conservative factor after 6 errors
            fallback_curvature = _safety_state.get_fallback_curvature()
            return fallback_curvature * ERROR_HANDLING['degradation_factors'][6]
        elif error_count >= 3:
            # Apply 70% conservative factor after 3 errors
            fallback_curvature = _safety_state.get_fallback_curvature()
            return fallback_curvature * ERROR_HANDLING['degradation_factors'][3]
        else:
            # Use historical data for first few errors
            return _safety_state.get_fallback_curvature()