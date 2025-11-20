#!/usr/bin/env python3
"""
Enhanced lateral safety functions for autonomous driving
Provides improved lateral control safety with curve anticipation and adaptive behavior
"""
import numpy as np
import math
from openpilot.common.constants import CV, ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.modeld.constants import ModelConstants

# Pre-computed constants for optimization
MAX_LATERAL_ACCEL = 3.0  # m/s^2
MAX_CURVATURE_RATE = 0.1  # Adjusted for safe response rates


def calculate_safe_curvature_limits(v_ego, max_lat_accel=MAX_LATERAL_ACCEL, roll_compensation=0.0):
  """
  Calculate safe curvature limits based on current speed and lateral acceleration constraints
  :param v_ego: Current vehicle speed in m/s
  :param max_lat_accel: Maximum allowed lateral acceleration
  :param roll_compensation: Compensation for road roll angle
  :return: Tuple of (min_curvature, max_curvature) allowed
  """
  if v_ego < 0.1:  # At very low speeds, allow any curvature (essentially no limit)
    return -0.2, 0.2

  v_ego_sq = v_ego ** 2
  max_lat_accel_adjusted = max_lat_accel + roll_compensation

  max_curvature = max_lat_accel_adjusted / v_ego_sq if v_ego_sq > 0.01 else 0.2
  min_curvature = -max_lat_accel_adjusted / v_ego_sq if v_ego_sq > 0.01 else -0.2

  # Clamp to reasonable bounds
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
    if not hasattr(path_y, '__len__') or len(path_y) < 10:
      return 0.0, 0.0
  except (TypeError, AttributeError):
    return 0.0, 0.0

  # Calculate how many points correspond to our lookahead distance
  # Model typically outputs 0.1s spaced points, and v_ego is in m/s
  points_per_meter = 10.0 / max(v_ego, 1.0)  # Approximate points per meter
  points_ahead = min(int(lookahead_distance * points_per_meter), len(model_v2.path.y))

  if points_ahead < 2:
    return 0.0, 0.0

  curvatures_ahead = []
  for i in range(1, points_ahead):  # Start from 1 to compute curvature from path points
    if i < len(model_v2.path.y) and i < len(model_v2.path.x):
      # Simple curvature approximation from path points
      if i + 1 < len(model_v2.path.y):
        # Calculate curvature from three consecutive points (if available)
        dx1 = model_v2.path.x[i] - model_v2.path.x[i-1] if i > 0 else 0.0
        dy1 = model_v2.path.y[i] - model_v2.path.y[i-1] if i > 0 else 0.0
        dx2 = model_v2.path.x[i+1] - model_v2.path.x[i] if i+1 < len(model_v2.path.x) else dx1
        dy2 = model_v2.path.y[i+1] - model_v2.path.y[i] if i+1 < len(model_v2.path.y) else dy1

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
  max_lat_accel = 3.0

  # Reduce for low model confidence
  if model_confidence < 0.8:
    confidence_factor = max(model_confidence / 0.8, 0.5)  # Reduce by up to 50% if confidence is very low
    max_lat_accel *= confidence_factor

  # Reduce for high curvature ahead
  if curvature_ahead > 0.005:  # Significant curve ahead
    curve_factor = max(0.5, 1.0 - (curvature_ahead - 0.005) * 100.0)  # More significant reduction for sharp curves
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
  if curvature_ahead > 0.008:  # Sharp curve ahead
    rate_limit_multiplier = 0.6  # Reduce rate of change by 40%

  # Be more conservative at high speeds
  if v_ego > 25.0:  # Above ~55 mph
    rate_limit_multiplier *= 0.8

  # Be more conservative with low model confidence
  if model_confidence < 0.6:
    rate_limit_multiplier *= 0.7

  # Further reduce rate when on hills
  if abs(road_pitch) > 0.08:  # Very steep grade
    rate_limit_multiplier *= 0.8

  return max_lat_accel, rate_limit_multiplier


def get_adaptive_lateral_curvature(v_ego, desired_curvature, prev_curvature, model_v2, params, 
                                   is_rainy=False, is_night=False, dt=0.01):
  """
  Get laterally safe and adaptive curvature considering various environmental factors
  :param v_ego: Current vehicle speed in m/s
  :param desired_curvature: Raw desired curvature from model
  :param prev_curvature: Previous curvature for rate limiting
  :param model_v2: Model output message
  :param params: Live parameters
  :param is_rainy: Rain condition flag
  :param is_night: Night condition flag
  :param dt: Time step
  :return: Laterally safe and adaptive curvature
  """
  # Get model confidence and road pitch information
  model_confidence = getattr(model_v2.meta, 'confidence', 1.0) if hasattr(model_v2, 'meta') else 1.0
  road_pitch = 0.0
  if hasattr(model_v2, 'orientationNED') and len(model_v2.orientationNED.x) > 0:
    road_pitch = model_v2.orientationNED.x[0]  # First element represents pitch

  # Anticipate upcoming curvature
  max_curv_ahead, avg_curv_ahead = anticipate_curvature_ahead(model_v2, v_ego, lookahead_distance=40.0)

  # Adjust limits based on conditions
  adjusted_max_lat_accel, rate_mult = adjust_lateral_limits_for_conditions(
    v_ego, max_curv_ahead, model_confidence, road_pitch, is_rainy, is_night
  )

  # Calculate safe limits
  roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
  min_curv_safe, max_curv_safe = calculate_safe_curvature_limits(v_ego, adjusted_max_lat_accel, roll_compensation)

  # Apply rate limiting to prevent excessive curvature changes
  v_ego_sq = max(v_ego**2, 1.0)
  max_curvature_rate = 5.0 / v_ego_sq  # Lateral jerk limit of 5 m/s^3 divided by v_ego^2
  max_delta = max_curvature_rate * dt * rate_mult  # Apply rate limit multiplier

  # Limit the desired curvature based on rate changes and safe bounds
  curvature_after_rate_limit = np.clip(desired_curvature,
                                       prev_curvature - max_delta,
                                       prev_curvature + max_delta)

  # Finally limit to safe bounds
  final_curvature = np.clip(curvature_after_rate_limit, min_curv_safe, max_curv_safe)

  return final_curvature