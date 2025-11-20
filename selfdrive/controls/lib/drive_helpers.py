import math
import numpy as np
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.realtime import DT_CTRL, DT_MDL
from .autonomous_params import LATERAL_SAFETY_PARAMS

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0
# This is a turn radius smaller than most cars can achieve
MAX_CURVATURE = 0.2
MAX_VEL_ERR = 5.0  # m/s

# EU guidelines
MAX_LATERAL_JERK = LATERAL_SAFETY_PARAMS['LATERAL_JERK_LIMIT']  # m/s^3
MAX_LATERAL_ACCEL_NO_ROLL = LATERAL_SAFETY_PARAMS['MAX_LATERAL_ACCEL']  # m/s^2


def clamp(val, min_val, max_val):
  clamped_val = float(np.clip(val, min_val, max_val))
  return clamped_val, clamped_val != val

def smooth_value(val, prev_val, tau, dt=DT_MDL):
  alpha = 1 - np.exp(-dt/tau) if tau > 0 else 1
  return alpha * val + (1 - alpha) * prev_val

# Optimized smooth value with pre-computed alpha values
class SmoothValueCache:
  def __init__(self):
    self.alpha_cache = {}

  def get_alpha(self, tau, dt=DT_MDL):
    key = (tau, dt)
    if key not in self.alpha_cache:
      self.alpha_cache[key] = 1 - np.exp(-dt/tau) if tau > 0 else 1
    return self.alpha_cache[key]

_smooth_value_cache = SmoothValueCache()

def smooth_value_optimized(val, prev_val, tau, dt=DT_MDL):
  alpha = _smooth_value_cache.get_alpha(tau, dt)
  return alpha * val + (1 - alpha) * prev_val

# Pre-computed constants for optimization
MAX_LATERAL_JERK_DT_CTRL = MAX_LATERAL_JERK * DT_CTRL
NEG_MAX_CURVATURE = -MAX_CURVATURE

def clip_curvature(v_ego, prev_curvature, new_curvature, roll):
  # This function respects ISO lateral jerk and acceleration limits + a max curvature
  v_ego = max(v_ego, MIN_SPEED)
  v_ego_sq = v_ego ** 2  # Compute once
  max_curvature_rate = MAX_LATERAL_JERK / v_ego_sq  # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  max_delta = max_curvature_rate * DT_CTRL  # Pre-computed: MAX_LATERAL_JERK_DT_CTRL / v_ego_sq
  new_curvature = np.clip(new_curvature,
                          prev_curvature - max_delta,
                          prev_curvature + max_delta)

  roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
  max_lat_accel = MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  min_lat_accel = -MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  new_curvature, limited_accel = clamp(new_curvature, min_lat_accel / v_ego_sq, max_lat_accel / v_ego_sq)

  new_curvature, limited_max_curv = clamp(new_curvature, NEG_MAX_CURVATURE, MAX_CURVATURE)  # Use pre-computed constant
  return new_curvature, limited_accel or limited_max_curv  # Return as-is, conversion happens at usage


# Pre-computed value for optimization
TWO_DT_MDL = 2 * DT_MDL

def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL, vEgoStopping=0.05):
  if len(speeds) == len(t_idxs):
    v_now = speeds[0]
    a_now = accels[0]
    v_target = np.interp(action_t, t_idxs, speeds)
    # Optimize division by multiplication with pre-computed value
    inv_action_t = 1.0 / action_t if action_t != 0 else 0
    a_target = (TWO_DT_MDL * (v_target - v_now) * inv_action_t) - a_now
    v_target_1sec = np.interp(action_t + 1.0, t_idxs, speeds)
  else:
    v_target = 0.0
    v_target_1sec = 0.0
    a_target = 0.0
  should_stop = (v_target < vEgoStopping and
                 v_target_1sec < vEgoStopping)
  return a_target, should_stop

def curv_from_psis(psi_target, psi_rate, vego, action_t):
  vego = np.clip(vego, MIN_SPEED, np.inf)
  # Precompute the divisor to use multiplication instead of division
  denom = vego * action_t
  curv_from_psi = psi_target / denom if denom != 0 else 0
  return (2 * curv_from_psi) - (psi_rate / vego if vego != 0 else 0)

def get_curvature_from_plan(yaws, yaw_rates, t_idxs, vego, action_t):
  psi_target = np.interp(action_t, t_idxs, yaws)
  psi_rate = yaw_rates[0]
  return curv_from_psis(psi_target, psi_rate, vego, action_t)


def get_safe_speed_from_curvature(curvature, max_lat_accel=None):
  """
  Calculate the maximum safe speed given a curvature and maximum lateral acceleration
  :param curvature: Road curvature (1/m)
  :param max_lat_accel: Maximum allowed lateral acceleration (m/s^2), if not provided uses default
  :return: Maximum safe speed (m/s)
  """
  if max_lat_accel is None:
    max_lat_accel = LATERAL_SAFETY_PARAMS['MAX_LATERAL_ACCEL']  # Use parameterized default

  abs_curvature = abs(curvature)
  if abs_curvature < 0.0001:  # Nearly straight
    return float('inf')  # No speed limit for straight roads

  # Safe speed formula: v = sqrt(a * r) where r = 1/curvature
  # Ensure we don't have negative values inside sqrt
  if max_lat_accel <= 0 or abs_curvature <= 0:
    return 0.0

  safe_speed = math.sqrt(max_lat_accel / abs_curvature)
  return safe_speed


def adjust_curvature_for_road_conditions(curvature, v_ego, road_pitch=0.0, model_confidence=1.0,
                                        is_rainy=False, is_night=False):
  """
  Adjust curvature for safer execution based on road and environmental conditions
  :param curvature: Desired curvature
  :param v_ego: Current speed (m/s)
  :param road_pitch: Road grade/pitch (rad)
  :param model_confidence: Model confidence (0-1)
  :param is_rainy: Whether it's raining
  :param is_night: Whether it's night
  :return: Adjusted curvature for safer execution
  """
  # Base adjustment factor
  adjustment = 1.0

  # Reduce adjustment (i.e., make turn wider) in adverse conditions
  if is_rainy:
    adjustment *= 0.8  # More conservative in rain
  if is_night:
    adjustment *= 0.9  # More conservative at night
  if model_confidence < 0.7:
    adjustment *= 0.85  # More conservative with low model confidence
  if abs(road_pitch) > 0.08:  # Steep grade
    adjustment *= 0.9  # Be more conservative on hills

  # At high speeds, be more conservative with curvature
  if v_ego > 20.0:  # Above ~45 mph
    adjustment *= 0.95

  # Adjust the curvature (reducing absolute value makes the turn less sharp)
  adjusted_curvature = curvature * adjustment

  # Apply additional constraints at high speeds and high curvatures
  safe_v = get_safe_speed_from_curvature(abs(adjusted_curvature))
  if safe_v < v_ego * 0.9:  # If current speed is 90% above safe speed
    # Reduce curvature further to ensure safety
    excessive_factor = min(0.9, safe_v / (v_ego * 0.9 + 0.1))  # Avoid division by zero
    adjusted_curvature = adjusted_curvature * excessive_factor

  return adjusted_curvature
