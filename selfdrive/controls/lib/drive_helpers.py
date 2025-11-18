import numpy as np
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.realtime import DT_CTRL, DT_MDL

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0
# This is a turn radius smaller than most cars can achieve
MAX_CURVATURE = 0.2
MAX_VEL_ERR = 5.0  # m/s

# EU guidelines
MAX_LATERAL_JERK = 5.0  # m/s^3
MAX_LATERAL_ACCEL_NO_ROLL = 3.0  # m/s^2


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

def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL, vEgoStopping=0.05, nav_instruction=None):
  import time
  from openpilot.selfdrive.common.metrics import Metrics, record_metric

  start_time = time.time()
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

  # Consider navigation instructions that may require stopping
  if nav_instruction and hasattr(nav_instruction, 'distanceToManeuver'):
    if 0 < nav_instruction.distanceToManeuver < 50.0:  # Within 50m of maneuver
      # Modify acceleration based on navigation requirements
      maneuver_type = getattr(nav_instruction, 'maneuverType', 'none')
      if maneuver_type in ['turn', 'arrive', 'stop', 'yield']:
        # Reduce speed when approaching maneuvers
        if v_target > 5.0:  # If going faster than 5 m/s
          a_target = min(a_target, -0.5)  # Gentle deceleration
        if nav_instruction.distanceToManeuver < 10.0:
          # More aggressive deceleration close to maneuver
          a_target = min(a_target, -1.0)
        # Override should_stop if approaching a maneuver
        if maneuver_type in ['arrive', 'stop'] and nav_instruction.distanceToManeuver < 5.0:
          should_stop = True

  # Record metrics for planning performance
  planning_time = time.time() - start_time
  record_metric(Metrics.PLANNING_LATENCY_MS, planning_time * 1000, {
      "operation": "get_accel_from_plan",
      "v_target": v_target,
      "a_target": a_target,
      "should_stop": should_stop,
      "nav_instruction_active": bool(nav_instruction)
  })

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
