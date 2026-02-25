from numpy import clip, interp
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED, MAX_LATERAL_JERK
from openpilot.sunnypilot.modeld.constants import ModelConstants

USE_DYNAMIC_DELAY_A = False


def get_lag_adjusted_curvature(steer_delay, v_ego, psis, curvatures):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
  v_ego = max(MIN_SPEED, v_ego)

  current_curvature_desired = curvatures[0]

  if USE_DYNAMIC_DELAY_A:
    try:
      from openpilot.sunnypilot.modeld_v2.e2e_torque.dynamic_delay import DynamicDelayPredictor

      delay_predictor = DynamicDelayPredictor()
      delay_pred = delay_predictor.predict_delay(
        current_curvature=current_curvature_desired,
        current_speed=v_ego,
        desired_curvature=current_curvature_desired
      )
      dynamic_delay = delay_pred.total_delay
    except Exception:
      load_factor = abs(current_curvature_desired) * v_ego
      dynamic_delay = steer_delay * (1.0 + 0.2 * clip(load_factor, 0.0, 1.0))
  else:
    load_factor = abs(current_curvature_desired) * v_ego
    dynamic_delay = steer_delay * (1.0 + 0.2 * clip(load_factor, 0.0, 1.0))

  psi = interp(dynamic_delay, ModelConstants.T_IDXS[:CONTROL_N], psis)
  average_curvature_desired = psi / (v_ego * dynamic_delay)
  desired_curvature = 2 * average_curvature_desired - current_curvature_desired

  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2)
  safe_desired_curvature = clip(desired_curvature,
                                current_curvature_desired - max_curvature_rate * DT_MDL,
                                current_curvature_desired + max_curvature_rate * DT_MDL)

  return float(safe_desired_curvature)
