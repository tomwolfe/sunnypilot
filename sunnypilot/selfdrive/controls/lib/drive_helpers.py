from numpy import clip, interp
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED, MAX_LATERAL_JERK
from openpilot.sunnypilot.modeld.constants import ModelConstants

# A+ Enhancement: Dynamic Delay Compensation - ENABLED
# E2E driving feels "disconnected" due to 100ms+ delay between camera frame and actuator movement.
# This enables the model to predict where the car should be at T + latency, not just T now.
USE_DYNAMIC_DELAY_A = True


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
      
      # A+ Enhancement: Use World Model for delay prediction if available
      # The World Model can predict the effective delay by comparing imagined vs actual outcomes
      try:
        from openpilot.sunnypilot.modeld_v2.e2e_torque import E2EController
        
        # Check if we have a global e2e_controller with world model
        if hasattr(get_lag_adjusted_curvature, '_e2e_controller') and get_lag_adjusted_curvature._e2e_controller:
          e2e_ctrl = get_lag_adjusted_curvature._e2e_controller
          if e2e_ctrl.enable_dynamic_delay and e2e_ctrl.delay_predictor:
            # Use the E2E controller's delay predictor (more accurate, learned)
            delay_pred = e2e_ctrl.predict_delay(
              current_curvature=current_curvature_desired,
              speed=v_ego,
              desired_curvature=current_curvature_desired
            )
            dynamic_delay = delay_pred.total_delay
            
            # Apply adaptive filtering for smoothness
            if e2e_ctrl.delay_filter:
              dynamic_delay = e2e_ctrl.get_adjusted_delay(steer_delay, dynamic_delay)
      except Exception:
        pass  # Fallback to standalone predictor
        
    except Exception:
      # Fallback: simple load-based delay
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
