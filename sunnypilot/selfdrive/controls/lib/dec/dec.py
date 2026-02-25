"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import messaging
from opendbc.car import structs
import numpy as np
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.dec.constants import WMACConstants
from typing import Literal, Optional
from dataclasses import dataclass


TRAJECTORY_SIZE = 33
SET_MODE_TIMEOUT = 15


class SmoothKalmanFilter:
  """Enhanced Kalman filter with smoothing for stable decision making."""

  def __init__(self, initial_value=0, measurement_noise=0.1, process_noise=0.01,
               alpha=1.0, smoothing_factor=0.85):
    self.x = initial_value
    self.P = 1.0
    self.R = measurement_noise
    self.Q = process_noise
    self.alpha = alpha
    self.smoothing_factor = smoothing_factor
    self.initialized = False
    self.history = []
    self.max_history = 10
    self.confidence = 0.0

  def add_data(self, measurement):
    if len(self.history) >= self.max_history:
      self.history.pop(0)
    self.history.append(measurement)

    if not self.initialized:
      self.x = measurement
      self.initialized = True
      self.confidence = 0.1
      return

    self.P = self.alpha * self.P + self.Q

    K = self.P / (self.P + self.R)
    effective_K = K * (1.0 - self.smoothing_factor) + self.smoothing_factor * 0.1

    innovation = measurement - self.x
    self.x = self.x + effective_K * innovation
    self.P = (1 - effective_K) * self.P

    if abs(innovation) < 0.1:
      self.confidence = min(1.0, self.confidence + 0.05)
    else:
      self.confidence = max(0.1, self.confidence - 0.02)

  def get_value(self):
    return self.x if self.initialized else None

  def get_confidence(self):
    return self.confidence

  def reset_data(self):
    self.initialized = False
    self.history = []
    self.confidence = 0.0


@dataclass
class VisionTrafficDetection:
  """Vision-only traffic signal detection results."""
  signal_type: str = 'none'
  distance: float = 0.0
  probability: float = 0.0
  should_precharge_brake: bool = False
  predictive_stop_prob: float = 0.0


class VisionOnlyTrafficAuditor:
  """
  E2E-Native Predictive Braking System.
  Detects stop signs and red lights using ONLY the E2E model's internal signals,
  without relying on map data. Uses model uncertainty and disengage predictions
  to proactively prepare for stops.
  """
  PREDICTIVE_LOOKAHEAD_SEC = 3.0
  STOP_SIGN_UNCERTAINTY_THRESHOLD = 8.0
  RED_LIGHT_UNCERTAINTY_THRESHOLD = 6.0
  STOP_PROB_THRESHOLD = 0.7
  PRECHARGE_SPEED_THRESHOLD_MS = 2.0

  def __init__(self):
    self._stop_sign_prob = 0.0
    self._red_light_prob = 0.0
    self._predictive_stop_prob = 0.0
    self._precharge_brake = False
    self._last_detection_time = 0

    self._uncertainty_stop_filter = SmoothKalmanFilter(
      measurement_noise=0.15,
      process_noise=0.1,
      alpha=1.05,
      smoothing_factor=0.7
    )

    self._stop_sign_filter = SmoothKalmanFilter(
      measurement_noise=0.2,
      process_noise=0.15,
      alpha=1.1,
      smoothing_factor=0.6
    )

    self._red_light_filter = SmoothKalmanFilter(
      measurement_noise=0.2,
      process_noise=0.15,
      alpha=1.1,
      smoothing_factor=0.6
    )

    self._predictive_stop_filter = SmoothKalmanFilter(
      measurement_noise=0.15,
      process_noise=0.1,
      alpha=1.05,
      smoothing_factor=0.65
    )

  def update(self, model_uncertainty: np.ndarray, disengage_probs: np.ndarray,
             v_ego: float) -> VisionTrafficDetection:
    """
    Analyze E2E model outputs to detect potential stops at stop signs or red lights.
    Returns a VisionTrafficDetection with the analysis results.
    """
    result = VisionTrafficDetection()

    if len(model_uncertainty) < TRAJECTORY_SIZE or len(disengage_probs) < TRAJECTORY_SIZE:
      return result

    max_uncertainty = float(np.max(model_uncertainty))
    self._uncertainty_stop_filter.add_data(max_uncertainty)
    filtered_uncertainty = self._uncertainty_stop_filter.get_value() or 0.0

    lookahead_idx = int(self.PREDICTIVE_LOOKAHEAD_SEC * 10)
    lookahead_idx = min(lookahead_idx, len(disengage_probs) - 1)

    predictive_prob = float(np.max(disengage_probs[:lookahead_idx + 1]))
    self._predictive_stop_filter.add_data(predictive_prob)
    self._predictive_stop_prob = self._predictive_stop_filter.get_value() or 0.0
    result.predictive_stop_prob = self._predictive_stop_prob

    stop_sign_detected = (
      filtered_uncertainty > self.STOP_SIGN_UNCERTAINTY_THRESHOLD and
      self._predictive_stop_prob > self.STOP_PROB_THRESHOLD
    )

    red_light_detected = (
      filtered_uncertainty > self.RED_LIGHT_UNCERTAINTY_THRESHOLD and
      self._predictive_stop_prob > self.STOP_PROB_THRESHOLD * 0.9 and
      not stop_sign_detected
    )

    self._stop_sign_filter.add_data(1.0 if stop_sign_detected else 0.0)
    self._red_light_filter.add_data(1.0 if red_light_detected else 0.0)

    self._stop_sign_prob = self._stop_sign_filter.get_value() or 0.0
    self._red_light_prob = self._red_light_filter.get_value() or 0.0

    if self._stop_sign_prob > 0.5:
      result.signal_type = 'stop_sign'
      result.probability = self._stop_sign_prob
      result.distance = filtered_uncertainty * 2.0
    elif self._red_light_prob > 0.5:
      result.signal_type = 'red_light'
      result.probability = self._red_light_prob
      result.distance = filtered_uncertainty * 1.5
    else:
      result.signal_type = 'none'
      result.probability = 0.0
      result.distance = 0.0

    if self._predictive_stop_prob > 0.5 and v_ego > self.PRECHARGE_SPEED_THRESHOLD_MS:
      self._precharge_brake = True
      result.should_precharge_brake = True
    else:
      self._precharge_brake = False
      result.should_precharge_brake = False

    return result

  def get_stop_probability(self) -> float:
    """Returns the probability of an imminent stop."""
    return self._predictive_stop_prob

  def should_precharge(self) -> bool:
    """Returns True if brake pre-charging should be activated."""
    return self._precharge_brake

  def reset(self) -> None:
    """Reset all filter states."""
    self._stop_sign_prob = 0.0
    self._red_light_prob = 0.0
    self._predictive_stop_prob = 0.0
    self._precharge_brake = False
    self._uncertainty_stop_filter.reset_data()
    self._stop_sign_filter.reset_data()
    self._red_light_filter.reset_data()
    self._predictive_stop_filter.reset_data()


ModeType = Literal['acc', 'blended', 'pure_e2e']


class ModeTransitionManager:
  """Manages smooth transitions between driving modes with continuous weighting."""

  def __init__(self):
    self.current_mode: ModeType = 'acc'
    self.mode_confidence = {'acc': 1.0, 'blended': 0.0, 'pure_e2e': 0.0}
    self.blended_weight = 0.0
    self.transition_timeout = 0
    self.min_mode_duration = 2
    self.mode_duration = 0
    self.emergency_override = False

  def request_mode(self, mode: ModeType, confidence: float = 1.0, emergency: bool = False):
    self.mode_confidence[mode] = min(1.0, self.mode_confidence[mode] + 0.2 * confidence)
    for m in self.mode_confidence:
      if m != mode:
        self.mode_confidence[m] = max(0.0, self.mode_confidence[m] - 0.1)

    if emergency:
      self.emergency_override = True
      self.current_mode = mode
      self.transition_timeout = SET_MODE_TIMEOUT
      self.mode_duration = 0
      self.blended_weight = 1.0 if mode in ['blended', 'pure_e2e'] else 0.0
      return

    if self.mode_duration < self.min_mode_duration and not self.emergency_override:
      return

    confidence_threshold = 0.5 if mode != self.current_mode else 0.2

    if self.mode_confidence[mode] > confidence_threshold:
      if mode != self.current_mode and self.transition_timeout == 0:
        self.transition_timeout = SET_MODE_TIMEOUT
        self.current_mode = mode
        self.mode_duration = 0

  def update(self):
    if self.transition_timeout > 0:
      self.transition_timeout -= 1
    self.mode_duration += 1

    if self.emergency_override and self.mode_duration > 10:
      self.emergency_override = False

    for mode in self.mode_confidence:
      self.mode_confidence[mode] *= 0.995

    target_weight = 1.0 if self.current_mode in ['blended', 'pure_e2e'] else 0.0
    self.blended_weight = 0.9 * self.blended_weight + 0.1 * target_weight

  def get_mode(self) -> ModeType:
    return self.current_mode
  
  def get_weight(self) -> float:
    return float(self.blended_weight)


class DynamicExperimentalController:
  def __init__(self, CP: structs.CarParams, mpc, params=None):
    self._CP = CP
    self._mpc = mpc
    self._params = params or Params()
    self._enabled: bool = self._params.get_bool("DynamicExperimentalControl")
    self._active: bool = False
    self._frame: int = 0
    self._urgency = 0.0

    self._mode_manager = ModeTransitionManager()
    self._calibration_mae = 0.0
    self._calibration_uncertainty_offset = 0.0
    self._calibration_confidence = 1.0

    self._vision_traffic_auditor = VisionOnlyTrafficAuditor()

    self._lead_filter = SmoothKalmanFilter(
      measurement_noise=0.15,
      process_noise=0.05,
      alpha=1.02,
      smoothing_factor=0.8
    )

    self._slow_down_filter = SmoothKalmanFilter(
      measurement_noise=0.1,
      process_noise=0.1,
      alpha=1.05,
      smoothing_factor=0.7
    )

    self._slowness_filter = SmoothKalmanFilter(
      measurement_noise=0.1,
      process_noise=0.06,
      alpha=1.015,
      smoothing_factor=0.92
    )

    self._mpc_fcw_filter = SmoothKalmanFilter(
      measurement_noise=0.2,
      process_noise=0.1,
      alpha=1.1,
      smoothing_factor=0.5
    )

    self._uncertainty_filter = SmoothKalmanFilter(
      measurement_noise=0.15,
      process_noise=0.08,
      alpha=1.02,
      smoothing_factor=0.85
    )

    self._disengage_prob_filter = SmoothKalmanFilter(
      measurement_noise=0.1,
      process_noise=0.1,
      alpha=1.05,
      smoothing_factor=0.6
    )
    self._disengage_prob_future_filter = SmoothKalmanFilter(
      measurement_noise=0.12,
      process_noise=0.08,
      alpha=1.03,
      smoothing_factor=0.75
    )
    self._engaged_prob_filter = SmoothKalmanFilter(
      measurement_noise=0.1,
      process_noise=0.1,
      alpha=1.02,
      smoothing_factor=0.85
    )
    self._has_lead_filtered = False
    self._has_slow_down = False
    self._has_slowness = False
    self._has_mpc_fcw = False
    self._hard_brake_predicted = False
    self._v_ego_kph = 0.0
    self._v_cruise_kph = 0.0
    self._has_standstill = False
    self._mpc_fcw_crash_cnt = 0
    self._standstill_count = 0
    self._endpoint_x = float('inf')
    self._expected_distance = 0.0
    self._trajectory_valid = False
    self._vision_traffic_detection = VisionTrafficDetection()

  def _read_params(self) -> None:
    if self._frame % int(1. / DT_MDL) == 0:
      self._enabled = self._params.get_bool("DynamicExperimentalControl")

  def mode(self) -> str:
    return self._mode_manager.get_mode()

  def blended_weight(self) -> float:
    return self._mode_manager.get_weight()

  def calibration_mae(self) -> float:
    return self._calibration_mae

  def calibration_uncertainty_offset(self) -> float:
    return self._calibration_uncertainty_offset

  def calibration_confidence(self) -> float:
    return self._calibration_confidence

  def blended_confidence(self) -> float:
    return float(self._mode_manager.mode_confidence['blended'])

  def enabled(self) -> bool:
    return self._enabled

  def active(self) -> bool:
    return self._active

  def set_mpc_fcw_crash_cnt(self) -> None:
    self._mpc_fcw_crash_cnt = self._mpc.crash_cnt

  def vision_traffic_detection(self) -> VisionTrafficDetection:
    return self._vision_traffic_detection

  def should_precharge_brake(self) -> bool:
    return self._vision_traffic_detection.should_precharge_brake

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    car_state = sm['carState']
    lead_one = sm['radarState'].leadOne
    md = sm['modelV2']

    self._v_ego_kph = car_state.vEgo * 3.6
    self._v_cruise_kph = car_state.vCruise
    self._has_standstill = car_state.standstill

    if self._has_standstill:
      self._standstill_count = min(20, self._standstill_count + 1)
    else:
      self._standstill_count = max(0, self._standstill_count - 1)

    self._lead_filter.add_data(float(lead_one.status))
    lead_value = self._lead_filter.get_value() or 0.0
    self._has_lead_filtered = lead_value > WMACConstants.LEAD_PROB

    fcw_filtered_value = self._mpc_fcw_filter.get_value() or 0.0
    self._mpc_fcw_filter.add_data(float(self._mpc_fcw_crash_cnt > 0))
    self._has_mpc_fcw = fcw_filtered_value > 0.5

    uncertainty_array = np.zeros(TRAJECTORY_SIZE)
    disengage_probs_array = np.zeros(TRAJECTORY_SIZE)

    if len(md.position.xStd) == TRAJECTORY_SIZE:
      longitudinal_uncertainty = float(md.position.xStd[TRAJECTORY_SIZE - 1])
      uncertainty_array = np.array(md.position.xStd, dtype=np.float32)
      self._uncertainty_filter.add_data(longitudinal_uncertainty)

    if len(md.meta.disengagePredictions.brakeDisengageProbs) > 0:
      brake_prob = float(md.meta.disengagePredictions.brakeDisengageProbs[0])
      self._disengage_prob_filter.add_data(brake_prob)

      max_brake_prob = float(max(md.meta.disengagePredictions.brakeDisengageProbs))
      self._disengage_prob_future_filter.add_data(max_brake_prob)
      disengage_probs_array = np.array(md.meta.disengagePredictions.brakeDisengageProbs, dtype=np.float32)

    self._engaged_prob_filter.add_data(float(md.meta.engagedProb))
    self._hard_brake_predicted = bool(md.meta.hardBrakePredicted)

    self._vision_traffic_detection = self._vision_traffic_auditor.update(
      uncertainty_array, disengage_probs_array, car_state.vEgo
    )

    self._calculate_slow_down(md)

    if not (self._standstill_count > 5) and not self._has_slow_down:
      current_slowness = float(self._v_ego_kph <= (self._v_cruise_kph * WMACConstants.SLOWNESS_CRUISE_OFFSET))
      self._slowness_filter.add_data(current_slowness)
      slowness_value = self._slowness_filter.get_value() or 0.0

      threshold = WMACConstants.SLOWNESS_PROB * (0.8 if self._has_slowness else 1.1)
      self._has_slowness = slowness_value > threshold

  def _calculate_slow_down(self, md):
    urgency = 0.0
    self._endpoint_x = float('inf')
    self._trajectory_valid = False

    position_valid = len(md.position.x) == TRAJECTORY_SIZE
    orientation_valid = len(md.orientation.x) == TRAJECTORY_SIZE

    if not (position_valid and orientation_valid):
      if self._v_ego_kph > 20.0:
        urgency = 0.3

      self._slow_down_filter.add_data(urgency)
      urgency_filtered = self._slow_down_filter.get_value() or 0.0
      self._has_slow_down = urgency_filtered > WMACConstants.SLOW_DOWN_PROB
      self._urgency = urgency_filtered
      return

    self._trajectory_valid = True

    endpoint_x = md.position.x[TRAJECTORY_SIZE - 1]
    self._endpoint_x = endpoint_x

    expected_distance = np.interp(self._v_ego_kph,
                               WMACConstants.SLOW_DOWN_BP,
                               WMACConstants.SLOW_DOWN_DIST)
    self._expected_distance = expected_distance

    if endpoint_x < expected_distance:
      shortage = expected_distance - endpoint_x
      shortage_ratio = shortage / expected_distance

      urgency = min(1.0, shortage_ratio * 2.0)

      critical_distance = expected_distance * 0.3
      if endpoint_x < critical_distance:
        urgency = min(1.0, urgency * 2.0)

      if self._v_ego_kph > 25.0:
        speed_factor = 1.0 + (self._v_ego_kph - 25.0) / 80.0
        urgency = min(1.0, urgency * speed_factor)

    uncertainty_filtered = self._uncertainty_filter.get_value() or 0.0
    brake_prob_filtered = self._disengage_prob_filter.get_value() or 0.0
    future_brake_prob_filtered = self._disengage_prob_future_filter.get_value() or 0.0

    v_ego = self._v_ego_kph / 3.6
    dist_comfortable = (v_ego**2) / (2 * 2.5)
    dist_hard = (v_ego**2) / (2 * 4.0)
    
    kinematic_urgency = 0.0
    if self._trajectory_valid and v_ego > 3.0:
      if self._endpoint_x < dist_hard:
        kinematic_urgency = 1.0
      elif self._endpoint_x < dist_comfortable:
        kinematic_urgency = np.interp(self._endpoint_x, [dist_hard, dist_comfortable], [1.0, 0.2])

    velocity_urgency = 0.0
    if len(md.velocity.x) > 0:
      v_ego_model = md.velocity.x[0]
      v_diff = max(0.0, (self._v_ego_kph / 3.6) - v_ego_model)
      velocity_urgency = min(1.0, v_diff / 4.0)

    curve_urgency = 0.0
    if len(md.orientationRate.z) > 0:
      yaw_rate_model = abs(md.orientationRate.z[0])
      curve_urgency = min(1.0, max(0.0, yaw_rate_model - 0.05) * 2.0)

    uncertainty_urgency = min(1.0, max(0.0, (uncertainty_filtered - 4.0) / 12.0))
    brake_prob_urgency = min(1.0, brake_prob_filtered * 2.0)
    future_brake_urgency = min(1.0, future_brake_prob_filtered * 1.5)

    engaged_prob = self._engaged_prob_filter.get_value() or 0.0
    
    hard_brake_urgency = 1.0 if self._hard_brake_predicted else 0.0

    vision_traffic_urgency = 0.0
    if self._vision_traffic_detection.signal_type != 'none':
      vision_traffic_urgency = self._vision_traffic_detection.probability * 0.9

    intents = np.array([urgency, uncertainty_urgency, brake_prob_urgency,
                        future_brake_urgency, velocity_urgency, curve_urgency,
                        hard_brake_urgency, kinematic_urgency, vision_traffic_urgency])
    
    weights = np.ones_like(intents)
    weights[:6] *= (0.5 + 0.5 * engaged_prob) * self._calibration_confidence
    weights[6] = 1.0
    weights[7] = 1.0
    weights[8] = 0.8

    weighted_sum_sq = np.sum(np.square(intents) * weights)
    combined_urgency = float(np.sqrt(weighted_sum_sq / np.sum(weights) * len(intents)))
    combined_urgency = min(1.0, combined_urgency)

    self._slow_down_filter.add_data(combined_urgency)
    urgency_filtered = self._slow_down_filter.get_value() or 0.0

    model_confidence = self._uncertainty_filter.get_confidence()
    dynamic_threshold = WMACConstants.SLOW_DOWN_PROB * (1.2 - 0.4 * model_confidence)
    dynamic_threshold *= (1.5 - 0.5 * self._calibration_confidence)
    
    self._has_slow_down = urgency_filtered > dynamic_threshold
    self._urgency = urgency_filtered

  def _radarless_mode(self) -> None:
    if self._has_mpc_fcw:
      self._mode_manager.request_mode('blended', confidence=1.0, emergency=True)
      return

    engaged_prob = self._engaged_prob_filter.get_value() or 0.0
    if self._calibration_confidence > 0.85 and engaged_prob > 0.92:
      self._mode_manager.request_mode('pure_e2e', confidence=0.8)
      return

    if self._standstill_count > 3:
      self._mode_manager.request_mode('blended', confidence=0.9)
      return

    if self._vision_traffic_detection.signal_type in ('stop_sign', 'red_light'):
      if self._vision_traffic_detection.probability > 0.6:
        self._mode_manager.request_mode('blended', confidence=self._vision_traffic_detection.probability, emergency=True)
        return

    if self._has_slow_down:
      if self._urgency > 0.7:
        self._mode_manager.request_mode('blended', confidence=1.0, emergency=True)
      else:
        confidence = min(1.0, self._urgency * 1.5)
        self._mode_manager.request_mode('blended', confidence=confidence)
      return

    if self._has_slowness and not self._has_slow_down:
      self._mode_manager.request_mode('acc', confidence=0.8)
      return

    self._mode_manager.request_mode('acc', confidence=0.7)

  def _radar_mode(self) -> None:
    if self._has_mpc_fcw:
      self._mode_manager.request_mode('blended', confidence=1.0, emergency=True)
      return

    engaged_prob = self._engaged_prob_filter.get_value() or 0.0
    if self._calibration_confidence > 0.85 and engaged_prob > 0.95:
      self._mode_manager.request_mode('pure_e2e', confidence=0.8)
      return

    if self._has_lead_filtered and not (self._standstill_count > 3):
      if self._has_slow_down and self._urgency > 0.6:
        self._mode_manager.request_mode('blended', confidence=self._urgency)
      else:
        self._mode_manager.request_mode('acc', confidence=1.0)
      return

    if self._vision_traffic_detection.signal_type in ('stop_sign', 'red_light'):
      if self._vision_traffic_detection.probability > 0.55:
        self._mode_manager.request_mode('blended', confidence=self._vision_traffic_detection.probability)
        return

    if self._has_slow_down:
      if self._urgency > 0.7:
        self._mode_manager.request_mode('blended', confidence=1.0, emergency=True)
      else:
        confidence = min(1.0, self._urgency * 1.3)
        self._mode_manager.request_mode('blended', confidence=confidence)
      return

    if self._standstill_count > 3:
      self._mode_manager.request_mode('blended', confidence=0.9)
      return

    if self._has_slowness and not self._has_slow_down:
      self._mode_manager.request_mode('acc', confidence=0.8)
      return

    self._mode_manager.request_mode('acc', confidence=0.7)

  def update(self, sm: messaging.SubMaster) -> None:
    self._read_params()

    self.set_mpc_fcw_crash_cnt()

    self._update_calculations(sm)

    if self._CP.radarUnavailable:
      self._radarless_mode()
    else:
      self._radar_mode()

    self._mode_manager.update()
    self._active = sm['selfdriveState'].experimentalMode and self._enabled
    self._frame += 1
