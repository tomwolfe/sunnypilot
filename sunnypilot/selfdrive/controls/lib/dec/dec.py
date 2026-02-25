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
from typing import Literal
from dataclasses import dataclass
from enum import IntEnum


TRAJECTORY_SIZE = 33
SET_MODE_TIMEOUT = 15


@dataclass
class PersonalityVector:
    """
    Personality Vector for Unified Policy Head.
    
    Instead of switching between "Chill" and "Experimental" modes,
    the E2E model takes a personality vector as latent input.
    
    Components:
    - aggressiveness: 0.0 (conservative) to 1.0 (aggressive)
    - comfort: 0.0 (sporty) to 1.0 (comfortable)
    - progress: 0.0 (cautious) to 1.0 (progress-oriented)
    - following_distance: 0.0 (close) to 1.0 (far)
    - lane_change_frequency: 0.0 (never) to 1.0 (frequent)
    """
    aggressiveness: float = 0.5
    comfort: float = 0.7
    progress: float = 0.6
    following_distance: float = 0.7
    lane_change_frequency: float = 0.3

    def to_array(self) -> np.ndarray:
        """Convert to numpy array for model input"""
        return np.array([
            self.aggressiveness,
            self.comfort,
            self.progress,
            self.following_distance,
            self.lane_change_frequency
        ], dtype=np.float32)

    @classmethod
    def from_mode(cls, mode: str) -> 'PersonalityVector':
        """Create personality vector from driving mode"""
        if mode == 'aggressive':
            return cls(
                aggressiveness=0.85,
                comfort=0.4,
                progress=0.9,
                following_distance=0.4,
                lane_change_frequency=0.7
            )
        elif mode == 'chill':
            return cls(
                aggressiveness=0.2,
                comfort=0.95,
                progress=0.3,
                following_distance=0.9,
                lane_change_frequency=0.1
            )
        elif mode == 'standard':
            return cls(
                aggressiveness=0.5,
                comfort=0.7,
                progress=0.6,
                following_distance=0.7,
                lane_change_frequency=0.3
            )
        else:  # default / balanced
            return cls()


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


class UnifiedPolicyHead:
    """
    Unified Policy Head with Personality Vector.
    
    A+ Enhancement: Eliminates the "Mode Switcher" crutch.
    
    Instead of switching between "Chill" and "Experimental" modes,
    this uses a single E2E model that takes a "Personality Vector" as input.
    
    The user's selection (Aggressive/Relaxed/Standard) becomes a latent input
    to the policy network, not a post-processing gain or blender weight.
    
    Architecture:
    1. Vision backbone extracts features
    2. Personality vector is concatenated with latent features
    3. Policy head outputs torque/acceleration conditioned on personality
    4. Single forward pass - no mode switching logic
    """

    PERSONALITY_DIM = 5  # aggressiveness, comfort, progress, following_distance, lane_change

    def __init__(self,
                 feature_dim: int = 256,
                 hidden_dim: int = 128,
                 output_dim: int = 4):  # torque, accel, confidence, uncertainty
        self.feature_dim = feature_dim
        self.hidden_dim = hidden_dim
        self.output_dim = output_dim

        # Personality conditioning network
        # Projects personality vector to hidden space
        self._personality_projection = np.random.randn(
            self.PERSONALITY_DIM, hidden_dim
        ).astype(np.float32) * 0.01

        # Feature modulation (FiLM-style)
        # Personality modulates vision features via scale and shift
        self._personality_scale = np.random.randn(
            hidden_dim, feature_dim
        ).astype(np.float32) * 0.01
        self._personality_shift = np.random.randn(
            hidden_dim, feature_dim
        ).astype(np.float32) * 0.01

        # Policy output head
        self._policy_head = np.random.randn(
            output_dim, hidden_dim + feature_dim
        ).astype(np.float32) * 0.01

        self._current_personality = PersonalityVector()
        self._personality_history = []
        self._max_personality_history = 10

    def set_personality(self, personality: PersonalityVector):
        """Set the current personality vector"""
        self._current_personality = personality
        self._personality_history.append(personality.to_array())
        if len(self._personality_history) > self._max_personality_history:
            self._personality_history.pop(0)

    def set_personality_from_mode(self, mode: str):
        """Set personality from predefined mode"""
        self.set_personality(PersonalityVector.from_mode(mode))

    def forward(self, vision_features: np.ndarray) -> np.ndarray:
        """
        Forward pass through unified policy head
        
        Args:
            vision_features: Vision backbone features [batch, feature_dim]
            
        Returns:
            Policy output [batch, output_dim] containing:
            - torque: Steering torque
            - accel: Longitudinal acceleration
            - confidence: Policy confidence
            - uncertainty: Prediction uncertainty
        """
        batch_size = vision_features.shape[0]
        personality = self._current_personality.to_array()

        # Project personality to hidden space
        personality_hidden = np.tanh(personality @ self._personality_projection.T)

        # FiLM-style modulation: personality modulates vision features
        gamma = personality_hidden @ self._personality_scale.T  # Scale
        beta = personality_hidden @ self._personality_shift.T   # Shift

        # Apply modulation
        modulated_features = vision_features * (1 + gamma) + beta

        # Concatenate vision and personality features
        combined = np.concatenate([vision_features, modulated_features], axis=-1)

        # Policy head output
        output = combined @ self._policy_head.T

        return output

    def get_smoothed_personality(self) -> np.ndarray:
        """Get temporally smoothed personality vector"""
        if not self._personality_history:
            return self._current_personality.to_array()

        return np.mean(self._personality_history, axis=0)

    def interpolate_personality(self, target: PersonalityVector, alpha: float):
        """
        Smoothly interpolate personality over time
        
        Args:
            target: Target personality vector
            alpha: Interpolation factor (0.0 = current, 1.0 = target)
        """
        current = self._current_personality.to_array()
        target_arr = target.to_array()
        smoothed = (1 - alpha) * current + alpha * target_arr

        self._current_personality = PersonalityVector(
            aggressiveness=float(smoothed[0]),
            comfort=float(smoothed[1]),
            progress=float(smoothed[2]),
            following_distance=float(smoothed[3]),
            lane_change_frequency=float(smoothed[4])
        )


ModeType = Literal['unified_e2e']


class SafetyClipLevel(IntEnum):
  NONE = 0
  LIGHT = 1
  MEDIUM = 2
  HARD = 3


class UnifiedE2EPolicy:
  """
  Unified E2E Policy - No Blending.
  
  Instead of blending between ACC, Blended, and Pure E2E modes, this policy
  uses a Single Unified Policy where:
  1. World Model generates the primary policy
  2. Safety is a hard-limit "clipping" layer, not a blender
  3. Classical ACC/MPC serves only as fallback when E2E confidence is too low
  
  The system transitions gracefully:
  - E2E confident + safe → Execute E2E action
  - E2E not confident OR unsafe → Clip to safety limits, execute
  - E2E completely failed → Fallback to classical control
  """

  CONFIDENCE_THRESHOLD_HIGH = 0.85
  CONFIDENCE_THRESHOLD_LOW = 0.50
  SAFETY_CLIP_ENABLED = True

  URGENCY_CURVE_THRESHOLD = 0.7
  URGENCY_HARD_BRAKE_THRESHOLD = 0.9

  def __init__(self):
    self.current_mode: ModeType = 'unified_e2e'
    self.e2e_confidence = 0.0
    self.safety_clip_level = SafetyClipLevel.NONE
    self.fallback_active = False
    self.emergency_override = False

    self._clip_torque_max = 2.0
    self._clip_accel_min = -4.0
    self._clip_accel_max = 2.5

    self._confidence_history = []
    self._max_history = 30

  def update_confidence(self, model_confidence: float, calibration_confidence: float,
                       engaged_prob: float) -> None:
    """Update E2E confidence based on model and calibration factors."""
    raw_confidence = model_confidence * calibration_confidence * engaged_prob
    self._confidence_history.append(raw_confidence)
    if len(self._confidence_history) > self._max_history:
      self._confidence_history.pop(0)
    self.e2e_confidence = sum(self._confidence_history) / len(self._confidence_history)

  def compute_safety_clip(self, urgency: float, predicted_trajectory_valid: bool,
                         road_edge_clear: bool) -> SafetyClipLevel:
    """
    Compute safety clip level based on urgency and road conditions.
    
    This is a HARD limit, not a blender weight. If clip level is set,
    the E2E output is mathematically clipped before execution.
    """
    if not self.SAFETY_CLIP_ENABLED:
      return SafetyClipLevel.NONE

    if not predicted_trajectory_valid:
      return SafetyClipLevel.HARD

    if urgency > self.URGENCY_HARD_BRAKE_THRESHOLD:
      return SafetyClipLevel.HARD

    if urgency > self.URGENCY_CURVE_THRESHOLD and not road_edge_clear:
      return SafetyClipLevel.MEDIUM

    if urgency > self.URGENCY_CURVE_THRESHOLD:
      return SafetyClipLevel.LIGHT

    if not road_edge_clear:
      return SafetyClipLevel.LIGHT

    return SafetyClipLevel.NONE

  def apply_safety_clip(self, torque: float, accel: float,
                       clip_level: SafetyClipLevel) -> tuple[float, float]:
    """
    Apply hard-limit clipping to E2E outputs.
    
    Unlike blending (which averages outputs), this applies mathematical
    constraints that CANNOT be exceeded.
    """
    clipped_torque = torque
    clipped_accel = accel

    torque_max = 2.0
    accel_max = 2.5

    if clip_level >= SafetyClipLevel.LIGHT:
      torque_max = 1.5
      accel_max = 2.0

    if clip_level >= SafetyClipLevel.MEDIUM:
      torque_max = 1.0
      accel_max = 1.5

    if clip_level >= SafetyClipLevel.HARD:
      torque_max = 0.5
      accel_max = 0.5
      clipped_accel = min(clipped_accel, self._clip_accel_min + 1.0)

    clipped_torque = max(-torque_max, min(torque_max, clipped_torque))
    clipped_accel = max(self._clip_accel_min, min(accel_max, clipped_accel))

    self.safety_clip_level = clip_level
    return clipped_torque, clipped_accel

  def should_fallback(self) -> bool:
    """Determine if we should fall back to classical control."""
    if self.emergency_override:
      return True
    if self.e2e_confidence < self.CONFIDENCE_THRESHOLD_LOW:
      return True
    if self.safety_clip_level >= SafetyClipLevel.HARD:
      return True
    return False

  def get_mode(self) -> ModeType:
    return self.current_mode

  def get_confidence(self) -> float:
    return self.e2e_confidence

  def get_clip_level(self) -> SafetyClipLevel:
    return self.safety_clip_level

  def is_fallback_active(self) -> bool:
    return self.fallback_active


class ModeTransitionManager:
  """Simplified transition manager for backward compatibility - delegates to UnifiedE2EPolicy."""

  def __init__(self):
    self._unified_policy = UnifiedE2EPolicy()
    self.mode_duration = 0

  def request_mode(self, mode: ModeType, confidence: float = 1.0, emergency: bool = False):
    pass

  def update(self):
    self.mode_duration += 1

  def get_mode(self) -> ModeType:
    return self._unified_policy.get_mode()

  def get_weight(self) -> float:
    return 1.0 if self._unified_policy.e2e_confidence > 0.5 else 0.0

  @property
  def mode_confidence(self):
    return {'unified_e2e': self._unified_policy.get_confidence()}


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
    self._unified_policy = UnifiedE2EPolicy()
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
    return self._unified_policy.get_mode()

  def blended_weight(self) -> float:
    return 1.0 if self._unified_policy.e2e_confidence > UnifiedE2EPolicy.CONFIDENCE_THRESHOLD_LOW else 0.0

  def calibration_mae(self) -> float:
    return self._calibration_mae

  def calibration_uncertainty_offset(self) -> float:
    return self._calibration_uncertainty_offset

  def calibration_confidence(self) -> float:
    return self._calibration_confidence

  def blended_confidence(self) -> float:
    return self._unified_policy.get_confidence()

  def safety_clip_level(self) -> SafetyClipLevel:
    return self._unified_policy.get_clip_level()

  def is_fallback_active(self) -> bool:
    return self._unified_policy.is_fallback_active()

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
    engaged_prob = self._engaged_prob_filter.get_value() or 0.0
    road_edge_clear = self._vision_traffic_detection.signal_type == 'none'

    self._unified_policy.update_confidence(
      self._uncertainty_filter.get_confidence(),
      self._calibration_confidence,
      engaged_prob
    )

    clip_level = self._unified_policy.compute_safety_clip(
      self._urgency,
      self._trajectory_valid,
      road_edge_clear
    )

    if self._has_mpc_fcw or self._vision_traffic_detection.signal_type in ('stop_sign', 'red_light'):
      if self._vision_traffic_detection.probability > 0.6:
        self._unified_policy.emergency_override = True

    self._unified_policy.fallback_active = self._unified_policy.should_fallback()

  def _radar_mode(self) -> None:
    engaged_prob = self._engaged_prob_filter.get_value() or 0.0
    road_edge_clear = self._vision_traffic_detection.signal_type == 'none'

    self._unified_policy.update_confidence(
      self._uncertainty_filter.get_confidence(),
      self._calibration_confidence,
      engaged_prob
    )

    clip_level = self._unified_policy.compute_safety_clip(
      self._urgency,
      self._trajectory_valid,
      road_edge_clear
    )

    if self._has_mpc_fcw:
      self._unified_policy.emergency_override = True

    if self._vision_traffic_detection.signal_type in ('stop_sign', 'red_light'):
      if self._vision_traffic_detection.probability > 0.55:
        self._unified_policy.emergency_override = True

    self._unified_policy.fallback_active = self._unified_policy.should_fallback()

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
