"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from collections import deque
from openpilot.common.realtime import DT_CTRL


class AdaptivePersonality:
  """
  Tracks user driving behavior to auto-adjust the neural personality.
  Learns from manual overrides to shift between Aggressive/Standard/Relaxed.
  """
  AGGRESSIVE = 2
  STANDARD = 1
  RELAXED = 0

  def __init__(self, window_s=60.0):
    self.window_steps = int(window_s / DT_CTRL)

    self.override_history = deque(maxlen=self.window_steps)
    self.late_brake_history = deque(maxlen=self.window_steps)
    self.early_accel_history = deque(maxlen=self.window_steps)

    self.current_personality = self.STANDARD
    self.personality_confidence = 0.5

    self.override_threshold = 0.3
    self.personality_switch_cooldown = 0

  def add_override(self, is_override: bool):
    """Record if user is overriding the neural control."""
    self.override_history.append(1.0 if is_override else 0.0)
    self._update_personality()

  def add_late_brake(self, is_late_brake: bool):
    """Record if user is braking later than expected."""
    self.late_brake_history.append(1.0 if is_late_brake else 0.0)

  def add_early_accel(self, is_early_accel: bool):
    """Record if user is accelerating earlier than expected."""
    self.early_accel_history.append(1.0 if is_early_accel else 0.0)

  def _update_personality(self):
    if self.personality_switch_cooldown > 0:
      self.personality_switch_cooldown -= 1
      return

    if len(self.override_history) < self.window_steps // 2:
      return

    override_rate = np.mean(self.override_history)
    late_brake_rate = np.mean(self.late_brake_history) if len(self.late_brake_history) > 0 else 0
    early_accel_rate = np.mean(self.early_accel_history) if len(self.early_accel_history) > 0 else 0

    aggressive_score = late_brake_rate * 0.4 + early_accel_rate * 0.4 - override_rate * 0.2
    relaxed_score = override_rate * 0.5 - late_brake_rate * 0.3 - early_accel_rate * 0.2

    new_personality = self.current_personality
    confidence = 0.0

    if aggressive_score > 0.3 and self.current_personality < self.AGGRESSIVE:
      new_personality = min(self.AGGRESSIVE, self.current_personality + 1)
      confidence = min(1.0, aggressive_score)
    elif relaxed_score > 0.3 and self.current_personality > self.RELAXED:
      new_personality = max(self.RELAXED, self.current_personality - 1)
      confidence = min(1.0, relaxed_score)

    if new_personality != self.current_personality:
      if confidence > 0.5:
        self.current_personality = new_personality
        self.personality_confidence = confidence
        self.personality_switch_cooldown = 300
        self._reset_histories()

  def _reset_histories(self):
    self.override_history.clear()
    self.late_brake_history.clear()
    self.early_accel_history.clear()

  def get_personality(self) -> int:
    """Returns current personality (0=Relaxed, 1=Standard, 2=Aggressive)."""
    return self.current_personality

  def get_confidence(self) -> float:
    """Returns confidence in the current personality (0.0 to 1.0)."""
    return self.personality_confidence

  def reset(self):
    self.override_history.clear()
    self.late_brake_history.clear()
    self.early_accel_history.clear()
    self.current_personality = self.STANDARD
    self.personality_confidence = 0.5
    self.personality_switch_cooldown = 0


class LongitudinalCalibrator:
  """
  Online calibrator for neural longitudinal uncertainty.
  Compares model's predicted acceleration to actual aEgo over a window.
  """

  def __init__(self, delay_s=0.25, window_s=30.0):
    self.delay_steps = int(delay_s / DT_CTRL)
    self.window_steps = int(window_s / DT_CTRL)

    self.adaptive_personality = AdaptivePersonality()

    self.accel_neural_history = deque(maxlen=self.delay_steps + 1)
    self.errors = deque(maxlen=self.window_steps)

    self.calibrated_uncertainty_offset = 0.0
    self.mae = 0.0
    self.confidence = 1.0
    self.active = False

    self.a_ego_history = deque(maxlen=self.delay_steps + 1)

  def update(self, accel_neural, a_ego, use_neural):
    """
    Update the calibrator with new data.
    accel_neural: model's predicted acceleration (m/s^2)
    a_ego: actual car acceleration (m/s^2)
    use_neural: whether the neural model is currently in control
    """
    self.active = use_neural

    self.accel_neural_history.append(accel_neural)
    self.a_ego_history.append(a_ego)

    is_override = False
    if use_neural and len(self.accel_neural_history) > self.delay_steps:
      delayed_accel = self.accel_neural_history[0]

      error = abs(delayed_accel - a_ego)

      if self.active:
        self.errors.append(error)

      if error > 1.5:
        is_override = True

      if len(self.errors) > 100:
        self.mae = float(np.mean(self.errors))

        self.calibrated_uncertainty_offset = self.mae * 2.0

        mae_confidence = max(0.0, 1.0 - (self.mae / 0.8))

        error_std = float(np.std(self.errors))
        stability_confidence = max(0.0, 1.0 - (error_std / 0.5))

        self.confidence = 0.7 * mae_confidence + 0.3 * stability_confidence
      else:
        self.calibrated_uncertainty_offset = 0.0
        self.confidence = 1.0

    self.adaptive_personality.add_override(is_override)

    return self.calibrated_uncertainty_offset

  def update_personality_from_override(self, a_ego: float, accel_neural: float, gas_pressed: float, brake_pressed: float):
    """
    Update personality based on user driving behavior.

    Args:
      a_ego: actual acceleration
      accel_neural: model predicted acceleration
      gas_pressed: gas pedal position (0-1)
      brake_pressed: brake pedal position (0-1)
    """
    if gas_pressed > 0.1 and accel_neural < a_ego - 0.5:
      self.adaptive_personality.add_early_accel(True)
    else:
      self.adaptive_personality.add_early_accel(False)

    if brake_pressed > 0.1 and accel_neural > a_ego + 0.5:
      self.adaptive_personality.add_late_brake(True)
    else:
      self.adaptive_personality.add_late_brake(False)

  def get_personality(self) -> int:
    """Returns the current adaptive personality level."""
    return self.adaptive_personality.get_personality()

  def get_personality_confidence(self) -> float:
    """Returns confidence in the current personality."""
    return self.adaptive_personality.get_confidence()

  def reset(self):
    self.accel_neural_history.clear()
    self.errors.clear()
    self.calibrated_uncertainty_offset = 0.0
    self.mae = 0.0
    self.confidence = 1.0
    self.a_ego_history.clear()
    self.adaptive_personality.reset()
