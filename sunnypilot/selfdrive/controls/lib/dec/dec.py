"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
# Version = 2025-6-30

from cereal import messaging
from opendbc.car import structs
from numpy import interp
import math
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.dec.constants import WMACConstants
from openpilot.system.statsd import statlog
from typing import Literal
import time

# d-e2e, from modeldata.h
TRAJECTORY_SIZE = 33
SET_MODE_TIMEOUT = 15

# Define the valid mode types
ModeType = Literal['acc', 'blended']


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


class ModeTransitionManager:
  """Manages smooth transitions between driving modes with hysteresis."""

  def __init__(self):
    self.current_mode: ModeType = 'acc'
    self.mode_confidence = {'acc': 1.0, 'blended': 0.0}
    self.transition_timeout = 0
    self.min_mode_duration = 10
    self.mode_duration = 0
    self.emergency_override = False

  def request_mode(self, mode: ModeType, confidence: float = 1.0, emergency: bool = False):
    # Emergency override for critical situations (stops, collisions)
    if emergency:
      self.emergency_override = True
      self.current_mode = mode
      self.transition_timeout = SET_MODE_TIMEOUT
      self.mode_duration = 0
      return

    self.mode_confidence[mode] = min(1.0, self.mode_confidence[mode] + 0.1 * confidence)
    for m in self.mode_confidence:
      if m != mode:
        self.mode_confidence[m] = max(0.0, self.mode_confidence[m] - 0.05)

    # Require minimum duration in current mode (unless emergency)
    if self.mode_duration < self.min_mode_duration and not self.emergency_override:
      return

    # Hysteresis: higher threshold for mode changes
    confidence_threshold = 0.6 if mode != self.current_mode else 0.3  # Lower threshold for faster response

    if self.mode_confidence[mode] > confidence_threshold:
      if mode != self.current_mode and self.transition_timeout == 0:
        self.transition_timeout = SET_MODE_TIMEOUT
        self.current_mode = mode
        self.mode_duration = 0

  def update(self):
    if self.transition_timeout > 0:
      self.transition_timeout -= 1
    self.mode_duration += 1

    # Reset emergency override after some time
    if self.emergency_override and self.mode_duration > 20:
      self.emergency_override = False

    # Gradual confidence decay
    for mode in self.mode_confidence:
      self.mode_confidence[mode] *= 0.98

  def get_mode(self) -> ModeType:
    return self.current_mode


class DynamicExperimentalController:
  def __init__(self, CP: structs.CarParams, mpc, params=None):
    self._CP = CP
    self._mpc = mpc
    self._params = params or Params()
    self._enabled: bool = self._params.get_bool("DynamicExperimentalControl")
    self._active: bool = False
    self._frame: int = 0
    self._urgency = 0.0

    # Monitoring variables
    self._last_mode: ModeType | None = None
    self._mode_switch_count = 0
    self._mode_timer = {'acc': 0, 'blended': 0}
    self._last_mode_change_time = time.monotonic()
    self._monitoring_enabled = True
    self._monitoring_frame_counter = 0
    self._monitoring_interval = int(1.0 / DT_MDL)  # Send metrics every second

    self._mode_manager = ModeTransitionManager()

    # Smooth filters for stable decision making with faster response for critical scenarios
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
    self._has_lead_filtered = False
    self._has_slow_down = False
    self._has_slowness = False
    self._has_mpc_fcw = False
    self._v_ego_kph = 0.0
    self._v_cruise_kph = 0.0
    self._has_standstill = False
    self._mpc_fcw_crash_cnt = 0
    self._standstill_count = 0
    # debug
    self._endpoint_x = float('inf')
    self._expected_distance = 0.0
    self._trajectory_valid = False

  def _read_params(self) -> None:
    if self._frame % int(1. / DT_MDL) == 0:
      self._enabled = self._params.get_bool("DynamicExperimentalControl")

  def mode(self) -> str:
    return self._mode_manager.get_mode()

  def enabled(self) -> bool:
    return self._enabled

  def active(self) -> bool:
    return self._active

  def set_mpc_fcw_crash_cnt(self) -> None:
    """Set MPC FCW crash count"""
    self._mpc_fcw_crash_cnt = self._mpc.crash_cnt

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    car_state = sm['carState']
    lead_one = sm['radarState'].leadOne
    md = sm['modelV2']

    self._v_ego_kph = car_state.vEgo * 3.6
    self._v_cruise_kph = car_state.vCruise
    self._has_standstill = car_state.standstill

    # standstill detection
    if self._has_standstill:
      self._standstill_count = min(20, self._standstill_count + 1)
    else:
      self._standstill_count = max(0, self._standstill_count - 1)

    # Lead detection
    self._lead_filter.add_data(float(lead_one.status))
    lead_value = self._lead_filter.get_value() or 0.0
    self._has_lead_filtered = lead_value > WMACConstants.LEAD_PROB

    # MPC FCW detection
    fcw_filtered_value = self._mpc_fcw_filter.get_value() or 0.0
    self._mpc_fcw_filter.add_data(float(self._mpc_fcw_crash_cnt > 0))
    self._has_mpc_fcw = fcw_filtered_value > 0.5

    # Slow down detection
    self._calculate_slow_down(md)

    # Slowness detection
    if not (self._standstill_count > 5) and not self._has_slow_down:
      current_slowness = float(self._v_ego_kph <= (self._v_cruise_kph * WMACConstants.SLOWNESS_CRUISE_OFFSET))
      self._slowness_filter.add_data(current_slowness)
      slowness_value = self._slowness_filter.get_value() or 0.0

      # Hysteresis for slowness
      threshold = WMACConstants.SLOWNESS_PROB * (0.8 if self._has_slowness else 1.1)
      self._has_slowness = slowness_value > threshold

  def _calculate_slow_down(self, md):
    """Calculate urgency based on trajectory endpoint vs expected distance."""

    # Reset to safe defaults
    urgency = 0.0
    self._endpoint_x = float('inf')
    self._trajectory_valid = False

    # Validate trajectory data - check if we have the expected size
    position_valid = len(md.position.x) == TRAJECTORY_SIZE if hasattr(md.position, 'x') else False
    orientation_valid = len(md.orientation.x) == TRAJECTORY_SIZE if hasattr(md.orientation, 'x') else False

    # Additional safety checks for trajectory data validity
    if (position_valid and hasattr(md.position, 'x') and
        len(md.position.x) > 0):

      # Check for NaN or infinity values in the trajectory data
      try:
        for x in md.position.x:
          if math.isnan(x) or math.isinf(x):
            position_valid = False
            break
      except (TypeError, ValueError):
        # If there's an issue with checking for NaN/inf, mark as invalid
        position_valid = False

      # Check if the endpoint value is reasonable (not negative or extremely large)
      if position_valid:
        endpoint_x = md.position.x[TRAJECTORY_SIZE - 1]
        if not (endpoint_x >= 0 and endpoint_x < 1000):  # Reasonable range for trajectory endpoint
          position_valid = False

    if not (position_valid and orientation_valid):
      # Invalid trajectory - this itself might indicate a stop scenario
      # Apply more conservative urgency for incomplete trajectories at speed
      if self._v_ego_kph > 5.0:  # Even at low speeds, consider possible stop
        urgency = 0.6

      self._slow_down_filter.add_data(urgency)
      urgency_filtered = self._slow_down_filter.get_value() or 0.0
      self._has_slow_down = urgency_filtered > WMACConstants.SLOW_DOWN_PROB
      self._urgency = urgency_filtered
      return

    # We have a valid full trajectory
    self._trajectory_valid = True

    # Use the exact endpoint (33rd point, index 32)
    endpoint_x = md.position.x[TRAJECTORY_SIZE - 1]
    self._endpoint_x = endpoint_x

    # Get expected distance based on current speed using tuned constants
    expected_distance = interp(self._v_ego_kph,
                               WMACConstants.SLOW_DOWN_BP,
                               WMACConstants.SLOW_DOWN_DIST)
    self._expected_distance = expected_distance

    # Calculate urgency based on trajectory shortage with more aggressive calculation
    if endpoint_x < expected_distance:
      shortage = expected_distance - endpoint_x
      shortage_ratio = shortage / expected_distance

      # Base urgency on shortage ratio with more aggressive scaling
      urgency = min(1.0, shortage_ratio * 3.0)  # Increased from 2.0 to 3.0

      # Increase urgency for very short trajectories (imminent stops) - more aggressive
      critical_distance = expected_distance * 0.5  # Increased from 0.3 to 0.5
      if endpoint_x < critical_distance:
        urgency = min(1.0, urgency * 3.0)  # Increased from 2.0 to 3.0

      # Speed-based urgency adjustment - more aggressive
      if self._v_ego_kph > 10.0:  # Lower threshold for speed-based adjustment
        speed_factor = 1.0 + (self._v_ego_kph - 10.0) / 40.0  # Adjusted for more responsiveness
        urgency = min(1.0, urgency * speed_factor)

    # Apply filtering but with less smoothing for stops
    self._slow_down_filter.add_data(urgency)
    urgency_filtered = self._slow_down_filter.get_value() or 0.0

    # Update state - use the original threshold instead of reduced one for more precise stop detection
    self._has_slow_down = urgency_filtered > WMACConstants.SLOW_DOWN_PROB
    self._urgency = urgency_filtered

  def _radarless_mode(self) -> None:
    """Radarless mode decision logic with emergency handling."""

    # EMERGENCY: MPC FCW - immediate blended mode
    if self._has_mpc_fcw:
      self._mode_manager.request_mode('blended', confidence=1.0, emergency=True)
      return

    # Standstill: use blended with high confidence to ensure complete stop
    if self._standstill_count > 3:
      self._mode_manager.request_mode('blended', confidence=1.0)  # Increased from 0.9 to 1.0
      return

    # Slow down scenarios: emergency for high urgency, normal for lower urgency
    # More aggressive standstill enforcement when approaching stop
    if self._has_slow_down:
      if self._urgency > 0.5:  # Lowered threshold from 0.7 for more responsive stopping
        # Emergency: immediate blended mode for high urgency stops
        confidence = min(1.0, self._urgency * 1.2)  # Add confidence based on urgency
        self._mode_manager.request_mode('blended', confidence=confidence, emergency=True)
      else:
        # Normal: blended with urgency-based confidence
        confidence = min(1.0, self._urgency * 1.8)  # Increased from 1.5 for more responsiveness
        self._mode_manager.request_mode('blended', confidence=confidence)
      return

    # Driving slow: use ACC (but not if actively slowing down)
    if self._has_slowness and not self._has_slow_down:
      self._mode_manager.request_mode('acc', confidence=0.8)
      return

    # Default: ACC
    self._mode_manager.request_mode('acc', confidence=0.7)

  def _radar_mode(self) -> None:
    """Radar mode with emergency handling."""

    # EMERGENCY: MPC FCW - immediate blended mode
    if self._has_mpc_fcw:
      self._mode_manager.request_mode('blended', confidence=1.0, emergency=True)
      return

    # If lead detected and not in standstill: always use ACC
    if self._has_lead_filtered and not (self._standstill_count > 3):
      self._mode_manager.request_mode('acc', confidence=1.0)
      return

    # Slow down scenarios: emergency for high urgency, normal for lower urgency
    # More aggressive standstill enforcement when approaching stop
    if self._has_slow_down:
      if self._urgency > 0.5:  # Lowered threshold from 0.7 for more responsive stopping
        # Emergency: immediate blended mode for high urgency stops
        confidence = min(1.0, self._urgency * 1.2)  # Add confidence based on urgency
        self._mode_manager.request_mode('blended', confidence=confidence, emergency=True)
      else:
        # Normal: blended with urgency-based confidence
        confidence = min(1.0, self._urgency * 1.8)  # Increased from 1.3 for more responsiveness
        self._mode_manager.request_mode('blended', confidence=confidence)
      return

    # Standstill: use blended with high confidence to ensure complete stop
    if self._standstill_count > 3:
      self._mode_manager.request_mode('blended', confidence=1.0)  # Increased from 0.9 to 1.0
      return

    # Driving slow: use ACC (but not if actively slowing down)
    if self._has_slowness and not self._has_slow_down:
      self._mode_manager.request_mode('acc', confidence=0.8)
      return

    # Default: ACC
    self._mode_manager.request_mode('acc', confidence=0.7)

  def update(self, sm: messaging.SubMaster) -> None:
    self._read_params()

    self.set_mpc_fcw_crash_cnt()

    self._update_calculations(sm)

    # Check if we should enforce standstill mode explicitly
    enforce_standstill = self._should_enforce_standstill(sm)

    # Enhanced experimental mode activation when navigation is active
    nav_active = False
    if 'navInstruction' in sm and sm.updated['navInstruction']:
      nav_instruction = sm['navInstruction']
      nav_active = nav_instruction.active
      # Be more willing to activate experimental mode during navigation
      if nav_active and self._enabled:
        # If we're approaching a maneuver, be more conservative
        if (hasattr(nav_instruction, 'distanceToManeuver') and
            0 < nav_instruction.distanceToManeuver < 50.0):
          # Request blended mode when approaching maneuvers
          maneuver_type = getattr(nav_instruction, 'maneuverType', 'none')
          if maneuver_type in ['turn', 'arrive', 'stop', 'yield']:
            self._mode_manager.request_mode('blended', confidence=0.8)

    if self._CP.radarUnavailable:
      self._radarless_mode()
    else:
      self._radar_mode()

    # If we should enforce standstill, make sure we stay in blended mode
    if enforce_standstill:
      self._mode_manager.request_mode('blended', confidence=1.0, emergency=True)

    # Boost confidence in blended mode during navigation maneuvers
    if nav_active and self._enabled and self.mode() == 'blended':
      # Increase confidence in blended mode when navigating
      self._mode_manager.mode_confidence['blended'] = min(1.0, self._mode_manager.mode_confidence['blended'] + 0.05)
      # Reduce confidence in acc mode when navigating
      self._mode_manager.mode_confidence['acc'] = max(0.0, self._mode_manager.mode_confidence['acc'] - 0.02)

    self._mode_manager.update()
    self._active = sm['selfdriveState'].experimentalMode and self._enabled
    self._frame += 1

    # Perform monitoring
    if self._monitoring_enabled:
      self._monitor_modes()
      self._monitoring_frame_counter += 1

      # Send metrics every second
      if self._monitoring_frame_counter >= self._monitoring_interval:
        self._send_metrics(sm)
        self._monitoring_frame_counter = 0

  def _monitor_modes(self) -> None:
    """Monitor mode changes and time spent in each mode."""
    current_mode = self._mode_manager.get_mode()

    # Track mode changes
    if self._last_mode is not None and self._last_mode != current_mode:
      self._mode_switch_count += 1
      # Log time spent in previous mode
      time_in_last_mode = time.monotonic() - self._last_mode_change_time
      self._mode_timer[self._last_mode] += time_in_last_mode
      self._last_mode_change_time = time.monotonic()

    self._last_mode = current_mode

  def _should_enforce_standstill(self, sm: messaging.SubMaster) -> bool:
    """Explicitly check if the vehicle should be at complete standstill."""
    car_state = sm['carState']

    # If car is currently at standstill, enforce blended mode to maintain the stop
    if car_state.standstill:
      return True

    # If longitudinal plan indicates should stop and vEgo is very low, enforce standstill
    # Check if the longitudinalPlan service exists in the SubMaster before accessing it
    if 'longitudinalPlan' in sm and sm.updated['longitudinalPlan']:
      long_plan = sm['longitudinalPlan']
      if long_plan.shouldStop and car_state.vEgo < 0.5:  # Less than 0.5 m/s should be considered stopped
        return True

    # Check if in a stop scenario (traffic light, stop sign, etc.) from model
    if hasattr(sm['modelV2'], 'meta') and hasattr(sm['modelV2'].meta, 'stopState'):
      # If model indicates stop state and we're at very low speed
      if sm['modelV2'].meta.stopState and car_state.vEgo < 2.0:
        return True

    return False

  def _send_metrics(self, sm) -> None:
    """Send monitoring metrics to stats system."""
    try:
      current_mode = self._mode_manager.get_mode()
      current_time = time.monotonic()

      # Update time in current mode (since last update)
      time_in_current_mode = current_time - self._last_mode_change_time
      self._mode_timer[current_mode] += time_in_current_mode
      self._last_mode_change_time = current_time

      # Send metrics
      if self._active:  # Only send when the system is actually active
        statlog.sample('DynamicExperimentalController.mode_switches', self._mode_switch_count)
        statlog.gauge('DynamicExperimentalController.current_mode', 1.0 if current_mode == 'blended' else 0.0)
        statlog.sample('DynamicExperimentalController.time_in_acc_mode', self._mode_timer['acc'])
        statlog.sample('DynamicExperimentalController.time_in_blended_mode', self._mode_timer['blended'])

        # Additional context metrics
        statlog.gauge('DynamicExperimentalController.v_ego_kph', self._v_ego_kph)
        statlog.gauge('DynamicExperimentalController.v_cruise_kph', self._v_cruise_kph)
        statlog.gauge('DynamicExperimentalController.has_lead_filtered', float(self._has_lead_filtered))
        statlog.gauge('DynamicExperimentalController.has_slow_down', float(self._has_slow_down))
        statlog.gauge('DynamicExperimentalController.has_slowness', float(self._has_slowness))
        statlog.gauge('DynamicExperimentalController.has_mpc_fcw', float(self._has_mpc_fcw))
        statlog.gauge('DynamicExperimentalController.has_standstill', float(self._has_standstill))
        statlog.gauge('DynamicExperimentalController.urgency', self._urgency)
        statlog.gauge('DynamicExperimentalController.enabled', float(self._enabled))
        statlog.gauge('DynamicExperimentalController.active', float(self._active))

        # Reset counters for next interval
        self._mode_switch_count = 0

    except Exception as e:
      # Avoid any monitoring issues from affecting functionality
      pass
