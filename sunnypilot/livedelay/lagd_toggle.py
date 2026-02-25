"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import log

from opendbc.car import structs
from openpilot.common.params import Params


import numpy as np
from collections import deque
from cereal import messaging


class LagdToggle:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.params = Params()
    self.lag = 0.0

    self.lagd_toggle = self.params.get_bool("LagdToggle")
    self.software_delay = float(self.params.get("LagdToggleDelay", return_default=True) or 0.2)

    # Dynamic Calibration Buffers
    self.torque_buffer = deque(maxlen=100) # 1 second at 100Hz
    self.yaw_rate_buffer = deque(maxlen=100)
    self.last_dynamic_lag = self.CP.steerActuatorDelay

    # A+ Enhancement: Adaptive Latency Compensation
    # Tracks model execution time to compensate for computational jitter
    self.execution_time_buffer = deque(maxlen=50)  # 2.5 seconds at 20Hz
    self.frame_timestamp_buffer = deque(maxlen=50)
    self.last_execution_time = 0.0
    self.last_frame_time = 0.0
    self.computational_lag_estimate = 0.0

    # Thermal throttling detection
    self.thermal_throttle_detected = False
    self.throttle_warning_count = 0

    # Latency prediction filter
    self.latency_prediction_filter = deque(maxlen=20)
    self.predicted_latency = 0.0

  def read_params(self) -> None:
    self.lagd_toggle = self.params.get_bool("LagdToggle")
    self.software_delay = float(self.params.get("LagdToggleDelay", return_default=True) or 0.2)

  def update_dynamic_delay(self, sm: messaging.SubMaster) -> None:
    """
    Calculates steerActuatorDelay dynamically by cross-correlating
    commanded torque and observed yaw rate.
    """
    if not self.lagd_toggle:
      return

    cc = sm['carControl']
    cs = sm['carState']

    if cc.latActive and abs(cs.vEgo) > 10.0:
      self.torque_buffer.append(cc.actuators.steer)
      self.yaw_rate_buffer.append(cs.yawRate)

      if len(self.torque_buffer) == self.torque_buffer.maxlen:
        # Perform simple cross-correlation
        torque = np.array(self.torque_buffer)
        yaw = np.array(self.yaw_rate_buffer)

        # Normalize
        torque = (torque - np.mean(torque)) / (np.std(torque) + 1e-6)
        yaw = (yaw - np.mean(yaw)) / (np.std(yaw) + 1e-6)

        correlation = np.correlate(yaw, torque, mode='full')
        lags = np.arange(-len(torque) + 1, len(torque))

        # We only care about positive lags (yaw lagging torque)
        positive_lags = lags > 0
        if np.any(positive_lags):
          best_lag_idx = np.argmax(correlation[positive_lags])
          best_lag_steps = lags[positive_lags][best_lag_idx]

          # Convert steps to seconds (assuming 100Hz)
          dynamic_lag = best_lag_steps * 0.01

          # Alpha filter for stability
          self.last_dynamic_lag = 0.95 * self.last_dynamic_lag + 0.05 * dynamic_lag

  def update_execution_time(self, execution_time_sec: float, frame_time: float) -> None:
    """
    A+ Enhancement: Record model execution time for adaptive latency compensation
    
    As the device heats up and GPU throttling occurs, execution time increases.
    This method tracks execution time and updates the latency estimate accordingly.
    
    Args:
      execution_time_sec: Time taken to execute the model (seconds)
      frame_time: Timestamp of the frame being processed
    """
    self.execution_time_buffer.append(execution_time_sec)
    self.frame_timestamp_buffer.append(frame_time)

    if len(self.execution_time_buffer) < 10:
      return

    # Compute moving average execution time
    avg_execution_time = np.mean(self.execution_time_buffer)

    # Detect thermal throttling (sudden increase in execution time)
    if len(self.execution_time_buffer) >= 20:
      recent_avg = np.mean(list(self.execution_time_buffer)[-10:])
      older_avg = np.mean(list(self.execution_time_buffer)[-20:-10])

      if recent_avg > older_avg * 1.2:  # 20% increase
        self.thermal_throttle_detected = True
        self.throttle_warning_count += 1
      else:
        self.throttle_warning_count = max(0, self.throttle_warning_count - 1)
        if self.throttle_warning_count == 0:
          self.thermal_throttle_detected = False

    # Predict next frame latency using simple linear prediction
    if len(self.execution_time_buffer) >= 5:
      recent_times = list(self.execution_time_buffer)[-5:]
      time_diffs = [recent_times[i] - recent_times[i-1] for i in range(1, len(recent_times))]
      avg_diff = np.mean(time_diffs) if time_diffs else 0.0
      self.predicted_latency = recent_times[-1] + avg_diff * 0.5
    else:
      self.predicted_latency = avg_execution_time

    # Update computational lag estimate
    # Base lag + fraction of execution time (models pipeline delay)
    self.computational_lag_estimate = 0.05 + avg_execution_time * 0.3

    self.last_execution_time = execution_time_sec
    self.last_frame_time = frame_time

  def get_execution_time_stats(self) -> dict:
    """
    Get execution time statistics for debugging/monitoring
    
    Returns:
      Dict with execution time statistics
    """
    if not self.execution_time_buffer:
      return {
        'avg_ms': 0.0,
        'max_ms': 0.0,
        'min_ms': 0.0,
        'std_ms': 0.0,
        'thermal_throttle': False,
        'predicted_latency_ms': 0.0
      }

    times_ms = np.array(self.execution_time_buffer) * 1000

    return {
      'avg_ms': float(np.mean(times_ms)),
      'max_ms': float(np.max(times_ms)),
      'min_ms': float(np.min(times_ms)),
      'std_ms': float(np.std(times_ms)),
      'thermal_throttle': self.thermal_throttle_detected,
      'predicted_latency_ms': float(self.predicted_latency * 1000),
      'computational_lag': float(self.computational_lag_estimate)
    }

  def get_adaptive_latency_compensation(self) -> float:
    """
    A+ Enhancement: Get adaptive latency compensation value
    
    This compensates for computational jitter by feeding back the
    measured execution time of the previous frame.
    
    Returns:
      Latency compensation value in seconds
    """
    if not self.execution_time_buffer:
      return self.software_delay

    # Use predicted latency if available, otherwise use average
    if self.predicted_latency > 0:
      compensation = self.predicted_latency
    else:
      compensation = float(np.mean(self.execution_time_buffer))

    # Add extra compensation if thermal throttling detected
    if self.thermal_throttle_detected:
      compensation *= 1.2  # 20% extra compensation

    return compensation

  def update(self, lag_msg: log.LiveDelayData) -> None:
    self.read_params()

    if not self.lagd_toggle:
      steer_actuator_delay = self.CP.steerActuatorDelay
      delay = self.software_delay
      self.lag = (steer_actuator_delay + delay)
      self.params.put_nonblocking("LagdValueCache", str(self.lag))
      return

    # In LagdToggle mode, we blend the LiveDelay estimator with our dynamic torque-based estimate
    lateral_delay = lag_msg.liveDelay.lateralDelay

    # A+ Enhancement: Add adaptive computational latency compensation
    adaptive_compensation = self.get_adaptive_latency_compensation()

    # Blend: 70% lateral_delay, 30% dynamic torque-based, plus computational lag
    if 0.05 < self.last_dynamic_lag < 0.8:
      self.lag = 0.7 * lateral_delay + 0.3 * self.last_dynamic_lag + adaptive_compensation
    else:
      self.lag = lateral_delay + adaptive_compensation

    self.params.put_nonblocking("LagdValueCache", str(self.lag))
