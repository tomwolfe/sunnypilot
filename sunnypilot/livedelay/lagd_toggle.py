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
from cereal import log, messaging
from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL


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
    
    # We use our dynamic lag if it's within a sane range, otherwise fallback to lateral_delay
    if 0.05 < self.last_dynamic_lag < 0.8:
      self.lag = 0.7 * lateral_delay + 0.3 * self.last_dynamic_lag
    else:
      self.lag = lateral_delay
      
    self.params.put_nonblocking("LagdValueCache", str(self.lag))
