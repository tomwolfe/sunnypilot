"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from collections import deque
from openpilot.common.realtime import DT_CTRL


class LongitudinalCalibrator:
  """
  Online calibrator for neural longitudinal uncertainty.
  Compares model's predicted acceleration to actual aEgo over a window.
  """

  def __init__(self, delay_s=0.25, window_s=30.0):
    self.delay_steps = int(delay_s / DT_CTRL)
    self.window_steps = int(window_s / DT_CTRL)
    
    # Buffers
    self.accel_neural_history = deque(maxlen=self.delay_steps + 1)
    self.errors = deque(maxlen=self.window_steps)
    
    # State
    self.calibrated_uncertainty_offset = 0.0
    self.mae = 0.0
    self.active = False

  def update(self, accel_neural, a_ego, use_neural):
    """
    Update the calibrator with new data.
    accel_neural: model's predicted acceleration (m/s^2)
    a_ego: actual car acceleration (m/s^2)
    use_neural: whether the neural model is currently in control
    """
    # We only care about accuracy when the neural model is active
    self.active = use_neural
    
    # Always track the history to maintain the delay buffer
    self.accel_neural_history.append(accel_neural)
    
    if len(self.accel_neural_history) > self.delay_steps:
      # Get the predicted acceleration from 'delay_s' ago
      delayed_accel = self.accel_neural_history[0]
      
      # Calculate absolute error
      error = abs(delayed_accel - a_ego)
      
      # We only update the error window if the model was 'in control' when it made the prediction
      # For simplicity, we check if it's currently active. 
      # In a perfect world, we'd buffer 'use_neural' too.
      if self.active:
        self.errors.append(error)
      
      # Calculate Mean Absolute Error (MAE) over the window
      if len(self.errors) > 100:  # Require at least 1 second of data
        self.mae = float(np.mean(self.errors))
        
        # Scaling: If MAE is 0.5 m/s^2, we add 1.0m to effective uncertainty.
        # This reduces w_neural significantly.
        self.calibrated_uncertainty_offset = self.mae * 2.0
      else:
        self.calibrated_uncertainty_offset = 0.0
    
    return self.calibrated_uncertainty_offset

  def reset(self):
    self.accel_neural_history.clear()
    self.errors.clear()
    self.calibrated_uncertainty_offset = 0.0
    self.mae = 0.0
