"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import os
import numpy as np
from tinygrad.tensor import Tensor
from tinygrad.nn.optim import Adam
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.system.hardware import TICI

class ShadowModeTrainer:
  """
  Pillar 2: On-Device Fine-Tuning (The Holy Grail)
  Uses 'disengagement' data to fine-tune the policy head of the E2E model.
  """
  def __init__(self, model_runner):
    self.params = Params()
    self.model_runner = model_runner
    self.learning_rate = 1e-5  # Low learning rate for safe fine-tuning
    self.optimizer = None
    self.weights = []
    self.enabled = self.params.get_bool("OnDeviceFineTuning")

  def collect_disengagement_data(self):
    """
    Identifies segments where human torque/accel diverged significantly 
    from model predictions.
    """
    # Placeholder: In a real implementation, this would read from the 
    # 'backups' or logged segments on disk.
    pass

  def train_step(self, inputs, human_action):
    """
    Performs a single gradient descent step on the policy head.
    """
    if not self.enabled:
      return

    # 1. Forward pass
    # we convert inputs to tinygrad Tensors
    t_inputs = {k: Tensor(v) for k, v in inputs.items()}
    predicted_action = self.model_runner.model_run(**t_inputs)
    
    # 2. Loss calculation (MSE between predicted and human action)
    t_human_action = Tensor(human_action)
    loss = (predicted_action - t_human_action).square().mean()
    
    # 3. Backward pass
    if self.optimizer is None:
      # Initialize optimizer with the policy head parameters
      # This requires the tinygrad model artifact to expose its trainable params
      # self.optimizer = Adam(self.trainable_params, lr=self.learning_rate)
      pass
      
    # loss.backward()
    # self.optimizer.step()
    
    return float(loss.numpy())

  def offroad_maintenance(self):
    """
    Main loop for offroad fine-tuning when the device is charging.
    """
    rk = Ratekeeper(1, print_delay_threshold=None)
    while True:
      # Check if charging and offroad
      # if is_offroad() and is_charging():
      #   self.run_batch_tuning()
      rk.keep_time()

def main():
  # This would be started by manager.py in the offroad state
  print("Shadow Mode Trainer: Standing by for offroad maintenance...")

if __name__ == "__main__":
  main()
