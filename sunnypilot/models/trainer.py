"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import os
import numpy as np
import pickle
from tinygrad.tensor import Tensor
from tinygrad.nn.optim import Adam
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.system.hardware import TICI
from openpilot.system.hardware.hw import Paths

class DisengagementBuffer:
  """
  A circular buffer to store inputs and actions for on-device fine-tuning.
  """
  def __init__(self, capacity=100):
    self.capacity = capacity
    self.buffer = []
    self.save_path = os.path.join(Paths.log_root(), "shadow_mode_data")
    os.makedirs(self.save_path, exist_ok=True)

  def add(self, inputs, predicted_action, human_action):
    if len(self.buffer) >= self.capacity:
      self.buffer.pop(0)
    self.buffer.append({
      "inputs": inputs,
      "predicted_action": predicted_action,
      "human_action": human_action
    })

  def save_to_disk(self):
    if not self.buffer:
      return
    
    timestamp = int(np.round(np.datetime64('now').astype(float) * 1e3))
    filename = f"disengagement_{timestamp}.pkl"
    full_path = os.path.join(self.save_path, filename)
    
    with open(full_path, "wb") as f:
      pickle.dump(self.buffer, f)
    
    self.buffer = []
    print(f"ShadowModeTrainer: Saved disengagement data to {full_path}")

class ShadowModeTrainer:
  """
  Pillar 2: On-Device Fine-Tuning (The Holy Grail)
  Uses 'disengagement' data to fine-tune the policy head of the E2E model.
  """
  def __init__(self, model_runner=None):
    self.params = Params()
    self.model_runner = model_runner
    self.learning_rate = 1e-5
    self.optimizer = None
    self.enabled = self.params.get_bool("OnDeviceFineTuning")
    self.buffer = DisengagementBuffer()

  def collect_disengagement_data(self):
    """
    Identifies segments where human torque/accel diverged significantly 
    from model predictions.
    """
    files = sorted([f for f in os.listdir(self.buffer.save_path) if f.endswith(".pkl")])
    if not files:
      return []
    
    data = []
    for f in files:
      with open(os.path.join(self.buffer.save_path, f), "rb") as pkl:
        data.extend(pickle.load(pkl))
    return data

  def run_batch_tuning(self):
    """
    Performs batch gradient descent on collected data.
    """
    data = self.collect_disengagement_data()
    if not data:
      return

    print(f"ShadowModeTrainer: Starting batch tuning on {len(data)} samples...")
    for sample in data:
      self.train_step(sample["inputs"], sample["human_action"])
    
    # Placeholder: In a real implementation, we would save the delta weights here
    # self.save_delta_weights()

  def save_delta_weights(self):
    """
    Saves the fine-tuned policy weights to a local file.
    """
    weight_path = os.path.join(Paths.model_root(), "delta_policy_weights.pkl")
    # with open(weight_path, "wb") as f:
    #   pickle.dump(self.trainable_params, f)
    print(f"ShadowModeTrainer: Saved delta weights to {weight_path}")

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
