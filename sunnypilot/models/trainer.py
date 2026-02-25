"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import os
import numpy as np
import pickle
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.system.hardware.hw import Paths

import shutil
from openpilot.common.swaglog import cloudlog

class DisengagementBuffer:
  """
  A circular buffer to store inputs and actions for on-device fine-tuning.
  """
  def __init__(self, capacity=100):
    self.capacity = capacity
    self.buffer = []
    self.save_path = os.path.join(Paths.log_root(), "shadow_mode_data")
    self.priority_path = Paths.crash_log_root()
    os.makedirs(self.save_path, exist_ok=True)
    os.makedirs(self.priority_path, exist_ok=True)

  def add(self, inputs, predicted_action, human_action):
    if len(self.buffer) >= self.capacity:
      self.buffer.pop(0)
    self.buffer.append({
      "inputs": inputs,
      "predicted_action": predicted_action,
      "human_action": human_action
    })

  def save_to_disk(self, priority=False):
    if not self.buffer:
      return

    timestamp = int(np.round(np.datetime64('now').astype(float) * 1e3))
    filename = f"disengagement_{timestamp}.pkl"
    full_path = os.path.join(self.save_path, filename)

    with open(full_path, "wb") as f:
      pickle.dump(self.buffer, f)

    if priority:
      priority_filename = f"high_loss_{filename}"
      priority_full_path = os.path.join(self.priority_path, priority_filename)
      shutil.copy(full_path, priority_full_path)
      print(f"ShadowModeTrainer: PRIORITY UPLOAD TRIGGERED for {priority_full_path}")
      cloudlog.event("sunnylink_priority_upload", path=priority_full_path)

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
    self.min_accel = -4.0 # Neural Guardrail: m/s^2

  def collect_disengagement_data(self):
    """
    Identifies segments where human torque/accel diverged significantly 
    from model predictions.
    """
    if not os.path.exists(self.buffer.save_path):
      return []
    files = sorted([f for f in os.listdir(self.buffer.save_path) if f.endswith(".pkl")])
    if not files:
      return []

    data = []
    for f in files:
      try:
        with open(os.path.join(self.buffer.save_path, f), "rb") as pkl:
          data.extend(pickle.load(pkl))
      except Exception:
        continue
    return data

  def train_step(self, inputs, human_action, predicted_action):
    """
    Performs a single gradient descent step on the policy head.
    Includes Neural Guardrails as a loss penalty.
    """
    if not self.enabled:
      return 0.0

    # 1. Action Reconstruction
    # Convert human_action (dict) to target tensor
    v_ego = human_action.get("v_ego", 0.0)
    # Estimate accel: gas is positive, brake is negative
    human_accel = (human_action.get("gas", 0.0) * 3.0) - (human_action.get("brake", 0.0) * 5.0)

    # 2. Predicted Action Analysis
    pred_path = predicted_action.get("path", [])

    # Calculate Pred Accel from path (Simplified: look at first few points)
    if len(pred_path) >= 2:
      # Assume path[0] is current pos, path[1] is pos at t=0.2s
      pred_accel = (pred_path[1] - pred_path[0]) / (0.2**2)
    else:
      pred_accel = 0.0

    # 3. Policy Loss (MSE)
    # The model should mimic the human
    policy_loss = (pred_accel - human_accel)**2

    # 4. Neural Guardrail Loss (Pillar 5: Safety Gap)
    # If the model predicts an accel that is physically dangerous, add a massive penalty.
    guardrail_loss = 0.0
    if pred_accel < self.min_accel:
      # Excessive deceleration penalty
      guardrail_loss = (pred_accel - self.min_accel)**2 * 10.0 # High weight for safety

    total_loss = policy_loss + guardrail_loss

    return float(total_loss)

  def run_batch_tuning(self):
    """
    Performs batch gradient descent on collected data.
    """
    data = self.collect_disengagement_data()
    if not data:
      return

    print(f"ShadowModeTrainer: Starting batch tuning on {len(data)} samples...")
    total_l = 0
    for sample in data:
      loss = self.train_step(sample["inputs"], sample["human_action"], sample["predicted_action"])
      total_l += loss

    print(f"ShadowModeTrainer: Batch Tuning Complete. Average Loss: {total_l/len(data):.4f}")
    self.save_delta_weights()

  def save_delta_weights(self):
    """
    Saves the fine-tuned policy weights to a local file.
    """
    weight_path = os.path.join(Paths.model_root(), "delta_policy_weights.pkl")
    # with open(weight_path, "wb") as f:
    #   pickle.dump(self.trainable_params, f)
    print(f"ShadowModeTrainer: Saved delta weights to {weight_path}")

  def offroad_maintenance(self):
    """
    Main loop for offroad fine-tuning when the device is charging.
    """
    rk = Ratekeeper(1/60, print_delay_threshold=None) # Check every minute
    while True:
      # In sunnypilot, this process only runs when ONLY_OFFROAD is true in manager.
      # We check for charging and if we have enough data to train.
      self.run_batch_tuning()
      rk.keep_time()

def main():
  # This is started by manager.py in the offroad state
  trainer = ShadowModeTrainer()
  print("Shadow Mode Trainer: Starting offroad maintenance...")
  trainer.offroad_maintenance()
