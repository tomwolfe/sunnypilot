
import os
import time
import pickle
import numpy as np
from collections import deque
from cereal import messaging
from openpilot.common.realtime import Ratekeeper, DT_CTRL
from openpilot.common.params import Params
from openpilot.sunnypilot.models.trainer import ShadowModeTrainer

class StateHistory:
  def __init__(self, max_length=100):
    self.buffer = deque(maxlen=max_length) # 5 seconds at 20Hz

  def add(self, model_msg, car_state, car_control, timestamp):
    self.buffer.append({
      "model": model_msg,
      "carState": car_state,
      "carControl": car_control,
      "timestamp": timestamp
    })

  def get_aligned_context(self, current_time, delay):
    target_time = current_time - delay
    # Find the closest frame in history
    if not self.buffer:
      return None
    
    best_match = min(self.buffer, key=lambda x: abs(x["timestamp"] - target_time))
    
    # Ensure the match is reasonably close (e.g., within 100ms)
    if abs(best_match["timestamp"] - target_time) > 0.1:
      return None
      
    return best_match

def main():
  params = Params()
  if not params.get_bool("OnDeviceFineTuning"):
    print("ShadowModeTrainer: On-device fine-tuning is disabled. Exiting...")
    return

  sm = messaging.SubMaster(['carState', 'carControl', 'modelV2', 'liveParameters'])
  trainer = ShadowModeTrainer()
  history = StateHistory(max_length=200) # 10 seconds buffer
  rk = Ratekeeper(20, print_delay_threshold=None)

  print("ShadowModeTrainer: Monitoring for disengagements with Temporal Alignment...")

  last_override_state = False
  high_loss_detected = False
  HIGH_LOSS_THRESHOLD = 0.5 # MSE threshold to trigger Sunnylink priority upload

  while True:
    sm.update()
    
    # Read latency from LagdToggle (in seconds)
    lag_val = params.get("LagdValueCache", encoding='utf-8')
    system_latency = float(lag_val) if lag_val else 0.2 # Default to 200ms
    
    if sm.updated['modelV2']:
      # Capture model context every frame
      history.add(sm['modelV2'], sm['carState'], sm['carControl'], time.time())
    
    if sm.updated['carState']:
      CS = sm['carState']
      CC = sm['carControl']
      
      # Detect human override
      is_steer_override = CS.steerOverride
      is_pedal_override = CS.gasPressed or CS.brakePressed
      is_currently_overridden = is_steer_override or is_pedal_override
      
      if is_currently_overridden:
        # 1. Capture Current Human Action (Ground Truth)
        human_action = {
          "torque": CS.steeringTorque,
          "gas": CS.gas,
          "brake": CS.brake,
          "v_ego": CS.vEgo,
          "yaw_rate": CS.yawRate
        }
        
        # 2. Get Aligned Model Context (Pillar 4: Dynamic Latency Compensation)
        # We look back in history to find what the model was predicting
        # at the moment the human *actually* decided to override.
        aligned_data = history.get_aligned_context(time.time(), system_latency)
        
        if aligned_data:
          model_msg = aligned_data["model"]
          model_action = {
            "path": model_msg.path.poly if hasattr(model_msg, 'path') and len(model_msg.path.poly) > 0 else [],
            "engaged_prob": model_msg.meta.engagedProb if hasattr(model_msg, 'meta') else 0.0
          }
          
          # 3. Calculate Loss (Step 2: Active Learning)
          # If the human action diverges significantly from the model, we want this data ASAP.
          loss = trainer.train_step(None, human_action, model_action)
          if loss > HIGH_LOSS_THRESHOLD:
            high_loss_detected = True

          # 4. Add to training buffer (Pillar 1: Policy Distillation)
          # We provide the model inputs (context) and the human's "better" action.
          trainer.buffer.add(
            inputs=aligned_data["carState"], # In a real implementation, this would be latent features
            predicted_action=model_action, 
            human_action=human_action
          )

        # 5. Periodic Save
        if len(trainer.buffer.buffer) >= 500: # Save in larger chunks
          trainer.buffer.save_to_disk(priority=high_loss_detected)
          high_loss_detected = False
          
      # Detect the end of an override to trigger a "session save"
      if last_override_state and not is_currently_overridden:
        trainer.buffer.save_to_disk(priority=high_loss_detected)
        high_loss_detected = False
        
      last_override_state = is_currently_overridden

    rk.keep_time()

if __name__ == "__main__":
  main()
