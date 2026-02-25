
import os
import time
import pickle
import numpy as np
from cereal import messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params
from openpilot.sunnypilot.models.trainer import ShadowModeTrainer

def main():
  params = Params()
  if not params.get_bool("OnDeviceFineTuning"):
    print("ShadowModeTrainer: On-device fine-tuning is disabled. Exiting...")
    return

  sm = messaging.SubMaster(['carState', 'carControl', 'modelV2', 'liveParameters'])
  trainer = ShadowModeTrainer()
  rk = Ratekeeper(20, print_delay_threshold=None)

  print("ShadowModeTrainer: Monitoring for disengagements...")

  while True:
    sm.update()
    
    if sm.updated['carState']:
      CS = sm['carState']
      CC = sm['carControl']
      model = sm['modelV2']
      
      # Detect human override (torque override, or pedal override if disengage on gas is off)
      # In sunnypilot, MADS might be active, so we check for steerOverride or pedal activity
      is_steer_override = CS.steerOverride
      is_pedal_override = CS.gasPressed or CS.brakePressed
      
      if is_steer_override or is_pedal_override:
        # 1. Capture Current State
        # In a full implementation, we would capture the latent features or vision tokens here
        # For this prototype, we'll log the fact that an override happened
        human_action = {
          "torque": CS.steeringTorque,
          "gas": CS.gas,
          "brake": CS.brake
        }
        
        # 2. Get Model Prediction (what the model wanted to do)
        model_action = {
          "path": model.path.poly if len(model.path.poly) > 0 else []
        }
        
        # 3. Add to buffer
        trainer.buffer.add(inputs={}, predicted_action=model_action, human_action=human_action)
        
        # 4. If override just ended, or we have a significant sequence, save it
        # (Simplified: save every 100 samples of override)
        if len(trainer.buffer.buffer) >= 100:
          trainer.buffer.save_to_disk()

    rk.keep_time()

if __name__ == "__main__":
  main()
