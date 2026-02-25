"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time
import numpy as np
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.swaglog import cloudlog

PROCESS_NAME = "sunnylink.firehosed"

class FirehoseDaemon:
  """
  Identifies 'High Entropy' or 'High Uncertainty' moments in the driving policy
  and flags them for prioritized upload (Active Learning).
  """
  def __init__(self):
    self.params = Params()
    self.pm = messaging.PubMaster(['sunnylinkState'])
    self.sm = messaging.SubMaster(['modelV2', 'carState', 'longitudinalPlanSP', 'carControl', 'driverStateV2'])
    
    self.entropy_filter = 0.0
    self.firehose_active = False

  def run(self):
    config_realtime_process([0, 1, 2, 3], 5)
    cloudlog.info("firehosed starting")

    while True:
      self.sm.update()
      if not self.sm.all_checks():
        continue
        
      md = self.sm['modelV2']
      cs = self.sm['carState']
      cc = self.sm['carControl']
      ds = self.sm['driverStateV2']
      
      # 1. Path Uncertainty (Entropy)
      # High standard deviation in the predicted trajectory indicates model confusion.
      path_uncertainty = float(np.mean(md.position.xStd) + np.mean(md.position.yStd))
      
      # 2. Human vs Model Divergence
      # If the driver is overriding or pressing steering, check how much the model's 
      # predicted path (pixels) differs from the human's actual path (torque).
      divergence = 0.0
      if cs.steeringPressed and len(md.orientation.x) > 0:
        pred_yaw_rate = md.orientationRate.z[0]
        actual_yaw_rate = cs.yawRate
        divergence = abs(pred_yaw_rate - actual_yaw_rate)
        
      # 3. Disengagement Prediction Spikes
      # If the model predicts a high probability of disengagement that doesn't 
      # match the current mode, it's a "surprising" moment.
      max_disengage_prob = float(max(md.meta.disengagePredictions.brakeDisengageProbs))

      # 4. Hard Negative Triggers (High-G / Evasive)
      # Trigger if the car experiences high longitudinal or lateral Gs, 
      # which often indicates a safety critical event or model failure.
      hard_brake = abs(cs.aEgo) > 3.0  # > 3.0 m/s^2 deceleration
      evasive_maneuver = abs(cs.yawRate) > 0.4 and cs.vEgo > 15.0 # High yaw rate at speed
      hard_negative_score = 20.0 if (hard_brake or evasive_maneuver) else 0.0

      # 5. Environmental Diversity
      # Trigger on low visibility or difficult lighting conditions.
      # driverStateV2.extra contains indicators for poor vision conditions.
      visibility_score = 0.0
      if hasattr(ds, 'extra') and (ds.extra.poorVisionProbs > 0.5):
        visibility_score = 10.0
      
      # Bayesian Entropy Fusion
      total_entropy = (path_uncertainty * 0.2) + (divergence * 5.0) + (max_disengage_prob * 10.0) + hard_negative_score + visibility_score
      
      # Smooth the entropy signal
      self.entropy_filter = 0.9 * self.entropy_filter + 0.1 * total_entropy
      
      # Flag for Firehose Mode (High-Priority Upload)
      # Threshold 15.0 is a heuristic for "Very High Uncertainty"
      is_high_entropy = self.entropy_filter > 15.0
      
      if is_high_entropy != self.firehose_active:
        self.firehose_active = is_high_entropy
        self.params.put_nonblocking("FirehoseModeActive", "1" if is_high_entropy else "0")
        if is_high_entropy:
          cloudlog.event("firehose_trigger", entropy=self.entropy_filter)
      
      time.sleep(0.1) # 10Hz is plenty for tagging

if __name__ == "__main__":
  daemon = FirehoseDaemon()
  try:
    daemon.run()
  except Exception:
    cloudlog.exception("firehosed failed")
    raise
