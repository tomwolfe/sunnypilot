"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import messaging, custom
from opendbc.car import structs
import numpy as np
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.selfdrive.modeld.constants import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_calibrator import LongitudinalCalibrator
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


class JerkLimitedNeuralFilter:
  """
  Second-Order Jerk-Limited Filter for E2E Longitudinal Smoothness.
  Filters acceleration targets based on the car's braking characteristics.
  """
  def __init__(self, dt=DT_MDL):
    self.dt = dt
    self.a = 0.0
    self.initialized = False
    self.jerk_limit_base = 2.0  # m/s^3
    self.braking_jerk_factor = 0.4  # more restrictive when braking

  def update(self, a_target, e2e_trust, calibration_confidence=1.0):
    if not self.initialized:
      self.a = a_target
      self.initialized = True
      return self.a

    # Adjust jerk limit based on braking intensity and trust
    # In high-trust E2E scenarios, we allow the model more authority but still limit the jerk
    # for human-like smoothness.
    is_braking = a_target < -0.5
    jerk_limit = self.jerk_limit_base * (self.braking_jerk_factor if is_braking else 1.0)

    # Scale jerk limit slightly with trust to allow more responsive E2E maneuvers
    # but restrict it if calibration confidence is low (MAE is high)
    jerk_limit *= (0.8 + 0.4 * e2e_trust * calibration_confidence)

    da = (a_target - self.a) / self.dt
    da_clipped = np.clip(da, -jerk_limit, jerk_limit)
    self.a += da_clipped * self.dt
    return self.a


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.events_sp = EventsSP()
    self.resolver = SpeedLimitResolver()
    self.dec = DynamicExperimentalController(CP, mpc)
    self.calibrator = LongitudinalCalibrator()
    self.scc = SmartCruiseControl()
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()

    self.output_v_target = 0.
    self.output_a_target = 0.
    self.a_filter = JerkLimitedNeuralFilter()

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.dec.active():
      return experimental_mode

    # Use continuous trust for high-level decision making
    return experimental_mode and (self.dec.blended_confidence() > 0.5 or self.dec.mode() == 'pure_e2e')

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Speed Limit Assist
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                    self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)

    # Pure E2E Override (Pillar 1)
    if self.dec.mode() == 'pure_e2e':
      md = sm['modelV2']
      if len(md.velocity.x) > 0:
        # 100% authority to Neural Policy in high-confidence scenarios
        self.output_v_target = min(md.velocity.x[0], v_cruise)
        if len(md.acceleration.x) > 0:
          self.output_a_target = md.acceleration.x[0]
        self.source = LongitudinalPlanSource.pureE2e

        # Still apply Jerk-Limited Neural Filter for smoothness
        self.output_a_target = self.a_filter.update(self.output_a_target, 1.0, self.calibrator.confidence)
        return self.output_v_target, self.output_a_target

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }

    # Neural Speed Limit Fusion:
    # If in E2E mode, we treat the speed limit as a "soft" feature rather than a hard constraint.
    # We blend the SLA target with the model's predicted trajectory endpoint velocity.
    e2e_trust = self.dec.blended_confidence() if self.dec.active() else float(sm['selfdriveState'].experimentalMode)

    if e2e_trust > 0.3 and (self.sla.is_active or self.scc.vision.is_active):
      md = sm['modelV2']
      if len(md.velocity.x) > 0:
        v_model = md.velocity.x[0]

        # Determine the "suggested" target from heuristics (SLA or Vision Curve Control)
        # We take the minimum of the two as the "Heuristic Suggestion"
        v_heuristic = min(self.sla.output_v_target,
                          self.scc.vision.output_v_target if self.scc.vision.is_active else float('inf'))

        # Bayesian blending: favor the model if uncertainty is low or trust is high
        # We use a dynamic blend factor that scales with our continuous trust in the neural model
        # AND the calibration confidence.
        blend_factor = 0.5 + 0.4 * e2e_trust * self.calibrator.confidence
        v_cruise_neural = v_model * blend_factor + v_heuristic * (1.0 - blend_factor)

        # Update the dominant heuristic source with our blended neural target
        if self.source == LongitudinalPlanSource.speedLimitAssist:
          targets[LongitudinalPlanSource.speedLimitAssist] = (v_cruise_neural, self.sla.output_a_target)
        elif self.source == LongitudinalPlanSource.sccVision:
          targets[LongitudinalPlanSource.sccVision] = (v_cruise_neural, self.scc.vision.output_a_target)

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]

    # Apply Jerk-Limited Neural Filter for smoothness
    self.output_a_target = self.a_filter.update(self.output_a_target, e2e_trust, self.calibrator.confidence)

    return self.output_v_target, self.output_a_target

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_sp.clear()

    # Update Neural Calibration
    md = sm['modelV2']
    if len(md.velocity.x) > 1:
      # Calculate model's intended immediate acceleration
      a_neural = (md.velocity.x[1] - md.velocity.x[0]) / DT_MDL
      self.calibrator.update(a_neural, sm['carState'].aEgo, self.dec.mode() == 'blended')

    # Update DEC with calibration data
    self.dec._calibration_mae = self.calibrator.mae
    self.dec._calibration_uncertainty_offset = self.calibrator.calibrated_uncertainty_offset
    self.dec._calibration_confidence = self.calibrator.confidence

    self.dec.update(sm)

    self.e2e_alerts_helper.update(sm, self.events_sp)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

    # Dynamic Experimental Control
    dec = longitudinalPlanSP.dec
    if self.dec.mode() == 'pure_e2e':
      dec.state = DecState.pureE2e
    elif self.dec.mode() == 'blended':
      dec.state = DecState.blended
    else:
      dec.state = DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()
    dec.blendedWeight = self.dec.blended_weight()
    dec.mae = self.calibrator.mae
    dec.uncertaintyOffset = self.calibrator.calibrated_uncertainty_offset

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    # Navigation Intent Fusion:
    # Translates map/vision curve control slowing into a 'Soft Intent' for the E2E model.
    nav_intent = 0.0
    v_ego = sm['carState'].vEgo
    if self.scc.map.is_active or self.scc.vision.is_active:
      v_target_min = min(self.scc.map.output_v_target if self.scc.map.is_active else float('inf'),
                         self.scc.vision.output_v_target if self.scc.vision.is_active else float('inf'))
      # We scale intent based on how much the system wants to slow down relative to current speed.
      # 1.0 intent at 5m/s (18km/h) or greater delta
      if v_target_min < v_ego:
        nav_intent = np.clip((v_ego - v_target_min) / 5.0, 0.0, 1.0)

    longitudinalPlanSP.navigationIntent = float(nav_intent)

    pm.send('longitudinalPlanSP', plan_sp_send)
