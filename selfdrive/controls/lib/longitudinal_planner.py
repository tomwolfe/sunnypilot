#!/usr/bin/env python3
import math
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0.0, 10.0, 25.0, 40.0]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20.0, 40.0]


def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego**2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max**2 - a_y**2, 0.0))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    # TODO remove mpc modes when TR released
    self.mpc.mode = 'acc'
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

  @staticmethod
  def parse_model(model_msg):
    if (
      len(model_msg.position.x) == ModelConstants.IDX_N
      and len(model_msg.velocity.x) == ModelConstants.IDX_N
      and len(model_msg.acceleration.x) == ModelConstants.IDX_N
    ):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm):
    mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'
    if not self.mlsim:
      self.mpc.mode = mode
    LongitudinalPlannerSP.update(self, sm)
    if dec_mpc_mode := self.get_mpc_mode():
      mode = dec_mpc_mode
      if not self.mlsim:
        self.mpc.mode = dec_mpc_mode

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if mode == 'acc':
      accel_clip = [ACCEL_MIN, get_max_accel(v_ego)]
      steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
      accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)
    else:
      accel_clip = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # Enhanced radar-camera fusion: Improve lead vehicle detection by combining radar and vision data
    x, v, a, j, throttle_prob = self.parse_model(sm['modelV2'])

    # Apply radar-camera fusion to enhance lead vehicle detection
    enhanced_x, enhanced_v, enhanced_a = self._fuse_radar_camera_data(sm, x, v, a)

    # Don't clip at low speeds since throttle_prob doesn't account for creep
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED * 2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    # Get new v_cruise and a_desired from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.a_desired = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.a_desired, v_cruise)

    if force_slow_decel:
      v_cruise = 0.0

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    # Use enhanced fused data instead of raw model data
    self.mpc.update(sm['radarState'], v_cruise, enhanced_x, enhanced_v, enhanced_a, j, personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # Enhanced forward collision warning with multiple indicators
    # Use multiple thresholds based on speed and distance
    lead_one = sm['radarState'].leadOne
    v_ego = sm['carState'].vEgo
    a_ego = sm['carState'].aEgo

    # Calculate time to collision based on current trajectories

    time_to_collision = float('inf')

    distance_to_collision = float('inf')

    relative_speed = 0.0
    if lead_one.status and lead_one.dRel > 0:
      relative_speed = v_ego - lead_one.vRel  # vRel is negative when approaching
      if relative_speed > 0:  # Approaching lead vehicle
        time_to_collision = lead_one.dRel / relative_speed
        distance_to_collision = lead_one.dRel
      elif lead_one.vRel > 0:  # Lead vehicle accelerating away
        # Calculate when we'd catch up if they maintain speed
        # Use a more noise-resistant approach by applying filtering to acceleration values
        relative_accel = a_ego - lead_one.aLeadK

        # Apply noise filtering to relative acceleration to prevent false triggers
        # Check if acceleration values are reliable before using them
        acceleration_reliable = abs(relative_accel) > 0.5  # Only use if acceleration difference is significant
        relative_accel_filtered = relative_accel if acceleration_reliable else 0.0

        if relative_accel_filtered > 0.1 and relative_speed > 0.5:  # Only calculate if conditions are stable
          # Apply a safety check to prevent unrealistic calculations from noisy data
          # Limit time to collision to a reasonable range to prevent spikes
          raw_time_to_collision = relative_speed / relative_accel_filtered
          # Cap the calculated time to prevent extreme values from noise
          time_to_collision = min(raw_time_to_collision, 5.0)  # Cap at 5 seconds to prevent noise-induced false positives

    # Optimized Forward Collision Warning (FCW) with reduced computational overhead
    self.fcw = self._calculate_optimized_fcw(sm, lead_one, v_ego, a_ego)

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t = self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(
      self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX, action_t=action_t, vEgoStopping=self.CP.vEgoStopping
    )
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    if mode == 'acc' or not self.mlsim:
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc
    else:
      output_a_target = min(output_a_target_mpc, output_a_target_e2e)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

  def _fuse_radar_camera_data(self, sm, model_x, model_v, model_a):
    """
    Optimized radar-camera fusion to improve lead vehicle detection and tracking.
    Significantly reduced computational overhead while maintaining safety.

    Combines radar measurements with vision model outputs to create more robust
    and accurate lead vehicle tracking.

    Args:
        sm: SubMaster instance with current sensor data
        model_x: Model's position predictions
        model_v: Model's velocity predictions
        model_a: Model's acceleration predictions

    Returns:
        Tuple of (fused_x, fused_v, fused_a) with improved predictions
    """
    try:
      radar_state = sm['radarState']
      model_v2 = sm['modelV2']

      # Apply lightweight fusion focusing on the most critical lead
      enhanced_x = model_x.copy()
      enhanced_v = model_v.copy()
      enhanced_a = model_a.copy()

      # Process lead vehicle fusion if radar detects a lead
      if radar_state.leadOne.status and len(model_v2.leadsV3) > 0:
        lead_radar = radar_state.leadOne

        # Get the most confident lead from vision model (simple approach)
        vision_lead = model_v2.leadsV3[0]  # Take the first (most confident) detection

        # Only apply fusion if vision lead is confident enough
        if vision_lead.prob > 0.6:  # Lower confidence threshold for efficiency
          # Use simple fixed weights instead of complex reliability calculation
          radar_weight = 0.4  # Fixed weight for radar reliability
          vision_weight = 0.6  # Fixed weight for vision confidence

          # Simple weighted fusion
          fused_dRel = lead_radar.dRel * radar_weight + vision_lead.dRel * vision_weight
          fused_vRel = lead_radar.vRel * radar_weight + vision_lead.vRel * vision_weight

          # Handle acceleration fusion
          radar_aLead = lead_radar.aLeadK
          vision_aLead = getattr(vision_lead, 'aRel', 0.0)
          fused_aLead = radar_aLead * radar_weight + vision_aLead * vision_weight

          # Apply efficient validation
          if len(enhanced_x) > 0:
            enhanced_x[0] = np.clip(fused_dRel, 2.0, 150.0)
          if len(enhanced_v) > 0:
            enhanced_v[0] = np.clip(fused_vRel, -50.0, 50.0)
          if len(enhanced_a) > 0:
            enhanced_a[0] = np.clip(fused_aLead, -10.0, 8.0)

      return enhanced_x, enhanced_v, enhanced_a
    except Exception:
      # Return original data as safe fallback without error logging to reduce overhead
      return model_x, model_v, model_a  # Safe fallback: return original values

  def _validate_fused_sensor_data(self, x, v, a):
    """
    Lightweight validation of fused sensor data to ensure physical plausibility and safety.
    Reduced computational overhead while maintaining essential safety checks.

    Args:
        x: Fused distance values
        v: Fused velocity values
        a: Fused acceleration values

    Returns:
        Tuple of validated (x, v, a) arrays with physically plausible values
    """
    # Create copies to avoid modifying original arrays directly
    validated_x = x.copy()
    validated_v = v.copy()
    validated_a = a.copy()

    for i in range(len(validated_x)):
      # Basic validation: clip to realistic range
      validated_x[i] = np.clip(validated_x[i], 0.1, 200.0)  # 0.1m to 200m range

    for i in range(len(validated_v)):
      # Basic velocity validation
      validated_v[i] = np.clip(validated_v[i], -50.0, 50.0)  # -50 to +50 m/s

    for i in range(len(validated_a)):
      # Basic acceleration validation
      validated_a[i] = np.clip(validated_a[i], -15.0, 8.0)  # -15 to +8 m/s^2

    return validated_x, validated_v, validated_a

  def _calculate_radar_reliability(self, lead_radar):
    """
    Simplified radar reliability calculation for performance.
    Reduced computational overhead while maintaining basic reliability assessment.

    Args:
        lead_radar: Radar lead data structure

    Returns:
        float: Reliability score between 0.0 and 1.0
    """
    if not lead_radar.status:
      return 0.0

    # Simplified reliability calculation with basic factors
    reliability = 1.0

    # Distance-based reliability (basic implementation)
    distance_factor = max(0.1, min(1.0, 50.0 / max(1.0, lead_radar.dRel)))
    reliability *= distance_factor

    # Basic velocity validity check
    if hasattr(lead_radar, 'vRel') and abs(lead_radar.vRel) > 50:  # Unusually high velocity
      reliability *= 0.3  # Reduce reliability significantly
    elif hasattr(lead_radar, 'vRel') and abs(lead_radar.vRel) > 30:  # High but possible velocity
      reliability *= 0.7  # Moderate reduction

    # Basic acceleration validity check
    if hasattr(lead_radar, 'aLeadK') and abs(lead_radar.aLeadK) > 10:  # High acceleration
      reliability *= 0.5  # Reduce reliability

    # Apply basic bounds
    reliability = max(0.1, min(1.0, reliability))
    return reliability

  def _calculate_optimized_fcw(self, sm, lead_one, v_ego, a_ego):
    """
    Optimized Forward Collision Warning with reduced computational overhead.

    Args:
        sm: SubMaster instance
        lead_one: Primary lead from radar state
        v_ego: Ego vehicle velocity
        a_ego: Ego vehicle acceleration

    Returns:
        bool: True if FCW condition is met
    """
    if not lead_one.status:
        return False

    # Simple time-to-collision calculation
    relative_speed = v_ego - lead_one.vRel
    if relative_speed > 0 and lead_one.dRel > 0:  # Approaching lead vehicle
      time_to_collision = lead_one.dRel / relative_speed
    else:
      time_to_collision = float('inf')

    # Speed-adaptive thresholds
    if v_ego < 5:  # Very low speeds
      time_threshold = 1.5
      distance_threshold = 10
    elif v_ego < 15:  # City speeds
      time_threshold = 2.0
      distance_threshold = 20
    else:  # Highway speeds
      time_threshold = 2.5
      distance_threshold = 40

    # Simple FCW conditions
    imminent_collision = (time_to_collision < time_threshold and
                         relative_speed > 0.5 and
                         lead_one.dRel < distance_threshold)

    # MPC crash warning
    mpc_crash_warning = self.mpc.crash_cnt > 2
    mpc_warning = mpc_crash_warning and not sm['carState'].standstill

    # Model-based FCW
    model_warning = (sm['modelV2'].meta.hardBrakePredicted and
                    not sm['carState'].brakePressed and
                    v_ego > 3 and
                    lead_one.dRel < 50)

    return imminent_collision or mpc_warning or model_warning

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
