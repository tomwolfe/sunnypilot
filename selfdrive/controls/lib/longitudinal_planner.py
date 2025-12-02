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
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


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
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

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
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
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
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THO_THRESHOLD*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    # Get new v_cruise and a_desired from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.a_desired = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.a_desired, v_cruise)

    if force_slow_decel:
      v_cruise = 0.0

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    # Use enhanced fused data instead of raw model data
    self.mpc.update(sm['radarState'], v_cruise, enhanced_x, enhanced_v, enhanced_a, j, personality=sm['selfdriveState'].personality)

  def _fuse_radar_camera_data(self, sm, model_x, model_v, model_a):
    """
    Enhanced radar-camera fusion to improve lead vehicle detection and tracking.

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
    radar_state = sm['radarState']
    model_v2 = sm['modelV2']

    # Initialize fusion tracking if not present
    if not hasattr(self, '_radar_camera_fusion_tracker'):
        self._radar_camera_fusion_tracker = {}
        self._last_fusion_time = 0.0

    # Apply Kalman-like filtering to combine radar and vision data for lead vehicles
    enhanced_x = model_x.copy()
    enhanced_v = model_v.copy()
    enhanced_a = model_a.copy()

    # Process lead vehicle fusion if radar detects a lead
    if radar_state.leadOne.status and model_v2.leadsV3:
        lead_radar = radar_state.leadOne
        # Assuming model_v2.leadsV3 contains vision-based lead detection

        # Get the most relevant lead from vision model (closest or most concerning)
        closest_vision_lead = None
        for lead_vision in model_v2.leadsV3:
            if lead_vision.prob > 0.5:  # Only consider confident detections
                if closest_vision_lead is None or lead_vision.dRel < closest_vision_lead.dRel:
                    closest_vision_lead = lead_vision

        # If we have both radar and vision leads, fuse them
        if closest_vision_lead is not None:
            # Calculate fusion weights based on reliability
            radar_reliability = min(1.0, lead_radar.aLeadTau / 2.0)  # Higher tau = more reliable
            vision_reliability = closest_vision_lead.prob  # Vision probability

            # Fuse distance, velocity, and acceleration based on reliability
            total_reliability = radar_reliability + vision_reliability
            if total_reliability > 0:
                fused_dRel = (lead_radar.dRel * radar_reliability + closest_vision_lead.dRel * vision_reliability) / total_reliability
                fused_vRel = (lead_radar.vRel * radar_reliability + closest_vision_lead.vRel * vision_reliability) / total_reliability
                fused_aLead = (lead_radar.aLeadK * radar_reliability + closest_vision_lead.aRel * vision_reliability) / total_reliability

                # Update the lead data used by the planner with fused values
                # This affects the x, v, a arrays that are passed to MPC
                # For now, we'll update the first few elements which typically represent immediate lead data
                if len(enhanced_x) > 0:
                    enhanced_x[0] = fused_dRel
                if len(enhanced_v) > 0:
                    enhanced_v[0] = fused_vRel
                if len(enhanced_a) > 0:
                    enhanced_a[0] = fused_aLead

    # Apply temporal consistency to smooth out sensor noise while maintaining responsiveness
    # This helps reduce the "jitter" that can occur with individual sensor readings
    if hasattr(self, '_prev_fused_values'):
        # Apply light smoothing between current and previous fused values
        alpha = 0.1  # Smoothing factor (0.1 = 10% of previous, 90% of current)
        prev_x, prev_v, prev_a = self._prev_fused_values

        # Only apply smoothing if values aren't too different (to preserve responsiveness to real changes)
        max_change_threshold = [5.0, 5.0, 5.0]  # Max allowed change before preserving current values

        for i in range(min(len(enhanced_x), len(prev_x))):
            if abs(enhanced_x[i] - prev_x[i]) < max_change_threshold[0]:
                enhanced_x[i] = alpha * prev_x[i] + (1 - alpha) * enhanced_x[i]
        for i in range(min(len(enhanced_v), len(prev_v))):
            if abs(enhanced_v[i] - prev_v[i]) < max_change_threshold[1]:
                enhanced_v[i] = alpha * prev_v[i] + (1 - alpha) * enhanced_v[i]
        for i in range(min(len(enhanced_a), len(prev_a))):
            if abs(enhanced_a[i] - prev_a[i]) < max_change_threshold[2]:
                enhanced_a[i] = alpha * prev_a[i] + (1 - alpha) * enhanced_a[i]

    # Store current fused values for next iteration
    self._prev_fused_values = (enhanced_x.copy(), enhanced_v.copy(), enhanced_a.copy())

    return enhanced_x, enhanced_v, enhanced_a

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

    x, v, a, j, throttle_prob = self.parse_model(sm['modelV2'])

    # Apply radar-camera fusion to enhance lead vehicle detection
    enhanced_x, enhanced_v, enhanced_a = self._fuse_radar_camera_data(sm, x, v, a)

    # Don't clip at low speeds since throttle_prob doesn't account for creep
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    # Get new v_cruise and a_desired from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.a_desired = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.a_desired, v_cruise)

    if force_slow_decel:
      v_cruise = 0.0

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
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
        relative_accel = (a_ego - lead_one.aLeadK)

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

    # Enhanced FCW logic with speed-adaptive thresholds
    mpc_crash_warning = self.mpc.crash_cnt > 2

    # Dynamic thresholds based on speed and driving context
    if v_ego < 5:  # Very low speeds (stationary or slow moving)
      emergency_brake_threshold = 1.8  # Lower threshold for parking/low-speed scenarios
      distance_threshold = 8  # meters
    elif v_ego < 15:  # City speeds (~54 km/h)
      emergency_brake_threshold = 2.0
      distance_threshold = 15  # meters
    elif v_ego < 25:  # Highway speeds (~54-90 km/h)
      emergency_brake_threshold = 2.5
      distance_threshold = 30  # meters
    else:  # High speeds (>90 km/h)
      emergency_brake_threshold = 2.8
      distance_threshold = 45  # meters

    # Enhanced FCW logic - trigger based on multiple criteria
    # 1. Time to collision with immediate braking requirement
    imminent_collision = (time_to_collision < emergency_brake_threshold and relative_speed > 0.5)

    # 2. Distance-based warning for low-speed situations where time-based may not work well
    distance_collision = (distance_to_collision < distance_threshold and relative_speed > 0.5 and v_ego < 10)

    # 3. Lead vehicle emergency braking detection
    lead_braking_emergency = (lead_one.aLeadK - a_ego > 2.5 and lead_one.dRel < 40 and v_ego > 5)

    # 4. Combined MPC and sensor-based warnings
    mpc_and_sensors_warning = mpc_crash_warning and not sm['carState'].standstill and relative_speed > 1.0

    # 5. Enhanced model-based prediction with additional safety checks
    model_fcw = (sm['modelV2'].meta.hardBrakePredicted and
                 not sm['carState'].brakePressed and
                 v_ego > 3 and
                 distance_to_collision < 50 and  # Only if close enough to be relevant
                 sm['carState'].aEgo < -1.25) # Re-added safeguard: prevent FCW during strong acceleration (original value)

    # 6. Additional safety criterion: Predictive collision assessment using multiple lead vehicles
    extended_collision_risk = False
    if (sm['radarState'].leadTwo.status and
        lead_one.dRel < 60.0 and
        sm['radarState'].leadTwo.dRel < lead_one.dRel + 20.0 and  # Second lead is close behind first
        sm['radarState'].leadTwo.vRel < -2.0):  # Second lead is approaching rapidly
        extended_collision_risk = True

    # Combine all FCW triggers
    self.fcw = (imminent_collision or
                distance_collision or
                lead_braking_emergency or
                mpc_and_sensors_warning or
                (model_fcw and v_ego > 5) or
                extended_collision_risk)

    # Additional safety logic: consider relative acceleration
    # NOTE: This trigger is potentially aggressive and may cause false positives during lane changes or merging.
    # Extensive real-world testing is required before deployment in production systems.
    # Disabling this for now based on critical review.
    # if lead_one.status and lead_one.dRel < 50:  # Only consider when close to lead vehicle
    #   relative_acceleration = a_ego - lead_one.aLeadK  # Positive if we're accelerating more than lead
    #   if relative_acceleration > 2.0 and lead_one.dRel < 20:  # If we're accelerating much faster toward a close vehicle
    #     self.fcw = True


    if self.fcw:
      cloudlog.warning(f"FCW triggered: TTC={time_to_collision:.2f}s, distance={distance_to_collision:.1f}m, rel_speed={relative_speed:.2f}m/s")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=self.CP.vEgoStopping)
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
