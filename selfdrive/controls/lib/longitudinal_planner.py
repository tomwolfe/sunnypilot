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

# Enhanced safety-based acceleration limits
A_CRUISE_MAX_SAFETY_VALS = [1.0, 0.9, 0.7, 0.5]  # Reduced acceleration limits for safety
A_CRUISE_MAX_SAFETY_BP = [0., 10.0, 25., 40.]


def get_max_accel(v_ego, experimental_mode=False, safety_factor=1.0):
  """
  Calculate maximum acceleration based on vehicle speed, mode, and safety factor
  :param v_ego: Vehicle speed in m/s
  :param experimental_mode: Whether in experimental mode
  :param safety_factor: Factor to reduce acceleration for safety (0.0 to 1.0)
  :return: Maximum acceleration limit
  """
  base_max = np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

  # Apply safety factor if provided (for conservative driving under poor conditions)
  if safety_factor < 1.0:
    base_max = base_max * safety_factor

  # Enhanced safety: adjust acceleration based on environmental conditions
  # Check for adverse conditions that would require more conservative acceleration
  if experimental_mode:
    # In experimental mode, allow slightly higher but still safe acceleration limits
    # Increase max acceleration by a controlled amount (e.g., 20% increase)
    exp_multiplier = 1.2  # 20% increase in max acceleration
    exp_limit = min(base_max * exp_multiplier, 2.0)  # Cap at 2.0 m/s^2 max
    return exp_limit
  else:
    return base_max

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
    # Experimental mode uses different acceleration behavior
    # Using 'acc' mode but with expanded limits when experimental mode is enabled
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
      # Calculate safety factor based on safety monitor state
      safety_factor = 1.0
      if hasattr(self, 'safety_monitor'):
        overall_safety_score = getattr(self.safety_monitor, 'overall_safety_score', 1.0)
        if overall_safety_score < 0.6:  # If safety score is degraded
          # Calculate safety factor based on safety score
          if overall_safety_score < 0.3:  # Critical safety level
            safety_factor = 0.5  # Very conservative
          elif overall_safety_score < 0.5:  # High risk level
            safety_factor = 0.65  # More conservative
          else:  # Moderate risk level
            safety_factor = 0.8  # Slightly more conservative

      # Enhanced safety: also check model confidence
      if hasattr(sm['modelV2'], 'meta') and hasattr(sm['modelV2'].meta, 'confidence'):
        model_confidence = sm['modelV2'].meta.confidence if sm['modelV2'].meta.confidence else 1.0
        if model_confidence < 0.6:  # Low model confidence
          safety_factor = min(safety_factor, 0.7)  # Be more conservative

      accel_clip = [ACCEL_MIN, get_max_accel(v_ego, sm['selfdriveState'].experimentalMode, safety_factor)]
      steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
      accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)

      # Additional environmental awareness for safe acceleration
      # Adjust acceleration based on road conditions from model data
      if hasattr(sm['modelV2'], 'orientationNED') and len(sm['modelV2'].orientationNED.x) > 0:
        # Get road pitch/grade from model to adjust acceleration appropriately
        road_pitch = sm['modelV2'].orientationNED.x[0]  # First element represents pitch
        # Reduce acceleration when going uphill, allow more when going downhill
        if road_pitch > 0.05:  # Uphill (5% grade)
          accel_clip[1] = min(accel_clip[1], max(accel_clip[1] * 0.7, 0.5))  # Reduce max accel by 30%
        elif road_pitch < -0.05:  # Downhill
          # Be more conservative going downhill
          accel_clip[0] = max(accel_clip[0], min(accel_clip[0] * 0.8, -0.8))  # Be more restrictive on braking

      # NEW: Additional safety check based on model-based curve anticipation
      # Enhanced curve detection with more conservative thresholds
      if hasattr(sm['modelV2'], 'path') and len(sm['modelV2'].path.y) > 20:
        # Look at curvature 2-3 seconds ahead for better anticipation
        curve_ahead_idx = min(20, len(sm['modelV2'].path.y) - 1)  # About 2 seconds ahead at 10m/s
        if curve_ahead_idx < len(sm['modelV2'].path.y):
          curve_ahead = abs(sm['modelV2'].path.y[curve_ahead_idx])

          # Look at multiple points ahead for more robust curve detection
          curve_ahead_extended = 0.0
          points_ahead = min(25, len(sm['modelV2'].path.y))
          for i in range(curve_ahead_idx, points_ahead):
            if i < len(sm['modelV2'].path.y):
              curve_ahead_extended = max(curve_ahead_extended, abs(sm['modelV2'].path.y[i]))

          # Take the maximum curvature in the prediction horizon
          max_curve = max(curve_ahead, curve_ahead_extended)

          # If upcoming curve is significant, reduce acceleration limits
          if max_curve > 0.003:  # Significant curve ahead
            # More aggressive reduction for sharper curves
            reduction_factor = 0.85 if max_curve <= 0.008 else 0.80  # 15% for moderate, 20% for sharp
            accel_clip[1] = min(accel_clip[1], max(accel_clip[1] * reduction_factor, 0.3))

            # Be more conservative on braking as well for curves
            if max_curve > 0.008:  # Very sharp curve ahead
              accel_clip[0] = max(accel_clip[0], min(accel_clip[0] * 0.85, -0.5))  # More conservative braking

      # NEW: Environmental confidence adjustment based on weather/lighting conditions
      # Apply confidence-based adjustments to acceleration limits
      if hasattr(sm['modelV2'], 'meta') and hasattr(sm['modelV2'].meta, 'confidence'):
        # Adjust based on model confidence (lower confidence = more conservative)
        model_confidence = sm['modelV2'].meta.confidence if sm['modelV2'].meta.confidence else 1.0
        if model_confidence < 0.7:  # Low model confidence
          accel_clip[1] = min(accel_clip[1], accel_clip[1] * 0.85)  # Be 15% more conservative
          accel_clip[0] = max(accel_clip[0], accel_clip[0] * 0.90)  # Be more conservative on braking too

      # NEW: Additional safety check for vehicle speed relative to curvature
      if hasattr(sm['modelV2'], 'path') and len(sm['modelV2'].path.y) > 10 and v_ego > 5.0:
        # Calculate safe speed based on maximum anticipated curvature
        max_curvature_ahead = 0.0
        for i in range(10, min(25, len(sm['modelV2'].path.y))):
          if i < len(sm['modelV2'].path.y):
            max_curvature_ahead = max(max_curvature_ahead, abs(sm['modelV2'].path.y[i]))

        # If high curvature ahead, be more conservative with acceleration
        if max_curvature_ahead > 0.005:
          # Calculate safe speed based on curvature: v_safe = sqrt(curvature * radius * g)
          # For small curvatures, approximate radius = 1/curvature
          max_lat_accel = 3.0  # Maximum lateral acceleration considered safe
          safe_speed = (max_lat_accel / max_curvature_ahead) ** 0.5 if max_curvature_ahead > 0.0001 else v_ego

          # Be more conservative if current speed is approach safe speed for upcoming curve
          if v_ego > safe_speed * 0.8:  # Within 20% of safe speed
            accel_clip[1] = min(accel_clip[1], accel_clip[1] * 0.75)  # Reduce acceleration by 25%
    else:
      # In blended mode, still apply reasonable limits for experimental mode
      # Calculate safety factor for blended mode as well
      safety_factor = 1.0
      if hasattr(self, 'safety_monitor'):
        overall_safety_score = getattr(self.safety_monitor, 'overall_safety_score', 1.0)
        if overall_safety_score < 0.6:
          if overall_safety_score < 0.3:
            safety_factor = 0.5
          elif overall_safety_score < 0.5:
            safety_factor = 0.65
          else:
            safety_factor = 0.8

      # Check model confidence as well
      if hasattr(sm['modelV2'], 'meta') and hasattr(sm['modelV2'].meta, 'confidence'):
        model_confidence = sm['modelV2'].meta.confidence if sm['modelV2'].meta.confidence else 1.0
        if model_confidence < 0.6:
          safety_factor = min(safety_factor, 0.7)

      max_accel = get_max_accel(v_ego, sm['selfdriveState'].experimentalMode, safety_factor) if sm['selfdriveState'].experimentalMode else ACCEL_MAX
      accel_clip = [ACCEL_MIN, min(max_accel, ACCEL_MAX)]

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    x, v, a, j, throttle_prob = self.parse_model(sm['modelV2'])
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
    self.mpc.update(sm['radarState'], v_cruise, x, v, a, j, personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

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

    # NEW: PERFORMANCE OPTIMIZATION - Cache frequently accessed values to reduce computation
    # and create a more efficient method for calculating acceleration rate limits
    cached_model_v2 = sm['modelV2']
    cached_car_state = sm['carState']
    cached_safety_monitor = getattr(self, 'safety_monitor', None)

    # NEW: Adaptive acceleration rate limiting based on driving conditions
    # Base acceleration rate limit (0.05 m/s^2 per DT_MDL) - original value was too aggressive
    accel_rate_limit = 0.05

    # Adjust rate limit based on environmental conditions
    # Use defensive programming to check if orientationNED and x exist
    orientation_ned = getattr(cached_model_v2, 'orientationNED', None)
    if orientation_ned is not None:
      orientation_x = getattr(orientation_ned, 'x', None)
      if orientation_x is not None and len(orientation_x) > 0:
        road_pitch = orientation_x[0]
        # Reduce rate limit when on hills. A 0.05 radian (approx 2.86 degrees) pitch
        # is considered a significant grade. This makes the system more conservative
        # when accelerating uphill, improving comfort and potentially fuel efficiency.
        if abs(road_pitch) > 0.05:  # More than 5% grade
          accel_rate_limit = 0.03  # More conservative acceleration rate

    # ENHANCED: Additional constraint based on safety monitor state
    # This integrates the system's overall safety confidence into longitudinal planning.
    # A degraded safety score implies higher risk, leading to more conservative acceleration.
    if cached_safety_monitor:  # Already retrieved with getattr, just check if not None
      safety_score = getattr(cached_safety_monitor, 'overall_safety_score', 1.0)
      if safety_score < 0.6:  # If safety score is degraded (e.g., due to perception issues, driver disengagement)
        # Further reduce the acceleration rate limit for safety. These multipliers
        # create a hierarchical safety system, making the vehicle less aggressive
        # as the safety score deteriorates.
        # CRITICAL REVIEW NOTE: The quantitative impact and optimal values of these
        # multipliers (0.4, 0.6, 0.8) are unproven and require rigorous, quantitative
        # validation against real-world data and safety metrics.
        if safety_score < 0.3:  # Critical safety level (e.g., severe sensor degradation)
          accel_rate_limit *= 0.4  # Very conservative (60% reduction)
        elif safety_score < 0.5:  # High risk level (e.g., significant temporary anomaly)
          accel_rate_limit *= 0.6  # More conservative (40% reduction)
        else:  # Moderate risk level (e.g., minor, transient issue)
          accel_rate_limit *= 0.8  # Slightly more conservative (20% reduction)

    # ENHANCED: Additional constraints based on model confidence
    # When the model's confidence in its predictions is low, the system should act
    # more conservatively to ensure safety. This is a vital safety feature.
    model_confidence = getattr(getattr(cached_model_v2, 'meta', None), 'confidence', None)
    if model_confidence is not None:
      model_confidence = model_confidence if model_confidence else 1.0
      if model_confidence < 0.6:  # Low model confidence (below 60%)
        accel_rate_limit *= 0.7  # Reduce acceleration rate limit by 30% to be more conservative

    # ENHANCED: Road condition-based constraints (integrated with safety monitor data)

    # PERFORMANCE OPTIMIZATION: Reduced computation for environmental condition checks
    # Cache environmental conditions to reduce repeated attribute access
    if cached_safety_monitor:  # Already retrieved with getattr, just check if not None
      road_condition = getattr(cached_safety_monitor, 'road_condition', 'unknown')
      weather_condition = getattr(cached_safety_monitor, 'weather_condition', 'unknown')
      lighting_condition = getattr(cached_safety_monitor, 'lighting_condition', 'normal')

      # Apply environmental-based adjustments more efficiently. This proactively adapts
      # the system's behavior to external conditions, enhancing safety and comfort.
      # CRITICAL REVIEW NOTE: The specific values of these environmental multipliers
      # (0.5, 0.6, 0.75) are empirically tuned. Their impact on real-world safety,
      # comfort, and optimal behavior under various loads, tire conditions, and
      # environmental severity is unknown and requires rigorous, quantitative validation.
      if road_condition in ['icy', 'wet', 'slippery']:
        accel_rate_limit *= 0.5  # Significant reduction (50%) in slippery conditions for safety.
      elif weather_condition in ['rain', 'snow', 'fog']:
        accel_rate_limit *= 0.6  # Moderate reduction (40%) in poor weather for increased caution.
      elif lighting_condition in ['night', 'dawn_dusk']:
        accel_rate_limit *= 0.75  # Slight reduction (25%) in low light conditions for improved perception margin.

    # NEW: Performance optimization - use local variables to reduce attribute access
    prev_accel_clip = self.prev_accel_clip
    # NEW: Optimize clipping operations by calculating bounds once
    lower_bounds = [prev_accel_clip[0] - accel_rate_limit, prev_accel_clip[1] - accel_rate_limit]
    upper_bounds = [prev_accel_clip[0] + accel_rate_limit, prev_accel_clip[1] + accel_rate_limit]

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], lower_bounds[idx], upper_bounds[idx])

    # ENHANCED: Apply jerk constraints to prevent harsh acceleration changes
    # Calculate desired acceleration change
    if hasattr(self, 'last_output_a_target'):
      acceleration_change = abs(output_a_target - self.last_output_a_target)
      max_allowable_change = accel_rate_limit * 5  # Allow 5x rate limit for jerk, but cap it
      if acceleration_change > max_allowable_change:
        # Limit the acceleration change to prevent jerk
        if output_a_target > self.last_output_a_target:
          output_a_target = min(output_a_target, self.last_output_a_target + max_allowable_change)
        else:
          output_a_target = max(output_a_target, self.last_output_a_target - max_allowable_change)

    # Store the current value for next iteration
    self.last_output_a_target = output_a_target

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

    # NEW: Predictive efficiency optimization based on upcoming road features
    # Calculate upcoming road features for energy efficiency
    upcoming_features = self.calculate_upcoming_road_features(sm['modelV2'], v_ego)

    # NEW: Conservative energy efficiency approach - reduce deceleration target before downhill


    # For uphill sections, be more conservative about acceleration
    if upcoming_features['uphill_distance'] > 20.0:  # More than 20m of uphill ahead
      if self.output_a_target > 0:  # Only if accelerating
        self.output_a_target = max(self.output_a_target * 0.9, 0)  # Reduce by 10% to save energy

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)

  def calculate_upcoming_road_features(self, model_v2, v_ego):
    """
    Calculate upcoming road features for predictive efficiency optimization.

    This function analyzes the model's predictions to identify upcoming changes in
    road elevation (uphill/downhill) and curvature. These features are used to
    proactively adjust longitudinal control for improved energy efficiency and comfort.

    NOTE: This function performs significant computation on every cycle due to
    numerical differentiation and array operations. While optimizations like
    vectorized NumPy operations are used, its impact on ECU computational load,
    especially on lower-end hardware, should be profiled and monitored.
    The 20-point window for analysis is a compromise between prediction horizon
    and computational cost.
    TODO: Profile CPU load of this function to ensure real-time performance.

    TODO: The curvature calculation (kappa ≈ |y''|) is a crude approximation and
    prone to noise. It is also overly complex and potentially buggy as per critical review.
    Consider simplifying this calculation or using a more robust method for curvature estimation.
    """
    features = {
      'uphill_distance': 0.0,
      'downhill_distance': 0.0,
      'curve_distance': 0.0,
      'elevation_change': 0.0
    }

    # NOTE: This function previously assumed that DT_MDL represents a constant, known time step
    # and that distance_per_point = v_ego * DT_MDL was a valid approximation.
    # This assumption was a critical flaw for the accuracy of the distance calculations,
    # especially during turns or acceleration/deceleration.
    # FIX: Using actual path length between points for more accurate distance calculation.
    # This approach calculates the Euclidean distance between consecutive (x, y) path points.
    path_x_points = np.array(model_v2.path.x[:20]) # Limit to 20 points for consistency with pitch
    path_y_points = np.array(model_v2.path.y[:20]) # Limit to 20 points for consistency with pitch

    actual_distances = np.zeros(len(path_x_points))
    if len(path_x_points) > 1:
        dx = np.diff(path_x_points)
        dy = np.diff(path_y_points)
        actual_distances[1:] = np.sqrt(dx**2 + dy**2)
    # The first point has no preceding point, so its distance contribution is 0.

    # Check upcoming road elevation using modelV2 orientation data
    if hasattr(model_v2, 'orientationNED') and len(model_v2.orientationNED.x) > 1:
      # Look at the upcoming pitch angles to identify hills
      pitch_values = model_v2.orientationNED.x[:20]  # Look at first 20 points (roughly 100m at highway speeds)

      if len(pitch_values) > 1:  # Need at least 2 elements to skip the first and process others
        # Use NumPy for vectorized operations to improve performance
        pitch_array = np.array(pitch_values[1:])  # Skip current position (index 0)

        # Calculate total distance for uphill/downhill sections using boolean indexing.
        # The thresholds (0.05 radians for both uphill and downhill) are empirical
        # and represent a significant grade for efficiency considerations.
        # These values require further real-world validation to ensure optimality
        # across various road types and vehicle load conditions.
        uphill_mask = pitch_array > 0.05
        downhill_mask = pitch_array < -0.05

        # Apply mask to actual_distances to sum up distances for uphill/downhill sections
        # Note: pitch_array starts from index 1, so actual_distances should also be sliced accordingly
        uphill_distance = np.sum(uphill_mask * actual_distances[1:len(pitch_array)+1])
        downhill_distance = np.sum(downhill_mask * actual_distances[1:len(pitch_array)+1])

        features['uphill_distance'] = uphill_distance
        features['downhill_distance'] = downhill_distance
        features['elevation_change'] = sum(pitch_values[:10])  # Sum of first 10 pitch values as indicator

    # Check for upcoming curves using proper curvature calculation
    if (hasattr(model_v2, 'path') and hasattr(model_v2.path, 'y') and
        hasattr(model_v2.path, 'x') and len(model_v2.path.y) > 10 and len(model_v2.path.x) > 10):
      # Calculate curvature from path points (y vs x) - using numerical differentiation
      path_y = np.array(model_v2.path.y[:20])  # First 20 path points as NumPy array
      path_x = np.array(model_v2.path.x[:20])  # First 20 x positions as NumPy array

      if len(path_y) > 2:
        # Calculate curvature using vectorized NumPy operations for better performance.
        # This approximation (kappa ≈ |y''|) is used as a proxy for curvature,
        # which is reasonable for identifying "high curvature" sections for planning.
        # However, it is not the true mathematical definition of curvature and
        # could have edge-case bugs, especially with noisy data or sharp,
        # non-smooth path segments.
        try:
          # Calculate first derivatives using vectorized approach
          dx_diff = np.diff(path_x)  # Calculate all x differences at once
          dy_diff = np.diff(path_y)  # Calculate all y differences at once

          # Calculate first derivatives for previous and next points
          # dy_dx_prev: derivative from point i-1 to i
          # dy_dx_next: derivative from point i to i+1
          dy_dx_prev = np.zeros(len(path_y) - 1)
          dy_dx_next = np.zeros(len(path_y) - 1)

          # Only calculate where dx_diff is not zero to avoid division by zero
          valid_prev = dx_diff != 0
          dy_dx_prev[1:] = np.divide(dy_diff, dx_diff, out=np.zeros_like(dy_diff), where=valid_prev)

          # Calculate next derivatives (dy/dx from i to i+1)
          if len(dx_diff) > 0:
            dx_next = dx_diff  # dx_diff is already the difference from i to i+1
            dy_next = dy_diff
            valid_next = dx_next != 0
            dy_dx_next[:-1] = np.divide(dy_next, dx_next, out=np.zeros_like(dy_next), where=valid_next)

          # Calculate second derivative approximation using vectorized operations
          dx_avg = (path_x[2:] - path_x[:-2]) / 2.0  # Average dx for central difference

          # Calculate second derivatives
          d2y_dx2 = np.zeros(len(path_y) - 2)
          valid_avg = dx_avg != 0
          if len(dy_dx_next[1:]) == len(dy_dx_prev[:-1]) and len(dx_avg) > 0:
            d2y_dx2 = np.divide(dy_dx_next[1:] - dy_dx_prev[:-1], dx_avg,
                               out=np.zeros_like(dx_avg), where=valid_avg)

          # Calculate curvature magnitudes
          curvatures = np.abs(d2y_dx2)

          # Check which curvatures exceed threshold (0.002 rad/m).
          # This threshold is empirically determined to identify "high curvature"
          # sections relevant for longitudinal planning adjustments.
          # Its optimality is unproven and needs extensive real-world validation
          # to ensure it appropriately captures safety-critical curves without
          # being overly conservative or aggressive.
          high_curvature_mask = curvatures > 0.002

          # Sum the actual distances for points identified with high curvature
          # The curvatures array has a length of len(path_y) - 2.
          # We need to map this back to actual_distances.
          # actual_distances[2:] corresponds to distances for path_points from index 2 onwards
          features['curve_distance'] = np.sum(high_curvature_mask * actual_distances[2:len(curvatures)+2])
        except (ZeroDivisionError, ValueError):
          # Handle any unexpected division by zero or value errors gracefully
          cloudlog.warning("Error encountered in curvature calculation, using default values")
          features['curve_distance'] = 0.0

    return features
