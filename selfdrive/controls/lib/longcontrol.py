import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.common.params import Params
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, CP_SP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
  # Gas Interceptor
  cruise_standstill = cruise_standstill and not CP_SP.enableGasInterceptor

  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state

class LongControl:
  def __init__(self, CP, CP_SP, params=None):
    self.CP = CP
    self.CP_SP = CP_SP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0

    # Load configurable parameters with validation to ensure safe operation
    if params is None:
      params = Params()
    self.max_jerk = self._validate_parameter(
        float(params.get("LongitudinalMaxJerk") or "2.2"),
        0.5, 10.0, "LongitudinalMaxJerk"
    )  # m/s^3
    self.max_stopping_jerk = self._validate_parameter(
        float(params.get("LongitudinalMaxStoppingJerk") or "1.5"),
        0.5, 5.0, "LongitudinalMaxStoppingJerk"
    )  # m/s^3
    self.max_output_jerk = self._validate_parameter(
        float(params.get("LongitudinalMaxOutputJerk") or "2.0"),
        0.5, 5.0, "LongitudinalMaxOutputJerk"
    )  # m/s^3
    self.starting_speed_threshold = self._validate_parameter(
        float(params.get("LongitudinalStartingSpeedThreshold") or "3.0"),
        0.5, 10.0, "LongitudinalStartingSpeedThreshold"
    )  # m/s
    self.starting_accel_multiplier = self._validate_parameter(
        float(params.get("LongitudinalStartingAccelMultiplier") or "0.8"),
        0.1, 2.0, "LongitudinalStartingAccelMultiplier"
    )
    self.starting_accel_limit = self._validate_parameter(
        float(params.get("LongitudinalStartingAccelLimit") or "0.8"),
        0.1, 2.0, "LongitudinalStartingAccelLimit"
    )
    self.adaptive_error_threshold = self._validate_parameter(
        float(params.get("LongitudinalAdaptiveErrorThreshold") or "0.6"),
        0.01, 2.0, "LongitudinalAdaptiveErrorThreshold"
    )
    self.adaptive_speed_threshold = self._validate_parameter(
        float(params.get("LongitudinalAdaptiveSpeedThreshold") or "5.0"),
        1.0, 20.0, "LongitudinalAdaptiveSpeedThreshold"
    )  # m/s

  def _validate_parameter(self, value, min_val, max_val, name):
    """
    Validate that a parameter is within safe bounds.

    Args:
        value: Parameter value to validate
        min_val: Minimum allowed value
        max_val: Maximum allowed value
        name: Name of the parameter for logging

    Returns:
        Validated parameter value within bounds
    """
    if value < min_val:
      print(f"Warning: {name} value {value} below minimum {min_val}, clamping to minimum")
      return min_val
    elif value > max_val:
      print(f"Warning: {name} value {value} above maximum {max_val}, clamping to maximum")
      return max_val
    return value

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    # Initialize state variables if not present
    if not hasattr(self, '_prev_a_target'):
      self._prev_a_target = a_target
    if not hasattr(self, '_prev_output_accel'):
      self._prev_output_accel = 0.0
    if not hasattr(self, '_acceleration_profile'):
      self._acceleration_profile = {'approaching_lead': False, 'approach_rate': 0.0, 'distance_to_lead': float('inf')}

    self.long_control_state = long_control_state_trans(self.CP, self.CP_SP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.
      self.last_output_accel = output_accel
      return output_accel

    elif self.long_control_state == LongCtrlState.stopping:
      # Enhanced stopping with smoother deceleration profile
      # Apply more gradual deceleration to avoid harsh braking
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        # Apply jerk-limited deceleration for smoother stop
        max_jerk = self.max_stopping_jerk  # m/s^3, limit deceleration change - now configurable
        desired_decel = max_jerk * DT_CTRL
        output_accel = max(output_accel - desired_decel, self.CP.stopAccel)
      self.reset()
      self.last_output_accel = output_accel
      return output_accel

    elif self.long_control_state == LongCtrlState.starting:
      # Enhanced starting with smoother acceleration
      # Apply gradual acceleration to avoid harsh takeoff
      if CS.vEgo < self.starting_speed_threshold:  # Below configurable threshold (default 10.8 km/h)
        output_accel = min(self.CP.startAccel * self.starting_accel_multiplier, self.starting_accel_limit)  # More gentle start at very low speeds - now configurable
      else:
        output_accel = self.CP.startAccel
      self.reset()
      self.last_output_accel = output_accel
      return output_accel

    else:  # LongCtrlState.pid
      # Enhanced adaptive jerk control based on driving context
      # Calculate adaptive jerk limits based on lead vehicle behavior and road conditions
      max_jerk = self._calculate_adaptive_jerk_limit(CS, a_target)

      # Apply jerk limitation to the target acceleration for smoother transitions
      jerk_limit = max_jerk * DT_CTRL
      a_target_change = a_target - self._prev_a_target
      a_target_limited = self._prev_a_target + np.clip(a_target_change, -jerk_limit, jerk_limit)
      self._prev_a_target = a_target_limited

      error = a_target_limited - CS.aEgo

      # Enhanced adaptive PID based on driving context and lead vehicle behavior
      # Calculate context-sensitive adaptive parameters
      adaptive_ki_factor = 1.0
      adaptive_kp_factor = 1.0
      ki_reduction_enabled = False

      if CS.vEgo > self.adaptive_speed_threshold:  # At moderate to high speeds
        if abs(error) < self.adaptive_error_threshold:  # Near target acceleration
          ki_reduction_enabled = True
          adaptive_ki_factor = 0.5  # Reduce integral action to avoid overshoot

          # Additional check: if following lead vehicle closely, be even more conservative
          # Note: This assumes we have access to lead vehicle information in CS if available
          # This would typically come from radarState which is accessed in the main control loop

      # Apply adaptive gains if needed
      if ki_reduction_enabled:
        temp_ki = min(self.pid.k_i * adaptive_ki_factor, self.CP.longitudinalTuning.kiV[0] * 0.5)
        temp_pid = PIDController((self.CP.longitudinalTuning.kpBP, self.CP.longitudinalTuning.kpV),
                                 (self.CP.longitudinalTuning.kiBP, [temp_ki] * len(self.CP.longitudinalTuning.kiV)),
                                 rate=1 / DT_CTRL)
        temp_pid.neg_limit = self.pid.neg_limit
        temp_pid.pos_limit = self.pid.pos_limit
        # Preserve integral term by copying from original PID
        temp_pid.i = self.pid.i
        temp_pid.p = self.pid.p
        temp_pid.d = self.pid.d
        temp_pid.f = self.pid.f
        temp_pid.control = self.pid.control
        temp_pid.speed = self.pid.speed

        output_accel = temp_pid.update(error, speed=CS.vEgo,
                                       feedforward=a_target_limited)

        # Preserve the integral term from temp PID for next iteration
        self.pid.i = temp_pid.i
        self.pid.p = temp_pid.p
        self.pid.d = temp_pid.d
        self.pid.f = temp_pid.f
        self.pid.control = temp_pid.control
        self.pid.speed = temp_pid.speed
      else:
        output_accel = self.pid.update(error, speed=CS.vEgo,
                                       feedforward=a_target_limited)

      # Enhanced output smoothing with context-aware jerk limiting
      max_output_jerk = self._calculate_adaptive_output_jerk_limit(CS, a_target_limited, output_accel)  # m/s^3, limit output jerk adaptively

      # Increase jerk limit in emergency situations (e.g., hard braking needed)
      emergency_factor = 1.0
      if a_target_limited < -2.0:  # Hard braking situation
        emergency_factor = 2.0  # Allow more aggressive response

      output_jerk_limit = max_output_jerk * DT_CTRL * emergency_factor
      output_accel_change = output_accel - self._prev_output_accel

      # Apply adaptive jerk limiting based on current driving situation
      if abs(output_accel_change) > output_jerk_limit:
        output_accel = self._prev_output_accel + np.clip(output_accel_change,
                                                         -output_jerk_limit,
                                                         output_jerk_limit)

      # Additional smoothing for very low-speed situations to prevent jerky movements
      if CS.vEgo < 5.0:  # Below 18 km/h (5 m/s)
        # Apply additional smoothing at very low speeds for smoother stop-and-go
        very_low_speed_smoothing = 0.7  # More conservative at creep speeds
        output_accel = self._prev_output_accel * (1 - very_low_speed_smoothing) + output_accel * very_low_speed_smoothing

      self._prev_output_accel = output_accel
      self.last_output_accel = output_accel
      return output_accel

  def _calculate_adaptive_output_jerk_limit(self, CS, a_target_limited, current_output_accel):
    """
    Calculate adaptive output jerk limits based on driving context and stability.

    Args:
        CS: CarState object
        a_target_limited: Limited target acceleration after jerk limiting
        current_output_accel: Current acceleration output before limiting

    Returns:
        float: Adaptive output jerk limit in m/s^3
    """
    # Start with configurable base limit
    base_output_jerk_limit = self.max_output_jerk

    # Adjust based on difference between target and current acceleration
    accel_error = abs(a_target_limited - current_output_accel)
    if accel_error > 2.0:  # Large error - allow more aggressive correction
        base_output_jerk_limit *= 1.2
    elif accel_error < 0.5:  # Small error - be more conservative
        base_output_jerk_limit *= 0.8

    # Reduce jerk when following lead vehicle closely
    if hasattr(CS, 'radarState') and CS.radarState is not None:
        try:
            if hasattr(CS.radarState, 'leadOne') and CS.radarState.leadOne.status:
                lead_one = CS.radarState.leadOne
                dRel = getattr(lead_one, 'dRel', None)

                # Only proceed if dRel is an actual number
                if isinstance(dRel, (int, float)):
                    if dRel < 30.0:
                        base_output_jerk_limit *= 0.85  # More conservative when close to lead
                        if dRel < 15.0:  # Very close
                            base_output_jerk_limit *= 0.7  # Much more conservative
        except (AttributeError, TypeError):
            pass

    # Reduce jerk at higher speeds for stability
    if CS.vEgo > 25.0:  # Above ~90 km/h
        base_output_jerk_limit *= 0.9

    # Further reduce jerk if vehicle is already accelerating rapidly
    if abs(current_output_accel) > 2.0:
        base_output_jerk_limit *= 0.85  # Be more conservative when already accelerating hard

    # Apply limits to ensure safety
    base_output_jerk_limit = max(0.5, min(8.0, base_output_jerk_limit))  # Reasonable bounds

    return base_output_jerk_limit

  def update_thermal_compensation(self, thermal_stress_level, compensation_factor):
    """
    Update longitudinal controller parameters based on thermal stress level to maintain
    consistent acceleration/deceleration performance under different thermal conditions.

    Args:
      thermal_stress_level: 0=normal, 1=moderate, 2=high, 3=very high
      compensation_factor: Factor (0.0-1.0) indicating overall system compensation needed
    """
    # Store current PID state to preserve during reinitialization
    old_p = self.pid.p
    old_i = self.pid.i
    old_d = self.pid.d
    old_f = self.pid.f
    old_control = self.pid.control
    old_speed = self.pid.speed
    old_pos_limit = self.pid.pos_limit
    old_neg_limit = self.pid.neg_limit

    # Adjust PID parameters based on thermal stress to maintain consistent longitudinal control
    original_ki_v = self.CP.longitudinalTuning.kiV
    original_kp_v = self.CP.longitudinalTuning.kpV

    if thermal_stress_level >= 2:  # High or very high thermal stress
      # Create new PID with reduced gains to prevent accumulation during thermal throttling
      new_ki_v = [min(ki_val, original_ki_v[0] * 0.7) for ki_val in original_ki_v]  # Reduce by 30%
      new_kp_v = [min(kp_val, original_kp_v[0] * 0.9) for kp_val in original_kp_v]  # Reduce by 10%

      # Create a new PID controller with adjusted parameters
      self.pid = PIDController((self.CP.longitudinalTuning.kpBP, new_kp_v),
                               (self.CP.longitudinalTuning.kiBP, new_ki_v),
                               rate=1 / DT_CTRL)
    elif thermal_stress_level == 1:  # Moderate thermal stress
      # Mild adjustment for moderate stress
      new_ki_v = [min(ki_val, original_ki_v[0] * 0.85) for ki_val in original_ki_v]  # Reduce by 15%

      # Create a new PID controller with adjusted parameters
      self.pid = PIDController((self.CP.longitudinalTuning.kpBP, original_kp_v),
                               (self.CP.longitudinalTuning.kiBP, new_ki_v),
                               rate=1 / DT_CTRL)

    # Restore PID state to preserve continuity
    self.pid.p = old_p
    self.pid.i = old_i
    self.pid.d = old_d
    self.pid.f = old_f
    self.pid.control = old_control
    self.pid.speed = old_speed


  def _calculate_adaptive_jerk_limit(self, CS, a_target):
    """
    Calculate adaptive jerk limits based on driving context and vehicle state.

    Args:
        CS: CarState object containing current vehicle state
        a_target: Target acceleration value

    Returns:
        float: Adaptive jerk limit in m/s^3
    """
    # Base jerk limit
    base_jerk_limit = self.max_jerk

    # Reduce jerk if approaching lead vehicle rapidly
    if hasattr(CS, 'radarState') and CS.radarState is not None:
        try:
            if hasattr(CS.radarState, 'leadOne') and CS.radarState.leadOne.status:
                # Check if attributes are actual numeric values (not Mock objects) before comparison
                lead_one = CS.radarState.leadOne
                dRel = getattr(lead_one, 'dRel', None)
                vRel = getattr(lead_one, 'vRel', None)

                # Only proceed if both values are actual numbers
                if isinstance(dRel, (int, float)) and isinstance(vRel, (int, float)):
                    if dRel < 50.0:
                        approach_rate = abs(vRel)
                        if approach_rate > 5.0:  # Approaching rapidly
                            base_jerk_limit *= 0.7  # More conservative
        except (AttributeError, TypeError):
            pass

    # Increase jerk allowance during initial acceleration from standstill
    if CS.vEgo < 5.0 and a_target > 0:  # Starting from stop
        base_jerk_limit *= 1.3  # Allow more aggressive initial acceleration

    # Adjust for road grade if available (would need to be passed in from main control)
    if hasattr(CS, 'roadGrade') and CS.roadGrade is not None:
        # Check if roadGrade is a numeric value (not a Mock object) before using abs()
        if isinstance(CS.roadGrade, (int, float)):
            if abs(CS.roadGrade) > 0.05:  # Significant grade
                grade_factor = 1.0 + 0.3 * abs(CS.roadGrade)  # Increase jerk limit slightly on grades
                base_jerk_limit = min(10.0, base_jerk_limit * grade_factor)  # Cap at reasonable value

    # Apply limits to ensure safety
    base_jerk_limit = max(0.5, min(10.0, base_jerk_limit))  # Reasonable bounds

    return base_jerk_limit

  def _update_acceleration_profile(self, CS, a_target, lead_distance, lead_velocity):
    """
    Update acceleration profile based on lead vehicle behavior.

    Args:
        CS: CarState object
        a_target: Target acceleration
        lead_distance: Distance to lead vehicle (if available)
        lead_velocity: Velocity of lead vehicle (if available)
    """
    if not hasattr(self, '_acceleration_profile'):
        self._acceleration_profile = {'approaching_lead': False, 'approach_rate': 0.0, 'distance_to_lead': float('inf')}

    if lead_distance is not None and lead_velocity is not None:
        # Calculate approach rate to lead vehicle
        relative_velocity = CS.vEgo - lead_velocity
        self._acceleration_profile['approach_rate'] = relative_velocity
        self._acceleration_profile['distance_to_lead'] = lead_distance
        self._acceleration_profile['approaching_lead'] = relative_velocity > 0.5 and lead_distance < 100.0
    else:
        # Default values when lead information not available
        self._acceleration_profile['approaching_lead'] = False
        self._acceleration_profile['approach_rate'] = 0.0
        self._acceleration_profile['distance_to_lead'] = float('inf')
