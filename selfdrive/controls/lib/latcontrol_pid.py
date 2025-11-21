import math

from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController
from openpilot.common.filter_simple import FirstOrderFilter # Import FirstOrderFilter
from openpilot.common.swaglog import cloudlog # Import cloudlog
from cereal import log


class LatControlPID(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    # Use curvature gain interpolation from car params if available, otherwise use default
    curvature_gain_interp = CP_SP.curvatureGainInterp if CP_SP.curvatureGainInterp else [[0.0], [1.0]]
    # Use configurable max curvature gain multiplier from car params, default to 4.0 for safety
    max_curvature_gain_multiplier = getattr(CP_SP, 'maxCurvatureGainMultiplier', 4.0)
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                             (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                             k_curvature=curvature_gain_interp,
                             max_curvature_gain_multiplier=max_curvature_gain_multiplier,
                             pos_limit=self.steer_max, neg_limit=-self.steer_max,
                             vehicle_mass=CP.mass)  # Pass vehicle mass for adaptive oscillation thresholds
    self.ff_factor = CP.lateralTuning.pid.kf
    self.get_steer_feedforward = CI.get_steer_feedforward_function()
    # Initialize FirstOrderFilter with base time constant and consider adaptive filtering
    self.base_time_constant = 0.1  # Base filter time constant (100ms)
    self.curvature_filter = FirstOrderFilter(0., self.base_time_constant, dt)
    # Add monitoring variables for field validation
    self.monitoring_initialized = False
    self.last_monitoring_log_time = 0

  def _get_adaptive_time_constant(self, v_ego, curvature):
    """
    Calculate adaptive time constant based on vehicle speed and curvature
    Lower time constant for high curvature at high speed (faster response)
    Higher time constant for low curvature at low speed (smoother response)
    """
    # Base time constant
    time_constant = self.base_time_constant

    # Adjust based on speed: at higher speeds, we may want slightly faster response
    if v_ego > 25.0:  # Above ~55 mph
        time_constant *= 0.8  # Slightly faster response
    elif v_ego < 5.0:  # At very low speeds
        time_constant *= 1.5  # More smoothing at low speeds

    # Adjust based on curvature: at higher curvature, maintain more smoothing to reduce noise impact
    if curvature > 0.05:  # High curvature turn
        time_constant = max(time_constant, 0.05)  # Maintain minimum smoothing
    elif curvature < 0.005:  # Nearly straight
        time_constant = min(time_constant, 0.15)  # Allow more responsiveness

    # Ensure time constant stays within reasonable bounds
    return max(0.02, min(0.2, time_constant))  # Clamp between 20ms and 200ms

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    # Apply adaptive low-pass filter to desired_curvature based on driving conditions
    # Use different time constants based on vehicle speed and curvature for optimal response
    adaptive_time_constant = self._get_adaptive_time_constant(CS.vEgo, abs(desired_curvature))
    # Update filter time constant if it has changed significantly
    if abs(self.curvature_filter.rc - adaptive_time_constant) > 0.001:
      self.curvature_filter.update_alpha(adaptive_time_constant)  # Update filter time constant if needed
    desired_curvature = self.curvature_filter.update(desired_curvature)
    cloudlog.debug(f"Desired Curvature: {desired_curvature:.4f}, Adaptive Time Constant: {adaptive_time_constant:.3f}") # Log desired curvature and time constant

    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)

    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
    angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg
    error = angle_steers_des - CS.steeringAngleDeg

    pid_log.steeringAngleDesiredDeg = angle_steers_des
    pid_log.angleError = error

    # Add curvature gain specific monitoring
    current_time = CS.aEgo  # Use a consistent time source, or implement proper time tracking
    try:
      # Get current curvature gain factor if available
      curvature_gain_factor = getattr(self.pid, 'oscillation_gain_factor', 1.0)
      oscillation_detected = getattr(self.pid, 'oscillation_detected', False)
      oscillation_damping_active = getattr(self.pid, 'oscillation_damping_active', False)
      oscillation_detection_count = getattr(self.pid, 'oscillation_detection_count', 0)
      oscillation_recovery_count = getattr(self.pid, 'oscillation_recovery_count', 0)

      # Add monitoring data to PID log
      pid_log.curvatureGainFactor = curvature_gain_factor
      pid_log.oscillationDetected = oscillation_detected
      pid_log.oscillationDampingActive = oscillation_damping_active
      pid_log.oscillationDetectionCount = oscillation_detection_count
      pid_log.oscillationRecoveryCount = oscillation_recovery_count
      # Add safe mode indicators to PID log
      try:
        pid_log.safeModeActive = getattr(self.pid, 'safe_mode_active', False)
        pid_log.safeModeTriggerCount = getattr(self.pid, 'safe_mode_trigger_count', 0)
        pid_log.safetyLimitTriggerCount = getattr(self.pid, 'safety_limit_trigger_count', 0)
      except Exception:
        # If getting safe mode data fails, continue with normal operation
        pid_log.safeModeActive = False
        pid_log.safeModeTriggerCount = 0
        pid_log.safetyLimitTriggerCount = 0

    except Exception:
      # If getting monitoring data fails, continue with normal operation
      pid_log.curvatureGainFactor = 1.0
      pid_log.oscillationDetected = False
      pid_log.oscillationDampingActive = False
      pid_log.oscillationDetectionCount = 0
      pid_log.oscillationRecoveryCount = 0

    if not active:
      output_torque = 0.0
      pid_log.active = False

    else:
      # offset does not contribute to resistive torque
      ff = self.ff_factor * self.get_steer_feedforward(angle_steers_des_no_offset, CS.vEgo)
      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5

      output_torque = self.pid.update(error,
                                feedforward=ff,
                                speed=CS.vEgo,
                                curvature=abs(desired_curvature),
                                freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(output_torque)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # Add periodic monitoring logging for field validation
    import time
    current_time = time.time()
    if not hasattr(self, 'last_monitoring_log_time'):
        self.last_monitoring_log_time = 0

    # Log monitoring data every 30 seconds
    if current_time - self.last_monitoring_log_time > 30.0:
      try:
        cloudlog.event("Curvature Gain Monitoring",
                      curvature_gain_factor=curvature_gain_factor,
                      oscillation_detected=oscillation_detected,
                      oscillation_damping_active=oscillation_damping_active,
                      oscillation_detection_count=oscillation_detection_count,
                      oscillation_recovery_count=oscillation_recovery_count,
                      safe_mode_active=getattr(self.pid, 'safe_mode_active', False),
                      safe_mode_trigger_count=getattr(self.pid, 'safe_mode_trigger_count', 0),
                      safety_limit_trigger_count=getattr(self.pid, 'safety_limit_trigger_count', 0),
                      desired_curvature=desired_curvature,
                      vEgo=CS.vEgo,
                      error=error,
                      output=output_torque)
        # If safety limits are being hit frequently, log a warning
        if getattr(self.pid, 'safety_limit_trigger_count', 0) > 50:
          cloudlog.warning(f"Safety limits triggered frequently: {getattr(self.pid, 'safety_limit_trigger_count', 0)} times",
                          curvature_gain_factor=curvature_gain_factor,
                          desired_curvature=desired_curvature,
                          vEgo=CS.vEgo,
                          error=error,
                          output=output_torque)
        # If safe mode is active, log an info message
        if getattr(self.pid, 'safe_mode_active', False):
          cloudlog.info(f"Curvature gain safe mode activated: {getattr(self.pid, 'safe_mode_trigger_count', 0)} triggers",
                       desired_curvature=desired_curvature,
                       vEgo=CS.vEgo,
                       curvature_gain_interp=getattr(CP_SP, 'curvatureGainInterp', None))
        self.last_monitoring_log_time = current_time
      except Exception as e:
        cloudlog.error(f"Error in curvature gain monitoring log: {e}")

    return output_torque, angle_steers_des, pid_log
