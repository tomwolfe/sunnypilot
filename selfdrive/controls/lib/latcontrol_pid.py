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
                             pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.ff_factor = CP.lateralTuning.pid.kf
    self.get_steer_feedforward = CI.get_steer_feedforward_function()
    self.curvature_filter = FirstOrderFilter(0., 0.1, dt) # Initialize FirstOrderFilter
    # Add monitoring variables for field validation
    self.monitoring_initialized = False
    self.last_monitoring_log_time = 0

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    # Apply low-pass filter to desired_curvature
    desired_curvature = self.curvature_filter.update(desired_curvature)
    cloudlog.debug(f"Desired Curvature: {desired_curvature:.4f}") # Log desired curvature

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
                      desired_curvature=desired_curvature,
                      vEgo=CS.vEgo,
                      error=error,
                      output=output_torque)
        self.last_monitoring_log_time = current_time
      except Exception as e:
        cloudlog.error(f"Error in curvature gain monitoring log: {e}")

    return output_torque, angle_steers_des, pid_log
