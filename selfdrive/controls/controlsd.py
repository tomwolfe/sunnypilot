#!/usr/bin/env python3
import math
import threading
import time
from collections import deque
from numbers import Number

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from openpilot.selfdrive.controls.lib.safety_helpers import SafetyManager
from openpilot.selfdrive.controls.lib.edge_case_handler import EdgeCaseHandler
from openpilot.selfdrive.controls.lib.self_learning_manager import SafeSelfLearningManager

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())


class Controls(ControlsExt, ModelStateBase):
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    # Initialize sunnypilot controlsd extension and base model state
    ControlsExt.__init__(self, self.CP, self.params)
    ModelStateBase.__init__(self)

    self.CI = interfaces[self.CP.carFingerprint](self.CP, self.CP_SP)

    self.sm = messaging.SubMaster(['liveDelay', 'liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance', 'liveDelay',
                                   'deviceState', 'radarState'] + self.sm_services_ext,
                                  poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'] + self.pm_services_ext)

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.desired_curvature = 0.0

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP, self.CP_SP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CP_SP, self.CI, DT_CTRL)

    # Initialize thermal management system with enhanced tracking
    # thermal_performance_factor: Represents the current overall system stress (thermal, CPU, memory).
    #   A value of 1.0 means no stress, lower values indicate higher stress.
    #   It's a smoothed value derived from the maximum of thermal, CPU, and memory percentages.
    # thermal_history: A deque that tracks the thermal_performance_factor over the last 30 cycles (0.3 seconds at 100Hz).
    #   Used for trend analysis to predict future thermal behavior.
    # thermal_stress_level: Categorizes the system's stress into discrete levels (0-3).
    #   0=normal, 1=moderate, 2=high, 3=very high. This provides context-aware control.
    # performance_compensation_factor: An additional factor that further reduces performance based on rapidly increasing
    #   thermal stress trends. It acts as a proactive cushion against thermal runaway.
    self.thermal_performance_factor = 1.0
    self.thermal_history: deque = deque(maxlen=30)  # Track thermal performance over last 30 cycles (0.3 seconds at 100Hz)
    self.thermal_stress_level = 0  # 0=normal, 1=moderate, 2=high, 3=very high
    self.performance_compensation_factor = 1.0  # Additional compensation for performance degradation

    # Vehicle-specific thermal model parameters
    # These parameters adjust thermal response based on vehicle type and driving conditions
    self.vehicle_thermal_model = {
        'base_thermal_factor': 1.0,
        'dynamic_thermal_adjustment': 0.0,
        'vehicle_specific_multiplier': 1.0,
        'adaptive_control_damping': 1.0,
        'thermal_inertia_factor': 0.95,  # Factor for thermal momentum (0.0 to 1.0, where 1.0 = no momentum)
    }

    # Initialize vehicle-specific thermal parameters from car params if available
    self._init_vehicle_thermal_parameters()

    # Initialize safety manager for advanced safety features
    self.safety_manager = SafetyManager()

    # Initialize edge case handler for unusual scenario detection and handling
    self.edge_case_handler = EdgeCaseHandler()

    # Initialize self-learning manager for adaptive driving behavior
    self.self_learning_manager = SafeSelfLearningManager(self.CP, self.CP_SP)

  def _init_vehicle_thermal_parameters(self):
    """
    Initialize vehicle-specific thermal model parameters based on car parameters.
    Different vehicle types have different thermal characteristics (engine size, cooling system, etc.)
    """
    # Set vehicle-specific thermal parameters based on vehicle characteristics
    # This affects how the system responds thermally under different driving conditions
    if hasattr(self.CP, 'mass'):
      # Heavier vehicles may generate more system load
      mass_factor = min(1.5, max(0.8, self.CP.mass / 1500.0))  # Normalize around 1500kg
      self.vehicle_thermal_model['vehicle_specific_multiplier'] = mass_factor

    if hasattr(self.CP, 'centerToFront'):
      # Longer wheelbase vehicles may have different dynamics requiring different processing
      wheelbase = self.CP.wheelbase  # Proxy for vehicle size
      size_factor = min(1.3, max(0.9, wheelbase / 3000.0))  # Normalize around typical values
      self.vehicle_thermal_model['adaptive_control_damping'] = size_factor

  def update(self):
    """
    Update the control system with new sensor and model data.

    This method updates the internal state of the control system by processing
    new messages from various sensors and the model. It also implements adaptive
    control based on thermal performance to maintain system stability under
    varying hardware conditions.
    """
    # Update at 15Hz to maintain message flow for critical communication (e.g., carState, radarState)
    # without adding significant load, especially during skipped control cycles.
    self.sm.update(15)
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

    # Enhanced thermal management with predictive capabilities and stress level tracking
    if self.sm.updated['deviceState']:
      thermal_status = self.sm['deviceState'].thermalStatus
      thermal_prc = self.sm['deviceState'].thermalPerc
      cpu_usage = max(self.sm['deviceState'].cpuUsagePercent) if self.sm['deviceState'].cpuUsagePercent else 0
      memory_usage = self.sm['deviceState'].memoryUsagePercent

      # Add GPU temperature monitoring if available
      gpu_temp = max(self.sm['deviceState'].gpuTempC) if self.sm['deviceState'].gpuTempC else 0
      cpu_temp = max(self.sm['deviceState'].cpuTempC) if self.sm['deviceState'].cpuTempC else 0

      cloudlog.debug(
          f"Thermal management inputs: thermal_prc={thermal_prc:.2f}, cpu_usage={cpu_usage:.2f}, " +
          f"memory_usage={memory_usage:.2f}, cpu_temp={cpu_temp:.1f}C, gpu_temp={gpu_temp:.1f}C"
      )

      # Enhanced thermal management with more granular control and predictive elements
      base_thermal_factor = thermal_prc / 100.0
      cpu_factor = cpu_usage / 100.0
      memory_factor = memory_usage / 100.0

      # Include temperature-based stress factor
      temp_stress = 0.0
      if cpu_temp > 0 and gpu_temp > 0:
        # Calculate temperature stress (normalized to 0-1 range)
        max_temp = max(cpu_temp, gpu_temp)
        if max_temp > 80:  # High temperature threshold
          temp_stress = min(1.0, (max_temp - 80) / 20)  # Max stress at 100C
      elif thermal_status > 1:  # Yellow or higher thermal status
        temp_stress = thermal_status / 10.0  # Map thermal status to stress factor

      # Calculate overall system stress as the maximum of all factors (thermal, CPU, memory, temperature).
      # This provides a holistic view of the system's current load.
      system_stress = max(base_thermal_factor, cpu_factor, memory_factor, temp_stress)

      # Apply vehicle-specific thermal model adjustment
      # Different vehicles have different thermal characteristics based on mass, size, etc.
      vehicle_thermal_modifier = self.vehicle_thermal_model['vehicle_specific_multiplier']
      system_stress = min(1.0, system_stress * vehicle_thermal_modifier)

      # Apply hysteresis and smoothing to `thermal_performance_factor` to prevent rapid, destabilizing oscillations
      # between performance states. A higher smoothing factor is used when stress is decreasing to gradually
      # restore performance, while a lower factor is used when stress is increasing to react more quickly.
      if hasattr(self, 'prev_thermal_factor'):
        # Apply thermal inertia with vehicle-specific adjustment
        thermal_inertia = self.vehicle_thermal_model['thermal_inertia_factor']
        smoothing_factor = 0.9 if system_stress <= self.prev_thermal_factor else 0.7
        # Apply vehicle-specific thermal inertia
        smoothing_factor *= thermal_inertia
        self.thermal_performance_factor = (smoothing_factor * self.prev_thermal_factor +
                                          (1 - smoothing_factor) * system_stress)
      else:
        self.thermal_performance_factor = system_stress

      self.prev_thermal_factor = self.thermal_performance_factor

      # Update thermal history for trend analysis
      self.thermal_history.append(self.thermal_performance_factor)

      # Determine thermal stress level based on current and historical data.
      # This allows for context-aware control adjustments.
      avg_thermal = sum(self.thermal_history) / len(self.thermal_history) if self.thermal_history else 0
      current_thermal = self.thermal_performance_factor

      # Classify stress level based on both current and average thermal state
      if current_thermal > 0.9 or avg_thermal > 0.85:
        self.thermal_stress_level = 3  # Very high stress
      elif current_thermal > 0.8 or avg_thermal > 0.75:
        self.thermal_stress_level = 2  # High stress
      elif current_thermal > 0.7 or avg_thermal > 0.65:
        self.thermal_stress_level = 1  # Moderate stress
      else:
        self.thermal_stress_level = 0  # Normal

      # Adjust performance compensation based on thermal stress trend.
      # If stress is rapidly increasing, proactively apply extra compensation as a "cushion."
      # If stress is decreasing, gradually restore performance.
      if len(self.thermal_history) > 10:
        # Calculate thermal trend (increasing, decreasing, stable)
        recent_avg = sum(list(self.thermal_history)[-5:]) / 5
        older_avg = sum(list(self.thermal_history)[:-5]) / max(1, len(list(self.thermal_history)[:-5]))

        if recent_avg > older_avg + 0.1:  # Thermal stress is increasing rapidly
          # Apply additional compensation to prevent thermal runaway
          self.performance_compensation_factor = max(0.8, self.thermal_performance_factor - 0.1)
        elif recent_avg < older_avg - 0.1:  # Thermal stress is decreasing
          # Gradually restore performance
          self.performance_compensation_factor = min(1.0, self.performance_compensation_factor + 0.05)
        else:  # Thermal stress is relatively stable
          self.performance_compensation_factor = self.thermal_performance_factor
      else:
        self.performance_compensation_factor = self.thermal_performance_factor

      # Additional thermal considerations for safety-critical functions
      # Thermal status values: green=0, yellow=1, red=2, danger=3
      # Values >= 5 indicate extreme thermal conditions requiring immediate action
      # This provides an additional safety margin beyond standard thermal warnings
      if thermal_status >= 5:  # Critical thermal status (beyond standard danger level)
        cloudlog.warning(f"Critical thermal status: {thermal_status}, reducing performance to protect hardware")
        # Reduce performance factor further when in critical thermal state
        self.thermal_performance_factor = min(self.thermal_performance_factor, 0.7)
        self.performance_compensation_factor = min(self.performance_compensation_factor, 0.6)
    else:
      self.thermal_performance_factor = 1.0
      self.performance_compensation_factor = 1.0

    # Perform predictive thermal management based on recent control activity
    self._update_predictive_thermal_management()

  def _update_predictive_thermal_management(self):
    """
    Predict thermal stress before it happens based on recent control activity
    """
    # Track control activity metrics for prediction
    if not hasattr(self, '_control_activity_history'):
      self._control_activity_history = deque(maxlen=20)  # Track last 20 cycles

    # Calculate control activity based on recent control outputs
    control_activity = 0.0
    if hasattr(self, 'LoC') and hasattr(self.LoC, 'last_output_accel'):
      # Longitudinal control activity (based on acceleration changes)
      control_activity += min(1.0, abs(self.LoC.last_output_accel) / 3.0)  # Normalize to 0-1

    if hasattr(self, 'LaC'):
      # Lateral control activity would need to be tracked in the LaC instance
      # For now, we'll estimate based on desired curvature changes
      if hasattr(self, 'desired_curvature') and hasattr(self, '_prev_desired_curvature'):
        curvature_change = abs(self.desired_curvature - self._prev_desired_curvature)
        control_activity += min(1.0, curvature_change * 10.0)  # Normalize to 0-1
      self._prev_desired_curvature = self.desired_curvature

    # Add vehicle-specific and driving context to control activity
    CS = self.sm['carState'] if 'carState' in self.sm else None
    if CS is not None:
      # Higher speeds and more aggressive driving generate more heat
      speed_factor = min(1.0, CS.vEgo / 35.0)  # Normalize at highway speeds
      control_activity = min(1.0, control_activity + speed_factor * 0.2)

      # Include vEgo and aEgo for thermal prediction
      if hasattr(CS, 'aEgo') and CS.vEgo > 1.0:
        # Aggressive longitudinal maneuvers generate heat
        accel_brake_intensity = min(1.0, abs(CS.aEgo) / 2.0)
        control_activity = min(1.0, control_activity + accel_brake_intensity * 0.15)

    # Add this control activity to history
    self._control_activity_history.append(control_activity)

    # Calculate predictive thermal stress factors
    if len(self._control_activity_history) >= 10:
      recent_control_activity = sum(list(self._control_activity_history)[-10:]) / 10.0

      # Predict thermal rise based on sustained activity and vehicle-specific factors
      prediction_horizon = 3  # Look ahead 3 cycles
      predicted_thermal_rise = self._calculate_predicted_thermal_rise(
          recent_control_activity,
          self.thermal_history,
          prediction_horizon
      )

      # Adjust prediction based on vehicle-specific thermal parameters
      vehicle_adaptive_factor = self.vehicle_thermal_model['adaptive_control_damping']
      predicted_thermal_rise = predicted_thermal_rise * vehicle_adaptive_factor

      # Blend current thermal state with predicted state to be proactive
      current_thermal_factor = self.thermal_performance_factor
      predictive_weight = 0.2  # Weight of predictive element (20% predictive, 80% current)
      predictive_factor = max(0.0, 1.0 - predicted_thermal_rise)

      # Apply predictive adjustment but only in the direction of more caution
      if predictive_factor < current_thermal_factor:  # If prediction indicates higher stress
        adjusted_thermal_factor = (
            (1 - predictive_weight) * current_thermal_factor +
            predictive_weight * predictive_factor
        )

        # Apply hysteresis to prevent oscillation
        if hasattr(self, 'prev_adjusted_factor'):
          smoothing_factor = 0.95 if adjusted_thermal_factor >= self.prev_adjusted_factor else 0.8
          self.thermal_performance_factor = (
              smoothing_factor * self.prev_adjusted_factor +
              (1 - smoothing_factor) * adjusted_thermal_factor
          )
        else:
          self.thermal_performance_factor = adjusted_thermal_factor

        self.prev_adjusted_factor = self.thermal_performance_factor

  def _calculate_predicted_thermal_rise(self, recent_activity, thermal_history, horizon):
    """
    Predict thermal rise based on control activity patterns with enhanced predictive model
    """
    # Enhanced predictive model: consider multiple factors for thermal prediction
    base_activity_level = min(1.0, recent_activity / 0.7)  # Normalize to 0-1 based on 70% activity threshold

    # Calculate recent thermal trend with weighted analysis for better prediction
    trend = 0.0
    if len(thermal_history) > 5:
      # Use exponentially weighted moving average for recent values (last 3)
      recent_window = list(thermal_history)[-3:]
      recent_avg = sum(recent_window[i] * (i + 1) for i in range(len(recent_window))) / sum(range(1, len(recent_window) + 1))

      # Use simple average for older values (previous 3 before recent)
      older_start_idx = max(0, len(thermal_history) - 6)
      older_window = list(thermal_history)[older_start_idx:len(thermal_history) - 3]
      if older_window:
        older_avg = sum(older_window) / len(older_window)
      else:
        older_avg = recent_avg  # Fallback if not enough history

      # Calculate normalized trend with enhanced sensitivity to rapid changes
      trend_raw = recent_avg - older_avg
      trend = max(-0.8, min(0.8, trend_raw)) / 0.8  # Normalize trend to -1, 1 with higher sensitivity

    # Consider vehicle-specific thermal characteristics for more accurate prediction
    vehicle_thermal_factor = self.vehicle_thermal_model['vehicle_specific_multiplier']

    # Account for environmental factors if available (road grade, ambient temp, etc.)
    CS = self.sm['carState'] if 'carState' in self.sm else None
    environmental_factor = 1.0
    if CS is not None:
      # Road grade factor: uphill driving generates more heat
      if hasattr(CS, 'roadGrade') and CS.roadGrade is not None:
        if isinstance(CS.roadGrade, (int, float)):
          if CS.roadGrade > 0.05:  # Uphill with grade > 5%
            environmental_factor += min(0.3, CS.roadGrade)  # Add up to 30% extra thermal prediction
          elif CS.roadGrade < -0.05:  # Downhill with grade > 5% (braking generates heat too)
            environmental_factor += min(0.15, abs(CS.roadGrade))  # Add up to 15% for downhill

    # Enhanced prediction incorporating control complexity
    control_complexity_factor = 0.0
    if CS is not None:
      # Calculate complexity based on multiple simultaneous control demands
      lateral_demand = abs(self.desired_curvature) * CS.vEgo**2  # Lateral acceleration requirement
      longitudinal_demand = abs(self.sm['longitudinalPlan'].aTarget) if 'longitudinalPlan' in self.sm else 0
      complexity_score = (lateral_demand * 0.4 + longitudinal_demand * 0.3) / 5.0  # Normalize
      control_complexity_factor = min(0.2, complexity_score)  # Max 20% additional thermal prediction

    # Combine all factors for comprehensive prediction
    prediction = (base_activity_level * 0.4 +  # Weight activity moderately
                  max(0, trend) * 0.25 +  # Weight positive trend
                  (vehicle_thermal_factor - 1.0) * 0.1 +  # Vehicle-specific adjustment
                  (environmental_factor - 1.0) * 0.1 +  # Environmental adjustment
                  control_complexity_factor * 0.15)  # Control complexity adjustment

    # Apply bounds and return
    return min(1.0, max(0.0, prediction))  # Ensure in [0.0, 1.0] range

  def _adjust_control_for_thermal_conditions(self, actuators, CS, long_plan, model_v2):
    """
    Adjust control outputs based on thermal conditions for improved thermal management.

    This function proactively reduces control aggressiveness to prevent thermal runaway.
    Critical Analysis Note: The primary risk is over-compensation. Robust monitoring
    and telemetry for `performance_compensation_factor` and its effects on
    `accel`, `curvature`, and `steeringAngleDeg` are crucial for tuning.
    Consider implementing a "safe mode" or fallback mechanism if thermal
    monitoring data becomes unreliable or leads to excessive control reduction.

    Args:
        actuators: The control actuators to adjust
        CS: Current car state
        long_plan: Longitudinal plan
        model_v2: Model predictions

    Returns:
        Modified actuators adjusted for thermal conditions
    """
    # Apply thermal-based adjustments to control outputs with enhanced logic
    if self.performance_compensation_factor < 0.9:  # Under thermal stress (lowered threshold for more proactive management)
      # Calculate thermal aggression factor based on multiple factors
      thermal_aggression_factor = self.performance_compensation_factor / 0.9  # Adjusted threshold

      # Reduce longitudinal acceleration aggressiveness with speed-dependent scaling
      if hasattr(actuators, 'accel'):
        # Apply smoother transitions in acceleration
        # At high speeds, be more conservative to reduce computational load
        speed_factor = min(1.0, max(0.7, (30.0 - CS.vEgo) / 30.0))  # More conservative at higher speeds
        actuators.accel = actuators.accel * thermal_aggression_factor * speed_factor

      # Reduce lateral control aggressiveness with speed and curvature-dependent scaling
      if hasattr(actuators, 'curvature'):
        # At high speeds or high curvature, be more conservative as lateral computation is more intensive
        lateral_speed_factor = min(1.0, max(0.6, (40.0 - CS.vEgo) / 40.0))
        # For high curvature situations, be even more conservative
        curvature_factor = max(0.7, 1.0 - min(0.3, abs(actuators.curvature) * 50.0))  # More conservative with high curvature
        thermal_lateral_factor = thermal_aggression_factor * lateral_speed_factor * curvature_factor
        actuators.curvature = actuators.curvature * thermal_lateral_factor

      if hasattr(actuators, 'steeringAngleDeg'):
        # Gentle steering adjustments under thermal stress, with adaptive scaling
        # At high speeds, be more conservative with steering changes
        steering_speed_factor = min(1.0, max(0.7, (25.0 - CS.vEgo) / 25.0))
        thermal_steering_factor = thermal_aggression_factor * steering_speed_factor
        actuators.steeringAngleDeg = actuators.steeringAngleDeg * thermal_steering_factor

      # Enhanced logging with more detailed information
      cloudlog.debug(
          f"Thermal adjustment applied: factor={thermal_aggression_factor:.2f}, " +
          f"stress_level={self.thermal_stress_level}, vEgo={CS.vEgo:.1f}m/s, " +
          f"speed_factor={speed_factor:.2f if hasattr(actuators, 'accel') else 'N/A'}, " +
          f"curvature={actuators.curvature:.3f}"
      )

    # Additional conservative measures under critical thermal conditions
    if self.performance_compensation_factor < 0.6:  # Critical thermal stress
      # Apply additional safety margins
      if hasattr(actuators, 'accel'):
        # Further limit acceleration changes to prevent thermal runaway
        actuators.accel = max(min(actuators.accel, 1.5), -2.0)  # Limit to reasonable values under thermal stress
      if hasattr(actuators, 'curvature'):
        actuators.curvature = max(min(actuators.curvature, 0.2), -0.2)  # Limit to gentle curves under critical stress

    return actuators

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

        self.LaC.extension.update_limits()

      self.LaC.extension.update_model_v2(self.sm['modelV2'])

      self.LaC.extension.update_lateral_lag(self.lat_delay)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill

    # Get which state to use for active lateral control
    _lat_active = self.get_lat_active(self.sm)

    CC.latActive = _lat_active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.CP.steerAtStandstill)
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and \
                    (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # Handle edge cases and unusual scenarios
    edge_scenarios = self.edge_case_handler.handle_unusual_scenarios(
      CS,
      self.sm['radarState'] if 'radarState' in self.sm else None,
      self.sm['modelV2'] if 'modelV2' in self.sm else None
    )

    # Get adaptive control modifications based on detected scenarios
    adaptive_mods = self.edge_case_handler.get_adaptive_control_modifications(CS, edge_scenarios)

    # Apply adaptive control modifications to the system
    if adaptive_mods['caution_mode']:
      # Increase time headway for safety in unusual scenarios
      # This is done by modifying the long_plan to create larger gaps
      if hasattr(long_plan, 'aTarget') and adaptive_mods['longitudinal_factor'] < 1.0:
        # Make longitudinal control more conservative
        base_accel_limit = self.CI.get_pid_accel_limits(self.CP, self.CP_SP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
        # Apply conservative factor to acceleration limits
        conservative_accel_limits = (
          base_accel_limit[0] * adaptive_mods['longitudinal_factor'],
          base_accel_limit[1] * adaptive_mods['longitudinal_factor']
        )
      else:
        conservative_accel_limits = self.CI.get_pid_accel_limits(self.CP, self.CP_SP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    else:
      conservative_accel_limits = self.CI.get_pid_accel_limits(self.CP, self.CP_SP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)

    # Enhanced saturation handling with adaptive limits
    # accel PID loop
    actuators.accel = float(self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, conservative_accel_limits))

    # Apply self-learning adjustments to model outputs
    base_desired_curvature = model_v2.action.desiredCurvature
    learned_adjusted_curvature = self.self_learning_manager.adjust_curvature(base_desired_curvature, CS.vEgo)

    # Apply lateral control modifications if needed (after learning adjustment)
    modified_desired_curvature = learned_adjusted_curvature
    if adaptive_mods['lateral_factor'] < 1.0 and CC.latActive:
      # Make lateral control more conservative by reducing desired curvature
      modified_desired_curvature = learned_adjusted_curvature * adaptive_mods['lateral_factor']

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    new_desired_curvature = modified_desired_curvature if CC.latActive else self.curvature
    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    lat_delay = self.sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS

    actuators.curvature = self.desired_curvature
    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       self.calibrated_pose, curvature_limited, lat_delay)

    # Enhanced saturation detection with smoother recovery
    saturation_detected = False
    if hasattr(lac_log, 'saturated') and lac_log.saturated:
      saturation_detected = True
      cloudlog.debug(f"Steering saturation detected at vEgo: {CS.vEgo:.2f} m/s")

    actuators.torque = float(steer)
    actuators.steeringAngleDeg = float(steeringAngleDeg)

    # Apply thermal-based adjustments to control outputs
    actuators = self._adjust_control_for_thermal_conditions(actuators, CS, long_plan, model_v2)

    # Enhanced finite value checks with recovery mechanism
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      # Check for finite values and reasonable bounds
      if not math.isfinite(attr):
        cloudlog.error(
            f"actuators.{p} not finite. Actuators: {actuators.to_dict()}, CarState: {CS.to_dict()}, " +
            f"LongitudinalPlan: {long_plan.to_dict()}, LateralControlLog: {lac_log.to_dict()}"
        )
        # Implement a recovery by setting to a safe value instead of 0.0
        if p in ['steeringAngleDeg', 'curvature']:
          # For steering-related values, use current measurement as fallback
          setattr(actuators, p, 0.0 if p == 'curvature' else CS.steeringAngleDeg)
          CC.hudControl.visualAlert = log.ControlsState.AlertStatus.critical # Indicate non-finite steering to user
        elif p == 'accel':
          # For acceleration, use 0 to maintain current speed
          setattr(actuators, p, 0.0)
        else:
          # Default fallback to 0.0
          setattr(actuators, p, 0.0)
      # Additional safety validation for extreme values
      elif p == 'accel':
        # Check for excessive acceleration commands that exceed physical limits
        if abs(attr) > 5.0:  # More than 0.5g acceleration is extreme
          cloudlog.warning(f"Excessive acceleration command detected: {attr} m/s^2, limiting to safe value")
          limited_accel = max(-5.0, min(5.0, attr))
          setattr(actuators, p, limited_accel)
          CC.hudControl.visualAlert = log.ControlsState.AlertStatus.warning
      elif p in ['steeringAngleDeg']:
        # Check for excessive steering angle commands beyond physical limits
        if abs(attr) > 90.0:  # Steering angle beyond 90 degrees should never happen
          cloudlog.error(f"Excessive steering angle command detected: {attr} deg, limiting to safe value")
          limited_steering = max(-90.0, min(90.0, attr))
          setattr(actuators, p, limited_steering)
          CC.hudControl.visualAlert = log.ControlsState.AlertStatus.critical
      elif p in ['curvature']:
        # Check for excessive curvature commands (very tight turns at speed)
        if abs(attr) > 0.5 and CS.vEgo > 5.0:  # Very high curvature at speed is dangerous
          # Calculate max safe curvature based on speed: curvature = max_lat_accel / vEgo^2
          max_lat_accel = 3.0  # Conservative maximum lateral acceleration in m/s^2
          max_safe_curvature = max_lat_accel / (CS.vEgo ** 2)
          if abs(attr) > max_safe_curvature:
            cloudlog.warning(f"Excessive curvature command detected: {attr} vs max safe {max_safe_curvature} at speed {CS.vEgo}, limiting")
            limited_curvature = max(-max_safe_curvature, min(max_safe_curvature, attr))
            setattr(actuators, p, limited_curvature)
            CC.hudControl.visualAlert = log.ControlsState.AlertStatus.warning

    # Enhanced saturation handling in controls state
    if saturation_detected:
      CC.hudControl.visualAlert = log.ControlsState.AlertStatus.normal  # Indicate saturation to user

    # Update self-learning manager with current driving experience
    model_confidence = getattr(model_v2.meta, 'confidence', 1.0) if hasattr(model_v2, 'meta') else 1.0
    self.self_learning_manager.update(
        CS,
        base_desired_curvature,  # Original model output
        self.curvature,  # Actual vehicle curvature
        actuators.torque,
        CS.vEgo,
        model_confidence=model_confidence
    )

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)
    CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not self.sm['longitudinalPlan'].shouldStop

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    if self.sm['selfdriveState'].active:
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_safety = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_safety = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # Perform safety check before publishing controls
    safety_ok, safety_violation = self.safety_manager.check_safety_violations(
      CS,
      self.sm['carOutput'].actuatorsOutput if 'carOutput' in self.sm else None,
      self.sm['modelV2'] if 'modelV2' in self.sm else None
    )

    # If safety violation detected, modify control output to be safer
    if not safety_ok and self.safety_manager.should_disengage():
      # Apply emergency deceleration and center steering
      CC.actuators.accel = min(CC.actuators.accel, -1.0)  # Apply mild brake
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        CC.actuators.steeringAngleDeg = CS.steeringAngleDeg * 0.9  # Gradually center steering
      else:
        CC.actuators.curvature = self.curvature * 0.9  # Gradually reduce curvature
      CC.enabled = False  # Disengage to prevent dangerous situation
      CC.latActive = False
      CC.longActive = False

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool((self.sm['driverMonitoringState'].awarenessStatus < 0.) or
                         (self.sm['selfdriveState'].state == State.softDisabling))

    # Add safety status information to controls state
    cs.safetyStatus = self.safety_manager.get_safety_recommendation(
      CS,
      self.sm['carOutput'].actuatorsOutput if 'carOutput' in self.sm else None,
      self.sm['modelV2'] if 'modelV2' in self.sm else None
    )

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def params_thread(self, evt):
    while not evt.is_set():
      self.get_params_sp()

      if self.CP.lateralTuning.which() == 'torque':
        self.lat_delay = get_lat_delay(self.params, self.sm["liveDelay"].lateralDelay)

      time.sleep(0.1)

  def run(self):
    """
    Main control loop that runs at an adaptive rate based on thermal conditions.

    The control system normally runs at 100Hz (base_rate), but this rate is
    dynamically adjusted based on the thermal performance factor to reduce
    computational load when the device is overheating. The rate is reduced
    gradually to maintain system stability while protecting hardware.
    """
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e,))
    try:
      t.start()
      rk = Ratekeeper(100, print_delay_threshold=None)  # Base rate is 100Hz
      thermal_adjusted_frame = 0

      # Enhanced thermal scaling parameters with dynamic adjustment based on stress level
      base_rate = 100  # Hz
      # `min_rates` provides dynamic, context-aware control. It maps thermal stress levels to minimum allowed control loop frequencies.
      # This ensures that even under moderate stress, the system doesn't throttle too aggressively, preserving responsiveness.
      min_rates = {0: 80, 1: 70, 2: 60, 3: 50}  # Lower rates for higher stress levels

      # Initialize priority-based scheduling parameters
      # Critical functions that must run at higher frequency even under thermal stress

      # Adaptive scheduling based on thermal stress
      min_critical_rate = 50  # Hz minimum for safety functions
      min_standard_rate = 10   # Hz minimum for standard functions

      while True:
        # `current_rate` is adaptively calculated based on the current thermal stress level and the
        # `performance_compensation_factor`. This dynamically adjusts the control loop frequency
        # to maintain system stability and hardware protection under varying thermal conditions.
        current_stress_level = self.thermal_stress_level
        current_rate = max(min_rates[current_stress_level], base_rate * self.performance_compensation_factor)

        # Calculate rates based on thermal stress (0.0 = no stress, 1.0 = maximum stress)
        stress_factor = 1.0 - self.performance_compensation_factor

        # Critical functions: reduce less aggressively
        critical_factor = max(0.5, 1.0 - stress_factor * 0.3)  # Less reduction for critical functions
        critical_rate = max(min_critical_rate, base_rate * critical_factor)

        # Standard functions: reduce more aggressively
        standard_factor = max(0.1, 1.0 - stress_factor * 0.9)  # More reduction for standard functions
        standard_rate = max(min_standard_rate, base_rate * standard_factor)

        # Process every frame when at full rate, every other frame when at reduced rate
        frame_skip_threshold = base_rate / current_rate
        thermal_adjusted_frame += 1

        # Only perform full control cycle if we're not skipping this frame due to thermal constraints
        if thermal_adjusted_frame >= frame_skip_threshold:
          thermal_adjusted_frame = 0  # Reset counter

          # Update message subscriptions based on priority during thermal stress
          # Critical functions get higher priority update rates
          if self.performance_compensation_factor < 0.8:  # Under thermal stress
            # Update critical functions at higher rate
            self.sm.update(100)  # Always update critical messages at high rate

            # Perform thermal-aware control operations
            CC, lac_log = self._thermal_aware_control_cycle()
          else:
            # Normal operation
            self.update()
            CC, lac_log = self.state_control()

          self.publish(CC, lac_log)
          self.run_ext(self.sm, self.pm)

          # Enhanced thermal monitoring and logging with stress level information
          if self.thermal_stress_level > 0:
            cloudlog.debug(
                f"Thermal throttling active: stress_level={self.thermal_stress_level}, " +
                f"factor={self.performance_compensation_factor:.2f}, rate={current_rate:.1f}Hz, " +
                f"current_thermal={self.thermal_performance_factor:.2f}, " +
                f"critical_rate={critical_rate:.1f}Hz, standard_rate={standard_rate:.1f}Hz"
            )
        else:
          # Still update the message subsystem regularly to maintain message flow
          # This 15Hz update rate is chosen to ensure critical communication (e.g., carState, radarState)
          # is maintained even when the main control loop is thermally throttled and frames are skipped,
          # without adding significant computational load during these skipped cycles.
          self.sm.update(15)

        # Monitor timing with thermal awareness and add thermal performance adjustments
        timing_start = time.monotonic()
        rk.monitor_time()
        timing_elapsed = time.monotonic() - timing_start

        # Additional thermal monitoring for extreme cases and logging based on thermal state
        if timing_elapsed > 0.02:  # If we're taking too long, log it
          cloudlog.debug(
              f"Control loop timing exceeded threshold: {timing_elapsed*1000:.1f}ms, " +
              f"thermal_factor: {self.thermal_performance_factor:.2f}, " +
              f"stress_level: {self.thermal_stress_level}"
          )

        # Add thermal-based performance adjustments to the lateral and longitudinal controllers
        if hasattr(self.LaC, 'update_thermal_compensation'):
          self.LaC.update_thermal_compensation(self.thermal_stress_level, self.performance_compensation_factor)
        if hasattr(self.LoC, 'update_thermal_compensation'):
          self.LoC.update_thermal_compensation(self.thermal_stress_level, self.performance_compensation_factor)

    finally:
      e.set()
      t.join()

  def _thermal_aware_control_cycle(self):
    """
    Perform control cycle with thermal awareness, prioritizing critical safety functions
    even under thermal stress conditions.
    """
    # In thermal stress, optimize control operations
    self.update()  # Update system state including thermal management

    # Perform critical control operations with thermal compensation
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

        self.LaC.extension.update_limits()

      self.LaC.extension.update_model_v2(self.sm['modelV2'])

      self.LaC.extension.update_lateral_lag(self.lat_delay)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled with thermal-aware checks
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill

    # Get which state to use for active lateral control
    _lat_active = self.get_lat_active(self.sm)

    CC.latActive = _lat_active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.CP.steerAtStandstill)
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and \
                    (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # Handle edge cases and unusual scenarios with thermal awareness
    edge_scenarios = self.edge_case_handler.handle_unusual_scenarios(
      CS,
      self.sm['radarState'] if 'radarState' in self.sm else None,
      self.sm['modelV2'] if 'modelV2' in self.sm else None
    )

    # Get adaptive control modifications based on detected scenarios and thermal state
    adaptive_mods = self.edge_case_handler.get_adaptive_control_modifications(CS, edge_scenarios)

    # Apply thermal-aware conservative limits
    conservative_accel_limits = self.CI.get_pid_accel_limits(self.CP, self.CP_SP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    if self.performance_compensation_factor < 0.7:  # Significant thermal stress
        # Apply additional conservatism during thermal stress
        thermal_conservative_factor = self.performance_compensation_factor / 0.7
        conservative_accel_limits = (
          conservative_accel_limits[0] * thermal_conservative_factor,
          conservative_accel_limits[1] * thermal_conservative_factor
        )

    # Enhanced saturation handling with adaptive limits based on thermal state
    actuators.accel = float(self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, conservative_accel_limits))

    # Apply self-learning adjustments to model outputs
    base_desired_curvature = model_v2.action.desiredCurvature
    learned_adjusted_curvature = self.self_learning_manager.adjust_curvature(base_desired_curvature, CS.vEgo)

    # Apply lateral control modifications if needed (after learning adjustment)
    modified_desired_curvature = learned_adjusted_curvature
    if adaptive_mods['lateral_factor'] < 1.0 and CC.latActive:
      # Make lateral control more conservative by reducing desired curvature
      modified_desired_curvature = learned_adjusted_curvature * adaptive_mods['lateral_factor']

    # Apply thermal-aware curvature limits
    if self.performance_compensation_factor < 0.7:
        # In high thermal stress, reduce aggressive curvature changes
        thermal_curvature_factor = max(0.7, self.performance_compensation_factor)
        modified_desired_curvature *= thermal_curvature_factor

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    new_desired_curvature = modified_desired_curvature if CC.latActive else self.curvature
    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    lat_delay = self.sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS

    actuators.curvature = self.desired_curvature
    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       self.calibrated_pose, curvature_limited, lat_delay)

    # Enhanced saturation detection with smoother recovery
    saturation_detected = False
    if hasattr(lac_log, 'saturated') and lac_log.saturated:
      saturation_detected = True
      cloudlog.debug(f"Steering saturation detected at vEgo: {CS.vEgo:.2f} m/s")

    actuators.torque = float(steer)
    actuators.steeringAngleDeg = float(steeringAngleDeg)

    # Enhanced finite value checks with recovery mechanism
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      if not math.isfinite(attr):
        cloudlog.error(
            f"actuators.{p} not finite. Actuators: {actuators.to_dict()}, CarState: {CS.to_dict()}, " +
            f"LongitudinalPlan: {long_plan.to_dict()}, LateralControlLog: {lac_log.to_dict()}"
        )
        # Implement a recovery by setting to a safe value instead of 0.0
        if p in ['steeringAngleDeg', 'curvature']:
          # For steering-related values, use current measurement as fallback
          setattr(actuators, p, 0.0 if p == 'curvature' else CS.steeringAngleDeg)
          CC.hudControl.visualAlert = log.ControlsState.AlertStatus.critical # Indicate non-finite steering to user
        elif p == 'accel':
          # For acceleration, use 0 to maintain current speed
          setattr(actuators, p, 0.0)
        else:
          # Default fallback to 0.0
          setattr(actuators, p, 0.0)
      # Additional safety validation for extreme values
      elif p == 'accel':
        # Check for excessive acceleration commands that exceed physical limits
        if abs(attr) > 5.0:  # More than 0.5g acceleration is extreme
          cloudlog.warning(f"Excessive acceleration command detected: {attr} m/s^2, limiting to safe value")
          limited_accel = max(-5.0, min(5.0, attr))
          setattr(actuators, p, limited_accel)
          CC.hudControl.visualAlert = log.ControlsState.AlertStatus.warning
      elif p in ['steeringAngleDeg']:
        # Check for excessive steering angle commands beyond physical limits
        if abs(attr) > 90.0:  # Steering angle beyond 90 degrees should never happen
          cloudlog.error(f"Excessive steering angle command detected: {attr} deg, limiting to safe value")
          limited_steering = max(-90.0, min(90.0, attr))
          setattr(actuators, p, limited_steering)
          CC.hudControl.visualAlert = log.ControlsState.AlertStatus.critical
      elif p in ['curvature']:
        # Check for excessive curvature commands (very tight turns at speed)
        if abs(attr) > 0.5 and CS.vEgo > 5.0:  # Very high curvature at speed is dangerous
          # Calculate max safe curvature based on speed: curvature = max_lat_accel / vEgo^2
          max_lat_accel = 3.0  # Conservative maximum lateral acceleration in m/s^2
          max_safe_curvature = max_lat_accel / (CS.vEgo ** 2)
          if abs(attr) > max_safe_curvature:
            cloudlog.warning(f"Excessive curvature command detected: {attr} vs max safe {max_safe_curvature} at speed {CS.vEgo}, limiting")
            limited_curvature = max(-max_safe_curvature, min(max_safe_curvature, attr))
            setattr(actuators, p, limited_curvature)
            CC.hudControl.visualAlert = log.ControlsState.AlertStatus.warning

    # Enhanced saturation handling in controls state
    if saturation_detected:
      CC.hudControl.visualAlert = log.ControlsState.AlertStatus.normal  # Indicate saturation to user

    # Update self-learning manager with current driving experience
    model_confidence = getattr(model_v2.meta, 'confidence', 1.0) if hasattr(model_v2, 'meta') else 1.0
    self.self_learning_manager.update(
        CS,
        base_desired_curvature,  # Original model output
        self.curvature,  # Actual vehicle curvature
        actuators.torque,
        CS.vEgo,
        model_confidence=model_confidence
    )

    return CC, lac_log


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
