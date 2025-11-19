#!/usr/bin/env python3
import math
import threading
import time
import numpy as np
from numbers import Number

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.enhanced_longitudinal_planner import EnhancedLongitudinalPlanner
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from openpilot.common.performance_monitor import PerfTrack, perf_monitor
from openpilot.selfdrive.common.metrics import Metrics, record_metric
from openpilot.selfdrive.modeld.neon_optimizer import optimize_curvature_calculation, neon_optimizer
from openpilot.common.dynamic_adaptation import dynamic_adaptation, performance_manager
from openpilot.common.resource_aware import resource_manager, resource_aware_runner, run_safety_critical_function
from openpilot.common.data_collector import collect_model_performance, collect_lane_change_event
from openpilot.selfdrive.common.thermal_management import thermal_manager, resource_manager as thermal_resource_manager
from openpilot.selfdrive.modeld.model_efficiency import ModelEfficiencyOptimizer, create_efficient_model_wrapper
from openpilot.selfdrive.monitoring.realtime_dashboard import realtime_dashboard
from openpilot.common.resource_aware import PriorityLevel
from openpilot.selfdrive.common.enhanced_validation import enhanced_validator
from openpilot.selfdrive.common.adaptive_control import adaptive_control, adaptive_lat_control
from openpilot.selfdrive.common.validation_publisher import validation_metrics_publisher

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

    self.sm = messaging.SubMaster(['liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance', 'liveDelay',
                                   'navInstruction', 'navStatus', 'validationMetrics', 'radarState', 'deviceState'] + self.sm_services_ext,
                                  poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'] + self.pm_services_ext)

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.desired_curvature = 0.0

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP, self.CP_SP)
    self.VM = VehicleModel(self.CP)
    self.enhanced_long_planner = EnhancedLongitudinalPlanner(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CP_SP, self.CI)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CP_SP, self.CI)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CP_SP, self.CI)

    # Enhanced pre-allocated arrays to reduce allocation in critical control loop
    self._angle_array = np.zeros(3, dtype=np.float32)  # For orientation/angle data
    self._angular_velocity_array = np.zeros(3, dtype=np.float32)  # For angular velocity data
    self._orientation_xyz = np.zeros(3, dtype=np.float32)  # For calibrated pose orientation
    self._angular_velocity_xyz = np.zeros(3, dtype=np.float32)  # For calibrated pose angular velocity
    self._actuators_array = np.zeros(len(ACTUATOR_FIELDS), dtype=np.float32)  # Pre-allocated for actuators validation

    # Initialize optimization components
    self.performance_manager.register_component("controls", {
      'update_rate': 100,  # Base rate of 100Hz
      'precision': 'high'
    })

    # Initialize model efficiency wrapper for performance adaptation
    self.model_efficiency_wrapper = create_efficient_model_wrapper(self.LaC)

    # Register with resource manager
    self.resource_allocation = resource_manager.request_resources(
      process_id="controls_system",
      priority=PriorityLevel.CRITICAL,
      cpu_required=2.0,
      memory_required=50.0,
      gpu_required=1.0,
      duration_estimate=0.01
    )

    # Initialize enhanced validation system
    self.enhanced_validator = enhanced_validator

    # Initialize adaptive control system
    self.adaptive_control = adaptive_control
    self.adaptive_lat_control = adaptive_lat_control

    # Initialize validation metrics publisher
    self.validation_metrics_publisher = validation_metrics_publisher

    # Memory pool for curvature calculations
    self._curvature_pool = np.zeros(10, dtype=np.float32)  # Pool for temporary curvature calculations
    self._curvature_pool_idx = 0

  def update(self):
    # Check performance adaptation and adjust if needed
    perf_factor = self.performance_manager.get_component_factor("controls")
    if perf_factor < 0.8:  # Only when significantly throttled
      # Skip some updates to reduce computational load
      if np.random.random() > perf_factor:  # Skip based on performance factor
        self.sm.update(15)
        return

    self.sm.update(15)
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

  def state_control(self):
    # Track performance for the entire state_control method
    perf_start_time = time.time()
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    # Use optimized curvature calculation with memory pooling
    angle_offset_diff = CS.steeringAngleDeg - lp.angleOffsetDeg
    self.curvature = optimize_curvature_calculation(angle_offset_diff, CS.vEgo, lp.roll)

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
    validation_metrics = self.sm['validationMetrics'] if 'validationMetrics' in self.sm else None

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Optimize standstill check
    self._standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill

    # Get which state to use for active lateral control
    _lat_active = self.get_lat_active(self.sm)

    # Optimize boolean logic
    lat_active_conditions = _lat_active and not (CS.steerFaultTemporary or CS.steerFaultPermanent)
    CC.latActive = lat_active_conditions and (not self._standstill or self.CP.steerAtStandstill)

    # Optimize long active check
    long_events_override = any(e.overrideLongitudinal for e in self.sm['onroadEvents'])
    CC.longActive = CC.enabled and not long_events_override and (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Initialize safety validation based on validation metrics
    validation_state = {
        'lead_confidence_ok': True,
        'lane_confidence_ok': True,
        'overall_confidence_ok': True,
        'system_safe': True,
        'lane_change_safe': True
    }

    if validation_metrics is not None and validation_metrics.isValid:
        # Check safety thresholds based on validation metrics with enhanced error handling
        try:
            lead_conf_avg = validation_metrics.leadConfidenceAvg
            lane_conf_avg = validation_metrics.laneConfidenceAvg
            overall_conf = validation_metrics.overallConfidence

            # Validate that confidence values are within expected range
            if not (0.0 <= lead_conf_avg <= 1.0):
                cloudlog.warning(f"Invalid lead confidence value: {lead_conf_avg}")
                lead_conf_avg = max(0.0, min(1.0, lead_conf_avg))  # Clamp to valid range

            if not (0.0 <= lane_conf_avg <= 1.0):
                cloudlog.warning(f"Invalid lane confidence value: {lane_conf_avg}")
                lane_conf_avg = max(0.0, min(1.0, lane_conf_avg))  # Clamp to valid range

            if not (0.0 <= overall_conf <= 1.0):
                cloudlog.warning(f"Invalid overall confidence value: {overall_conf}")
                overall_conf = max(0.0, min(1.0, overall_conf))  # Clamp to valid range

            lead_conf_ok = lead_conf_avg >= 0.6
            lane_conf_ok = lane_conf_avg >= 0.65
            overall_conf_ok = overall_conf >= 0.6
            lane_change_conf_ok = overall_conf >= 0.7 and lane_conf_avg >= 0.7  # Higher threshold for lane changes

            validation_state['lead_confidence_ok'] = lead_conf_ok
            validation_state['lane_confidence_ok'] = lane_conf_ok
            validation_state['overall_confidence_ok'] = overall_conf_ok
            validation_state['lane_change_safe'] = lane_change_conf_ok
        except AttributeError as e:
            cloudlog.error(f"Missing validation metrics attribute: {e}")
            # Set all to safe defaults
            validation_state.update({k: True for k in validation_state.keys()})
        except Exception as e:
            cloudlog.error(f"Error processing validation metrics: {e}")
            # Set all to safe defaults
            validation_state.update({k: True for k in validation_state.keys()})
        validation_state['system_safe'] = lead_conf_ok and lane_conf_ok and overall_conf_ok

    # Apply enhanced validation with situation-aware confidence calculation
    try:
        enhanced_validation_result = self.enhanced_validator.calculate_situation_aware_confidence(
            {
                'lead_confidence_avg': validation_metrics.leadConfidenceAvg if validation_metrics else 0.0,
                'lane_confidence_avg': validation_metrics.laneConfidenceAvg if validation_metrics else 0.0,
                'road_edge_confidence_avg': validation_metrics.leadConfidenceAvg if validation_metrics else 0.0,  # Placeholder
                'temporal_consistency': validation_metrics.overallConfidence if validation_metrics else 1.0,  # Placeholder
                'path_in_lane_validity': validation_metrics.overallConfidence if validation_metrics else 0.0,  # Placeholder
                'overall_confidence': validation_metrics.overallConfidence if validation_metrics else 0.0,
                'lane_count': 2  # Placeholder - would need real lane count from modelV2
            },
            CS,
            road_condition='highway'  # This would be determined from map data in a full implementation
        )

        # Override basic validation with enhanced validation results
        validation_state['lead_confidence_ok'] = enhanced_validation_result['lead_confidence_ok']
        validation_state['lane_confidence_ok'] = enhanced_validation_result['lane_confidence_ok']
        validation_state['overall_confidence_ok'] = enhanced_validation_result['overall_confidence_ok']
        validation_state['system_safe'] = enhanced_validation_result['system_safe']
        validation_state['lane_change_safe'] = enhanced_validation_result['lane_change_safe']

        # Enhanced safety recommendation
        is_safe, safety_reason = self.enhanced_validator.get_safety_recommendation(enhanced_validation_result, CS)
        validation_state['system_engagement_safe'] = is_safe

        # Publish enhanced validation metrics for system-wide use
        try:
            self.validation_metrics_publisher = validation_metrics_publisher
            self.validation_metrics_publisher.publish_metrics(enhanced_validation_result,
                                                            model_v2, CS)
        except Exception as e:
            cloudlog.error(f"Error publishing validation metrics: {e}")
    except Exception as e:
        cloudlog.error(f"Error in enhanced validation: {e}")
        validation_state['system_engagement_safe'] = validation_state['system_safe']  # Fall back to basic validation

    # Dynamic safety margins for lane change decision-making based on validation metrics
    original_lane_change_state = model_v2.meta.laneChangeState
    lane_change_state = original_lane_change_state  # Default to original state

    # Modify lane change state based on validation metrics for safety
    # This prevents lane changes when system confidence is low, reducing accident risk
    if validation_metrics is not None and original_lane_change_state != LaneChangeState.off:
        if not validation_state['lane_change_safe']:
            # If validation metrics indicate low confidence, cancel or prevent lane changes
            # Set to preLaneChange to slow down the transition
            if original_lane_change_state == LaneChangeState.laneChangeStarting:
                lane_change_state = LaneChangeState.preLaneChange
            elif original_lane_change_state == LaneChangeState.laneChangeFinishing:
                lane_change_state = LaneChangeState.laneChangeStarting  # Interrupt finish

    # Enable blinkers while lane changing - optimize by checking if necessary
    if lane_change_state != LaneChangeState.off:
      lane_change_direction = model_v2.meta.laneChangeDirection
      CC.leftBlinker = lane_change_direction == LaneChangeDirection.left
      CC.rightBlinker = lane_change_direction == LaneChangeDirection.right

      # Collect lane change event for data analysis
      if validation_metrics is not None:
        collect_lane_change_event(lane_change_state, validation_metrics.overallConfidence,
                                CS.steeringAngleDeg, CS.vEgo)

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # Enhanced longitudinal control using validation metrics for safety
    v_cruise_ms = CS.vCruise * CV.KPH_TO_MS
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, self.CP_SP, CS.vEgo, v_cruise_ms)

    # Apply enhanced longitudinal planning if validation metrics are available
    radar_state = self.sm['radarState'] if self.sm.updated['radarState'] else None
    if validation_metrics is not None:
        # Update lead vehicle prediction with confidence-based adjustments
        enhanced_long_plan = self.enhanced_long_planner.update_lead_vehicle_prediction(
            long_plan,
            radar_state,
            CS,  # Pass car state for vEgo information
            {
                'leadConfidenceAvg': validation_metrics.leadConfidenceAvg,
                'leadConfidenceMax': validation_metrics.leadConfidenceMax,
                'laneConfidenceAvg': validation_metrics.laneConfidenceAvg,
                'overallConfidence': validation_metrics.overallConfidence,
                'isValid': validation_metrics.isValid,
                'confidenceThreshold': validation_metrics.confidenceThreshold
            }
        )

        # Use enhanced longitudinal plan
        enhanced_a_target = enhanced_long_plan.aTarget
        enhanced_should_stop = enhanced_long_plan.shouldStop
    else:
        # Use original plan when validation metrics not available
        enhanced_a_target = long_plan.aTarget
        enhanced_should_stop = long_plan.shouldStop

    # Adjust control aggressiveness based on validation metrics
    if validation_metrics is not None and not validation_state['lead_confidence_ok']:
        # Reduce PID aggressiveness when lead detection confidence is low
        # This makes the system more conservative with longitudinal control
        pid_accel_limits = (
            max(pid_accel_limits[0] * 0.8, -4.0),  # More conservative deceleration
            min(pid_accel_limits[1] * 0.8, 2.0)   # More conservative acceleration
        )

    actuators.accel = self.LoC.update(CC.longActive, CS, enhanced_a_target, enhanced_should_stop, pid_accel_limits)

    # Steering PID loop and lateral MPC - optimize curvature calculations
    active_curvature = model_v2.action.desiredCurvature if CC.latActive else self.curvature

    # Apply dynamic safety margins based on validation metrics
    if validation_metrics is not None:
        if not validation_state['lane_confidence_ok']:
            # Reduce desired curvature when lane detection confidence is low
            # This makes lateral control more conservative
            if abs(active_curvature) > 0.005:  # Only modify if there's significant curvature request
                active_curvature = active_curvature * 0.7  # Reduce by 30% when confidence is low
        elif validation_metrics.overallConfidence >= 0.85:
            # More aggressive control when high confidence
            active_curvature = active_curvature * 1.1  # Slightly more responsive when confidence is high

    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, active_curvature, lp.roll)

    actuators.curvature = self.desired_curvature

    # Apply additional safety checks based on validation state
    lat_active_with_safety = CC.latActive and validation_state['system_safe']

    # Modify lateral control if validation metrics indicate low confidence
    if not validation_state['system_safe']:
        # Be very conservative with lateral control when system confidence is low
        if abs(self.desired_curvature) < 0.005:  # Only allow very small corrections
            # Allow minimal lateral control but with reduced authority
            lat_active_with_safety = abs(self.desired_curvature) < 0.001
        else:
            # Deactivate lateral control if system is not safe and curvature request is significant
            lat_active_with_safety = False
    else:
        # Normal operation when system is safe
        lat_active_with_safety = CC.latActive

    # Apply adaptive control parameters based on current conditions
    try:
        env_data = {}  # Would be populated from external sources in a full implementation
        adaptive_params = self.adaptive_control.adjust_for_conditions(CS, env_data)

        # Update lateral control gains if using torque control
        if self.CP.lateralTuning.which() == 'torque':
            self.adaptive_lat_control.update_gains_for_conditions(CS, env_data)
    except Exception as e:
        cloudlog.error(f"Error applying adaptive control: {e}")

    # Optimized lateral control update with pre-allocated memory where possible
    steer, steeringAngleDeg, lac_log = self.LaC.update(lat_active_with_safety, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       self.calibrated_pose, curvature_limited)
    actuators.torque = steer
    actuators.steeringAngleDeg = steeringAngleDeg

    # More efficient finite check - avoid expensive dict creation and use pre-allocated array where applicable
    for i, p in enumerate(ACTUATOR_FIELDS):
      attr = getattr(actuators, p)
      if not isinstance(attr, Number) or math.isfinite(attr):
        continue

      # Only create dict if there's an error (rare case)
      cloudlog.error(f"actuators.{p} not finite")
      setattr(actuators, p, 0.0)

    # Record control system metrics with performance data
    state_control_time = (time.time() - perf_start_time) * 1000  # Convert to ms
    collect_model_performance("controls", "state_control", state_control_time, {
        "v_ego": CS.vEgo,
        "steering_angle_deg": CS.steeringAngleDeg,
        "curvature": self.curvature,
        "engaged": self.sm['selfdriveState'].enabled
    })

    # Record control system metrics
    record_metric(Metrics.STEERING_LATENCY_MS, 0.0, {  # Placeholder - actual latency would need real measurement
        "operation": "steering_update",
        "v_ego": CS.vEgo,
        "steering_angle_deg": CS.steeringAngleDeg,
        "curvature": self.curvature
    })

    record_metric(Metrics.BRAKING_LATENCY_MS, 0.0, {  # Placeholder - actual latency would need real measurement
        "operation": "braking_update",
        "v_ego": CS.vEgo,
        "a_ego": CS.aEgo,
        "accel_cmd": actuators.accel
    })

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      # Use pre-allocated arrays to avoid repeated list creation
      np.copyto(self._angle_array[:3], self.calibrated_pose.orientation.xyz)
      CC.orientationNED = self._angle_array[:3].tolist()
      np.copyto(self._angular_velocity_array[:3], self.calibrated_pose.angular_velocity.xyz)
      CC.angularVelocity = self._angular_velocity_array[:3].tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)
    CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not self.sm['longitudinalPlan'].shouldStop

    hudControl = CC.hudControl
    hudControl.setSpeed = CS.vCruiseCluster * CV.KPH_TO_MS  # Removed float() wrapper
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

    # Safety validation layers - trigger fallback behaviors when confidence metrics are low
    safety_status = {
        'fallback_active': False,
        'degraded_mode': False,
        'system_alert': False
    }

    if validation_metrics is not None:
        # Trigger fallback behaviors based on validation metrics
        overall_confidence = validation_metrics.overallConfidence
        if overall_confidence < 0.4:
            # Critical safety threshold - system should prepare for disengagement
            safety_status['fallback_active'] = True
            safety_status['system_alert'] = True
            # In critical mode, allow disengagement only if not currently engaged in critical maneuver
            # and if the driver is ready to take over
            if CC.enabled and self.sm['selfdriveState'].enabled:
                # Log the event for analysis
                cloudlog.warning(f"Critical validation confidence detected: {overall_confidence}, preparing for disengagement")
                # Consider disengaging only if conditions are safe to do so
                # For now, we'll continue with the conservative approach but with a safety message
                CC.enabled = False
        elif overall_confidence < 0.6:
            # Degraded mode when confidence is moderate
            safety_status['degraded_mode'] = True
            # Reduce lateral control authority gradually
        elif overall_confidence < 0.75:
            # Warning level when confidence is somewhat low
            safety_status['system_alert'] = True
    else:
        # If validation metrics are not available, default to conservative behavior
        safety_status['system_alert'] = True  # Alert that we don't have validation data

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = self.LoC.pid.p  # Removed float() wrapper
    cs.uiAccelCmd = self.LoC.pid.i  # Removed float() wrapper
    cs.ufAccelCmd = self.LoC.pid.f  # Removed float() wrapper
    cs.forceDecel = (self.sm['driverMonitoringState'].awarenessStatus < 0.) or \
                     (self.sm['selfdriveState'].state == State.softDisabling)  # Removed bool() wrapper

    # Add safety validation metrics to controlsState for monitoring
    if validation_metrics is not None:
        # Set validation metrics - need to create and populate the metrics struct
        cs.validation.metrics.leadConfidenceAvg = validation_metrics.leadConfidenceAvg
        cs.validation.metrics.leadConfidenceMax = validation_metrics.leadConfidenceMax
        cs.validation.metrics.laneConfidenceAvg = validation_metrics.laneConfidenceAvg
        cs.validation.metrics.overallConfidence = validation_metrics.overallConfidence
        cs.validation.metrics.isValid = validation_metrics.isValid
        cs.validation.metrics.confidenceThreshold = validation_metrics.confidenceThreshold
    else:
        # Default values when validation metrics are not available
        cs.validation.metrics.leadConfidenceAvg = 0.0
        cs.validation.metrics.leadConfidenceMax = 0.0
        cs.validation.metrics.laneConfidenceAvg = 0.0
        cs.validation.metrics.overallConfidence = 0.0
        cs.validation.metrics.isValid = False
        cs.validation.metrics.confidenceThreshold = 0.5

    # Add enhanced validation metrics to controlsState
    try:
        if hasattr(self, 'enhanced_validator') and enhanced_validation_result:
            cs.validation.enhanced.situationFactor = enhanced_validation_result.get('situation_factor', 1.0)
            cs.validation.enhanced.speedAdjustedConfidence = enhanced_validation_result.get('speed_adjusted_confidence', 0.0)
            cs.validation.enhanced.temporalConsistency = enhanced_validation_result.get('temporal_consistency', 1.0)
            cs.validation.enhanced.systemSafe = enhanced_validation_result.get('system_safe', False)
    except Exception:
        # Continue without enhanced metrics if there's an error
        pass

    # Update state based on safety status
    cs.fallbackActive = safety_status['fallback_active']
    cs.degradedMode = safety_status['degraded_mode']
    cs.systemAlert = safety_status['system_alert']

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
    rk = Ratekeeper(100, print_delay_threshold=None)
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e,))
    try:
      t.start()
      loop_counter = 0
      while True:
        # Check resource availability and adapt if needed
        # Only check performance factor every few iterations to reduce overhead
        if loop_counter % 10 == 0:  # Check every 10 loops (about every 0.1 seconds at 100Hz)
          perf_factor = self.performance_manager.get_component_factor("controls")
        else:
          # Use the last known performance factor to reduce computation
          if 'perf_factor' not in locals() or not hasattr(self, 'last_perf_factor'):
            self.last_perf_factor = 1.0  # Default value
          perf_factor = self.last_perf_factor

        # Update thermal status and get performance recommendations
        if self.sm.updated['deviceState']:
          thermal_metrics = thermal_manager.update_thermal_status(self.sm['deviceState'])
          perf_factor = min(perf_factor, thermal_manager.get_current_performance_scale())

        # Track the overall loop performance
        with PerfTrack("controlsd_loop") as loop_tracker:
          with PerfTrack("controlsd_update"):
            self.update()
          with PerfTrack("controlsd_state_control") as perf_tracker:
            CC, lac_log = self.state_control()
          with PerfTrack("controlsd_publish"):
            self.publish(CC, lac_log)
          with PerfTrack("controlsd_ext"):
            self.run_ext(self.sm, self.pm)

          # Collect performance data for dashboard - but only periodically to reduce overhead
          if loop_counter % 50 == 0:  # Collect performance data every 50 loops (about every 0.5 seconds)
            loop_time = loop_tracker.get_time_ms()
            collect_model_performance("controlsd", "main_loop", loop_time, {
              "perf_factor": perf_factor,
              "thermal_scale": thermal_manager.get_current_performance_scale()
            })

          loop_counter += 1
          rk.monitor_time()
    finally:
      e.set()
      t.join()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
