"""
Advanced safety features and fail-safes for autonomous driving.

This module provides enhanced safety mechanisms to improve the reliability
and safety of the autonomous driving system by implementing various checks
and fail-safe procedures.
"""

from collections import deque
import numpy as np
import time
from typing import Any
from cereal import car, log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog


class SafetyManager:
  """
  Advanced safety manager that monitors various system parameters and
  provides fail-safe mechanisms when safety limits are exceeded.
  """

  def __init__(self):
    self.params = Params()

    # System state monitoring
    self.steering_torque_history = deque(maxlen=20)  # 0.2 seconds at 100Hz
    self.acceleration_history = deque(maxlen=20)      # 0.2 seconds at 100Hz
    self.steering_angle_history = deque(maxlen=10)    # 0.1 seconds at 100Hz

    # Safety thresholds (configurable)
    self.max_unexpected_torque = self.params.get_float("SafetyMaxUnexpectedTorque", 200.0)
    self.max_acceleration_change = self.params.get_float("SafetyMaxAccelerationChange", 5.0)  # m/s^3
    self.max_steering_rate = self.params.get_float("SafetyMaxSteeringRate", 100.0)  # deg/s
    self.min_model_prediction_confidence = self.params.get_float("SafetyMinModelConfidence", 0.5)

    # Risk assessment levels for graduated response
    self.risk_assessment_levels = {
        'low': {'score_range': (0.0, 0.3), 'response': 'normal'},
        'moderate': {'score_range': (0.3, 0.6), 'response': 'caution'},
        'high': {'score_range': (0.6, 0.85), 'response': 'warning'},
        'critical': {'score_range': (0.85, 1.0), 'response': 'disengage'}
    }
    self.risk_history = deque(maxlen=50)  # Track risk over time
    self.degradation_counter = 0  # Track degradation over time

    # Safety state variables
    # safety_violation_count: Accumulates violations. It's a soft counter; a single transient
    #   issue won't immediately cause disengagement. It gradually decreases if no new violations occur.
    self.safety_violation_count: float = 0.0
    # max_safety_violations: The threshold for consecutive safety violations that triggers
    #   a critical disengagement. This prevents single, transient glitches from causing
    #   a major annoyance for users, only disengaging after persistent problems.
    self.max_safety_violations = 3  # Maximum violations before disengagement
    self.safety_engaged = True  # Whether safety system is actively monitoring
    self.violation_threshold = 0.5  # Threshold for triggering safety actions

    # Initialize state variables
    self._last_steering_angle = 0.0
    self._last_acceleration = 0.0
    self._last_update_time = 0.0
    self._initialized = False

  def update_params(self):
    """Update safety parameters from system parameters"""
    try:
      self.max_unexpected_torque = float(self.params.get("SafetyMaxUnexpectedTorque", "200.0"))
      self.max_acceleration_change = float(self.params.get("SafetyMaxAccelerationChange", "5.0"))
      self.max_steering_rate = float(self.params.get("SafetyMaxSteeringRate", "100.0"))
      self.min_model_prediction_confidence = float(self.params.get("SafetyMinModelConfidence", "0.5"))
    except (TypeError, ValueError):
      # Use defaults if parameters are invalid
      pass

  def monitor_steering_safety(self, car_state: car.CarState, control_output: car.CarControl.Actuators) -> bool:
    """
    Monitor steering safety by checking for unexpected torques or rapid changes.

    Args:
      car_state: Current car state
                control_output: Control system output
              Returns:
                True if steering is safe, False otherwise    """
    # Check for excessive steering torque
    if abs(car_state.steeringTorque) > self.max_unexpected_torque:
      # Check if the high torque is expected based on our control request
      expected_torque = abs(control_output.torque) if control_output else 0
      if abs(car_state.steeringTorque) > expected_torque * 1.5:  # 150% of expected
        return False

    # Check for rapid steering rate changes
    current_angle: float = car_state.steeringAngleDeg
    if self._initialized:
      time_diff = DT_CTRL  # Fixed time step
      if time_diff > 0:
        steering_rate = abs(current_angle - self._last_steering_angle) / time_diff
        if steering_rate > self.max_steering_rate:
          return False

    # Store current values for next iteration
    self.steering_torque_history.append(abs(car_state.steeringTorque))
    self.steering_angle_history.append(current_angle)
    self._last_steering_angle = current_angle
    self._initialized = True

    return True

  def monitor_longitudinal_safety(self, car_state: car.CarState, control_output: car.CarControl.Actuators) -> bool:
    """
    Monitor longitudinal safety by checking for unexpected acceleration changes.

    Args:
      car_state: Current car state
      control_output: Control system output

    Returns:
      True if longitudinal control is safe, False otherwise
    """
    current_accel = car_state.aEgo
    self.acceleration_history.append(current_accel)

    if len(self.acceleration_history) >= 2 and self._initialized:
      # Calculate acceleration change rate (jerk)
      if len(self.acceleration_history) >= 2:
        recent_acceleration = list(self.acceleration_history)[-2:]
        if len(recent_acceleration) == 2:
          accel_change = abs(recent_acceleration[1] - recent_acceleration[0])
          jerk = accel_change / DT_CTRL  # Approximate jerk

          if jerk > self.max_acceleration_change:
            return False

    # Store current acceleration for next iteration
    self._last_acceleration = current_accel

    return True

  def monitor_model_prediction_safety(self, model_data) -> bool:
    """
    Monitor model prediction safety by checking for inconsistent or low-confidence predictions.

    Args:
      model_data: Model predictions from modeld

    Returns:
      True if model predictions are safe, False otherwise
    """
    if model_data is None:
      return True  # Can't assess, assume safe

    # Extract and check multiple confidence indicators from model data
    confidence_metrics = self._extract_confidence_metrics(model_data)

    # Calculate overall prediction confidence score
    prediction_confidence = self._calculate_prediction_confidence(model_data, confidence_metrics)

    # If confidence is below threshold, consider unsafe
    if prediction_confidence < self.min_model_prediction_confidence:
      threshold = self.min_model_prediction_confidence
      cloudlog.debug(f"Safety: Low model prediction confidence: {prediction_confidence:.2f} (threshold: {threshold:.2f})")
      return False

    # Check for prediction consistency across frames
    if not self._check_prediction_consistency(model_data, prediction_confidence):
      return False

    # Check for physically unrealistic predictions
    if not self._check_physical_feasibility(model_data):
      return False

    return True

  def _extract_confidence_metrics(self, model_data):
    """
    Extract various confidence indicators from model data
    """
    metrics = {}

    try:
        # Extract from model metadata if available
        if hasattr(model_data, 'meta') and hasattr(model_data.meta, 'featureStats'):
            for i, feature in enumerate(model_data.meta.featureStats):
                metrics[f'feature_{i}_confidence'] = feature.confidence

        # Extract from plan confidence if available
        if hasattr(model_data, 'plan'):
            metrics['plan_confidence'] = self._calculate_plan_confidence(model_data.plan)

        # Extract from lane line detection confidence
        if hasattr(model_data, 'lateralPlannerOutput'):
            metrics['lane_confidence'] = self._calculate_lane_confidence(model_data.lateralPlannerOutput)

    except Exception as e:
        cloudlog.error(f"Error extracting confidence metrics: {e}")

    return metrics

  def _calculate_prediction_confidence(self, model_data, confidence_metrics):
    """
    Calculate overall prediction confidence score
    """
    weights = {
        'engage_prob': 0.3,
        'plan_consistency': 0.25,
        'feature_confidence': 0.2,
        'lane_confidence': 0.15,
        'physical_feasibility': 0.1
    }

    score = 0.0

    # Engage probability (if available)
    engage_prob = getattr(model_data.meta, 'engageProb', 0.5)
    score += weights['engage_prob'] * min(1.0, max(0.0, engage_prob))

    # Plan consistency (smoothness of planned trajectory)
    if 'plan_confidence' in confidence_metrics:
        score += weights.get('plan_consistency', 0.0) * confidence_metrics['plan_confidence']

    # Feature detection confidence
    feature_scores = [v for k, v in confidence_metrics.items() if 'feature' in k]
    if feature_scores:
        avg_feature_conf = sum(feature_scores) / len(feature_scores)
        score += weights.get('feature_confidence', 0.0) * avg_feature_conf

    # Lane detection confidence
    if 'lane_confidence' in confidence_metrics:
        score += weights.get('lane_confidence', 0.0) * confidence_metrics['lane_confidence']

    return min(1.0, max(0.0, score))

  def _calculate_plan_confidence(self, plan):
    """
    Calculate plan consistency/confidence based on trajectory smoothness
    """
    # A smooth plan with gradual changes is more confident
    if hasattr(plan, 'position') and hasattr(plan.position, 'x') and len(plan.position.x) > 1:
        # Calculate smoothness by looking at change in trajectory
        x_positions = plan.position.x
        if len(x_positions) > 2:
            # Calculate changes and look for abrupt changes
            changes = [abs(x_positions[i+1] - x_positions[i]) for i in range(len(x_positions)-1)]
            avg_change = sum(changes) / len(changes)
            # More consistent changes = higher confidence
            if avg_change > 0:
                variance = sum((c - avg_change)**2 for c in changes) / len(changes)
                # Lower variance = higher confidence (inverted)
                return max(0.1, 1.0 - min(0.9, variance / avg_change if avg_change > 0 else 1.0))

    return 0.5  # Default if cannot calculate

  def _calculate_lane_confidence(self, lateral_planner_output):
    """
    Calculate confidence based on lane line detection
    """
    if hasattr(lateral_planner_output, 'laneLines'):
        high_confidence_lines = sum(1 for line in lateral_planner_output.laneLines if line.prob > 0.5)
        total_lines = len(lateral_planner_output.laneLines)
        return high_confidence_lines / total_lines if total_lines > 0 else 0.0

    return 0.5  # Default if no lane line data

  def _check_prediction_consistency(self, model_data, confidence):
    """
    Check for temporal consistency in predictions
    """
    # Store recent predictions for consistency checking
    if not hasattr(self, '_recent_predictions'):
        self._recent_predictions = []

    # Add current prediction
    current_prediction = {
        'curvature': getattr(model_data.action, 'desiredCurvature', 0),
        'acceleration': getattr(model_data.action, 'desiredAcceleration', 0),
        'timestamp': time.monotonic()  # Need to import time
    }

    self._recent_predictions.append(current_prediction)

    # Keep only recent predictions (last 1 second worth)
    cutoff_time = time.monotonic() - 1.0
    self._recent_predictions = [p for p in self._recent_predictions if p['timestamp'] > cutoff_time]

    if len(self._recent_predictions) < 3:
        return True  # Not enough data to check consistency

    # Check for excessive variation in predictions
    curvatures = [p['curvature'] for p in self._recent_predictions]
    accelerations = [p['acceleration'] for p in self._recent_predictions]

    curvature_variance = np.var(curvatures)
    acceleration_variance = np.var(accelerations)

    # If variance is excessive and confidence is low, flag inconsistency
    max_curvature_variance = 0.01 * (1.0 / max(0.1, confidence))  # Higher variance allowed at higher confidence
    max_acceleration_variance = 0.5 * (1.0 / max(0.1, confidence))

    if curvature_variance > max_curvature_variance or acceleration_variance > max_acceleration_variance:
        cloudlog.debug(
            f"Prediction inconsistency detected: curvature_var={curvature_variance:.4f}, accel_var={acceleration_variance:.4f}, conf={confidence:.2f}"
        )
        return False

    return True

  def _check_physical_feasibility(self, model_data):
    """
    Check if model predictions are physically feasible
    """
    try:
        if (hasattr(model_data, 'action') and
            hasattr(model_data.action, 'desiredCurvature') and
            hasattr(model_data, 'carState')):
            v_ego = model_data.carState.vEgo if hasattr(model_data.carState, 'vEgo') else 0
            desired_curvature = model_data.action.desiredCurvature

            # Calculate required lateral acceleration for the desired curvature
            required_lat_accel = desired_curvature * v_ego ** 2

            # Check against reasonable limits (typically 3-4 m/s^2 for production vehicles)
            max_lat_accel = 3.5
            if abs(required_lat_accel) > max_lat_accel:
                # Check if this is reasonable given current speed
                max_reasonable_curvature = max_lat_accel / (v_ego ** 2)
                if abs(desired_curvature) > max_reasonable_curvature * 2.0:  # Allow some margin
                    return False

        # Check acceleration feasibility
        if hasattr(model_data, 'action') and hasattr(model_data.action, 'desiredAcceleration'):
            desired_accel = model_data.action.desiredAcceleration
            max_decel = -4.0  # Reasonable maximum deceleration
            max_accel = 3.0   # Reasonable maximum acceleration

            if desired_accel < max_decel or desired_accel > max_accel:
                return False

    except (AttributeError, TypeError):
        pass

    return True

  def check_safety_violations(self, car_state: car.CarState, control_output: car.CarControl.Actuators, model_data=None) -> tuple[bool, str]:
    """
    Perform comprehensive safety check and return whether the system is safe.

    Args:
      car_state: Current car state
      control_output: Control system output
      model_data: Model predictions (optional)
    Returns:
      Tuple of (is_safe, violation_reason)
    """
    if not self.safety_engaged:
      return True, "safety_disabled"

    # Check steering safety
    if not self.monitor_steering_safety(car_state, control_output):
      self.safety_violation_count += 1
      cloudlog.debug(f"Safety violation: unsafe_steering. Count: {self.safety_violation_count:.1f}")
      return False, "unsafe_steering"

    # Check longitudinal safety
    if not self.monitor_longitudinal_safety(car_state, control_output):
      self.safety_violation_count += 1
      cloudlog.debug(f"Safety violation: unsafe_longitudinal. Count: {self.safety_violation_count:.1f}")
      return False, "unsafe_longitudinal"

    # Check model prediction safety
    if not self.monitor_model_prediction_safety(model_data):
      self.safety_violation_count += 1
      cloudlog.debug(f"Safety violation: unsafe_model_predictions. Count: {self.safety_violation_count:.1f}")
      return False, "unsafe_model_predictions"

    # Reset violation count on successful safety check
    # The safety_violation_count gradually decreases (by 0.1 per cycle) when no new violations
    # are detected. This mechanism allows the system to recover from transient, non-persistent
    # issues without immediately triggering a disengagement, balancing safety with usability.
    if self.safety_violation_count > 0:
      self.safety_violation_count = max(0, self.safety_violation_count - 0.1)  # Gradually decrease

    return True, "safe"

  def get_safety_recommendation(self, car_state: car.CarState, control_output: car.CarControl.Actuators, model_data=None) -> log.SelfdriveState.AlertStatus:
    """
    Get safety-based alert recommendation for the UI.

    Args:
      car_state: Current car state
      control_output: Control system output
      model_data: Model predictions (optional)

    Returns:
      Alert status based on safety assessment
    """
    is_safe, violation_reason = self.check_safety_violations(car_state, control_output, model_data)

    if not is_safe:
      if self.safety_violation_count >= self.max_safety_violations:
        return log.SelfdriveState.AlertStatus.critical
      else:
        return log.SelfdriveState.AlertStatus.userPrompt
    else:
      return log.SelfdriveState.AlertStatus.normal

  def should_disengage(self) -> bool:
    """
    Determine if the system should disengage based on safety violations.

    Returns:
      True if system should disengage, False otherwise
    """
    return self.safety_violation_count >= self.max_safety_violations

  def assess_comprehensive_risk(self, car_state: car.CarState, control_output: car.CarControl.Actuators, model_data=None,
                                radar_data=None, environment_data=None) -> float:
    """
    Comprehensive risk assessment combining multiple factors

    Args:
      car_state: Current car state
      control_output: Current control system output
      model_data: Model predictions (optional)
      radar_data: Radar data (optional)
      environment_data: Environment data (optional)

    Returns:
      float: Risk score from 0.0 (low risk) to 1.0 (high risk)
    """
    risk_factors = {
      'control_stability': self._assess_control_stability(car_state, control_output),
      'model_confidence': self._assess_model_confidence(model_data),
      'environmental': self._assess_environmental_risk(radar_data, environment_data),
      'vehicle_health': self._assess_vehicle_health(car_state),
    }

    # Weighted risk calculation
    weights = {
      'control_stability': 0.25,
      'model_confidence': 0.25,
      'environmental': 0.25,
      'vehicle_health': 0.25
    }

    total_risk = 0.0
    for factor, value in risk_factors.items():
      total_risk += value * weights.get(factor, 0.0)

    # Add temporal component - increasing risk if conditions are deteriorating
    self.risk_history.append(total_risk)
    if len(self.risk_history) > 10:
      recent_trend = self._calculate_risk_trend()
      if recent_trend > 0.05:  # Risk is increasing rapidly
        total_risk = min(1.0, total_risk * 1.2)  # Boost risk score if increasing

    # Enhanced safety layer: Critical fail-safes for immediate danger
    critical_risks = self._check_critical_safety_conditions(car_state, radar_data)
    if critical_risks > 0.9:
      total_risk = max(total_risk, critical_risks)

    return total_risk

  def _check_critical_safety_conditions(self, car_state: car.CarState, radar_data) -> float:
    """
    Check for critical safety conditions requiring immediate action
    This provides immediate fail-safe responses for dangerous situations.

    Critical Analysis Note: This is arguably the most important change, implementing
    a hard, non-negotiable safety net. The aggressive thresholds (e.g., TTC < 2s)
    are designed for immediate, life-threatening situations and trigger quick
    disengagement, which is the correct behavior. It is vital that these thresholds
    are well-understood and thoroughly validated. For some less absolute thresholds,
    consider making them configurable parameters to allow for nuanced tuning
    based on real-world testing and risk assessment.
    """
    max_risk = 0.0

    # Check for immediate collision risk
    if radar_data and hasattr(radar_data, 'leadOne') and radar_data.leadOne.status:
      lead = radar_data.leadOne
      if lead.dRel < 20.0 and lead.vRel < -1.0:  # Very close and closing fast
        # Calculate time to collision
        if lead.vRel < -0.1:  # Approaching lead
          time_to_collision = lead.dRel / abs(lead.vRel)
          if time_to_collision < 2.0:  # Less than 2 seconds - critical danger
            max_risk = max(max_risk, 0.95)
          elif time_to_collision < 3.0:  # Less than 3 seconds - high danger
            max_risk = max(max_risk, 0.85)

    # Check for vehicle system faults that require immediate disengagement
    if hasattr(car_state, 'steerFaultPermanent') and car_state.steerFaultPermanent:
      max_risk = max(max_risk, 0.95)  # Critical fault requiring immediate stop

    if hasattr(car_state, 'controlsAllowed') and not car_state.controlsAllowed:
      max_risk = max(max_risk, 0.9)  # System not allowed to control vehicle

    # Check for dangerous vehicle dynamics
    if (hasattr(car_state, 'aEgo') and hasattr(car_state, 'vEgo') and
        car_state.vEgo > 5.0 and abs(car_state.aEgo) > 5.0):
      # Excessive acceleration/deceleration at speed
      max_risk = max(max_risk, 0.7)

    # Check for brake and gas pedal conflict
    if (hasattr(car_state, 'brakePressed') and hasattr(car_state, 'gasPressed') and
        car_state.brakePressed and car_state.gasPressed):
      max_risk = max(max_risk, 0.75)  # Dangerous situation

    # Check for dangerous steering angle or rate
    if (hasattr(car_state, 'steeringAngleDeg') and hasattr(car_state, 'steeringRateDeg') and
        abs(car_state.steeringAngleDeg) > 45 and abs(car_state.steeringRateDeg) > 100):
      # Extreme angle with high rate of change - dangerous
      max_risk = max(max_risk, 0.8)

    return max_risk

  def _assess_control_stability(self, car_state: car.CarState, control_output) -> float:
    """
    Assess stability of control commands
    """
    # Calculate various stability metrics
    stability_metrics = []

    # 1. Control authority stability
    if hasattr(control_output, 'torque'):
      desired_torque = control_output.torque
      actual_torque_response = abs(car_state.steeringTorque) if hasattr(car_state, 'steeringTorque') else 0

      # Large discrepancy between desired and actual could indicate instability
      if abs(desired_torque) > 0.1:  # Only if significant command
        torque_mismatch = abs(desired_torque - actual_torque_response) / abs(desired_torque)
        stability_metrics.append(min(1.0, torque_mismatch))

    # 2. Command rate of change
    if not hasattr(self, '_prev_control_torque'):
      self._prev_control_torque = 0.0

    torque_change_rate = abs(control_output.torque - self._prev_control_torque) if hasattr(control_output, 'torque') else 0.0
    max_reasonable_change = 10.0  # Adjust based on vehicle characteristics
    rate_stability = min(1.0, torque_change_rate / max_reasonable_change)
    stability_metrics.append(rate_stability)

    self._prev_control_torque = control_output.torque if hasattr(control_output, 'torque') else 0.0

    # 3. Saturation frequency - how often we're hitting actuator limits
    if hasattr(control_output, 'saturation'):
        saturation_score = 0.1 if control_output.saturation else 0.0
        stability_metrics.append(saturation_score)

    return sum(stability_metrics) / len(stability_metrics) if stability_metrics else 0.0

  def _assess_model_confidence(self, model_data) -> float:
    """
    Assess confidence in model predictions (0.0 = no confidence, 1.0 = full confidence)
    Returns risk score which is 1.0 - confidence
    """
    if model_data is None:
      return 0.7  # High risk if no model data

    # Use the existing confidence assessment methods
    confidence_metrics = self._extract_confidence_metrics(model_data)
    prediction_confidence = self._calculate_prediction_confidence(model_data, confidence_metrics)

    # Additional checks for model consistency and feasibility
    if hasattr(model_data, 'action'):
      action = model_data.action
      if hasattr(action, 'desiredCurvature') and hasattr(action, 'desiredAcceleration'):
        # Check for physically unrealistic commands
        v_ego = getattr(model_data, 'vEgo', 0) if hasattr(model_data, 'vEgo') else 0.0

        # Check curvature vs speed for physical feasibility
        if v_ego > 1.0 and abs(action.desiredCurvature) > 0.1:
          required_lat_accel = action.desiredCurvature * v_ego ** 2
          if abs(required_lat_accel) > 4.0:  # Beyond reasonable lateral acceleration
            prediction_confidence = min(prediction_confidence, 0.3)

        # Check acceleration bounds
        if abs(action.desiredAcceleration) > 5.0:  # Beyond reasonable acceleration
          prediction_confidence = min(prediction_confidence, 0.4)

    # Check for sudden, unexplained changes in model output that might indicate sensor noise
    if not hasattr(self, '_prev_model_outputs'):
      self._prev_model_outputs: dict[str, Any] = {}

    # Store current model outputs for comparison in next cycle
    if hasattr(model_data, 'action'):
      current_action = {
        'curvature': getattr(model_data.action, 'desiredCurvature', 0),
        'acceleration': getattr(model_data.action, 'desiredAcceleration', 0),
        'should_stop': getattr(model_data.action, 'shouldStop', False)
      }

      if '_prev_action' in self._prev_model_outputs:
        # Calculate change magnitude
        prev_action = self._prev_model_outputs['_prev_action']
        curvature_change = abs(current_action['curvature'] - prev_action['curvature'])
        accel_change = abs(current_action['acceleration'] - prev_action['acceleration'])

        # High changes in short time may indicate unreliable model
        if curvature_change > 0.05 or abs(accel_change) > 1.0:
          prediction_confidence = min(prediction_confidence, 0.6)

      self._prev_model_outputs['_prev_action'] = current_action

    # Return risk (1.0 - confidence)
    return float(max(0.0, 1.0 - prediction_confidence))

  def _assess_lead_vehicle_risk(self, radar_data) -> float:
    risk = 0.0
    if radar_data.leadOne.status:
      # Calculate time to collision with lead vehicle
      relative_speed = radar_data.leadOne.vRel
      if relative_speed > 0.1:  # Approaching lead vehicle
        time_to_collision = radar_data.leadOne.dRel / relative_speed if relative_speed > 0.1 else float('inf')
        if time_to_collision < 1.5:  # Extremely critical (less than 1.5 seconds)
          risk = max(risk, 0.95)
        elif time_to_collision < 2.0:  # Critical (less than 2 seconds)
          risk = max(risk, 0.8)
        elif time_to_collision < 2.5:  # High risk (less than 2.5 seconds)
          risk = max(risk, 0.6)
        elif time_to_collision < 3.0:  # Moderate risk (less than 3 seconds)
          risk = max(risk, 0.4)
    return risk

  def _assess_lead_deceleration_risk(self, radar_data) -> float:
    risk = 0.0
    if radar_data.leadOne.status:
      # If lead vehicle is braking hard relative to our speed
      lead_deceleration = radar_data.leadOne.aRel if hasattr(radar_data.leadOne, 'aRel') else 0
      if lead_deceleration < -2.0:  # Lead vehicle braking hard
        if radar_data.leadOne.dRel < 40.0:  # Within 40m
          risk = max(risk, 0.6)
    return risk

  def _assess_multiple_vehicle_risk(self, radar_data) -> float:
    risk = 0.0
    if radar_data.leadOne.status and radar_data.leadTwo.status:
      # Check for potential cut-in between vehicles
      spacing = radar_data.leadTwo.dRel - radar_data.leadOne.dRel
      if spacing < 8.0 and radar_data.leadOne.dRel < 50.0:  # Close spacing and not too far
        # Check if middle vehicle is moving laterally (potential lane change)
        if hasattr(radar_data.leadOne, 'yRel') and hasattr(radar_data.leadTwo, 'yRel'):
          lateral_separation = abs(radar_data.leadTwo.yRel - radar_data.leadOne.yRel)
          if lateral_separation < 5.0:  # Vehicles close laterally too
            risk = max(risk, 0.5)
    return risk

  def _assess_surrounding_vehicle_tracks_risk(self, radar_data) -> float:
    risk = 0.0
    if hasattr(radar_data, 'tracks') and len(radar_data.tracks) > 0:
      for track in radar_data.tracks:
        if track.status and track.dRel < 60.0:  # Within 60m
          # Calculate potential collision time for this track
          relative_speed_track = track.vRel if hasattr(track, 'vRel') else 0
          if relative_speed_track > 0.5:  # Approaching
            time_to_collision_track = track.dRel / relative_speed_track if relative_speed_track > 0.5 else float('inf')
            if time_to_collision_track < 2.5:
              risk = max(risk, 0.3)
    return risk

  def _assess_dense_traffic_risk(self, radar_data) -> float:
    risk = 0.0
    close_vehicles = 0
    for lead in [radar_data.leadOne, radar_data.leadTwo]:
      if lead.status and lead.dRel < 30.0:
        close_vehicles += 1
    if close_vehicles >= 2:
      risk = max(risk, 0.4)  # High density traffic increases risk
    return risk

  def _assess_environmental_risk(self, radar_data, environment_data) -> float:
    """
    Assess risk from environmental factors
    """
    risk = 0.0

    if radar_data:
      try:
        risk = max(risk, self._assess_lead_vehicle_risk(radar_data))
        risk = max(risk, self._assess_lead_deceleration_risk(radar_data))
        risk = max(risk, self._assess_multiple_vehicle_risk(radar_data))
        risk = max(risk, self._assess_surrounding_vehicle_tracks_risk(radar_data))
        risk = max(risk, self._assess_dense_traffic_risk(radar_data))

        # Enhanced risk assessment: Traffic pattern analysis
        risk = max(risk, self._assess_traffic_pattern_risk(radar_data))

      except AttributeError as e:
        cloudlog.debug(f"Radar data attribute error in risk assessment: {e}")

    # 3. Environmental factors if available
    if environment_data:
      # This would include weather, visibility, traffic density data
      # For now, placeholder
      risk = max(risk, self._assess_environment_data_risk(environment_data))

    return risk

  def _assess_traffic_pattern_risk(self, radar_data) -> float:
    """
    Assess risk based on traffic patterns and flow
    """
    risk = 0.0

    # Assess traffic flow consistency
    if radar_data.leadOne.status and radar_data.leadTwo.status:
      # Calculate relative speeds and positions to detect dangerous patterns
      rel_speed_diff = abs(radar_data.leadOne.vRel - radar_data.leadTwo.vRel)
      distance_diff = abs(radar_data.leadTwo.dRel - radar_data.leadOne.dRel)

      # High relative speed difference with close distance creates dangerous situation
      if distance_diff < 30.0 and rel_speed_diff > 10.0:
        risk = max(risk, 0.6)  # High risk traffic pattern

      # Check for potential cut-in situations
      if (distance_diff < 15.0 and
          radar_data.leadOne.vRel > radar_data.leadTwo.vRel + 5.0 and  # First lead is pulling away from second
          radar_data.leadTwo.dRel < 50.0):  # Both leads are close
        risk = max(risk, 0.5)  # Potential lane change/cut-in risk

    return risk

  def _assess_environment_data_risk(self, environment_data) -> float:
    """
    Assess risk from environment data (weather, visibility, etc.)
    """
    risk = 0.0

    # Placeholder for environment data processing
    # Would typically include weather data, visibility metrics, etc.
    # For now, we'll look for any environmental risk indicators
    if hasattr(environment_data, 'weatherRisk') and environment_data.weatherRisk > 0.7:
      risk = max(risk, environment_data.weatherRisk * 0.6)

    if hasattr(environment_data, 'visibility') and environment_data.visibility < 0.3:  # Poor visibility
      risk = max(risk, 0.4)

    return risk

  def _check_system_faults(self, car_state: car.CarState) -> float:
    risk = 0.0
    if hasattr(car_state, 'steerFaultTemporary') and car_state.steerFaultTemporary:
      risk = max(risk, 0.7)
    if hasattr(car_state, 'steerFaultPermanent') and car_state.steerFaultPermanent:
      risk = max(risk, 0.9)
    return risk

  def _check_unusual_sensor_readings(self, car_state: car.CarState) -> float:
    risk = 0.0
    if hasattr(car_state, 'steeringAngleDeg'):
      if abs(car_state.steeringAngleDeg) > 60:  # Extreme angle
        risk = max(risk, 0.3)
    return risk

  def _check_unstable_vehicle_dynamics(self, car_state: car.CarState) -> float:
    risk = 0.0
    if hasattr(car_state, 'aEgo') and hasattr(car_state, 'vEgo'):
      if car_state.vEgo > 10 and abs(car_state.aEgo) > 4.0:  # High acceleration/deceleration
        risk = max(risk, 0.4)
    return risk

  def _check_sensor_inconsistencies(self, car_state: car.CarState) -> float:
    risk = 0.0
    if (hasattr(car_state, 'vEgo') and hasattr(car_state, 'vCruise') and
        car_state.vEgo > car_state.vCruise + 15.0):  # Significant difference between ego and cruise speed
      risk = max(risk, 0.5)
    return risk

  def _check_pedal_inconsistencies(self, car_state: car.CarState) -> float:
    risk = 0.0
    if (hasattr(car_state, 'brakePressed') and hasattr(car_state, 'gasPressed') and
        car_state.brakePressed and car_state.gasPressed):  # Both pressed simultaneously
      risk = max(risk, 0.6)
    return risk

  def _check_wheel_speed_inconsistencies(self, car_state: car.CarState) -> float:
    risk = 0.0
    if hasattr(car_state, 'wheelSpeeds') and hasattr(car_state, 'vEgo'):
      avg_wheel_speed = 0.0
      valid_wheel_speeds = 0
      if hasattr(car_state.wheelSpeeds, 'fl') and car_state.wheelSpeeds.fl > 0:
        avg_wheel_speed += car_state.wheelSpeeds.fl
        valid_wheel_speeds += 1
      if hasattr(car_state.wheelSpeeds, 'fr') and car_state.wheelSpeeds.fr > 0:
        avg_wheel_speed += car_state.wheelSpeeds.fr
        valid_wheel_speeds += 1
      if hasattr(car_state.wheelSpeeds, 'rl') and car_state.wheelSpeeds.rl > 0:
        avg_wheel_speed += car_state.wheelSpeeds.rl
        valid_wheel_speeds += 1
      if hasattr(car_state.wheelSpeeds, 'rr') and car_state.wheelSpeeds.rr > 0:
        avg_wheel_speed += car_state.wheelSpeeds.rr
        valid_wheel_speeds += 1

      if valid_wheel_speeds > 0:
        avg_wheel_speed /= valid_wheel_speeds
        speed_diff = abs(avg_wheel_speed - car_state.vEgo)
        if speed_diff > 5.0 and car_state.vEgo > 5.0:  # Significant difference at higher speeds
          risk = max(risk, 0.4)
    return risk

  def _check_steering_rate_inconsistencies(self, car_state: car.CarState) -> float:
    risk = 0.0
    if (hasattr(car_state, 'steeringRateDeg') and hasattr(car_state, 'steeringTorque') and
        abs(car_state.steeringRateDeg) > 100 and abs(car_state.steeringTorque) < 10):  # High rate, low torque
      risk = max(risk, 0.3)
    return risk

  def _assess_vehicle_health(self, car_state: car.CarState) -> float:
    """
    Assess health of vehicle systems
    """
    risk = 0.0

    risk = max(risk, self._check_system_faults(car_state))
    risk = max(risk, self._check_unusual_sensor_readings(car_state))
    risk = max(risk, self._check_unstable_vehicle_dynamics(car_state))
    risk = max(risk, self._check_sensor_inconsistencies(car_state))
    risk = max(risk, self._check_pedal_inconsistencies(car_state))
    risk = max(risk, self._check_wheel_speed_inconsistencies(car_state))
    risk = max(risk, self._check_steering_rate_inconsistencies(car_state))

    return risk

  def _calculate_risk_trend(self) -> float:
    """
    Calculate trend of risk over time (positive = increasing risk)
    """
    if len(self.risk_history) < 5:
      return 0.0

    recent_values = list(self.risk_history)[-5:]
    # Simple linear trend calculation
    x = list(range(len(recent_values)))
    y = recent_values

    if len(set(y)) == 1:  # All values are the same
      return 0.0

    # Calculate slope using simple method
    n = len(x)
    if n > 1:
      slope = (n * sum(x[i] * y[i] for i in range(n)) - sum(x) * sum(y)) / (n * sum(xi**2 for xi in x) - sum(x)**2)
      return float(slope)
    return 0.0

  def get_graduated_safety_response(self, risk_level: float) -> dict:
    """
    Get graduated safety response based on risk level

    Args:
      risk_level: Risk level from 0.0 to 1.0

    Returns:
      dict: Response configuration with appropriate safety measures
    """
    for level_name, config in self.risk_assessment_levels.items():
      min_score, max_score = config['score_range']
      if min_score <= risk_level < max_score:
        response_type = config['response']

        response = {
          'level': level_name,
          'response_type': response_type,
          'disengage': response_type == 'disengage',
          'reduce_controls': response_type in ['warning', 'disengage'],
          'increase_monitoring': response_type in ['moderate', 'high', 'critical'],
          'alert_user': response_type in ['moderate', 'warning', 'disengage'],
          'reduce_speed': response_type in ['high', 'critical'],
          'increase_gap': response_type in ['warning', 'critical']
        }
        return response

    # Default to critical if above all ranges
    return {
      'level': 'critical',
      'response_type': 'disengage',
      'disengage': True,
      'reduce_controls': True,
      'increase_monitoring': True,
      'alert_user': True,
      'reduce_speed': True,
      'increase_gap': True
    }

  def reset_safety_state(self):
    """Reset safety violation counters and state."""
    self.safety_violation_count = 0.0
    self._initialized = False
