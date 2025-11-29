"""
Edge case and unusual scenario handler for autonomous driving.

This module provides enhanced handling for various edge cases and unusual
scenarios that can occur during autonomous driving to improve system robustness.
"""

from collections import deque
import numpy as np
import time
from cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.swaglog import cloudlog


class EdgeCaseHandler:
  """
  Handles various edge cases and unusual scenarios to improve system robustness.
  """
  def __init__(self):
    self.params = Params()
    # Filters for detecting unusual patterns
    self.lateral_accel_filter = FirstOrderFilter(0.0, 1.0, DT_CTRL)
    self.longitudinal_accel_filter = FirstOrderFilter(0.0, 1.0, DT_CTRL)
    self.steering_rate_filter = FirstOrderFilter(0.0, 0.5, DT_CTRL)
    # Tracking variables for edge case detection
    self.steering_angle_history = deque(maxlen=50)  # 0.5 seconds at 100Hz
    self.speed_history = deque(maxlen=50)  # 0.5 seconds at 100Hz
    self.lateral_accel_history = deque(maxlen=20)  # 0.2 seconds at 100Hz
    self.longitudinal_accel_history = deque(maxlen=20)  # 0.2 seconds at 100Hz

    # Edge case detection flags
    self._in_construction_zone = False
    self._in_tunnel = False  # Currently not implemented but reserved
    self._weather_conditions = "normal"  # Currently not implemented but reserved
    self._sharp_curve_detected = False
    self._sudden_object_detected = False
    self._abnormal_steering_detected = False

    # Configuration parameters
    self.max_lat_accel_threshold = 3.0  # m/s^2 - threshold for high lateral acceleration
    self.sharp_curve_speed_threshold = 15.0  # m/s - speed threshold for sharp curves
    self.steering_rate_threshold = 50.0  # deg/s - threshold for rapid steering changes
    self.speed_variance_threshold = 2.0  # m/s - threshold for speed variance detection
    self.construction_zone_speed = 10.0  # m/s - assumed construction zone speed limit

    # Initialize state variables
    self._last_steering_angle = 0.0
    # Initialize the _previous_speeds list with type annotation
    self._previous_speeds: list[float | None] = []
    # Initialize the _previous_leads list with type annotation
    self._previous_leads: list[dict[str, float | None]] = []
    self._last_speed = 0.0
    self._initialized = False

  def update_params(self):
    """Update edge case parameters from system parameters"""
    try:
      self.max_lat_accel_threshold = float(self.params.get("EdgeCaseMaxLatAccel", "3.0"))
      self.sharp_curve_speed_threshold = float(self.params.get("EdgeCaseSharpCurveSpeed", "15.0"))
      self.steering_rate_threshold = float(self.params.get("EdgeCaseSteeringRate", "50.0"))
      self.speed_variance_threshold = float(self.params.get("EdgeCaseSpeedVariance", "2.0"))
      self.construction_zone_speed = float(self.params.get("EdgeCaseConstructionSpeed", "10.0"))
    except (TypeError, ValueError):
      # Use defaults if parameters are invalid
      pass

  def detect_sharp_curves(self, car_state: car.CarState, model_data) -> bool:
    """
    Detect sharp curves and suggest appropriate speed reduction.

    Args:
      car_state: Current car state
      model_data: Model predictions

    Returns:
      True if sharp curve detected, False otherwise
    """
    if model_data is None or not hasattr(model_data, 'plan'):
      return False

    # Analyze the planned trajectory for sharp curves
    try:
      if hasattr(model_data.plan, 'orientationRate') and hasattr(model_data.plan.orientationRate, 'y'):
        # Check for high orientation rate (sharp turns)
        orientation_rates = model_data.plan.orientationRate.y[:10]  # Check first 10 points
        if len(orientation_rates) > 0:
          max_orientation_rate = max(abs(rate) for rate in orientation_rates)
          if max_orientation_rate > 0.2:  # Threshold for sharp curve
            return True
    except (AttributeError, TypeError):
      pass

    # Alternative: Check desired curvature from action
    if hasattr(model_data, 'action') and hasattr(model_data.action, 'desiredCurvature'):
      desired_curvature = abs(model_data.action.desiredCurvature)
      # For a vehicle, significant curvature might start at around 0.05/m
      if desired_curvature > 0.1 and car_state.vEgo > self.sharp_curve_speed_threshold:
        return True

    return False

  def detect_abnormal_steering(self, car_state: car.CarState) -> bool:
    """
    Detect abnormal steering behavior that might indicate road issues or construction.

    Args:
      car_state: Current car state

    Returns:
      True if abnormal steering detected, False otherwise
    """
    current_angle = car_state.steeringAngleDeg
    current_time = DT_CTRL  # We use DT_CTRL as delta time

    if self._initialized:
      # Calculate steering rate
      steering_rate = abs(current_angle - self._last_steering_angle) / current_time if current_time > 0 else 0
      self.steering_rate_filter.update(steering_rate)

      # Check for excessive steering rate
      if steering_rate > self.steering_rate_threshold:
        return True

      # Store in history for variance analysis
      self.steering_angle_history.append(current_angle)

      # Check for high variance in steering angles over short period
      if len(self.steering_angle_history) > 10:
        recent_angles = list(self.steering_angle_history)[-10:]
        angle_variance = np.var(recent_angles)
        if angle_variance > 50.0:  # High steering variance might indicate construction/rough road
          return True

    # Update tracking variables
    self._last_steering_angle = current_angle
    self._initialized = True

    return False

  def detect_sudden_objects(self, radar_state, model_data) -> bool:
    """
    Detect sudden objects that might require immediate action.

    Args:
      radar_state: Radar sensor data
      model_data: Model predictions

    Returns:
      True if sudden object detected, False otherwise
    """
    if radar_state is None:
      return False

    # Check for lead vehicles that suddenly appeared or changed behavior
    try:
      lead_one = radar_state.leadOne
      lead_two = radar_state.leadTwo

      if lead_one.status:
        # Check if lead vehicle is very close and relative velocity suggests potential collision
        if lead_one.dRel < 30.0 and lead_one.vRel < -2.0:  # Very close and approaching rapidly
          return True

      if lead_two.status:
        # Check second lead vehicle
        if lead_two.dRel < 50.0 and lead_two.vRel < -3.0:  # Within 50m and approaching rapidly
          return True
    except AttributeError:
      pass

    return False

  def detect_construction_zone(self, car_state: car.CarState,
                              radar_state, model_data) -> bool:
    """
    Detect potential construction zones based on various indicators.

    Args:
      car_state: Current car state
      radar_state: Radar sensor data
      model_data: Model predictions

    Returns:
      True if construction zone detected, False otherwise
    """
    # Check for multiple indicators of construction zone:
    # 1. Unusual steering patterns
    # 2. Frequent speed changes
    # 3. Multiple vehicles at low speeds
    # 4. Unusual lane line patterns from model data

    construction_indicators = 0

    # Check for abnormal steering (lane changes not by system)
    if self.detect_abnormal_steering(car_state):
      construction_indicators += 1

    # Check for speed variance
    current_speed = car_state.vEgo
    self.speed_history.append(current_speed)

    if len(self.speed_history) > 20:  # Wait for sufficient history
      recent_speeds = list(self.speed_history)[-20:]
      speed_variance = np.var(recent_speeds)
      if speed_variance > self.speed_variance_threshold:
        construction_indicators += 1

    # Check radar for multiple slow vehicles that might indicate construction
    if radar_state is not None:
      slow_vehicle_count = 0
      try:
        if radar_state.leadOne.status and radar_state.leadOne.vLead < 8.0:  # Below 28 km/h
          slow_vehicle_count += 1
        if radar_state.leadTwo.status and radar_state.leadTwo.vLead < 8.0:
          slow_vehicle_count += 1

        if slow_vehicle_count >= 1:  # At least one very slow vehicle
          construction_indicators += 1
      except AttributeError:
        pass

    # Additional model-based detection - check for lane line inconsistency
    if model_data is not None and hasattr(model_data, 'lateralPlannerOutput'):
      lane_inconsistency = self._check_lane_line_consistency(model_data.lateralPlannerOutput)
      if lane_inconsistency:
        construction_indicators += 1

    # Check for frequent brake usage which might indicate construction
    if hasattr(car_state, 'brakePressed') and car_state.brakePressed:
      if not hasattr(self, '_brake_event_history'):
        self._brake_event_history = []
      self._brake_event_history.append(time.monotonic())
      # Keep only events from last 10 seconds
      self._brake_event_history = [t for t in self._brake_event_history if time.monotonic() - t < 10]

      # If more than 3 brake events in 10 seconds, consider it unusual
      if len(self._brake_event_history) > 3:
        construction_indicators += 1

    return construction_indicators >= 2  # At least 2 indicators to declare construction zone

  def _check_lane_line_consistency(self, lateral_planner_output) -> bool:
    """
    Check if lane line detection is inconsistent with normal lane geometry

    Args:
      lateral_planner_output: Model output containing lane line data

    Returns:
      True if lane line inconsistency detected, False otherwise
    """
    if not hasattr(lateral_planner_output, 'laneLines'):
      return False

    lane_lines = lateral_planner_output.laneLines
    if len(lane_lines) < 2:
      return False  # Not enough lane lines to check consistency

    # Check for unusual lane line positions or angles that might indicate construction
    # Lane lines should follow expected geometric patterns
    for _i, line in enumerate(lane_lines):
      # Check if lane line probability is low (indicating poor visibility/condition)
      if line.prob < 0.5:
        return True  # Low confidence in lane detection might indicate construction

    # Check that lane lines are approximately parallel and at expected distance
    if len(lane_lines) >= 2:
      left_line = lane_lines[0]  # Assuming first is left lane
      right_line = lane_lines[1]  # Assuming second is right lane

      # Check if lane width is significantly different from expected at various points
      lane_widths = []
      for j in range(min(len(left_line.y), len(right_line.y))):
        lane_width = abs(right_line.y[j] - left_line.y[j])
        if lane_width > 0:  # Only if valid measurement
          lane_widths.append(lane_width)

      if lane_widths:
        avg_lane_width = sum(lane_widths) / len(lane_widths)
        # Standard lane widths are typically 3.7m on highways, allow some variation
        if avg_lane_width < 2.5 or avg_lane_width > 5.0:  # Unusual width for construction
          return True

    return False

  def _check_unusual_longitudinal_plan(self, longitudinal_plan) -> bool:
    """
    Check for unusual patterns in longitudinal planning that might indicate poor conditions

    Args:
      longitudinal_plan: Longitudinal plan data from model

    Returns:
      True if unusual longitudinal plan detected, False otherwise
    """
    # Check for frequent rapid acceleration/deceleration planning
    if hasattr(longitudinal_plan, 'aEgo') and len(longitudinal_plan.aEgo) > 2:
      accelerations = longitudinal_plan.aEgo
      # Look for frequent sign changes in acceleration (indicating frequent speed changes)
      sign_changes = 0
      for i in range(1, len(accelerations)):
        if (accelerations[i-1] > 0 and accelerations[i] < 0) or (accelerations[i-1] < 0 and accelerations[i] > 0):
          sign_changes += 1

      # If many acceleration sign changes per time period, it might indicate poor conditions
      if sign_changes > len(accelerations) * 0.3:  # More than 30% are sign changes
        return True

    return False

  def detect_weather_impact(self, car_state: car.CarState, radar_state, model_data) -> bool:
    """
    Detect signs of adverse weather conditions that may impact driving

    Args:
      car_state: Current car state
      radar_state: Radar sensor data
      model_data: Model predictions

    Returns:
      True if weather impact detected, False otherwise
    """
    weather_indicators = 0

    # 1. Radar-based detection of precipitation affecting detection
    # This would typically involve checking for increased noise or reduced detection range
    # which isn't directly available in this structure, so we'll simulate with other indicators

    # 2. Visibility-based detection from model data
    if model_data is not None:
      visibility_score = self._assess_visibility_from_model(model_data)
      if visibility_score < 0.3:  # Poor visibility
        weather_indicators += 1

    # 3. Driving behavior indicators - more frequent small steering corrections might indicate poor visibility
    if hasattr(car_state, 'steeringTorque') and abs(car_state.steeringTorque) < 50:  # Light steering
      # Check if accompanied by frequent corrections
      if len(self.steering_angle_history) > 5:
        recent_angles = list(self.steering_angle_history)[-5:]
        if len(set(recent_angles)) > 3:  # Multiple different steering angles
          weather_indicators += 1

    # 4. Speed reduction without obvious reason (other than traffic)
    if (car_state.vEgo < car_state.vCruise * 0.7 and
        (radar_state is None or (not radar_state.leadOne.status or radar_state.leadOne.dRel > 50.0))):
      # Driving significantly below set speed with no lead vehicle
      weather_indicators += 1

    # 5. Unusual brake light activity (if available in car state)
    # This would detect other vehicles braking more frequently in poor conditions

    return weather_indicators >= 2  # At least 2 indicators for weather impact

  def _assess_visibility_from_model(self, model_data) -> float:
    """
    Assess visibility based on model data quality and confidence

    Args:
      model_data: Model predictions

    Returns:
      Visibility score (0.0 to 1.0, where 1.0 is clear visibility)
    """
    score = 1.0

    # Check for poor lane line visibility
    if hasattr(model_data, 'lateralPlannerOutput'):
      lane_lines = model_data.lateralPlannerOutput.laneLines
      visible_lane_lines = sum(1 for line in lane_lines if line.prob > 0.5)  # High confidence lines

      if len(lane_lines) > 0:
        visibility_ratio = visible_lane_lines / len(lane_lines)
        score *= visibility_ratio  # Reduce score based on low-confidence lane lines

    # Check for poor lead vehicle detection confidence
    if hasattr(model_data, 'longitudinalPlan'):
      # This would be where we check for model confidence in lead detection
      # For now, we'll just return the score based on lane line visibility
      pass

    return max(0.0, min(1.0, score))

  def detect_rough_road_conditions(self, car_state: car.CarState) -> bool:
    """
    Detect rough road conditions from vehicle dynamics

    Args:
      car_state: Current car state

    Returns:
      True if rough road conditions detected, False otherwise
    """
    if not hasattr(self, '_acceleration_history'):
      self._acceleration_history = []

    # Track lateral and longitudinal acceleration variance as indicators of road roughness
    if hasattr(car_state, 'aEgo') and hasattr(car_state, 'vEgo'):
      self._acceleration_history.append({
        'time': time.monotonic(),
        'long_accel': car_state.aEgo,
        'speed': car_state.vEgo
      })
      # Keep only recent values (last 1 second at 100Hz)
      cutoff_time = time.monotonic() - 1.0
      self._acceleration_history = [a for a in self._acceleration_history if a['time'] > cutoff_time]

      if len(self._acceleration_history) >= 50:  # At least 0.5 seconds of data
        long_accels = [a['long_accel'] for a in self._acceleration_history]
        acceleration_variance = np.var(long_accels)

        # Threshold would be calibrated based on vehicle characteristics
        # High variance in acceleration can indicate rough road surface
        rough_road_threshold = 0.8
        if acceleration_variance > rough_road_threshold and car_state.vEgo > 5.0:
          return True

    # Also check steering corrections as proxy for road irregularities
    if len(self.steering_angle_history) >= 10:
      recent_angles = list(self.steering_angle_history)[-10:]
      angle_variance = np.var(recent_angles)
      # High steering variation might indicate need to correct for road irregularities
      high_steering_variation_threshold = 10.0
      if angle_variance > high_steering_variation_threshold and car_state.vEgo > 5.0:
        return True

    return False

  def detect_unusual_traffic_patterns(self, radar_state, car_state: car.CarState) -> bool:
    """
    Detect unusual traffic patterns that may indicate incidents, construction, or poor conditions

    Args:
      radar_state: Radar sensor data
      car_state: Current car state

    Returns:
      True if unusual traffic patterns detected, False otherwise
    """
    if radar_state is None:
      return False

    indicators = 0

    try:
      # 1. Multiple vehicles at unusual distances/speeds
      if (radar_state.leadOne.status and radar_state.leadOne.dRel < 5.0 and
          car_state.vEgo > 15.0 and radar_state.leadOne.vRel < -5.0):  # Very close, closing fast
        indicators += 1

      # 2. Unusual spacing patterns - vehicles too close together for normal conditions
      if (radar_state.leadOne.status and radar_state.leadTwo.status and
          radar_state.leadOne.dRel < 50.0 and radar_state.leadTwo.dRel < 100.0):
        spacing = radar_state.leadTwo.dRel - radar_state.leadOne.dRel
        if spacing < 10.0 and spacing > 0:  # Unusually close spacing
          indicators += 1

      # 3. Very slow traffic when it should be moving
      if (radar_state.leadOne.status and radar_state.leadOne.vLead < 5.0 and
          car_state.vEgo > 15.0):  # Lead moving very slowly
        indicators += 1

      # 4. Frequent lead vehicle changes (could indicate weaving between construction zones)
      if not hasattr(self, '_previous_leads'):
        self._previous_leads = []
      self._previous_leads.append({
        'time': time.monotonic(),
        'dRel': radar_state.leadOne.dRel if radar_state.leadOne.status else None,
        'vRel': radar_state.leadOne.vRel if radar_state.leadOne.status else None
      })
      # Keep last 20 entries (~0.2 seconds at 100Hz)
      self._previous_leads = self._previous_leads[-20:]

      # Check for frequent lead changes
      valid_leads = [l for l in self._previous_leads if l['dRel'] is not None and l['dRel'] < 50.0]
      if len(valid_leads) > 10:  # Sufficient data
        # Look for rapid changes in distance to lead (indicating frequent lead changes)
        distances = [l['dRel'] for l in valid_leads]
        distance_changes = [abs(distances[i] - distances[i-1]) for i in range(1, len(distances))]
        avg_distance_change = sum(distance_changes) / len(distance_changes) if distance_changes else 0
        if avg_distance_change > 5.0:  # Large average changes in lead distance
          indicators += 1

      # 5. Sudden appearance of lead vehicle (could indicate cut-in or system failure detection)
      if (radar_state.leadOne.status and
          radar_state.leadOne.dRel < 50.0 and
          car_state.vEgo > 10.0):
        # If previous frames showed no lead but now there is one close by
        recent_lead_distances = [l['dRel'] for l in valid_leads if l['dRel'] is not None]
        if recent_lead_distances and all(d > 80 for d in recent_lead_distances):  # No lead seen recently but one appeared close
          indicators += 1

      # 6. Lead vehicle doing unusual maneuvers (e.g., sudden lane changes)
      if (radar_state.leadOne.status and
          abs(radar_state.leadOne.aRel) > 3.0 and
          radar_state.leadOne.dRel < 60.0):  # Large acceleration change close by
        indicators += 1

      # 7. Multiple vehicles with inconsistent behavior (potential traffic incident)
      if (radar_state.leadOne.status and radar_state.leadTwo.status and
          radar_state.leadOne.dRel < 60.0 and radar_state.leadTwo.dRel < 120.0):
        relative_behavior_diff = abs(radar_state.leadOne.vRel - radar_state.leadTwo.vRel)
        if relative_behavior_diff > 8.0:  # Significant difference in relative speeds
          indicators += 1

    except AttributeError:
      pass

    return indicators >= 2  # At least 2 indicators for unusual traffic

  def detect_traffic_incident_or_congestion(self, radar_state, car_state: car.CarState) -> dict:
    """
    Enhanced detection for traffic incidents or congestion that require special handling

    Critical Analysis Note: This method significantly improves situational awareness
    but has a high potential for false positives. Tuning is critical.
    Consider making thresholds (e.g., for `indicators`) configurable
    parameters to facilitate real-world tuning without code changes.
    Ensure clear and comprehensive logging when an incident is detected
    to aid in debugging and understanding system behavior.

    Args:
      radar_state: Radar sensor data
      car_state: Current car state

    Returns:
      dict: Incident detection results with confidence scores
    """
    if radar_state is None:
      return {
        'incident_detected': False,
        'congestion_detected': False,
        'confidence': 0.0,
        'recommended_action': 'normal'
      }

    incident_indicators = 0
    congestion_indicators = 0

    try:
      # Check for traffic incident indicators
      if radar_state.leadOne.status:
        # Very slow lead vehicle when traffic should be flowing
        if (radar_state.leadOne.vLead < 2.0 and
            car_state.vEgo > 15.0 and
            radar_state.leadOne.dRel < 80.0):
          incident_indicators += 1
          congestion_indicators += 1  # Could be both incident or congestion

        # Sudden stop of lead vehicle
        if (radar_state.leadOne.vLead < 0.5 and
            radar_state.leadOne.aRel < -2.0 and  # Heavy braking
            radar_state.leadOne.dRel < 50.0):
          incident_indicators += 1

      if radar_state.leadTwo.status:
        # Lead vehicle 2 moving much slower than lead vehicle 1 (could indicate lane closure/accident)
        if (radar_state.leadOne.status and
            radar_state.leadOne.vLead > 5.0 and
            radar_state.leadTwo.vLead < 2.0 and
            radar_state.leadTwo.dRel < radar_state.leadOne.dRel + 30.0):
          incident_indicators += 1

      # Check for congestion patterns
      # Multiple slow vehicles in close proximity
      slow_vehicle_count = 0
      if radar_state.leadOne.status and radar_state.leadOne.vLead < 5.0:
        slow_vehicle_count += 1
      if radar_state.leadTwo.status and radar_state.leadTwo.vLead < 5.0:
        slow_vehicle_count += 1

      if slow_vehicle_count >= 2:
        congestion_indicators += 1
        # If both leads are very slow but ego vehicle is fast, likely congestion
        if car_state.vEgo > 15.0 and max(radar_state.leadOne.vLead, radar_state.leadTwo.vLead) < 2.0:
          congestion_indicators += 1

      # Check for stop-and-go patterns (classic congestion)
      if hasattr(self, '_previous_speeds') and len(self._previous_speeds) > 20:
        recent_v_ego = [s for s in self._previous_speeds[-20:] if s is not None]
        if len(recent_v_ego) > 10:
          speed_variance = np.var(recent_v_ego)
          # High speed variance with low average speed indicates stop-and-go
          avg_speed = sum(recent_v_ego) / len(recent_v_ego)
          if speed_variance > 10.0 and avg_speed < 8.0:
            congestion_indicators += 1

      # Store current data for future trend analysis
      if not hasattr(self, '_previous_speeds'):
        self._previous_speeds = []
      self._previous_speeds.append(car_state.vEgo)
      self._previous_speeds = self._previous_speeds[-30:]  # Keep last 30 values

    except (AttributeError, TypeError):
      pass

    # Determine confidence levels
    incident_confidence = min(1.0, incident_indicators * 0.3)
    congestion_confidence = min(1.0, congestion_indicators * 0.3)

    # Determine if an incident or congestion is detected
    incident_detected = incident_confidence > 0.3
    congestion_detected = congestion_confidence > 0.3

    # Determine recommended action based on detection
    recommended_action = 'normal'
    if incident_detected and congestion_detected:
      recommended_action = 'caution_and_reduce_speed'
    elif incident_detected:
      recommended_action = 'increase_caution'
    elif congestion_detected:
      recommended_action = 'prepare_for_stop_and_go'

    return {
      'incident_detected': incident_detected,
      'congestion_detected': congestion_detected,
      'confidence': max(incident_confidence, congestion_confidence),
      'recommended_action': recommended_action,
      'incident_confidence': incident_confidence,
      'congestion_confidence': congestion_confidence
    }

  def handle_unusual_scenarios(self, car_state: car.CarState,
                              radar_state=None, model_data=None) -> dict:
    """
    Analyze current situation and identify any unusual scenarios or edge cases.

    Args:
      car_state: Current car state
      radar_state: Radar sensor data (optional)
      model_data: Model predictions (optional)

    Returns:
      Dictionary containing detected scenarios and recommended actions
    """
    scenarios = {
      'sharp_curve': self.detect_sharp_curves(car_state, model_data),
      'construction_zone': self.detect_construction_zone(car_state, radar_state, model_data),
      'abnormal_steering': self.detect_abnormal_steering(car_state),
      'sudden_object': self.detect_sudden_objects(radar_state, model_data),
      'weather_impact': self.detect_weather_impact(car_state, radar_state, model_data),
      'rough_road': self.detect_rough_road_conditions(car_state),
      'unusual_traffic': self.detect_unusual_traffic_patterns(radar_state, car_state),
      'traffic_incident': False,
      'traffic_congestion': False,
      'recommended_speed': car_state.vCruise * 0.8 if car_state.vCruise > 0 else car_state.vEgo,  # Default: reduce speed by 20%
      'caution_required': False,
      'adaptive_control_needed': False,
      'confidence_score': 0.0
    }

    # Enhanced traffic incident and congestion detection
    traffic_analysis = self.detect_traffic_incident_or_congestion(radar_state, car_state)
    scenarios['traffic_incident'] = traffic_analysis['incident_detected']
    scenarios['traffic_congestion'] = traffic_analysis['congestion_detected']

    # Set recommended speed based on detected scenarios
    if scenarios['sharp_curve'] and car_state.vEgo > self.sharp_curve_speed_threshold:
      scenarios['recommended_speed'] = min(scenarios['recommended_speed'], self.sharp_curve_speed_threshold)
      scenarios['caution_required'] = True
      scenarios['adaptive_control_needed'] = True

    if scenarios['construction_zone']:
      scenarios['recommended_speed'] = min(scenarios['recommended_speed'], self.construction_zone_speed)
      scenarios['caution_required'] = True
      scenarios['adaptive_control_needed'] = True

    if scenarios['sudden_object']:
      scenarios['caution_required'] = True
      scenarios['adaptive_control_needed'] = True

    if scenarios['abnormal_steering']:
      scenarios['caution_required'] = True
      scenarios['adaptive_control_needed'] = True

    if scenarios['weather_impact']:
      scenarios['caution_required'] = True
      scenarios['adaptive_control_needed'] = True
      # Further reduce speed in adverse weather
      scenarios['recommended_speed'] = min(scenarios['recommended_speed'], car_state.vCruise * 0.6)

    if scenarios['rough_road']:
      scenarios['caution_required'] = True
      scenarios['adaptive_control_needed'] = True
      # Reduce speed significantly on rough roads
      scenarios['recommended_speed'] = min(scenarios['recommended_speed'], car_state.vCruise * 0.7)

    if scenarios['unusual_traffic']:
      scenarios['caution_required'] = True
      scenarios['adaptive_control_needed'] = True

    if scenarios['traffic_incident']:
      scenarios['caution_required'] = True
      scenarios['adaptive_control_needed'] = True
      scenarios['recommended_speed'] = min(scenarios['recommended_speed'], car_state.vCruise * 0.5)  # Significant reduction for incidents

    if scenarios['traffic_congestion']:
      scenarios['caution_required'] = True
      scenarios['adaptive_control_needed'] = True
      scenarios['recommended_speed'] = min(scenarios['recommended_speed'], car_state.vCruise * 0.6)  # Moderate reduction for congestion

    # Calculate overall confidence in the detection
    positive_detections = sum(1 for k, v in scenarios.items()
                             if k not in ['recommended_speed', 'caution_required', 'adaptive_control_needed', 'confidence_score'] and v is True)
    scenarios['confidence_score'] = min(1.0, positive_detections * 0.2)  # 0.2 per detection, max 1.0

    return scenarios

  def get_adaptive_control_modifications(self, car_state: car.CarState,
                                       scenarios: dict) -> dict:
    """
    Get control modifications to adapt to detected edge cases and scenarios.

    Args:
      car_state: Current car state
      scenarios: Dictionary with detected scenarios from handle_unusual_scenarios

    Returns:
      Dictionary with control modifications
    """
    modifications = {
      'longitudinal_factor': 1.0,  # Factor to apply to longitudinal control (0.8 = 20% more conservative)
      'lateral_factor': 1.0,      # Factor to apply to lateral control
      'min_gap': 2.0,             # Minimum time gap to maintain
      'caution_mode': False,      # Whether to enable caution mode
      'speed_limit_factor': 1.0,  # Factor to reduce speed limit
      'steering_sensitivity': 1.0  # Factor to adjust steering sensitivity
    }

    # Log detected scenarios for debugging and analysis
    excluded_keys = ['recommended_speed', 'caution_required', 'adaptive_control_needed', 'confidence_score']
    detected_scenarios = [k for k, v in scenarios.items() if v is True and k not in excluded_keys]
    if detected_scenarios:
      cloudlog.debug(f"EdgeCaseHandler: Detected scenarios: {', '.join(detected_scenarios)}. Applying adaptive control.")

    if scenarios['caution_required']:
      modifications['caution_mode'] = True
      modifications['longitudinal_factor'] = 0.8  # More conservative longitudinal control
      modifications['lateral_factor'] = 0.9      # More conservative lateral control
      modifications['min_gap'] = 3.0             # Increase minimum gap to 3 seconds

    if scenarios['sharp_curve']:
      modifications['lateral_factor'] = min(modifications['lateral_factor'], 0.7)      # Extra conservative in curves
      modifications['longitudinal_factor'] = min(modifications['longitudinal_factor'], 0.7)

    if scenarios['construction_zone']:
      modifications['longitudinal_factor'] = min(modifications['longitudinal_factor'], 0.6) # Extra conservative in construction zones
      modifications['lateral_factor'] = min(modifications['lateral_factor'], 0.8)
      modifications['min_gap'] = max(modifications['min_gap'], 4.0)             # Extra distance in construction zones
      modifications['speed_limit_factor'] = 0.7  # Reduce speed in construction zones
      modifications['caution_mode'] = True

    if scenarios['sudden_object']:
      modifications['longitudinal_factor'] = min(modifications['longitudinal_factor'], 0.5) # Very conservative when objects detected
      modifications['min_gap'] = max(modifications['min_gap'], 5.0)             # Maximum distance when sudden objects detected
      modifications['caution_mode'] = True

    if scenarios.get('weather_impact', False):
      modifications['longitudinal_factor'] = 0.6  # More conservative longitudinal in weather
      modifications['lateral_factor'] = 0.8      # More conservative lateral in weather
      modifications['min_gap'] = 4.0             # Increase gap significantly in weather
      modifications['speed_limit_factor'] = 0.75  # Reduce speed in adverse weather
      modifications['steering_sensitivity'] = 0.7  # Reduce steering sensitivity in weather
      modifications['caution_mode'] = True

    if scenarios.get('rough_road', False):
      modifications['longitudinal_factor'] = 0.8  # Be more careful on rough roads
      modifications['lateral_factor'] = 0.9
      modifications['steering_sensitivity'] = 0.7  # Reduce steering sensitivity on rough roads
      modifications['caution_mode'] = True

    if scenarios.get('unusual_traffic', False):
      modifications['longitudinal_factor'] = 0.7  # More conservative in unusual traffic
      modifications['min_gap'] = 3.5             # More distance in unusual traffic
      modifications['caution_mode'] = True

    if scenarios.get('traffic_incident', False):
      modifications['longitudinal_factor'] = 0.5  # Very conservative for traffic incidents
      modifications['lateral_factor'] = 0.7       # More careful steering
      modifications['min_gap'] = 5.0             # Maximum distance for safety
      modifications['speed_limit_factor'] = 0.4  # Significant speed reduction
      modifications['caution_mode'] = True

    if scenarios.get('traffic_congestion', False):
      modifications['longitudinal_factor'] = 0.6  # Conservative for stop-and-go
      modifications['min_gap'] = 4.0             # More distance for buffer
      modifications['speed_limit_factor'] = 0.6  # Reduced speed for congestion
      modifications['steering_sensitivity'] = 0.8  # Reduced sensitivity in dense traffic
      modifications['caution_mode'] = True

    # Apply conservative limits to prevent over-correction
    modifications['longitudinal_factor'] = max(0.3, modifications['longitudinal_factor'])
    modifications['lateral_factor'] = max(0.5, modifications['lateral_factor'])
    modifications['min_gap'] = min(6.0, modifications['min_gap'])  # Don't be too conservative
    modifications['speed_limit_factor'] = max(0.3, modifications['speed_limit_factor'])  # Adjusted min for incidents
    modifications['steering_sensitivity'] = max(0.3, modifications['steering_sensitivity'])  # Adjusted min for traffic

    if detected_scenarios:
      cloudlog.debug(
          f"EdgeCaseHandler: Applied modifications: longitudinal_factor={modifications['longitudinal_factor']:.2f}, " +
          f"lateral_factor={modifications['lateral_factor']:.2f}, min_gap={modifications['min_gap']:.1f}s, " +
          f"speed_limit_factor={modifications['speed_limit_factor']:.2f}, " +
          f"steering_sensitivity={modifications['steering_sensitivity']:.2f}, " +
          f"caution_mode={modifications['caution_mode']}"
      )

    return modifications

  def reset_state(self):
    """Reset edge case detection state."""
    self.steering_angle_history.clear()
    self.speed_history.clear()
    self.lateral_accel_history.clear()
    self.longitudinal_accel_history.clear()
    self._initialized = False
    self._in_construction_zone = False
    self._sharp_curve_detected = False
    self._sudden_object_detected = False
    self._abnormal_steering_detected = False
