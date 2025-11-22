"""
Enhanced Safety Monitoring for sunnypilot - Implements sophisticated safety checks

Phase 1 Improvements (Enhanced Safety Monitoring):
- Advanced Anomaly Detection: Detects velocity inconsistencies, high jerk, high steering rate, and confidence trends
- Enhanced Environmental Condition Detection: Improved lighting, weather, and road condition assessment
- Anomaly-based Safety Penalties: Reduces safety score based on detected anomalies
- Anomaly-based Intervention Logic: Triggers intervention for critical anomalies
- Environmental-aware Confidence Weighting: Adjusts sensor confidence based on environmental conditions

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import logging
from typing import Dict, Tuple, Optional
import time # Import time for performance measurements
from cereal import log
import cereal.messaging as messaging
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.params import Params, UnknownKeyName
from collections import deque


class AdvancedAnomalyDetector:
    """
    Advanced anomaly detection for autonomous driving safety
    """
    def __init__(self):
        self.velocity_variance_threshold = 0.5  # m/s
        self.acceleration_jerk_threshold = 5.0  # m/s³
        self.steering_rate_threshold = 100.0    # deg/s
        self.velocity_buffer = deque(maxlen=10)  # Store last 10 velocity readings for trend analysis
        self.acceleration_buffer = deque(maxlen=10)  # Store last 10 acceleration readings
        self.confidence_buffer = deque(maxlen=10)  # Store last 10 confidence readings for trend analysis

    def detect_anomalies(self, car_state, model_v2, radar_state):
        anomalies = {}

        # Velocity consistency check - compare car state with model prediction
        if len(model_v2.velocity.x) > 0:
            vel_diff = abs(car_state.vEgo - model_v2.velocity.x[0])
            if vel_diff > self.velocity_variance_threshold:
                anomalies['velocity_inconsistency'] = {
                    'car_state_vEgo': car_state.vEgo,
                    'model_v2_velocity': model_v2.velocity.x[0],
                    'difference': vel_diff,
                    'threshold': self.velocity_variance_threshold
                }

        # Acceleration jerk detection
        if len(self.acceleration_buffer) > 0:
            prev_acceleration = self.acceleration_buffer[-1]
            current_jerk = abs(car_state.aEgo - prev_acceleration) / DT_MDL
            if current_jerk > self.acceleration_jerk_threshold:
                anomalies['high_jerk'] = {
                    'current_jerk': current_jerk,
                    'threshold': self.acceleration_jerk_threshold,
                    'current_accel': car_state.aEgo,
                    'prev_accel': prev_acceleration
                }

        # Add current values to buffers
        self.velocity_buffer.append(car_state.vEgo)
        self.acceleration_buffer.append(car_state.aEgo)

        # Steering rate check
        if hasattr(car_state, 'steeringRateDeg') and abs(car_state.steeringRateDeg) > self.steering_rate_threshold:
            anomalies['high_steering_rate'] = {
                'steering_rate': car_state.steeringRateDeg,
                'threshold': self.steering_rate_threshold
            }

        # Model confidence trend analysis
        if hasattr(model_v2, 'meta') and hasattr(model_v2.meta, 'confidence'):
            # Add current confidence to buffer for trend analysis
            self.confidence_buffer.append(model_v2.meta.confidence)
            # Check if confidence is consistently low over recent readings
            recent_confidence_avg = np.mean(list(self.confidence_buffer)) if len(self.confidence_buffer) > 0 else 1.0
            if model_v2.meta.confidence < 0.5 and recent_confidence_avg < 0.6:
                anomalies['low_confidence_trend'] = {
                    'current_confidence': model_v2.meta.confidence,
                    'recent_avg_confidence': recent_confidence_avg
                }

        return anomalies


class EnvironmentalConditionDetector:
    """
    Advanced environmental condition detection using multi-sensor fusion
    """
    def __init__(self):
        self.lighting_threshold = 50  # Arbitrary threshold for low light
        self.weather_sensitivity = 0.7  # Sensitivity for weather detection
        self.road_condition_thresholds = {
            'roughness': 0.5,  # Threshold for bumpy roads based on IMU data
            'slippery': 0.3   # Threshold for potentially slippery conditions
        }
        self.gps_confidence = 0.0
        self.gps_signal_lost = True # Assume lost until proven otherwise

    def detect_conditions(self, model_v2, live_pose, car_state, gps_location_msg):
        conditions = {}
        confidences = {}

        # Advanced lighting detection using modelV2 scene analysis
        # Analyze brightness/contrast features from model if available
        lighting_condition, lighting_confidence = self.assess_lighting_conditions(model_v2, car_state, gps_location_msg)
        conditions['lighting'] = lighting_condition
        confidences['lighting'] = lighting_confidence

        # Road condition assessment using IMU data
        if live_pose is not None:
            road_condition, road_confidence = self.assess_road_condition(
                live_pose.angular_velocity, live_pose.acceleration, car_state
            )
            conditions['road_condition'] = road_condition
            confidences['road_condition'] = road_confidence
        else:
            conditions['road_condition'] = "unknown"
            confidences['road_condition'] = 0.0 # No IMU data means no confidence

        # Weather assessment using multiple indicators
        weather_condition, weather_confidence = self.assess_weather_condition(model_v2, car_state)
        conditions['weather'] = weather_condition
        confidences['weather'] = weather_confidence

        return conditions, confidences

    def assess_lighting_conditions(self, model_v2, car_state_msg, gps_location_msg):
        # Analyze lighting conditions using multi-sensor fusion approach
        # Combining lane line strength, camera brightness metrics, and time-based estimates

        lighting_condition = "normal"  # Default assumption
        lighting_confidence = 0.5  # Base confidence

        # Initialize confidence contribution counters
        valid_indicators = 0
        total_confidence = 0.0

        # Indicator 1: Lane line strength (original approach, weighted less in tunnels)
        lane_line_confidence = 0.0
        if hasattr(model_v2, 'laneLines') and len(model_v2.laneLines) > 0:
            lane_line_strengths = [line.strength for line in model_v2.laneLines if line.strength is not None]
            if len(lane_line_strengths) > 0:
                avg_lane_line_strength = np.mean(lane_line_strengths)

                if avg_lane_line_strength < 0.25:
                    # Do not set lighting_condition to "dark" based solely on lane line strength
                    # lighting_condition = "dark" # Removed this line
                    lane_line_confidence = max(0.1, 1.0 - (0.25 - avg_lane_line_strength) / 0.25)
                elif avg_lane_line_strength < 0.5:
                    if lighting_condition == "normal":
                        lighting_condition = "dawn_dusk"
                    lane_line_confidence = 0.5
                else:
                    lane_line_confidence = min(0.7, 0.4 + avg_lane_line_strength / 2.0)
                
                # De-emphasize lane line strength if GPS suggests it's night
                # or if model confidence is low, indicating potential dark environment
                if gps_location_msg is not None and gps_location_msg.unixTimestampMillis > 0:
                    # Use GPS time for accurate day/night calculation
                    gps_time = datetime.datetime.fromtimestamp(gps_location_msg.unixTimestampMillis / 1000)
                    hour = gps_time.hour
                    is_night_time = hour >= 19 or hour <= 6
                else:
                    # Fallback to system time if GPS time is not available
                    current_time_dt = datetime.datetime.now()
                    hour = current_time_dt.hour
                    is_night_time = hour >= 19 or hour <= 6

                if is_night_time:
                    # Reduce weight of lane line strength significantly at night
                    total_confidence += lane_line_confidence * 0.05  # Reduced weight from 0.1
                else:
                    total_confidence += lane_line_confidence * 0.2  # Reduced weight from 0.4
                valid_indicators += 1

        # Indicator 2: Time of day based on system time / GPS time (for nighttime detection)
        # Prioritize GPS time for accuracy
        import datetime
        is_night_time = False
        if gps_location_msg is not None and gps_location_msg.unixTimestampMillis > 0:
            gps_time = datetime.datetime.fromtimestamp(gps_location_msg.unixTimestampMillis / 1000)
            hour = gps_time.hour
            is_night_time = hour >= 19 or hour <= 6 # Night time is roughly 7 PM to 6 AM
        else:
            # Fallback to system time if GPS time is not available
            current_time_dt = datetime.datetime.now()
            hour = current_time_dt.hour
            is_night_time = hour >= 19 or hour <= 6

        time_based_confidence = 0.0

        if is_night_time:
            time_based_confidence = 0.9  # High confidence for night classification, increased from 0.8
            if lighting_condition != "dark":
                lighting_condition = "dark"
        else:
            time_based_confidence = 0.6  # Moderate confidence for not night, increased from 0.5

        total_confidence += time_based_confidence * 0.5  # Increased weight to 50%
        valid_indicators += 1

        # Indicator 3: GPS-based sunrise/sunset calculation (placeholder, now potentially more accurate time)
        # This would require GPS coordinates and date for accurate calculation
        # For now, we'll continue using this as a confidence booster, relying on the time-of-day for primary
        gps_based_confidence = 0.4 # Increased from 0.3 for consistency
        total_confidence += gps_based_confidence * 0.1  # Reduced weight, as time-based is now more robust
        valid_indicators += 1

        # Calculate final confidence with multi-indicator fusion
        if valid_indicators > 0:
            lighting_confidence = total_confidence / valid_indicators
        else:
            lighting_confidence = 0.4  # Default if no indicators are valid

        # Apply minimum confidence threshold check
        if lighting_confidence < 0.6:  # Slightly reduced threshold for flexibility
            # If confidence is low, default to "unknown" for safety
            lighting_condition = "unknown"
            lighting_confidence = 0.3  # Reduced confidence for unknown state

        return lighting_condition, lighting_confidence

    def assess_road_condition(self, angular_velocity, acceleration, car_state):
        # Analyze IMU data for road conditions
        road_condition = "normal"
        road_confidence = 0.5  # Base confidence

        if angular_velocity is not None and acceleration is not None:
            # Calculate metrics that indicate road condition
            angular_magnitude = np.sqrt(angular_velocity[0]**2 + angular_velocity[1]**2 + angular_velocity[2]**2)
            accel_magnitude = np.sqrt(acceleration[0]**2 + acceleration[1]**2 + acceleration[2]**2)

            # Check for rough road based on angular magnitude
            if angular_magnitude > self.road_condition_thresholds['roughness']:
                road_condition = "rough"
                road_confidence = min(0.9, 0.5 + (angular_magnitude / (2 * self.road_condition_thresholds['roughness'])))
            
            # Check for slippery road based on acceleration magnitude at speed
            # Also integrate wheel slip if available
            slippage_detected = False
            if hasattr(car_state, 'wheelSpeeds') and car_state.wheelSpeeds is not None:
                # Assuming wheelSpeeds has front and rear wheel speeds
                # Simple check: large difference between front/rear or left/right wheels might indicate slip
                if len(car_state.wheelSpeeds.fl) > 0 and len(car_state.wheelSpeeds.fr) > 0 and \
                   len(car_state.wheelSpeeds.rl) > 0 and len(car_state.wheelSpeeds.rr) > 0:
                    
                    avg_front_speed = (car_state.wheelSpeeds.fl + car_state.wheelSpeeds.fr) / 2
                    avg_rear_speed = (car_state.wheelSpeeds.rl + car_state.wheelSpeeds.rr) / 2
                    
                    # If there's a significant difference between average front and rear speeds, or individual wheel slips
                    if car_state.vEgo > 5.0 and (abs(avg_front_speed - avg_rear_speed) > 1.0 or \
                                                 abs(car_state.wheelSpeeds.fl - car_state.vEgo) > 1.0 or \
                                                 abs(car_state.wheelSpeeds.fr - car_state.vEgo) > 1.0):
                        slippage_detected = True
                        
            if car_state.vEgo > 0 and (accel_magnitude > self.road_condition_thresholds['slippery'] or slippage_detected):
                road_condition = "slippery"
                # Confidence in slippery increases with accel_magnitude and if slippage is detected
                slippery_confidence_imu = min(0.9, 0.5 + (accel_magnitude / (2 * self.road_condition_thresholds['slippery'])))
                slippery_confidence_wheel = 0.9 if slippage_detected else 0.0
                road_confidence = max(slippery_confidence_imu, slippery_confidence_wheel)

        return road_condition, road_confidence

    def assess_weather_condition(self, model_v2, car_state):
        # Analyze multiple indicators to estimate weather
        # Use available model outputs and car state to infer weather conditions

        weather_condition = "clear"  # Default assumption
        weather_confidence = 0.5  # Base confidence

        # Prioritize wiper status for rain/snow detection
        if hasattr(car_state, 'rightWiper') and car_state.rightWiper:
            weather_condition = "rain"  # Or "snow" if temperature data is available
            weather_confidence = 0.9  # High confidence from direct sensor
            return weather_condition, weather_confidence
        if hasattr(car_state, 'leftWiper') and car_state.leftWiper: # Some cars have left/right independent wiper status
            weather_condition = "rain"
            weather_confidence = 0.9
            return weather_condition, weather_confidence

        # Analyze object detection confidence for weather indicators
        # Rain, snow, fog often reduce object detection reliability
        if hasattr(model_v2, 'leadsV3') and len(model_v2.leadsV3) > 0:
            valid_leads = [lead for lead in model_v2.leadsV3 if lead.prob > 0.5]
            detection_reliability = len(valid_leads) / len(model_v2.leadsV3) if len(model_v2.leadsV3) > 0 else 1.0

            if detection_reliability < 0.3:
                weather_condition = "fog"
                weather_confidence = 0.8
            elif detection_reliability < 0.6:
                weather_condition = "rain"
                weather_confidence = 0.7
            else:
                # If leads are reliable, increase confidence in clear weather
                weather_confidence = max(weather_confidence, 0.5 + detection_reliability / 2.0)

        # Check if road edge detection is reduced (common in poor weather)
        if hasattr(model_v2, 'roadEdges') and len(model_v2.roadEdges) > 0:
            avg_edge_confidence = np.mean([edge.strength for edge in model_v2.roadEdges if edge.strength is not None])
            if avg_edge_confidence < 0.4:
                # If existing weather_condition is clear, change to poor_visibility
                if weather_condition == "clear":
                    weather_condition = "poor_visibility"
                # Reduce confidence if road edges are poor
                weather_confidence = min(weather_confidence, 0.4 + avg_edge_confidence / 2.0)

        return weather_condition, weather_confidence

    def assess_gps_signal_quality(self, gps_location_msg) -> None:
        """Assess GPS signal quality and update gps_confidence and gps_signal_lost"""
        if gps_location_msg is not None and gps_location_msg.hasFix:
            # GPS has a fix, confidence is high, signal is not lost
            self.gps_confidence = min(1.0, max(0.5, 1.0 - (gps_location_msg.horizontalAccuracy / 20.0))) # Higher accuracy (lower value) means higher confidence
            self.gps_signal_lost = False
        else:
            # No GPS fix or message is None, confidence is low, signal is lost
            self.gps_confidence = 0.0
            self.gps_signal_lost = True


class SafetyMonitor:
  """
  Enhanced Safety Monitoring System for Autonomous Driving
  Implements multi-sensor fusion validation, confidence thresholds, and environmental adaptation
  """

  def __init__(self):
    # Initialize filters for smooth monitoring
    # Use different time constants for different types of values
    # Confidence values: longer time constant for stability in control decisions
    # Lane deviation: minimal time constant for immediate safety detection
    self.model_confidence_filter = FirstOrderFilter(1.0, 0.5, DT_MDL)
    self.radar_confidence_filter = FirstOrderFilter(1.0, 0.5, DT_MDL)
    self.lane_deviation_filter = FirstOrderFilter(0.0, 0.01, DT_MDL)  # Almost instantaneous for safety detection

    # Initialize new safety enhancement components
    self.anomaly_detector = AdvancedAnomalyDetector()
    self.environmental_detector = EnvironmentalConditionDetector()

    # Initialize params system to allow configurable thresholds
    self.params = Params()

    # Confidence thresholds for safety decisions - with configurable defaults
    # Use proper UnknownKeyName handling for missing parameters
    # Justification: This threshold is set to a relatively high value (0.7) to ensure that the system operates only when the model's prediction
    # is sufficiently reliable. Lower values could lead to unsafe maneuvers. This value is subject to empirical tuning.
    try:
        model_confidence_param = self.params.get("ModelConfidenceThreshold")
        self.model_confidence_threshold = float(model_confidence_param) if model_confidence_param else 0.7
    except UnknownKeyName:
        self.model_confidence_threshold = 0.7  # Default value if parameter not found
    except (TypeError, ValueError):
        self.model_confidence_threshold = 0.7  # Default value if parameter is invalid type

    # Justification: This threshold (0.6) balances responsiveness to radar data with robustness against noise or false positives.
    # It allows for some uncertainty but demands a reasonable level of confidence from the radar system. Subject to empirical tuning.
    try:
        radar_confidence_param = self.params.get("RadarConfidenceThreshold")
        self.radar_confidence_threshold = float(radar_confidence_param) if radar_confidence_param else 0.6
    except UnknownKeyName:
        self.radar_confidence_threshold = 0.6  # Default value if parameter not found
    except (TypeError, ValueError):
        self.radar_confidence_threshold = 0.6  # Default value if parameter is invalid type

    # Justification: This threshold (0.8m) defines the maximum allowable deviation from the lane center before a safety concern is raised.
    # It's a balance between comfort and safety, preventing excessive weaving while allowing for normal driving adjustments.
    # This value needs to be tuned considering vehicle dynamics and typical lane widths.
    try:
        lane_deviation_param = self.params.get("LaneDeviationThreshold")
        self.lane_deviation_threshold = float(lane_deviation_param) if lane_deviation_param else 0.8
    except UnknownKeyName:
        self.lane_deviation_threshold = 0.8  # Default value if parameter not found
    except (TypeError, ValueError):
        self.lane_deviation_threshold = 0.8  # Default value if parameter is invalid type

    # Low speed safety threshold for conservative behavior
    # Justification: At low speeds (below ~20 km/h), control can be more sensitive, and immediate intervention might be critical.
    # A higher threshold (0.4) at low speeds prioritizes safety over smoothness in tight maneuvers.
    try:
        low_speed_safety_param = self.params.get("LowSpeedSafetyThreshold")
        self.low_speed_safety_threshold = float(low_speed_safety_param) if low_speed_safety_param else 0.4
    except UnknownKeyName:
        self.low_speed_safety_threshold = 0.4  # Default value if parameter not found
    except (TypeError, ValueError):
        self.low_speed_safety_threshold = 0.4  # Default value if parameter is invalid type  # meters from center

    # Model confidence threshold multiplier for critical safety checks
    # Justification: This multiplier (0.8) reduces the effective model confidence threshold during critical safety checks,
    # making the system more sensitive to low confidence situations. This creates a buffer for intervention.
    try:
        model_confidence_multiplier_param = self.params.get("ModelConfidenceThresholdMultiplier")
        self.model_confidence_threshold_multiplier = float(model_confidence_multiplier_param) if model_confidence_multiplier_param else 0.8
        # Add validation for multiplier to ensure it is within a valid range [0.5, 1.0]
        if not (0.5 <= self.model_confidence_threshold_multiplier <= 1.0):
            logging.warning(f"ModelConfidenceThresholdMultiplier {self.model_confidence_threshold_multiplier} out of valid range [0.5, 1.0]. Using default 0.8")
            self.model_confidence_threshold_multiplier = 0.8
    except UnknownKeyName:
        self.model_confidence_threshold_multiplier = 0.8  # Default value if parameter not found
    except (TypeError, ValueError):
        self.model_confidence_threshold_multiplier = 0.8  # Default value if parameter is invalid type
    
    # Environmental confidence thresholds - with configurable defaults
    try:
        lighting_confidence_param = self.params.get("LightingConfidenceThreshold")
        self.lighting_confidence_threshold = float(lighting_confidence_param) if lighting_confidence_param else 0.7
    except UnknownKeyName:
        self.lighting_confidence_threshold = 0.7
    except (TypeError, ValueError):
        self.lighting_confidence_threshold = 0.7

    try:
        weather_confidence_param = self.params.get("WeatherConfidenceThreshold")
        self.weather_confidence_threshold = float(weather_confidence_param) if weather_confidence_param else 0.7
    except UnknownKeyName:
        self.weather_confidence_threshold = 0.7
    except (TypeError, ValueError):
        self.weather_confidence_threshold = 0.7

    try:
        road_confidence_param = self.params.get("RoadConfidenceThreshold")
        self.road_confidence_threshold = float(road_confidence_param) if road_confidence_param else 0.7
    except UnknownKeyName:
        self.road_confidence_threshold = 0.7
    except (TypeError, ValueError):
        self.road_confidence_threshold = 0.7
    
    # Curve anticipation parameters
    # Justification: This value (2.0 m/s^2) represents a conservative maximum lateral acceleration the vehicle is allowed to experience.
    # It's a key parameter for calculating safe speeds around curves. This value should be tuned based on vehicle dynamics and comfort.
    try:
        max_lat_accel_param = self.params.get("MaxLateralAcceleration")
        self.max_lat_accel = float(max_lat_accel_param) if max_lat_accel_param else 2.0
    except UnknownKeyName:
        self.max_lat_accel = 2.0
    except (TypeError, ValueError):
        self.max_lat_accel = 2.0

    # Justification: This threshold (0.5) determines what level of curvature is considered significant enough to trigger curve anticipation logic.
    # A higher value would mean only sharper curves are detected, while a lower value would trigger for gentler curves.
    # This needs to be tuned to balance between false positives and missing critical curves.
    try:
        curve_threshold_param = self.params.get("CurveDetectionThreshold")
        self.curve_detection_threshold = float(curve_threshold_param) if curve_threshold_param else 0.5
    except UnknownKeyName:
        self.curve_detection_threshold = 0.5
    except (TypeError, ValueError):
        self.curve_detection_threshold = 0.5

    # Radar confidence parameters
    # Justification: This parameter (150.0m) defines the maximum range at which radar detections contribute to confidence.
    # Beyond this distance, the reliability of radar measurements for immediate safety assessment may diminish,
    # or the relevance to the ego vehicle's immediate safety context decreases.
    try:
        max_radar_distance_param = self.params.get("MaxRadarDistanceForConfidence")
        self.max_radar_distance_for_confidence = float(max_radar_distance_param) if max_radar_distance_param else 150.0
    except UnknownKeyName:
        self.max_radar_distance_for_confidence = 150.0
    except (TypeError, ValueError):
        self.max_radar_distance_for_confidence = 150.0

    # Justification: This scale (30.0) is used to calculate confidence based on the relative velocity of lead vehicles.
    # A larger scale makes the confidence decay slower with increasing relative velocity, implying that even higher relative velocities
    # can be considered with some confidence. This value needs careful empirical tuning to match real-world radar performance and
    # safety requirements across various vehicle types and driving scenarios.
    try:
        velocity_confidence_scale_param = self.params.get("VelocityConfidenceScale")
        self.velocity_confidence_scale = float(velocity_confidence_scale_param) if velocity_confidence_scale_param else 30.0
    except UnknownKeyName:
        self.velocity_confidence_scale = 30.0
    except (TypeError, ValueError):
        self.velocity_confidence_scale = 30.0

    # Justification: This factor (1.0) determines how much the `max_radar_distance_for_confidence` scales with vehicle speed.
    # A higher factor means the system expects to track objects further away at higher speeds.
    # This value needs empirical tuning to reflect safe braking distances and radar capabilities at various speeds.
    try:
        speed_scaling_factor_param = self.params.get("SpeedScalingFactorForRadarDistance")
        self.speed_scaling_factor_for_radar_distance = float(speed_scaling_factor_param) if speed_scaling_factor_param else 1.0
    except UnknownKeyName:
        self.speed_scaling_factor_for_radar_distance = 1.0
    except (TypeError, ValueError):
        self.speed_scaling_factor_for_radar_distance = 1.0

    # Radar confidence parameters for acceleration
    # Justification: This threshold (5.0 m/s^2) determines the maximum acceleration difference considered 'safe' for radar confidence.
    # It assumes that relative accelerations beyond this value significantly reduce the reliability or safety relevance of lead vehicle data.
    # This value is critical and should be tuned based on vehicle dynamics and specific radar performance characteristics.
    try:
        acceleration_confidence_threshold_param = self.params.get("AccelerationConfidenceThreshold")
        self.acceleration_confidence_threshold = float(acceleration_confidence_threshold_param) if acceleration_confidence_threshold_param else 5.0
    except UnknownKeyName:
        self.acceleration_confidence_threshold = 5.0
    except (TypeError, ValueError):
        self.acceleration_confidence_threshold = 5.0

    # Environmental condition detection
    self.lighting_condition = "night"  # "night", "dawn_dusk", "normal", "tunnel"
    self.weather_condition = "unknown"    # "clear", "rain", "snow", "fog", "unknown"
    self.road_condition = "unknown"         # "dry", "wet", "icy", "snow", "unknown"
    self.in_tunnel = True # NEW: Tunnel detection state
    
    # Sensor fusion confidence scores
    self.camera_confidence = 1.0
    self.radar_confidence = 1.0
    self.imu_confidence = 1.0
    self.lighting_confidence = 0.0
    self.weather_confidence = 0.0
    self.road_confidence = 0.0

    # Raw confidence scores for immediate safety assessments (unfiltered)
    self.raw_model_confidence = 1.0
    
    # Sensor health flags - True if healthy, False if failed or degraded
    self.model_healthy = True
    self.radar_healthy = True
    self.camera_healthy = True
    self.imu_healthy = True # For livePose data used in environmental detection
    self.driver_monitoring_healthy = True # New: for driver monitoring health
    self.gps_healthy = True # New: for GPS health

    # Environmental condition filters
    self.lighting_change_filter = FirstOrderFilter(0.0, 1.0, DT_MDL)
    self.weather_change_filter = FirstOrderFilter(0.0, 2.0, DT_MDL)
    
    # Curve anticipation enhancement
    self.curve_anticipation_active = False
    self.curve_anticipation_score = 0.0
    self.max_anticipation_distance = 200.0  # meters ahead
    
    # System safety state
    self.overall_safety_score = 1.0
    self.requires_intervention = False
    self.confidence_degraded = False
    
    # Performance metrics
    self.monitoring_cycles = 0

    # Sensor staleness threshold - if data is older than this, it's considered stale
    # Justification: 0.5 seconds is a reasonable threshold for real-time autonomous driving systems.
    # Older data significantly reduces the reliability and safety of decisions.
    try:
        staleness_threshold_param = self.params.get("SensorStalenessThreshold")
        self.STALENESS_THRESHOLD_SECONDS = float(staleness_threshold_param) if staleness_threshold_param else 0.5
    except (UnknownKeyName, ValueError):
        self.STALENESS_THRESHOLD_SECONDS = 0.5 # Default to 0.5 seconds
    
    # Environmental weight adjustment factor for dynamic sensor weighting
    try:
        environmental_adj_factor_param = self.params.get("EnvironmentalWeightAdjustmentFactor")
        self.environmental_weight_adjustment_factor = float(environmental_adj_factor_param) if environmental_adj_factor_param else 1.0
    except UnknownKeyName:
        self.environmental_weight_adjustment_factor = 1.0
    except (TypeError, ValueError):
        self.environmental_weight_adjustment_factor = 1.0

    # Sensor confidence decay rate (per second of staleness)
    try:
        sensor_decay_rate_param = self.params.get("SensorConfidenceDecayRate")
        self.sensor_confidence_decay_rate = float(sensor_decay_rate_param) if sensor_decay_rate_param else 0.1 # 10% decay per second
    except UnknownKeyName:
        self.sensor_confidence_decay_rate = 0.1
    except (TypeError, ValueError):
        self.sensor_confidence_decay_rate = 0.1
    
    # Store last update times for staleness checks
    self.last_model_time = 0
    self.last_radar_time = 0
    self.last_camera_time = 0
    self.last_imu_time = 0
    self.last_driver_monitoring_time = 0 # New: for driver monitoring staleness
    self.last_gps_time = 0 # New: for GPS staleness

    # Store last valid confidence for decay
    self.last_valid_model_confidence = 1.0
    self.last_valid_radar_confidence = 1.0
    self.last_valid_camera_confidence = 1.0
    self.last_valid_imu_confidence = 1.0
    self.last_valid_driver_monitoring_awareness = 1.0
    self.last_valid_gps_confidence = 1.0 # New: for GPS confidence decay

    # Initialize safety score hysteresis
    self.previous_safety_score = 1.0  # Start with high safety score

  def update_model_confidence(self, model_v2_msg) -> None:
    """Update model confidence based on neural network outputs"""
    if hasattr(model_v2_msg, 'meta') and hasattr(model_v2_msg.meta, 'confidence'):
      raw_confidence = model_v2_msg.meta.confidence if model_v2_msg.meta.confidence else 1.0
      self.model_confidence = self.model_confidence_filter.update(raw_confidence)
      # Also store raw confidence for immediate safety decisions
      self.raw_model_confidence = raw_confidence
    else:
      logging.warning("modelV2 data not available. Defaulting model confidence to a conservative 0.1.")
      self.model_confidence = 0.1  # Fallback to conservative confidence
      self.raw_model_confidence = 0.1  # Fallback to conservative confidence

  def update_radar_confidence(self, radar_state_msg, car_state_msg) -> None:
    """Update radar confidence based on lead detection reliability"""
    if hasattr(radar_state_msg, 'leadOne') and radar_state_msg.leadOne.status:
      # Calculate confidence based on lead tracking quality with a continuous decay
      lead = radar_state_msg.leadOne
      # Use configurable maximum radar distance, dynamically adjusted by vehicle speed
      max_radar_distance_for_confidence_scaled = self.max_radar_distance_for_confidence + \
                                                  (car_state_msg.vEgo * self.speed_scaling_factor_for_radar_distance)
      distance_confidence = max(0.0, 1.0 - (abs(lead.dRel) / max_radar_distance_for_confidence_scaled))
      # Use configurable velocity confidence scale
      velocity_confidence = min(1.0, max(0.0, (self.velocity_confidence_scale - abs(lead.vRel)) / self.velocity_confidence_scale))
      # Add acceleration-based confidence component
      # Assuming lead.aRel exists and is populated in radar_state_msg.leadOne
      # Add acceleration-based confidence component
      # The critical review suggests a fallback of 0.7 if aRel is not available.
      acceleration_confidence = 0.7 # Fallback to a moderate confidence as suggested by critical review if aRel is not available or None.
      if hasattr(lead, 'aRel') and lead.aRel is not None:
        acceleration_confidence = min(1.0, max(0.0, (self.acceleration_confidence_threshold - abs(lead.aRel)) / self.acceleration_confidence_threshold))
      else:
        logging.warning("aRel not available for radar acceleration confidence. Defaulting to a moderate 0.7.")
      self.radar_confidence = self.radar_confidence_filter.update(
        (distance_confidence * 0.5 + velocity_confidence * 0.3 + acceleration_confidence * 0.2)
      )
    else:
      self.radar_confidence = 0.3  # Lower confidence when no lead detected

  def update_camera_confidence(self, model_v2_msg, car_state_msg) -> None:
    """Update camera confidence based on lane detection, visual clarity, and basic car state plausibility"""
    # Plausibility check for vEgo
    if not (0.0 <= car_state_msg.vEgo <= 45.0): # Assuming plausible speed range 0-162 km/h (0-45 m/s)
      logging.warning(f"CarState vEgo ({car_state_msg.vEgo:.2f} m/s) out of plausible range. Reducing camera confidence.")
      self.camera_confidence = 0.1
      self.camera_healthy = False
      return # Skip further processing if vEgo is implausible
    
    if hasattr(model_v2_msg, 'lateralPlan') and hasattr(model_v2_msg.lateralPlan, 'laneWidth'):
      # Calculate lane keeping confidence
      lane_width = model_v2_msg.lateralPlan.laneWidth
      if lane_width > 3.0:  # Reasonable lane width
        # Calculate lane deviation from center
        if len(model_v2_msg.lateralPlan.dPath) > 0:
          center_deviation = model_v2_msg.lateralPlan.dPath[0]  # Immediate deviation
          self.lane_deviation_filter.update(abs(center_deviation))
          
          # Lane keeping confidence decreases with deviation
          lane_confidence = max(0.1, 1.0 - min(0.9, abs(center_deviation) / 2.0))
        else:
          lane_confidence = 0.8
      else:
        lane_confidence = 0.3  # Low confidence for unusual lane widths
      
      # Adjust confidence based on vehicle speed
      speed_factor = min(1.0, max(0.3, (30.0 - car_state_msg.vEgo) / 30.0))
      self.camera_confidence = lane_confidence * speed_factor
    else:
      self.camera_confidence = 0.6

      def detect_environmental_conditions(self, model_v2_msg, car_state_msg, car_control_msg, live_pose_msg=None, gps_location_msg=None) -> None:
          """Detect environmental conditions and adjust safety accordingly"""
          # Use the new EnvironmentalConditionDetector for better environmental assessment
          environmental_conditions, environmental_confidences = self.environmental_detector.detect_conditions(model_v2_msg, live_pose_msg, car_state_msg, gps_location_msg)
  
          # Update internal state with detected conditions, but only if confidence is high enough
          # Include hysteresis to prevent rapid state changes

          # Lighting with hysteresis
          new_lighting_condition = environmental_conditions.get('lighting', 'unknown')
          new_lighting_confidence = environmental_confidences.get('lighting', 0.0)

          # Only update lighting condition if confidence is high enough OR if the change is significant (hysteresis)
          if new_lighting_confidence >= self.lighting_confidence_threshold or \
             (new_lighting_confidence >= 0.5 and new_lighting_condition != self.lighting_condition):
              # Apply hysteresis: only change if new confidence is significantly higher or condition is significantly different
              if (new_lighting_confidence >= self.lighting_confidence_threshold and
                  new_lighting_condition != self.lighting_condition):
                  self.lighting_condition = new_lighting_condition
              elif new_lighting_confidence < 0.5 and self.lighting_confidence < 0.5:
                  # If low confidence in both, only update if the condition is very different from current
                  if (new_lighting_condition in ['dark', 'unknown'] and
                      self.lighting_condition not in ['dark', 'unknown']):
                      self.lighting_condition = 'unknown'  # Default to unknown if confidence is low
          self.lighting_confidence = new_lighting_confidence

          # Road condition with hysteresis
          new_road_condition = environmental_conditions.get('road_condition', 'unknown')
          new_road_confidence = environmental_confidences.get('road_condition', 0.0)

          if new_road_confidence >= self.road_confidence_threshold or \
             (new_road_confidence >= 0.6 and new_road_condition != self.road_condition):
              # For road conditions, we're more conservative due to safety implications
              if (new_road_confidence >= self.road_confidence_threshold and
                  new_road_condition != self.road_condition):
                  self.road_condition = new_road_condition
              elif new_road_confidence < 0.6 and self.road_confidence < 0.6:
                  # If both are low confidence, default to conservative condition
                  if new_road_condition in ['slippery', 'rough']:
                      self.road_condition = 'unknown'  # Default to unknown if low confidence
          self.road_confidence = new_road_confidence

          # Weather with hysteresis
          new_weather_condition = environmental_conditions.get('weather', 'unknown')
          new_weather_confidence = environmental_confidences.get('weather', 0.0)

          if new_weather_confidence >= self.weather_confidence_threshold or \
             (new_weather_confidence >= 0.6 and new_weather_condition != self.weather_condition):
              # For weather, prioritize conditions that affect safety (rain, snow, fog)
              if new_weather_confidence >= self.weather_confidence_threshold:
                  self.weather_condition = new_weather_condition
              elif (new_weather_confidence >= 0.6 and
                    new_weather_condition in ['rain', 'snow', 'fog'] and
                    self.weather_condition not in ['rain', 'snow', 'fog']):
                  # Only update to adverse weather conditions if we're reasonably confident
                  self.weather_condition = new_weather_condition
              elif new_weather_confidence < 0.6 and self.weather_confidence < 0.6:
                  # If low confidence in both, default to conservative 'unknown'
                  self.weather_condition = 'unknown'
          self.weather_confidence = new_weather_confidence

          # Tunnel detection logic - more robust with multiple condition checks
          # A tunnel is characterized by sustained darkness and GPS signal loss, with potentially clear road surface.
          # However, we need a more reliable approach because lane markings can be strong in tunnels
          # The original approach was flawed because strong lane lines are often present in tunnels.

          # Improved tunnel detection: GPS signal loss is a much more reliable indicator
          gps_signal_lost = self.environmental_detector.gps_signal_lost
          gps_confidence_low = self.environmental_detector.gps_confidence < 0.5

          # Check if lighting is dark, but we need to be more careful not to rely on lane line strength
          # Since lane line strength can be misleading in tunnels, we'll use GPS-based lighting condition
          # instead of relying on the lighting_condition that could be influenced by lane lines
          # Check if GPS suggests we're in a daylight area but lighting is still showing as dark/unknown
          is_night_time = False
          if gps_location_msg is not None and gps_location_msg.unixTimestampMillis > 0:
              import datetime
              gps_time = datetime.datetime.fromtimestamp(gps_location_msg.unixTimestampMillis / 1000)
              hour = gps_time.hour
              is_night_time = hour >= 19 or hour <= 6

          # We'll consider lighting as genuinely dark if it's dark and it's either night time or the confidence is high
          # But we'll also check if this conflicts with GPS time (daylight time but lighting is dark = potential tunnel)
          lighting_appears_dark = (self.lighting_condition in ['dark', 'unknown'] and
                                  self.lighting_confidence > self.lighting_confidence_threshold)

          # More robust lighting condition for tunnel detection:
          # If it's GPS daytime but lighting system says it's dark, that's a strong tunnel indicator
          gps_suggests_daylight = not is_night_time
          lighting_suggests_darkness = lighting_appears_dark
          gps_dark_lighting_conflict = gps_suggests_daylight and lighting_suggests_darkness

          # Road confidence being high is normal in tunnels (surface is often smooth and clear)
          road_confidence_high = self.road_confidence > self.road_confidence_threshold

          # Add a proxy for "lack of outdoor visual cues" - for example, if model confidence is low.
          # Using raw_model_confidence for immediate detection.
          model_confidence_low = self.raw_model_confidence < self.model_confidence_threshold

          # Improved tunnel detection logic:
          # A tunnel is primarily identified by GPS signal loss combined with other indicators
          # and especially by the contradiction between GPS daylight and dark lighting conditions
          if gps_signal_lost and (lighting_appears_dark or gps_dark_lighting_conflict):
              self.in_tunnel = True
          # Additionally, if GPS suggests daylight but we have low lighting confidence and GPS signal loss, likely tunnel
          elif gps_signal_lost and gps_dark_lighting_conflict and road_confidence_high:
              self.in_tunnel = True
          # If GPS signal is good and conditions match expected daytime, definitely not a tunnel
          elif not gps_signal_lost and self.environmental_detector.gps_confidence > 0.7 and not gps_dark_lighting_conflict:
              self.in_tunnel = False
          # If GPS indicates daylight but signal is not lost and lighting is normal, not a tunnel
          elif not gps_signal_lost and self.environmental_detector.gps_confidence > 0.7 and not lighting_appears_dark:
              self.in_tunnel = False
          # If we reach here, we maintain the previous state to avoid flickering.
          # This provides hysteresis for ambiguous situations.

    def detect_curve_anticipation(self, model_v2_msg, car_state_msg) -> None:
        """Enhanced curve anticipation with improved safety margins"""
        if hasattr(model_v2_msg, 'path') and len(model_v2_msg.path.x) > 10:
            # Properly calculate path curvature using first and second derivatives
            max_curvature_ahead = 0.0
            x_coords = model_v2_msg.path.x
            y_coords = model_v2_msg.path.y
            z_coords = model_v2_msg.path.z # Assuming z-coordinates are available similarly

            if len(x_coords) > 2 and len(y_coords) > 2 and len(z_coords) > 2:
                # Convert lists to numpy arrays for efficient computation
                x_coords_np = np.array(x_coords)
                y_coords_np = np.array(y_coords)
                z_coords_np = np.array(z_coords)

                # Assuming path points are equally spaced (0.5m)
                path_interval = 0.5 # Typically 0.5 meters between path points

                # Add proper path length validation
                min_points_needed = int(self.max_anticipation_distance / path_interval)
                current_max_anticipation_distance = self.max_anticipation_distance
                if len(x_coords_np) < min_points_needed:
                    logging.warning(f"Path is shorter than expected for {self.max_anticipation_distance}m anticipation. Actual: {len(x_coords_np)} points, Expected: {min_points_needed} points. Using available path length.")
                    # Fall back to shorter distance for calculation
                    current_max_anticipation_distance = len(x_coords_np) * path_interval

                # Calculate first derivatives (dx/ds, dy/ds) and second derivatives
                dx_ds = np.gradient(x_coords_np, path_interval)
                dy_ds = np.gradient(y_coords_np, path_interval)
                d2x_ds2 = np.gradient(dx_ds, path_interval)
                d2y_ds2 = np.gradient(dy_ds, path_interval)

                # Calculate dz/ds (slope along the path)
                dz_ds = np.gradient(z_coords_np, path_interval)
                # Approximate road grade (angle in radians)
                # Assuming ds is roughly constant and small enough for tan(angle) approx angle
                # grade_angle = np.arctan(dz_ds)
                # For a simplified approach, use a smoothed version of dz_ds or an average over a segment
                # Here, we will just use the dz_ds directly as a proxy for grade.

                # Calculate curvature using the vectorized formula: curvature = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
                numerator = np.abs(dx_ds * d2y_ds2 - dy_ds * d2x_ds2)
                denominator_squared = dx_ds**2 + dy_ds**2

                # Avoid division by zero: set curvature to 0 where denominator is too small
                valid_indices = denominator_squared > 1e-6

                local_curvatures = np.zeros_like(numerator)
                local_curvatures[valid_indices] = numerator[valid_indices] / (denominator_squared[valid_indices]**1.5)

                effective_path_length = min(len(x_coords_np), int(current_max_anticipation_distance / path_interval))

                if effective_path_length > 0:
                    max_curvature_ahead = np.max(np.abs(local_curvatures[:effective_path_length]))
                    # Get the grade at the point of max curvature for simplicity
                    # This is a simplification; a more robust solution would consider grade over the entire curve
                    max_curve_idx = np.argmax(np.abs(local_curvatures[:effective_path_length]))

                    # Simple approximation of grade at max curvature point
                    # Avoid division by zero for small dx_ds and dy_ds (flat path)
                    if np.linalg.norm([dx_ds[max_curve_idx], dy_ds[max_curve_idx]]) > 1e-6:
                        grade_at_max_curvature = dz_ds[max_curve_idx] / np.linalg.norm([dx_ds[max_curve_idx], dy_ds[max_curve_idx]])
                    else:
                        grade_at_max_curvature = 0.0
                else:
                    max_curvature_ahead = 0.0
                    grade_at_max_curvature = 0.0


            # Calculate safe speed based on curvature and now, road grade
            if max_curvature_ahead > 0.001:  # Significant curve
                # Adjust max_lat_accel based on road grade for safety
                # On a downhill grade, the effective lateral friction available might be reduced.
                # On an uphill grade, it might be increased.
                # A conservative approach for now: reduce safe speed on downhill grades.
                grade_factor = 1.0
                if grade_at_max_curvature < 0: # Downhill
                    # Reduce safe speed for downhill curves. For example, a 5% grade (approx 0.05 rad)
                    # might reduce the effective lateral acceleration by a small percentage.
                    # This is a placeholder and needs empirical tuning.
                    grade_factor = max(0.8, 1.0 + grade_at_max_curvature * 5.0) # Reduce by up to 20% for steep downhill
                elif grade_at_max_curvature > 0: # Uphill
                    # For uphill, we might slightly increase the safe speed, or keep it neutral for conservatism.
                    # For now, let's keep it neutral to prioritize safety.
                    grade_factor = 1.0

                effective_max_lat_accel = self.max_lat_accel * grade_factor
                safe_speed = (effective_max_lat_accel / max_curvature_ahead) ** 0.5 if max_curvature_ahead > 0.0001 else car_state_msg.vEgo

                # Calculate anticipation score based on speed vs safe speed
                if car_state_msg.vEgo > 5.0:  # Only for meaningful speeds
                    speed_ratio = min(1.0, max(0.0, safe_speed / car_state_msg.vEgo))
                    self.curve_anticipation_score = max_curvature_ahead * (1.0 - speed_ratio)
                    self.curve_anticipation_active = speed_ratio < 0.9  # 10% margin
                else:
                    self.curve_anticipation_score = 0.0
                    self.curve_anticipation_active = False
            else:
                self.curve_anticipation_score = 0.0
                self.curve_anticipation_active = False
        else:
            self.curve_anticipation_score = 0.0
            self.curve_anticipation_active = False

  def calculate_overall_safety_score(self, car_state_msg, driver_monitoring_state_msg) -> float:
    """Calculate overall safety score based on all inputs"""
    # Weighted combination of all confidence measures
    # Adjust weights based on environmental conditions with quantitative validation
    # Define initial base weights for each sensor. These weights are conceptual and
    # are dynamically adjusted based on environmental conditions and their detection confidence.
    # These initial values serve as a starting point before environmental adjustments are applied.
    # Updated weights based on critical review: More emphasis on radar in poor conditions, less on camera
    base_weights = {
      'model': 0.35,  # Reduced from 0.4 to account for potential model failures
      'camera': 0.25,  # Reduced from 0.3 for better robustness in poor visibility
      'radar': 0.25,   # Increased from 0.2 since radar is more reliable in adverse conditions
      'imu': 0.15      # Increased from 0.1 for better stability input
    }

    # Load environmental weight adjustment factor from Params
    try:
        environmental_adj_factor_param = self.params.get("EnvironmentalWeightAdjustmentFactor")
        self.environmental_weight_adjustment_factor = float(environmental_adj_factor_param) if environmental_adj_factor_param else 1.0
    except UnknownKeyName:
        self.environmental_weight_adjustment_factor = 1.0
    except (TypeError, ValueError):
        self.environmental_weight_adjustment_factor = 1.0

    # Apply sensor health status to raw confidence values
    model_conf_val = self.model_confidence if self.model_healthy else 0.1
    radar_conf_val = self.radar_confidence if self.radar_healthy else 0.1
    camera_conf_val = self.camera_confidence if self.camera_healthy else 0.1
    imu_conf_val = self.imu_confidence if self.imu_healthy else 0.1

    # Initialize adjusted weights with base weights
    adjusted_weights = base_weights.copy()

    # Adjust weights based on environmental conditions and their confidence
    # The influence of environmental conditions is scaled by their detection confidence
    # and a configurable adjustment factor.

    # Lighting adjustments
    lighting_influence = self.environmental_weight_adjustment_factor * self.lighting_confidence
    if self.lighting_condition in ["night", "dawn_dusk", "dark", "tunnel"]:
        adjusted_weights['model'] *= (1.0 - 0.3 * lighting_influence)  # Reduce model weight
        adjusted_weights['camera'] *= (1.0 - 0.4 * lighting_influence) # More reduction for camera
        adjusted_weights['radar'] *= (1.0 + 0.2 * lighting_influence)  # Increase radar weight
    elif self.lighting_condition == "bright":
        adjusted_weights['camera'] *= (1.0 + 0.1 * lighting_influence) # Slightly increase camera weight

    # Road condition adjustments
    road_influence = self.environmental_weight_adjustment_factor * self.road_confidence
    if self.road_condition == "rough":
        adjusted_weights['model'] *= (1.0 - 0.1 * road_influence)
        adjusted_weights['camera'] *= (1.0 - 0.15 * road_influence)
    elif self.road_condition == "slippery":
        adjusted_weights['model'] *= (1.0 - 0.15 * road_influence)
        adjusted_weights['camera'] *= (1.0 - 0.2 * road_influence)
        adjusted_weights['radar'] *= (1.0 + 0.1 * road_influence)

    # Weather adjustments
    weather_influence = self.environmental_weight_adjustment_factor * self.weather_confidence
    if self.weather_condition in ["rain", "snow", "fog", "unknown", "poor_visibility"]:
        adjusted_weights['model'] *= (1.0 - 0.4 * weather_influence)
        adjusted_weights['camera'] *= (1.0 - 0.5 * weather_influence)
        adjusted_weights['radar'] *= (1.0 + 0.3 * weather_influence)
    elif self.weather_condition == "clear":
        adjusted_weights['model'] *= (1.0 + 0.05 * weather_influence)
        adjusted_weights['camera'] *= (1.0 + 0.05 * weather_influence)

    # Ensure weights sum to 1 after adjustments
    total_weight = sum(adjusted_weights.values())
    if total_weight > 0:
        for key in adjusted_weights:
            adjusted_weights[key] /= total_weight
    
    # Calculate safety score using adjusted weights and confidence values
    safety_score = (
      model_conf_val * adjusted_weights['model'] +
      camera_conf_val * adjusted_weights['camera'] +
      radar_conf_val * adjusted_weights['radar'] +
      imu_conf_val * adjusted_weights['imu']
    )

    # Apply critical safety penalties for values significantly below thresholds
    # Use raw confidence for immediate safety response to avoid filter delays
    model_confidence_threshold_adjusted = self.model_confidence_threshold * self.model_confidence_threshold_multiplier  # 0.56 if threshold is 0.7
    if self.raw_model_confidence < model_confidence_threshold_adjusted:
      # Apply additional penalty when raw model confidence is critically low
      confidence_deficit = (model_confidence_threshold_adjusted - self.raw_model_confidence) / model_confidence_threshold_adjusted
      safety_penalty = min(0.3, confidence_deficit)  # Max 30% reduction for very low confidence
      safety_score *= (1.0 - safety_penalty)
    # Note: This penalty is applied per frame. If raw_model_confidence improves above the threshold,
    # the penalty is automatically removed in the subsequent frame, ensuring dynamic recovery.

    # Apply curve anticipation safety factor
    if self.curve_anticipation_active and car_state_msg.vEgo > 10.0:
      # Reduce safety score when approaching curves at high speed
      curve_factor = max(0.5, 1.0 - (self.curve_anticipation_score * 2.0))
      safety_score *= curve_factor

    # Apply lane deviation penalty
    if self.lane_deviation_filter.x > self.lane_deviation_threshold:
      deviation_penalty = max(0.1, 1.0 - (self.lane_deviation_filter.x / self.lane_deviation_threshold))
      safety_score *= deviation_penalty

    # Apply driver monitoring penalty
    if not self.driver_monitoring_healthy:
      logging.warning("Driver monitoring system unhealthy. Applying significant safety score penalty.")
      safety_score *= 0.5 # Significant penalty if system is not working
    elif driver_monitoring_state_msg is not None and hasattr(driver_monitoring_state_msg, 'awarenessStatus') and driver_monitoring_state_msg.awarenessStatus is not None:
      # Assuming awarenessStatus is 0-1, where low is bad
      if driver_monitoring_state_msg.awarenessStatus < 0.3: # Critically low awareness
        logging.warning(f"Critically low driver awareness ({driver_monitoring_state_msg.awarenessStatus:.2f}). Applying significant safety score penalty.")
        safety_score *= 0.6
      elif driver_monitoring_state_msg.awarenessStatus < 0.5: # Moderate low awareness
        logging.warning(f"Low driver awareness ({driver_monitoring_state_msg.awarenessStatus:.2f}). Applying moderate safety score penalty.")
        safety_score *= 0.8

    # NEW: Apply anomaly-based penalties with temporal consistency
    if hasattr(self, 'anomalies') and self.anomalies:
      for anomaly_type, anomaly_data in self.anomalies.items():
        if anomaly_type in ['high_jerk', 'velocity_inconsistency', 'high_steering_rate']:
          # Apply severity-based penalty based on the anomaly data
          if anomaly_type == 'high_jerk':
            # More severe penalty for higher jerk values
            current_jerk = anomaly_data.get('current_jerk', 0)
            severity = min(0.3, current_jerk / 15.0)  # Cap penalty at 30%
            safety_score *= (1.0 - severity)
          elif anomaly_type == 'velocity_inconsistency':
            # Penalty based on velocity difference
            difference = anomaly_data.get('difference', 0)
            severity = min(0.25, difference / 2.0)  # Cap penalty at 25%
            safety_score *= (1.0 - severity)
          elif anomaly_type == 'high_steering_rate':
            # Penalty based on steering rate
            steering_rate = abs(anomaly_data.get('steering_rate', 0))
            severity = min(0.2, steering_rate / 150.0)  # Cap penalty at 20%
            safety_score *= (1.0 - severity)

    # Apply safety score hysteresis to prevent oscillation
    # Only allow rapid changes in safety score when there are critical anomalies
    if hasattr(self, 'previous_safety_score'):
        score_delta = abs(safety_score - self.previous_safety_score)

        # If the change is significant (>0.2) but not due to critical anomalies, smooth it
        if score_delta > 0.2 and not self.requires_intervention:
            # Limit the rate of change to prevent oscillation
            max_change_rate = 0.1  # Maximum 10% change per update in normal conditions
            adjusted_delta = min(score_delta, max_change_rate)

            if safety_score > self.previous_safety_score:
                safety_score = min(safety_score, self.previous_safety_score + adjusted_delta)
            else:
                safety_score = max(safety_score, self.previous_safety_score - adjusted_delta)

    # Store current safety score for next iteration
    self.previous_safety_score = safety_score

    return max(0.0, min(1.0, safety_score))

  def evaluate_safety_intervention_needed(self, safety_score: float, car_state_msg, driver_monitoring_state_msg) -> bool:
    """Determine if safety intervention is required"""
    # Check multiple conditions for intervention
    intervention_needed = False

    # Model confidence too low
    if self.model_confidence < self.model_confidence_threshold * self.model_confidence_threshold_multiplier:
      intervention_needed = True

    # If any critical sensor is unhealthy, intervention is more likely
    if not self.model_healthy or not self.radar_healthy or not self.camera_healthy:
        logging.warning("Critical sensor unhealthy, increasing likelihood of intervention.")
        intervention_needed = True # Even if other scores are good, sensor failure is critical

    # Driver monitoring: critical awareness or unhealthy system leads to intervention
    if not self.driver_monitoring_healthy:
      logging.warning("Driver monitoring system unhealthy. Intervention needed.")
      intervention_needed = True
    elif driver_monitoring_state_msg is not None and hasattr(driver_monitoring_state_msg, 'awarenessStatus') and driver_monitoring_state_msg.awarenessStatus is not None:
      if driver_monitoring_state_msg.awarenessStatus < 0.2: # Very low awareness, immediate intervention
        logging.warning(f"Very low driver awareness ({driver_monitoring_state_msg.awarenessStatus:.2f}). Intervention needed.")
        intervention_needed = True

    # Lane deviation too high
    if self.lane_deviation_filter.x > self.lane_deviation_threshold * 1.2:
      intervention_needed = True

    # Approaching curve too fast
    if self.curve_anticipation_active and self.curve_anticipation_score > 0.05:
      intervention_needed = True

    # Overall safety score too low
    if safety_score < 0.3:
      intervention_needed = True

    # At very low speeds, be more conservative with intervention
    if car_state_msg.vEgo < 5.0 and safety_score < self.low_speed_safety_threshold:
      intervention_needed = True

    # NEW: Check for critical anomalies that require intervention
    if hasattr(self, 'anomalies') and self.anomalies:
      # Check for critical anomalies that should trigger immediate intervention
      critical_anomalies = ['high_jerk', 'velocity_inconsistency', 'high_steering_rate']
      for anomaly_type, anomaly_data in self.anomalies.items():
        if anomaly_type in critical_anomalies:
          if anomaly_type == 'high_jerk' and anomaly_data.get('current_jerk', 0) > 8.0:  # Very high jerk (8 m/s³)
            intervention_needed = True
            logging.warning(f"Critical anomaly detected: Excessive jerk of {anomaly_data.get('current_jerk', 0)} m/s³")
          elif anomaly_type == 'velocity_inconsistency' and anomaly_data.get('difference', 0) > 2.5:  # Very large velocity difference
            intervention_needed = True
            logging.warning(f"Critical anomaly detected: Velocity inconsistency of {anomaly_data.get('difference', 0)} m/s")
          elif anomaly_type == 'high_steering_rate' and abs(anomaly_data.get('steering_rate', 0)) > 150.0:  # Very high steering rate
            intervention_needed = True
            logging.warning(f"Critical anomaly detected: High steering rate of {anomaly_data.get('steering_rate', 0)} deg/s")

    return intervention_needed

  def _update_sensor_confidence_with_staleness(self, sensor_type: str, healthy_flag: bool, current_time: float,
                                                sensor_mono_time: Optional[int], last_sensor_time: float,
                                                last_valid_confidence_attr: str, update_func, *args) -> None:
    """
    Helper method to update sensor confidence, handle staleness, and apply decay.
    Args:
        sensor_type: String identifier for the sensor (e.g., "model", "radar").
        healthy_flag: Current healthy status of the sensor (e.g., self.model_healthy).
        current_time: Current monotonic time in nanoseconds.
        sensor_mono_time: Monotonic time of the last sensor message (can be None for optional messages).
        last_sensor_time: Last recorded monotonic time for this sensor.
        last_valid_confidence_attr: Name of the attribute storing the last valid confidence (e.g., "last_valid_model_confidence").
        update_func: The sensor-specific function to call for updating confidence (e.g., self.update_model_confidence).
        *args: Arguments to pass to the update_func.
    """
    # Get current confidence value (e.g., self.model_confidence)
    current_confidence_attr = f'{sensor_type}_confidence'
    
    if healthy_flag:
      try:
        update_func(*args)
        # Update current confidence attribute from the result of update_func
        # If update_func modifies self.sensor_confidence directly, no need to re-fetch
        current_confidence = getattr(self, current_confidence_attr)
        setattr(self, last_valid_confidence_attr, current_confidence)
      except Exception as e:
        logging.error(f"Error in {update_func.__name__} for {sensor_type}: {e}")
        setattr(self, current_confidence_attr, 0.5)  # Default to moderate confidence on error
        setattr(self, f'{sensor_type}_healthy', False)
        setattr(self, last_valid_confidence_attr, getattr(self, current_confidence_attr)) # Also update last valid on error
    else:
      # Apply decay to last valid confidence if stale
      time_since_update = (current_time - last_sensor_time) * 1e-9
      decay_factor = max(0.0, 1.0 - self.sensor_confidence_decay_rate * time_since_update)
      decayed_confidence = getattr(self, last_valid_confidence_attr) * decay_factor
      setattr(self, current_confidence_attr, decayed_confidence)
      logging.warning(f"{sensor_type.capitalize()} confidence decayed to {getattr(self, current_confidence_attr):.2f} due to staleness.")

    def _initialize_health_flags(self) -> None:
        """Initializes all sensor health flags to True at the start of each update cycle."""
        self.model_healthy = True
        self.radar_healthy = True
        self.camera_healthy = True
        self.imu_healthy = True
        self.driver_monitoring_healthy = True
        self.gps_healthy = True

    def _perform_staleness_checks(self, current_time: float, model_v2_mono_time: int, radar_state_mono_time: int, car_state_mono_time: int, live_pose_mono_time: Optional[int], driver_monitoring_state_mono_time: Optional[int], gps_location_mono_time: Optional[int]) -> None:
        """Performs staleness checks for all sensors and updates their healthy flags."""
        # Model Staleness
        time_since_model_update = (current_time - model_v2_mono_time) * 1e-9
        if time_since_model_update > self.STALENESS_THRESHOLD_SECONDS:
            self.model_healthy = False
            logging.warning(f"Model data is stale. Last update: {time_since_model_update:.2f}s ago.")
        else:
            self.model_healthy = True
            self.last_model_time = model_v2_mono_time

        # Radar Staleness
        time_since_radar_update = (current_time - radar_state_mono_time) * 1e-9
        if time_since_radar_update > self.STALENESS_THRESHOLD_SECONDS:
            self.radar_healthy = False
            logging.warning(f"Radar data is stale. Last update: {time_since_radar_update:.2f}s ago.")
        else:
            self.radar_healthy = True
            self.last_radar_time = radar_state_mono_time

        # Camera (carState is used as a proxy for freshness of car data influencing camera confidence)
        time_since_car_state_update = (current_time - car_state_mono_time) * 1e-9
        if time_since_car_state_update > self.STALENESS_THRESHOLD_SECONDS:
            self.camera_healthy = False
            logging.warning(f"CarState data (influencing camera confidence) is stale. Last update: {time_since_car_state_update:.2f}s ago.")
        else:
            self.camera_healthy = True
            self.last_camera_time = car_state_mono_time # Update last_camera_time based on carState

        # IMU Staleness (livePose)
        if live_pose_mono_time is not None:
            time_since_imu_update = (current_time - live_pose_mono_time) * 1e-9
            if time_since_imu_update > self.STALENESS_THRESHOLD_SECONDS:
                self.imu_healthy = False
                logging.warning(f"IMU data (livePose) is stale. Last update: {time_since_imu_update:.2f}s ago.")
            else:
                self.imu_healthy = True
                self.last_imu_time = live_pose_mono_time
        else: # If live_pose_msg is None, then IMU data is not available
            self.imu_healthy = False
            logging.warning("LivePose message is not available. IMU data confidence reduced.")

        # Driver Monitoring Staleness
        if driver_monitoring_state_mono_time is not None:
            time_since_dm_update = (current_time - driver_monitoring_state_mono_time) * 1e-9
            if time_since_dm_update > self.STALENESS_THRESHOLD_SECONDS:
                self.driver_monitoring_healthy = False
                logging.warning(f"Driver Monitoring data is stale. Last update: {time_since_dm_update:.2f}s ago.")
            else:
                self.driver_monitoring_healthy = True
                self.last_driver_monitoring_time = driver_monitoring_state_mono_time
        else:
            self.driver_monitoring_healthy = False
            logging.warning("Driver Monitoring message is not available. Driver monitoring confidence reduced.")

        # GPS Staleness
        if gps_location_mono_time is not None:
            time_since_gps_update = (current_time - gps_location_mono_time) * 1e-9
            if time_since_gps_update > self.STALENESS_THRESHOLD_SECONDS:
                self.gps_healthy = False
                logging.warning(f"GPS data is stale. Last update: {time_since_gps_update:.2f}s ago.")
            else:
                self.gps_healthy = True
                self.last_gps_time = gps_location_mono_time
        else: # If gps_location_msg is None, then GPS data is not available
            self.gps_healthy = False
            logging.warning("GPS location message is not available. GPS data confidence reduced.")

        def _run_anomaly_detection(self, car_state_msg, model_v2_msg, radar_state_msg) -> None:
            """Runs anomaly detection and stores the results."""
            try:
                self.anomalies = self.anomaly_detector.detect_anomalies(car_state_msg, model_v2_msg, radar_state_msg)
            except Exception as e:
                logging.error(f"Error in anomaly detection: {e}")
                self.anomalies = {}
    
            def _update_all_confidences(self, current_time: float, model_v2_msg, radar_state_msg, car_state_msg, car_control_msg, live_pose_msg, gps_location_msg, gps_location_mono_time: Optional[int]) -> None:
    
                """Updates confidence measures for all sensors and environmental conditions."""
    
                # Model confidence
    
                self._update_sensor_confidence_with_staleness(
    
                    "model", self.model_healthy, current_time, model_v2_msg.logMonoTime if model_v2_msg else 0, self.last_model_time,
    
                    "last_valid_model_confidence", self.update_model_confidence, model_v2_msg
    
                )
    
          
    
                # Radar confidence
    
                self._update_sensor_confidence_with_staleness(
    
                    "radar", self.radar_healthy, current_time, radar_state_msg.logMonoTime if radar_state_msg else 0, self.last_radar_time,
    
                    "last_valid_radar_confidence", self.update_radar_confidence, radar_state_msg, car_state_msg
    
                )
    
          
    
                # Camera confidence (using carState for freshness)
    
                self._update_sensor_confidence_with_staleness(
    
                    "camera", self.camera_healthy, current_time, car_state_msg.logMonoTime if car_state_msg else 0, self.last_camera_time, # Using carState mono time here
    
                    "last_valid_camera_confidence", self.update_camera_confidence, model_v2_msg, car_state_msg
    
                )
    
          
    
                # IMU and Environmental conditions
    
                # Note: detect_environmental_conditions updates IMU confidence indirectly via its sub-components
    
                # and also sets lighting, weather, road conditions. We track imu_healthy here.
    
                if self.imu_healthy:
    
                    try:
    
                        self.detect_environmental_conditions(model_v2_msg, car_state_msg, car_control_msg, live_pose_msg, gps_location_msg)
    
                        self.last_valid_imu_confidence = self.imu_confidence
    
                    except Exception as e:
    
                        logging.error(f"Error in detect_environmental_conditions: {e}")
    
                        self.imu_healthy = False
    
                        self.imu_confidence = 0.1 # Set IMU confidence to low on processing error
    
                        self.last_valid_imu_confidence = self.imu_confidence
    
                        # Also set environmental conditions to unknown on error
    
                        self.lighting_condition = "unknown"
    
                        self.weather_condition = "unknown"
    
                        self.road_condition = "unknown"
    
                        self.lighting_confidence = 0.0
    
                        self.weather_confidence = 0.0
    
                        self.road_confidence = 0.0
    
                else:
    
                    time_since_update = (current_time - self.last_imu_time) * 1e-9
    
                    decay_factor = max(0.0, 1.0 - self.sensor_confidence_decay_rate * time_since_update)
    
                    self.imu_confidence = self.last_valid_imu_confidence * decay_factor
    
                    logging.warning(f"IMU confidence decayed to {self.imu_confidence:.2f} due to staleness.")
    
                    # If IMU is unhealthy, set environmental conditions to unknown and confidence low
    
                    self.lighting_condition = "unknown"
    
                    self.weather_condition = "unknown"
    
                    self.road_condition = "unknown"
    
                    self.lighting_confidence = 0.0
    
                    self.weather_confidence = 0.0
    
                    self.road_confidence = 0.0
    
          
    
                # GPS Confidence
    
                self._update_sensor_confidence_with_staleness(
    
                    "gps", self.gps_healthy, current_time, gps_location_mono_time if gps_location_mono_time is not None else 0,
    
                    self.last_gps_time, "last_valid_gps_confidence", self.environmental_detector.assess_gps_signal_quality, gps_location_msg
    
                )
    
        
    
                def _calculate_safety_outputs(self, model_v2_msg, car_state_msg, driver_monitoring_state_msg) -> None:
    
        
    
                    """Calculates overall safety score, intervention needs, and confidence degradation."""
    
        
    
                    # Detect curve anticipation
    
        
    
                    self.detect_curve_anticipation(model_v2_msg, car_state_msg)
    
        
    
            
    
        
    
                    # Calculate overall safety with error handling
    
        
    
                    try:
    
        
    
                        self.overall_safety_score = self.calculate_overall_safety_score(car_state_msg, driver_monitoring_state_msg)
    
        
    
                    except Exception as e:
    
        
    
                        logging.error(f"Error in calculate_overall_safety_score: {e}")
    
        
    
                        self.overall_safety_score = 0.5
    
        
    
                
    
        
    
                    try:
    
        
    
                        self.requires_intervention = self.evaluate_safety_intervention_needed(
    
        
    
                        self.overall_safety_score, car_state_msg, driver_monitoring_state_msg
    
        
    
                        )
    
        
    
                    except Exception as e:
    
        
    
                        logging.error(f"Error in evaluate_safety_intervention_needed: {e}")
    
        
    
                        self.requires_intervention = True  # Default to intervention on error
    
        
    
                
    
        
    
                    # Determine if confidence is degraded (requires more conservative driving)
    
        
    
                    self.confidence_degraded = self.overall_safety_score < 0.6
    
        
    
            
    
        
    
                    def _prepare_safety_report(self) -> Tuple[float, bool, Dict]:
    
        
    
            
    
        
    
                        """Prepares the final safety report and adjusts scores based on anomalies."""
    
        
    
            
    
        
    
                        safety_report = {
    
        
    
            
    
        
    
                            'model_confidence': getattr(self, 'model_confidence', 0.5),
    
        
    
            
    
        
    
                            'radar_confidence': getattr(self, 'radar_confidence', 0.5),
    
        
    
            
    
        
    
                            'camera_confidence': getattr(self, 'camera_confidence', 0.5),
    
        
    
            
    
        
    
                            'imu_confidence': getattr(self, 'imu_confidence', 0.5),
    
        
    
            
    
        
    
                            'gps_confidence': getattr(self, 'gps_confidence', 0.5),
    
        
    
            
    
        
    
                            'model_healthy': self.model_healthy,
    
        
    
            
    
        
    
                            'radar_healthy': self.radar_healthy,
    
        
    
            
    
        
    
                            'camera_healthy': self.camera_healthy,
    
        
    
            
    
        
    
                            'imu_healthy': self.imu_healthy,
    
        
    
            
    
        
    
                            'driver_monitoring_healthy': self.driver_monitoring_healthy,
    
        
    
            
    
        
    
                            'gps_healthy': self.gps_healthy,
    
        
    
            
    
        
    
                            'lighting_condition': self.lighting_condition,
    
        
    
            
    
        
    
                            'weather_condition': self.weather_condition,
    
        
    
            
    
        
    
                            'road_condition': self.road_condition,
    
        
    
            
    
        
    
                            'curve_anticipation_active': getattr(self, 'curve_anticipation_active', False),
    
        
    
            
    
        
    
                            'curve_anticipation_score': getattr(self, 'curve_anticipation_score', 0.0),
    
        
    
            
    
        
    
                            'lane_deviation': getattr(self.lane_deviation_filter, 'x', 0.0),
    
        
    
            
    
        
    
                            'overall_safety_score': self.overall_safety_score,
    
        
    
            
    
        
    
                            'confidence_degraded': self.confidence_degraded,
    
        
    
            
    
        
    
                            'monitoring_cycles': self.monitoring_cycles,
    
        
    
            
    
        
    
                            'anomalies_detected': self.anomalies
    
        
    
            
    
        
    
                        }
    
        
    
            
    
        
    
                        self.monitoring_cycles += 1
    
        
    
            
    
        
    
                  
    
        
    
            
    
        
    
                        # Add error flag if any sub-component failed
    
        
    
            
    
        
    
                        safety_report['error_occurred'] = False
    
        
    
            
    
        
    
                  
    
        
    
            
    
        
    
                        # Check if any anomalies detected that require immediate safety response
    
        
    
            
    
        
    
                        if self.anomalies:
    
        
    
            
    
        
    
                            # If significant anomalies are detected, reduce safety score
    
        
    
            
    
        
    
                            if any(key in ['high_jerk', 'velocity_inconsistency', 'high_steering_rate'] for key in self.anomalies.keys()):
    
        
    
            
    
        
    
                                # Add penalty based on anomalies
    
        
    
            
    
        
    
                                self.overall_safety_score *= 0.8  # Reduce score by 20% if anomalies detected
    
        
    
            
    
        
    
                                self.confidence_degraded = True
    
        
    
            
    
        
    
                                if self.overall_safety_score < 0.3:  # If safety score becomes critical
    
        
    
            
    
        
    
                                    self.requires_intervention = True
    
        
    
            
    
        
    
                        
    
        
    
            
    
        
    
                        return self.overall_safety_score, self.requires_intervention, safety_report
    
        
    
            
    
        
    
                
    
        
    
            
    
        
    
    def update(self, model_v2_msg, model_v2_mono_time, radar_state_msg, radar_state_mono_time, car_state_msg, car_state_mono_time, car_control_msg, live_pose_msg=None, live_pose_mono_time=None, driver_monitoring_state_msg=None, driver_monitoring_state_mono_time=None, gps_location_msg=None, gps_location_mono_time=None) -> Tuple[float, bool, Dict]:
        """Main update function - processes all inputs and returns safety assessment"""
        current_time = time.monotonic() * 1e9 # Get current time once in nanoseconds. time.monotonic returns seconds.
        start_time_update = time.monotonic() # Start timing for the entire update method


        try:
            self._initialize_health_flags() # Initialize health flags


            self._perform_staleness_checks(current_time, model_v2_mono_time, radar_state_mono_time, car_state_mono_time, live_pose_mono_time, driver_monitoring_state_mono_time, gps_location_mono_time)


            self._run_anomaly_detection(car_state_msg, model_v2_msg, radar_state_msg)


            # --- Update Confidence Measures with Error Handling and Decay ---


            self._update_all_confidences(current_time, model_v2_msg, radar_state_msg, car_state_msg, car_control_msg, live_pose_msg, gps_location_msg, gps_location_mono_time)


            self._calculate_safety_outputs(model_v2_msg, car_state_msg, driver_monitoring_state_msg)
    
        
    
            
    
        
    
                
    
        
    
            
    
        
    
            # Return the safety assessment and report


            return self._prepare_safety_report()


        except Exception as e:
            import logging
            import traceback
            logging.error(f"Critical error in SafetyMonitor.update: {e}")
            logging.error(f"Traceback: {traceback.format_exc()}")

            # Return safe defaults in case of critical error
            safety_report = {
            'model_confidence': 0.1,
            'radar_confidence': 0.1,
            'camera_confidence': 0.1,
            'imu_confidence': 0.1,
            'lighting_condition': 'normal',
            'weather_condition': 'clear',
            'road_condition': 'dry',
            'curve_anticipation_active': False,
            'curve_anticipation_score': 0.0,
            'lane_deviation': 0.0,
            'overall_safety_score': 0.1,  # Critical low safety score
            'confidence_degraded': True,
            'monitoring_cycles': self.monitoring_cycles,
            'error_occurred': True,
            'error_details': str(e),
            'anomalies_detected': {}
            }
            self.overall_safety_score = 0.1
            self.requires_intervention = True
            self.confidence_degraded = True
            self.monitoring_cycles += 1

            return self.overall_safety_score, self.requires_intervention, safety_report