"""
Environmental Condition Detection for sunnypilot - Separated module

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from typing import Tuple, Dict
import numpy as np


class EnvironmentalConditionDetector:
    """
    Environmental condition detection using multi-sensor fusion
    Separated from main SafetyMonitor to reduce complexity
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
        """Detect environmental conditions using multi-sensor fusion"""
        conditions = {}
        confidences = {}

        # Advanced lighting detection using modelV2 scene analysis
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
        """Assess lighting conditions using GPS and vision data"""
        from datetime import datetime

        lighting_condition = "normal"  # Default assumption
        lighting_confidence = 0.5  # Base confidence

        # Initialize confidence contribution counters
        valid_indicators = 0
        total_confidence = 0.0

        # Indicator 1: Time of day based on system time / GPS time (for nighttime detection)
        # Prioritize GPS time for accuracy
        is_night_time = False
        if gps_location_msg is not None and gps_location_msg.unixTimestampMillis > 0:
            gps_time = datetime.fromtimestamp(gps_location_msg.unixTimestampMillis / 1000)
            hour = gps_time.hour
            is_night_time = hour >= 19 or hour <= 6 # Night time is roughly 7 PM to 6 AM
        else:
            # Fallback to system time if GPS time is not available
            current_time_dt = datetime.now()
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

        # Indicator 2: Vision-based lighting analysis using modelV2 scene data
        # Check for absence of sky as an indicator of tunnels (important for tunnel detection)
        sky_present = False
        sky_confidence = 0.0
        if hasattr(model_v2, 'roadEdges') and len(model_v2.roadEdges) > 1:
            # If we have both left and right road edges, we might be in an enclosed area like a tunnel
            # The absence of upper visual cues like sky can indicate tunnel
            if hasattr(model_v2, 'meta') and hasattr(model_v2.meta, 'upperLaneLineProbs'):
                # If upper lane line probabilities are consistently high, it might indicate absence of sky
                # This can be a tunnel indicator
                if len(model_v2.meta.upperLaneLineProbs) > 0:
                    avg_upper_line_prob = sum(model_v2.meta.upperLaneLineProbs) / len(model_v2.meta.upperLaneLineProbs)
                    # Higher upper line probabilities might indicate less sky visibility (potential tunnel)
                    if avg_upper_line_prob > 0.8:  # Threshold for potential tunnel indicator
                        sky_confidence = 0.7
                        sky_present = False  # No sky visible
                        # This is a strong indicator for tunnel when combined with other factors
                        if lighting_confidence > 0.6 and time_based_confidence > 0.6:
                            lighting_confidence = min(0.9, lighting_confidence + 0.1)  # Increase confidence in dark classification if sky not visible

        total_confidence += sky_confidence * 0.3  # Significant weight for sky detection
        if sky_confidence > 0.0:  # Only increment if we got a valid sky assessment
            valid_indicators += 1

        # Indicator 3: GPS-based lighting consistency check
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
        """Assess road conditions based on IMU data"""
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
                if hasattr(car_state.wheelSpeeds, 'fl') and hasattr(car_state.wheelSpeeds, 'fr') and \
                   hasattr(car_state.wheelSpeeds, 'rl') and hasattr(car_state.wheelSpeeds, 'rr'):

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
        """Assess weather conditions using sensor fusion"""
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