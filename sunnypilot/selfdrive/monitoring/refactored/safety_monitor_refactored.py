"""
Refactored Safety Monitoring for sunnypilot - Main orchestrator

This version addresses the complexity concerns by using separated modules
while maintaining all safety functionality.

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import logging
import time
from typing import Dict, Tuple, Optional
from collections import deque

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params

from .anomaly_detector import AdvancedAnomalyDetector
from .environmental_detector import EnvironmentalConditionDetector
from .confidence_calculator import ConfidenceCalculator
from .curve_anticipation import CurveAnticipation
from .safety_validator import SafetyValidator


class SafetyMonitor:
    """
    Enhanced Safety Monitoring System for Autonomous Driving
    Now using separated modules to maintain single-responsibility principle
    """
    
    def __init__(self):
        # Initialize separated modules
        self.anomaly_detector = AdvancedAnomalyDetector()
        self.environmental_detector = EnvironmentalConditionDetector()
        self.confidence_calculator = ConfidenceCalculator()
        self.curve_anticipation = CurveAnticipation(
            max_lat_accel=self.confidence_calculator.max_lat_accel,
            max_anticipation_distance=200.0
        )
        self.safety_validator = SafetyValidator()
        
        # Initialize system state variables
        self.params = Params()
        
        # Environmental condition tracking
        self.lighting_condition = "normal"
        self.weather_condition = "clear"
        self.road_condition = "dry"
        self.in_tunnel = False

        # Confidence tracking
        self.model_confidence = 1.0
        self.radar_confidence = 1.0
        self.camera_confidence = 1.0
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
        self.imu_healthy = True
        self.driver_monitoring_healthy = True
        self.gps_healthy = True

        # System safety state
        self.overall_safety_score = 1.0
        self.requires_intervention = False
        self.confidence_degraded = False

        # Performance metrics
        self.monitoring_cycles = 0
        self.anomalies = {}

        # Store last update times for staleness checks
        self.last_model_time = 0
        self.last_radar_time = 0
        self.last_camera_time = 0
        self.last_imu_time = 0
        self.last_driver_monitoring_time = 0
        self.last_gps_time = 0

        # Store last valid confidence for decay
        self.last_valid_model_confidence = 1.0
        self.last_valid_radar_confidence = 1.0
        self.last_valid_camera_confidence = 1.0
        self.last_valid_imu_confidence = 1.0
        self.last_valid_driver_monitoring_awareness = 1.0
        self.last_valid_gps_confidence = 1.0

        # Initialize safety score hysteresis
        self.previous_safety_score = 1.0  # Start with high safety score

    def _initialize_health_flags(self) -> None:
        """Initializes all sensor health flags to True at the start of each update cycle."""
        self.model_healthy = True
        self.radar_healthy = True
        self.camera_healthy = True
        self.imu_healthy = True
        self.driver_monitoring_healthy = True
        self.gps_healthy = True

    def _perform_staleness_checks(self, current_time: float, model_v2_mono_time: int, radar_state_mono_time: int, 
                                 car_state_mono_time: int, live_pose_mono_time: Optional[int], 
                                 driver_monitoring_state_mono_time: Optional[int], 
                                 gps_location_mono_time: Optional[int]) -> None:
        """Performs staleness checks for all sensors and updates their healthy flags."""
        threshold = self.confidence_calculator.STALENESS_THRESHOLD_SECONDS
        
        # Model Staleness
        time_since_model_update = (current_time - model_v2_mono_time) * 1e-9
        if time_since_model_update > threshold:
            self.model_healthy = False
            logging.warning(f"Model data is stale. Last update: {time_since_model_update:.2f}s ago.")
        else:
            self.model_healthy = True
            self.last_model_time = model_v2_mono_time

        # Radar Staleness
        time_since_radar_update = (current_time - radar_state_mono_time) * 1e-9
        if time_since_radar_update > threshold:
            self.radar_healthy = False
            logging.warning(f"Radar data is stale. Last update: {time_since_radar_update:.2f}s ago.")
        else:
            self.radar_healthy = True
            self.last_radar_time = radar_state_mono_time

        # Camera (carState is used as a proxy for freshness of car data influencing camera confidence)
        time_since_car_state_update = (current_time - car_state_mono_time) * 1e-9
        if time_since_car_state_update > threshold:
            self.camera_healthy = False
            logging.warning(f"CarState data (influencing camera confidence) is stale. Last update: {time_since_car_state_update:.2f}s ago.")
        else:
            self.camera_healthy = True
            self.last_camera_time = car_state_mono_time

        # IMU Staleness (livePose)
        if live_pose_mono_time is not None:
            time_since_imu_update = (current_time - live_pose_mono_time) * 1e-9
            if time_since_imu_update > threshold:
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
            if time_since_dm_update > threshold:
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
            if time_since_gps_update > threshold:
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

    def _update_all_confidences(self, current_time: float, model_v2_msg, radar_state_msg, car_state_msg, 
                               car_control_msg, live_pose_msg, gps_location_msg, 
                               gps_location_mono_time: Optional[int]) -> None:
        """Updates confidence measures for all sensors and environmental conditions."""

        # Update individual confidences
        self.model_confidence = self.confidence_calculator.update_model_confidence(model_v2_msg)
        self.radar_confidence = self.confidence_calculator.update_radar_confidence(radar_state_msg, car_state_msg)
        self.camera_confidence = self.confidence_calculator.update_camera_confidence(model_v2_msg, car_state_msg)
        
        # Update raw model confidence for immediate safety decisions
        if hasattr(model_v2_msg, 'meta') and hasattr(model_v2_msg.meta, 'confidence'):
            self.raw_model_confidence = model_v2_msg.meta.confidence if model_v2_msg.meta.confidence else 1.0
        else:
            self.raw_model_confidence = 0.1

        # Update environmental conditions
        if self.imu_healthy and live_pose_msg is not None:
            try:
                environmental_conditions, environmental_confidences = self.environmental_detector.detect_conditions(
                    model_v2_msg, live_pose_msg, car_state_msg, gps_location_msg)
                
                # Update internal state with detected conditions
                self.lighting_condition = environmental_conditions.get('lighting', self.lighting_condition)
                self.weather_condition = environmental_conditions.get('weather', self.weather_condition)
                self.road_condition = environmental_conditions.get('road_condition', self.road_condition)
                
                self.lighting_confidence = environmental_confidences.get('lighting', 0.0)
                self.weather_confidence = environmental_confidences.get('weather', 0.0)
                self.road_confidence = environmental_confidences.get('road_condition', 0.0)
                
                # Update tunnel detection
                self._update_tunnel_detection(gps_location_msg, car_state_msg, model_v2_msg)
                
            except Exception as e:
                logging.error(f"Error in environmental detection: {e}")
                self.imu_healthy = False
        else:
            # Apply confidence decay if IMU is unhealthy
            time_since_update = (current_time - self.last_imu_time) * 1e-9
            decay_factor = max(0.0, 1.0 - self.confidence_calculator.sensor_confidence_decay_rate * time_since_update)
            self.imu_confidence = self.last_valid_imu_confidence * decay_factor
            logging.warning(f"IMU confidence decayed to {self.imu_confidence:.2f} due to staleness.")

        # Update GPS confidence
        self.environmental_detector.assess_gps_signal_quality(gps_location_msg)
        self.gps_confidence = self.environmental_detector.gps_confidence

    def _update_tunnel_detection(self, gps_location_msg, car_state_msg, model_v2_msg):
        """Improved tunnel detection using multiple indicators"""
        gps_signal_lost = self.environmental_detector.gps_signal_lost
        gps_confidence_low = self.environmental_detector.gps_confidence < 0.5

        # Check if lighting is dark, but we need to be more careful not to rely on lane line strength
        lighting_appears_dark = (self.lighting_condition in ['dark', 'unknown'] and
                                self.lighting_confidence > self.confidence_calculator.lighting_confidence_threshold)

        # Use GPS time to determine if it should be daylight
        import datetime
        is_night_time = False
        if gps_location_msg is not None and gps_location_msg.unixTimestampMillis > 0:
            gps_time = datetime.datetime.fromtimestamp(gps_location_msg.unixTimestampMillis / 1000)
            hour = gps_time.hour
            is_night_time = hour >= 19 or hour <= 6

        # More robust lighting condition for tunnel detection:
        # If it's GPS daytime but lighting system says it's dark, that's a strong tunnel indicator
        gps_suggests_daylight = not is_night_time
        lighting_suggests_darkness = lighting_appears_dark
        gps_dark_lighting_conflict = gps_suggests_daylight and lighting_suggests_darkness

        # Check for vision-based tunnel indicators (absence of sky)
        vision_tunnel_indicators = 0
        if hasattr(model_v2_msg, 'meta') and hasattr(model_v2_msg.meta, 'upperLaneLineProbs'):
            if len(model_v2_msg.meta.upperLaneLineProbs) > 0:
                avg_upper_line_prob = sum(model_v2_msg.meta.upperLaneLineProbs) / len(model_v2_msg.meta.upperLaneLineProbs)
                # Higher upper line probabilities indicate less sky visibility (potential tunnel)
                if avg_upper_line_prob > 0.8:
                    vision_tunnel_indicators += 1

        # Enhanced tunnel detection combining GPS, lighting, and vision indicators
        tunnel_conditions = 0

        # GPS signal loss is a strong indicator
        if gps_signal_lost:
            tunnel_conditions += 1

        # Lighting contradiction (GPS says daylight but appears dark)
        if gps_dark_lighting_conflict:
            tunnel_conditions += 1

        # Dark lighting with high confidence
        if lighting_appears_dark:
            tunnel_conditions += 1

        # Vision-based tunnel indicators
        if vision_tunnel_indicators > 0:
            tunnel_conditions += 1

        # Set tunnel state based on multiple indicators
        if tunnel_conditions >= 2:  # At least 2 out of 4 indicators
            self.in_tunnel = True
        elif tunnel_conditions == 0 and not gps_signal_lost and self.environmental_detector.gps_confidence > 0.7:
            # If no indicators and GPS signal is good, definitely not a tunnel
            self.in_tunnel = False
        # Otherwise, maintain previous state to avoid flickering (hysteresis)

    def _calculate_safety_outputs(self, model_v2_msg, car_state_msg, driver_monitoring_state_msg) -> None:
        """Calculates overall safety score and intervention status"""

        # Detect curve anticipation
        self.curve_anticipation_active, self.curve_anticipation_score = self.curve_anticipation.detect_and_anticipate_curve(
            model_v2_msg, car_state_msg)

        # Calculate overall safety with error handling
        try:
            self.overall_safety_score = self.calculate_overall_safety_score(car_state_msg, driver_monitoring_state_msg)
        except Exception as e:
            logging.error(f"Error in calculate_overall_safety_score: {e}")
            self.overall_safety_score = 0.5

        try:
            self.requires_intervention = self.evaluate_safety_intervention_needed(
                self.overall_safety_score, car_state_msg, driver_monitoring_state_msg)
        except Exception as e:
            logging.error(f"Error in evaluate_safety_intervention_needed: {e}")
            self.requires_intervention = True  # Default to intervention on error

        # Determine if confidence is degraded (requires more conservative driving)
        self.confidence_degraded = self.overall_safety_score < 0.6

    def calculate_overall_safety_score(self, car_state_msg, driver_monitoring_state_msg) -> float:
        """Calculate overall safety score based on all inputs"""
        # Get sensor confidences with health status
        model_conf_val = self.model_confidence if self.model_healthy else 0.1
        radar_conf_val = self.radar_confidence if self.radar_healthy else 0.1
        camera_conf_val = self.camera_confidence if self.camera_healthy else 0.1
        imu_conf_val = self.imu_confidence if self.imu_healthy else 0.1

        # Get environmental conditions and confidences
        environmental_conditions = {
            'lighting': self.lighting_condition,
            'weather': self.weather_condition, 
            'road_condition': self.road_condition
        }
        environmental_confidences = {
            'lighting': self.lighting_confidence,
            'weather': self.weather_confidence,
            'road_condition': self.road_confidence
        }

        # Calculate sensor weights based on environmental conditions
        adjusted_weights = self.confidence_calculator.calculate_sensor_weights(
            environmental_conditions, environmental_confidences)

        # Calculate safety score using adjusted weights and confidence values
        safety_score = (
            model_conf_val * adjusted_weights['model'] +
            camera_conf_val * adjusted_weights['camera'] +
            radar_conf_val * adjusted_weights['radar'] +
            imu_conf_val * adjusted_weights['imu']
        )

        # Apply critical safety penalties for values significantly below thresholds
        model_confidence_threshold_adjusted = (self.confidence_calculator.model_confidence_threshold * 
                                             self.confidence_calculator.model_confidence_threshold_multiplier)
        if self.raw_model_confidence < model_confidence_threshold_adjusted:
            # Apply additional penalty when raw model confidence is critically low
            confidence_deficit = (model_confidence_threshold_adjusted - self.raw_model_confidence) / model_confidence_threshold_adjusted
            safety_penalty = min(0.3, confidence_deficit)  # Max 30% reduction for very low confidence
            safety_score *= (1.0 - safety_penalty)

        # Apply curve anticipation safety factor
        if self.curve_anticipation_active and car_state_msg.vEgo > 10.0:
            # Reduce safety score when approaching curves at high speed
            curve_factor = max(0.5, 1.0 - (self.curve_anticipation_score * 2.0))
            safety_score *= curve_factor

        # Apply anomaly-based penalties
        if self.anomalies:
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
        if self.model_confidence < (self.confidence_calculator.model_confidence_threshold * 
                                   self.confidence_calculator.model_confidence_threshold_multiplier):
            intervention_needed = True

        # If any critical sensor is unhealthy, intervention is more likely
        if not self.model_healthy or not self.radar_healthy or not self.camera_healthy:
            logging.warning("Critical sensor unhealthy, increasing likelihood of intervention.")
            intervention_needed = True

        # Driver monitoring: critical awareness or unhealthy system leads to intervention
        if not self.driver_monitoring_healthy:
            logging.warning("Driver monitoring system unhealthy. Intervention needed.")
            intervention_needed = True
        elif (driver_monitoring_state_msg is not None and 
              hasattr(driver_monitoring_state_msg, 'awarenessStatus') and 
              driver_monitoring_state_msg.awarenessStatus is not None):
            if driver_monitoring_state_msg.awarenessStatus < 0.2:  # Very low awareness, immediate intervention
                logging.warning(f"Very low driver awareness ({driver_monitoring_state_msg.awarenessStatus:.2f}). Intervention needed.")
                intervention_needed = True

        # Overall safety score too low
        if safety_score < 0.3:
            intervention_needed = True

        # At very low speeds, be more conservative with intervention
        if car_state_msg.vEgo < 5.0 and safety_score < self.confidence_calculator.low_speed_safety_threshold:
            intervention_needed = True

        # Check for critical anomalies that require intervention
        if self.anomalies:
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

    def _prepare_safety_report(self) -> Tuple[float, bool, Dict]:
        """Prepares the final safety report"""
        safety_report = {
            'model_confidence': self.model_confidence,
            'radar_confidence': self.radar_confidence,
            'camera_confidence': self.camera_confidence,
            'imu_confidence': self.imu_confidence,
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
            'lane_deviation': getattr(self.confidence_calculator.lane_deviation_filter, 'x', 0.0),
            'overall_safety_score': self.overall_safety_score,
            'confidence_degraded': self.confidence_degraded,
            'monitoring_cycles': self.monitoring_cycles,
            'anomalies_detected': self.anomalies,
            'in_tunnel': self.in_tunnel
        }

        self.monitoring_cycles += 1

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

    def update(self, model_v2_msg, model_v2_mono_time, radar_state_msg, radar_state_mono_time, 
               car_state_msg, car_state_mono_time, car_control_msg, 
               live_pose_msg=None, live_pose_mono_time=None, 
               driver_monitoring_state_msg=None, driver_monitoring_state_mono_time=None, 
               gps_location_msg=None, gps_location_mono_time=None) -> Tuple[float, bool, Dict]:
        """Main update function - processes all inputs and returns safety assessment"""
        current_time = time.monotonic() * 1e9  # Get current time once in nanoseconds

        try:
            self._initialize_health_flags()  # Initialize health flags

            self._perform_staleness_checks(current_time, model_v2_mono_time, radar_state_mono_time, 
                                         car_state_mono_time, live_pose_mono_time, 
                                         driver_monitoring_state_mono_time, gps_location_mono_time)

            self._run_anomaly_detection(car_state_msg, model_v2_msg, radar_state_msg)

            # Update Confidence Measures with Error Handling and Decay
            self._update_all_confidences(current_time, model_v2_msg, radar_state_msg, car_state_msg, 
                                       car_control_msg, live_pose_msg, gps_location_msg, gps_location_mono_time)

            self._calculate_safety_outputs(model_v2_msg, car_state_msg, driver_monitoring_state_msg)

            # Return the safety assessment and report
            return self._prepare_safety_report()

        except Exception as e:
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
                'anomalies_detected': {},
                'in_tunnel': False
            }
            self.overall_safety_score = 0.1
            self.requires_intervention = True
            self.confidence_degraded = True
            self.monitoring_cycles += 1

            return self.overall_safety_score, self.requires_intervention, safety_report