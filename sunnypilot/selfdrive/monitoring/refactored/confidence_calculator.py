"""
Confidence Calculator for sunnypilot - Separated module for sensor confidence calculations

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import logging
import numpy as np
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params, UnknownKeyName


class ConfidenceCalculator:
    """
    Calculate confidence scores for different sensors and systems
    Separated from main SafetyMonitor to reduce complexity
    """
    def __init__(self):
        # Initialize filters for smooth confidence calculations
        self.model_confidence_filter = FirstOrderFilter(1.0, 0.5, DT_MDL)
        self.radar_confidence_filter = FirstOrderFilter(1.0, 0.5, DT_MDL)
        self.lane_deviation_filter = FirstOrderFilter(0.0, 0.01, DT_MDL)  # Almost instantaneous for safety detection

        # Initialize params system to allow configurable thresholds
        self.params = Params()

        # Confidence thresholds for safety decisions - with configurable defaults
        self.model_confidence_threshold = self._get_param_with_default("ModelConfidenceThreshold", 0.7)
        self.radar_confidence_threshold = self._get_param_with_default("RadarConfidenceThreshold", 0.6)
        self.lane_deviation_threshold = self._get_param_with_default("LaneDeviationThreshold", 0.8)
        self.low_speed_safety_threshold = self._get_param_with_default("LowSpeedSafetyThreshold", 0.4)
        self.model_confidence_threshold_multiplier = self._get_param_with_default("ModelConfidenceThresholdMultiplier", 0.8)
        
        # Validate multiplier range
        if not (0.5 <= self.model_confidence_threshold_multiplier <= 1.0):
            logging.warning(f"ModelConfidenceThresholdMultiplier {self.model_confidence_threshold_multiplier} out of valid range [0.5, 1.0]. Using default 0.8")
            self.model_confidence_threshold_multiplier = 0.8

        # Environmental confidence thresholds - with configurable defaults
        self.lighting_confidence_threshold = self._get_param_with_default("LightingConfidenceThreshold", 0.7)
        self.weather_confidence_threshold = self._get_param_with_default("WeatherConfidenceThreshold", 0.7)
        self.road_confidence_threshold = self._get_param_with_default("RoadConfidenceThreshold", 0.7)

        # Curve anticipation parameters
        self.max_lat_accel = self._get_param_with_default("MaxLateralAcceleration", 2.0)
        self.curve_detection_threshold = self._get_param_with_default("CurveDetectionThreshold", 0.5)

        # Radar confidence parameters
        self.max_radar_distance_for_confidence = self._get_param_with_default("MaxRadarDistanceForConfidence", 150.0)
        self.velocity_confidence_scale = self._get_param_with_default("VelocityConfidenceScale", 30.0)
        self.speed_scaling_factor_for_radar_distance = self._get_param_with_default("SpeedScalingFactorForRadarDistance", 1.0)

        # Radar confidence parameters for acceleration
        self.acceleration_confidence_threshold = self._get_param_with_default("AccelerationConfidenceThreshold", 5.0)

        # Environmental weight adjustment factor for dynamic sensor weighting
        self.environmental_weight_adjustment_factor = self._get_param_with_default("EnvironmentalWeightAdjustmentFactor", 1.0)

        # Sensor confidence decay rate (per second of staleness)
        self.sensor_confidence_decay_rate = self._get_param_with_default("SensorConfidenceDecayRate", 0.1) # 10% decay per second

        # Sensor staleness threshold - if data is older than this, it's considered stale
        self.STALENESS_THRESHOLD_SECONDS = self._get_param_with_default("SensorStalenessThreshold", 0.5)

    def _get_param_with_default(self, param_name: str, default_value: float) -> float:
        """Helper to get parameter with default"""
        try:
            param_value = self.params.get(param_name)
            return float(param_value) if param_value else default_value
        except UnknownKeyName:
            return default_value
        except (TypeError, ValueError):
            return default_value

    def update_model_confidence(self, model_v2_msg) -> float:
        """Update model confidence based on neural network outputs"""
        if hasattr(model_v2_msg, 'meta') and hasattr(model_v2_msg.meta, 'confidence'):
            raw_confidence = model_v2_msg.meta.confidence if model_v2_msg.meta.confidence else 1.0
            filtered_confidence = self.model_confidence_filter.update(raw_confidence)
            return filtered_confidence
        else:
            logging.warning("modelV2 data not available. Defaulting model confidence to a conservative 0.1.")
            return 0.1

    def update_radar_confidence(self, radar_state_msg, car_state_msg) -> float:
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
            acceleration_confidence = 0.7 # Fallback to a moderate confidence if aRel is not available or None.
            if hasattr(lead, 'aRel') and lead.aRel is not None:
                acceleration_confidence = min(1.0, max(0.0, (self.acceleration_confidence_threshold - abs(lead.aRel)) / self.acceleration_confidence_threshold))
            else:
                logging.warning("aRel not available for radar acceleration confidence. Defaulting to a moderate 0.7.")
            filtered_confidence = self.radar_confidence_filter.update(
                (distance_confidence * 0.5 + velocity_confidence * 0.3 + acceleration_confidence * 0.2)
            )
            return filtered_confidence
        else:
            return 0.3  # Lower confidence when no lead detected

    def update_camera_confidence(self, model_v2_msg, car_state_msg) -> float:
        """Update camera confidence based on lane detection and visual clarity"""
        # Plausibility check for vEgo
        if not (0.0 <= car_state_msg.vEgo <= 45.0): # Assuming plausible speed range 0-162 km/h (0-45 m/s)
            logging.warning(f"CarState vEgo ({car_state_msg.vEgo:.2f} m/s) out of plausible range. Reducing camera confidence.")
            return 0.1

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
            return lane_confidence * speed_factor
        else:
            return 0.6

    def calculate_sensor_weights(self, environmental_conditions: dict, environmental_confidences: dict) -> dict:
        """Calculate sensor weights based on environmental conditions"""
        # Define initial base weights for each sensor
        base_weights = {
            'model': 0.35,  # Reduced from 0.4 to account for potential model failures
            'camera': 0.25,  # Reduced from 0.3 for better robustness in poor visibility
            'radar': 0.25,   # Increased from 0.2 since radar is more reliable in adverse conditions
            'imu': 0.15      # Increased from 0.1 for better stability input
        }

        # Initialize adjusted weights with base weights
        adjusted_weights = base_weights.copy()

        # Adjust weights based on environmental conditions and their confidence
        # The influence of environmental conditions is scaled by their detection confidence
        # and a configurable adjustment factor.

        # Lighting adjustments
        lighting_influence = self.environmental_weight_adjustment_factor * environmental_confidences.get('lighting', 0.0)
        lighting_condition = environmental_conditions.get('lighting', 'normal')
        if lighting_condition in ["night", "dawn_dusk", "dark", "tunnel"]:
            adjusted_weights['model'] *= (1.0 - 0.3 * lighting_influence)  # Reduce model weight
            adjusted_weights['camera'] *= (1.0 - 0.4 * lighting_influence) # More reduction for camera
            adjusted_weights['radar'] *= (1.0 + 0.2 * lighting_influence)  # Increase radar weight
        elif lighting_condition == "bright":
            adjusted_weights['camera'] *= (1.0 + 0.1 * lighting_influence) # Slightly increase camera weight

        # Road condition adjustments
        road_influence = self.environmental_weight_adjustment_factor * environmental_confidences.get('road_condition', 0.0)
        road_condition = environmental_conditions.get('road_condition', 'normal')
        if road_condition == "rough":
            adjusted_weights['model'] *= (1.0 - 0.1 * road_influence)
            adjusted_weights['camera'] *= (1.0 - 0.15 * road_influence)
        elif road_condition == "slippery":
            adjusted_weights['model'] *= (1.0 - 0.15 * road_influence)
            adjusted_weights['camera'] *= (1.0 - 0.2 * road_influence)
            adjusted_weights['radar'] *= (1.0 + 0.1 * road_influence)

        # Weather adjustments
        weather_influence = self.environmental_weight_adjustment_factor * environmental_confidences.get('weather', 0.0)
        weather_condition = environmental_conditions.get('weather', 'clear')
        if weather_condition in ["rain", "snow", "fog", "unknown", "poor_visibility"]:
            adjusted_weights['model'] *= (1.0 - 0.4 * weather_influence)
            adjusted_weights['camera'] *= (1.0 - 0.5 * weather_influence)
            adjusted_weights['radar'] *= (1.0 + 0.3 * weather_influence)
        elif weather_condition == "clear":
            adjusted_weights['model'] *= (1.0 + 0.05 * weather_influence)
            adjusted_weights['camera'] *= (1.0 + 0.05 * weather_influence)

        # Ensure weights sum to 1 after adjustments
        total_weight = sum(adjusted_weights.values())
        if total_weight > 0:
            for key in adjusted_weights:
                adjusted_weights[key] /= total_weight

        return adjusted_weights