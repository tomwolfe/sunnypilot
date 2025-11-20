#!/usr/bin/env python3
"""
Environmental awareness module for sunnypilot
Detects and responds to environmental conditions like weather, lighting, etc.
"""
import numpy as np
import math
import threading
from cereal import log
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants
from .autonomous_params import ENVIRONMENTAL_PARAMS
from .backup_safety import RedundantControlValidator

# Import environmental parameters
VISIBILITY_THRESHOLD = ENVIRONMENTAL_PARAMS['VISIBILITY_THRESHOLD']
WEATHER_CONFIDENCE_BASELINE = ENVIRONMENTAL_PARAMS['WEATHER_CONFIDENCE_BASELINE']
MODEL_UNCERTAINTY_FACTOR = ENVIRONMENTAL_PARAMS['MODEL_UNCERTAINTY_FACTOR']
ROAD_QUALITY_FACTOR = ENVIRONMENTAL_PARAMS['ROAD_QUALITY_FACTOR']
SURFACE_CONDITION_FACTOR = ENVIRONMENTAL_PARAMS['SURFACE_CONDITION_FACTOR']
SPEED_NORMALIZATION_BASELINE = ENVIRONMENTAL_PARAMS['SPEED_NORMALIZATION_BASELINE']
RISK_SPEED_FACTOR = ENVIRONMENTAL_PARAMS['RISK_SPEED_FACTOR']
HIGH_RISK_THRESHOLD = ENVIRONMENTAL_PARAMS['HIGH_RISK_THRESHOLD']
MEDIUM_RISK_THRESHOLD = ENVIRONMENTAL_PARAMS['MEDIUM_RISK_THRESHOLD']
RISK_CURVATURE_FACTOR = ENVIRONMENTAL_PARAMS['RISK_CURVATURE_FACTOR']
RAIN_RISK_FACTOR = ENVIRONMENTAL_PARAMS['RAIN_RISK_FACTOR']
SNOW_RISK_FACTOR = ENVIRONMENTAL_PARAMS['SNOW_RISK_FACTOR']
VISIBILITY_RISK_FACTOR = ENVIRONMENTAL_PARAMS['VISIBILITY_RISK_FACTOR']
NIGHT_RISK_FACTOR = ENVIRONMENTAL_PARAMS['NIGHT_RISK_FACTOR']
MODEL_UNCERTAINTY_RISK_FACTOR = ENVIRONMENTAL_PARAMS['MODEL_UNCERTAINTY_RISK_FACTOR']
ROAD_QUALITY_RISK_FACTOR = ENVIRONMENTAL_PARAMS['ROAD_QUALITY_RISK_FACTOR']
SURFACE_CONDITION_RISK_FACTOR = ENVIRONMENTAL_PARAMS['SURFACE_CONDITION_RISK_FACTOR']


class EnvironmentalConditionMonitor:
    """
    Monitors environmental conditions that affect autonomous driving
    including weather, lighting, visibility, and road conditions
    """
    
    def __init__(self):
        # Initialize environmental condition flags
        self.is_rainy = False
        self.is_snowy = False
        self.is_night = False
        self.low_visibility = False
        self.weather_confidence = 0.0
        self.lighting_condition = 0.5  # 0.0 dark, 1.0 bright
        
        # Track model-based environmental indicators
        self.model_road_quality = 1.0  # 1.0 is good, lower is poor
        self.model_surface_condition = 1.0  # 1.0 is dry, lower indicates wet/slippery
        
        # Track recent environmental readings
        self.visibility_buffer = []
        self.weather_buffer = []
        self.lighting_buffer = []
        
    def update_from_model(self, model_v2_msg):
        """
        Update environmental awareness from model data
        """
        # Extract environmental indicators from model metadata
        if hasattr(model_v2_msg, 'meta'):
            meta = model_v2_msg.meta
            
            # Update lighting condition based on model confidence and image quality indicators
            if hasattr(meta, 'confidence'):
                self.weather_confidence = meta.confidence
                
            # Check for indicators of poor conditions in model outputs
            # For example, if line detection confidence is low, might indicate bad weather
            if hasattr(meta, 'lanelessProbs') and hasattr(meta, 'laneLineProbs'):
                # If laneless probability is high AND lane line probabilities are low, indicates poor visibility
                # If laneless probability is high BUT lane line probabilities are high, indicates actual laneless road
                laneless_prob = meta.lanelessProbs[1] if len(meta.lanelessProbs) > 1 else 0.0
                avg_lane_prob = 0.0

                # Calculate average lane line probability to distinguish between poor visibility vs laneless road
                if hasattr(meta, 'laneLineProbs') and len(getattr(meta, 'laneLineProbs', [])) > 0:
                    lane_line_probs = getattr(meta, 'laneLineProbs', [])
                    if lane_line_probs:
                        avg_lane_prob = sum(lane_line_probs) / len(lane_line_probs)

                # Poor visibility: high laneless prob AND low lane line prob
                # Laneless road: high laneless prob BUT high lane line prob (would mean system recognizes it's laneless)
                if laneless_prob > 0.7 and avg_lane_prob < 0.3:
                    self.low_visibility = True
                else:
                    self.low_visibility = False
            elif hasattr(meta, 'lanelessProbs'):
                # Fallback: if we only have lanelessProbs, be more conservative but add context
                if len(meta.lanelessProbs) > 1 and meta.lanelessProbs[1] > 0.8:  # Higher threshold for conservative assumption
                    # Only assume low visibility if we also have other indicators (like poor path confidence)
                    path_stds = getattr(meta, 'pathStds', [])
                    if path_stds and len(path_stds) > 5:
                        avg_path_std = sum(path_stds[:5]) / min(5, len(path_stds))
                        if avg_path_std > 0.4:  # High uncertainty in path prediction
                            self.low_visibility = True
                        else:
                            self.low_visibility = False
                    else:
                        self.low_visibility = False  # Default to not low visibility if no additional data
                else:
                    self.low_visibility = False
            
            # Analyze path confidence as indicator of visibility/road conditions
            if hasattr(meta, 'pathPoints') and hasattr(meta, 'pathStds'):
                # High standard deviation in path prediction might indicate poor road conditions
                path_stds = getattr(meta, 'pathStds', [])
                try:
                    if hasattr(path_stds, '__len__') and len(path_stds) > 10:
                        avg_std = sum(path_stds[:10]) / min(10, len(path_stds))
                        if avg_std > 0.5:  # High uncertainty
                            self.model_road_quality = max(0.3, 1.0 - avg_std)
                        else:
                            self.model_road_quality = min(1.0, 1.0 - avg_std * 0.3)
                except (TypeError, AttributeError):
                    # Handle Mock objects or other issues
                    pass
        
        # Analyze path data for road surface quality
        path_y = getattr(getattr(model_v2_msg, 'path', None), 'y', [])
        if hasattr(path_y, '__len__') and len(path_y) > 20:
                # Analyze path smoothness as indicator of road quality
                path_changes = []
                for i in range(1, min(20, len(path_y))):
                    change = abs(path_y[i] - path_y[i-1]) if i < len(path_y) else 0
                    path_changes.append(change)

                if path_changes:
                    avg_change = sum(path_changes) / len(path_changes)
                    # Lower smoothness indicates poorer road quality
                    self.model_surface_condition = max(0.2, min(1.0, 1.0 - avg_change * 5))
    
    def update_from_sensors(self, car_state, device_state, road_camera_state):
        """
        Update environmental awareness from vehicle sensors
        """
        # Use car state to estimate environmental conditions
        v_ego = car_state.vEgo
        
        # Check for night using ambient light sensor if available, or from device state
        # For now, using time-based estimation (this would be enhanced with actual sensors)
        if hasattr(device_state, 'ubloxGnss') and device_state.ubloxGnss:
            # Use GPS to determine if we're in daylight based on time and location
            # This is a simplified version - in reality would use more sophisticated calculations
            pass
        
        # Determine night condition based on camera settings if available
        if hasattr(road_camera_state, 'intensity'):
            # High camera intensity might indicate night/dark conditions
            if road_camera_state.intensity > 150:  # Threshold to be tuned
                self.is_night = True
            else:
                self.is_night = False
        
        # Update lighting condition based on camera gain/exposure
        if hasattr(road_camera_state, 'exposure') and road_camera_state.exposure > 0:
            # Higher exposure typically indicates darker conditions
            normalized_exposure = min(1.0, road_camera_state.exposure / 100.0)
            self.lighting_condition = max(0.1, 1.0 - normalized_exposure)

        # Visibility assessment based on camera data
        self.low_visibility = self.lighting_condition < VISIBILITY_THRESHOLD or not self.weather_confidence > (1.0 - MODEL_UNCERTAINTY_FACTOR)
    
    def detect_weather_conditions(self, model_v2_msg, car_state):
        """
        Detect weather-related conditions from multiple sources
        """
        # Initialize weather detection
        detected_conditions = []
        
        # Check model-based weather indicators
        if hasattr(model_v2_msg, 'temporalBatch'):
            # Analyze frame variations that might indicate precipitation
            # This is a simplified approach - real implementation would use more sophisticated analysis
            pass
        
        # Check for conditions that might indicate rain or snow
        # Based on model uncertainty, path prediction quality, etc.
        model_uncertainty = 1.0 - self.weather_confidence
        if model_uncertainty > 0.4 and self.model_surface_condition < 0.7:
            detected_conditions.append("poor_grip")
            
        # Check for low visibility conditions
        if self.low_visibility:
            detected_conditions.append("low_visibility")
        
        # Return weather status
        return {
            'is_rainy': 'poor_grip' in detected_conditions,
            'is_snowy': False,  # Simplified for now
            'is_low_visibility': 'low_visibility' in detected_conditions,
            'weather_confidence': self.weather_confidence,
            'road_quality': self.model_road_quality,
            'surface_condition': self.model_surface_condition
        }
    
    def get_environmental_risk_score(self, v_ego, curvature_ahead):
        """
        Calculate an overall environmental risk score
        :param v_ego: Current vehicle speed
        :param curvature_ahead: Anticipated curvature ahead
        :return: Risk score from 0.0 (low risk) to 1.0 (high risk)
        """
        # Base risk components
        weather_risk = 0.0
        condition_risk = 0.0
        dynamic_risk = 0.0

        # Calculate weather-related risk with non-linear interactions
        # Rain + low visibility is more dangerous than sum of individual risks
        weather_factors = []
        if self.is_rainy:
            weather_factors.append(RAIN_RISK_FACTOR)
        if self.is_snowy:
            weather_factors.append(SNOW_RISK_FACTOR)
        if self.low_visibility:
            weather_factors.append(VISIBILITY_RISK_FACTOR)
        if self.is_night:
            weather_factors.append(NIGHT_RISK_FACTOR)

        # Non-linear combination of weather factors (multiplicative effect for compounding risks)
        if len(weather_factors) > 0:
            # Use a formula that increases risk more significantly as multiple factors combine
            combined_weather_risk = sum(weather_factors)  # Base additive
            if len(weather_factors) > 1:  # Multiple factors
                # Apply compounding effect: multiply by factor based on number of conditions
                compounding_factor = 1.0 + (len(weather_factors) - 1) * 0.2  # 20% extra risk per additional factor
                combined_weather_risk = min(1.0, combined_weather_risk * compounding_factor)
            weather_risk = combined_weather_risk

        # Calculate condition-related risk (road quality, surface, model uncertainty)
        # Apply non-linear interactions between road quality and surface condition
        base_road_risk = (1.0 - self.model_road_quality) * ROAD_QUALITY_RISK_FACTOR
        base_surface_risk = (1.0 - self.model_surface_condition) * SURFACE_CONDITION_RISK_FACTOR

        # If both road quality and surface condition are poor, risk compounds
        if (self.model_road_quality < 0.6 and self.model_surface_condition < 0.6):
            condition_risk = min(1.0, (base_road_risk + base_surface_risk) * 1.5)  # 50% extra risk for both being poor
        else:
            condition_risk = base_road_risk + base_surface_risk

        # Model uncertainty risk
        model_uncertainty = (1.0 - self.weather_confidence) * MODEL_UNCERTAINTY_RISK_FACTOR
        condition_risk = max(condition_risk, model_uncertainty)

        # Calculate dynamic risk (speed and curvature interactions)
        # At high speeds, poor conditions become exponentially more dangerous
        base_speed_factor = (v_ego / SPEED_NORMALIZATION_BASELINE) ** 2  # Quadratic scaling
        speed_risk = min(0.5, RISK_SPEED_FACTOR * base_speed_factor)

        # High curvature ahead is especially risky when combined with poor conditions
        if abs(curvature_ahead) > 0.008:  # Sharp curves
            base_curvature_risk = min(0.3, abs(curvature_ahead) * RISK_CURVATURE_FACTOR)
            # If poor conditions exist, increase curvature risk
            if (weather_risk > 0.3 or condition_risk > 0.3):
                base_curvature_risk *= 1.5  # 50% extra risk for sharp curves in poor conditions
            dynamic_risk = base_curvature_risk

        # Combine all risk components with saturation
        # Use a formula that approaches 1.0 but never exceeds it
        total_risk = weather_risk + condition_risk + dynamic_risk
        # Apply saturation to prevent exceeding 1.0, with a smooth transition
        risk_score = total_risk / (1.0 + total_risk) if total_risk < float('inf') else 1.0

        # Additional safety check to ensure risk is in valid range
        risk_score = max(0.0, min(1.0, risk_score))

        return risk_score
    
    def get_adjusted_limits(self, original_limits, v_ego, curvature_ahead):
        """
        Get adjusted acceleration limits based on environmental conditions
        :param original_limits: Original [min_accel, max_accel] limits
        :param v_ego: Current speed
        :param curvature_ahead: Curvature ahead
        :return: Adjusted [min_accel, max_accel] limits
        """
        # Start with original limits
        min_accel, max_accel = original_limits
        
        # Get environmental risk score
        risk_score = self.get_environmental_risk_score(v_ego, curvature_ahead)
        
        # Apply safety reduction based on risk score
        safety_factor = 1.0 - (risk_score * 0.6)  # Reduce limits by up to 60%
        
        # Adjust acceleration limits
        adj_max_accel = max_accel * safety_factor
        adj_min_accel = min_accel * safety_factor  # For deceleration, keep magnitude but be more conservative
        
        # Additional adjustments for specific conditions
        if self.is_rainy:
            adj_max_accel *= 0.7  # Reduce max acceleration by 30% in rain
            adj_min_accel = min(adj_min_accel, min(-1.5, adj_min_accel * 0.85))  # More conservative braking in rain
        elif self.low_visibility:
            adj_max_accel *= 0.85  # Reduce acceleration by 15% in low visibility
            adj_min_accel = min(adj_min_accel, min(-2.0, adj_min_accel * 0.9))  # More conservative braking with low visibility
            
        if self.is_night:
            adj_max_accel *= 0.9  # Reduce acceleration by 10% at night
            adj_min_accel = min(adj_min_accel, adj_min_accel * 0.95)  # Slightly more conservative braking at night
            
        return [adj_min_accel, adj_max_accel]


class EnvironmentalConditionProcessor:
    """
    Processes environmental data and provides inputs to the control system
    """

    def __init__(self):
        self.monitor = EnvironmentalConditionMonitor()
        self.environmental_conditions = {
            'is_rainy': False,
            'is_snowy': False,
            'is_night': False,
            'low_visibility': False,
            'weather_confidence': 0.8,
            'road_quality': 1.0,
            'surface_condition': 1.0
        }
        # Add thread lock for safe access to shared state
        self._lock = threading.Lock()

        # Add redundant safety validation
        self.redundant_validator = RedundantControlValidator()
    
    def update(self, sm):
        """
        Update environmental conditions from various sources
        :param sm: SubMaster with messages from different services
        """
        with self._lock:  # Use lock to protect shared state access
            # Validate that required messages are available before processing
            required_messages = ['modelV2', 'carState', 'deviceState', 'roadCameraState']
            available_messages = [msg for msg in required_messages if sm.updated.get(msg, False)]

            if len(available_messages) < 3:  # At least 3 out of 4 required
                cloudlog.warning(f"EnvironmentalConditionProcessor: Insufficient data: {available_messages}")
                # Use conservative defaults when data is insufficient
                self._use_conservative_defaults()
                return

            # Update from model
            if sm.updated.get('modelV2', False):
                self.monitor.update_from_model(sm['modelV2'])

            # Update from sensors if available - add proper checks
            if (sm.updated.get('carState', False) and
                sm.updated.get('deviceState', False) and
                sm.updated.get('roadCameraState', False)):
                self.monitor.update_from_sensors(
                    sm['carState'],
                    sm['deviceState'],
                    sm['roadCameraState']
                )
            else:
                cloudlog.warning("EnvironmentalConditionProcessor: Missing required sensor data")
                # Use conservative defaults when sensor data is missing
                self._use_conservative_defaults()
                return

            # Detect and update weather conditions
            weather_data = self.monitor.detect_weather_conditions(sm['modelV2'], sm['carState'])

            # Validate the detected weather data before applying
            validated_weather_data = self._validate_weather_data(weather_data)
            self.environmental_conditions.update(validated_weather_data)

    def _use_conservative_defaults(self):
        """Set conservative defaults when sensor data is insufficient"""
        self.environmental_conditions.update({
            'is_rainy': True,  # Default to worst case
            'is_snowy': True,
            'is_night': True,
            'low_visibility': True,
            'weather_confidence': 0.5,  # Lower confidence
            'road_quality': 0.5,  # Medium quality
            'surface_condition': 0.6  # Slightly poor
        })

    def _validate_weather_data(self, weather_data):
        """Validate weather data before applying to ensure it's within reasonable ranges"""
        validated_data = {}

        # Validate boolean flags
        for key in ['is_rainy', 'is_snowy', 'is_night', 'low_visibility']:
            if key in weather_data:
                try:
                    validated_data[key] = bool(weather_data[key])  # Ensure boolean type
                except (TypeError, ValueError):
                    validated_data[key] = False  # Default to False if conversion fails
            else:
                validated_data[key] = False  # Default to False if not provided

        # Validate floating point values with ranges
        if 'weather_confidence' in weather_data:
            try:
                confidence = float(weather_data['weather_confidence'])
                # Validate confidence is a valid number and clamp to [0, 1]
                if not (0.0 <= confidence <= 1.0) or math.isnan(confidence) or math.isinf(confidence):
                    validated_data['weather_confidence'] = 0.8  # Default to safe value
                else:
                    validated_data['weather_confidence'] = confidence
            except (TypeError, ValueError):
                validated_data['weather_confidence'] = 0.8  # Default to safe value
        else:
            validated_data['weather_confidence'] = 0.8  # Default confidence

        if 'road_quality' in weather_data:
            try:
                road_quality = float(weather_data['road_quality'])
                # Validate road_quality is a valid number and clamp to [0, 1]
                if not (0.0 <= road_quality <= 1.0) or math.isnan(road_quality) or math.isinf(road_quality):
                    validated_data['road_quality'] = 0.7  # Default to moderate quality (conservative)
                else:
                    validated_data['road_quality'] = road_quality
            except (TypeError, ValueError):
                validated_data['road_quality'] = 0.7  # Default to moderate quality (conservative)
        else:
            validated_data['road_quality'] = 1.0  # Default to good quality

        if 'surface_condition' in weather_data:
            try:
                surface_condition = float(weather_data['surface_condition'])
                # Validate surface_condition is a valid number and clamp to [0, 1]
                if not (0.0 <= surface_condition <= 1.0) or math.isnan(surface_condition) or math.isinf(surface_condition):
                    validated_data['surface_condition'] = 0.7  # Default to moderate condition (conservative)
                else:
                    validated_data['surface_condition'] = surface_condition
            except (TypeError, ValueError):
                validated_data['surface_condition'] = 0.7  # Default to moderate condition (conservative)
        else:
            validated_data['surface_condition'] = 1.0  # Default to good condition

        return validated_data
    
    def get_environmental_limits(self, original_limits, v_ego, curvature_ahead):
        """
        Get acceleration limits adjusted for environmental conditions
        """
        with self._lock:  # Use lock to protect shared state access
            return self.monitor.get_adjusted_limits(original_limits, v_ego, curvature_ahead)

    def get_risk_score(self, v_ego, curvature_ahead):
        """
        Get overall environmental risk score
        """
        with self._lock:  # Use lock to protect shared state access
            return self.monitor.get_environmental_risk_score(v_ego, curvature_ahead)

    def should_reduce_speed(self, v_ego, desired_v_cruise):
        """
        Determine if speed should be reduced based on environmental conditions
        """
        with self._lock:  # Use lock to protect shared state access
            risk_score = self.get_risk_score(v_ego, 0.0)  # Current curvature not critical for speed reduction

            if risk_score > HIGH_RISK_THRESHOLD:  # High risk
                return desired_v_cruise * 0.7  # Reduce speed by 30%
            elif risk_score > MEDIUM_RISK_THRESHOLD:  # Medium risk
                return desired_v_cruise * 0.85  # Reduce speed by 15%
            else:
                return desired_v_cruise  # No reduction needed


def validate_environmental_risk(risk_score: float) -> float:
    """
    Validate environmental risk score to ensure it's in valid range [0.0, 1.0]
    :param risk_score: Risk score to validate
    :return: Risk score clamped to valid range
    """
    return max(0.0, min(1.0, float(risk_score)))