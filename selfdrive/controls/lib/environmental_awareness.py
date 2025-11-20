#!/usr/bin/env python3
"""
Environmental awareness module for sunnypilot
Detects and responds to environmental conditions like weather, lighting, etc.
"""
import numpy as np
import threading
from cereal import log
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants
from .autonomous_params import ENVIRONMENTAL_PARAMS

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
            if hasattr(meta, 'lanelessProbs'):
                # If laneless probability is high, indicates poor visibility
                if len(meta.lanelessProbs) > 1 and meta.lanelessProbs[1] > 0.7:
                    self.low_visibility = True
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
        risk_score = 0.0

        # Weather condition risk
        if self.is_rainy:
            risk_score += RAIN_RISK_FACTOR
        if self.is_snowy:
            risk_score += SNOW_RISK_FACTOR
        if self.low_visibility:
            risk_score += VISIBILITY_RISK_FACTOR

        # Time of day risk
        if self.is_night:
            risk_score += NIGHT_RISK_FACTOR

        # Model uncertainty risk
        model_uncertainty = (1.0 - self.weather_confidence) * MODEL_UNCERTAINTY_RISK_FACTOR
        risk_score = max(risk_score, model_uncertainty)

        # Road condition risk
        road_quality_factor = (1.0 - self.model_road_quality) * ROAD_QUALITY_RISK_FACTOR
        risk_score = max(risk_score, road_quality_factor)

        # Surface condition risk
        surface_factor = (1.0 - self.model_surface_condition) * SURFACE_CONDITION_RISK_FACTOR
        risk_score = max(risk_score, surface_factor)

        # Combine with speed and curvature for dynamic risk
        # Quadratic scaling for speed: risk ∝ v² to reflect kinetic energy relationship
        speed_factor = (v_ego / SPEED_NORMALIZATION_BASELINE) ** 2  # Normalize to baseline
        # Apply speed factor with saturation to prevent excessive risk scores
        risk_score = min(1.0, risk_score * (1.0 + RISK_SPEED_FACTOR * speed_factor))

        # High curvature ahead increases risk in poor conditions
        if curvature_ahead > 0.008:  # Sharp curves
            risk_score = min(1.0, risk_score * RISK_CURVATURE_FACTOR)

        return min(1.0, risk_score)
    
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
                validated_data[key] = bool(weather_data[key])  # Ensure boolean type
            else:
                validated_data[key] = False  # Default to False if not provided

        # Validate floating point values with ranges
        if 'weather_confidence' in weather_data:
            confidence = float(weather_data['weather_confidence'])
            validated_data['weather_confidence'] = max(0.0, min(1.0, confidence))  # Clamp to [0, 1]
        else:
            validated_data['weather_confidence'] = 0.8  # Default confidence

        if 'road_quality' in weather_data:
            road_quality = float(weather_data['road_quality'])
            validated_data['road_quality'] = max(0.0, min(1.0, road_quality))  # Clamp to [0, 1]
        else:
            validated_data['road_quality'] = 1.0  # Default to good quality

        if 'surface_condition' in weather_data:
            surface_condition = float(weather_data['surface_condition'])
            validated_data['surface_condition'] = max(0.0, min(1.0, surface_condition))  # Clamp to [0, 1]
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