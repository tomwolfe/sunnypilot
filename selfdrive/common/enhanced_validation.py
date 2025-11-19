"""
Enhanced validation system with sophisticated confidence-based decision trees
and situation-aware safety validation for Sunnypilot
"""

import numpy as np
from typing import Dict, Any, Tuple, List
from cereal import log
import cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.selfdrive.common.validation_publisher import validation_metrics_publisher
from openpilot.selfdrive.common.weather_data import weather_data_interface


class EnhancedSafetyValidator:
    """
    Enhanced validation system with sophisticated confidence-based decision trees
    and situation-aware safety validation for Sunnypilot
    """

    def __init__(self):
        self.params = Params()
        self.validation_thresholds = {
            'critical': 0.6,    # Minimum for any operation
            'normal': 0.7,      # Standard operation
            'aggressive': 0.85  # More responsive behavior
        }

        # Situation-based factors
        self.situation_factors = {
            'highway': {
                'speed_factor': 1.1,  # Higher confidence needed at high speeds
                'traffic_factor': 1.0,
                'weather_factor': 1.0
            },
            'city': {
                'speed_factor': 0.9,  # More cautious in city
                'traffic_factor': 0.8,  # More complex traffic
                'weather_factor': 0.9   # Weather more impactful
            },
            'curvy_road': {
                'speed_factor': 0.8,   # Extra caution on curves
                'traffic_factor': 0.9,
                'weather_factor': 0.8
            },
            'intersection': {
                'speed_factor': 0.7,    # Extra caution at intersections
                'traffic_factor': 0.6,  # Complex traffic patterns
                'weather_factor': 0.75
            },
            'highway_merge': {
                'speed_factor': 0.75,   # Caution during merges
                'traffic_factor': 0.7,  # High-speed traffic interaction
                'weather_factor': 0.8
            },
            'school_zone': {
                'speed_factor': 0.6,    # Extra caution around schools
                'traffic_factor': 0.6,
                'weather_factor': 0.7
            },
            'residential': {
                'speed_factor': 0.85,   # Moderate caution in neighborhoods
                'traffic_factor': 0.75,
                'weather_factor': 0.85
            }
        }

        # Scenario detection parameters
        self.intersection_detection = {
            'traffic_light_distance': 50.0,  # Distance to consider intersection (m)
            'stop_sign_distance': 30.0,      # Distance to consider stop sign (m)
            'intersection_angle_threshold': 45.0  # Angle threshold for intersection detection (degrees)
        }

        # Initialize messaging for receiving model outputs
        try:
            self.sm = messaging.SubMaster(['modelV2', 'carState', 'radarState', 'liveCalibration',
                                         'navInstruction', 'modelV2'], ignore_alive=True)
        except Exception as e:
            cloudlog.warning(f"Could not initialize SubMaster in EnhancedSafetyValidator: {e}")
            self.sm = None

        # Cache for scenario detection
        self.last_road_edge_positions = None
        self.road_curvature_history = []
        self.max_curvature_history = 10
    
    def detect_driving_scenario(self, model_output: Dict[str, Any],
                              car_state: log.CarState,
                              nav_instruction: Any = None) -> str:
        """
        Detect the current driving scenario based on model outputs, car state, and navigation

        Args:
            model_output: Model outputs with lane lines, path, etc.
            car_state: Current car state
            nav_instruction: Navigation instruction if available

        Returns:
            Detected scenario string
        """
        # Start with default highway assumption
        scenario = 'highway'

        # Get vehicle speed
        v_ego = car_state.vEgo

        # Detect if in city based on speed and environment
        if v_ego < 15:  # ~54 km/h
            # Check for city characteristics
            city_indicators = 0

            # Check for dense lead vehicles (city traffic)
            if 'leads_v3' in model_output and model_output['leads_v3']:
                close_leads = [lead for lead in model_output['leads_v3'] if lead.dRel < 30 and lead.prob > 0.5]
                if len(close_leads) > 1:
                    city_indicators += 1

            # Check for frequent steering (city driving)
            if hasattr(car_state, 'steeringAngleDeg') and abs(car_state.steeringAngleDeg) > 10:
                city_indicators += 1

            if city_indicators >= 1:
                scenario = 'city'

        # Detect curves based on road model
        if 'laneLines' in model_output and len(model_output['laneLines']) >= 2:
            # Check curvature in path
            if 'position' in model_output and hasattr(model_output['position'], 'y'):
                # Calculate approximate curvature from path points
                path_points = model_output['position'].y if hasattr(model_output['position'], 'y') else []
                if len(path_points) > 5:
                    # Estimate curvature from the path
                    y_vals = path_points[:10]  # First 10 points should be sufficient
                    if len(y_vals) >= 3:
                        # Estimate curvature using second derivative approximation
                        curve_measures = []
                        for i in range(1, len(y_vals)-1):
                            curvature = abs(y_vals[i-1] - 2*y_vals[i] + y_vals[i+1])
                            curve_measures.append(curvature)

                        if curve_measures:
                            avg_curvature = np.mean(curve_measures)
                            self.road_curvature_history.append(avg_curvature)
                            if len(self.road_curvature_history) > self.max_curvature_history:
                                self.road_curvature_history.pop(0)

                            if avg_curvature > 0.05:  # Threshold for curve detection
                                scenario = 'curvy_road'

        # Detect intersections using navigation and model data
        if nav_instruction and hasattr(nav_instruction, ' maneuverType'):
            maneuver_str = str(nav_instruction.maneuverType) if hasattr(nav_instruction.maneuverType, '__str__') else str(nav_instruction.maneuverType)
            if any(maneuver in maneuver_str.lower() for maneuver in ['turn', 'intersection', 'stop', 'traffic', 'light']):
                scenario = 'intersection'

        # Additional checks for specific scenarios
        if scenario == 'highway' and v_ego > 20:  # Highway speeds
            # Check for merge indicators
            if 'leads_v3' in model_output and model_output['leads_v3']:
                merge_leads = [lead for lead in model_output['leads_v3']
                              if (lead.dRel < 50 and lead.yRel > 2.5 and lead.prob > 0.6)]
                if len(merge_leads) > 0:
                    scenario = 'highway_merge'

        # Check for residential areas based on speed and lead distribution
        if scenario == 'city' and v_ego < 10 and car_state.gasPressed < 0.1:
            # Low speed, not accelerating - might be in residential area
            if 'leads_v3' in model_output and model_output['leads_v3']:
                lead_distribution = [lead for lead in model_output['leads_v3']
                                   if lead.dRel < 20 and lead.prob > 0.6]
                if len(lead_distribution) < 2:  # Not much traffic
                    scenario = 'residential'

        return scenario

    def _adjust_for_weather(self, base_confidence: float,
                           car_state: log.CarState,
                           detected_scenario: str) -> Tuple[float, Dict[str, Any]]:
        """
        Adjust confidence based on weather conditions

        Args:
            base_confidence: Base confidence before weather adjustment
            car_state: Current car state
            detected_scenario: Driving scenario

        Returns:
            Tuple of (weather_adjusted_confidence, weather_metrics_dict)
        """
        # Get current weather data
        try:
            weather_data = weather_data_interface.get_weather_data()
            precipitation_factor = weather_data_interface.get_precipitation_factor()
        except Exception as e:
            cloudlog.warning(f"Could not get weather data for validation: {e}")
            # Default to normal conditions if weather data unavailable
            return base_confidence, {
                'weather_factor': 1.0,
                'precipitation_type': 'none',
                'precipitation_intensity': 0.0,
                'visibility': 200.0,
                'road_condition': 'dry'
            }

        # Get weather-based adjustments
        weather_factor = precipitation_factor  # This already accounts for precipitation

        # Adjust further based on visibility
        try:
            visibility = weather_data.get('visibility', 200.0)
        except (AttributeError, TypeError):
            visibility = 200.0  # Default visibility if not available

        if visibility < 50:  # Poor visibility
            weather_factor *= 0.7
        elif visibility < 100:  # Moderate visibility
            weather_factor *= 0.85

        # Adjust based on road surface condition
        try:
            road_condition = weather_data.get('road_surface_condition', 'dry')
        except (AttributeError, TypeError):
            road_condition = 'dry'

        if road_condition == 'wet':
            weather_factor *= 0.85  # Reduce confidence on wet roads
        elif road_condition in ['icy', 'snowy']:
            weather_factor *= 0.6   # Significantly reduce in icy/snowy conditions

        # For specific scenarios, adjust even more conservatively
        if detected_scenario in ['curvy_road', 'intersection']:
            # Curves and intersections are more hazardous in bad weather
            if weather_factor < 0.8:
                weather_factor *= 0.85

        # Apply weather adjustment - ensure confidence doesn't go below 0
        weather_adjusted_confidence = max(0.0, base_confidence * weather_factor)

        weather_metrics = {
            'weather_factor': weather_factor,
            'precipitation_type': weather_data.get('precipitation_type', 'none'),
            'precipitation_intensity': weather_data.get('precipitation_intensity', 0.0),
            'visibility': visibility,
            'road_condition': road_condition,
            'adjusted_confidence': weather_adjusted_confidence
        }

        return weather_adjusted_confidence, weather_metrics

    def calculate_situation_aware_confidence(self, model_output: Dict[str, Any],
                                           car_state: log.CarState,
                                           road_condition: str = 'highway',
                                           nav_instruction: Any = None) -> Dict[str, float]:
        """
        Calculate confidence based on current driving scenario

        Args:
            model_output: Dictionary containing model outputs with confidence metrics
            car_state: Current car state with speed, etc.
            road_condition: Type of road ('highway', 'city', 'curvy_road', etc.)
            nav_instruction: Navigation instruction for context

        Returns:
            Dictionary with enhanced confidence metrics
        """
        # Detect current scenario if not provided
        detected_scenario = self.detect_driving_scenario(model_output, car_state, nav_instruction)

        # Base confidence from model outputs
        base_confidence = self._calculate_base_confidence(model_output)

        # Apply situation-based factors based on detected scenario
        situation_factor = self._get_situation_factor(car_state, detected_scenario)

        # Adjust for weather conditions
        weather_adjusted_confidence, weather_metrics = self._adjust_for_weather(
            base_confidence, car_state, detected_scenario
        )

        # Calculate enhanced metrics
        enhanced_metrics = {
            'base_confidence': base_confidence,
            'situation_factor': situation_factor,
            'situation_adjusted_confidence': base_confidence * situation_factor,
            'weather_adjusted_confidence': weather_adjusted_confidence,
            'speed_adjusted_confidence': self._adjust_for_speed(weather_adjusted_confidence, car_state.vEgo),
            'system_safe': self._is_system_safe(weather_adjusted_confidence, car_state, detected_scenario),
            'detected_scenario': detected_scenario
        }

        # Add weather metrics to the result
        enhanced_metrics.update(weather_metrics)

        # Additional safety checks
        enhanced_metrics['lead_confidence_ok'] = self._check_lead_confidence(model_output, car_state)
        enhanced_metrics['lane_confidence_ok'] = self._check_lane_confidence(model_output)

        # Use weather-adjusted confidence for overall check
        enhanced_metrics['overall_confidence_ok'] = enhanced_metrics['weather_adjusted_confidence'] >= 0.6
        enhanced_metrics['lane_change_safe'] = self._is_lane_change_safe(model_output, car_state, enhanced_metrics)

        # Scenario-specific checks
        enhanced_metrics['intersection_safe'] = self._is_intersection_safe(model_output, car_state, detected_scenario)
        enhanced_metrics['curve_navigation_safe'] = self._is_curve_navigation_safe(model_output, car_state, detected_scenario)
        enhanced_metrics['merge_safe'] = self._is_merge_safe(model_output, car_state, detected_scenario)

        return enhanced_metrics
    
    def _calculate_base_confidence(self, model_output: Dict[str, Any]) -> float:
        """Calculate base confidence from model outputs"""
        # Extract confidence metrics from model outputs
        lead_conf_avg = model_output.get('lead_confidence_avg', 0.0)
        lane_conf_avg = model_output.get('lane_confidence_avg', 0.0)
        road_edge_conf_avg = model_output.get('road_edge_confidence_avg', 0.0)
        temporal_consistency = model_output.get('temporal_consistency', 1.0)
        path_in_lane_validity = model_output.get('path_in_lane_validity', 0.0)
        
        # Weighted confidence calculation
        base_confidence = (
            lead_conf_avg * 0.2 +
            lane_conf_avg * 0.2 +
            road_edge_conf_avg * 0.1 +
            temporal_consistency * 0.15 +
            path_in_lane_validity * 0.15 +
            model_output.get('overall_confidence', 0.0) * 0.2
        )
        
        # Ensure confidence is within bounds
        return max(0.0, min(1.0, base_confidence))
    
    def _get_situation_factor(self, car_state: log.CarState, road_condition: str) -> float:
        """Get situation-based adjustment factor"""
        if road_condition not in self.situation_factors:
            road_condition = 'highway'  # Default
            
        factors = self.situation_factors[road_condition]
        
        # Adjust based on speed
        speed_factor = factors['speed_factor']
        if car_state.vEgo > 25:  # Above ~90 km/h
            speed_factor *= 0.9  # Extra caution at high speed
        elif car_state.vEgo < 5:  # Below ~18 km/h
            speed_factor *= 1.05  # Slightly more responsive at low speed
        
        # Combine factors
        combined_factor = (
            speed_factor * 0.4 +
            factors['traffic_factor'] * 0.3 +
            factors['weather_factor'] * 0.3
        )
        
        return min(1.1, max(0.5, combined_factor))  # Clamp between 0.5 and 1.1
    
    def _adjust_for_speed(self, base_confidence: float, v_ego: float) -> float:
        """Adjust confidence based on vehicle speed"""
        # At high speeds, require higher confidence
        if v_ego > 35:  # ~125 km/h
            return base_confidence * 0.8  # Reduce effective confidence
        elif v_ego > 25:  # ~90 km/h
            return base_confidence * 0.9
        elif v_ego < 2:  # Stationary or very slow
            return min(1.0, base_confidence * 1.1)  # Can be slightly more confident
        else:
            return base_confidence
    
    def _is_system_safe(self, base_confidence: float, car_state: log.CarState, road_condition: str) -> bool:
        """Determine if system is safe to operate"""
        # Get required confidence threshold based on situation
        if road_condition == 'city' or car_state.vEgo < 15:  # City or low speed
            required_threshold = self.validation_thresholds['normal']
        else:
            required_threshold = self.validation_thresholds['aggressive']
        
        return base_confidence >= required_threshold
    
    def _check_lead_confidence(self, model_output: Dict[str, Any], car_state: log.CarState) -> bool:
        """Check if lead vehicle detection is reliable"""
        lead_conf_avg = model_output.get('lead_confidence_avg', 0.0)
        lead_conf_max = model_output.get('lead_confidence_max', 0.0)
        
        # More stringent requirements at higher speeds
        min_lead_conf = 0.6 if car_state.vEgo > 15 else 0.5  # 0.5 at low speed, 0.6 at high speed
        
        return lead_conf_avg >= min_lead_conf and lead_conf_max >= min_lead_conf * 1.2
    
    def _check_lane_confidence(self, model_output: Dict[str, Any]) -> bool:
        """Check if lane detection is reliable"""
        lane_conf_avg = model_output.get('lane_confidence_avg', 0.0)
        lane_count = model_output.get('lane_count', 0)
        
        # Need at least 2 lanes detected with good confidence
        return lane_conf_avg >= 0.65 and lane_count >= 2
    
    def _is_lane_change_safe(self, model_output: Dict[str, Any], car_state: log.CarState, 
                           enhanced_metrics: Dict[str, float]) -> bool:
        """Determine if lane change is safe"""
        # Lane change requires higher confidence than normal operation
        if not enhanced_metrics['lane_confidence_ok']:
            return False
            
        # Check for lead vehicles in target lane
        overall_conf = model_output.get('overall_confidence', 0.0)
        lane_conf = model_output.get('lane_confidence_avg', 0.0)
        
        return overall_conf >= 0.7 and lane_conf >= 0.7  # Higher thresholds for lane changes
    
    def _calculate_dynamic_safety_margin(self, base_margin: float,
                                       weather_factor: float = 1.0,
                                       road_curvature: float = 0.0,
                                       visibility: float = 200.0,
                                       surface_condition: str = 'dry') -> float:
        """
        Calculate dynamic safety margin based on environmental conditions

        Args:
            base_margin: Base safety margin
            weather_factor: Current weather impact factor (0.0-1.0)
            road_curvature: Current road curvature measure
            visibility: Current visibility in meters
            surface_condition: Road surface condition

        Returns:
            Adjusted safety margin
        """
        # Start with base margin
        safety_margin = base_margin

        # Adjust for weather
        if weather_factor < 0.8:
            safety_margin *= (1.5 - weather_factor)  # Increase margin as weather gets worse

        # Adjust for visibility
        if visibility < 100:  # Poor visibility
            safety_margin *= max(1.2, (200.0 / visibility))
        elif visibility < 200:  # Moderate visibility
            safety_margin *= (200.0 / visibility)

        # Adjust for road curvature (sharp curves need more margin)
        if road_curvature > 0.05:
            safety_margin *= (1.0 + road_curvature * 10)

        # Adjust for surface condition
        if surface_condition == 'wet':
            safety_margin *= 1.3  # 30% extra margin on wet roads
        elif surface_condition in ['icy', 'snowy']:
            safety_margin *= 2.0  # Double margin on icy/snowy roads

        return safety_margin

    def _is_intersection_safe(self, model_output: Dict[str, Any], car_state: log.CarState,
                            scenario: str) -> bool:
        """Check safety at intersections"""
        if scenario != 'intersection':
            return True  # Only check if actually in intersection scenario

        # Get environmental conditions
        weather_factor = self.get_weather_factor()
        visibility = self.get_current_visibility()
        surface_condition = self.get_current_road_surface()

        # Calculate dynamic margins for intersection
        stopping_margin = self._calculate_dynamic_safety_margin(
            1.0, weather_factor, 0.0, visibility, surface_condition
        )

        # At intersections, we need high confidence in multiple areas
        checks = []

        # Lead vehicle detection should be very reliable
        lead_conf = model_output.get('lead_confidence_avg', 0.0)
        checks.append(lead_conf >= 0.7)

        # Lane detection should be clear
        lane_conf = model_output.get('lane_confidence_avg', 0.0)
        checks.append(lane_conf >= 0.75)

        # Vehicle should be able to stop in time if needed with dynamic safety margin
        # Use more conservative deceleration in poor conditions
        decel_factor = 0.7 if weather_factor < 0.8 or surface_condition != 'dry' else 1.0
        max_decel = 3.5 * decel_factor
        stopping_distance = (car_state.vEgo ** 2) / (2 * max_decel) if max_decel > 0 else float('inf')

        if 'leads_v3' in model_output and model_output['leads_v3']:
            closest_lead = min([lead.dRel for lead in model_output['leads_v3'] if lead.prob > 0.5] or [100])
            # Apply dynamic safety margin at intersection
            checks.append(stopping_distance < closest_lead * stopping_margin)

        return all(checks)

    def get_weather_factor(self) -> float:
        """Get current weather factor from the weather interface"""
        try:
            return weather_data_interface.get_precipitation_factor()
        except:
            return 1.0  # Default to normal conditions if unavailable

    def get_current_visibility(self) -> float:
        """Get current visibility from weather data"""
        try:
            weather_data = weather_data_interface.get_weather_data()
            return weather_data.get('visibility', 200.0)
        except:
            return 200.0  # Default to good visibility

    def get_current_road_surface(self) -> str:
        """Get current road surface condition"""
        try:
            weather_data = weather_data_interface.get_weather_data()
            return weather_data.get('road_surface_condition', 'dry')
        except:
            return 'dry'  # Default to dry road

    def _is_curve_navigation_safe(self, model_output: Dict[str, Any], car_state: log.CarState,
                                 scenario: str) -> bool:
        """Check safety when navigating curves"""
        if scenario != 'curvy_road':
            return True  # Only check if actually in curvy road scenario

        # Get environmental conditions
        weather_factor = self.get_weather_factor()
        visibility = self.get_current_visibility()
        surface_condition = self.get_current_road_surface()

        # Calculate curvature from path
        max_curvature = 0.0
        if 'position' in model_output and hasattr(model_output['position'], 'y'):
            path_points = model_output['position'].y if hasattr(model_output['position'], 'y') else []
            if len(path_points) > 5:
                # Estimate curvature from the path (simplified)
                y_vals = path_points[:10] if len(path_points) > 10 else path_points
                if len(y_vals) >= 3:
                    # More sophisticated curvature calculation
                    curvature_measures = []
                    for i in range(1, len(y_vals)-1):
                        # Calculate change in direction
                        dy1 = y_vals[i] - y_vals[i-1]
                        dy2 = y_vals[i+1] - y_vals[i]
                        # Approximate curvature as change in direction
                        curvature = abs(dy2 - dy1)
                        curvature_measures.append(curvature)

                    if curvature_measures:
                        max_curvature = max(curvature_measures)

        # Calculate dynamic safety margin for curves
        curve_margin = self._calculate_dynamic_safety_margin(
            1.2, weather_factor, max_curvature, visibility, surface_condition
        )

        if max_curvature > 0.01:  # Significant curvature
            max_safe_radius = 1.0 / max_curvature if max_curvature > 0 else float('inf')

            # Adjust max lateral acceleration based on surface condition
            base_lat_accel = 2.5  # m/s^2
            if surface_condition == 'wet':
                lat_accel = base_lat_accel * 0.7  # Reduce on wet roads
            elif surface_condition in ['icy', 'snowy']:
                lat_accel = base_lat_accel * 0.4  # Significantly reduce on icy/snowy roads
            else:
                lat_accel = base_lat_accel

            max_safe_speed = np.sqrt(max_safe_radius * lat_accel)
            return car_state.vEgo <= max_safe_speed * curve_margin  # Apply dynamic margin

        # If we can't determine curve safety from path, use simpler checks with environmental factors
        safe_base_speed = 15  # 15 m/s = 54 km/h, a safe speed in good conditions

        # Adjust safe speed based on environmental conditions
        adjusted_safe_speed = safe_base_speed * weather_factor
        if surface_condition == 'wet':
            adjusted_safe_speed *= 0.8
        elif surface_condition in ['icy', 'snowy']:
            adjusted_safe_speed *= 0.5
        if visibility < 100:
            adjusted_safe_speed *= min(1.0, visibility / 100.0)

        return car_state.vEgo <= adjusted_safe_speed

    def _is_merge_safe(self, model_output: Dict[str, Any], car_state: log.CarState,
                      scenario: str) -> bool:
        """Check safety during highway merges"""
        if scenario != 'highway_merge':
            return True  # Only check if actually merging

        # Get environmental conditions
        weather_factor = self.get_weather_factor()
        visibility = self.get_current_visibility()
        surface_condition = self.get_current_road_surface()

        # Calculate dynamic safety margin for merging
        merge_margin = self._calculate_dynamic_safety_margin(
            1.0, weather_factor, 0.0, visibility, surface_condition
        )

        checks = []

        # Must have good lead detection
        lead_conf = model_output.get('lead_confidence_avg', 0.0)
        checks.append(lead_conf >= 0.75)

        # Check for safe merging gap with dynamic margin
        if 'leads_v3' in model_output and model_output['leads_v3']:
            # Look for gaps in traffic ahead
            adequate_gaps = 0
            for lead in model_output['leads_v3']:
                if lead.prob > 0.7 and lead.dRel < 100:  # Within 100m
                    # Check if there's a safe gap using dynamic margin
                    gap_size = lead.dRel
                    required_gap = car_state.vEgo * (2.0 * merge_margin)  # Apply safety margin
                    if gap_size > required_gap:
                        adequate_gaps += 1
            checks.append(adequate_gaps > 0)

        return all(checks)

    def get_safety_recommendation(self, enhanced_metrics: Dict[str, float],
                                car_state: log.CarState) -> Tuple[bool, str]:
        """
        Get safety recommendation based on enhanced validation metrics

        Returns:
            Tuple of (is_safe_to_engage, reason_string)
        """
        issues = []

        if not enhanced_metrics['system_safe']:
            issues.append("system confidence below threshold")

        if not enhanced_metrics['lead_confidence_ok']:
            issues.append("lead detection unreliable")

        if not enhanced_metrics['lane_confidence_ok']:
            issues.append("lane detection unreliable")

        if not enhanced_metrics['overall_confidence_ok']:
            issues.append("overall confidence too low")

        # Check weather-related issues
        if enhanced_metrics['weather_factor'] < 0.7:
            weather_type = enhanced_metrics.get('precipitation_type', 'unknown')
            visibility = enhanced_metrics.get('visibility', 'unknown')
            road_condition = enhanced_metrics.get('road_condition', 'unknown')

            if weather_type != 'none':
                issues.append(f"weather conditions adverse: {weather_type}")
            if visibility != 'unknown' and visibility < 100:
                issues.append(f"poor visibility: {visibility}m")
            if road_condition != 'dry':
                issues.append(f"road surface condition: {road_condition}")

        # Check scenario-specific safety
        if enhanced_metrics['detected_scenario'] == 'intersection' and not enhanced_metrics['intersection_safe']:
            issues.append("intersection navigation unsafe")

        if enhanced_metrics['detected_scenario'] == 'curvy_road' and not enhanced_metrics['curve_navigation_safe']:
            issues.append("curve navigation unsafe")

        if enhanced_metrics['detected_scenario'] == 'highway_merge' and not enhanced_metrics['merge_safe']:
            issues.append("merge maneuver unsafe")

        if car_state.vEgo > 5 and (car_state.leftBlinker or car_state.rightBlinker):
            # Check lane change safety if blinker is on
            if not enhanced_metrics['lane_change_safe']:
                issues.append("lane change unsafe")

        is_safe = len(issues) == 0

        if is_safe:
            reason = f"All safety checks passed for scenario: {enhanced_metrics['detected_scenario']}"
        else:
            reason = f"Safety concerns in {enhanced_metrics['detected_scenario']} scenario: {'; '.join(issues)}"

        return is_safe, reason


# Singleton instance for use across the system
enhanced_validator = EnhancedSafetyValidator()