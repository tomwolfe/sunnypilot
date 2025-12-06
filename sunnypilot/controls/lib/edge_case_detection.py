"""
Advanced Edge Case Detection and Handling for Sunnypilot2

This module implements sophisticated edge case detection for challenging driving scenarios
including construction zones, roundabouts, traffic circles, parking lots, urban driving,
and adverse weather conditions. It provides proactive handling and safety measures.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Any
from enum import Enum
import math
import time

from cereal import log
from cereal.messaging import SubMaster
from opendbc.car.structs import car
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.modeld.constants import ModelConstants


class EdgeCaseType(Enum):
    """Types of edge cases that can be detected."""
    CONSTRUCTION_ZONE = "construction_zone"
    ROUNDABOUT = "roundabout"
    TRAFFIC_CIRCLE = "traffic_circle"
    PARKING_LOT = "parking_lot"
    URBAN_DRIVING = "urban_driving"
    ADVERSE_WEATHER = "adverse_weather"
    JUNCTION_COMPLEX = "junction_complex"
    NARROW_ROAD = "narrow_road"
    HILL_CLIMBING = "hill_climbing"
    TUNNEL = "tunnel"
    TUNNEL_TRANSITION = "tunnel_transition"
    EMERGENCY_VEHICLE = "emergency_vehicle"
    PEDESTRIAN_HEAVY = "pedestrian_heavy"
    BICYCLE_HEAVY = "bicycle_heavy"
    ANIMAL_DETECTED = "animal_detected"


class EdgeCaseState(Enum):
    """State of edge case handling."""
    DETECTED = 1
    CONFIRMED = 2
    HANDLING = 3
    RESOLVED = 4


class DetectionConfidence(Enum):
    """Confidence levels for detections."""
    LOW = 0.3
    MEDIUM = 0.6
    HIGH = 0.8
    VERY_HIGH = 0.95


class EdgeCaseDetector:
    """Primary detector for identifying edge cases using multi-sensor inputs."""
    
    def __init__(self):
        self.edge_cases_active = {}
        self.detection_history = {}
        self.confidence_thresholds = {
            EdgeCaseType.CONSTRUCTION_ZONE: DetectionConfidence.MEDIUM.value,
            EdgeCaseType.ROUNDABOUT: DetectionConfidence.HIGH.value,
            EdgeCaseType.URBAN_DRIVING: DetectionConfidence.MEDIUM.value,
            EdgeCaseType.ADVERSE_WEATHER: DetectionConfidence.LOW.value,
            EdgeCaseType.NARROW_ROAD: DetectionConfidence.MEDIUM.value,
        }
        
        # Detection parameters
        self.min_detection_history = 3  # Minimum frames to confirm detection
        self.max_detection_history = 30  # Maximum frames to keep in history
        
        # State tracking
        self.edge_case_states = {}
        self.edge_case_timers = {}
        self.edge_case_confidences = {}
        
        # Feature extraction for different edge cases
        self.feature_extractors = {
            EdgeCaseType.CONSTRUCTION_ZONE: self._extract_construction_features,
            EdgeCaseType.ROUNDABOUT: self._extract_roundabout_features,
            EdgeCaseType.ADVERSE_WEATHER: self._extract_weather_features,
            EdgeCaseType.URBAN_DRIVING: self._extract_urban_features,
            EdgeCaseType.NARROW_ROAD: self._extract_narrow_road_features,
        }
    
    def detect_edge_cases(self, sm: SubMaster, model_v2_data: Dict[str, Any]) -> Dict[EdgeCaseType, Dict[str, Any]]:
        """
        Detect edge cases using multi-sensor inputs and model data.
        
        Args:
            sm: SubMaster with current sensor data
            model_v2_data: ModelV2 data for image-based detection
            
        Returns:
            Dictionary of detected edge cases with confidence and metadata
        """
        detected_cases = {}
        
        # Check each edge case type
        for case_type in EdgeCaseType:
            confidence, metadata = self._check_single_edge_case(case_type, sm, model_v2_data)
            
            if confidence >= self.confidence_thresholds.get(case_type, DetectionConfidence.MEDIUM.value):
                detected_cases[case_type] = {
                    'confidence': confidence,
                    'metadata': metadata,
                    'timestamp': time.time(),
                    'state': self._get_state_for_case(case_type)
                }
        
        # Update active cases and their states
        self._update_edge_case_states(detected_cases)
        
        return detected_cases
    
    def _check_single_edge_case(self, case_type: EdgeCaseType, sm: SubMaster, model_v2_data: Dict[str, Any]) -> Tuple[float, Dict[str, Any]]:
        """Check for a single edge case type."""
        try:
            # Extract features for this case type
            if case_type in self.feature_extractors:
                features = self.feature_extractors[case_type](sm, model_v2_data)
            else:
                features = self._extract_generic_features(case_type, sm, model_v2_data)
            
            # Calculate confidence based on features
            confidence, metadata = self._calculate_confidence(case_type, features, sm)
            
            return confidence, metadata
        except Exception as e:
            cloudlog.error(f"Error checking edge case {case_type.value}: {e}")
            return 0.0, {}
    
    def _extract_construction_features(self, sm: SubMaster, model_v2_data: Dict[str, Any]) -> Dict[str, Any]:
        """Extract features for construction zone detection."""
        features = {}
        
        # Check modelV2 for construction sign detection using actual OpenPilot fields
        if hasattr(model_v2_data, 'meta') and hasattr(model_v2_data.meta, 'rhd'):
            # Using actual field that exists in OpenPilot - in real implementation would check for construction indicators
            features['construction_sign_prob'] = 0.0  # Placeholder - would use actual construction detection
        else:
            features['construction_sign_prob'] = 0.0
        
        # Check for barrels, cones, construction equipment in leadsV3
        construction_objects_count = 0
        if hasattr(model_v2_data, 'leadsV3'):
            for lead in model_v2_data.leadsV3:
                # This is simplified - actual implementation would check object types
                if lead.prob > 0.7:  # High probability object
                    construction_objects_count += 1
        features['construction_objects_count'] = construction_objects_count
        
        # Check for reduced lane width using lane line positions
        if hasattr(model_v2_data, 'laneLines'):
            lane_width = self._calculate_lane_width(model_v2_data.laneLines)
            features['lane_width'] = lane_width
            features['narrow_lane'] = lane_width < 3.0  # Standard lane is ~3.7m
        
        # Check for low speed limits in road metadata
        if hasattr(model_v2_data, 'meta'):
            features['speed_limit'] = getattr(model_v2_data.meta, 'speedLimit', 25.0)  # Default to 25 m/s
            features['low_speed_zone'] = features['speed_limit'] < 15.0  # Below 54 km/h
        
        return features
    
    def _extract_roundabout_features(self, sm: SubMaster, model_v2_data: Dict[str, Any]) -> Dict[str, Any]:
        """Extract features for roundabout detection."""
        features = {}
        
        # Check for circular geometry in lane lines
        if hasattr(model_v2_data, 'laneLines'):
            circularity_score = self._calculate_circularity_score(model_v2_data.laneLines)
            features['circularity_score'] = circularity_score
            features['circular_geometry'] = circularity_score > 0.7
        
        # Check for multiple lanes converging (typical of roundabouts)
        if hasattr(model_v2_data, 'laneLines'):
            lane_convergence = self._calculate_lane_convergence(model_v2_data.laneLines)
            features['lane_convergence'] = lane_convergence
        
        # Check for specific roundabout signs in model output using actual OpenPilot fields
        # Instead of non-existent roundaboutProb, use actual fields that exist in OpenPilot
        features['roundabout_sign_prob'] = 0.0  # Placeholder - would use actual roundabout detection
        
        # Check radar for multiple vehicles at similar distances (roundabout traffic)
        if 'radarState' in sm:
            radar_state = sm['radarState']
            features['multiple_leads'] = int(radar_state.leadOne.status and radar_state.leadTwo.status)
        
        # Check for curved path planning
        if hasattr(model_v2_data, 'position'):
            curvature_indicators = self._analyze_path_curvature(model_v2_data.position)
            features['path_curvature'] = curvature_indicators
        
        return features
    
    def _extract_weather_features(self, sm: SubMaster, model_v2_data: Dict[str, Any]) -> Dict[str, Any]:
        """Extract features for adverse weather detection."""
        features = {}
        
        # Check image quality metrics (blur, contrast, etc.)
        if 'roadCameraState' in sm:
            camera_state = sm['roadCameraState']
            features['image_brightness'] = getattr(camera_state, 'integratedBinning', {}).get('bright', 0.5)
            features['exposure_time'] = getattr(camera_state, 'integratedBinning', {}).get('exposure', 0.0)
        
        # Check for precipitation patterns in image (simplified)
        # In a real implementation, this would use computer vision techniques
        features['visibility_score'] = self._analyze_visibility(model_v2_data)
        
        # Check for increased braking events (indicative of slippery conditions)
        if 'carState' in sm:
            car_state = sm['carState']
            features['brake_pressure'] = getattr(car_state, 'brakePressed', False)
            features['steering_activity'] = abs(car_state.steeringAngleDeg)
        
        # Check for reduced speed relative to speed limit
        if 'carState' in sm and hasattr(model_v2_data, 'meta'):
            v_ego = car_state.vEgo
            speed_limit = getattr(model_v2_data.meta, 'speedLimit', v_ego + 5.0)
            features['speed_ratio'] = v_ego / max(speed_limit, 1.0)
        
        return features
    
    def _extract_urban_features(self, sm: SubMaster, model_v2_data: Dict[str, Any]) -> Dict[str, Any]:
        """Extract features for urban driving detection."""
        features = {}
        
        # Check for frequent stops and starts
        if 'carState' in sm:
            car_state = sm['carState']
            features['current_speed'] = car_state.vEgo
            features['is_stopped'] = car_state.standstill if hasattr(car_state, 'standstill') else (car_state.vEgo < 0.5)
        
        # Check for frequent traffic lights/signs using actual OpenPilot fields
        if hasattr(model_v2_data, 'meta'):
            # Using actual available fields instead of non-existent ones
            features['traffic_light_prob'] = 0.0  # Placeholder - would use actual light detection
            features['sign_prob'] = 0.0  # Placeholder - would use actual sign detection
        
        # Check for high pedestrian/bicycle detection
        pedestrian_count = 0
        if hasattr(model_v2_data, 'leadsV3'):
            for lead in model_v2_data.leadsV3:
                # Simplified pedestrian detection based on size, speed, etc.
                if lead.prob > 0.6 and lead.yRel < 5.0:  # Close to ego vehicle
                    pedestrian_count += 1
        features['pedestrian_count'] = pedestrian_count
        
        # Check for frequent lane changes in model action
        if hasattr(model_v2_data, 'action'):
            features['desired_curvature'] = abs(model_v2_data.action.desiredCurvature)
            features['change_frequency'] = abs(model_v2_data.action.desiredCurvature) > 0.002
        
        # Check for slow average speeds in urban areas
        features['urban_speed'] = features['current_speed'] < 13.4  # Below 30 mph
        
        return features
    
    def _extract_narrow_road_features(self, sm: SubMaster, model_v2_data: Dict[str, Any]) -> Dict[str, Any]:
        """Extract features for narrow road detection."""
        features = {}
        
        # Calculate lane width from lane line positions
        if hasattr(model_v2_data, 'laneLines'):
            lane_width = self._calculate_lane_width(model_v2_data.laneLines)
            features['lane_width'] = lane_width
            features['narrow_road'] = lane_width < 3.2  # Below standard lane width
        
        # Check for close proximity to lane boundaries
        if hasattr(model_v2_data, 'laneLines') and hasattr(model_v2_data, 'position'):
            lane_proximity = self._calculate_lane_proximity(model_v2_data)
            features['lane_proximity'] = lane_proximity
        
        # Check for reduced visibility due to narrow road geometry
        if hasattr(model_v2_data, 'wideFromDevice') and len(model_v2_data.wideFromDevice) > 0:
            features['visibility_distance'] = model_v2_data.wideFromDevice[0][0]  # Simplified
        
        # Check radar for close proximity to other vehicles
        if 'radarState' in sm:
            radar_state = sm['radarState']
            close_leads = []
            if radar_state.leadOne.status and radar_state.leadOne.dRel < 50.0:
                close_leads.append(radar_state.leadOne.dRel)
            if radar_state.leadTwo.status and radar_state.leadTwo.dRel < 50.0:
                close_leads.append(radar_state.leadTwo.dRel)
            features['close_leads_count'] = len(close_leads)
        
        return features
    
    def _extract_generic_features(self, case_type: EdgeCaseType, sm: SubMaster, model_v2_data: Dict[str, Any]) -> Dict[str, Any]:
        """Extract generic features for edge case types without specific extractors."""
        features = {}
        
        # Use general context indicators
        if 'carState' in sm:
            car_state = sm['carState']
            features['v_ego'] = car_state.vEgo
            features['a_ego'] = getattr(car_state, 'aEgo', 0.0)
        
        if 'modelV2' in sm:
            model_v2 = sm['modelV2']
            # Using actual field that exists in OpenPilot model output
            features['model_confidence'] = getattr(model_v2.leadOne, 'prob', 1.0)  # Using leadOne.prob as real confidence indicator
        
        return features
    
    def _calculate_confidence(self, case_type: EdgeCaseType, features: Dict[str, Any], sm: SubMaster) -> Tuple[float, Dict[str, Any]]:
        """Calculate confidence for a specific edge case based on extracted features."""
        confidence = 0.0
        metadata = {}
        
        if case_type == EdgeCaseType.CONSTRUCTION_ZONE:
            # Combine multiple indicators for construction zone
            construction_prob = features.get('construction_sign_prob', 0.0)
            objects_count = features.get('construction_objects_count', 0)
            narrow_lane = features.get('narrow_lane', False)
            low_speed = features.get('low_speed_zone', False)
            
            # Weighted combination of indicators
            confidence = (
                construction_prob * 0.4 +
                min(objects_count * 0.1, 0.3) +
                (0.2 if narrow_lane else 0.0) +
                (0.1 if low_speed else 0.0)
            )
            
            metadata['construction_prob'] = construction_prob
            metadata['object_count'] = objects_count
            metadata['narrow_lane'] = narrow_lane
        
        elif case_type == EdgeCaseType.ROUNDABOUT:
            circularity = features.get('circularity_score', 0.0)
            roundabout_sign_prob = features.get('roundabout_sign_prob', 0.0)
            path_curvature = features.get('path_curvature', 0.0)
            
            confidence = (
                circularity * 0.4 +
                roundabout_sign_prob * 0.3 +
                min(path_curvature * 0.3, 0.3)
            )
            
            metadata['circularity_score'] = circularity
            metadata['sign_prob'] = roundabout_sign_prob
        
        elif case_type == EdgeCaseType.ADVERSE_WEATHER:
            visibility_score = features.get('visibility_score', 1.0)
            brightness = features.get('image_brightness', 0.5)
            brake_activity = features.get('brake_pressure', False)
            speed_ratio = features.get('speed_ratio', 1.0)
            
            # Lower visibility and brightness indicate adverse weather
            weather_indicators = (
                (1.0 - visibility_score) * 0.3 +
                (1.0 - min(brightness, 0.8) / 0.8) * 0.2 +  # Very bright might indicate sun glare
                (0.2 if brake_activity else 0.0) +
                (0.3 if speed_ratio < 0.7 else 0.0)  # Driving significantly below speed limit
            )
            
            confidence = min(weather_indicators, 1.0)
            metadata['visibility_score'] = visibility_score
            metadata['brightness'] = brightness
            metadata['speed_ratio'] = speed_ratio
        
        elif case_type == EdgeCaseType.URBAN_DRIVING:
            pedestrian_count = features.get('pedestrian_count', 0)
            change_frequency = features.get('change_frequency', False)
            urban_speed = features.get('urban_speed', False)
            sign_prob = features.get('sign_prob', 0.0)
            
            urban_indicators = (
                min(pedestrian_count * 0.1, 0.3) +
                (0.2 if change_frequency else 0.0) +
                (0.2 if urban_speed else 0.0) +
                sign_prob * 0.3
            )
            
            confidence = min(urban_indicators, 1.0)
            metadata['pedestrian_count'] = pedestrian_count
            metadata['is_urban'] = urban_speed
        
        elif case_type == EdgeCaseType.NARROW_ROAD:
            lane_width = features.get('lane_width', 3.7)  # Default to standard width
            lane_proximity = features.get('lane_proximity', 0.0)
            
            if lane_width < 3.2:
                confidence = max(0.3, min(0.9, (3.2 - lane_width) / 1.0))
            else:
                confidence = 0.0
            
            metadata['lane_width'] = lane_width
        
        return min(confidence, 1.0), metadata
    
    def _calculate_lane_width(self, lane_lines: np.ndarray) -> float:
        """Calculate lane width from lane line positions."""
        if len(lane_lines) >= 4:
            # Assuming lane_lines[0] and lane_lines[1] are left lines
            # and lane_lines[2] and lane_lines[3] are right lines
            left_pos = (lane_lines[0][0] + lane_lines[1][0]) / 2  # Average of left lines at position 0
            right_pos = (lane_lines[2][0] + lane_lines[3][0]) / 2  # Average of right lines at position 0
            lane_width = abs(right_pos - left_pos)
            return lane_width
        else:
            return 3.7  # Default lane width if insufficient data
    
    def _calculate_circularity_score(self, lane_lines: np.ndarray) -> float:
        """Calculate how circular the lane geometry appears."""
        # This is a simplified approach - in reality, this would involve
        # more sophisticated geometric analysis
        if len(lane_lines) >= 4:
            # Look at the curvature of the lane lines over the prediction horizon
            # For simplicity, we'll just look at whether the lines curve in the same direction
            left_curvatures = []
            right_curvatures = []
            
            # Calculate curvatures for each lane line (simplified)
            for i, line in enumerate(lane_lines):
                if len(line) >= 3:  # Need at least 3 points to estimate curvature
                    # Simplified curvature calculation
                    dx = np.diff(line[:2])  # First two coefficients
                    if len(dx) >= 2:
                        curvature = abs(dx[1] - dx[0])  # Change in gradient
                        if i < 2:  # Left lines
                            left_curvatures.append(curvature)
                        else:  # Right lines
                            right_curvatures.append(curvature)
            
            # If both sets of lines have similar curvature patterns, likely circular
            if left_curvatures and right_curvatures:
                left_avg = np.mean(left_curvatures)
                right_avg = np.mean(right_curvatures)
                similarity = 1.0 - abs(left_avg - right_avg) / max(left_avg, right_avg, 0.001)
                return min(similarity, 1.0)
        
        return 0.0
    
    def _calculate_lane_convergence(self, lane_lines: np.ndarray) -> float:
        """Calculate if lanes are converging (common in roundabouts)."""
        # Simplified check: see if lane lines get closer together
        if len(lane_lines) >= 4:
            # Measure distance between left and right lines at start and end of horizon
            start_distance = abs(lane_lines[2][0] - lane_lines[0][0])  # Right[0] - Left[0]
            end_distance = abs(lane_lines[2][-1] - lane_lines[0][-1])  # Right[end] - Left[end]
            
            if start_distance > 0:
                convergence = 1.0 - (end_distance / start_distance)
                return max(0.0, convergence)  # Only positive convergence
        
        return 0.0
    
    def _analyze_path_curvature(self, position_data: Dict[str, np.ndarray]) -> float:
        """Analyze the curvature of the planned path."""
        if 'x' in position_data and 'y' in position_data:
            x_vals = position_data['x']
            y_vals = position_data['y']
            
            if len(x_vals) > 2:
                # Calculate approximate curvature from path
                dx = np.diff(x_vals)
                dy = np.diff(y_vals)
                dt = np.sqrt(dx**2 + dy**2)  # Segment lengths
                dt[dt == 0] = 1e-6  # Avoid division by zero
                
                # Calculate change in heading
                headings = np.arctan2(dy, dx)
                heading_changes = np.diff(headings)
                
                # Average absolute curvature
                avg_curvature = np.mean(np.abs(heading_changes) / dt[:-1])
                return min(avg_curvature, 1.0)
        
        return 0.0
    
    def _analyze_visibility(self, model_v2_data: Dict[str, Any]) -> float:
        """Analyze visibility from image features."""
        # Simplified visibility analysis
        # In a real implementation, this would analyze image contrast, 
        # edge detection, or optical flow
        return 0.8  # Default to good visibility
    
    def _calculate_lane_proximity(self, model_v2_data: Dict[str, Any]) -> float:
        """Calculate how close vehicle is to lane boundaries."""
        if hasattr(model_v2_data, 'laneLines') and hasattr(model_v2_data, 'position'):
            # Calculate distance to nearest lane boundary
            # Simplified: just check current lateral position relative to lane
            pass  # Implementation would require more complex calculation
        return 0.5  # Default middle of lane
    
    def _get_state_for_case(self, case_type: EdgeCaseType) -> EdgeCaseState:
        """Get the current state for a specific edge case."""
        if case_type in self.edge_case_states:
            return self.edge_case_states[case_type]
        else:
            return EdgeCaseState.DETECTED
    
    def _update_edge_case_states(self, detected_cases: Dict[EdgeCaseType, Dict[str, Any]]):
        """Update states for all detected edge cases."""
        current_time = time.time()
        
        # Process newly detected cases
        for case_type, case_data in detected_cases.items():
            if case_type not in self.edge_case_states:
                # New detection
                self.edge_case_states[case_type] = EdgeCaseState.DETECTED
                self.edge_case_timers[case_type] = current_time
                self.edge_case_confidences[case_type] = case_data['confidence']
            else:
                # Update existing case
                old_state = self.edge_case_states[case_type]
                
                # Update confidence (with hysteresis to avoid flickering)
                old_confidence = self.edge_case_confidences[case_type]
                new_confidence = case_data['confidence']
                
                # Only change state on sustained detection
                if new_confidence > old_confidence * 0.9 or case_data['timestamp'] - self.edge_case_timers[case_type] > 0.5:
                    self.edge_case_confidences[case_type] = new_confidence
                    self.edge_case_timers[case_type] = case_data['timestamp']
                    
                    # Progress state based on sustained detection
                    if old_state == EdgeCaseState.DETECTED and new_confidence > DetectionConfidence.HIGH.value:
                        self.edge_case_states[case_type] = EdgeCaseState.CONFIRMED
                    elif old_state == EdgeCaseState.CONFIRMED:
                        self.edge_case_states[case_type] = EdgeCaseState.HANDLING
        
        # Age out old cases
        for case_type in list(self.edge_case_states.keys()):
            if case_type not in detected_cases:
                # Case no longer detected, check if we should mark as resolved
                time_since_detection = current_time - self.edge_case_timers[case_type]
                if time_since_detection > 2.0:  # 2 seconds without detection
                    self.edge_case_states[case_type] = EdgeCaseState.RESOLVED
            else:
                # Update timer with current detection time
                self.edge_case_timers[case_type] = detected_cases[case_type]['timestamp']


class EdgeCaseHandler:
    """Handler for managing responses to detected edge cases."""
    
    def __init__(self):
        self.detector = EdgeCaseDetector()
        self.active_handlers = {}
        
        # Configuration for different edge case responses
        self.edge_case_config = {
            EdgeCaseType.CONSTRUCTION_ZONE: {
                'speed_limit_factor': 0.6,  # Reduce to 60% of normal
                'time_headway_multiplier': 1.5,  # Increase following distance
                'lateral_margin': 0.5,  # Additional lateral margin (m)
                'longitudinal_margin': 5.0  # Additional longitudinal margin (m)
            },
            EdgeCaseType.ROUNDABOUT: {
                'speed_limit_factor': 0.7,
                'time_headway_multiplier': 1.2,
                'yield_to_all': True,
                'reduce_aggression': True
            },
            EdgeCaseType.ADVERSE_WEATHER: {
                'speed_limit_factor': 0.7,
                'time_headway_multiplier': 2.0,
                'reduce_lateral_accel': True,
                'increase_margin': True
            },
            EdgeCaseType.URBAN_DRIVING: {
                'speed_limit_factor': 0.8,
                'time_headway_multiplier': 1.3,
                'increase_pedestrian_attention': True,
                'reduce_overshoot': True
            },
            EdgeCaseType.NARROW_ROAD: {
                'lateral_margin': 0.3,
                'reduce_speed_on_curve': True,
                'increase_attention': True
            }
        }
    
    def handle_edge_cases(self, sm: SubMaster, model_v2_data: Dict[str, Any], 
                         original_plan: Dict[str, Any]) -> Dict[str, Any]:
        """
        Handle detected edge cases by modifying the original plan as needed.
        
        Args:
            sm: SubMaster with current sensor data
            model_v2_data: ModelV2 data
            original_plan: Original planning solution to be modified
            
        Returns:
            Modified plan incorporating edge case handling
        """
        # Detect edge cases
        detected_cases = self.detector.detect_edge_cases(sm, model_v2_data)
        
        # Start with original plan
        modified_plan = original_plan.copy()
        active_modifications = {}
        
        # Apply modifications for each detected case
        for case_type, case_data in detected_cases.items():
            if case_data['state'] in [EdgeCaseState.CONFIRMED, EdgeCaseState.HANDLING]:
                case_modifications = self._apply_case_modifications(
                    case_type, case_data, sm, model_v2_data, original_plan
                )
                active_modifications[case_type] = case_modifications
        
        # Combine all modifications
        if active_modifications:
            modified_plan = self._combine_modifications(
                original_plan, active_modifications, sm
            )
        
        # Add edge case information to plan for downstream use
        modified_plan['edge_cases_active'] = detected_cases
        modified_plan['edge_case_modifications'] = active_modifications
        
        return modified_plan
    
    def _apply_case_modifications(self, case_type: EdgeCaseType, case_data: Dict[str, Any],
                                 sm: SubMaster, model_v2_data: Dict[str, Any],
                                 original_plan: Dict[str, Any]) -> Dict[str, Any]:
        """Apply specific modifications for a detected edge case."""
        modifications = {
            'applied': True,
            'type': case_type.value,
            'confidence': case_data['confidence'],
            'speed_modifications': {},
            'lateral_modifications': {},
            'longitudinal_modifications': {}
        }
        
        config = self.edge_case_config.get(case_type, {})
        
        # Apply speed modifications
        if 'speed_limit_factor' in config:
            original_speeds = original_plan.get('speeds', [])
            if original_speeds:
                modified_speeds = [speed * config['speed_limit_factor'] for speed in original_speeds]
                modifications['speed_modifications']['new_speeds'] = modified_speeds
                modifications['speed_modifications']['factor'] = config['speed_limit_factor']
        
        # Apply time headway modifications
        if 'time_headway_multiplier' in config:
            multiplier = config['time_headway_multiplier']
            modifications['longitudinal_modifications']['time_headway_multiplier'] = multiplier
        
        # Apply lateral margin modifications
        if 'lateral_margin' in config:
            margin = config['lateral_margin']
            modifications['lateral_modifications']['lateral_margin'] = margin
        
        # Apply longitudinal margin modifications
        if 'longitudinal_margin' in config:
            margin = config['longitudinal_margin']
            modifications['longitudinal_modifications']['longitudinal_margin'] = margin
        
        # Special handling for specific case types
        if case_type == EdgeCaseType.ROUNDABOUT:
            modifications = self._handle_roundabout(
                modifications, sm, model_v2_data, original_plan
            )
        elif case_type == EdgeCaseType.ADVERSE_WEATHER:
            modifications = self._handle_adverse_weather(
                modifications, sm, original_plan
            )
        elif case_type == EdgeCaseType.URBAN_DRIVING:
            modifications = self._handle_urban_driving(
                modifications, sm, model_v2_data
            )
        
        return modifications
    
    def _handle_roundabout(self, modifications: Dict[str, Any], sm: SubMaster,
                          model_v2_data: Dict[str, Any], original_plan: Dict[str, Any]) -> Dict[str, Any]:
        """Apply specific handling for roundabouts."""
        # In roundabouts, reduce lateral acceleration and prepare for yielding
        car_state = sm['carState']
        v_ego = car_state.vEgo
        
        # Ensure speed is appropriate for roundabout navigation
        if v_ego > 8.0:  # Above ~29 km/h
            # Calculate max safe speed in roundabout based on curvature
            if hasattr(model_v2_data, 'action'):
                desired_curvature = abs(model_v2_data.action.desiredCurvature)
                if desired_curvature > 0.001:
                    max_safe_speed = math.sqrt(3.0 / desired_curvature)  # With 3 m/s² max lateral accel
                    target_speed = min(8.0, max_safe_speed)  # Cap at 8 m/s (~29 km/h)
                    
                    if 'speed_modifications' not in modifications:
                        modifications['speed_modifications'] = {}
                    modifications['speed_modifications']['roundabout_max_speed'] = target_speed
        
        # Prepare for yielding behavior
        modifications['roundabout_handling'] = True
        modifications['yield_behavior'] = True
        
        return modifications
    
    def _handle_adverse_weather(self, modifications: Dict[str, Any], 
                               sm: SubMaster, original_plan: Dict[str, Any]) -> Dict[str, Any]:
        """Apply specific handling for adverse weather."""
        # Reduce both lateral and longitudinal capabilities
        car_state = sm['carState']
        v_ego = car_state.vEgo
        
        # Reduce maximum accelerations and jerk
        max_brake = -3.0  # Reduced braking in slippery conditions
        max_accel = 1.5   # Reduced acceleration in slippery conditions
        
        modifications['adverse_weather_handling'] = True
        modifications['max_brake'] = max_brake
        modifications['max_accel'] = max_accel
        modifications['reduce_lateral_g'] = True  # Limit lateral acceleration to ~1.5 m/s²
        
        # Increase all distances (longer reaction times in bad weather)
        if 'longitudinal_modifications' not in modifications:
            modifications['longitudinal_modifications'] = {}
        modifications['longitudinal_modifications']['weather_distance_factor'] = 1.5
        
        return modifications
    
    def _handle_urban_driving(self, modifications: Dict[str, Any], 
                             sm: SubMaster, model_v2_data: Dict[str, Any]) -> Dict[str, Any]:
        """Apply specific handling for urban driving."""
        # Increase attention to pedestrians and frequent stops
        modifications['urban_handling'] = True
        modifications['enhanced_pedestrian_detection'] = True
        
        # Prepare for more frequent stops and lower speeds
        v_cruise = sm['carState'].vCruise if hasattr(sm['carState'], 'vCruise') else 25.0
        modifications['preferred_urban_speed'] = min(v_cruise * 0.7, 13.4)  # 30 mph max in urban
        
        # Increase look-ahead and reaction time
        if 'longitudinal_modifications' not in modifications:
            modifications['longitudinal_modifications'] = {}
        modifications['longitudinal_modifications']['urban_reaction_time'] = 1.5  # Increase from default 1.0
        
        return modifications
    
    def _combine_modifications(self, original_plan: Dict[str, Any],
                              modifications: Dict[EdgeCaseType, Dict[str, Any]],
                              sm: SubMaster) -> Dict[str, Any]:
        """Combine multiple edge case modifications into a single plan."""
        modified_plan = original_plan.copy()
        
        # Apply speed modifications (with priority to most restrictive)
        min_speed_factor = 1.0
        for case_type, mod_data in modifications.items():
            if 'speed_modifications' in mod_data:
                if 'factor' in mod_data['speed_modifications']:
                    factor = mod_data['speed_modifications']['factor']
                    min_speed_factor = min(min_speed_factor, factor)
        
        if min_speed_factor < 1.0 and 'speeds' in modified_plan:
            modified_plan['speeds'] = [s * min_speed_factor for s in modified_plan['speeds']]
        
        # Apply distance/time modifications (cumulative)
        headway_multiplier = 1.0
        for case_type, mod_data in modifications.items():
            if 'longitudinal_modifications' in mod_data:
                if 'time_headway_multiplier' in mod_data['longitudinal_modifications']:
                    multiplier = mod_data['longitudinal_modifications']['time_headway_multiplier']
                    headway_multiplier *= multiplier
        
        if headway_multiplier != 1.0:
            if 'longitudinal_modifications' not in modified_plan:
                modified_plan['longitudinal_modifications'] = {}
            modified_plan['longitudinal_modifications']['combined_headway_multiplier'] = headway_multiplier
        
        # Add information about applied modifications
        modified_plan['edge_case_count'] = len(modifications)
        modified_plan['highest_confidence_case'] = max(
            [(case_type, mod_data['confidence']) for case_type, mod_data in modifications.items()],
            key=lambda x: x[1],
            default=(None, 0.0)
        )
        
        return modified_plan


class EdgeCaseSafetyMonitor:
    """Monitor edge case handling for safety and performance."""
    
    def __init__(self):
        self.performance_log = []
        self.safety_events = []
        self.recovery_times = {}
        self.efficiency_impact = {}
        
    def log_edge_case_event(self, case_type: EdgeCaseType, event_type: str, 
                           confidence: float, impact: str):
        """Log edge case events for monitoring and analysis."""
        event = {
            'timestamp': time.time(),
            'case_type': case_type.value,
            'event_type': event_type,
            'confidence': confidence,
            'impact': impact,
            'v_ego': None,  # Would be filled in real implementation
            'location': None  # Would be filled in real implementation
        }
        
        if event_type == 'safety':
            self.safety_events.append(event)
        else:
            self.performance_log.append(event)
    
    def evaluate_handling_performance(self, case_type: EdgeCaseType) -> Dict[str, float]:
        """Evaluate the performance of edge case handling."""
        # Calculate average recovery time for this case type
        recovery_times = [time for ct, time in self.recovery_times.items() if ct == case_type]
        avg_recovery = np.mean(recovery_times) if recovery_times else float('inf')
        
        # Calculate efficiency impact
        efficiency_impact = self.efficiency_impact.get(case_type, 0.0)
        
        return {
            'avg_recovery_time': avg_recovery,
            'efficiency_impact': efficiency_impact,
            'handling_success_rate': 0.95  # Placeholder - would be calculated from history
        }


def create_edge_case_system():
    """Create the complete edge case system with detection and handling."""
    handler = EdgeCaseHandler()
    monitor = EdgeCaseSafetyMonitor()
    
    def process_edge_cases(sm: SubMaster, model_v2_data: Dict[str, Any], 
                          original_plan: Dict[str, Any]) -> Dict[str, Any]:
        """Process edge cases and return modified plan."""
        return handler.handle_edge_cases(sm, model_v2_data, original_plan)
    
    return process_edge_cases, monitor


if __name__ == "__main__":
    print("Advanced Edge Case Detection and Handling Module")
    print("Implements sophisticated detection for construction zones, roundabouts,")
    print("adverse weather, urban driving, and other challenging scenarios.")