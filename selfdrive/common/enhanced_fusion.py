"""
Enhanced multi-camera fusion system for Sunnypilot
Improves temporal consistency and object tracking across camera boundaries
"""

import numpy as np
from typing import Dict, Any, Optional
from cereal import log
import cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix
import time


class EnhancedCameraFusion:
    """
    Enhanced camera fusion system that improves temporal consistency
    and tracking across multiple camera feeds
    """
    
    def __init__(self):
        self.prev_road_outputs = None
        self.prev_wide_outputs = None
        self.temporal_consistency_buffer = []
        self.max_buffer_size = 10
        
        # Initialize messaging for camera states
        try:
            self.sm = messaging.SubMaster(['roadCameraState', 'wideRoadCameraState', 
                                         'modelV2', 'cameraOdometry'], ignore_alive=True)
        except Exception as e:
            cloudlog.warning(f"Could not initialize SubMaster in EnhancedCameraFusion: {e}")
            self.sm = None
        
        # Camera calibration tracking
        self.road_calibration = None
        self.wide_calibration = None
        self.calibration_valid = False
        
        # Object tracking across frames
        self.tracked_objects = {}
        self.last_track_id = 0
        
    def update_calibrations(self, live_calib_data) -> None:
        """Update camera calibrations from live calibration data"""
        if live_calib_data and len(live_calib_data.rpyCalib) >= 3:
            device_from_calib_euler = np.array(live_calib_data.rpyCalib, dtype=np.float32)
            
            # Update calibration matrices for both cameras
            if self.sm and 'roadCameraState' in self.sm and 'deviceState' in self.sm:
                road_cam = DEVICE_CAMERAS[(str(self.sm['deviceState'].deviceType), 
                                          str(self.sm['roadCameraState'].sensor))]
                self.road_calibration = get_warp_matrix(device_from_calib_euler, 
                                                       road_cam.fcam.intrinsics, False).astype(np.float32)
                
                wide_cam = DEVICE_CAMERAS[(str(self.sm['deviceState'].deviceType), 
                                          str(self.sm['wideRoadCameraState'].sensor))]
                self.wide_calibration = get_warp_matrix(device_from_calib_euler, 
                                                       wide_cam.ecam.intrinsics, True).astype(np.float32)
                
                self.calibration_valid = True
    
    def enhanced_camera_fusion(self, road_camera_output: Dict[str, Any], 
                             wide_camera_output: Dict[str, Any]) -> Dict[str, Any]:
        """
        Enhanced fusion of road and wide camera outputs with temporal consistency
        
        Args:
            road_camera_output: Output from standard road camera model
            wide_camera_output: Output from wide angle camera model
        
        Returns:
            Fused output with improved consistency and accuracy
        """
        # Start with road camera as primary (better resolution at distance)
        fused_output = road_camera_output.copy()
        
        # Enhance near-field detection using wide camera
        fused_output = self._fuse_near_field_detection(fused_output, wide_camera_output)
        
        # Enhance lane detection using both cameras
        fused_output = self._fuse_lane_detection(fused_output, wide_camera_output)
        
        # Apply temporal consistency
        fused_output = self._apply_temporal_consistency(fused_output)
        
        # Enhance object tracking across camera feeds
        fused_output = self._enhance_object_tracking(fused_output, wide_camera_output)
        
        # Update output consistency metrics
        fused_output['temporal_consistency'] = self._calculate_temporal_consistency(fused_output)
        
        # Store for next frame's temporal consistency
        self.prev_road_outputs = road_camera_output
        self.prev_wide_outputs = wide_camera_output
        
        return fused_output
    
    def _fuse_near_field_detection(self, fused_output: Dict[str, Any], 
                                 wide_output: Dict[str, Any]) -> Dict[str, Any]:
        """Fuse near-field object detection using wide camera's wider FOV"""
        # Wide camera is better for near-field detection, so enhance leads
        # that are detected in both cameras but have better confidence in wide camera
        if ('leads_v3' in fused_output and 'leads_v3' in wide_output and 
            fused_output['leads_v3'] and wide_output['leads_v3']):
            
            # Merge lead detections, preferring wide camera for near objects
            for i, (main_lead, wide_lead) in enumerate(zip(fused_output['leads_v3'], 
                                                         wide_output['leads_v3'])):
                if i < len(fused_output['leads_v3']) and i < len(wide_output['leads_v3']):
                    # If wide camera has higher confidence and object is near, use wide camera data
                    if (wide_lead.prob > main_lead.prob and 
                        wide_lead.dRel < 50):  # Near object (less than 50m)
                        fused_output['leads_v3'][i] = wide_lead
        
        return fused_output
    
    def _fuse_lane_detection(self, fused_output: Dict[str, Any], 
                           wide_output: Dict[str, Any]) -> Dict[str, Any]:
        """Fuse lane detection from both cameras"""
        if ('lane_lines' in fused_output and 'lane_lines' in wide_output and 
            fused_output['lane_lines'] and wide_output['lane_lines']):
            
            # Use road camera for distant lane detection (better resolution)
            # Use wide camera for near lane detection and edge detection
            for i in range(min(len(fused_output['lane_lines']), len(wide_output['lane_lines']))):
                main_lane = fused_output['lane_lines'][i]
                wide_lane = wide_output['lane_lines'][i]
                
                # For edge lines (outside lanes), wide camera might be better
                if i <= 1:  # Left/right edge lines
                    if wide_lane.prob > main_lane.prob:
                        fused_output['lane_lines'][i] = wide_lane
                else:  # Inner lane lines
                    # Prefer road camera for inner lines unless wide is significantly more confident
                    if wide_lane.prob > main_lane.prob + 0.1:
                        fused_output['lane_lines'][i] = wide_lane
        
        return fused_output
    
    def _apply_temporal_consistency(self, fused_output: Dict[str, Any]) -> Dict[str, Any]:
        """Apply temporal consistency checks to smooth outputs across frames"""
        if self.prev_road_outputs is None:
            return fused_output
        
        # Smooth plan trajectory to reduce jerkiness
        if 'plan' in fused_output and 'plan' in self.prev_road_outputs:
            fused_output['plan'] = self._smooth_plan(fused_output['plan'], 
                                                   self.prev_road_outputs['plan'])
        
        # Smooth lane lines to reduce flickering
        if 'lane_lines' in fused_output and 'lane_lines' in self.prev_road_outputs:
            fused_output['lane_lines'] = self._smooth_lane_lines(
                fused_output['lane_lines'], self.prev_road_outputs['lane_lines'])
        
        # Add to consistency buffer
        self.temporal_consistency_buffer.append(fused_output)
        if len(self.temporal_consistency_buffer) > self.max_buffer_size:
            self.temporal_consistency_buffer.pop(0)
        
        return fused_output
    
    def _smooth_plan(self, current_plan: np.ndarray, prev_plan: np.ndarray, 
                    smoothing_factor: float = 0.3) -> np.ndarray:
        """Smooth plan trajectory between frames"""
        if current_plan.shape == prev_plan.shape:
            # Smooth the plan with previous frame's plan
            smoothed_plan = (1 - smoothing_factor) * current_plan + smoothing_factor * prev_plan
            return smoothed_plan
        return current_plan
    
    def _smooth_lane_lines(self, current_lines: list, prev_lines: list, 
                          smoothing_factor: float = 0.2) -> list:
        """Smooth lane line detection between frames"""
        smoothed_lines = []
        
        for i in range(min(len(current_lines), len(prev_lines))):
            curr_line = current_lines[i]
            prev_line = prev_lines[i]
            
            # Create a copy of the current line
            smoothed_line = curr_line
            
            # Smooth the points if they exist
            if hasattr(curr_line, 'points') and hasattr(prev_line, 'points'):
                if len(curr_line.points) == len(prev_line.points):
                    smoothed_points = []
                    for j in range(len(curr_line.points)):
                        smoothed_point = ((1 - smoothing_factor) * curr_line.points[j] + 
                                        smoothing_factor * prev_line.points[j])
                        smoothed_points.append(smoothed_point)
                    smoothed_line.points = smoothed_points
            
            smoothed_lines.append(smoothed_line)
        
        return smoothed_lines if smoothed_lines else current_lines
    
    def _enhance_object_tracking(self, fused_output: Dict[str, Any], 
                               wide_output: Dict[str, Any]) -> Dict[str, Any]:
        """Enhance object tracking across camera feeds"""
        # Create or update tracked objects
        if 'leads_v3' in fused_output and fused_output['leads_v3']:
            for lead in fused_output['leads_v3']:
                if lead.prob > 0.5:  # Only track confident detections
                    self._update_tracked_object(lead)
        
        # Update fused output with tracking information
        fused_output['tracked_objects'] = list(self.tracked_objects.values())
        
        return fused_output
    
    def _update_tracked_object(self, lead_detection: Any) -> None:
        """Update tracked object based on new detection"""
        # Simple tracking based on position proximity
        closest_id = None
        min_distance = float('inf')
        
        for obj_id, tracked_obj in self.tracked_objects.items():
            # Calculate distance to existing track
            dx = abs(tracked_obj.get('dRel', 0) - lead_detection.dRel)
            dy = abs(tracked_obj.get('yRel', 0) - lead_detection.yRel)
            distance = (dx**2 + dy**2)**0.5
            
            if distance < min_distance and distance < 5.0:  # Within 5m
                min_distance = distance
                closest_id = obj_id
        
        if closest_id is not None:
            # Update existing track
            self.tracked_objects[closest_id].update({
                'dRel': lead_detection.dRel,
                'yRel': lead_detection.yRel,
                'vRel': lead_detection.vRel,
                'prob': lead_detection.prob,
                'timestamp': time.time()
            })
        else:
            # Create new track
            self.last_track_id += 1
            self.tracked_objects[self.last_track_id] = {
                'id': self.last_track_id,
                'dRel': lead_detection.dRel,
                'yRel': lead_detection.yRel,
                'vRel': lead_detection.vRel,
                'prob': lead_detection.prob,
                'timestamp': time.time()
            }
    
    def _calculate_temporal_consistency(self, fused_output: Dict[str, Any]) -> float:
        """Calculate temporal consistency score"""
        if not self.temporal_consistency_buffer:
            return 1.0
        
        # Calculate consistency as the stability of key outputs
        consistency_scores = []
        
        # Compare current with most recent frame
        if len(self.temporal_consistency_buffer) >= 2:
            prev_output = self.temporal_consistency_buffer[-2]
            
            # Calculate plan consistency
            if ('plan' in fused_output and 'plan' in prev_output and
                fused_output['plan'].shape == prev_output['plan'].shape):
                plan_diff = np.mean(np.abs(fused_output['plan'] - prev_output['plan']))
                plan_consistency = max(0.0, 1.0 - plan_diff)  # Higher difference = lower consistency
                consistency_scores.append(plan_consistency)
        
        return np.mean(consistency_scores) if consistency_scores else 0.8  # Default to 0.8 if no comparison possible


# Singleton instance for use across the system
enhanced_fusion = EnhancedCameraFusion()