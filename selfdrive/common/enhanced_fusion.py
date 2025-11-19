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

# Import the new Kalman filter tracking system
from openpilot.selfdrive.common.kalman_filter import object_tracker


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

        # Use enhanced Kalman filter for object tracking
        # Object tracking is now handled by the Kalman filter system
        self.kalman_tracker = object_tracker
        self.frame_count = 0
        
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
        """Enhance object tracking across camera feeds using Kalman filters"""
        # Prepare detections for tracking
        detections = []

        # Collect leads from fused output
        if 'leads_v3' in fused_output and fused_output['leads_v3']:
            for lead in fused_output['leads_v3']:
                if lead.prob > 0.3:  # Lower threshold since Kalman filter handles noise
                    # Create detection object with dRel (longitudinal distance) and yRel (lateral distance)
                    detection = type('Detection', (), {})()
                    detection.dRel = lead.dRel
                    detection.yRel = lead.yRel
                    detection.prob = lead.prob
                    detections.append(detection)

        # Also include leads from wide camera if available (for better near-field tracking)
        if 'leads_v3' in wide_output and wide_output['leads_v3']:
            for lead in wide_output['leads_v3']:
                if lead.prob > 0.3 and lead.dRel < 50:  # Focus on near-field for wide camera
                    # Check if this detection is already in the main list to avoid duplicates
                    is_duplicate = False
                    for main_detection in detections:
                        dist = ((lead.dRel - main_detection.dRel)**2 + (lead.yRel - main_detection.yRel)**2)**0.5
                        if dist < 3.0:  # Within 3m considered same object
                            is_duplicate = True
                            break

                    if not is_duplicate:
                        detection = type('Detection', (), {})()
                        detection.dRel = lead.dRel
                        detection.yRel = lead.yRel
                        detection.prob = lead.prob
                        detections.append(detection)

        # Update the Kalman tracker with current detections
        current_time = time.time()
        self.frame_count += 1
        confirmed_tracks = self.kalman_tracker.update(detections, current_time)

        # Update fused output with enhanced tracking information
        fused_output['tracked_objects'] = confirmed_tracks
        fused_output['tracking_quality'] = len(confirmed_tracks)  # Number of confirmed tracks

        # Add tracking confidence based on track quality
        if confirmed_tracks:
            avg_track_confidence = sum(track['prob'] for track in confirmed_tracks) / len(confirmed_tracks)
            fused_output['track_confidence_avg'] = avg_track_confidence
            fused_output['track_count'] = len(confirmed_tracks)
        else:
            fused_output['track_confidence_avg'] = 0.0
            fused_output['track_count'] = 0

        return fused_output
    
    def _calculate_temporal_consistency(self, fused_output: Dict[str, Any]) -> float:
        """Calculate temporal consistency score"""
        if not self.temporal_consistency_buffer:
            # If we have Kalman-filtered tracking, start with that as baseline
            if 'track_count' in fused_output and fused_output['track_count'] > 0:
                # Use tracking quality as initial consistency if available
                track_quality = min(1.0, fused_output['track_count'] * 0.1)  # Up to 0.1 per tracked object
                return min(1.0, 0.5 + track_quality)  # Base consistency + tracking contribution
            return 0.8  # Default baseline

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

            # Calculate lane line consistency
            if ('lane_lines' in fused_output and 'lane_lines' in prev_output and
                len(fused_output['lane_lines']) == len(prev_output['lane_lines'])):
                lane_diffs = []
                for curr_line, prev_line in zip(fused_output['lane_lines'], prev_output['lane_lines']):
                    if hasattr(curr_line, 'points') and hasattr(prev_line, 'points'):
                        if len(curr_line.points) == len(prev_line.points):
                            point_diff = np.mean(np.abs(np.array(curr_line.points) - np.array(prev_line.points)))
                            lane_diffs.append(point_diff)
                if lane_diffs:
                    avg_lane_diff = np.mean(lane_diffs)
                    lane_consistency = max(0.0, 1.0 - avg_lane_diff * 0.1)  # Scale appropriately
                    consistency_scores.append(lane_consistency)

            # Calculate tracking consistency based on number of tracked objects
            current_tracks = fused_output.get('track_count', 0)
            prev_tracks = prev_output.get('track_count', 0)
            if current_tracks > 0 or prev_tracks > 0:
                track_stability = 1.0 - abs(current_tracks - prev_tracks) / max(current_tracks, prev_tracks, 1) * 0.3
                consistency_scores.append(track_stability)

        # Combine base consistency with tracking quality for more robust measure
        base_consistency = np.mean(consistency_scores) if consistency_scores else 0.7

        # Enhance with tracking quality if available in current output
        track_quality_contribution = 0.0
        if 'track_confidence_avg' in fused_output and fused_output['track_count'] > 0:
            # Higher weight for good tracking quality
            track_quality_contribution = fused_output['track_confidence_avg'] * min(0.2, fused_output['track_count'] * 0.02)

        return min(1.0, base_consistency + track_quality_contribution)


# Singleton instance for use across the system
enhanced_fusion = EnhancedCameraFusion()