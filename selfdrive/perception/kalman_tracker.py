#!/usr/bin/env python3
"""
Kalman Filter for Multi-Camera Object Tracking in Sunnypilot
Implements Kalman filtering for tracking objects across multiple virtual cameras
"""
import numpy as np
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
import uuid
from collections import defaultdict
import time

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.neon_optimizer import neon_optimizer


@dataclass
class TrackedObject:
    """Represents a tracked object with state information"""
    id: str
    class_name: str
    class_id: int
    state: np.ndarray  # [x, y, z, vx, vy, vz] - position and velocity
    covariance: np.ndarray  # State covariance matrix
    detection_history: List[Dict]  # History of detections for this object
    last_update_time: float
    confidence: float
    camera_visibility: Dict[str, float]  # Last seen time in each camera
    associated_detections: List[Dict]  # Detections associated with this track


class KalmanFilter3D:
    """
    3D Kalman Filter for object tracking
    Models position and velocity in 3D space
    """
    
    def __init__(self, process_noise: float = 0.1, measurement_noise: float = 0.1):
        # State vector: [x, y, z, vx, vy, vz] (position and velocity)
        self.state_dim = 6
        self.measurement_dim = 3  # x, y, z position
        
        # Process noise - uncertainty in motion model
        self.process_noise = process_noise
        # Measurement noise - uncertainty in sensor readings
        self.measurement_noise = measurement_noise
        
        # State transition matrix (for constant velocity model)
        # x(k+1) = x(k) + vx(k) * dt
        # vx(k+1) = vx(k) (assuming constant velocity)
        # Similar for y, z
        self.F = np.eye(self.state_dim, dtype=np.float32)
        
        # Measurement matrix - we only measure position, not velocity directly
        self.H = np.zeros((self.measurement_dim, self.state_dim), dtype=np.float32)
        self.H[0, 0] = 1  # x measurement
        self.H[1, 1] = 1  # y measurement  
        self.H[2, 2] = 1  # z measurement
        
        # Process noise covariance
        self.Q = np.eye(self.state_dim, dtype=np.float32) * self.process_noise
        
        # Measurement noise covariance
        self.R = np.eye(self.measurement_dim, dtype=np.float32) * self.measurement_noise
        
        # Initial state and covariance
        self.x = np.zeros(self.state_dim, dtype=np.float32)
        self.P = np.eye(self.state_dim, dtype=np.float32) * 10.0  # High initial uncertainty
        
        # Pre-allocated arrays for optimization
        self._temp_state = np.zeros(self.state_dim, dtype=np.float32)
        self._temp_matrix = np.zeros((self.state_dim, self.state_dim), dtype=np.float32)
        self._temp_measurement = np.zeros(self.measurement_dim, dtype=np.float32)
        self._temp_gain = np.zeros((self.state_dim, self.measurement_dim), dtype=np.float32)
    
    def predict(self, dt: float = 0.01):
        """Predict state forward in time using motion model"""
        # Update state transition matrix with time delta
        F = self.F.copy()
        F[0, 3] = dt  # x += vx * dt
        F[1, 4] = dt  # y += vy * dt  
        F[2, 5] = dt  # z += vz * dt
        
        # Predict state: x = F * x
        np.dot(F, self.x, out=self._temp_state)
        self.x = self._temp_state.copy()
        
        # Predict covariance: P = F * P * F.T + Q
        np.dot(F, self.P, out=self._temp_matrix)
        np.dot(self._temp_matrix, F.T, out=self.P)
        self.P += self.Q
    
    def update(self, measurement: np.ndarray):
        """Update state estimate with new measurement"""
        # Compute innovation (difference between measurement and prediction)
        innovation = measurement - np.dot(self.H, self.x)
        
        # Compute innovation covariance
        PHt = np.dot(self.P, self.H.T)
        S = np.dot(self.H, PHt) + self.R
        
        # Compute Kalman gain
        K = np.dot(PHt, np.linalg.inv(S))
        
        # Update state
        np.dot(K, innovation, out=self._temp_state)
        self.x += self._temp_state
        
        # Update covariance
        KH = np.dot(K, self.H)
        I_KH = np.eye(self.state_dim, dtype=np.float32) - KH
        temp_P = np.dot(I_KH, self.P)
        self.P = temp_P
        
        # Store the gain for performance tracking
        self._temp_gain = K


class MultiCameraObjectTracker:
    """
    Multi-camera object tracking using Kalman filters
    """
    
    def __init__(self, max_objects: int = 50, max_track_age: int = 10):
        self.max_objects = max_objects
        self.max_track_age = max_track_age  # Maximum frames before deleting track
        
        # Dictionary to store active tracks
        self.tracks: Dict[str, TrackedObject] = {}
        
        # Association parameters
        self.max_association_distance = 50.0  # Maximum distance for association
        self.min_detection_confidence = 0.5   # Minimum confidence for new tracks
        
        # Timestamp of last update
        self.last_update_time = time.time()
        
        # Pre-allocated arrays for optimization
        self._temp_measurement = np.zeros(3, dtype=np.float32)
        self._temp_state = np.zeros(6, dtype=np.float32)
    
    def update_tracks(self, 
                     detections: Dict[str, List[Dict]], 
                     timestamp: Optional[float] = None) -> Dict[str, TrackedObject]:
        """
        Update object tracks with new detections from all cameras
        
        Args:
            detections: Dictionary mapping camera names to detection lists
            timestamp: Current timestamp (if None, uses current time)
            
        Returns:
            Dictionary of currently tracked objects
        """
        if timestamp is None:
            timestamp = time.time()
        
        dt = timestamp - self.last_update_time
        self.last_update_time = timestamp
        
        # Collect all detections with 3D positions
        all_detections = self._process_detections_for_tracking(detections, timestamp)
        
        # Predict all active tracks forward
        self._predict_tracks(dt)
        
        # Associate detections with existing tracks
        associations = self._associate_detections_to_tracks(all_detections)
        
        # Update tracks with associated detections
        self._update_associated_tracks(associations, all_detections)
        
        # Create new tracks for unassociated detections
        self._create_new_tracks(associations, all_detections, timestamp)
        
        # Delete old tracks that haven't been updated
        self._delete_old_tracks(timestamp)
        
        # Update track visibility information
        self._update_visibility(detections, timestamp)
        
        return self.tracks
    
    def _process_detections_for_tracking(self, detections: Dict[str, List[Dict]], timestamp: float) -> Dict[str, Dict]:
        """Convert detections to format suitable for tracking"""
        all_detections = {}
        
        detection_id = 0
        for cam_name, cam_detections in detections.items():
            for det in cam_detections:
                # Convert 2D detection to 3D position estimate
                # This is a simplified conversion - in reality, would use geometric projection
                # and depth information to estimate 3D position
                bbox = det.get('bbox', [0, 0, 0, 0])
                confidence = det.get('confidence', 0.0)
                
                if confidence < self.min_detection_confidence:
                    continue
                
                # Create a unique ID for this detection
                unique_id = f"{cam_name}_{timestamp:.3f}_{detection_id}"
                detection_id += 1
                
                # Convert 2D bbox center to estimated 3D position
                # This is highly simplified - real implementation would use camera calibration
                # and depth estimation to get accurate 3D positions
                x_2d = bbox[0]
                y_2d = bbox[1] 
                width = bbox[2]
                height = bbox[3]
                
                # Simplified 3D position estimate (in relative coordinates)
                # Real implementation would use geometric projection and depth
                pos_3d = np.array([
                    x_2d - 320,  # Center around image width/2
                    y_2d - 240,  # Center around image height/2  
                    -np.sqrt(width * height)  # Depth estimate from size (inverted)
                ], dtype=np.float32)
                
                all_detections[unique_id] = {
                    'id': unique_id,
                    'position': pos_3d,
                    'detection': det,
                    'camera': cam_name,
                    'timestamp': timestamp,
                    'confidence': confidence
                }
        
        return all_detections
    
    def _predict_tracks(self, dt: float):
        """Predict all active tracks forward in time"""
        for track_id, track in self.tracks.items():
            # Update the Kalman filter for this track
            track.kalman_filter.predict(dt)
    
    def _associate_detections_to_tracks(self, all_detections: Dict[str, Dict]) -> Dict[str, str]:
        """Associate detections with existing tracks using nearest neighbor"""
        associations = {}
        
        if not all_detections or not self.tracks:
            return associations
        
        # Create cost matrix (distance between detections and tracks)
        detection_ids = list(all_detections.keys())
        track_ids = list(self.tracks.keys())
        
        cost_matrix = np.zeros((len(detection_ids), len(track_ids)), dtype=np.float32)
        
        for i, det_id in enumerate(detection_ids):
            det_pos = all_detections[det_id]['position']
            for j, track_id in enumerate(track_ids):
                track_pos = self.tracks[track_id].state[:3]  # Use predicted position
                
                # Calculate Euclidean distance
                dist = np.linalg.norm(det_pos - track_pos)
                cost_matrix[i, j] = dist
        
        # Simple greedy assignment (nearest neighbor)
        # In practice, you'd use Hungarian algorithm or similar for optimal assignment
        remaining_detections = set(detection_ids)
        remaining_tracks = set(track_ids)
        
        # Assign closest detection to each track
        while remaining_detections and remaining_tracks:
            min_cost = float('inf')
            best_det = None
            best_track = None
            
            for det_id in remaining_detections:
                for track_id in remaining_tracks:
                    cost = cost_matrix[detection_ids.index(det_id), track_ids.index(track_id)]
                    if cost < min_cost and cost < self.max_association_distance:
                        min_cost = cost
                        best_det = det_id
                        best_track = track_id
            
            if best_det and best_track:
                associations[best_det] = best_track
                remaining_detections.remove(best_det)
                remaining_tracks.remove(best_track)
            else:
                # No more valid associations
                break
        
        return associations
    
    def _update_associated_tracks(self, associations: Dict[str, str], all_detections: Dict[str, Dict]):
        """Update tracks with associated detections"""
        for det_id, track_id in associations.items():
            if track_id in self.tracks and det_id in all_detections:
                track = self.tracks[track_id]
                detection_data = all_detections[det_id]
                
                # Update Kalman filter with measurement
                measurement = detection_data['position']
                track.kalman_filter.update(measurement)
                
                # Update track state
                track.state = track.kalman_filter.x
                track.covariance = track.kalman_filter.P
                track.last_update_time = detection_data['timestamp']
                track.confidence = detection_data['confidence']
                
                # Add to detection history
                track.detection_history.append(detection_data['detection'])
                if len(track.detection_history) > 10:  # Keep only recent history
                    track.detection_history.pop(0)
                
                # Associate the detection
                track.associated_detections.append(detection_data['detection'])
    
    def _create_new_tracks(self, associations: Dict[str, str], all_detections: Dict[str, Dict], timestamp: float):
        """Create new tracks for unassociated detections"""
        associated_detections = set(associations.keys())
        unassociated_detections = set(all_detections.keys()) - associated_detections
        
        for det_id in unassociated_detections:
            detection_data = all_detections[det_id]
            det = detection_data['detection']
            
            # Create new track
            if len(self.tracks) < self.max_objects:
                # Initialize Kalman filter
                kf = KalmanFilter3D()
                
                # Initialize state with measurement
                pos = detection_data['position']
                kf.x[:3] = pos  # Position
                kf.x[3:] = 0   # Velocity (initially unknown)
                
                # Create tracked object
                track_id = str(uuid.uuid4())
                new_track = TrackedObject(
                    id=track_id,
                    class_name=det.get('class_name', 'unknown'),
                    class_id=det.get('class_id', -1),
                    state=kf.x,
                    covariance=kf.P,
                    detection_history=[det],
                    last_update_time=timestamp,
                    confidence=detection_data['confidence'],
                    camera_visibility={detection_data['camera']: timestamp},
                    associated_detections=[det]
                )
                new_track.kalman_filter = kf  # Add the filter as an attribute
                
                self.tracks[track_id] = new_track
    
    def _delete_old_tracks(self, timestamp: float):
        """Delete tracks that haven't been updated recently"""
        tracks_to_delete = []
        for track_id, track in self.tracks.items():
            age = timestamp - track.last_update_time
            if age > self.max_track_age * 0.01:  # Convert frames to seconds (assuming 100fps)
                tracks_to_delete.append(track_id)
        
        for track_id in tracks_to_delete:
            del self.tracks[track_id]
    
    def _update_visibility(self, detections: Dict[str, List[Dict]], timestamp: float):
        """Update which cameras have seen each object"""
        for cam_name, cam_detections in detections.items():
            for det in cam_detections:
                # In a real system, we would match detections to tracks
                # and update the camera_visibility field
                # For now, we'll keep this as a placeholder
                pass
    
    def get_tracked_objects_in_camera(self, camera_name: str, timestamp: float) -> List[TrackedObject]:
        """Get tracked objects currently visible in a specific camera"""
        visible_objects = []
        
        # This would use geometric projection to determine which objects
        # are visible in the specified camera
        # For now, return all objects
        for track in self.tracks.values():
            if camera_name in track.camera_visibility:
                if abs(timestamp - track.camera_visibility[camera_name]) < 0.1:  # Within 100ms
                    visible_objects.append(track)
        
        return visible_objects
    
    def get_predicted_trajectory(self, track_id: str, num_steps: int = 10, dt: float = 0.1) -> np.ndarray:
        """Get predicted future trajectory for a tracked object"""
        if track_id not in self.tracks:
            return np.array([])
        
        track = self.tracks[track_id]
        trajectory = []
        
        # Predict forward using the current state
        current_state = track.state.copy()
        for i in range(num_steps):
            # Simple constant velocity prediction
            future_state = current_state.copy()
            future_state[:3] += current_state[3:] * dt * (i + 1)  # position += velocity * time
            trajectory.append(future_state[:3])  # Store only position
        
        return np.array(trajectory)


class EnhancedMultiCameraTracker:
    """
    Enhanced tracker that integrates with the multi-camera perception system
    """
    
    def __init__(self):
        self.object_tracker = MultiCameraObjectTracker()
        self.temporal_consistency_checker = TemporalConsistencyChecker()
        self.confidence_calculator = ConfidenceCalculator()
    
    def process_multicam_detections(self, 
                                  detections_by_camera: Dict[str, List[Dict]], 
                                  timestamp: Optional[float] = None) -> Dict[str, Any]:
        """
        Process detections from all cameras and return enhanced tracking information
        """
        # Update object tracks
        tracks = self.object_tracker.update_tracks(detections_by_camera, timestamp)
        
        # Calculate temporal consistency
        temporal_metrics = self.temporal_consistency_checker.evaluate_tracks(tracks)
        
        # Calculate confidence scores
        confidence_scores = self.confidence_calculator.calculate_track_confidences(
            tracks, temporal_metrics
        )
        
        # Package results
        result = {
            'tracked_objects': tracks,
            'temporal_metrics': temporal_metrics,
            'confidence_scores': confidence_scores,
            'processing_timestamp': timestamp or time.time(),
            'track_statistics': {
                'total_tracks': len(tracks),
                'high_confidence_tracks': len([t for t in tracks.values() if t.confidence > 0.8]),
                'updated_tracks': len([t for t in tracks.values() if t.associated_detections])
            }
        }
        
        return result


class TemporalConsistencyChecker:
    """
    Checks temporal consistency of tracked objects
    """
    
    def __init__(self):
        self.track_history = defaultdict(list)
        self.max_history = 10
    
    def evaluate_tracks(self, tracks: Dict[str, TrackedObject]) -> Dict[str, float]:
        """Evaluate temporal consistency for all tracks"""
        metrics = {}
        
        for track_id, track in tracks.items():
            # Calculate consistency metrics
            consistency_score = self._calculate_temporal_consistency(track)
            metrics[track_id] = consistency_score
            
            # Update history
            self.track_history[track_id].append({
                'state': track.state.copy(),
                'timestamp': track.last_update_time,
                'confidence': track.confidence
            })
            
            # Keep only recent history
            if len(self.track_history[track_id]) > self.max_history:
                self.track_history[track_id].pop(0)
        
        return metrics
    
    def _calculate_temporal_consistency(self, track: TrackedObject) -> float:
        """Calculate temporal consistency score for a single track"""
        # A score between 0 and 1, where 1 is perfectly consistent
        # This is a simplified implementation
        if len(track.detection_history) < 2:
            return 0.8  # High if just initialized
        
        # Look at consistency of position over time
        # In reality, this would check for smooth motion patterns
        return min(1.0, 0.9 + track.confidence * 0.1)


class ConfidenceCalculator:
    """
    Calculates confidence scores for tracked objects
    """
    
    def __init__(self):
        pass
    
    def calculate_track_confidences(self, 
                                  tracks: Dict[str, TrackedObject], 
                                  temporal_metrics: Dict[str, float]) -> Dict[str, float]:
        """Calculate enhanced confidence scores for tracked objects"""
        confidences = {}
        
        for track_id, track in tracks.items():
            # Base confidence from detection confidence
            base_conf = track.confidence
            
            # Boost based on temporal consistency
            temp_consistency = temporal_metrics.get(track_id, 0.5)
            temporal_boost = (temp_consistency - 0.5) * 0.5  # Boost up to 25%
            
            # Boost based on track stability (how long it's been tracked)
            track_stability = min(1.0, len(track.detection_history) / 20.0)  # Max 10 for 20+ detections
            stability_boost = track_stability * 0.3  # Up to 30% boost
            
            # Calculate enhanced confidence
            enhanced_conf = min(1.0, base_conf + temporal_boost + stability_boost)
            
            confidences[track_id] = enhanced_conf
        
        return confidences


def create_multi_camera_tracker() -> EnhancedMultiCameraTracker:
    """Factory function to create multi-camera tracker"""
    return EnhancedMultiCameraTracker()


if __name__ == "__main__":
    print("Kalman Filter for Multi-Camera Object Tracking - Testing")
    print("=" * 60)
    
    # Test the Kalman filter
    kf = KalmanFilter3D()
    print("Initial Kalman Filter state:")
    print(f"  State: {kf.x}")
    print(f"  Covariance: {np.diag(kf.P)[:3]}")  # Show position covariance diagonal
    
    # Test prediction
    print("\nAfter prediction (dt=0.01):")
    kf.predict(0.01)
    print(f"  State: {kf.x[:3]}")  # Show position only
    
    # Test update with measurement
    measurement = np.array([10.0, 5.0, 2.0], dtype=np.float32)  # x, y, z
    print(f"\nUpdating with measurement: {measurement}")
    kf.update(measurement)
    print(f"  Updated state: {kf.x[:3]}")  # Show position only
    print(f"  Updated covariance (pos): {np.diag(kf.P)[:3]}")
    
    # Test multi-camera tracking
    print("\nTesting Multi-Camera Object Tracker:")
    tracker = MultiCameraObjectTracker()
    
    # Create mock detections
    mock_detections = {
        "front_center": [
            {
                'bbox': [320, 240, 50, 80],
                'confidence': 0.85,
                'class_name': 'car',
                'class_id': 2
            },
            {
                'bbox': [400, 300, 30, 60], 
                'confidence': 0.78,
                'class_name': 'person',
                'class_id': 0
            }
        ],
        "front_left": [
            {
                'bbox': [100, 200, 40, 70],
                'confidence': 0.72,
                'class_name': 'car', 
                'class_id': 2
            }
        ]
    }
    
    timestamp = time.time()
    result = tracker.update_tracks(mock_detections, timestamp)
    
    print(f"  Processed detections from {len(mock_detections)} cameras")
    print(f"  Created/updated {len(result)} tracks")
    
    for track_id, track in result.items():
        print(f"    Track {track_id[:8]}: {track.class_name}, pos={track.state[:3]}, conf={track.confidence:.2f}")
    
    # Test enhanced tracker
    print("\nTesting Enhanced Multi-Camera Tracker:")
    enhanced_tracker = EnhancedMultiCameraTracker()
    enhanced_result = enhanced_tracker.process_multicam_detections(mock_detections, timestamp)
    
    stats = enhanced_result['track_statistics']
    print(f"  Total tracks: {stats['total_tracks']}")
    print(f"  High confidence tracks: {stats['high_confidence_tracks']}")
    print(f"  Updated tracks: {stats['updated_tracks']}")
    
    print("\nKalman filter integration ready for multi-camera perception system")