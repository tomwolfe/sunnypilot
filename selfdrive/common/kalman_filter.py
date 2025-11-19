"""
Kalman Filter implementation for object tracking in Sunnypilot
Provides robust tracking of objects across camera frames with uncertainty estimation
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class TrackedObject:
    """Represents a tracked object with Kalman filter state"""
    id: int
    state: np.ndarray  # [x, y, vx, vy] - position and velocity
    covariance: np.ndarray  # 4x4 covariance matrix
    age: int = 0
    hits: int = 0
    misses: int = 0
    last_seen_time: float = 0.0
    prob: float = 1.0  # Detection probability


class KalmanFilter1D:
    """
    1D Kalman Filter for simple tracking scenarios
    State: [position, velocity]
    """
    
    def __init__(self, dt: float = 0.05, process_noise: float = 1.0, measurement_noise: float = 1.0):
        """
        Initialize 1D Kalman Filter
        
        Args:
            dt: Time step between measurements
            process_noise: Process noise (system dynamics uncertainty)
            measurement_noise: Measurement noise (sensor uncertainty)
        """
        self.dt = dt
        
        # State vector: [position, velocity]
        self.x = np.array([0., 0.])
        
        # State covariance matrix
        self.P = np.diag([1000., 1000.])  # Initially high uncertainty
        
        # Process noise covariance
        self.Q = np.array([[0.25 * dt**4, 0.5 * dt**3],
                          [0.5 * dt**3, dt**2]]) * process_noise
        
        # Measurement matrix (we only observe position)
        self.H = np.array([1., 0.]).reshape(1, 2)
        
        # Measurement noise covariance
        self.R = np.array([[measurement_noise]])
        
        # State transition matrix
        self.F = np.array([[1., dt],
                          [0., 1.]])

    def predict(self) -> np.ndarray:
        """Predict the next state"""
        # State prediction: x = F * x
        self.x = self.F @ self.x
        
        # Covariance prediction: P = F * P * F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.x

    def update(self, measurement: float) -> np.ndarray:
        """Update the state with a new measurement"""
        # Innovation: y = z - H * x
        y = measurement - self.H @ self.x
        
        # Innovation covariance: S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain: K = P * H^T * S^(-1)
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # State update: x = x + K * y
        self.x = self.x + K @ y.flatten()
        
        # Covariance update: P = (I - K * H) * P
        I = np.eye(len(self.x))
        self.P = (I - K @ self.H) @ self.P
        
        return self.x


class KalmanFilter2D:
    """
    2D Kalman Filter for tracking objects in x,y space
    State: [x, y, vx, vy] - position and velocity in 2D
    """
    
    def __init__(self, dt: float = 0.05, process_noise: float = 1.0, measurement_noise: float = 1.0):
        """
        Initialize 2D Kalman Filter
        
        Args:
            dt: Time step between measurements
            process_noise: Process noise (system dynamics uncertainty)
            measurement_noise: Measurement noise (sensor uncertainty)
        """
        self.dt = dt
        
        # State vector: [x, y, vx, vy]
        self.x = np.array([0., 0., 0., 0.])
        
        # State covariance matrix (4x4)
        self.P = np.diag([1000., 1000., 1000., 1000.])  # Initially high uncertainty
        
        # Process noise covariance (4x4)
        # Using the discrete white noise model
        q_var = process_noise
        self.Q = np.array([
            [0.25*dt**4, 0, 0.5*dt**3, 0],
            [0, 0.25*dt**4, 0, 0.5*dt**3],
            [0.5*dt**3, 0, dt**2, 0],
            [0, 0.5*dt**3, 0, dt**2]
        ]) * q_var
        
        # Measurement matrix (we observe [x, y])
        self.H = np.array([[1., 0., 0., 0.],
                          [0., 1., 0., 0.]])
        
        # Measurement noise covariance (2x2)
        self.R = np.array([[measurement_noise, 0],
                          [0, measurement_noise]])
        
        # State transition matrix (4x4)
        # Constant velocity model
        self.F = np.array([
            [1., 0., dt, 0.],
            [0., 1., 0., dt],
            [0., 0., 1., 0.],
            [0., 0., 0., 1.]
        ])

    def predict(self) -> np.ndarray:
        """Predict the next state"""
        # State prediction: x = F * x
        self.x = self.F @ self.x
        
        # Covariance prediction: P = F * P * F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.x

    def update(self, measurement: np.ndarray) -> np.ndarray:
        """Update the state with a new measurement [x, y]"""
        # Ensure measurement is a 2D vector
        if measurement.ndim == 1 and len(measurement) == 2:
            z = measurement
        else:
            raise ValueError(f"Measurement must be a 2-element array, got shape {measurement.shape}")

        # Innovation: y = z - H * x
        y = z - self.H @ self.x
        
        # Innovation covariance: S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain: K = P * H^T * S^(-1)
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # State update: x = x + K * y
        self.x = self.x + K @ y
        
        # Covariance update: P = (I - K * H) * P
        I = np.eye(len(self.x))
        self.P = (I - K @ self.H) @ self.P
        
        return self.x

    def get_position(self) -> np.ndarray:
        """Get the current position [x, y]"""
        return self.x[:2]

    def get_velocity(self) -> np.ndarray:
        """Get the current velocity [vx, vy]"""
        return self.x[2:]


class ObjectTracker:
    """
    Multi-object tracker using Kalman filters
    Handles data association, track management, and prediction
    """
    
    def __init__(self, max_distance: float = 5.0, max_age: int = 10, min_hits: int = 3):
        """
        Initialize object tracker
        
        Args:
            max_distance: Maximum distance for association (in meters)
            max_age: Maximum age of track before deletion (in frames)
            min_hits: Minimum hits before track is considered confirmed
        """
        self.max_distance = max_distance
        self.max_age = max_age
        self.min_hits = min_hits
        self.trackers = {}  # {track_id: TrackedObject}
        self.next_id = 0
        self.dt = 0.05  # 20 Hz (assuming 20 FPS)

    def update(self, detections: list, current_time: float) -> list:
        """
        Update tracker with new detections
        
        Args:
            detections: List of detections, each with dRel, yRel, prob
            current_time: Current timestamp
            
        Returns:
            List of confirmed tracked objects
        """
        # Predict new positions for all existing trackers
        for track_id, tracker in list(self.trackers.items()):
            # Predict next state
            kf = KalmanFilter2D(dt=self.dt)
            kf.x = tracker.state
            kf.P = tracker.covariance
            predicted_state = kf.predict()
            
            # Update tracker state
            tracker.state = predicted_state
            tracker.covariance = kf.P

        # Associate detections with existing trackers
        matched_trackers = set()
        matched_detections = set()
        
        # For each detection, find the closest tracker
        for det_idx, detection in enumerate(detections):
            det_pos = np.array([detection.dRel, detection.yRel])
            best_track_id = None
            best_distance = float('inf')
            
            for track_id, tracker in self.trackers.items():
                if track_id in matched_trackers:
                    continue
                    
                track_pos = tracker.state[:2]  # [x, y]
                distance = np.linalg.norm(det_pos - track_pos)
                
                if distance < best_distance and distance < self.max_distance:
                    best_distance = distance
                    best_track_id = track_id
            
            # If we found a good match
            if best_track_id is not None:
                # Update the tracker with the new measurement
                kf = KalmanFilter2D(dt=self.dt)
                kf.x = self.trackers[best_track_id].state
                kf.P = self.trackers[best_track_id].covariance
                updated_state = kf.update(det_pos)
                
                # Update tracker
                tracker = self.trackers[best_track_id]
                tracker.state = updated_state
                tracker.covariance = kf.P
                tracker.hits += 1
                tracker.misses = 0
                tracker.last_seen_time = current_time
                tracker.prob = detection.prob  # Update with new detection probability
                
                matched_trackers.add(best_track_id)
                matched_detections.add(det_idx)

        # Create new trackers for unmatched detections
        for det_idx, detection in enumerate(detections):
            if det_idx in matched_detections:
                continue
                
            # Create new Kalman filter for this detection
            kf = KalmanFilter2D(dt=self.dt)
            state = np.array([detection.dRel, detection.yRel, 0., 0.])  # Initialize with position, zero velocity
            kf.x = state
            updated_state = kf.update(np.array([detection.dRel, detection.yRel]))
            
            # Create new tracker
            new_track = TrackedObject(
                id=self.next_id,
                state=updated_state,
                covariance=kf.P,
                hits=1,
                misses=0,
                last_seen_time=current_time,
                prob=detection.prob
            )
            
            self.trackers[self.next_id] = new_track
            self.next_id += 1

        # Mark unmatched trackers (increment misses)
        for track_id, tracker in self.trackers.items():
            if track_id not in matched_trackers:
                tracker.misses += 1

        # Remove old trackers
        for track_id in list(self.trackers.keys()):
            tracker = self.trackers[track_id]
            if tracker.misses > self.max_age or tracker.age > 100:  # Max age of 100 frames
                del self.trackers[track_id]
                continue
                
            tracker.age += 1

        # Return confirmed tracks (those with enough hits)
        confirmed_tracks = []
        for track_id, tracker in self.trackers.items():
            if tracker.hits >= self.min_hits:  # Only return confirmed tracks
                confirmed_tracks.append({
                    'id': track_id,
                    'dRel': float(tracker.state[0]),
                    'yRel': float(tracker.state[1]),
                    'vRel': float(tracker.state[2]),  # Velocity component in dRel direction
                    'prob': tracker.prob,
                    'covariance': tracker.covariance
                })

        return confirmed_tracks


# Global instance for use across the system
object_tracker = ObjectTracker()