"""
Anomaly Detection for sunnypilot - Separated module for advanced anomaly detection

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
from typing import Dict
import time
from collections import deque
from openpilot.common.realtime import DT_MDL


class BasicAnomalyDetector:
    """
    Basic anomaly detection for autonomous driving safety
    Separated from main SafetyMonitor to reduce complexity
    """
    def __init__(self):
        self.velocity_variance_threshold = 0.5  # m/s
        self.acceleration_jerk_threshold = 5.0  # m/s³
        self.steering_rate_threshold = 100.0    # deg/s
        self.velocity_buffer = deque(maxlen=10)  # Store last 10 velocity readings for trend analysis
        self.acceleration_buffer = deque(maxlen=10)  # Store last 10 acceleration readings
        self.confidence_buffer = deque(maxlen=10)  # Store last 10 confidence readings for trend analysis

    def detect_anomalies(self, car_state, model_v2, radar_state):
        """Detect anomalies in vehicle behavior and sensor data"""
        anomalies = {}

        # Velocity consistency check - compare car state with model prediction
        if hasattr(model_v2, 'velocity') and hasattr(model_v2.velocity, 'x') and len(model_v2.velocity.x) > 0:
            vel_diff = abs(car_state.vEgo - model_v2.velocity.x[0])
            if vel_diff > self.velocity_variance_threshold:
                anomalies['velocity_inconsistency'] = {
                    'car_state_vEgo': float(car_state.vEgo),
                    'model_v2_velocity': float(model_v2.velocity.x[0]),
                    'difference': float(vel_diff),
                    'threshold': self.velocity_variance_threshold
                }

        # Acceleration jerk detection
        if len(self.acceleration_buffer) > 0:
            prev_acceleration = self.acceleration_buffer[-1]
            current_jerk = abs(car_state.aEgo - prev_acceleration) / DT_MDL
            if current_jerk > self.acceleration_jerk_threshold:
                anomalies['high_jerk'] = {
                    'current_jerk': float(current_jerk),
                    'threshold': self.acceleration_jerk_threshold,
                    'current_accel': float(car_state.aEgo),
                    'prev_accel': float(prev_acceleration)
                }

        # Add current values to buffers
        self.velocity_buffer.append(float(car_state.vEgo))
        self.acceleration_buffer.append(float(car_state.aEgo))

        # Steering rate check
        if hasattr(car_state, 'steeringRateDeg') and abs(car_state.steeringRateDeg) > self.steering_rate_threshold:
            anomalies['high_steering_rate'] = {
                'steering_rate': float(car_state.steeringRateDeg),
                'threshold': self.steering_rate_threshold
            }

        # Model confidence trend analysis
        if hasattr(model_v2, 'meta') and hasattr(model_v2.meta, 'confidence'):
            # Add current confidence to buffer for trend analysis
            self.confidence_buffer.append(model_v2.meta.confidence)
            # Check if confidence is consistently low over recent readings
            recent_confidence_avg = np.mean(list(self.confidence_buffer)) if len(self.confidence_buffer) > 0 else 1.0
            if model_v2.meta.confidence < 0.5 and recent_confidence_avg < 0.6:
                anomalies['low_confidence_trend'] = {
                    'current_confidence': float(model_v2.meta.confidence),
                    'recent_avg_confidence': float(recent_confidence_avg)
                }

        return anomalies


class AdvancedAnomalyDetector:
    """
    Advanced anomaly detection with additional checks
    """
    def __init__(self):
        self.basic_detector = BasicAnomalyDetector()

    def detect_anomalies(self, car_state, model_v2, radar_state):
        """Detect anomalies using both basic and advanced methods"""
        # Get basic anomalies
        anomalies = self.basic_detector.detect_anomalies(car_state, model_v2, radar_state)
        
        # Add any advanced detection here in the future
        return anomalies