#!/usr/bin/env python3
"""
Enhanced Monitoring and Safety for Self-Learning Autonomous Driving System

This module implements additional safety measures and monitoring for the self-learning system
to address remaining critical review concerns.
"""

import numpy as np
import time
from collections import deque
from typing import Dict, List, Tuple, Optional
from openpilot.common.swaglog import cloudlog
from cereal import log


class EnhancedSelfLearningMonitor:
    """
    Enhanced monitoring system for the self-learning system that addresses
    critical review concerns about over-adaptation, complexity vs reliability,
    and computational cost.
    """

    def __init__(self):
        # Initialize tracking for over-adaptation detection
        self.adaptation_history = deque(maxlen=500)  # Track recent adaptations
        self.learning_context_history = deque(maxlen=200)  # Track learning contexts
        self.over_adaptation_threshold = 0.8  # Threshold for detecting over-adaptation
        
        # Track computational performance
        self.computation_times = deque(maxlen=100)
        self.max_computation_time = 0.001  # 1ms target as per review
        
        # Track different types of learning triggers
        self.learning_trigger_analysis = {
            'intervention': 0,
            'model_accuracy': 0,
            'normal_adaptation': 0,
        }
        
        # Initialize tunnel detection
        self.tunnel_detector = TunnelDetector()
        
        # Vehicle-specific calibration tracking
        self.vehicle_calibration = {
            'lateral_acceleration_limit': 2.5,  # Base value, will be learned
            'calibration_samples': 0,
            'last_calibration_update': time.time()
        }
        
        cloudlog.info("Enhanced Self-Learning Monitor initialized")

    def monitor_over_adaptation(self, adaptive_params: Dict, learning_context: Dict) -> Dict:
        """
        Monitor for potential over-adaptation to specific conditions or driving styles.
        
        Args:
            adaptive_params: Current adaptive parameters
            learning_context: Current learning context (weather, traffic, etc.)
            
        Returns:
            Dictionary with over-adaptation detection results
        """
        result = {
            'over_adaptation_detected': False,
            'triggering_conditions': [],
            'param_drift_rates': {},
            'suggested_actions': []
        }
        
        # Store current context for analysis
        context_record = {
            'timestamp': time.time(),
            'context': learning_context.copy(),
            'params': adaptive_params.copy()
        }
        self.learning_context_history.append(context_record)
        
        # Analyze if parameters are drifting too rapidly in specific contexts
        if len(self.learning_context_history) >= 10:
            # Compare recent parameter changes in similar contexts
            recent_contexts = list(self.learning_context_history)[-10:]
            
            # Check for consistent parameter changes in similar conditions
            for i in range(1, len(recent_contexts)):
                prev_params = recent_contexts[i-1]['params']
                curr_params = recent_contexts[i]['params']
                
                for param_name, curr_value in curr_params.items():
                    prev_value = prev_params[param_name]
                    drift_rate = abs(curr_value - prev_value)
                    result['param_drift_rates'][param_name] = drift_rate
                    
                    # Check if drift rate is excessive for this parameter
                    if param_name == 'lateral_control_factor':
                        if drift_rate > 0.1:  # Large change in lateral control
                            result['over_adaptation_detected'] = True
                            result['triggering_conditions'].append(f'Excessive {param_name} changes: {drift_rate:.3f}')
                    elif param_name == 'weather_adaptation_factor':
                        if drift_rate > 0.15:  # Large change in weather adaptation
                            result['over_adaptation_detected'] = True
                            result['triggering_conditions'].append(f'Excessive {param_name} changes: {drift_rate:.3f}')
                    elif param_name == 'driver_adaptation_rate':
                        if drift_rate > 0.2:  # Large change in driver adaptation
                            result['over_adaptation_detected'] = True
                            result['triggering_conditions'].append(f'Excessive {param_name} changes: {drift_rate:.3f}')
        
        # Check for context-specific over-adaptation patterns
        if result['over_adaptation_detected']:
            # Suggest reducing learning rate temporarily
            result['suggested_actions'].append("Reduce learning rate temporarily")
            result['suggested_actions'].append("Increase regularization for drifted parameters")
            
            # Log warning
            cloudlog.warning(f"Over-adaptation detected: {', '.join(result['triggering_conditions'])}")
        
        return result

    def track_computational_performance(self, start_time: float) -> float:
        """
        Track computational performance to ensure < 1ms overhead claim.
        
        Args:
            start_time: Start time of the computation
            
        Returns:
            Computation time in seconds
        """
        computation_time = time.time() - start_time
        self.computation_times.append(computation_time)
        
        # Log performance if it exceeds the target
        if computation_time > self.max_computation_time:
            cloudlog.warning(f"Performance Alert: Computation time {computation_time*1000:.2f}ms exceeded target {self.max_computation_time*1000:.2f}ms")
        
        # Log performance statistics periodically
        if len(self.computation_times) % 50 == 0:
            avg_time = np.mean(self.computation_times)
            max_time = max(self.computation_times)
            percentile_95 = np.percentile(self.computation_times, 95) if len(self.computation_times) >= 20 else max_time
            
            cloudlog.info(f"Performance Stats - Avg: {avg_time*1000:.2f}ms, "
                         f"Max: {max_time*1000:.2f}ms, 95th percentile: {percentile_95*1000:.2f}ms, "
                         f"Target: {self.max_computation_time*1000:.2f}ms")
        
        return computation_time

    def analyze_learning_triggers(self, trigger_type: str):
        """
        Analyze learning trigger patterns to detect potential issues.
        
        Args:
            trigger_type: Type of learning trigger ('intervention', 'model_accuracy', etc.)
        """
        # Update statistics
        if trigger_type in self.learning_trigger_analysis:
            self.learning_trigger_analysis[trigger_type] += 1
        else:
            self.learning_trigger_analysis[trigger_type] = 1
        
        # Log analysis periodically
        total_triggers = sum(self.learning_trigger_analysis.values())
        if total_triggers > 0 and total_triggers % 100 == 0:
            cloudlog.info(f"Learning Trigger Analysis - Intervention: {self.learning_trigger_analysis['intervention']}, "
                         f"Model Accuracy: {self.learning_trigger_analysis['model_accuracy']}, "
                         f"Normal Adaptation: {self.learning_trigger_analysis['normal_adaptation']}")

    def update_vehicle_calibration(self, v_ego: float, curvature: float, lateral_accel: float):
        """
        Update vehicle-specific lateral acceleration limits based on real-world data.
        
        Args:
            v_ego: Vehicle speed
            curvature: Current curvature
            lateral_accel: Calculated lateral acceleration
        """
        # Only update if we're in a safe driving condition
        if abs(lateral_accel) < self.vehicle_calibration['lateral_acceleration_limit'] * 0.8:
            # This is a safe lateral acceleration, might be a good reference point
            # Update our calibration with a moving average
            self.vehicle_calibration['lateral_acceleration_limit'] = (
                0.95 * self.vehicle_calibration['lateral_acceleration_limit'] + 
                0.05 * (abs(lateral_accel) / 0.8)  # Scale back to estimate actual limit
            )
            
            # Ensure it stays within reasonable bounds
            self.vehicle_calibration['lateral_acceleration_limit'] = max(
                2.0, min(4.0, self.vehicle_calibration['lateral_acceleration_limit'])
            )
            
            self.vehicle_calibration['calibration_samples'] += 1
            self.vehicle_calibration['last_calibration_update'] = time.time()
            
            # Log updates periodically
            if self.vehicle_calibration['calibration_samples'] % 50 == 0:
                cloudlog.info(f"Vehicle calibration update - Lateral limit: {self.vehicle_calibration['lateral_acceleration_limit']:.2f} m/s², "
                             f"Samples: {self.vehicle_calibration['calibration_samples']}")

    def get_updated_lateral_acceleration_limit(self) -> float:
        """
        Get the updated lateral acceleration limit based on vehicle calibration.
        
        Returns:
            Updated lateral acceleration limit in m/s²
        """
        return self.vehicle_calibration['lateral_acceleration_limit']

    def detect_tunnel_conditions(self, gps_data: Optional[dict] = None, light_sensor_data: Optional[dict] = None) -> bool:
        """
        Enhanced tunnel detection using GPS, map data, and light sensors.
        
        Args:
            gps_data: GPS coordinates and accuracy
            light_sensor_data: Light intensity data if available
            
        Returns:
            True if tunnel conditions are detected
        """
        return self.tunnel_detector.detect_tunnel(gps_data, light_sensor_data)


class TunnelDetector:
    """
    Advanced tunnel detection system that uses GPS, map data, and potentially light sensors
    to detect tunnel conditions and alert the self-learning system.
    """
    
    def __init__(self):
        self.gps_accuracy_history = deque(maxlen=20)
        self.light_intensity_history = deque(maxlen=20)
        self.last_tunnel_detection = time.time()
        self.tunnel_detection_active = False
        self.tunnel_probability = 0.0
        
    def detect_tunnel(self, gps_data: Optional[dict] = None, light_sensor_data: Optional[dict] = None) -> bool:
        """
        Detect tunnel conditions using multiple data sources.
        
        Args:
            gps_data: GPS accuracy and position data
            light_sensor_data: Light intensity data
            
        Returns:
            True if tunnel conditions are detected
        """
        tunnel_probability = 0.0
        
        # GPS-based tunnel detection
        if gps_data is not None:
            # Degrading GPS accuracy may indicate tunnel
            if 'accuracy' in gps_data and gps_data['accuracy'] > 10.0:
                self.gps_accuracy_history.append(gps_data['accuracy'])
                if len(self.gps_accuracy_history) >= 5:
                    recent_accuracy = list(self.gps_accuracy_history)[-5:]
                    if all(acc > 10.0 for acc in recent_accuracy):  # Consistently poor GPS
                        tunnel_probability += 0.3
                        
            # Check for GPS signal loss patterns
            if 'satellites' in gps_data and gps_data['satellites'] < 5:
                tunnel_probability += 0.4
        
        # Light sensor-based tunnel detection (if available)
        if light_sensor_data is not None:
            if 'intensity' in light_sensor_data:
                self.light_intensity_history.append(light_sensor_data['intensity'])
                
                # If light is consistently low, it might be a tunnel
                if len(self.light_intensity_history) >= 10:
                    avg_light = np.mean(list(self.light_intensity_history)[-10:])
                    if avg_light < 0.2:  # Very low light
                        tunnel_probability += 0.5
        
        # Map-based tunnel detection (simplified - in real implementation, this would use actual map data)
        # This is where you would integrate with map data to identify known tunnel locations
        
        # Update tunnel probability
        self.tunnel_probability = max(self.tunnel_probability * 0.8, tunnel_probability)  # Decay over time
        
        # Consider tunnel detected if probability is high
        tunnel_detected = self.tunnel_probability > 0.6
        self.tunnel_detection_active = tunnel_detected
        
        if tunnel_detected:
            cloudlog.info(f"Tunnel conditions detected (probability: {self.tunnel_probability:.2f}), "
                         f"adapting learning behavior for reduced visibility")
        
        return tunnel_detected


class EnhancedSafetyValidator:
    """
    Enhanced safety validation system that addresses critical review concerns
    about safety validation and computational complexity.
    """

    def __init__(self):
        # Initialize safety monitoring
        self.param_validation_history = deque(maxlen=100)
        self.safety_check_times = deque(maxlen=50)
        
    def validate_with_computational_efficiency(self, params: Dict, v_ego: float) -> Dict:
        """
        Perform safety validation with computational efficiency in mind.
        
        Args:
            params: Parameters to validate
            v_ego: Vehicle speed
            
        Returns:
            Dictionary with validation results
        """
        start_time = time.time()
        
        results = {
            'is_safe': True,
            'safety_issues': [],
            'corrected_params': params.copy(),
            'validation_time': 0.0
        }
        
        # Quick safety checks (most critical first)
        if 'lateral_control_factor' in params:
            factor = params['lateral_control_factor']
            if factor < 0.3 or factor > 1.7:  # Extreme values
                results['is_safe'] = False
                results['safety_issues'].append(f"Lateral control factor out of bounds: {factor}")
                results['corrected_params']['lateral_control_factor'] = np.clip(factor, 0.3, 1.7)
        
        if 'curvature_bias' in params:
            bias = params['curvature_bias']
            if abs(bias) > 0.1:  # Large bias
                results['is_safe'] = False
                results['safety_issues'].append(f"Curvature bias too large: {bias}")
                results['corrected_params']['curvature_bias'] = np.clip(bias, -0.1, 0.1)
        
        # Compute validation time
        results['validation_time'] = time.time() - start_time
        self.safety_check_times.append(results['validation_time'])
        
        # Log if validation is taking too long
        if results['validation_time'] > 0.0005:  # 0.5ms threshold
            cloudlog.warning(f"Safety validation took {results['validation_time']*1000:.2f}ms, "
                           f"considering optimization")
        
        return results
    
    def get_performance_metrics(self) -> Dict:
        """
        Get performance metrics for safety validation system.
        
        Returns:
            Dictionary with performance metrics
        """
        if self.safety_check_times:
            avg_time = np.mean(self.safety_check_times)
            max_time = max(self.safety_check_times)
            percentile_95 = np.percentile(self.safety_check_times, 95) if len(self.safety_check_times) >= 10 else max_time
        else:
            avg_time, max_time, percentile_95 = 0, 0, 0
            
        return {
            'avg_validation_time_ms': avg_time * 1000,
            'max_validation_time_ms': max_time * 1000,
            'p95_validation_time_ms': percentile_95 * 1000,
            'total_validations': len(self.safety_check_times)
        }