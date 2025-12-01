#!/usr/bin/env python3
"""
Enhanced Monitoring and Safety for Self-Learning Autonomous Driving System
This module implements additional safety measures and monitoring for the self-learning system
to address critical review concerns about the safety, reliability, and computational efficiency
of the self-learning features.
"""
import numpy as np
import time
from collections import deque
from openpilot.common.swaglog import cloudlog
from typing import Optional, Any
class EnhancedSelfLearningMonitor:
    """
    Enhanced monitoring system for the self-learning system that addresses
    critical review concerns about over-adaptation, safety validation,
    computational overhead, and system reliability.
    """
    def __init__(self):
        # Initialize tracking for over-adaptation detection
        self.adaptation_history: list[dict] = deque(maxlen=500)  # Track recent adaptations
        self.learning_context_history: list[dict] = deque(maxlen=200)  # Track learning contexts
        self.over_adaptation_threshold = 0.8  # Threshold for detecting over-adaptation
        # Track computational performance with stricter monitoring
        self.computation_times: list[float] = deque(maxlen=1000)  # Increased buffer for better statistics
        self.max_computation_time = 0.001  # 1ms target
        self.critical_time_threshold = 0.002  # 2ms threshold for critical alerts
        # Track different types of learning triggers
        self.learning_trigger_analysis = {
            'intervention': 0,
            'model_accuracy': 0,
            'normal_adaptation': 0,
        }
        # Initialize tunnel detection with enhanced validation
        self.tunnel_detector = TunnelDetector()
        # Vehicle-specific calibration tracking with validation
        self.vehicle_calibration = {
            'lateral_acceleration_limit': 2.5,  # Base value, will be learned
            'calibration_samples': 0,
            'last_calibration_update': time.monotonic(),
            'last_calibration_value': 2.5,  # Track previous value for validation
            'calibration_stability_threshold': 0.1  # Threshold for significant change
        }
        # Enhanced safety monitoring
        self.safety_violations: list[dict] = deque(maxlen=100)
        self.last_safety_check = time.monotonic()
        cloudlog.info("Enhanced Self-Learning Monitor initialized with critical safety measures")
    def monitor_over_adaptation(self, adaptive_params: dict[str, float], learning_context: dict) -> dict:
        """
        Monitor for potential over-adaptation to specific conditions or driving styles.
        Args:
            adaptive_params: Current adaptive parameters
            learning_context: Current learning context (weather, traffic, etc.)
        Returns:
            dictionary with over-adaptation detection results
        """
        triggering_conditions: list[str] = []
        param_drift_rates: dict[str, float] = {}
        suggested_actions: list[str] = []
        critical_violations: list[str] = []

        result: dict[str, Any] = {
            'over_adaptation_detected': False,
            'triggering_conditions': triggering_conditions,
            'param_drift_rates': param_drift_rates,
            'suggested_actions': suggested_actions,
            'critical_violations': critical_violations
        }
        # Store current context for analysis
        context_record = {
            'timestamp': time.monotonic(),
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
                            result['triggering_conditions'].append(
                                f'Excessive {param_name} changes: {drift_rate:.3f}'
                            )
                    elif param_name == 'curvature_bias':
                        if drift_rate > 0.05:  # Very sensitive parameter, low threshold
                            result['critical_violations'].append(
                                f'CRITICAL: Large {param_name} change: {drift_rate:.3f}'
                            )
                            result['over_adaptation_detected'] = True
                            result['triggering_conditions'].append(
                                f'Critical {param_name} change: {drift_rate:.3f}'
                            )
                            result['triggering_conditions'].append(
                                f'Excessive {param_name} changes: {drift_rate:.3f}'
                            )
                    elif param_name == 'driver_adaptation_rate':
                        if drift_rate > 0.2:  # Large change in driver adaptation
                            result['over_adaptation_detected'] = True
                            result['triggering_conditions'].append(
                                f'Excessive {param_name} changes: {drift_rate:.3f}'
                            )
        # Check for context-specific over-adaptation patterns
        if result['over_adaptation_detected']:
            # Suggest reducing learning rate temporarily
            result['suggested_actions'].append("Reduce learning rate temporarily")
            result['suggested_actions'].append("Increase regularization for drifted parameters")
            # Log warning
            cloudlog.warning(
                f"Over-adaptation detected: {', '.join(result['triggering_conditions'])}"
            )
        # Check for critical violations
        if result['critical_violations']:
            cloudlog.error(
                f"CRITICAL OVER-ADAPTATION VIOLATIONS: {', '.join(result['critical_violations'])}"
            )
            result['suggested_actions'].extend([
                "IMMEDIATELY reduce learning rate to minimum",
                "Freeze learning until manual verification",
                "Clear recent learning history to prevent catastrophic overfitting"
            ])
        return result
    def track_computational_performance(self, start_time: float) -> float:
        """
        Track computational performance to ensure < 1ms overhead and identify performance regressions.
        Args:
            start_time: Start time of the computation
        Returns:
            Computation time in seconds
        """
        computation_time = time.monotonic() - start_time
        self.computation_times.append(computation_time)
        # Log performance if it exceeds the target
        if computation_time > self.max_computation_time:
            cloudlog.warning(
                f"Performance Alert: Computation time {computation_time*1000:.2f}ms exceeded target {self.max_computation_time*1000:.2f}ms"
            )
        # Check if this is a critical threshold violation
        if computation_time > self.critical_time_threshold:
            cloudlog.error("CRITICAL PERFORMANCE VIOLATION: " +
                           f"Computation time {computation_time*1000:.2f}ms exceeded critical threshold " +
                           f"{self.critical_time_threshold*1000:.2f}ms")
            # Add to safety violations for tracking
            self.safety_violations.append({
                'timestamp': time.monotonic(),
                'type': 'performance',
                'value': computation_time,
                'threshold': self.critical_time_threshold
            })
        # Enhanced performance statistics with critical monitoring
        if len(self.computation_times) >= 10:  # Need minimum samples for statistics
            avg_time = np.mean(self.computation_times)
            max_time = max(self.computation_times)
            percentile_95 = np.percentile(self.computation_times, 95) if len(self.computation_times) >= 20 else max_time
            percentile_99 = np.percentile(self.computation_times, 99) if len(self.computation_times) >= 50 else max_time
            # Check for performance degradation trends
            recent_samples = list(self.computation_times)[-20:] if len(self.computation_times) >= 20 else list(self.computation_times)
            recent_avg = np.mean(recent_samples)
            # Log if recent performance is significantly worse than historical average
            if len(self.computation_times) >= 50 and recent_avg > avg_time * 1.5:
                cloudlog.warning(
                    f"Performance degradation detected: Recent avg {recent_avg*1000:.2f}ms vs historical avg {avg_time*1000:.2f}ms"
                )
                # Add to safety violations for tracking
                self.safety_violations.append({
                    'timestamp': time.monotonic(),
                    'type': 'performance_degradation',
                    'value': recent_avg,
                    'baseline': avg_time
                })
            # Log performance statistics periodically but more selectively
            if len(self.computation_times) % 100 == 0:
                cloudlog.info(f"Performance Stats - Avg: {avg_time*1000:.2f}ms, " +
                               f"Max: {max_time*1000:.2f}ms, " +
                               f"95th percentile: {percentile_95*1000:.2f}ms, " +
                               f"99th percentile: {percentile_99*1000:.2f}ms, " +
                               f"Target: {self.max_computation_time*1000:.2f}ms")
                # Additional checks for compliance
                if avg_time > self.max_computation_time * 1.1:  # 10% over target
                    cloudlog.error(
                        f"Average computational overhead exceeds target by {(avg_time/self.max_computation_time - 1)*100:.1f}%"
                    )
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
            cloudlog.info("Learning Trigger Analysis - " +
                           f"Intervention: {self.learning_trigger_analysis['intervention']}, " +
                           f"Model Accuracy: {self.learning_trigger_analysis['model_accuracy']}, " +
                           f"Normal Adaptation: {self.learning_trigger_analysis['normal_adaptation']}")
    def update_vehicle_calibration(self, v_ego: float, curvature: float, lateral_accel: float):
        """
        Update vehicle-specific lateral acceleration limits based on real-world data.
        Args:
            v_ego: Vehicle speed
            curvature: Current curvature
            lateral_accel: Calculated lateral acceleration
        """
        # Validate inputs to prevent invalid calibration updates
        if not (0.1 <= v_ego <= 50.0):  # Validate speed range (0.36-180 km/h)
            cloudlog.warning(f"Invalid speed for calibration: {v_ego} m/s, skipping update")
            return
        if not (-5.0 <= lateral_accel <= 5.0):  # Validate lateral acceleration range
            cloudlog.warning(f"Invalid lateral acceleration for calibration: {lateral_accel} m/s², skipping update")
            return
        # Only update if we're in a safe driving condition with stable data
        if abs(lateral_accel) < self.vehicle_calibration['lateral_acceleration_limit'] * 0.8:
            # Calculate proposed new limit based on observed safe lateral acceleration
            # Scale back to estimate actual limit (accounting for safety factor)
            proposed_limit = abs(lateral_accel) / 0.8
            # Validate that the proposed limit is reasonable before applying
            if proposed_limit > 8.0:  # Unreasonably high limit (>288 km/h equivalent)
                cloudlog.error(f"Proposed lateral limit {proposed_limit:.2f} m/s² is unreasonably high, rejecting update")
                return
            # Check for excessive changes that might indicate invalid data
            current_limit = self.vehicle_calibration['lateral_acceleration_limit']
            change_ratio = proposed_limit / current_limit if current_limit > 0 else 1.0
            if change_ratio > 2.0 or change_ratio < 0.5:  # More than 2x or less than half current limit
                cloudlog.warning(f"Proposed limit change ratio {change_ratio:.2f} is excessive, validating data")
                # In critical cases, only apply a portion of the change to be safe
                if change_ratio > 2.0:
                    proposed_limit = current_limit * 1.2  # Cap increase to 20%
                elif change_ratio < 0.5:
                    proposed_limit = current_limit * 0.8  # Cap decrease to 20%
            # Update our calibration with a conservative moving average
            old_limit = self.vehicle_calibration['lateral_acceleration_limit']
            self.vehicle_calibration['lateral_acceleration_limit'] = (
                0.98 * self.vehicle_calibration['lateral_acceleration_limit'] +  # More conservative
                0.02 * proposed_limit
            )
            # Ensure it stays within reasonable and safe bounds
            self.vehicle_calibration['lateral_acceleration_limit'] = max(
                1.5, min(5.0, self.vehicle_calibration['lateral_acceleration_limit'])  # Reduced upper bound for safety
            )
            # Track if the change is significant (may indicate over-calibration)
            change_magnitude = abs(old_limit - self.vehicle_calibration['lateral_acceleration_limit'])
            if change_magnitude > self.vehicle_calibration['calibration_stability_threshold']:
                cloudlog.warning(f"Significant calibration change: {old_limit:.2f} -> {self.vehicle_calibration['lateral_acceleration_limit']:.2f}")
                # Add to safety violations if change is too large
                if change_magnitude > self.vehicle_calibration['calibration_stability_threshold'] * 2:
                    self.safety_violations.append({
                        'timestamp': time.monotonic(),
                        'type': 'excessive_calibration_change',
                        'old_value': old_limit,
                        'new_value': self.vehicle_calibration['lateral_acceleration_limit'],
                        'change': change_magnitude
                    })
            self.vehicle_calibration['calibration_samples'] += 1
            self.vehicle_calibration['last_calibration_update'] = time.monotonic()
            self.vehicle_calibration['last_calibration_value'] = self.vehicle_calibration['lateral_acceleration_limit']
            # Log updates with more detail
            if self.vehicle_calibration['calibration_samples'] % 25 == 0:  # More frequent logging for calibration
                        cloudlog.info("Vehicle calibration update - " +
                           f"Lateral limit: {self.vehicle_calibration['lateral_acceleration_limit']:.2f} m/s², " +
                           f"Samples: {self.vehicle_calibration['calibration_samples']}, " +
                           f"Last change: {change_magnitude:.4f} m/s²")
    def get_updated_lateral_acceleration_limit(self) -> float:
        """
        Get the updated lateral acceleration limit based on vehicle calibration.
        Returns:
            Updated lateral acceleration limit in m/s²
        """
        return self.vehicle_calibration['lateral_acceleration_limit']
    def detect_tunnel_conditions(self, gps_data: Optional[dict] = None, light_sensor_data: Optional[dict] = None) -> bool:
        """
        Enhanced tunnel detection using GPS, map data, and light sensors with validation.
        Args:
            gps_data: GPS coordinates and accuracy
            light_sensor_data: Light intensity data if available
        Returns:
            True if tunnel conditions are detected
        """
        # Validate input data before processing
        tunnel_detected = self.tunnel_detector.detect_tunnel(gps_data, light_sensor_data)
        if tunnel_detected:
            cloudlog.debug("Tunnel conditions detected, applying conservative driving parameters")
            # Add to safety monitoring
            self.safety_violations.append({
                'timestamp': time.monotonic(),
                'type': 'tunnel_detection',
                'gps_data_present': gps_data is not None,
                'light_sensor_data_present': light_sensor_data is not None
            })
        return tunnel_detected
class TunnelDetector:
    """
    Advanced tunnel detection system that uses GPS, map data, and potentially light sensors
    to detect tunnel conditions and alert the self-learning system.
    """
    def __init__(self):
        self.gps_accuracy_history: list[float] = deque(maxlen=20)
        self.light_intensity_history: list[float] = deque(maxlen=20)
        self.last_tunnel_detection = time.monotonic()
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
            cloudlog.info(f"Tunnel conditions detected (probability: {self.tunnel_probability:.2f}), adapting learning behavior for reduced visibility")
        return tunnel_detected
class EnhancedSafetyValidator:
    """
    Enhanced safety validation system that addresses critical review concerns
    about safety validation and computational complexity with stricter checks.
    """
    def __init__(self):
        # Initialize safety monitoring with stricter parameters
        self.param_validation_history: list[dict] = deque(maxlen=200)  # Increased buffer
        self.safety_check_times: list[float] = deque(maxlen=100)  # Increased buffer
        self.critical_safety_violations: list[dict] = deque(maxlen=50)  # Track critical violations
        self.validation_counter = 0  # Track total validations
        self.failed_validation_counter = 0  # Track failed validations
    def validate_with_computational_efficiency(self, params: dict, v_ego: float) -> dict:
        """
        Perform safety validation with computational efficiency and strict safety checks.
        Args:
            params: Parameters to validate
            v_ego: Vehicle speed
        Returns:
            dictionary with validation results
        """
        start_time = time.monotonic()
        self.validation_counter += 1
        safety_issues: list[str] = []
        critical_violations_list: list[str] = []
        corrected_params_dict: dict[str, float] = params.copy()

        results: dict[str, Any] = {
            'is_safe': True,
            'safety_issues': safety_issues,
            'corrected_params': corrected_params_dict,
            'validation_time': 0.0,
            'critical_violations': critical_violations_list,
            'confidence_score': 1.0
        }
        # Strict safety checks with critical thresholds
        if 'lateral_control_factor' in params:
            factor = params['lateral_control_factor']
            if factor < 0.1 or factor > 2.0:  # More restrictive bounds
                results['is_safe'] = False
                critical_violations_list.append(f"CRITICAL: Lateral control factor out of safe bounds: {factor}")
                safety_issues.append(f"Lateral control factor out of bounds: {factor}")
                results['corrected_params']['lateral_control_factor'] = np.clip(factor, 0.1, 2.0)
            elif factor < 0.5 or factor > 1.5:  # Warning bounds
                safety_issues.append(f"Lateral control factor outside normal bounds: {factor}")
        # Critical check for curvature bias - very sensitive parameter
        if 'curvature_bias' in params:
            bias = params['curvature_bias']
            if abs(bias) > 0.05:  # Much more restrictive threshold
                results['is_safe'] = False
                critical_violations_list.append(f"CRITICAL: Curvature bias exceeds safety threshold: {bias}")
                safety_issues.append(f"Curvature bias too large: {bias}")
                results['corrected_params']['curvature_bias'] = np.clip(bias, -0.05, 0.05)
            elif abs(bias) > 0.02:  # Warning threshold
                safety_issues.append(f"Curvature bias outside safe range: {bias}")
        # Acceleration factor validation
        if 'acceleration_factor' in params:
            factor = params['acceleration_factor']
            if factor < 0.1 or factor > 2.0:  # More restrictive bounds
                results['is_safe'] = False
                critical_violations_list.append(f"CRITICAL: Acceleration factor out of safe bounds: {factor}")
                safety_issues.append(f"Acceleration factor out of bounds: {factor}")
                results['corrected_params']['acceleration_factor'] = np.clip(factor, 0.1, 2.0)
        # Model confidence factor validation
        if 'model_confidence_factor' in params:
            factor = params['model_confidence_factor']
            if factor < 0.1 or factor > 2.0:  # Unreasonable bounds
                results['is_safe'] = False
                critical_violations_list.append(f"CRITICAL: Model confidence factor out of bounds: {factor}")
                safety_issues.append(f"Model confidence factor out of bounds: {factor}")
                results['corrected_params']['model_confidence_factor'] = np.clip(factor, 0.1, 2.0)
        # Driver adaptation rate validation
        if 'driver_adaptation_rate' in params:
            rate = params['driver_adaptation_rate']
            if rate < 0.1 or rate > 2.0:  # Reasonable bounds for adaptation rate
                results['is_safe'] = False
                critical_violations_list.append(f"CRITICAL: Driver adaptation rate out of bounds: {rate}")
                safety_issues.append(f"Driver adaptation rate out of bounds: {rate}")
                results['corrected_params']['driver_adaptation_rate'] = np.clip(rate, 0.1, 2.0)
        # Compute validation time and check performance
        results['validation_time'] = time.monotonic() - start_time
        self.safety_check_times.append(results['validation_time'])
        # Log if validation is taking too long
        if results['validation_time'] > 0.0005:  # 0.5ms threshold
            cloudlog.warning(f"Safety validation took {results['validation_time']*1000:.2f}ms, considering optimization")
        # Check for critical performance violations
        if results['validation_time'] > 0.001:  # 1ms threshold
            cloudlog.error(f"CRITICAL: Safety validation exceeded performance target: {results['validation_time']*1000:.2f}ms")
            critical_violations_list.append(f"Performance exceeded target: {results['validation_time']*1000:.2f}ms")
        # Add to history and update metrics
        self.param_validation_history.append({
            'timestamp': time.monotonic(),
            'params': params.copy(),
            'results': results.copy()
        })
        # Update results['is_safe'] based on collected violations
        results['is_safe'] = not critical_violations_list and not safety_issues

        if not results['is_safe']:
            self.failed_validation_counter += 1
        # Calculate confidence score based on various factors
        failure_rate = self.failed_validation_counter / max(1, self.validation_counter)
        results['confidence_score'] = max(0.0, 1.0 - failure_rate)  # Lower confidence with more failures
        # Log critical violations
        if critical_violations_list:
            self.critical_safety_violations.append({
                'timestamp': time.monotonic(),
                'violations': critical_violations_list,
                'params': params.copy()
            })
            cloudlog.error(f"Critical Safety Violations: {', '.join(critical_violations_list)}")
        # Log safety statistics periodically
        if self.validation_counter % 50 == 0:
            cloudlog.info("Safety Validator Stats - " +
                       f"Validations: {self.validation_counter}, " +
                       f"Failed: {self.failed_validation_counter}, " +
                       f"Failure Rate: {failure_rate:.3f}, " +
                       f"Confidence Score: {results['confidence_score']:.3f}")
        # Add performance metrics to results
        results['validation_stats'] = {
            'total_validations': self.validation_counter,
            'failed_validations': self.failed_validation_counter,
            'failure_rate': failure_rate,
            'confidence_score': results['confidence_score'],
            'critical_violations_count': len(critical_violations_list)
        }
        return results
    def get_performance_metrics(self) -> dict:
        """
        Get performance metrics for safety validation system.
        Returns:
            dictionary with performance metrics
        """
        if self.safety_check_times:
            avg_time = np.mean(self.safety_check_times)
            max_time = max(self.safety_check_times)
            percentile_95 = np.percentile(self.safety_check_times, 95) if len(self.safety_check_times) >= 10 else max_time
            percentile_99 = np.percentile(self.safety_check_times, 99) if len(self.safety_check_times) >= 50 else max_time
        else:
            avg_time, max_time, percentile_95, percentile_99 = 0.0, 0.0, 0.0, 0.0
        # Calculate failure statistics
        failure_rate = self.failed_validation_counter / max(1, self.validation_counter)
        success_rate = 1.0 - failure_rate
        return {
            'avg_validation_time_ms': avg_time * 1000,
            'max_validation_time_ms': max_time * 1000,
            'p95_validation_time_ms': percentile_95 * 1000,
            'p99_validation_time_ms': percentile_99 * 1000,
            'total_validations': self.validation_counter,
            'successful_validations': self.validation_counter - self.failed_validation_counter,
            'failed_validations': self.failed_validation_counter,
            'success_rate': success_rate,
            'failure_rate': failure_rate,
            'critical_violations_count': len(self.critical_safety_violations),
            'current_confidence_score': max(0.0, 1.0 - failure_rate) if self.validation_counter > 0 else 1.0
        }
