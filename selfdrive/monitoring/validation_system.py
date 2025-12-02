#!/usr/bin/env python3
"""
Monitoring and Validation System for sunnypilot2
This module implements comprehensive monitoring and validation to ensure safe operation
of the enhanced autonomous driving capabilities. Based on the 80/20 Pareto-optimal plan.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from collections import deque
import time
import threading
import json
from datetime import datetime

# Import existing sunnypilot components
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from cereal import car, log


class SystemHealthMonitor:
    """
    System health monitoring for computational resources, thermal state, and sensor health.
    """
    def __init__(self):
        self.computational_metrics = {
            'cpu_usage': deque(maxlen=100),  # 1 second history at 100Hz
            'memory_usage': deque(maxlen=100),
            'gpu_usage': deque(maxlen=100),
            'thermal_state': deque(maxlen=100),
            'process_cpu_time': deque(maxlen=100)
        }
        
        self.sensor_metrics = {
            'camera_frame_rate': deque(maxlen=50),
            'radar_update_rate': deque(maxlen=50),
            'imu_update_rate': deque(maxlen=50),
            'gps_fix_quality': deque(maxlen=50)
        }
        
        self.performance_thresholds = {
            'cpu_max_safe': 85.0,       # Percentage
            'memory_max_safe': 80.0,    # Percentage
            'thermal_max_safe': 80.0,   # Celsius
            'min_camera_fps': 15.0,     # Minimum expected camera FPS
            'max_control_latency': 0.1  # Maximum safe control delay (seconds)
        }
        
        self.health_status = {
            'system_overall': 'nominal',
            'components': {},
            'last_update': time.monotonic()
        }
        
    def update_computational_metrics(self, device_state):
        """
        Update computational resource metrics from device state.
        """
        current_time = time.monotonic()
        
        # CPU usage (from device state)
        if hasattr(device_state, 'cpuUsagePercent') and device_state.cpuUsagePercent:
            avg_cpu = sum(device_state.cpuUsagePercent) / len(device_state.cpuUsagePercent)
            self.computational_metrics['cpu_usage'].append(avg_cpu)
        
        # Memory usage
        if hasattr(device_state, 'memoryUsagePercent'):
            self.computational_metrics['memory_usage'].append(device_state.memoryUsagePercent)
        
        # Thermal state
        if hasattr(device_state, 'cpuTempC') and device_state.cpuTempC:
            max_temp = max(device_state.cpuTempC) if isinstance(device_state.cpuTempC, list) else device_state.cpuTempC
            self.computational_metrics['thermal_state'].append(max_temp)
        
        # Update health status based on thresholds
        self.health_status['last_update'] = current_time
        self._assess_system_health()
        
    def update_sensor_metrics(self, car_state, radar_state, model_data):
        """
        Update sensor health and performance metrics.
        """
        # Camera frame rate estimation (would need timestamp history in practice)
        if hasattr(model_data, 'frameDropPerc'):
            fps = 20.0 * (1.0 - model_data.frameDropPerc / 100.0)  # Estimate based on drops
            self.sensor_metrics['camera_frame_rate'].append(fps)
        
        # GPS fix quality
        if hasattr(car_state, 'gps') and car_state.gps.isValid:
            gps_qual = 3 if car_state.gps.source == 'sensor' else 2  # Higher is better
            self.sensor_metrics['gps_fix_quality'].append(gps_qual)
    
    def _assess_system_health(self):
        """
        Assess overall system health based on current metrics.
        """
        health_issues = []
        
        # Check computational resources
        if self.computational_metrics['cpu_usage']:
            current_cpu = self.computational_metrics['cpu_usage'][-1]
            if current_cpu > self.performance_thresholds['cpu_max_safe']:
                health_issues.append(f"High CPU usage: {current_cpu:.1f}%")
        
        if self.computational_metrics['memory_usage']:
            current_mem = self.computational_metrics['memory_usage'][-1]
            if current_mem > self.performance_thresholds['memory_max_safe']:
                health_issues.append(f"High memory usage: {current_mem:.1f}%")
        
        if self.computational_metrics['thermal_state']:
            current_temp = self.computational_metrics['thermal_state'][-1]
            if current_temp > self.performance_thresholds['thermal_max_safe']:
                health_issues.append(f"High thermal state: {current_temp:.1f}C")
        
        # Check sensor performance
        if self.sensor_metrics['camera_frame_rate']:
            current_fps = self.sensor_metrics['camera_frame_rate'][-1]
            if current_fps < self.performance_thresholds['min_camera_fps']:
                health_issues.append(f"Low camera frame rate: {current_fps:.1f} FPS")
        
        # Update health status
        if health_issues:
            if len(health_issues) > 2:
                self.health_status['system_overall'] = 'critical'
            elif len(health_issues) > 0:
                self.health_status['system_overall'] = 'degraded'
        else:
            self.health_status['system_overall'] = 'nominal'
        
        self.health_status['issues'] = health_issues
        
    def get_health_report(self) -> Dict:
        """
        Get comprehensive health report.
        """
        return {
            'timestamp': time.monotonic(),
            'health_status': self.health_status,
            'computational_metrics': {
                'cpu_avg': np.mean(self.computational_metrics['cpu_usage']) if self.computational_metrics['cpu_usage'] else 0,
                'memory_avg': np.mean(self.computational_metrics['memory_usage']) if self.computational_metrics['memory_usage'] else 0,
                'thermal_max': max(self.computational_metrics['thermal_state']) if self.computational_metrics['thermal_state'] else 0,
            },
            'sensor_metrics': {
                'camera_fps_avg': np.mean(self.sensor_metrics['camera_frame_rate']) if self.sensor_metrics['camera_frame_rate'] else 0,
                'gps_quality_avg': np.mean(self.sensor_metrics['gps_fix_quality']) if self.sensor_metrics['gps_fix_quality'] else 0,
            }
        }


class SafetyValidator:
    """
    Multi-level safety validation system for inputs, processing, and outputs.
    """
    def __init__(self):
        self.safety_history = deque(maxlen=200)  # 2 seconds at 100Hz
        self.validation_results = {
            'input_valid': True,
            'processing_valid': True,
            'output_valid': True,
            'system_safe': True
        }
        
        self.safety_thresholds = {
            'max_lat_accel': 3.0,      # Maximum lateral acceleration (m/s^2)
            'max_long_accel': 4.0,     # Maximum longitudinal acceleration (m/s^2)
            'min_safe_distance': 20.0, # Minimum safe distance to lead vehicle (m)
            'max_curvature': 0.5,      # Maximum safe curvature (1/m)
            'max_steering_rate': 100.0 # Maximum steering rate (deg/s)
        }
        
    def validate_inputs(self, car_state, radar_state, model_data) -> bool:
        """
        Validate sensor inputs and model data for plausibility.
        """
        input_issues = []
        
        # Validate car state
        if hasattr(car_state, 'vEgo') and car_state.vEgo < -1.0:
            input_issues.append("Negative ego velocity")
        
        if hasattr(car_state, 'aEgo') and abs(car_state.aEgo) > 10.0:
            input_issues.append("Excessive ego acceleration")
        
        if hasattr(car_state, 'steeringAngleDeg') and abs(car_state.steeringAngleDeg) > 70.0:
            input_issues.append("Excessive steering angle")
        
        # Validate radar state
        if (hasattr(radar_state, 'leadOne') and radar_state.leadOne.status and 
            radar_state.leadOne.dRel < -5.0):
            input_issues.append("Negative lead distance")
        
        # Validate model data
        if hasattr(model_data, 'plan'):
            plan = model_data.plan
            if len(plan.velocity.x) > 0 and any(v < -1.0 for v in plan.velocity.x):
                input_issues.append("Negative velocities in plan")
        
        # Record validation result
        is_valid = len(input_issues) == 0
        self.validation_results['input_valid'] = is_valid
        
        if not is_valid:
            cloudlog.warning(f"Input validation failed: {input_issues}")
        
        return is_valid
    
    def validate_processing(self, perception_output, planning_output) -> bool:
        """
        Validate processing results for consistency and plausibility.
        """
        processing_issues = []
        
        # Check for consistency between perception and planning
        if ('fused_detections' in perception_output and 
            'best_plan' in planning_output and 
            planning_output['best_plan']):
            # Verify plan is consistent with perception
            plan = planning_output['best_plan']
            if hasattr(plan, 'velocity') and any(v > 50.0 for v in plan.velocity or []):
                processing_issues.append("Excessive velocities in plan")
        
        is_valid = len(processing_issues) == 0
        self.validation_results['processing_valid'] = is_valid
        
        if not is_valid:
            cloudlog.warning(f"Processing validation failed: {processing_issues}")
        
        return is_valid
    
    def validate_outputs(self, actuators, car_state, radar_state) -> bool:
        """
        Validate control outputs for safety.
        """
        output_issues = []
        
        # Validate acceleration limits
        if hasattr(actuators, 'accel') and abs(actuators.accel) > self.safety_thresholds['max_long_accel']:
            output_issues.append(f"Excessive acceleration: {actuators.accel:.2f}")
        
        # Validate curvature limits
        if hasattr(actuators, 'curvature') and abs(actuators.curvature) > self.safety_thresholds['max_curvature']:
            output_issues.append(f"Excessive curvature: {actuators.curvature:.4f}")
        
        # Validate steering angle limits if applicable
        if hasattr(actuators, 'steeringAngleDeg') and abs(actuators.steeringAngleDeg) > 70.0:
            output_issues.append(f"Excessive steering angle: {actuators.steeringAngleDeg:.1f}")
        
        # Check for lateral acceleration safety
        if (hasattr(actuators, 'curvature') and hasattr(car_state, 'vEgo') and
            abs(actuators.curvature * car_state.vEgo**2) > self.safety_thresholds['max_lat_accel']):
            output_issues.append(f"Excessive lateral acceleration: {actuators.curvature * car_state.vEgo**2:.2f}")
        
        is_valid = len(output_issues) == 0
        self.validation_results['output_valid'] = is_valid
        
        if not is_valid:
            cloudlog.warning(f"Output validation failed: {output_issues}")
        
        return is_valid
    
    def validate_system_safety(self, car_state, radar_state, actuators) -> bool:
        """
        Validate overall system safety considering all components.
        """
        safety_issues = []
        
        # Forward collision risk
        if (hasattr(radar_state, 'leadOne') and radar_state.leadOne.status and
            car_state.vEgo > 0.1):
            ttc = radar_state.leadOne.dRel / (car_state.vEgo - radar_state.leadOne.vRel + 1e-6)
            if ttc < 2.0 and radar_state.leadOne.dRel < 50.0:  # Less than 2s TTC and close
                # Check if our control is appropriate
                if hasattr(actuators, 'accel') and actuators.accel > 0:  # Accelerating toward lead
                    safety_issues.append(f"Accelerating with TTC: {ttc:.1f}s, distance: {radar_state.leadOne.dRel:.1f}m")
        
        # Validate system state consistency
        if car_state.standstill and abs(actuators.accel) > 0.5:
            safety_issues.append("Non-zero acceleration requested while standstill")
        
        is_safe = len(safety_issues) == 0
        self.validation_results['system_safe'] = is_safe
        
        if not is_safe:
            cloudlog.warning(f"System safety validation failed: {safety_issues}")
        
        return is_safe
    
    def validate_all(self, car_state, radar_state, model_data, 
                    perception_output, planning_output, actuators) -> Dict:
        """
        Perform comprehensive safety validation.
        
        Returns:
            Dictionary with validation results
        """
        # Perform all validations
        input_valid = self.validate_inputs(car_state, radar_state, model_data)
        processing_valid = self.validate_processing(perception_output, planning_output)
        output_valid = self.validate_outputs(actuators, car_state, radar_state)
        system_safe = self.validate_system_safety(car_state, radar_state, actuators)
        
        # Overall safety assessment
        all_valid = all([input_valid, processing_valid, output_valid, system_safe])
        
        validation_record = {
            'timestamp': time.monotonic(),
            'input_valid': input_valid,
            'processing_valid': processing_valid,
            'output_valid': output_valid,
            'system_safe': system_safe,
            'all_valid': all_valid
        }
        
        self.safety_history.append(validation_record)
        
        return validation_record


class PerformanceMonitor:
    """
    Performance monitoring system for tracking control accuracy, comfort, and efficiency.
    """
    def __init__(self):
        self.metrics_history = deque(maxlen=1000)  # Keep 10 seconds of metrics at 100Hz
        self.performance_metrics = {
            'lateral_error': deque(maxlen=100),
            'longitudinal_error': deque(maxlen=100),
            'jerk_longitudinal': deque(maxlen=100),
            'jerk_lateral': deque(maxlen=100),
            'tracking_accuracy': deque(maxlen=100),
            'comfort_score': deque(maxlen=100),
            'efficiency_score': deque(maxlen=100)
        }
        
        self.performance_targets = {
            'max_avg_lateral_error': 0.2,      # meters
            'max_avg_long_error': 0.5,        # m/s velocity tracking error
            'max_avg_long_jerk': 1.5,         # m/s^3
            'max_avg_lat_jerk': 2.0,          # m/s^3
            'min_tracking_accuracy': 0.8      # 0-1 scale
        }

        # Initialize acceleration tracking
        self.prev_accel = 0.0
        
    def update_metrics(self, car_state, actuators, model_data, plan_data) -> Dict:
        """
        Update performance metrics based on current state and control.
        """
        current_time = time.monotonic()
        
        # Calculate lateral error (difference between actual and planned lateral position)
        lateral_error = 0.0  # Would need planned vs actual lateral position
        self.performance_metrics['lateral_error'].append(abs(lateral_error))
        
        # Calculate longitudinal error (velocity tracking error)
        if hasattr(model_data, 'velocity') and len(model_data.velocity.x) > 0:
            planned_vel = model_data.velocity.x[0] if model_data.velocity.x else 0.0
            velocity_error = abs(car_state.vEgo - planned_vel)
            self.performance_metrics['longitudinal_error'].append(velocity_error)
        else:
            self.performance_metrics['longitudinal_error'].append(0.0)
        
        # Calculate longitudinal jerk (rate of acceleration change)
        current_accel = actuators.accel if hasattr(actuators, 'accel') else 0.0
        prev_accel = self._get_prev_accel()
        jerk = abs(current_accel - prev_accel) / 0.01  # Assuming 100Hz
        self.performance_metrics['jerk_longitudinal'].append(jerk)

        # Store current acceleration for next iteration
        self.prev_accel = current_accel
        
        # Calculate lateral jerk
        # Would need rate of curvature change
        self.performance_metrics['jerk_lateral'].append(0.0)
        
        # Calculate tracking accuracy (proportion of time within acceptable error bounds)
        lat_acc = len([e for e in self.performance_metrics['lateral_error'] if e < 0.5]) / len(self.performance_metrics['lateral_error'])
        self.performance_metrics['tracking_accuracy'].append(lat_acc)
        
        # Calculate comfort score (inversely related to jerk and acceleration)
        avg_long_jerk = np.mean(self.performance_metrics['jerk_longitudinal']) if self.performance_metrics['jerk_longitudinal'] else 0.0
        comfort_score = max(0.0, min(1.0, 1.0 - avg_long_jerk / 3.0))  # Normalize to 0-1
        self.performance_metrics['comfort_score'].append(comfort_score)
        
        # Calculate efficiency (fuel/speed efficiency - simplified)
        efficiency_score = max(0.0, min(1.0, 1.0 - abs(car_state.aEgo) / 5.0))  # Higher efficiency when smooth driving
        self.performance_metrics['efficiency_score'].append(efficiency_score)
        
        # Create performance record
        perf_record = {
            'timestamp': current_time,
            'lateral_error_avg': np.mean(self.performance_metrics['lateral_error']),
            'longitudinal_error_avg': np.mean(self.performance_metrics['longitudinal_error']),
            'jerk_longitudinal_avg': np.mean(self.performance_metrics['jerk_longitudinal']),
            'jerk_lateral_avg': np.mean(self.performance_metrics['jerk_lateral']),
            'tracking_accuracy_avg': np.mean(self.performance_metrics['tracking_accuracy']),
            'comfort_score_avg': np.mean(self.performance_metrics['comfort_score']),
            'efficiency_score_avg': np.mean(self.performance_metrics['efficiency_score'])
        }
        
        self.metrics_history.append(perf_record)
        
        return perf_record
    
    def _get_prev_accel(self):
        """Helper to get previous acceleration from history."""
        # Return the last acceleration value if available, otherwise 0
        if hasattr(self, 'prev_accel') and self.prev_accel is not None:
            return self.prev_accel
        return 0.0
    
    def get_performance_report(self) -> Dict:
        """
        Get current performance report.
        """
        if not self.metrics_history:
            return {}
        
        latest = self.metrics_history[-1]
        
        # Assess performance against targets
        performance_status = {
            'lateral_control': 'good' if latest['lateral_error_avg'] < self.performance_targets['max_avg_lateral_error'] else 'poor',
            'longitudinal_control': 'good' if latest['longitudinal_error_avg'] < self.performance_targets['max_avg_long_error'] else 'poor',
            'comfort': 'good' if latest['jerk_longitudinal_avg'] < self.performance_targets['max_avg_long_jerk'] else 'poor',
            'tracking': 'good' if latest['tracking_accuracy_avg'] > self.performance_targets['min_tracking_accuracy'] else 'poor'
        }
        
        return {
            'current_performance': latest,
            'performance_status': performance_status,
            'performance_trend': self._calculate_trend(),
            'recommendations': self._generate_recommendations()
        }
    
    def _calculate_trend(self) -> Dict:
        """Calculate performance trends over time."""
        if len(self.metrics_history) < 10:
            return {}
        
        recent = list(self.metrics_history)[-10:]
        older = list(self.metrics_history)[-20:-10] if len(self.metrics_history) >= 20 else []
        
        trends = {}
        for metric in ['lateral_error_avg', 'longitudinal_error_avg', 
                      'jerk_longitudinal_avg', 'tracking_accuracy_avg']:
            if older:
                recent_avg = np.mean([r[metric] for r in recent])
                older_avg = np.mean([o[metric] for o in older])
                trend = 'improving' if recent_avg < older_avg else 'declining'
                trends[metric] = trend
        
        return trends
    
    def _generate_recommendations(self) -> List[str]:
        """Generate performance improvement recommendations."""
        recommendations = []
        
        latest = self.metrics_history[-1] if self.metrics_history else {}
        
        if latest.get('lateral_error_avg', 0) > self.performance_targets['max_avg_lateral_error']:
            recommendations.append("Improve lateral tracking accuracy")
        
        if latest.get('jerk_longitudinal_avg', 0) > self.performance_targets['max_avg_long_jerk']:
            recommendations.append("Reduce longitudinal jerk for better comfort")
        
        if latest.get('tracking_accuracy_avg', 1) < self.performance_targets['min_tracking_accuracy']:
            recommendations.append("Improve overall tracking performance")
        
        return recommendations


class ScenarioValidator:
    """
    Scenario-based validation and edge case monitoring.
    """
    def __init__(self):
        self.edge_case_history = deque(maxlen=500)  # Track edge cases
        self.scenario_performance = {}
        self.edge_case_metrics = {
            'construction_zone': 0.0,
            'intersection': 0.0,
            'highway_merge': 0.0,
            'weather_adverse': 0.0,
            'traffic_dense': 0.0
        }
        
    def validate_scenario_handling(self, scenario_type: str, 
                                 inputs, outputs, success_indicators) -> Dict:
        """
        Validate performance in specific scenarios.
        
        Args:
            scenario_type: Type of scenario being validated
            inputs: Input data for the scenario
            outputs: System outputs for the scenario
            success_indicators: Indicators of successful handling
            
        Returns:
            Validation results for the scenario
        """
        validation_result = {
            'scenario_type': scenario_type,
            'timestamp': time.monotonic(),
            'success': True,
            'issues': [],
            'performance_score': 1.0
        }
        
        # Validate based on scenario type
        if scenario_type == 'construction_zone':
            # Check for appropriate speed reduction and increased following distance
            if success_indicators.get('speed_reduced', True) is False:
                validation_result['issues'].append("Did not properly reduce speed in construction zone")
                validation_result['success'] = False
            if success_indicators.get('following_distance', 0) < 50:  # meters
                validation_result['issues'].append("Following distance too short in construction zone")
                validation_result['success'] = False
        
        elif scenario_type == 'intersection':
            # Check for appropriate stopping behavior at traffic lights/stop signs
            if success_indicators.get('stopped_at_light', False) is False:
                validation_result['issues'].append("Did not stop properly at traffic light")
                validation_result['success'] = False
        
        # Calculate performance score (simpler for this example)
        if validation_result['issues']:
            validation_result['performance_score'] = max(0.1, 1.0 - len(validation_result['issues']) * 0.2)
        
        # Store for history
        self.edge_case_history.append(validation_result)
        
        # Update scenario performance tracking
        if scenario_type not in self.scenario_performance:
            self.scenario_performance[scenario_type] = {'attempts': 0, 'successes': 0}
        
        self.scenario_performance[scenario_type]['attempts'] += 1
        if validation_result['success']:
            self.scenario_performance[scenario_type]['successes'] += 1
        
        return validation_result
    
    def get_scenario_report(self) -> Dict:
        """
        Get report on scenario handling performance.
        """
        scenario_report = {}
        
        for scenario, stats in self.scenario_performance.items():
            success_rate = stats['successes'] / max(1, stats['attempts'])
            scenario_report[scenario] = {
                'success_rate': success_rate,
                'attempts': stats['attempts'],
                'successes': stats['successes'],
                'status': 'excellent' if success_rate > 0.9 else 'good' if success_rate > 0.7 else 'needs_improvement'
            }
        
        return {
            'scenario_performance': scenario_report,
            'recent_edge_cases': list(self.edge_case_history)[-10:],
            'total_edge_cases_handled': len(self.edge_case_history)
        }


class MonitoringSystem:
    """
    Main monitoring and validation system integrating all components.
    """
    def __init__(self):
        self.system_health_monitor = SystemHealthMonitor()
        self.safety_validator = SafetyValidator()
        self.performance_monitor = PerformanceMonitor()
        self.scenario_validator = ScenarioValidator()
        
        self.monitoring_enabled = True
        self.validation_enabled = True
        self.logging_level = 'info'  # 'debug', 'info', 'warning', 'error'
        
        # Data collection for analysis
        self.event_log = deque(maxlen=10000)
        
    def update_system_monitoring(self, sm):
        """
        Update monitoring with current system data.
        
        Args:
            sm: SubMaster with current system data
        """
        if not self.monitoring_enabled:
            return
        
        # Update system health
        if 'deviceState' in sm:
            self.system_health_monitor.update_computational_metrics(sm['deviceState'])
        
        # Update sensor metrics
        if all(key in sm for key in ['carState', 'radarState', 'modelV2']):
            self.system_health_monitor.update_sensor_metrics(
                sm['carState'], sm['radarState'], sm['modelV2']
            )
    
    def validate_current_operation(self, sm, perception_output=None, 
                                 planning_output=None) -> Dict:
        """
        Perform comprehensive validation of current operation.
        
        Args:
            sm: SubMaster with current system data
            perception_output: Current perception system output
            planning_output: Current planning system output
            
        Returns:
            Comprehensive validation results
        """
        if not self.validation_enabled:
            return {'validation_passed': True, 'issues': []}
        
        # Get current data
        car_state = sm['carState'] if 'carState' in sm else None
        radar_state = sm['radarState'] if 'radarState' in sm else None
        model_data = sm['modelV2'] if 'modelV2' in sm else None
        actuators = sm['controlsState'].actuators if 'controlsState' in sm else None
        
        if not all([car_state, radar_state, model_data, actuators]):
            cloudlog.warning("Missing required data for validation")
            return {'validation_passed': False, 'issues': ['Missing required data']}
        
        # Perform safety validation
        validation_result = self.safety_validator.validate_all(
            car_state, radar_state, model_data, 
            perception_output or {}, planning_output or {}, actuators
        )
        
        # Update performance metrics if controls state is available
        if 'longitudinalPlan' in sm:
            self.performance_monitor.update_metrics(
                car_state, actuators, model_data, sm['longitudinalPlan']
            )
        
        # Log validation results
        validation_log = {
            'timestamp': time.monotonic(),
            'validation_result': validation_result,
            'health_status': self.system_health_monitor.get_health_report()
        }
        
        self.event_log.append(validation_log)
        
        return validation_result
    
    def validate_scenario(self, scenario_type: str, inputs, outputs, success_indicators) -> Dict:
        """
        Validate specific scenario handling.
        """
        return self.scenario_validator.validate_scenario_handling(
            scenario_type, inputs, outputs, success_indicators
        )
    
    def get_comprehensive_report(self) -> Dict:
        """
        Get comprehensive monitoring and validation report.
        """
        return {
            'system_health': self.system_health_monitor.get_health_report(),
            'performance_metrics': self.performance_monitor.get_performance_report(),
            'scenario_report': self.scenario_validator.get_scenario_report(),
            'recent_validations': list(self.event_log)[-10:],  # Last 10 validation logs
            'system_uptime': getattr(self, '_start_time', time.monotonic()) - time.monotonic()
        }
    
    def log_event(self, event_type: str, data: Dict, level: str = 'info'):
        """
        Log an event for monitoring and analysis.
        
        Args:
            event_type: Type of event being logged
            data: Event data
            level: Logging level ('debug', 'info', 'warning', 'error')
        """
        event = {
            'timestamp': time.monotonic(),
            'event_type': event_type,
            'data': data,
            'level': level
        }
        
        self.event_log.append(event)
        
        # Log to swaglog based on level
        if level == 'debug':
            cloudlog.debug(f"Monitoring Event - {event_type}: {data}")
        elif level == 'info':
            cloudlog.info(f"Monitoring Event - {event_type}: {data}")
        elif level == 'warning':
            cloudlog.warning(f"Monitoring Event - {event_type}: {data}")
        elif level == 'error':
            cloudlog.error(f"Monitoring Event - {event_type}: {data}")


# Example usage and testing
if __name__ == "__main__":
    from openpilot.common.params import Params
    cloudlog.info("Initializing Monitoring and Validation System")
    
    # Initialize monitoring system
    monitoring_system = MonitoringSystem()
    
    # Log initialization
    monitoring_system.log_event("system_init", {"component": "MonitoringSystem"}, "info")
    
    cloudlog.info("Monitoring and Validation System initialized successfully")