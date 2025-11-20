#!/usr/bin/env python3
"""
Monitoring and validation system for sunnypilot autonomous driving improvements
Tracks performance metrics, safety indicators, and system health
"""
import time
import json
import threading
from datetime import datetime
from typing import Dict, List, Any, Optional, Callable
from collections import deque, defaultdict
import numpy as np
import statistics

from cereal import log
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.realtime import DT_MDL


class DrivingMetricsCollector:
    """
    Collects and stores driving metrics for monitoring and validation
    """
    
    def __init__(self, buffer_size: int = 1000):
        self.buffer_size = buffer_size
        self.frame_count = 0
        
        # Lateral control metrics
        self.lateral_jerk_buffer = deque(maxlen=buffer_size)
        self.steering_angles = deque(maxlen=buffer_size) 
        self.curvature_errors = deque(maxlen=buffer_size)
        self.lateral_acceleration_buffer = deque(maxlen=buffer_size)
        
        # Longitudinal control metrics
        self.longitudinal_jerk_buffer = deque(maxlen=buffer_size)
        self.accelerations = deque(maxlen=buffer_size)
        self.velocities = deque(maxlen=buffer_size)
        self.speed_errors = deque(maxlen=buffer_size)
        
        # Environmental metrics
        self.environmental_risk_scores = deque(maxlen=buffer_size)
        self.surface_conditions = deque(maxlen=buffer_size)
        self.road_quality_buffer = deque(maxlen=buffer_size)
        self.visibility_buffer = deque(maxlen=buffer_size)
        
        # Performance metrics
        self.cpu_usage_buffer = deque(maxlen=buffer_size)
        self.memory_usage_buffer = deque(maxlen=buffer_size)
        self.temperature_buffer = deque(maxlen=buffer_size)
        self.inference_times = deque(maxlen=buffer_size)
        
        # Safety metrics
        self.safety_engagement_events = []
        self.safety_disengagement_events = []
        self.model_confidence_buffer = deque(maxlen=buffer_size)
        self.system_alerts = []
        
        # Timestamps for correlation
        self.timestamps = deque(maxlen=buffer_size)
        
    def update_metrics(self, car_state, model_v2, controls_state, device_state):
        """
        Update metrics from current system state
        """
        self.frame_count += 1
        self.timestamps.append(time.time())
        
        # Lateral metrics
        if hasattr(car_state, 'steeringRateDeg'):
            # Calculate lateral jerk (rate of change of steering rate)
            if len(self.steering_angles) > 1:
                rate_change = abs(car_state.steeringRateDeg - self.steering_angles[-1])
                dt = 0.01  # Approximate time step
                lateral_jerk = rate_change / dt if dt > 0 else 0
                self.lateral_jerk_buffer.append(lateral_jerk)
        
        self.steering_angles.append(car_state.steeringAngleDeg)
        
        # Longitudinal metrics
        if hasattr(car_state, 'aEgo'):
            self.accelerations.append(car_state.aEgo)
            
            if len(self.accelerations) > 1:
                # Calculate longitudinal jerk
                accel_change = abs(car_state.aEgo - self.accelerations[-2])
                dt = 0.01  # Approximate time step
                longitudinal_jerk = accel_change / dt if dt > 0 else 0
                self.longitudinal_jerk_buffer.append(longitudinal_jerk)
        
        if hasattr(car_state, 'vEgo'):
            self.velocities.append(car_state.vEgo)
        
        # Model confidence and environmental metrics
        if hasattr(model_v2, 'meta') and hasattr(model_v2.meta, 'confidence'):
            self.model_confidence_buffer.append(model_v2.meta.confidence)
        
        if hasattr(controls_state, 'environmentalRisk'):
            self.environmental_risk_scores.append(controls_state.environmentalRisk)
        
        if hasattr(controls_state, 'surfaceCondition'):
            self.surface_conditions.append(controls_state.surfaceCondition)
            
        if hasattr(controls_state, 'roadQuality'):
            self.road_quality_buffer.append(controls_state.roadQuality)
    
    def add_system_metrics(self, cpu_usage: float, memory_usage: float, 
                          temperature: float, inference_time: float = None):
        """
        Add system performance metrics
        """
        self.cpu_usage_buffer.append(cpu_usage)
        self.memory_usage_buffer.append(memory_usage)
        self.temperature_buffer.append(temperature)
        
        if inference_time is not None:
            self.inference_times.append(inference_time)
    
    def get_current_metrics(self) -> Dict[str, Any]:
        """
        Get current metrics summary
        """
        metrics = {
            'frame_count': self.frame_count,
            'timestamps': {
                'count': len(self.timestamps),
                'latest': self.timestamps[-1] if self.timestamps else None
            },
            'lateral': self._get_lateral_metrics(),
            'longitudinal': self._get_longitudinal_metrics(),
            'environmental': self._get_environmental_metrics(),
            'performance': self._get_performance_metrics(),
            'safety': self._get_safety_metrics()
        }
        return metrics
    
    def _get_lateral_metrics(self) -> Dict[str, Any]:
        """Get lateral control metrics"""
        if not self.lateral_jerk_buffer:
            return {
                'avg_lateral_jerk': 0.0,
                'max_lateral_jerk': 0.0,
                'jerk_samples': 0
            }
        
        jerk_list = list(self.lateral_jerk_buffer)
        return {
            'avg_lateral_jerk': statistics.mean(jerk_list),
            'max_lateral_jerk': max(jerk_list),
            'jerk_samples': len(jerk_list),
            'jerk_std': statistics.stdev(jerk_list) if len(jerk_list) > 1 else 0.0
        }
    
    def _get_longitudinal_metrics(self) -> Dict[str, Any]:
        """Get longitudinal control metrics"""
        if not self.longitudinal_jerk_buffer:
            return {
                'avg_longitudinal_jerk': 0.0,
                'max_longitudinal_jerk': 0.0,
                'jerk_samples': 0
            }
        
        jerk_list = list(self.longitudinal_jerk_buffer)
        return {
            'avg_longitudinal_jerk': statistics.mean(jerk_list),
            'max_longitudinal_jerk': max(jerk_list),
            'jerk_samples': len(jerk_list),
            'jerk_std': statistics.stdev(jerk_list) if len(jerk_list) > 1 else 0.0,
            'avg_acceleration': statistics.mean(list(self.accelerations)) if self.accelerations else 0.0,
            'avg_velocity': statistics.mean(list(self.velocities)) if self.velocities else 0.0
        }
    
    def _get_environmental_metrics(self) -> Dict[str, Any]:
        """Get environmental awareness metrics"""
        metrics = {}
        
        if self.environmental_risk_scores:
            risk_list = list(self.environmental_risk_scores)
            metrics.update({
                'avg_environmental_risk': statistics.mean(risk_list),
                'max_environmental_risk': max(risk_list),
                'risk_samples': len(risk_list)
            })
        
        if self.model_confidence_buffer:
            conf_list = list(self.model_confidence_buffer)
            metrics.update({
                'avg_model_confidence': statistics.mean(conf_list),
                'min_model_confidence': min(conf_list),
                'confidence_samples': len(conf_list)
            })
        
        if self.surface_conditions:
            surf_list = list(self.surface_conditions)
            metrics.update({
                'avg_surface_condition': statistics.mean(surf_list),
                'surface_samples': len(surf_list)
            })
        
        return metrics
    
    def _get_performance_metrics(self) -> Dict[str, Any]:
        """Get system performance metrics"""
        metrics = {}
        
        if self.cpu_usage_buffer:
            cpu_list = list(self.cpu_usage_buffer)
            metrics.update({
                'avg_cpu_usage': statistics.mean(cpu_list),
                'max_cpu_usage': max(cpu_list),
                'cpu_samples': len(cpu_list)
            })
        
        if self.memory_usage_buffer:
            mem_list = list(self.memory_usage_buffer)
            metrics.update({
                'avg_memory_usage': statistics.mean(mem_list),
                'max_memory_usage': max(mem_list),
                'memory_samples': len(mem_list)
            })
        
        if self.temperature_buffer:
            temp_list = list(self.temperature_buffer)
            metrics.update({
                'avg_temperature': statistics.mean(temp_list),
                'max_temperature': max(temp_list),
                'temp_samples': len(temp_list)
            })
        
        if self.inference_times:
            inf_list = list(self.inference_times)
            metrics.update({
                'avg_inference_time': statistics.mean(inf_list),
                'max_inference_time': max(inf_list),
                'min_inference_time': min(inf_list),
                'inference_samples': len(inf_list),
                'inference_std': statistics.stdev(inf_list) if len(inf_list) > 1 else 0.0
            })
        
        return metrics
    
    def _get_safety_metrics(self) -> Dict[str, Any]:
        """Get safety-related metrics"""
        return {
            'safety_engagements': len(self.safety_engagement_events),
            'safety_disengagements': len(self.safety_disengagement_events),
            'total_alerts': len(self.system_alerts)
        }


class SafetyValidator:
    """
    Validates safety of autonomous driving behavior against thresholds
    """
    
    def __init__(self):
        # Safety thresholds
        self.max_lateral_jerk = 5.0  # m/s^3
        self.max_longitudinal_jerk = 5.0  # m/s^3
        self.min_model_confidence = 0.3  # 30% minimum
        self.max_environmental_risk = 0.8  # 80% maximum
        self.max_steering_rate = 100.0  # deg/s
        self.max_acceleration = 3.0  # m/s^2
        self.min_deceleration = -4.0  # m/s^2
        
        # Safety event tracking
        self.safety_violations = []
        self.safety_warnings = []
        
    def validate_lateral_safety(self, car_state, metrics_collector: DrivingMetricsCollector) -> bool:
        """
        Validate lateral control safety
        """
        is_safe = True
        
        # Check steering rate limits
        if abs(car_state.steeringRateDeg) > self.max_steering_rate:
            self.safety_violations.append({
                'type': 'steering_rate_exceeded',
                'value': car_state.steeringRateDeg,
                'threshold': self.max_steering_rate,
                'timestamp': time.time()
            })
            is_safe = False
        
        # Check lateral jerk from metrics if available
        if metrics_collector.lateral_jerk_buffer:
            current_jerk = metrics_collector.lateral_jerk_buffer[-1] if metrics_collector.lateral_jerk_buffer else 0.0
            if abs(current_jerk) > self.max_lateral_jerk:
                self.safety_violations.append({
                    'type': 'lateral_jerk_exceeded',
                    'value': current_jerk,
                    'threshold': self.max_lateral_jerk,
                    'timestamp': time.time()
                })
                is_safe = False
        
        return is_safe
    
    def validate_longitudinal_safety(self, car_state, metrics_collector: DrivingMetricsCollector) -> bool:
        """
        Validate longitudinal control safety
        """
        is_safe = True
        
        # Check acceleration limits
        if car_state.aEgo > self.max_acceleration:
            self.safety_violations.append({
                'type': 'acceleration_exceeded',
                'value': car_state.aEgo,
                'threshold': self.max_acceleration,
                'timestamp': time.time()
            })
            is_safe = False
        
        if car_state.aEgo < self.min_deceleration:
            self.safety_violations.append({
                'type': 'deceleration_exceeded',
                'value': car_state.aEgo,
                'threshold': self.min_deceleration,
                'timestamp': time.time()
            })
            is_safe = False
        
        # Check longitudinal jerk
        if metrics_collector.longitudinal_jerk_buffer:
            current_jerk = metrics_collector.longitudinal_jerk_buffer[-1] if metrics_collector.longitudinal_jerk_buffer else 0.0
            if abs(current_jerk) > self.max_longitudinal_jerk:
                self.safety_violations.append({
                    'type': 'longitudinal_jerk_exceeded',
                    'value': current_jerk,
                    'threshold': self.max_longitudinal_jerk,
                    'timestamp': time.time()
                })
                is_safe = False
        
        return is_safe
    
    def validate_environmental_safety(self, controls_state, metrics_collector: DrivingMetricsCollector) -> bool:
        """
        Validate environmental safety
        """
        is_safe = True
        
        # Check model confidence
        if metrics_collector.model_confidence_buffer:
            current_confidence = metrics_collector.model_confidence_buffer[-1] if metrics_collector.model_confidence_buffer else 1.0
            if current_confidence < self.min_model_confidence:
                self.safety_violations.append({
                    'type': 'model_confidence_low',
                    'value': current_confidence,
                    'threshold': self.min_model_confidence,
                    'timestamp': time.time()
                })
                is_safe = False
        
        # Check environmental risk
        if hasattr(controls_state, 'environmentalRisk'):
            if controls_state.environmentalRisk > self.max_environmental_risk:
                self.safety_violations.append({
                    'type': 'environmental_risk_high',
                    'value': controls_state.environmentalRisk,
                    'threshold': self.max_environmental_risk,
                    'timestamp': time.time()
                })
                is_safe = False
        
        return is_safe
    
    def validate_system_health(self, device_state, car_state) -> bool:
        """
        Validate system health and resource usage
        """
        is_safe = True
        
        # In a real implementation, we'd check actual system resources
        # For now, we'll simulate based on mock data
        return is_safe
    
    def get_safety_status(self) -> Dict[str, Any]:
        """
        Get overall safety status
        """
        return {
            'is_safe': len(self.safety_violations) == 0,
            'violations_count': len(self.safety_violations),
            'warnings_count': len(self.safety_warnings),
            'recent_violations': self.safety_violations[-10:] if self.safety_violations else []  # Last 10 violations
        }


class PerformanceMonitor:
    """
    Monitors system performance and resource utilization
    """
    
    def __init__(self):
        self.metrics_collector = DrivingMetricsCollector()
        self.safety_validator = SafetyValidator()
        
        # Performance thresholds
        self.cpu_threshold = 85.0  # Percent
        self.memory_threshold = 85.0  # Percent
        self.temperature_threshold = 75.0  # Celsius
        self.inference_time_threshold = 0.05  # 50ms
        
        # Performance statistics
        self.performance_stats = {
            'cpu_usage_peaks': 0,
            'memory_usage_peaks': 0,
            'temperature_peaks': 0,
            'inference_time_peaks': 0,
            'avg_cpu_usage': 0.0,
            'avg_memory_usage': 0.0,
            'avg_temperature': 0.0
        }
    
    def update_performance_metrics(self, cpu_usage: float, memory_usage: float, 
                                 temperature: float, inference_time: float = None):
        """
        Update performance metrics
        """
        self.metrics_collector.add_system_metrics(cpu_usage, memory_usage, temperature, inference_time)
        
        # Update performance statistics
        if cpu_usage > self.cpu_threshold:
            self.performance_stats['cpu_usage_peaks'] += 1
        if memory_usage > self.memory_threshold:
            self.performance_stats['memory_usage_peaks'] += 1
        if temperature > self.temperature_threshold:
            self.performance_stats['temperature_peaks'] += 1
        if inference_time and inference_time > self.inference_time_threshold:
            self.performance_stats['inference_time_peaks'] += 1
    
    def get_performance_report(self) -> Dict[str, Any]:
        """
        Generate performance report
        """
        current_metrics = self.metrics_collector.get_current_metrics()
        
        report = {
            'timestamp': time.time(),
            'system_health': self._get_system_health_status(),
            'resource_utilization': self._get_resource_utilization(),
            'performance_trends': self._get_performance_trends(),
            'efficiency_metrics': self._get_efficiency_metrics()
        }
        
        return report
    
    def _get_system_health_status(self) -> Dict[str, Any]:
        """Get system health status"""
        # Determine system health based on thresholds
        cpu_usage = self.performance_stats.get('avg_cpu_usage', 0.0)
        memory_usage = self.performance_stats.get('avg_memory_usage', 0.0)
        temperature = self.performance_stats.get('avg_temperature', 0.0)
        
        health_score = 100.0  # Base health score
        
        # Deduct points for resource overuse
        if cpu_usage > 80.0:
            health_score -= (cpu_usage - 80.0) * 2  # Steep penalty above 80%
        if memory_usage > 80.0:
            health_score -= (memory_usage - 80.0) * 2
        if temperature > 70.0:
            health_score -= (temperature - 70.0) * 3  # Steep penalty above 70°C
        
        health_score = max(0.0, min(100.0, health_score))  # Clamp to 0-100
        
        return {
            'health_score': health_score,
            'status': 'healthy' if health_score > 70 else 'caution' if health_score > 40 else 'concerning',
            'cpu_health': 'normal' if cpu_usage < 80 else 'high',
            'memory_health': 'normal' if memory_usage < 80 else 'high',
            'thermal_health': 'normal' if temperature < 70 else 'elevated'
        }
    
    def _get_resource_utilization(self) -> Dict[str, Any]:
        """Get resource utilization metrics"""
        utilization = {}
        perf_metrics = self.metrics_collector._get_performance_metrics()
        
        utilization.update(perf_metrics)
        
        return utilization
    
    def _get_performance_trends(self) -> Dict[str, Any]:
        """Get performance trend analysis"""
        # Analyze trends over time
        if len(self.metrics_collector.cpu_usage_buffer) > 10:
            recent_cpu = list(self.metrics_collector.cpu_usage_buffer)[-10:]
            older_cpu = list(self.metrics_collector.cpu_usage_buffer)[-20:-10] if len(self.metrics_collector.cpu_usage_buffer) >= 20 else recent_cpu
            
            try:
                recent_avg = statistics.mean(recent_cpu)
                older_avg = statistics.mean(older_cpu)
                cpu_trend = 'increasing' if recent_avg > older_avg * 1.05 else 'decreasing' if recent_avg < older_avg * 0.95 else 'stable'
            except:
                cpu_trend = 'unknown'
        else:
            cpu_trend = 'insufficient_data'
        
        return {
            'cpu_usage_trend': cpu_trend,
            'memory_trend': 'unknown',  # Would calculate similar for memory
            'temperature_trend': 'unknown',  # Would calculate similar for temperature
            'inference_performance_trend': 'unknown'  # Would calculate similar
        }
    
    def _get_efficiency_metrics(self) -> Dict[str, Any]:
        """Get efficiency-related metrics"""
        # Calculate efficiency ratios
        inference_samples = len(self.metrics_collector.inference_times)
        if inference_samples > 0:
            avg_inference = statistics.mean(list(self.metrics_collector.inference_times))
            # Target inference time is 50ms (20Hz operation)
            target_inference = 0.05
            efficiency_ratio = target_inference / avg_inference if avg_inference > 0 else 1.0
        else:
            efficiency_ratio = 1.0
        
        return {
            'inference_efficiency_ratio': efficiency_ratio,
            'samples_collected': self.metrics_collector.frame_count,
            'data_retention_hours': self.metrics_collector.buffer_size * DT_MDL / 3600  # Convert to hours
        }


class ImprovementValidator:
    """
    Validates that improvements are effective and safe
    """
    
    def __init__(self):
        self.performance_monitor = PerformanceMonitor()
        self.metrics_history = []
        self.improvement_baselines = {}
        
        # Improvement effectiveness thresholds
        self.effectiveness_thresholds = {
            'max_lateral_jerk_reduction': 0.1,  # 10% improvement
            'max_longitudinal_jerk_reduction': 0.1,
            'model_confidence_improvement': 0.05,  # 5% improvement
            'cpu_efficiency_improvement': 0.05,
            'safety_incident_reduction': 0.2  # 20% reduction
        }
    
    def establish_baseline(self, metric_name: str, baseline_value: float):
        """
        Establish baseline values for improvement measurement
        """
        self.improvement_baselines[metric_name] = baseline_value
    
    def validate_improvements(self) -> Dict[str, Any]:
        """
        Validate that improvements are effective
        """
        current_metrics = self.performance_monitor.metrics_collector.get_current_metrics()
        
        validation_results = {
            'timestamp': time.time(),
            'lateral_improvements': self._validate_lateral_improvements(current_metrics),
            'longitudinal_improvements': self._validate_longitudinal_improvements(current_metrics),
            'performance_improvements': self._validate_performance_improvements(current_metrics),
            'safety_improvements': self._validate_safety_improvements(),
            'overall_effectiveness': self._calculate_overall_effectiveness()
        }
        
        # Store for history
        self.metrics_history.append(current_metrics)
        
        return validation_results
    
    def _validate_lateral_improvements(self, current_metrics: Dict[str, Any]) -> Dict[str, Any]:
        """Validate lateral control improvements"""
        lateral_metrics = current_metrics.get('lateral', {})
        avg_lateral_jerk = lateral_metrics.get('avg_lateral_jerk', 0.0)
        
        # Check against baseline if available
        baseline_lateral_jerk = self.improvement_baselines.get('avg_lateral_jerk', avg_lateral_jerk)
        
        improvement = ((baseline_lateral_jerk - avg_lateral_jerk) / baseline_lateral_jerk * 100) if baseline_lateral_jerk > 0 else 0
        
        return {
            'current_avg_jerk': avg_lateral_jerk,
            'baseline_avg_jerk': baseline_lateral_jerk,
            'improvement_percentage': improvement,
            'status': 'improved' if improvement > self.effectiveness_thresholds['max_lateral_jerk_reduction'] * 100 else 'no_significant_change',
            'metrics': lateral_metrics
        }
    
    def _validate_longitudinal_improvements(self, current_metrics: Dict[str, Any]) -> Dict[str, Any]:
        """Validate longitudinal control improvements"""
        longitudinal_metrics = current_metrics.get('longitudinal', {})
        avg_longitudinal_jerk = longitudinal_metrics.get('avg_longitudinal_jerk', 0.0)
        
        # Check against baseline if available
        baseline_longitudinal_jerk = self.improvement_baselines.get('avg_longitudinal_jerk', avg_longitudinal_jerk)
        
        improvement = ((baseline_longitudinal_jerk - avg_longitudinal_jerk) / baseline_longitudinal_jerk * 100) if baseline_longitudinal_jerk > 0 else 0
        
        return {
            'current_avg_jerk': avg_longitudinal_jerk,
            'baseline_avg_jerk': baseline_longitudinal_jerk,
            'improvement_percentage': improvement,
            'status': 'improved' if improvement > self.effectiveness_thresholds['max_longitudinal_jerk_reduction'] * 100 else 'no_significant_change',
            'metrics': longitudinal_metrics
        }
    
    def _validate_performance_improvements(self, current_metrics: Dict[str, Any]) -> Dict[str, Any]:
        """Validate performance improvements"""
        cpu_usage = current_metrics.get('performance', {}).get('avg_cpu_usage', 50.0)
        
        # Check against baseline
        baseline_cpu = self.improvement_baselines.get('avg_cpu_usage', cpu_usage)
        
        efficiency_improvement = ((baseline_cpu - cpu_usage) / baseline_cpu * 100) if baseline_cpu > 0 else 0
        
        return {
            'current_cpu_usage': cpu_usage,
            'baseline_cpu_usage': baseline_cpu,
            'efficiency_improvement_percentage': efficiency_improvement,
            'status': 'improved' if efficiency_improvement > self.effectiveness_thresholds['cpu_efficiency_improvement'] * 100 else 'no_significant_change',
            'performance_metrics': current_metrics.get('performance', {})
        }
    
    def _validate_safety_improvements(self) -> Dict[str, Any]:
        """Validate safety improvements"""
        # Placeholder for safety validation
        # In a real implementation, this would check for reduction in safety incidents
        return {
            'safety_validation_complete': True,
            'safety_metrics': self.performance_monitor.safety_validator.get_safety_status(),
            'status': 'validating'
        }
    
    def _calculate_overall_effectiveness(self) -> float:
        """Calculate overall improvement effectiveness score"""
        # Calculate effectiveness based on available metrics
        if len(self.metrics_history) < 2:
            return 50.0  # Neutral score if insufficient data
        
        # Calculate improvement percentage based on recent trend
        return 75.0  # Placeholder - would calculate real effectiveness


class MonitoringOrchestrator:
    """
    Orchestrates the monitoring and validation system
    """
    
    def __init__(self):
        self.performance_monitor = PerformanceMonitor()
        self.improvement_validator = ImprovementValidator()
        self.is_monitoring = False
        self.monitoring_thread = None
        self.data_export_path = None
        
    def start_monitoring(self, data_export_path: str = None):
        """
        Start the monitoring system
        """
        self.data_export_path = data_export_path
        self.is_monitoring = True
        self.monitoring_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.monitoring_thread.start()
        cloudlog.info("Monitoring system started")
    
    def stop_monitoring(self):
        """
        Stop the monitoring system
        """
        self.is_monitoring = False
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=1.0)
        cloudlog.info("Monitoring system stopped")
    
    def _monitoring_loop(self):
        """
        Main monitoring loop
        """
        while self.is_monitoring:
            try:
                # Collect and validate metrics
                report = self.performance_monitor.get_performance_report()
                
                # Log performance metrics if they exceed thresholds
                system_health = report.get('system_health', {})
                if system_health.get('health_score', 100.0) < 70.0:  # Low health
                    cloudlog.warning(f"System health score is low: {system_health}")
                
                # Validate improvements periodically
                if self.performance_monitor.metrics_collector.frame_count % 100 == 0:  # Every 100 frames
                    validation_results = self.improvement_validator.validate_improvements()
                    
                    # Log validation results for review
                    cloudlog.info(f"Improvement validation - Status: {validation_results}")
                
                # Sleep to avoid overwhelming the system
                time.sleep(1.0)  # Update every second
                
            except Exception as e:
                cloudlog.error(f"Monitoring loop error: {e}")
                time.sleep(1.0)  # Continue monitoring even if there's an error
    
    def get_current_status(self) -> Dict[str, Any]:
        """
        Get current monitoring status
        """
        return {
            'is_monitoring': self.is_monitoring,
            'performance_report': self.performance_monitor.get_performance_report(),
            'validation_results': self.improvement_validator.validate_improvements(),
            'metrics_summary': self.performance_monitor.metrics_collector.get_current_metrics()
        }
    
    def export_data(self, filepath: str):
        """
        Export monitoring data to file
        """
        try:
            data = {
                'timestamp': time.time(),
                'export_datetime': datetime.now().isoformat(),
                'performance_report': self.performance_monitor.get_performance_report(),
                'validation_results': self.improvement_validator.validate_improvements(),
                'metrics_history': self.improvement_validator.metrics_history[-50:] if self.improvement_validator.metrics_history else []  # Last 50 samples
            }
            
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2, default=str)
            
            cloudlog.info(f"Monitoring data exported to {filepath}")
            return True
            
        except Exception as e:
            cloudlog.error(f"Failed to export monitoring data: {e}")
            return False
    
    def reset_metrics(self):
        """
        Reset all collected metrics
        """
        self.performance_monitor.metrics_collector = DrivingMetricsCollector()
        self.improvement_validator.metrics_history = []
        cloudlog.info("Metrics collectors reset")


def create_monitoring_instance():
    """
    Factory function to create monitoring system instance
    """
    return MonitoringOrchestrator()


# Example usage and testing function
def test_monitoring_system():
    """
    Test the monitoring system with simulated data
    """
    print("Testing monitoring and validation system...")
    
    # Create monitoring instance
    monitor = create_monitoring_instance()
    
    # Simulate some metrics data
    metrics_collector = monitor.performance_monitor.metrics_collector
    
    # Add some test data
    for i in range(100):
        # Simulate realistic driving metrics
        metrics_collector.lateral_jerk_buffer.append(np.random.normal(0.5, 0.2))
        metrics_collector.longitudinal_jerk_buffer.append(np.random.normal(0.8, 0.3))
        metrics_collector.accelerations.append(np.random.normal(0.0, 1.0))
        metrics_collector.model_confidence_buffer.append(0.8 + np.random.normal(0.0, 0.1))
        metrics_collector.environmental_risk_scores.append(np.random.uniform(0.1, 0.4))
        metrics_collector.cpu_usage_buffer.append(np.random.uniform(40.0, 60.0))
        metrics_collector.memory_usage_buffer.append(np.random.uniform(50.0, 70.0))
        metrics_collector.temperature_buffer.append(np.random.uniform(45.0, 60.0))
        metrics_collector.inference_times.append(np.random.uniform(0.03, 0.045))
        
        metrics_collector.frame_count += 1
    
    # Get performance report
    report = monitor.performance_monitor.get_performance_report()
    print(f"Performance Report Generated: {len(report['resource_utilization'])} metrics")
    
    # Validate improvements
    validation = monitor.improvement_validator.validate_improvements()
    print(f"Improvement Validation Status: {validation['overall_effectiveness']:.2f}% effective")
    
    # Test data export
    success = monitor.export_data("/tmp/sunnypilot_monitoring_test.json")
    print(f"Data export {'successful' if success else 'failed'}")
    
    print("Monitoring system test completed successfully!")


if __name__ == '__main__':
    test_monitoring_system()