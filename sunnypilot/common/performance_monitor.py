"""
Performance Monitor for sunnypilot autonomous driving system
Implements real-time performance evaluation and self-tuning capabilities

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
from collections import deque
import time
from typing import Dict, Tuple, Optional
import logging


class RunningStat:
    """Simple running statistics calculator for performance metrics"""
    def __init__(self, window_size: int = 100):
        self.values = deque(maxlen=window_size)
        self.window_size = window_size
        
    def update(self, value: float) -> None:
        self.values.append(value)
        
    def mean(self) -> float:
        return float(np.mean(self.values)) if self.values else 0.0
        
    def std(self) -> float:
        return float(np.std(self.values)) if len(self.values) > 1 else 0.0
        
    def min(self) -> float:
        return float(np.min(self.values)) if self.values else 0.0
        
    def max(self) -> float:
        return float(np.max(self.values)) if self.values else 0.0


class PerformanceMonitor:
    """Real-time performance evaluation system for autonomous driving"""
    
    def __init__(self):
        # Performance metrics with running statistics
        self.performance_metrics = {
            'lateral_accuracy': RunningStat(window_size=100),
            'longitudinal_accuracy': RunningStat(window_size=100), 
            'model_confidence_trend': RunningStat(window_size=50),
            'control_effort': RunningStat(window_size=100),
            'ride_comfort': RunningStat(window_size=75),
            'path_following_error': RunningStat(window_size=100),
            'tracking_stability': RunningStat(window_size=50)
        }
        
        # Performance trend tracking
        self.performance_history = {
            'timestamp': deque(maxlen=500),
            'lateral_performance': deque(maxlen=500),
            'longitudinal_performance': deque(maxlen=500),
            'overall_performance': deque(maxlen=500)
        }
        
        # Adaptation parameters
        self.adaptation_threshold = 0.15  # Threshold for triggering parameter adaptation
        self.performance_baseline = {
            'lateral_accuracy': 0.15,    # Average lateral error threshold (meters)
            'longitudinal_accuracy': 0.3, # Average longitudinal error threshold (m/s)
            'ride_comfort': 0.8         # Comfort index threshold (higher is better)
        }
        
        # Self-tuning parameters tracking
        self.tuning_params = {
            'lateral_kp_factor': 1.0,    # Factor to adjust lateral proportional gain
            'lateral_ki_factor': 1.0,    # Factor to adjust lateral integral gain  
            'longitudinal_accel_limit_factor': 1.0,  # Factor for longitudinal limits
            'model_confidence_threshold': 0.7        # Base confidence threshold
        }
        
        # Performance evaluation state
        self.last_evaluation_time = time.time()
        self.performance_degraded_count = 0
        self.performance_improved_count = 0
        
    def evaluate_performance(self, desired_state: Dict, actual_state: Dict, 
                           model_output: Dict, control_output: Dict) -> Dict:
        """Evaluate system performance metrics in real-time"""
        
        # Calculate performance metrics
        lateral_error = abs(desired_state.get('lateral', 0) - actual_state.get('lateral', 0))
        longitudinal_error = abs(desired_state.get('longitudinal', 0) - actual_state.get('longitudinal', 0))
        
        # Calculate ride comfort (based on jerk and lateral acceleration)
        longitudinal_jerk = abs(control_output.get('jerk', 0))
        lateral_accel = abs(actual_state.get('lateral_accel', 0))
        comfort_metric = self.calculate_comfort_metric(longitudinal_jerk, lateral_accel)
        
        # Calculate path following error
        path_error = abs(desired_state.get('path_deviation', 0))
        
        # Calculate tracking stability (based on consistency of errors)
        stability_metric = self.calculate_tracking_stability(lateral_error, longitudinal_error)
        
        # Store metrics
        self.performance_metrics['lateral_accuracy'].update(lateral_error)
        self.performance_metrics['longitudinal_accuracy'].update(longitudinal_error) 
        self.performance_metrics['control_effort'].update(abs(control_output.get('output', 0)))
        self.performance_metrics['ride_comfort'].update(comfort_metric)
        self.performance_metrics['path_following_error'].update(path_error)
        self.performance_metrics['tracking_stability'].update(stability_metric)
        
        # Store model confidence if available
        if model_output and 'meta' in model_output and 'confidence' in model_output['meta']:
            model_confidence = model_output['meta']['confidence']
            self.performance_metrics['model_confidence_trend'].update(model_confidence)
        
        # Add to history for trend analysis
        current_time = time.time()
        self.performance_history['timestamp'].append(current_time)
        self.performance_history['lateral_performance'].append(lateral_error < self.performance_baseline['lateral_accuracy'])
        self.performance_history['longitudinal_performance'].append(longitudinal_error < self.performance_baseline['longitudinal_accuracy'])
        overall_performance = (lateral_error < self.performance_baseline['lateral_accuracy'] and 
                              longitudinal_error < self.performance_baseline['longitudinal_accuracy'] and 
                              comfort_metric > self.performance_baseline['ride_comfort'])
        self.performance_history['overall_performance'].append(overall_performance)
        
        return {
            'lateral_accuracy': lateral_error,
            'longitudinal_accuracy': longitudinal_error,
            'ride_comfort': comfort_metric,
            'path_following_error': path_error,
            'tracking_stability': stability_metric,
            'model_confidence': model_output.get('meta', {}).get('confidence', 1.0) if model_output else 1.0,
            'timestamp': current_time
        }
    
    def calculate_comfort_metric(self, longitudinal_jerk: float, lateral_accel: float) -> float:
        """Calculate ride comfort index (0-1 scale, higher is better)"""
        # Normalize jerk and lateral acceleration to 0-1 scale based on comfort thresholds
        # Higher jerk or lateral acceleration reduces comfort
        jerk_score = max(0, 1.0 - longitudinal_jerk / 5.0)  # 5.0 m/s³ as threshold
        lateral_accel_score = max(0, 1.0 - lateral_accel / 3.0)  # 3.0 m/s² as threshold
        
        # Weighted combination (jerk has more impact on comfort than lateral accel)
        comfort_metric = 0.6 * jerk_score + 0.4 * lateral_accel_score
        return max(0.0, min(1.0, comfort_metric))  # Clamp to [0, 1]
    
    def calculate_tracking_stability(self, lateral_error: float, longitudinal_error: float) -> float:
        """Calculate tracking stability index based on error consistency"""
        # Stability is higher when errors are consistent and low
        # Use running statistics to evaluate stability
        lat_std = self.performance_metrics['lateral_accuracy'].std()
        long_std = self.performance_metrics['longitudinal_accuracy'].std()
        
        # Lower standard deviation indicates more stable tracking
        lat_stability = max(0.0, 1.0 - lat_std / 0.3)  # Normalize by expected std
        long_stability = max(0.0, 1.0 - long_std / 0.5)
        
        # Combined stability metric
        stability = 0.5 * lat_stability + 0.5 * long_stability
        return max(0.0, min(1.0, stability))
    
    def should_adapt_parameters(self) -> Tuple[bool, Dict]:
        """Determine if parameters should be adapted based on performance trends"""
        current_time = time.time()
        
        # Check if enough time has passed for evaluation
        if current_time - self.last_evaluation_time < 5.0:  # 5 seconds minimum
            return False, {}
        
        self.last_evaluation_time = current_time
        
        # Evaluate current performance against baselines
        lat_acc_mean = self.performance_metrics['lateral_accuracy'].mean()
        long_acc_mean = self.performance_metrics['longitudinal_accuracy'].mean()
        comfort_mean = self.performance_metrics['ride_comfort'].mean()
        
        # Check if performance is degraded (worse than baseline by threshold)
        lat_degraded = lat_acc_mean > self.performance_baseline['lateral_accuracy'] + self.adaptation_threshold
        long_degraded = long_acc_mean > self.performance_baseline['longitudinal_accuracy'] + self.adaptation_threshold
        comfort_degraded = comfort_mean < self.performance_baseline['ride_comfort'] - self.adaptation_threshold
        
        if lat_degraded or long_degraded or comfort_degraded:
            self.performance_degraded_count += 1
            self.performance_improved_count = max(0, self.performance_improved_count - 1)  # Decay improvement count
            
            adaptation_needed = False
            adaptation_params = {}
            
            # Adjust lateral control parameters if tracking is poor
            if lat_degraded:
                adaptation_needed = True
                if lat_acc_mean > self.performance_baseline['lateral_accuracy'] * 1.5:
                    # Significantly degraded tracking - make larger adjustment
                    adaptation_params['lateral_kp_factor'] = max(0.5, self.tuning_params['lateral_kp_factor'] * 1.1)
                    adaptation_params['lateral_ki_factor'] = max(0.3, self.tuning_params['lateral_ki_factor'] * 1.05)
                else:
                    # Mild degradation - make smaller adjustment
                    adaptation_params['lateral_kp_factor'] = max(0.5, self.tuning_params['lateral_kp_factor'] * 1.05)
            
            # Adjust longitudinal parameters if comfort is poor (too much jerk)
            if comfort_degraded:
                adaptation_needed = True
                adaptation_params['longitudinal_accel_limit_factor'] = min(1.5, self.tuning_params['longitudinal_accel_limit_factor'] * 0.95)
            
            return adaptation_needed, adaptation_params
        else:
            # Performance is acceptable, might be improving
            self.performance_improved_count += 1
            self.performance_degraded_count = max(0, self.performance_degraded_count - 1)  # Decay degradation count
            
            # Gradually return parameters to baseline if consistently improved
            if self.performance_improved_count > 10 and self.performance_degraded_count < 3:
                adaptation_needed = True
                adaptation_params = {}
                
                # Gradually return to baseline values
                if self.tuning_params['lateral_kp_factor'] > 1.0:
                    adaptation_params['lateral_kp_factor'] = max(1.0, self.tuning_params['lateral_kp_factor'] * 0.99)
                if self.tuning_params['longitudinal_accel_limit_factor'] != 1.0:
                    adaptation_params['longitudinal_accel_limit_factor'] = min(1.0, self.tuning_params['longitudinal_accel_limit_factor'] * 1.01)
                
                return adaptation_needed, adaptation_params
        
        return False, {}
    
    def get_performance_summary(self) -> Dict:
        """Get summary of current performance metrics"""
        return {
            'lateral_accuracy': {
                'mean': self.performance_metrics['lateral_accuracy'].mean(),
                'std': self.performance_metrics['lateral_accuracy'].std(),
                'min': self.performance_metrics['lateral_accuracy'].min(),
                'max': self.performance_metrics['lateral_accuracy'].max()
            },
            'longitudinal_accuracy': {
                'mean': self.performance_metrics['longitudinal_accuracy'].mean(),
                'std': self.performance_metrics['longitudinal_accuracy'].std(),
                'min': self.performance_metrics['longitudinal_accuracy'].min(),
                'max': self.performance_metrics['longitudinal_accuracy'].max()
            },
            'ride_comfort': {
                'mean': self.performance_metrics['ride_comfort'].mean(),
                'std': self.performance_metrics['ride_comfort'].std()
            },
            'model_confidence_trend': {
                'mean': self.performance_metrics['model_confidence_trend'].mean(),
                'trend_direction': 'improving' if len(self.performance_metrics['model_confidence_trend'].values) > 1 and 
                                   self.performance_metrics['model_confidence_trend'].values[-1] > 
                                   self.performance_metrics['model_confidence_trend'].values[0] else 'declining' if 
                                   self.performance_metrics['model_confidence_trend'].values[-1] < 
                                   self.performance_metrics['model_confidence_trend'].values[0] else 'stable'
            },
            'tracking_stability': {
                'mean': self.performance_metrics['tracking_stability'].mean()
            },
            'performance_trend': {
                'degraded_count': self.performance_degraded_count,
                'improved_count': self.performance_improved_count
            }
        }