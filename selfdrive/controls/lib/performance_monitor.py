#!/usr/bin/env python3
"""
Performance monitoring module for tracking fallback triggers and safety system performance
"""

import time
from typing import Dict, Any
from collections import defaultdict, deque
from openpilot.common.swaglog import cloudlog


class PerformanceMonitor:
    """
    Monitors performance of safety and environmental systems,
    tracking when fallbacks are triggered and system behavior
    """
    
    def __init__(self):
        # Track fallback triggers
        self.fallback_counts = defaultdict(int)
        self.fallback_times = defaultdict(deque)
        
        # Track system performance metrics
        self.system_metrics = defaultdict(int)
        # Initialize default values for common system names
        self.system_metrics.update({
            'environmental_processor_healthy': 0,
            'environmental_processor_failed': 0,
            'adaptive_behavior_healthy': 0,
            'adaptive_behavior_failed': 0,
            'curvature_limiter_healthy': 0,
            'curvature_limiter_failed': 0
        })
        
        # Track system degradation over time
        self.degradation_events = []
        
    def record_fallback_trigger(self, fallback_type: str, details: Dict[str, Any] = None):
        """
        Record when a fallback is triggered
        :param fallback_type: Type of fallback (e.g., 'environmental_processor', 'adaptive_behavior')
        :param details: Additional details about the fallback
        """
        self.fallback_counts[fallback_type] += 1
        self.fallback_times[fallback_type].append(time.time())
        
        # Keep only recent fallbacks (last 100)
        if len(self.fallback_times[fallback_type]) > 100:
            self.fallback_times[fallback_type].popleft()
        
        # Log the event
        cloudlog.event(f"PerformanceMonitor: Fallback triggered", 
                      fallback_type=fallback_type, 
                      count=self.fallback_counts[fallback_type],
                      details=details)
        
    def record_system_state(self, system_name: str, is_healthy: bool):
        """
        Record the health state of a system
        :param system_name: Name of the system
        :param is_healthy: Whether the system is operating normally
        """
        if is_healthy:
            self.system_metrics[f'{system_name}_healthy'] += 1
        else:
            self.system_metrics[f'{system_name}_failed'] += 1
            
    def record_degradation_event(self, event_type: str, severity: str, details: Dict[str, Any]):
        """
        Record a degradation event
        :param event_type: Type of degradation event
        :param severity: Severity level ('low', 'medium', 'high')
        :param details: Additional details about the event
        """
        event = {
            'timestamp': time.time(),
            'event_type': event_type,
            'severity': severity,
            'details': details
        }
        self.degradation_events.append(event)
        
        # Keep only recent events (last 50)
        if len(self.degradation_events) > 50:
            self.degradation_events.pop(0)
            
        # Log significant degradation events
        if severity in ['high', 'medium']:
            cloudlog.event(f"PerformanceMonitor: {severity.capitalize()} degradation", 
                          event_type=event_type,
                          details=details)
    
    def get_fallback_frequency(self, fallback_type: str, time_window: float = 300.0) -> float:
        """
        Get the frequency of a specific fallback type in a time window
        :param fallback_type: Type of fallback to check
        :param time_window: Time window in seconds (default 5 minutes)
        :return: Fallback frequency per second
        """
        current_time = time.time()
        recent_times = [t for t in self.fallback_times[fallback_type] 
                       if current_time - t <= time_window]
        return len(recent_times) / time_window if time_window > 0 else 0.0
    
    def get_performance_report(self) -> Dict[str, Any]:
        """
        Get comprehensive performance report
        :return: Dictionary with performance metrics
        """
        current_time = time.time()
        
        # Calculate fallback frequencies for different time windows
        fallback_frequencies = {}
        for fallback_type in self.fallback_counts:
            fallback_frequencies[fallback_type] = {
                'last_60s': self.get_fallback_frequency(fallback_type, 60.0),
                'last_300s': self.get_fallback_frequency(fallback_type, 300.0),
                'last_3600s': self.get_fallback_frequency(fallback_type, 3600.0)
            }
        
        return {
            'fallback_counts': dict(self.fallback_counts),
            'fallback_frequencies': fallback_frequencies,
            'system_health': dict(self.system_metrics),
            'total_degradation_events': len(self.degradation_events),
            'recent_degradation_severity': {
                'high': len([e for e in self.degradation_events if e['severity'] == 'high']),
                'medium': len([e for e in self.degradation_events if e['severity'] == 'medium']),
                'low': len([e for e in self.degradation_events if e['severity'] == 'low'])
            },
            'report_timestamp': current_time
        }
    
    def log_performance_summary(self):
        """
        Log a summary of performance metrics periodically
        """
        report = self.get_performance_report()
        
        cloudlog.event("PerformanceMonitor: Summary Report",
                      fallback_counts=report['fallback_counts'],
                      system_health=report['system_health'],
                      recent_high_severity_degradations=report['recent_degradation_severity']['high'])


# Global instance for easy access
performance_monitor = PerformanceMonitor()