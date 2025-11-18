"""
Metrics tracking module for autonomous driving performance monitoring.
This module provides tools to measure and track key metrics required for autonomous driving.
"""
import time
import threading
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from collections import defaultdict, deque
import json
from pathlib import Path

@dataclass
class MetricPoint:
    """Represents a single metric measurement with timestamp and context."""
    name: str
    value: float
    timestamp: float
    context: Dict[str, Any] = field(default_factory=dict)

@dataclass
class MetricSummary:
    """Summary statistics for a metric over time."""
    name: str
    count: int
    min: float
    max: float
    avg: float
    latest: float
    p95: float = 0.0
    p99: float = 0.0

class MetricsRecorder:
    """Thread-safe metrics recorder for autonomous driving performance tracking."""
    
    def __init__(self, max_history: int = 10000):
        self.max_history = max_history
        self.metrics: Dict[str, deque] = defaultdict(lambda: deque(maxlen=max_history))
        self.lock = threading.Lock()
        
        # Performance tracking
        self.start_time = time.time()
        
    def record(self, name: str, value: float, context: Optional[Dict[str, Any]] = None) -> None:
        """Record a metric value with optional context."""
        with self.lock:
            metric = MetricPoint(
                name=name,
                value=value,
                timestamp=time.time(),
                context=context or {}
            )
            self.metrics[name].append(metric)
    
    def get_latest(self, name: str) -> Optional[MetricPoint]:
        """Get the most recent value for a metric."""
        with self.lock:
            if name in self.metrics and len(self.metrics[name]) > 0:
                return self.metrics[name][-1]
            return None
    
    def get_summary(self, name: str) -> Optional[MetricSummary]:
        """Get summary statistics for a metric."""
        with self.lock:
            if name not in self.metrics or len(self.metrics[name]) == 0:
                return None
            
            values = [m.value for m in self.metrics[name]]
            if len(values) == 0:
                return None
                
            sorted_values = sorted(values)
            n = len(sorted_values)
            
            return MetricSummary(
                name=name,
                count=n,
                min=min(values),
                max=max(values),
                avg=sum(values) / n,
                latest=values[-1],
                p95=sorted_values[int(0.95 * n)] if n > 0 else values[0] if values else 0.0,
                p99=sorted_values[int(0.99 * n)] if n > 0 else values[0] if values else 0.0
            )
    
    def get_all_summaries(self) -> Dict[str, MetricSummary]:
        """Get summary statistics for all tracked metrics."""
        with self.lock:
            return {name: self.get_summary(name) for name in self.metrics.keys()}
    
    def export_to_file(self, filepath: str) -> None:
        """Export metrics to JSON file."""
        with self.lock:
            export_data = {}
            for name, metric_points in self.metrics.items():
                export_data[name] = [
                    {
                        'name': mp.name,
                        'value': mp.value,
                        'timestamp': mp.timestamp,
                        'context': mp.context
                    }
                    for mp in metric_points
                ]
        
        with open(filepath, 'w') as f:
            json.dump(export_data, f, indent=2)
    
    def get_system_uptime(self) -> float:
        """Get system uptime in seconds."""
        return time.time() - self.start_time

# Global metrics recorder instance
_metrics_recorder = MetricsRecorder()

def record_metric(name: str, value: float, context: Optional[Dict[str, Any]] = None) -> None:
    """Record a metric value with optional context."""
    _metrics_recorder.record(name, value, context)

def get_metric_summary(name: str) -> Optional[MetricSummary]:
    """Get summary statistics for a metric."""
    return _metrics_recorder.get_summary(name)

def get_all_metric_summaries() -> Dict[str, MetricSummary]:
    """Get summary statistics for all tracked metrics."""
    return _metrics_recorder.get_all_summaries()

def export_metrics(filepath: str) -> None:
    """Export all metrics to JSON file."""
    _metrics_recorder.export_to_file(filepath)

# Pre-defined metric names for sunnypilot
class Metrics:
    # Perception metrics
    PERCEPTION_LATENCY_MS = "perception.latency.ms"
    OBJECT_DETECTION_ACCURACY = "perception.object_detection.accuracy"
    FRAME_PROCESSING_RATE = "perception.frame_rate.fps"
    
    # Control system metrics
    STEERING_LATENCY_MS = "control.steering_latency.ms"
    BRAKING_LATENCY_MS = "control.braking_latency.ms"
    STEERING_ACCURACY = "control.steering_accuracy"
    
    # Navigation metrics
    NAVIGATION_ACCURACY = "navigation.accuracy.m"
    ROUTE_COMPLETION_RATE = "navigation.route_completion_rate"
    MANEUVER_SUCCESS_RATE = "navigation.maneuver_success_rate"
    NAVIGATION_LATENCY_MS = "navigation.latency.ms"
    
    # Hardware optimization metrics
    CPU_USAGE_PERCENT = "hardware.cpu_usage.percent"
    RAM_USAGE_MB = "hardware.ram_usage.mb"
    RAM_USAGE_PERCENT = "hardware.ram_usage.percent"
    POWER_DRAW_WATTS = "hardware.power_draw.watts"
    
    # Route planning metrics
    PLANNING_LATENCY_MS = "planning.latency.ms"
    TRAJECTORY_SMOOTHNESS = "planning.trajectory_smoothness"
    
    # Safety metrics
    SAFETY_MARGIN_VIOLATIONS = "safety.margin_violations.count"
    STOP_SIGN_DETECTION_RATE = "safety.stop_sign_detection_rate"
    TRAFFIC_LIGHT_DETECTION_RATE = "safety.traffic_light_detection_rate"