"""
Real-time Monitoring Dashboard for sunnypilot
Provides real-time visualization and alerting for critical metrics
"""
import time
import threading
import json
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass
from collections import deque
import numpy as np

import cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE
from openpilot.common.performance_monitor import perf_monitor
from openpilot.common.data_collector import data_collection_manager


@dataclass
class MetricThreshold:
  """Threshold configuration for metrics"""
  name: str
  warning_threshold: float
  critical_threshold: float
  unit: str = ""
  lower_is_better: bool = False


@dataclass
class AlertEvent:
  """Alert event data structure"""
  timestamp: float
  metric_name: str
  value: float
  threshold: float
  level: str  # 'warning' or 'critical'
  description: str


class RealtimeDashboard:
  """Real-time dashboard for monitoring system metrics"""
  
  def __init__(self, update_interval: float = 0.1):  # 100ms update interval
    self.update_interval = update_interval
    self.running = True
    
    # Metrics storage
    self.metrics_history: Dict[str, deque] = {}
    self.max_history_points = 1000  # Keep last 100 points for each metric
    
    # Alerting system
    self.alerts: deque = deque(maxlen=100)
    self.thresholds: List[MetricThreshold] = []
    self.alert_callbacks: List[Callable[[AlertEvent], None]] = []
    
    # SubMaster for data
    self.sm = messaging.SubMaster([
      'deviceState', 'carState', 'modelV2', 'selfdriveState',
      'radarState', 'controlsState', 'validationMetrics'
    ])
    
    # Params for settings
    self.params = Params()
    
    # Initialize thresholds
    self._setup_thresholds()
    
    # Start monitoring thread
    self.monitoring_thread = threading.Thread(target=self._update_loop, daemon=True)
    self.monitoring_thread.start()
    
    cloudlog.info("Real-time dashboard initialized")
  
  def _setup_thresholds(self):
    """Setup metric thresholds for alerting"""
    self.thresholds = [
      # CPU monitoring
      MetricThreshold("cpu_usage_avg", 70.0, 90.0, "%", False),
      MetricThreshold("cpu_usage_max", 85.0, 95.0, "%", False),
      
      # Memory monitoring
      MetricThreshold("memory_usage", 75.0, 90.0, "%", False),
      
      # GPU monitoring
      MetricThreshold("gpu_usage", 80.0, 95.0, "%", False),
      
      # Temperature monitoring
      MetricThreshold("cpu_temp_max", 75.0, 85.0, "°C", False),
      MetricThreshold("gpu_temp_max", 70.0, 80.0, "°C", False),
      
      # Model performance
      MetricThreshold("model_latency", 50.0, 80.0, "ms", True),
      MetricThreshold("model_confidence", 0.6, 0.4, "", True),
      
      # System performance
      MetricThreshold("controlsd_loop_time", 8.0, 12.0, "ms", True),
      MetricThreshold("end_to_end_latency", 60.0, 80.0, "ms", True),
    ]
  
  def add_alert_callback(self, callback: Callable[[AlertEvent], None]):
    """Add callback for alert events"""
    self.alert_callbacks.append(callback)
  
  def _update_loop(self):
    """Main update loop for collecting metrics"""
    while self.running:
      try:
        # Update data
        self.sm.update(0)  # Non-blocking update
        
        # Collect metrics
        self._collect_device_metrics()
        self._collect_model_metrics()
        self._collect_control_metrics()
        self._collect_validation_metrics()
        
        # Check thresholds and generate alerts
        self._check_thresholds()
        
        time.sleep(self.update_interval)
        
      except Exception as e:
        cloudlog.error(f"Dashboard update error: {e}")
        time.sleep(1.0)
  
  def _collect_device_metrics(self):
    """Collect device-related metrics"""
    if not self.sm.valid['deviceState']:
      return
    
    device_state = self.sm['deviceState']
    
    # CPU metrics
    for i, cpu_usage in enumerate(device_state.cpuUsagePercent):
      if f"cpu{i}_usage" not in self.metrics_history:
        self.metrics_history[f"cpu{i}_usage"] = deque(maxlen=self.max_history_points)
      self.metrics_history[f"cpu{i}_usage"].append(cpu_usage)
    
    # Average CPU usage
    avg_cpu = np.mean(device_state.cpuUsagePercent) if device_state.cpuUsagePercent else 0.0
    if "cpu_usage_avg" not in self.metrics_history:
      self.metrics_history["cpu_usage_avg"] = deque(maxlen=self.max_history_points)
    self.metrics_history["cpu_usage_avg"].append(avg_cpu)
    
    # Max CPU usage
    max_cpu = max(device_state.cpuUsagePercent) if device_state.cpuUsagePercent else 0.0
    if "cpu_usage_max" not in self.metrics_history:
      self.metrics_history["cpu_usage_max"] = deque(maxlen=self.max_history_points)
    self.metrics_history["cpu_usage_max"].append(max_cpu)
    
    # Memory metrics
    if "memory_usage" not in self.metrics_history:
      self.metrics_history["memory_usage"] = deque(maxlen=self.max_history_points)
    self.metrics_history["memory_usage"].append(device_state.memoryUsagePercent)
    
    # GPU metrics
    if "gpu_usage" not in self.metrics_history:
      self.metrics_history["gpu_usage"] = deque(maxlen=self.max_history_points)
    self.metrics_history["gpu_usage"].append(device_state.gpuUsagePercent)
    
    # Temperature metrics
    max_cpu_temp = max(device_state.cpuTempC) if device_state.cpuTempC else 0.0
    if "cpu_temp_max" not in self.metrics_history:
      self.metrics_history["cpu_temp_max"] = deque(maxlen=self.max_history_points)
    self.metrics_history["cpu_temp_max"].append(max_cpu_temp)
    
    max_gpu_temp = max(device_state.gpuTempC) if device_state.gpuTempC else 0.0
    if "gpu_temp_max" not in self.metrics_history:
      self.metrics_history["gpu_temp_max"] = deque(maxlen=self.max_history_points)
    self.metrics_history["gpu_temp_max"].append(max_gpu_temp)
  
  def _collect_model_metrics(self):
    """Collect model-related metrics"""
    if not self.sm.valid['modelV2']:
      return
    
    model_v2 = self.sm['modelV2']
    
    # Model execution time
    if "model_latency" not in self.metrics_history:
      self.metrics_history["model_latency"] = deque(maxlen=self.max_history_points)
    self.metrics_history["model_latency"].append(model_v2.modelExecutionTime)
  
  def _collect_control_metrics(self):
    """Collect control system metrics"""
    if not self.sm.valid['controlsState']:
      return
    
    controls_state = self.sm['controlsState']
    
    # Calculate end-to-end latency
    model_time = controls_state.lateralPlanMonoTime / 1e9 if controls_state.lateralPlanMonoTime else 0
    control_time = time.time()  # Current time as control output time
    e2e_latency = (control_time * 1e9 - model_time) / 1e6 if model_time > 0 else 0  # Convert to ms
    
    if "end_to_end_latency" not in self.metrics_history:
      self.metrics_history["end_to_end_latency"] = deque(maxlen=self.max_history_points)
    self.metrics_history["end_to_end_latency"].append(e2e_latency)
    
    # Controls loop time from performance monitor
    loop_time_avg = perf_monitor.get_average_time("controlsd_loop")
    if loop_time_avg and "controlsd_loop_time" not in self.metrics_history:
      self.metrics_history["controlsd_loop_time"] = deque(maxlen=self.max_history_points)
      self.metrics_history["controlsd_loop_time"].append(loop_time_avg)
  
  def _collect_validation_metrics(self):
    """Collect validation metrics"""
    if not self.sm.valid['validationMetrics']:
      return
    
    validation_metrics = self.sm['validationMetrics']
    
    # Overall confidence
    if "model_confidence" not in self.metrics_history:
      self.metrics_history["model_confidence"] = deque(maxlen=self.max_history_points)
    self.metrics_history["model_confidence"].append(validation_metrics.overallConfidence)
  
  def _check_thresholds(self):
    """Check if any metrics exceed thresholds and trigger alerts"""
    current_time = time.time()
    
    for threshold in self.thresholds:
      if threshold.name not in self.metrics_history:
        continue
      
      if not self.metrics_history[threshold.name]:
        continue
      
      # Get the latest value
      latest_value = self.metrics_history[threshold.name][-1]
      
      # Check if threshold is exceeded
      exceeded = False
      alert_level = ""
      exceeded_threshold = 0.0
      
      if threshold.lower_is_better:
        # Lower values are better - alert if value is too low
        if latest_value < threshold.critical_threshold:
          exceeded = True
          alert_level = "critical"
          exceeded_threshold = threshold.critical_threshold
        elif latest_value < threshold.warning_threshold:
          exceeded = True
          alert_level = "warning"
          exceeded_threshold = threshold.warning_threshold
      else:
        # Higher values are worse - alert if value is too high
        if latest_value > threshold.critical_threshold:
          exceeded = True
          alert_level = "critical"
          exceeded_threshold = threshold.critical_threshold
        elif latest_value > threshold.warning_threshold:
          exceeded = True
          alert_level = "warning"
          exceeded_threshold = threshold.warning_threshold
      
      if exceeded:
        alert = AlertEvent(
          timestamp=current_time,
          metric_name=threshold.name,
          value=latest_value,
          threshold=exceeded_threshold,
          level=alert_level,
          description=f"{threshold.name} exceeded {alert_level} threshold: {latest_value:.2f} vs {exceeded_threshold:.2f} {threshold.unit}"
        )
        
        # Add to alerts
        self.alerts.append(alert)
        
        # Call all registered callbacks
        for callback in self.alert_callbacks:
          try:
            callback(alert)
          except Exception as e:
            cloudlog.error(f"Alert callback error: {e}")
  
  def get_current_metrics(self) -> Dict[str, float]:
    """Get current values for all tracked metrics"""
    current_metrics = {}
    for name, history in self.metrics_history.items():
      if history:
        current_metrics[name] = history[-1]
    
    return current_metrics
  
  def get_metric_history(self, metric_name: str, num_points: int = 50) -> List[float]:
    """Get historical values for a specific metric"""
    if metric_name not in self.metrics_history:
      return []
    
    history = list(self.metrics_history[metric_name])
    return history[-num_points:] if len(history) >= num_points else history
  
  def get_recent_alerts(self, num_alerts: int = 10) -> List[AlertEvent]:
    """Get recent alert events"""
    return list(self.alerts)[-num_alerts:]
  
  def get_system_health_status(self) -> Dict[str, str]:
    """Get overall system health status"""
    current_metrics = self.get_current_metrics()
    health_status = {"status": "normal", "issues": []}
    
    # Check CPU health
    cpu_avg = current_metrics.get("cpu_usage_avg", 0)
    if cpu_avg > 90:
      health_status["status"] = "critical"
      health_status["issues"].append(f"High CPU usage: {cpu_avg:.1f}%")
    elif cpu_avg > 75:
      if health_status["status"] != "critical":
        health_status["status"] = "warning"
      health_status["issues"].append(f"Moderate CPU usage: {cpu_avg:.1f}%")
    
    # Check memory health
    memory_usage = current_metrics.get("memory_usage", 0)
    if memory_usage > 90:
      health_status["status"] = "critical"
      health_status["issues"].append(f"High memory usage: {memory_usage:.1f}%")
    elif memory_usage > 75:
      if health_status["status"] != "critical":
        health_status["status"] = "warning"
      health_status["issues"].append(f"Moderate memory usage: {memory_usage:.1f}%")
    
    # Check temperature health
    cpu_temp = current_metrics.get("cpu_temp_max", 0)
    if cpu_temp > 85:
      health_status["status"] = "critical"
      health_status["issues"].append(f"High CPU temperature: {cpu_temp:.1f}°C")
    elif cpu_temp > 75:
      if health_status["status"] != "critical":
        health_status["status"] = "warning"
      health_status["issues"].append(f"Moderate CPU temperature: {cpu_temp:.1f}°C")
    
    # Check model confidence
    confidence = current_metrics.get("model_confidence", 1.0)
    if confidence < 0.4:
      health_status["status"] = "critical"
      health_status["issues"].append(f"Low model confidence: {confidence:.2f}")
    elif confidence < 0.6:
      if health_status["status"] != "critical":
        health_status["status"] = "warning"
      health_status["issues"].append(f"Moderate model confidence: {confidence:.2f}")
    
    # Check latency health
    latency = current_metrics.get("end_to_end_latency", 0)
    if latency > 100:
      health_status["status"] = "critical"
      health_status["issues"].append(f"High system latency: {latency:.1f}ms")
    elif latency > 80:
      if health_status["status"] != "critical":
        health_status["status"] = "warning"
      health_status["issues"].append(f"Moderate system latency: {latency:.1f}ms")
    
    return health_status
  
  def stop(self):
    """Stop the dashboard"""
    self.running = False


class DashboardServer:
  """Server to provide dashboard data via API or web interface"""
  
  def __init__(self, dashboard: RealtimeDashboard):
    self.dashboard = dashboard
    self.last_api_call_time = 0
    self.api_call_interval = 0.5  # Limit API calls to 2Hz
  
  def get_dashboard_data(self) -> Dict[str, any]:
    """Get comprehensive dashboard data"""
    # Rate limit API calls
    current_time = time.time()
    if current_time - self.last_api_call_time < self.api_call_interval:
      time.sleep(self.api_call_interval - (current_time - self.last_api_call_time))
    
    self.last_api_call_time = time.time()
    
    return {
      "timestamp": time.time(),
      "current_metrics": self.dashboard.get_current_metrics(),
      "system_health": self.dashboard.get_system_health_status(),
      "recent_alerts": [
        {
          "timestamp": alert.timestamp,
          "metric_name": alert.metric_name,
          "value": alert.value,
          "threshold": alert.threshold,
          "level": alert.level,
          "description": alert.description
        }
        for alert in self.dashboard.get_recent_alerts()
      ],
      "metric_names": list(self.dashboard.metrics_history.keys()),
      "data_collection_stats": data_collection_manager.collector.get_collection_stats()
    }
  
  def get_metric_history(self, metric_name: str, points: int = 50) -> Dict[str, any]:
    """Get historical data for a specific metric"""
    return {
      "metric_name": metric_name,
      "values": self.dashboard.get_metric_history(metric_name, points),
      "timestamp": time.time()
    }


# Global dashboard instance
realtime_dashboard = RealtimeDashboard()
dashboard_server = DashboardServer(realtime_dashboard)


def dashboard_alert_handler(alert: AlertEvent):
  """Default alert handler that logs alerts"""
  log_level = "warning" if alert.level == "warning" else "error"
  getattr(cloudlog, log_level)(f"DASHBOARD ALERT: {alert.description}")


# Register the default alert handler
realtime_dashboard.add_alert_callback(dashboard_alert_handler)


__all__ = [
  "MetricThreshold", "AlertEvent", "RealtimeDashboard", "DashboardServer",
  "realtime_dashboard", "dashboard_server", "dashboard_alert_handler"
]