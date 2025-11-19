"""
Data Collection Pipeline for sunnypilot
Collects edge cases, system anomalies, and performance metrics for ongoing model improvement
"""
import json
import time
import threading
import queue
from pathlib import Path
from typing import Dict, Any, Optional, List
from dataclasses import dataclass

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import HARDWARE


@dataclass
class PerformanceMetric:
  """Performance metric data structure"""
  timestamp: float
  component: str
  operation: str
  execution_time_ms: float
  cpu_usage: float
  memory_usage: float
  gpu_usage: float
  thermal_status: int
  additional_data: Optional[Dict[str, Any]] = None


@dataclass
class EdgeCaseEvent:
  """Edge case event data structure"""
  timestamp: float
  event_type: str
  description: str
  severity: str  # 'low', 'medium', 'high', 'critical'
  data: Dict[str, Any]
  engaged: bool
  v_ego: float
  model_confidence: float


class DataCollector:
  """Main data collection system"""

  def __init__(self, output_dir: str = "/data/sunnypilot_metrics"):
    self.output_dir = Path(output_dir)
    self.output_dir.mkdir(parents=True, exist_ok=True)

    # Queues for different types of data
    self.performance_queue = queue.Queue(maxsize=1000)
    self.edge_cases_queue = queue.Queue(maxsize=500)

    # Stats tracking
    self.collected_metrics = 0
    self.collected_edge_cases = 0

    # Threading
    self.data_processing_thread = threading.Thread(target=self._process_data, daemon=True)
    self.data_processing_thread.start()

    # SubMaster for getting relevant data
    self.sm = messaging.SubMaster([
      'deviceState', 'carState', 'modelV2', 'selfdriveState', 'validationMetrics'
    ])

    # Flags for data collection
    self.collection_enabled = True
    self.params = Params()

    cloudlog.info("Data collection system initialized")

  def collect_performance_metric(self, metric: PerformanceMetric):
    """Collect a performance metric with proper error handling"""
    if not self.collection_enabled:
      return

    if not isinstance(metric, PerformanceMetric):
      cloudlog.error(f"Invalid metric type passed to collection: {type(metric)}")
      return

    try:
      self.performance_queue.put_nowait(metric)
      self.collected_metrics += 1
    except queue.Full:
      cloudlog.warning("Performance metric queue is full, dropping metric")
    except Exception as e:
      cloudlog.error(f"Error collecting performance metric: {e}")

  def collect_edge_case(self, event: EdgeCaseEvent):
    """Collect an edge case event"""
    if not self.collection_enabled:
      return

    try:
      self.edge_cases_queue.put_nowait(event)
      self.collected_edge_cases += 1
    except queue.Full:
      cloudlog.warning("Edge case queue is full, dropping event")

  def _process_data(self):
    """Process collected data in background thread"""
    while True:
      try:
        # Process performance metrics
        try:
          while True:
            metric = self.performance_queue.get_nowait()
            self._write_performance_metric(metric)
        except queue.Empty:
          pass

        # Process edge cases
        try:
          while True:
            event = self.edge_cases_queue.get_nowait()
            self._write_edge_case(event)
        except queue.Empty:
          pass

        # Update submaster regularly
        self.sm.update(0)

        # Sleep to prevent excessive CPU usage
        time.sleep(0.05)

      except Exception as e:
        cloudlog.error(f"Data processing error: {e}")
        time.sleep(0.05)

  def _write_performance_metric(self, metric: PerformanceMetric):
    """Write performance metric to storage"""
    try:
      filename = self.output_dir / f"perf_{int(metric.timestamp)}_{metric.component.replace(':', '_')}.json"
      data_dict = {
        'type': 'performance',
        'timestamp': metric.timestamp,
        'component': metric.component,
        'operation': metric.operation,
        'execution_time_ms': metric.execution_time_ms,
        'cpu_usage': metric.cpu_usage,
        'memory_usage': metric.memory_usage,
        'gpu_usage': metric.gpu_usage,
        'thermal_status': metric.thermal_status,
        'additional_data': metric.additional_data or {}
      }
      with open(filename, 'w') as f:
        json.dump(data_dict, f, separators=(',', ':'))  # Compact JSON format
    except Exception as e:
      cloudlog.error(f"Failed to write performance metric: {e}")

  def _write_edge_case(self, event: EdgeCaseEvent):
    """Write edge case to storage"""
    try:
      filename = self.output_dir / f"edge_{int(event.timestamp)}_{event.event_type.lower()}.json"
      data_dict = {
        'type': 'edge_case',
        'timestamp': event.timestamp,
        'event_type': event.event_type,
        'description': event.description,
        'severity': event.severity,
        'data': event.data,
        'engaged': event.engaged,
        'v_ego': event.v_ego,
        'model_confidence': event.model_confidence
      }
      with open(filename, 'w') as f:
        json.dump(data_dict, f, separators=(',', ':'))  # Compact JSON format
    except Exception as e:
      cloudlog.error(f"Failed to write edge case: {e}")

  def get_collection_stats(self) -> Dict[str, int]:
    """Get statistics about collected data"""
    return {
      'performance_metrics': self.collected_metrics,
      'edge_cases': self.collected_edge_cases,
      'perf_queue_size': self.performance_queue.qsize(),
      'edge_queue_size': self.edge_cases_queue.qsize()
    }


# Global data collection instance
data_collector = DataCollector()


def collect_model_performance(component: str, operation: str,
                            execution_time_ms: float, additional_data: Optional[Dict[str, Any]] = None):
  """Helper function to collect model performance metrics"""
  if not component or not operation:
    return

  if execution_time_ms < 0:
    return

  import psutil
  cpu_usage = psutil.cpu_percent(interval=None)
  memory_usage = psutil.virtual_memory().percent
  gpu_usage = HARDWARE.get_gpu_usage_percent()

  metric = PerformanceMetric(
    timestamp=time.time(),
    component=component,
    operation=operation,
    execution_time_ms=execution_time_ms,
    cpu_usage=cpu_usage,
    memory_usage=memory_usage,
    gpu_usage=gpu_usage,
    thermal_status=0,
    additional_data=additional_data
  )

  data_collector.collect_performance_metric(metric)


__all__ = [
  "PerformanceMetric", "EdgeCaseEvent", "data_collector", "collect_model_performance"
]