"""
Data Collection Pipeline for sunnypilot
Collects edge cases, system anomalies, and performance metrics for ongoing model improvement
"""
import os
import json
import time
import threading
import queue
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, Optional, List, Callable
from dataclasses import dataclass, asdict
from collections import deque

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
  additional_data: Dict[str, Any] = None


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


@dataclass
class SystemAnomaly:
  """System anomaly data structure"""
  timestamp: float
  anomaly_type: str
  description: str
  severity: str  # 'warning', 'error', 'critical'
  metrics_before: Dict[str, Any]
  metrics_after: Dict[str, Any]


class DataCollector:
  """Main data collection system"""
  
  def __init__(self, output_dir: str = "/data/sunnypilot_metrics"):
    self.output_dir = Path(output_dir)
    self.output_dir.mkdir(parents=True, exist_ok=True)
    
    # Queues for different types of data
    self.performance_queue = queue.Queue(maxsize=1000)
    self.edge_cases_queue = queue.Queue(maxsize=500)
    self.anomalies_queue = queue.Queue(maxsize=500)
    
    # Stats tracking
    self.collected_metrics = 0
    self.collected_edge_cases = 0
    self.collected_anomalies = 0
    
    # Threading
    self.data_processing_thread = threading.Thread(target=self._process_data, daemon=True)
    self.data_processing_thread.start()
    
    # SubMaster for getting relevant data
    self.sm = messaging.SubMaster([
      'deviceState', 'carState', 'modelV2', 'selfdriveState',
      'radarState', 'driverMonitoringState', 'validationMetrics'
    ])
    
    # Flags for data collection
    self.collection_enabled = True
    self.params = Params()
    
    cloudlog.info("Data collection system initialized")
  
  def collect_performance_metric(self, metric: PerformanceMetric):
    """Collect a performance metric"""
    if not self.collection_enabled:
      return
    
    try:
      self.performance_queue.put_nowait(metric)
      self.collected_metrics += 1
    except queue.Full:
      cloudlog.warning("Performance metric queue is full, dropping metric")
  
  def collect_edge_case(self, event: EdgeCaseEvent):
    """Collect an edge case event"""
    if not self.collection_enabled:
      return
    
    try:
      self.edge_cases_queue.put_nowait(event)
      self.collected_edge_cases += 1
    except queue.Full:
      cloudlog.warning("Edge case queue is full, dropping event")
  
  def collect_anomaly(self, anomaly: SystemAnomaly):
    """Collect a system anomaly"""
    if not self.collection_enabled:
      return
    
    try:
      self.anomalies_queue.put_nowait(anomaly)
      self.collected_anomalies += 1
    except queue.Full:
      cloudlog.warning("Anomaly queue is full, dropping anomaly")
  
  def detect_edge_cases(self, model_data: Dict[str, Any], car_state: Dict[str, Any], 
                       validation_metrics: Optional[Dict[str, Any]] = None) -> List[EdgeCaseEvent]:
    """Detect potential edge cases in current data"""
    edge_cases = []
    
    # Check for low confidence scenarios
    if validation_metrics and validation_metrics.get('overallConfidence', 1.0) < 0.5:
      edge_cases.append(EdgeCaseEvent(
        timestamp=time.time(),
        event_type="LOW_MODEL_CONFIDENCE",
        description="Model confidence below threshold",
        severity="high",
        data={
          'overall_confidence': validation_metrics.get('overallConfidence', 0.0),
          'lead_confidence': validation_metrics.get('leadConfidenceAvg', 0.0),
          'lane_confidence': validation_metrics.get('laneConfidenceAvg', 0.0)
        },
        engaged=car_state.get('enabled', False),
        v_ego=car_state.get('vEgo', 0.0),
        model_confidence=validation_metrics.get('overallConfidence', 0.0) if validation_metrics else 0.0
      ))
    
    # Check for unusual driving scenarios
    v_ego = car_state.get('vEgo', 0.0)
    a_ego = car_state.get('aEgo', 0.0)
    
    if v_ego > 30 and abs(a_ego) > 4.0:  # High speed with high acceleration/deceleration
      edge_cases.append(EdgeCaseEvent(
        timestamp=time.time(),
        event_type="HIGH_SPEED_HIGH_ACCEL",
        description="High speed with high acceleration/deceleration",
        severity="medium",
        data={
          'v_ego': v_ego,
          'a_ego': a_ego,
          'steering_angle': car_state.get('steeringAngleDeg', 0.0)
        },
        engaged=car_state.get('enabled', False),
        v_ego=v_ego,
        model_confidence=validation_metrics.get('overallConfidence', 1.0) if validation_metrics else 1.0
      ))
    
    # Check for lane departure without signal
    if (model_data.get('meta', {}).get('laneChangeState', 0) == 0 and 
        abs(car_state.get('steeringAngleDeg', 0.0)) > 15.0 and v_ego > 10):
      edge_cases.append(EdgeCaseEvent(
        timestamp=time.time(),
        event_type="HIGH_STEERING_NO_LANE_CHANGE",
        description="High steering angle without lane change state",
        severity="medium",
        data={
          'steering_angle': car_state.get('steeringAngleDeg', 0.0),
          'lane_change_state': model_data.get('meta', {}).get('laneChangeState', 0),
          'v_ego': v_ego
        },
        engaged=car_state.get('enabled', False),
        v_ego=v_ego,
        model_confidence=validation_metrics.get('overallConfidence', 1.0) if validation_metrics else 1.0
      ))
    
    return edge_cases
  
  def detect_anomalies(self, metrics_before: Dict[str, Any], 
                      metrics_after: Dict[str, Any]) -> List[SystemAnomaly]:
    """Detect system anomalies by comparing metrics"""
    anomalies = []
    
    # Check for sudden performance drops
    if (metrics_before.get('execution_time_ms', 0) > 0 and
        metrics_after.get('execution_time_ms', 0) > metrics_before['execution_time_ms'] * 3.0):
      anomalies.append(SystemAnomaly(
        timestamp=time.time(),
        anomaly_type="PERFORMANCE_SPIKE",
        description="Sudden increase in execution time",
        severity="warning",
        metrics_before=metrics_before,
        metrics_after=metrics_after
      ))
    
    # Check for thermal anomalies
    temp_diff = (metrics_after.get('cpu_temp', 0) - metrics_before.get('cpu_temp', 0))
    if temp_diff > 10.0:  # Temperature increased by more than 10C
      anomalies.append(SystemAnomaly(
        timestamp=time.time(),
        anomaly_type="THERMAL_SPIKE",
        description="Sudden temperature increase",
        severity="warning",
        metrics_before=metrics_before,
        metrics_after=metrics_after
      ))
    
    # Check for memory usage anomalies
    memory_diff = (metrics_after.get('memory_usage', 0) - metrics_before.get('memory_usage', 0))
    if memory_diff > 20.0:  # Memory usage increased by more than 20%
      anomalies.append(SystemAnomaly(
        timestamp=time.time(),
        anomaly_type="MEMORY_SPIKE",
        description="Sudden memory usage increase",
        severity="warning",
        metrics_before=metrics_before,
        metrics_after=metrics_after
      ))
    
    return anomalies
  
  def _process_data(self):
    """Process collected data in background thread"""
    while True:
      try:
        # Process all queued items
        processed = 0
        
        # Process performance metrics
        while not self.performance_queue.empty():
          try:
            metric = self.performance_queue.get_nowait()
            self._write_performance_metric(metric)
          except queue.Empty:
            break
          processed += 1
          
        # Process edge cases
        while not self.edge_cases_queue.empty():
          try:
            edge_case = self.edge_cases_queue.get_nowait()
            self._write_edge_case(edge_case)
          except queue.Empty:
            break
          processed += 1
          
        # Process anomalies
        while not self.anomalies_queue.empty():
          try:
            anomaly = self.anomalies_queue.get_nowait()
            self._write_anomaly(anomaly)
          except queue.Empty:
            break
          processed += 1
        
        # Update submaster regularly
        self.sm.update(0)
        
        # Sleep if no data was processed
        if processed == 0:
          time.sleep(0.1)
          
      except Exception as e:
        cloudlog.error(f"Data processing error: {e}")
        time.sleep(1.0)
  
  def _write_performance_metric(self, metric: PerformanceMetric):
    """Write performance metric to storage"""
    try:
      filename = self.output_dir / f"perf_{int(metric.timestamp)}.json"
      with open(filename, 'w') as f:
        json.dump({
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
        }, f)
    except Exception as e:
      cloudlog.error(f"Failed to write performance metric: {e}")
  
  def _write_edge_case(self, event: EdgeCaseEvent):
    """Write edge case to storage"""
    try:
      filename = self.output_dir / f"edge_{int(event.timestamp)}_{event.event_type.lower()}.json"
      with open(filename, 'w') as f:
        json.dump({
          'type': 'edge_case',
          'timestamp': event.timestamp,
          'event_type': event.event_type,
          'description': event.description,
          'severity': event.severity,
          'data': event.data,
          'engaged': event.engaged,
          'v_ego': event.v_ego,
          'model_confidence': event.model_confidence
        }, f)
    except Exception as e:
      cloudlog.error(f"Failed to write edge case: {e}")
  
  def _write_anomaly(self, anomaly: SystemAnomaly):
    """Write anomaly to storage"""
    try:
      filename = self.output_dir / f"anomaly_{int(anomaly.timestamp)}_{anomaly.anomaly_type.lower()}.json"
      with open(filename, 'w') as f:
        json.dump({
          'type': 'anomaly',
          'timestamp': anomaly.timestamp,
          'anomaly_type': anomaly.anomaly_type,
          'description': anomaly.description,
          'severity': anomaly.severity,
          'metrics_before': anomaly.metrics_before,
          'metrics_after': anomaly.metrics_after
        }, f)
    except Exception as e:
      cloudlog.error(f"Failed to write anomaly: {e}")
  
  def get_collection_stats(self) -> Dict[str, int]:
    """Get statistics about collected data"""
    return {
      'performance_metrics': self.collected_metrics,
      'edge_cases': self.collected_edge_cases,
      'anomalies': self.collected_anomalies,
      'perf_queue_size': self.performance_queue.qsize(),
      'edge_queue_size': self.edge_cases_queue.qsize(),
      'anomaly_queue_size': self.anomalies_queue.qsize()
    }


class DataCollectionManager:
  """Manager for coordinating data collection across the system"""
  
  def __init__(self):
    self.collector = DataCollector()
    self.collection_threads = []
    self.is_collecting = True
    self.collection_interval = 1.0  # seconds between detailed collections
  
  def start_collection(self):
    """Start data collection process"""
    collection_thread = threading.Thread(target=self._collect_continuous_data, daemon=True)
    collection_thread.start()
    self.collection_threads.append(collection_thread)
    
    cloudlog.info("Data collection manager started")
  
  def _collect_continuous_data(self):
    """Continuously collect data in a background thread"""
    last_collection_time = time.time()
    
    while self.is_collecting:
      try:
        # Update submaster
        self.collector.sm.update(1000)  # 1 second timeout
        
        # Collect system metrics
        if time.time() - last_collection_time >= self.collection_interval:
          self._collect_system_metrics()
          last_collection_time = time.time()
        
        # Check for edge cases in current data
        self._check_for_edge_cases()
        
        time.sleep(0.1)  # Small delay to prevent busy waiting
        
      except Exception as e:
        cloudlog.error(f"Continuous data collection error: {e}")
        time.sleep(1.0)
  
  def _collect_system_metrics(self):
    """Collect comprehensive system metrics"""
    try:
      # Get system metrics
      import psutil
      
      cpu_percent = psutil.cpu_percent(interval=None)
      memory_percent = psutil.virtual_memory().percent
      gpu_percent = HARDWARE.get_gpu_usage_percent()
      
      # Get thermal data
      device_state = self.collector.sm['deviceState']
      thermal_status = device_state.thermalStatus if self.collector.sm.valid['deviceState'] else 0
      
      # Create performance metric
      metric = PerformanceMetric(
        timestamp=time.time(),
        component='system_monitor',
        operation='health_check',
        execution_time_ms=0.0,
        cpu_usage=cpu_percent,
        memory_usage=memory_percent,
        gpu_usage=gpu_percent,
        thermal_status=thermal_status,
        additional_data={
          'cpu_temps': device_state.cpuTempC if self.collector.sm.valid['deviceState'] else [],
          'gpu_temps': device_state.gpuTempC if self.collector.sm.valid['deviceState'] else [],
          'memory_temp': device_state.memoryTempC if self.collector.sm.valid['deviceState'] else 0.0,
          'pmic_temps': device_state.pmicTempC if self.collector.sm.valid['deviceState'] else []
        }
      )
      
      self.collector.collect_performance_metric(metric)
      
    except Exception as e:
      cloudlog.error(f"Error collecting system metrics: {e}")
  
  def _check_for_edge_cases(self):
    """Check current data for edge cases"""
    try:
      sm = self.collector.sm
      
      # Prepare data for edge case detection
      model_data = {}
      if sm.valid['modelV2']:
        model_data = {
          'meta': {
            'laneChangeState': int(sm['modelV2'].meta.laneChangeState),
            'laneChangeDirection': int(sm['modelV2'].meta.laneChangeDirection)
          },
          'position': sm['modelV2'].position.x,
          'velocity': sm['modelV2'].velocity.x
        }
      
      car_state = {}
      if sm.valid['carState']:
        car_state = {
          'vEgo': sm['carState'].vEgo,
          'aEgo': sm['carState'].aEgo,
          'steeringAngleDeg': sm['carState'].steeringAngleDeg,
          'enabled': sm['selfdriveState'].enabled if sm.valid['selfdriveState'] else False
        }
      
      validation_metrics = None
      if sm.valid['validationMetrics']:
        validation_metrics = {
          'overallConfidence': sm['validationMetrics'].overallConfidence,
          'leadConfidenceAvg': sm['validationMetrics'].leadConfidenceAvg,
          'laneConfidenceAvg': sm['validationMetrics'].laneConfidenceAvg,
          'isValid': sm['validationMetrics'].isValid
        }
      
      # Detect edge cases
      edge_cases = self.collector.detect_edge_cases(model_data, car_state, validation_metrics)
      
      # Collect detected edge cases
      for edge_case in edge_cases:
        self.collector.collect_edge_case(edge_case)
      
    except Exception as e:
      cloudlog.error(f"Error checking for edge cases: {e}")
  
  def stop_collection(self):
    """Stop data collection"""
    self.is_collecting = False
    cloudlog.info("Data collection manager stopped")


# Global data collection manager instance
data_collection_manager = DataCollectionManager()


def collect_model_performance(component: str, operation: str, 
                            execution_time_ms: float, additional_data: Dict[str, Any] = None):
  """Helper function to collect model performance metrics"""
  try:
    import psutil
    
    metric = PerformanceMetric(
      timestamp=time.time(),
      component=component,
      operation=operation,
      execution_time_ms=execution_time_ms,
      cpu_usage=psutil.cpu_percent(interval=None),
      memory_usage=psutil.virtual_memory().percent,
      gpu_usage=HARDWARE.get_gpu_usage_percent(),
      thermal_status=0,  # Will be updated later with real thermal status
      additional_data=additional_data
    )
    
    data_collection_manager.collector.collect_performance_metric(metric)
  except Exception as e:
    cloudlog.error(f"Error collecting performance metric: {e}")


def collect_lane_change_event(lane_change_state: int, confidence: float, 
                            steering_angle: float, v_ego: float):
  """Collect lane change related events"""
  if lane_change_state != 0:  # Active lane change
    event = EdgeCaseEvent(
      timestamp=time.time(),
      event_type="LANE_CHANGE_EVENT",
      description=f"Lane change state: {lane_change_state}",
      severity="low",
      data={
        'lane_change_state': lane_change_state,
        'confidence': confidence,
        'steering_angle': steering_angle
      },
      engaged=True,  # Assuming engaged during lane change
      v_ego=v_ego,
      model_confidence=confidence
    )
    data_collection_manager.collector.collect_edge_case(event)


__all__ = [
  "PerformanceMetric", "EdgeCaseEvent", "SystemAnomaly",
  "DataCollector", "DataCollectionManager", "data_collection_manager",
  "collect_model_performance", "collect_lane_change_event"
]