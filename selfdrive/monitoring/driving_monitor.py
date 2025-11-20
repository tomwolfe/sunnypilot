"""
Monitoring Service for Autonomous Driving Improvement Plan
This service collects and reports on metrics for the enhanced DEC system
"""
import os
import time
import json
from typing import Dict, Any
import numpy as np

import cereal.messaging as messaging
from cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.monitoring.autonomous_metrics import get_metrics_collector, AutonomousMetricsCollector
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController


class AutonomousDrivingMonitor:
  """Main monitoring service for autonomous driving improvements"""
  
  def __init__(self):
    self.params = Params()
    self.shutdown = False
    
    # Initialize metrics collector
    self.metrics_collector = get_metrics_collector()
    
    # Initialize with messaging
    self.sm = messaging.SubMaster(['carState', 'modelV2', 'controlsState', 'deviceState', 'radarState'])
    self.pm = messaging.PubMaster(['carParams', 'onroadEvents', 'driverAssistance'])
    
    # Initialize the enhanced DEC system
    self.dec_controller = None  # Will be initialized later with proper parameters
    
    # Performance tracking
    self.frame_count = 0
    self.last_report_time = time.time()
    self.reporting_interval = 5.0  # Report every 5 seconds
    
    # Initialize metrics collector with submaster
    self.metrics_collector.initialize_submaster(self.sm)
    
    cloudlog.info("Autonomous Driving Monitor initialized")
  
  def initialize_dec_system(self, CP):
    """Initialize the Dynamic Experimental Controller"""
    # This would require access to MPC and other systems in real implementation
    try:
      from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
      mpc = LongitudinalMpc()
      self.dec_controller = DynamicExperimentalController(CP, mpc)
    except Exception as e:
      cloudlog.error(f"Could not initialize DEC controller: {e}")
      # Continue without DEC for monitoring purposes only
  
  def collect_and_monitor(self) -> Dict[str, Any]:
    """Main monitoring function - collect metrics and monitor system performance"""
    self.sm.update(0)
    self.frame_count += 1

    # If DEC controller exists, update it first (critical path)
    if self.dec_controller:
      try:
        # Update DEC controller first to ensure critical control is not delayed
        self.dec_controller.update(self.sm)
      except Exception as e:
        cloudlog.error(f"Error updating DEC controller: {e}")

    # Non-critical monitoring operations happen after critical control updates
    # This moves monitoring off the critical control path
    current_metrics = None
    try:
      # Collect metrics in a non-blocking way or with timeout
      current_metrics = self.metrics_collector.collect_metrics()
    except Exception as e:
      cloudlog.warning(f"Non-critical metrics collection failed: {e}")
      # Continue operation even if metrics collection fails

    # Check if it's time for a detailed report (non-critical)
    current_time = time.time()
    if current_time - self.last_report_time >= self.reporting_interval:
      try:
        performance_report = self.metrics_collector.get_performance_report()
        system_health = self.metrics_collector.get_system_health()

        # Log important metrics
        cloudlog.info(f"Driving Performance Report: "
                     f"Duration: {performance_report['duration']:.1f}s, "
                     f"Avg CPU: {performance_report['avg_cpu_util']:.1f}%, "
                     f"Avg Lat Jerk: {performance_report['avg_lateral_jerk']:.2f}, "
                     f"Health: {system_health['status']}")

        self.last_report_time = current_time

        return {
          "performance_report": performance_report,
          "system_health": system_health,
          "current_metrics": current_metrics.__dict__ if current_metrics else {}
        }
      except Exception as e:
        cloudlog.warning(f"Non-critical reporting failed: {e}")

    # Return minimal essential data quickly
    return {
      "system_health": self.metrics_collector.get_system_health() if current_metrics else {"status": "healthy"},
      "current_metrics": current_metrics.__dict__ if current_metrics else {}
    }
  
  def run_monitoring_cycle(self, duration: float = 60.0):
    """Run a monitoring cycle for specified duration"""
    start_time = time.time()
    cloudlog.info(f"Starting autonomous driving monitoring for {duration} seconds")
    
    try:
      while time.time() - start_time < duration and not self.shutdown:
        # Collect metrics
        data = self.collect_and_monitor()
        
        # In a real implementation, we would send this data to a central service
        # For now, we just log important changes
        health_status = data["system_health"]["status"]
        if health_status != "healthy":
          cloudlog.warning(f"System health alert: {health_status}")
        
        # Sleep for the model update interval
        time.sleep(DT_MDL)
        
    except KeyboardInterrupt:
      cloudlog.info("Monitoring interrupted by user")
    except Exception as e:
      cloudlog.error(f"Error in monitoring cycle: {e}")
    
    cloudlog.info("Autonomous driving monitoring completed")


def main():
  """Main entry point for the monitoring service"""
  cloudlog.info("Starting Autonomous Driving Improvement Monitor")
  
  try:
    monitor = AutonomousDrivingMonitor()
    
    # Run continuous monitoring
    cloudlog.info("Starting continuous monitoring... Press Ctrl+C to stop")
    monitor.run_monitoring_cycle(duration=600)  # Run for 10 minutes by default
    
  except KeyboardInterrupt:
    cloudlog.info("Monitoring stopped by user")
  except Exception as e:
    cloudlog.error(f"Fatal error in monitoring service: {e}")


if __name__ == "__main__":
  main()