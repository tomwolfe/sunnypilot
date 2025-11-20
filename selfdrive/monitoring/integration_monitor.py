"""
Integration module for autonomous driving improvements
Connects monitoring, DEC enhancements, and other improvement areas
"""
import time
from typing import Dict, Any, Optional

import cereal.messaging as messaging
from cereal import custom
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.monitoring.autonomous_metrics import get_metrics_collector
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist


class AutonomousDrivingIntegrator:
  """Integrates all autonomous driving improvements and manages their interactions"""
  
  def __init__(self):
    self.params = Params()
    self.shutdown = False
    self.enabled = True
    
    # Initialize components
    self.metrics_collector = get_metrics_collector()
    self.dec_controller: Optional[DynamicExperimentalController] = None
    self.sla_controller: Optional[SpeedLimitAssist] = None
    
    # State tracking
    self.sm = messaging.SubMaster(['carState', 'modelV2', 'controlsState', 'radarState', 'selfdriveState'])
    self.pm = messaging.PubMaster(['driverAssistance'])
    
    # Performance and safety tracking
    self.system_stability_score = 1.0  # 0.0 - 1.0, with 1.0 being perfect stability
    self.last_update_time = time.time()
    
    cloudlog.info("Autonomous Driving Integrator initialized")
  
  def initialize_systems(self, CP, CP_SP):
    """Initialize all autonomous driving improvement systems"""
    try:
      # Initialize DEC controller
      from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
      mpc = LongitudinalMpc()
      self.dec_controller = DynamicExperimentalController(CP, mpc)
      
      # Initialize Speed Limit Assist
      self.sla_controller = SpeedLimitAssist(CP, CP_SP)
      
      # Initialize metrics collector
      self.metrics_collector.initialize_submaster(self.sm)
      
      cloudlog.info("All autonomous driving systems initialized successfully")
      return True
    except Exception as e:
      cloudlog.error(f"Error initializing autonomous driving systems: {e}")
      return False
  
  def update_integration(self) -> Dict[str, Any]:
    """Update all integrated systems and return status"""
    # Update messaging system with timeout to avoid blocking
    self.sm.update(0)

    # Collect metrics first
    metrics = self.metrics_collector.collect_metrics()

    results = {
      'dec_active': False,
      'sla_active': False,
      'system_health': 'unknown',
      'stability_score': self.system_stability_score
    }

    # Update DEC if available
    if self.dec_controller:
      try:
        self.dec_controller.update(self.sm)
        results['dec_active'] = self.dec_controller.active()
      except Exception as e:
        cloudlog.error(f"Error updating DEC: {e}")

    # Update Speed Limit Assist if available
    if self.sla_controller:
      try:
        # Get necessary parameters for SLA update
        car_state = self.sm['carState']
        v_ego = getattr(car_state, 'vEgo', 0.0)
        a_ego = getattr(car_state, 'aEgo', 0.0)
        v_cruise_cluster = getattr(car_state, 'vCruise', 255.0)

        # Dummy values for other required parameters (in real implementation these would come from proper sources)
        long_enabled = True
        long_override = False
        speed_limit = 0.0
        speed_limit_final_last = 0.0
        has_speed_limit = False
        distance = 0.0

        # Import EventsSP for SLA update
        from openpilot.sunnypilot.selfdrive.events import EventsSP
        events_sp = EventsSP()

        self.sla_controller.update(
          long_enabled, long_override, v_ego, a_ego, v_cruise_cluster,
          speed_limit, speed_limit_final_last, has_speed_limit,
          distance, events_sp
        )
        results['sla_active'] = self.sla_controller.is_active
      except Exception as e:
        cloudlog.error(f"Error updating SLA: {e}")

    # Update system health based on metrics
    health_report = self.metrics_collector.get_system_health()
    results['system_health'] = health_report.get('status', 'unknown')

    # Calculate stability score based on key metrics
    avg_lateral_jerk = health_report.get('avg_lateral_jerk', 0.0)
    # Lower jerk = higher stability
    jerk_factor = max(0.0, 1.0 - avg_lateral_jerk / 10.0)  # Normalize
    self.system_stability_score = jerk_factor

    results['stability_score'] = self.system_stability_score

    # Log important changes (only periodically to reduce logging overhead)
    if results['system_health'] != 'healthy' and self.metrics_collector.frame_count % 100 == 0:
      cloudlog.warning(f"System health issue detected: {results['system_health']}")

    return results
  
  def run_integration_loop(self, duration: float = 300.0):  # 5 minutes by default
    """Run the integration loop for monitoring and improvement"""
    start_time = time.time()
    cloudlog.info(f"Starting autonomous driving integration monitoring for {duration} seconds")
    
    loop_count = 0
    report_interval = 200  # Report every ~10 seconds at 20Hz (DT_MDL)
    
    try:
      while time.time() - start_time < duration and not self.shutdown:
        results = self.update_integration()
        
        # Periodic reporting
        if loop_count % report_interval == 0:
          cloudlog.info(f"Integration Status - Health: {results['system_health']}, "
                       f"DEC Active: {results['dec_active']}, "
                       f"SLA Active: {results['sla_active']}, "
                       f"Stability: {results['stability_score']:.2f}")
        
        # In a real system, we would adjust parameters based on feedback
        if self.system_stability_score < 0.5:
          # System is unstable, could temporarily reduce experimental mode usage
          cloudlog.warning("System stability low, considering conservative mode")
        
        loop_count += 1
        time.sleep(0.05)  # DT_MDL equivalent
        
    except KeyboardInterrupt:
      cloudlog.info("Integration monitoring interrupted by user")
    except Exception as e:
      cloudlog.error(f"Error in integration loop: {e}")
    
    cloudlog.info("Autonomous driving integration monitoring completed")
  
  def get_performance_insights(self) -> Dict[str, Any]:
    """Get insights about system performance for continuous improvement"""
    try:
      metrics_report = self.metrics_collector.get_performance_report()
      health_report = self.metrics_collector.get_system_health()
      
      insights = {
        "performance_summary": metrics_report,
        "health_summary": health_report,
        "recommendations": [],
        "stability_trend": "unknown"
      }
      
      # Analyze performance data and generate recommendations
      if 'avg_lateral_jerk' in metrics_report:
        if metrics_report['avg_lateral_jerk'] > 2.0:
          insights['recommendations'].append(
            "High lateral jerk detected. Consider smoothing steering control parameters."
          )
      
      if 'avg_longitudinal_jerk' in metrics_report:
        if metrics_report['avg_longitudinal_jerk'] > 1.5:
          insights['recommendations'].append(
            "High longitudinal jerk detected. Consider smoothing acceleration/deceleration profiles."
          )
      
      if 'avg_cpu_util' in metrics_report:
        if metrics_report['avg_cpu_util'] > 85.0:
          insights['recommendations'].append(
            "High CPU utilization detected. Consider model optimization or computational load balancing."
          )
      
      # Determine stability trend
      if hasattr(self, '_previous_stability_score'):
        if self.system_stability_score > self._previous_stability_score:
          insights['stability_trend'] = "improving"
        elif self.system_stability_score < self._previous_stability_score:
          insights['stability_trend'] = "degrading"
        else:
          insights['stability_trend'] = "stable"
      
      self._previous_stability_score = self.system_stability_score
      
      return insights
    except Exception as e:
      cloudlog.error(f"Error generating performance insights: {e}")
      return {"error": str(e)}


def main():
  """Main entry point for integration monitoring"""
  cloudlog.info("Starting Autonomous Driving Integration Monitor")
  
  try:
    integrator = AutonomousDrivingIntegrator()
    
    # This would normally need proper CarParams to initialize, 
    # but we'll simulate the initialization for demonstration
    cloudlog.info("Starting integration monitoring... (Press Ctrl+C to stop)")
    integrator.run_integration_loop(duration=120)  # Run for 2 minutes
    
    # Generate final insights
    insights = integrator.get_performance_insights()
    cloudlog.info(f"Final performance insights: {insights}")
    
  except KeyboardInterrupt:
    cloudlog.info("Integration monitoring stopped by user")
  except Exception as e:
    cloudlog.error(f"Fatal error in integration service: {e}")


if __name__ == "__main__":
  main()