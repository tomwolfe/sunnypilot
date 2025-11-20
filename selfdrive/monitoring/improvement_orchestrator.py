"""
Continuous Improvement Framework for Autonomous Driving
Main orchestrator for the autonomous driving improvement plan execution
"""
import time
import json
import threading
from pathlib import Path
from typing import Dict, Any, Callable, Optional
from datetime import datetime

import cereal.messaging as messaging
from cereal import log
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.monitoring.autonomous_metrics import get_metrics_collector
from openpilot.selfdrive.monitoring.integration_monitor import AutonomousDrivingIntegrator
from openpilot.selfdrive.monitoring.nn_optimizer import get_lateral_optimizer


class ImprovementPlanOrchestrator:
  """Main orchestrator for the autonomous driving improvement plan"""
  
  def __init__(self):
    self.params = Params()
    self.shutdown = False
    self.active = False
    
    # Initialize all improvement components
    self.metrics_collector = get_metrics_collector()
    self.integrator = AutonomousDrivingIntegrator()
    self.lateral_optimizer = get_lateral_optimizer()
    
    # Messaging setup
    self.sm = messaging.SubMaster(['carState', 'modelV2', 'controlsState', 'deviceState', 'radarState'])
    self.pm = messaging.PubMaster(['onroadEvents', 'driverAssistance'])
    
    # Improvement tracking
    self.improvement_progress = {
      'nn_lateral_optimization': {'completed': False, 'score': 0.0, 'timestamp': None},
      'dec_refinement': {'completed': False, 'score': 0.0, 'timestamp': None},
      'environmental_adaptation': {'completed': False, 'score': 0.0, 'timestamp': None},
      'speed_limit_enhancement': {'completed': False, 'score': 0.0, 'timestamp': None},
      'system_integration': {'completed': False, 'score': 0.0, 'timestamp': None}
    }
    
    # Performance baselines (these would be established during initial calibration)
    self.baseline_performance = {
      'avg_lateral_jerk': 2.5,  # Baseline for comparison
      'avg_longitudinal_jerk': 1.8,
      'cpu_utilization': 75.0,
      'system_stability': 0.7,
      'driver_intervention_rate': 0.15  # Interventions per minute
    }
    
    # Improvement goals
    self.improvement_goals = {
      'lateral_jerk_reduction': 0.40,  # 40% reduction target
      'longitudinal_jerk_reduction': 0.35,  # 35% reduction target
      'cpu_efficiency': 0.20,  # 20% improvement target
      'stability_increase': 0.30,  # 30% improvement target
      'intervention_reduction': 0.25  # 25% reduction target
    }
    
    # Continuous learning components
    self.learning_enabled = True
    self.feedback_collector = {}
    self.improvement_algorithms = []
    
    cloudlog.info("Improvement Plan Orchestrator initialized")
  
  def initialize_systems(self, CP, CP_SP):
    """Initialize all improvement systems"""
    success = True
    
    # Initialize the integrator systems
    if not self.integrator.initialize_systems(CP, CP_SP):
      cloudlog.error("Failed to initialize integrator systems")
      success = False
    
    # Set up improvement algorithms
    self._setup_improvement_algorithms()
    
    return success
  
  def _setup_improvement_algorithms(self):
    """Set up various improvement algorithms"""
    # Algorithm for lateral control optimization
    self.improvement_algorithms.append({
      'name': 'lateral_control_optimization',
      'function': self._optimize_lateral_control,
      'frequency': 20,  # Every 20 frames (1 second at 20Hz)
      'enabled': True
    })
    
    # Algorithm for DEC refinement
    self.improvement_algorithms.append({
      'name': 'dec_refinement',
      'function': self._refine_dec_system,
      'frequency': 10,  # Every 10 frames (0.5 seconds at 20Hz)
      'enabled': True
    })
    
    # Algorithm for environmental adaptation
    self.improvement_algorithms.append({
      'name': 'environmental_adaptation',
      'function': self._adapt_to_environment,
      'frequency': 40,  # Every 40 frames (2 seconds at 20Hz)
      'enabled': True
    })
    
    # Algorithm for performance optimization
    self.improvement_algorithms.append({
      'name': 'performance_optimization',
      'function': self._optimize_performance,
      'frequency': 100,  # Every 100 frames (5 seconds at 20Hz)
      'enabled': True
    })
  
  def _optimize_lateral_control(self) -> Dict[str, Any]:
    """Optimize lateral control parameters"""
    try:
      # Get current car state
      car_state = self.sm['carState']
      v_ego = getattr(car_state, 'vEgo', 0.0)
      curvature = getattr(car_state, 'steeringAngleDeg', 0.0) * 0.01745 / 2.5  # Convert to curvature

      # Prepare input for neural network optimizer with better defaults
      input_data = [v_ego, curvature, 0.0]
      # Extend with more realistic data from messaging system if available
      if hasattr(car_state, 'aEgo'):
        input_data.append(getattr(car_state, 'aEgo', 0.0))
      else:
        input_data.append(0.0)

      # Fill with zeros for other required inputs
      input_data.extend([0.0] * max(0, 24 - len(input_data)))

      # Apply optimization
      optimized_steering, opt_info = self.lateral_optimizer.update(input_data, v_ego, curvature)

      # Update progress
      self.improvement_progress['nn_lateral_optimization'] = {
        'completed': True,
        'score': opt_info.get('performance_score', 0.7),
        'timestamp': time.time(),
        'details': opt_info
      }

      return {
        'success': True,
        'optimized_steering': optimized_steering,
        'optimization_info': opt_info
      }
    except Exception as e:
      cloudlog.error(f"Lateral control optimization error: {e}")
      return {'success': False, 'error': str(e)}
  
  def _refine_dec_system(self) -> Dict[str, Any]:
    """Refine Dynamic Experimental Controller"""
    try:
      # Update DEC system
      results = self.integrator.update_integration()
      
      # Calculate DEC refinement score based on system health
      health_status = results.get('system_health', 'unknown')
      dec_active = results.get('dec_active', False)
      
      score = 0.5  # Base score
      if health_status == 'healthy':
        score += 0.3
      if dec_active:
        score += 0.2
      
      # Update progress
      self.improvement_progress['dec_refinement'] = {
        'completed': True,
        'score': min(1.0, score),
        'timestamp': time.time(),
        'details': results
      }
      
      return {
        'success': True,
        'refinement_score': score,
        'system_status': results
      }
    except Exception as e:
      cloudlog.error(f"DEC refinement error: {e}")
      return {'success': False, 'error': str(e)}
  
  def _adapt_to_environment(self) -> Dict[str, Any]:
    """Adapt system to environmental conditions"""
    try:
      device_state = self.sm['deviceState'] if 'deviceState' in self.sm else None
      cpu_load = device_state.cpuUsagePercent[0] if device_state and device_state.cpuUsagePercent else 50.0
      thermal_status = device_state.cpuTempC[0] if device_state and device_state.cpuTempC else 30.0 if device_state else 30.0
      
      # Update optimizer with system status
      self.lateral_optimizer.update_system_status(cpu_load, thermal_status)
      
      # Calculate environment adaptation score
      score = 0.8  # Base score
      if cpu_load < 80.0:
        score += 0.1
      if thermal_status < 70.0:  # Good thermal management
        score += 0.1
      
      # Update progress
      self.improvement_progress['environmental_adaptation'] = {
        'completed': True,
        'score': min(1.0, score),
        'timestamp': time.time(),
        'details': {'cpu_load': cpu_load, 'thermal_status': thermal_status}
      }
      
      return {
        'success': True,
        'adaptation_score': score,
        'system_load': {'cpu': cpu_load, 'thermal': thermal_status}
      }
    except Exception as e:
      cloudlog.error(f"Environmental adaptation error: {e}")
      return {'success': False, 'error': str(e)}
  
  def _optimize_performance(self) -> Dict[str, Any]:
    """Overall system performance optimization"""
    try:
      # Collect performance metrics
      metrics_report = self.metrics_collector.get_performance_report()
      health_report = self.metrics_collector.get_system_health()
      
      # Calculate performance optimization score
      score = 0.5  # Base score
      
      # Improve score based on good metrics
      if 'avg_lateral_jerk' in metrics_report:
        # Lower jerk = better score
        base_jerk = self.baseline_performance['avg_lateral_jerk']
        current_jerk = metrics_report['avg_lateral_jerk']
        improvement = max(0, (base_jerk - current_jerk) / base_jerk)
        score += improvement * 0.3
      
      if 'avg_cpu_util' in metrics_report:
        # Lower CPU usage = better score
        base_cpu = self.baseline_performance['cpu_utilization']
        current_cpu = metrics_report['avg_cpu_util']
        efficiency = max(0, (base_cpu - current_cpu) / base_cpu)
        score += efficiency * 0.2
      
      # Update progress
      self.improvement_progress['system_integration'] = {
        'completed': True,
        'score': min(1.0, score),
        'timestamp': time.time(),
        'details': {
          'metrics_report': metrics_report,
          'health_report': health_report
        }
      }
      
      return {
        'success': True,
        'optimization_score': min(1.0, score),
        'performance_data': {
          'metrics_report': metrics_report,
          'health_report': health_report
        }
      }
    except Exception as e:
      cloudlog.error(f"Performance optimization error: {e}")
      return {'success': False, 'error': str(e)}
  
  def execute_improvement_cycle(self) -> Dict[str, Any]:
    """Execute one cycle of improvement algorithms"""
    # Use a non-blocking update
    self.sm.update(0)

    results = {
      'timestamp': time.time(),
      'frame_count': getattr(self.metrics_collector, 'frame_count', 0),
      'algorithm_results': {}
    }

    # Execute enabled improvement algorithms based on their frequency
    for algo in self.improvement_algorithms:
      if not algo.get('enabled', False):
        continue

      # Check if it's time to run this algorithm
      frame_count = getattr(self.metrics_collector, 'frame_count', 0)
      if frame_count % algo['frequency'] == 0:
        try:
          result = algo['function']()
          results['algorithm_results'][algo['name']] = result
        except Exception as e:
          cloudlog.error(f"Error in {algo['name']} algorithm: {e}")
          results['algorithm_results'][algo['name']] = {'success': False, 'error': str(e)}

    # Additionally, update system status for neural network optimizer
    try:
      device_state = self.sm['deviceState'] if 'deviceState' in self.sm else None
      if device_state:
        cpu_load = device_state.cpuUsagePercent[0] if device_state.cpuUsagePercent else 50.0
        thermal_status = device_state.cpuTempC[0] if device_state.cpuTempC else 30.0
        self.lateral_optimizer.update_system_status(cpu_load, thermal_status)
    except Exception as e:
      cloudlog.error(f"Error updating system status for lateral optimizer: {e}")

    return results
  
  def evaluate_overall_progress(self) -> Dict[str, Any]:
    """Evaluate overall improvement progress"""
    completed_improvements = sum(1 for v in self.improvement_progress.values() if v['completed'])
    total_improvements = len(self.improvement_progress)
    
    # Calculate overall improvement score
    total_score = sum(v['score'] for v in self.improvement_progress.values() if v['completed'])
    avg_score = total_score / completed_improvements if completed_improvements > 0 else 0.0
    
    # Compare with baselines to calculate actual improvement
    current_metrics = self.metrics_collector.get_performance_report()
    health_status = self.metrics_collector.get_system_health()
    
    improvement_analysis = {
      'progress_percentage': (completed_improvements / total_improvements) * 100,
      'overall_score': avg_score,
      'completed_components': completed_improvements,
      'total_components': total_improvements,
      'current_performance': {
        'avg_lateral_jerk': current_metrics.get('avg_lateral_jerk', self.baseline_performance['avg_lateral_jerk']),
        'avg_longitudinal_jerk': current_metrics.get('avg_longitudinal_jerk', self.baseline_performance['avg_longitudinal_jerk']),
        'cpu_utilization': current_metrics.get('avg_cpu_util', self.baseline_performance['cpu_utilization']),
        'system_stability': health_status.get('longitudinal_smoothness', self.baseline_performance['system_stability'])
      },
      'improvement_vs_baseline': {},
      'recommendations': []
    }
    
    # Calculate improvement vs baselines
    current = improvement_analysis['current_performance']
    baseline = self.baseline_performance
    
    for key in ['avg_lateral_jerk', 'avg_longitudinal_jerk', 'cpu_utilization']:
      if key in current and key in baseline:
        # Lower is better for these metrics
        improvement = ((baseline[key] - current[key]) / baseline[key]) * 100
        improvement_analysis['improvement_vs_baseline'][key] = improvement
    
    # System stability - higher is better
    if 'system_stability' in current and 'system_stability' in baseline:
      improvement = ((current['system_stability'] - baseline['system_stability']) / baseline['system_stability']) * 100
      improvement_analysis['improvement_vs_baseline']['system_stability'] = improvement
    
    # Generate recommendations based on metrics
    if current.get('avg_lateral_jerk', 0) > baseline['avg_lateral_jerk'] * 1.2:
      improvement_analysis['recommendations'].append("Lateral jerk is higher than baseline, consider reviewing steering control parameters")
    
    if current.get('cpu_utilization', 0) > baseline['cpu_utilization'] * 1.1:
      improvement_analysis['recommendations'].append("CPU utilization is higher than baseline, consider optimization of computational load")
    
    return improvement_analysis
  
  def run_continuous_improvement(self, duration: float = 600.0):  # 10 minutes by default
    """Run continuous improvement loop"""
    start_time = time.time()
    last_evaluation_time = start_time
    evaluation_interval = 60.0  # Evaluate progress every minute
    
    cloudlog.info(f"Starting continuous improvement cycle for {duration} seconds")
    
    try:
      while time.time() - start_time < duration and not self.shutdown:
        # Execute improvement cycle
        cycle_results = self.execute_improvement_cycle()
        
        # Periodically evaluate overall progress
        current_time = time.time()
        if current_time - last_evaluation_time >= evaluation_interval:
          progress_analysis = self.evaluate_overall_progress()
          
          cloudlog.info(f"Improvement Progress: {progress_analysis['progress_percentage']:.1f}% "
                       f"Overall Score: {progress_analysis['overall_score']:.2f}")
          
          # Log current performance vs baseline
          improvement_vs_base = progress_analysis.get('improvement_vs_baseline', {})
          if 'avg_lateral_jerk' in improvement_vs_base:
            jerk_improvement = improvement_vs_base['avg_lateral_jerk']
            cloudlog.info(f"Lateral jerk improvement: {jerk_improvement:+.1f}% ")
          
          last_evaluation_time = current_time
        
        # Sleep for model update interval
        time.sleep(DT_MDL)
        
    except KeyboardInterrupt:
      cloudlog.info("Continuous improvement interrupted by user")
    except Exception as e:
      cloudlog.error(f"Error in continuous improvement loop: {e}")
    
    # Final evaluation
    final_analysis = self.evaluate_overall_progress()
    cloudlog.info(f"Continuous improvement completed. Final score: {final_analysis['overall_score']:.2f}")
    
    return final_analysis
  
  def generate_improvement_report(self) -> Dict[str, Any]:
    """Generate a comprehensive improvement report"""
    progress_analysis = self.evaluate_overall_progress()
    
    report = {
      'timestamp': datetime.now().isoformat(),
      'improvement_progress': self.improvement_progress,
      'progress_analysis': progress_analysis,
      'recommendations': progress_analysis.get('recommendations', []),
      'next_steps': self._determine_next_steps(progress_analysis)
    }
    
    # Save report to file
    report_path = Path("/tmp/autonomous_improvement_report.json")
    with open(report_path, 'w') as f:
      json.dump(report, f, indent=2, default=str)
    
    cloudlog.info(f"Improvement report saved to {report_path}")
    
    return report
  
  def _determine_next_steps(self, analysis: Dict[str, Any]) -> list:
    """Determine next steps based on current analysis"""
    next_steps = []
    
    # If overall score is low, recommend focused improvements
    if analysis['overall_score'] < 0.6:
      next_steps.append("Focus on critical system stability improvements")
      next_steps.append("Review and optimize high-jerk driving behaviors")
    elif analysis['overall_score'] < 0.8:
      next_steps.append("Continue with planned optimization improvements")
      next_steps.append("Monitor for additional edge case improvements")
    else:
      next_steps.append("Maintain current improvements and continue monitoring")
      next_steps.append("Consider advanced features for next development cycle")
    
    # Add specific recommendations
    if "Lateral jerk is higher than baseline" in str(analysis.get('recommendations', [])):
      next_steps.append("Prioritize lateral control smoothing algorithms")
    
    if "CPU utilization is higher than baseline" in str(analysis.get('recommendations', [])):
      next_steps.append("Implement additional computational optimizations")
    
    return next_steps


def main():
  """Main entry point for continuous improvement framework"""
  cloudlog.info("Starting Autonomous Driving Continuous Improvement Framework")
  
  try:
    orchestrator = ImprovementPlanOrchestrator()
    
    # Note: In a real implementation, we would need to pass actual CP and CP_SP parameters
    # For demonstration, we'll proceed with monitoring only
    cloudlog.info("Starting continuous improvement monitoring... (Press Ctrl+C to stop)")
    
    # Run for 5 minutes to demonstrate the framework
    final_analysis = orchestrator.run_continuous_improvement(duration=300)
    
    # Generate final report
    report = orchestrator.generate_improvement_report()
    
    cloudlog.info(f"Improvement framework completed. Overall score: {final_analysis['overall_score']:.2f}")
    cloudlog.info(f"Recommendations: {report['recommendations']}")
    
  except KeyboardInterrupt:
    cloudlog.info("Continuous improvement framework stopped by user")
  except Exception as e:
    cloudlog.error(f"Fatal error in improvement framework: {e}")


if __name__ == "__main__":
  main()