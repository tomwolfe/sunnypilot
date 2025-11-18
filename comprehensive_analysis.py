#!/usr/bin/env python3
"""
Comprehensive analysis framework for sunnypilot autonomous driving metrics.
This script performs Step 1 analysis as required by the original prompt.
"""

import os
import time
import psutil
import subprocess
import json
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, field
from pathlib import Path

# Define metrics categories and targets as specified in the original prompt
@dataclass
class MetricTargets:
    # Perception (30%)
    object_detection_accuracy: float = 0.95  # Target: 95% accuracy on 100+ test cases
    frame_processing_latency: float = 50.0   # Target: <50ms at 20fps
    false_positive_rate: float = 0.001       # Target: <0.1% for critical objects
    
    # Localization (15%)
    positional_accuracy: float = 1.0         # Target: <1m error vs GPS + map data
    sensor_fusion_robustness: float = 0.95   # Target: 95% accuracy during sensor failures
    
    # Path Planning (20%)
    route_completion_rate: float = 0.98      # Target: ≥98% in 50+ test routes
    trajectory_smoothness: float = 3.0       # Target: jerk <3 m/s³
    obstacle_avoidance_success: float = 0.99 # Target: 99% in edge cases
    
    # Control System (15%)
    steering_braking_latency: float = 30.0   # Target: <30ms
    safety_margin_compliance: float = 0.99   # Target: 99% compliance with safety margins
    fail_safe_behavior: float = 0.95         # Target: 95% success during failures
    
    # Traffic Signal Handling (10%)
    dec_module_accuracy: float = 0.995       # Target: 99.5% compliance on 200+ scenarios
    false_stop_rate: float = 0.0001          # Target: <0.01% false stops
    
    # Hardware Optimization (10%)
    cpu_usage_limit: float = 35.0           # Target: <35% on all cores
    ram_usage_limit: float = 1433.6         # Target: <1.4GB (1433.6 MB)
    power_draw_limit: float = 8.0           # Target: <8W during operation

@dataclass
class AnalysisResult:
    category: str
    metric_name: str
    current_value: float
    target_value: float
    status: str  # "PASS", "FAIL", "NOT_MEASURED"
    confidence: str  # "HIGH", "MEDIUM", "LOW", "NOT_AVAILABLE"
    notes: str = ""

class SunnypilotAnalyzer:
    """Performs comprehensive analysis of sunnypilot codebase against required metrics."""
    
    def __init__(self):
        self.results: List[AnalysisResult] = []
        self.targets = MetricTargets()
        self.hardware_info = self._gather_hardware_info()
        
    def _gather_hardware_info(self) -> Dict[str, Any]:
        """Gather information about the target platform."""
        return {
            "platform": "comma three",
            "ram_gb": 2.0,
            "cpu_cores": 4,
            "cpu_arch": "ARM",
            "power_budget_w": 10.0
        }
    
    def analyze_perception(self) -> List[AnalysisResult]:
        """Analyze perception capabilities - Object detection accuracy, latency, false positives."""
        print("Analyzing perception capabilities...")
        results = []
        
        # 1. Object detection accuracy
        accuracy_result = AnalysisResult(
            category="Perception",
            metric_name="object_detection_accuracy",
            current_value=0.0,  # Unknown - needs to be measured
            target_value=self.targets.object_detection_accuracy,
            status="NOT_MEASURED",
            confidence="NOT_AVAILABLE",
            notes="No current measurement available. Requires test on 100+ real-world cases"
        )
        results.append(accuracy_result)
        
        # 2. Frame processing latency
        latency_result = AnalysisResult(
            category="Perception",
            metric_name="frame_processing_latency_ms",
            current_value=0.0,  # Unknown - needs to be measured
            target_value=self.targets.frame_processing_latency,
            status="NOT_MEASURED",
            confidence="NOT_AVAILABLE",
            notes="No current measurement available. Requires performance profiling at 20fps"
        )
        results.append(latency_result)
        
        # 3. False positive rate
        fps_result = AnalysisResult(
            category="Perception",
            metric_name="false_positive_rate",
            current_value=0.0,  # Unknown - needs to be measured
            target_value=self.targets.false_positive_rate,
            status="NOT_MEASURED",
            confidence="NOT_AVAILABLE",
            notes="No current measurement available. Requires testing for critical objects"
        )
        results.append(fps_result)
        
        print(f"  - Found {len(results)} perception metrics to evaluate")
        return results
    
    def analyze_localization(self) -> List[AnalysisResult]:
        """Analyze localization capabilities - Positional accuracy, sensor fusion."""
        print("Analyzing localization capabilities...")
        results = []
        
        # 1. Positional accuracy
        pos_accuracy_result = AnalysisResult(
            category="Localization",
            metric_name="positional_accuracy_m",
            current_value=0.0,  # Unknown - needs to be measured
            target_value=self.targets.positional_accuracy,
            status="NOT_MEASURED",
            confidence="NOT_AVAILABLE",
            notes="No current measurement available. Requires comparison with GPS + map data"
        )
        results.append(pos_accuracy_result)
        
        # 2. Sensor fusion robustness
        fusion_result = AnalysisResult(
            category="Localization", 
            metric_name="sensor_fusion_robustness",
            current_value=0.0,  # Unknown - needs to be measured
            target_value=self.targets.sensor_fusion_robustness,
            status="NOT_MEASURED",
            confidence="NOT_AVAILABLE",
            notes="No current measurement available. Requires testing during sensor failures"
        )
        results.append(fusion_result)
        
        print(f"  - Found {len(results)} localization metrics to evaluate")
        return results
    
    def analyze_path_planning(self) -> List[AnalysisResult]:
        """Analyze path planning capabilities - Route completion, trajectory smoothness, obstacle avoidance."""
        print("Analyzing path planning capabilities...")
        results = []
        
        # 1. Route completion rate
        route_completion_result = AnalysisResult(
            category="Path Planning",
            metric_name="route_completion_rate",
            current_value=0.0,  # Unknown - needs to be measured
            target_value=self.targets.route_completion_rate,
            status="NOT_MEASURED",
            confidence="NOT_AVAILABLE",
            notes="No current measurement available. Requires testing on 50+ test routes"
        )
        results.append(route_completion_result)
        
        # 2. Trajectory smoothness (jerk)
        jerk_result = AnalysisResult(
            category="Path Planning",
            metric_name="trajectory_smoothness_jerk",
            current_value=0.0,  # Unknown - needs to be measured
            target_value=self.targets.trajectory_smoothness,
            status="NOT_MEASURED",
            confidence="NOT_AVAILABLE",
            notes="No current measurement available. Requires jerk calculation"
        )
        results.append(jerk_result)
        
        # 3. Obstacle avoidance success
        obstacle_result = AnalysisResult(
            category="Path Planning",
            metric_name="obstacle_avoidance_success_rate",
            current_value=0.0,  # Unknown - needs to be measured
            target_value=self.targets.obstacle_avoidance_success,
            status="NOT_MEASURED",
            confidence="NOT_AVAILABLE",
            notes="No current measurement available. Requires testing in edge cases"
        )
        results.append(obstacle_result)
        
        print(f"  - Found {len(results)} path planning metrics to evaluate")
        return results
    
    def analyze_control_system(self) -> List[AnalysisResult]:
        """Analyze control system - Steering/braking latency, safety margins, fail-safe."""
        print("Analyzing control system capabilities...")
        results = []
        
        # 1. Steering/braking latency
        latency_result = AnalysisResult(
            category="Control System",
            metric_name="steering_braking_latency_ms",
            current_value=0.0,  # Unknown - needs to be measured
            target_value=self.targets.steering_braking_latency,
            status="NOT_MEASURED",
            confidence="NOT_AVAILABLE",
            notes="No current measurement available. Requires latency testing"
        )
        results.append(latency_result)
        
        # 2. Safety margin compliance
        safety_result = AnalysisResult(
            category="Control System",
            metric_name="safety_margin_compliance",
            current_value=0.0,  # Unknown - needs to be measured
            target_value=self.targets.safety_margin_compliance,
            status="NOT_MEASURED",
            confidence="NOT_AVAILABLE",
            notes="No current measurement available. Requires safety margin validation"
        )
        results.append(safety_result)
        
        # 3. Fail-safe behavior
        failsafe_result = AnalysisResult(
            category="Control System",
            metric_name="fail_safe_behavior_rate",
            current_value=0.0,  # Unknown - needs to be measured
            target_value=self.targets.fail_safe_behavior,
            status="NOT_MEASURED",
            confidence="NOT_AVAILABLE",
            notes="No current measurement available. Requires failure simulation testing"
        )
        results.append(failsafe_result)
        
        print(f"  - Found {len(results)} control system metrics to evaluate")
        return results
    
    def analyze_traffic_signals(self) -> List[AnalysisResult]:
        """Analyze traffic signal handling - DEC module accuracy, false stop rate."""
        print("Analyzing traffic signal handling capabilities...")
        results = []
        
        # 1. DEC module accuracy
        dec_result = AnalysisResult(
            category="Traffic Signal Handling",
            metric_name="dec_module_accuracy",
            current_value=0.0,  # Unknown - needs to be measured
            target_value=self.targets.dec_module_accuracy,
            status="NOT_MEASURED",
            confidence="NOT_AVAILABLE",
            notes="No current measurement available. Requires testing on 200+ scenarios"
        )
        results.append(dec_result)
        
        # 2. False stop rate
        false_stop_result = AnalysisResult(
            category="Traffic Signal Handling",
            metric_name="false_stop_rate",
            current_value=0.0,  # Unknown - needs to be measured
            target_value=self.targets.false_stop_rate,
            status="NOT_MEASURED",
            confidence="NOT_AVAILABLE",
            notes="No current measurement available. Requires testing to ensure <0.01% rate"
        )
        results.append(false_stop_result)
        
        print(f"  - Found {len(results)} traffic signal handling metrics to evaluate")
        return results
    
    def analyze_hardware_optimization(self) -> List[AnalysisResult]:
        """Analyze hardware optimization - CPU, RAM, power usage."""
        print("Analyzing hardware optimization...")
        results = []
        
        # Get current system resource usage
        current_cpu = psutil.cpu_percent(interval=1)
        current_memory = psutil.virtual_memory()
        current_ram_mb = current_memory.used / (1024 * 1024)
        
        # 1. CPU usage
        cpu_result = AnalysisResult(
            category="Hardware Optimization",
            metric_name="cpu_usage_percent",
            current_value=current_cpu,
            target_value=self.targets.cpu_usage_limit,
            status="PASS" if current_cpu < self.targets.cpu_usage_limit else "FAIL",
            confidence="HIGH",
            notes=f"Current CPU usage: {current_cpu:.2f}%"
        )
        results.append(cpu_result)
        
        # 2. RAM usage
        ram_result = AnalysisResult(
            category="Hardware Optimization",
            metric_name="ram_usage_mb",
            current_value=current_ram_mb,
            target_value=self.targets.ram_usage_limit,
            status="PASS" if current_ram_mb < self.targets.ram_usage_limit else "FAIL",
            confidence="HIGH",
            notes=f"Current RAM usage: {current_ram_mb:.2f} MB"
        )
        results.append(ram_result)
        
        # 3. Power draw (estimated)
        # Simplified power estimation based on CPU and RAM usage
        estimated_power = 2.0 + (current_cpu / 100.0) * 6.0 + (current_ram_mb / 2048.0) * 1.0
        power_result = AnalysisResult(
            category="Hardware Optimization",
            metric_name="power_draw_watts",
            current_value=estimated_power,
            target_value=self.targets.power_draw_limit,
            status="PASS" if estimated_power < self.targets.power_draw_limit else "FAIL",
            confidence="MEDIUM",
            notes=f"Estimated power draw: {estimated_power:.2f}W based on CPU/RAM usage"
        )
        results.append(power_result)
        
        print(f"  - Found {len(results)} hardware optimization metrics evaluated")
        return results
    
    def run_full_analysis(self) -> List[AnalysisResult]:
        """Run comprehensive analysis of all categories."""
        print("Starting comprehensive analysis of sunnypilot codebase...")
        print(f"Target platform: {self.hardware_info['platform']}")
        print(f"Hardware specs: {self.hardware_info['ram_gb']}GB RAM, {self.hardware_info['cpu_cores']} cores")
        print("="*80)
        
        all_results = []
        
        # Analyze each category
        all_results.extend(self.analyze_perception())
        all_results.extend(self.analyze_localization())
        all_results.extend(self.analyze_path_planning())
        all_results.extend(self.analyze_control_system())
        all_results.extend(self.analyze_traffic_signals())
        all_results.extend(self.analyze_hardware_optimization())
        
        self.results = all_results
        return all_results
    
    def generate_report(self) -> Dict[str, Any]:
        """Generate a detailed analysis report."""
        print("\n" + "="*80)
        print("COMPREHENSIVE ANALYSIS REPORT")
        print("="*80)
        
        # Group results by category
        categories = {}
        for result in self.results:
            if result.category not in categories:
                categories[result.category] = []
            categories[result.category].append(result)
        
        # Calculate category scores
        category_scores = {}
        total_score = 0.0
        total_weight = 0
        
        for category, results in categories.items():
            print(f"\n{category.upper()}:")
            print("-" * 50)
            
            category_weight = self._get_category_weight(category)
            category_points = 0
            total_possible = 0
            
            for result in results:
                print(f"  {result.metric_name}:")
                print(f"    Current: {result.current_value} | Target: {result.target_value}")
                print(f"    Status: {result.status} | Confidence: {result.confidence}")
                print(f"    Notes: {result.notes}")
                
                # Calculate points for this metric
                if result.status == "PASS":
                    category_points += 100  # Full points for passing
                elif result.status == "FAIL":
                    category_points += 0    # No points for failing
                else:  # NOT_MEASURED
                    category_points += 0    # No points if not measured
                
                total_possible += 100
            
            # Calculate weighted category score
            if total_possible > 0:
                category_score = (category_points / total_possible) * category_weight
            else:
                category_score = 0
                
            category_scores[category] = {
                "raw_score": category_points / max(total_possible, 1) if total_possible > 0 else 0,
                "weighted_score": category_score,
                "weight": category_weight,
                "metrics_count": len(results)
            }
            
            total_score += category_score
            total_weight += category_weight
            
            print(f"  Category Score: {category_score:.1f}% ({category_points}/{total_possible} @ {category_weight}%)")
        
        print("\n" + "="*80)
        print("SUMMARY:")
        print(f"Total Score: {total_score:.1f}% (Raw: {(sum(cs['raw_score'] for cs in category_scores.values()) * 100 / len(category_scores)):.1f}%)")
        print(f"Categories Evaluated: {len(categories)}")
        print(f"Metrics Tracked: {len(self.results)}")
        
        # Identify critical risks
        print("\nCRITICAL RISKS IDENTIFIED:")
        critical_risks = self._identify_critical_risks()
        for risk in critical_risks:
            print(f"  - {risk}")
        
        return {
            "total_score": total_score,
            "category_scores": category_scores,
            "results": [r.__dict__ for r in self.results],
            "critical_risks": critical_risks,
            "hardware_info": self.hardware_info
        }
    
    def _get_category_weight(self, category: str) -> int:
        """Get the weight percentage for each category."""
        weights = {
            "Perception": 30,
            "Localization": 15, 
            "Path Planning": 20,
            "Control System": 15,
            "Traffic Signal Handling": 10,
            "Hardware Optimization": 10
        }
        return weights.get(category, 0)
    
    def _identify_critical_risks(self) -> List[str]:
        """Identify critical risks based on analysis."""
        risks = []
        
        # Check for metrics that should have been measured but weren't
        not_measured = [r for r in self.results if r.status == "NOT_MEASURED"]
        if len(not_measured) > 0:
            risks.append(f"{len(not_measured)} critical metrics not measured - cannot assess safety/correctness")
        
        # Check for actual failures in measurable metrics
        failed_metrics = [r for r in self.results if r.status == "FAIL"]
        if len(failed_metrics) > 0:
            for metric in failed_metrics:
                risks.append(f"Hardware constraint violation: {metric.metric_name} ({metric.current_value} vs target {metric.target_value})")
        
        return risks

def main():
    """Main function to run the comprehensive analysis."""
    print("Sunnypilot Comprehensive Analysis Tool")
    print("=====================================")
    
    # Create analyzer and run analysis
    analyzer = SunnypilotAnalyzer()
    results = analyzer.run_full_analysis()
    
    # Generate report
    report = analyzer.generate_report()
    
    # Save report to file
    report_filename = f"comprehensive_analysis_report_{int(time.time())}.json"
    with open(report_filename, 'w') as f:
        json.dump(report, f, indent=2, default=str)
    
    print(f"\nReport saved to: {report_filename}")
    
    return report

if __name__ == "__main__":
    main()