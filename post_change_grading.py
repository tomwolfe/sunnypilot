"""
Post-Change Grading Validation
This completes Step 4: Post-Change Grading after implementing improvements
"""

import json
import time
from typing import Dict, Any

from comprehensive_analysis import ComprehensiveValidator
from pareto_analysis import ParetoAnalyzer
from implementation_execution import ImplementationExecutor


class PostChangeGrader:
    """
    Performs post-change grading validation as specified in Step 4 of the original prompt.
    """
    
    def __init__(self):
        self.comprehensive_validator = ComprehensiveValidator()
        self.pareto_analyzer = ParetoAnalyzer()
    
    def run_post_change_analysis(self) -> Dict[str, Any]:
        """Run comprehensive analysis after changes have been implemented."""
        print("Starting Post-Change Grading Analysis...")
        print("=" * 60)
        print("Step 4: Post-Change Grading (After implementing improvements from Steps 2 & 3)")
        print()
        
        # Run the comprehensive validation after improvements
        print("1. Running comprehensive validation after improvements...")
        post_change_results = self.comprehensive_validator.run_all_validations()
        
        # Compare with initial results if available
        print("2. Loading initial (baseline) results for comparison...")
        initial_results = self._load_initial_results()
        
        # Calculate the improvements
        print("3. Calculating improvements...")
        improvements = self._calculate_improvements(initial_results, post_change_results)
        
        # Generate the final report
        print("4. Generating final report...")
        report = self._generate_final_report(initial_results, post_change_results, improvements)
        
        # Check if all criteria are met
        criteria_met = self._check_completion_criteria(post_change_results)
        
        # Save the comprehensive report
        report["criteria_met"] = criteria_met
        filename = f"post_change_grading_report_{int(time.time())}.json"
        with open(filename, 'w') as f:
            json.dump(report, f, indent=2, default=str)
        
        print(f"\nPost-change grading report saved to: {filename}")
        
        # Output summary
        print("\n" + "=" * 60)
        print("POST-CHANGE GRADING SUMMARY")
        print("=" * 60)
        print(f"Overall Score: {report['total_weighted_score']:.1f}%")
        
        # Category scores
        for category, score in report['category_scores'].items():
            print(f"{category}: {score['weighted_score']:.1f}%")
        
        print(f"\nCriteria Met: {'✓' if criteria_met['all_met'] else '✗'}")
        print(f"- All categories ≥85%: {'✓' if criteria_met['categories_85'] else '✗'}")
        print(f"- Safety-critical issues resolved: {'✓' if criteria_met['safety_resolved'] else '✗'}")
        print(f"- Overall grade ≥95%: {'✓' if criteria_met['overall_95'] else '✗'}")
        
        if criteria_met['all_met']:
            print("\n🎉 SUCCESS: System meets all post-change criteria!")
        else:
            print(f"\n❌ IMPROVEMENT NEEDED: System does not meet all criteria.")
            print("Recommendation: Return to Step 1 for further analysis.")
        
        return report
    
    def _load_initial_results(self) -> Dict[str, Any]:
        """Load initial results from the baseline analysis."""
        # In a real implementation, this would load the actual baseline results
        # For this demo, we'll run the validator again to establish baseline
        print("   Establishing baseline results...")
        # We'll create a new validator instance to get independent results
        baseline_validator = ComprehensiveValidator()
        baseline_results = baseline_validator.run_all_validations()
        return baseline_results
    
    def _calculate_improvements(self, initial_results: Dict[str, Any], 
                              post_change_results: Dict[str, Any]) -> Dict[str, Any]:
        """Calculate improvements in each category."""
        improvements = {}
        
        categories = ['perception', 'localization', 'path_planning', 'control_system', 
                     'traffic_signals', 'hardware_optimization']
        
        for category in categories:
            for key, (new_value, details) in post_change_results[category].items():
                if key in initial_results[category]:
                    old_value, _ = initial_results[category][key]
                    improvement = new_value - old_value
                    improvements[f"{category}_{key}"] = {
                        "before": old_value,
                        "after": new_value,
                        "improvement": improvement,
                        "improvement_pct": (improvement / old_value * 100) if old_value != 0 else 0
                    }
        
        return improvements
    
    def _generate_final_report(self, initial_results: Dict[str, Any], 
                             post_change_results: Dict[str, Any], 
                             improvements: Dict[str, Any]) -> Dict[str, Any]:
        """Generate the final grading report."""
        # Get the summary information from the validator
        # We need to run the summary generation logic again for the updated results
        
        # Calculate category scores with weighted scoring as specified in the original prompt
        category_weights = {
            "Perception": 30,
            "Localization": 15,
            "Path Planning": 20,
            "Control System": 15,
            "Traffic Signal Handling": 10,
            "Hardware Optimization": 10
        }

        # Calculate category scores
        category_scores = {}

        # Perception (30% of total score)
        perception_result = post_change_results["perception"]["object_detection_accuracy"][0]
        latency_result = post_change_results["perception"]["frame_processing_latency"][0]
        fps_result = post_change_results["perception"]["false_positive_rate"][0]
        perception_coverage = post_change_results["perception"]["testing_coverage"][0]

        perception_score = 0
        perception_tests_passed = 0
        perception_total_tests = 4

        # Object detection accuracy (target: >= 0.95)
        if perception_result >= 0.95:
            perception_score += 10
            perception_tests_passed += 1
        else:
            # Proportional scoring for accuracy (max 10 points)
            perception_score += min(10, (perception_result / 0.95) * 10)

        # Frame processing latency (target: <= 50ms)
        if latency_result <= 50:
            perception_score += 10
            perception_tests_passed += 1
        else:
            # Inverse proportional for latency (more lenient than binary)
            latency_score = max(0, 10 * (1 - max(0, latency_result - 50) / 100))
            perception_score += latency_score

        # False positive rate (target: <= 0.001)
        if fps_result <= 0.001:
            perception_score += 10
            perception_tests_passed += 1
        else:
            # Inverse proportional for false positive rate
            fps_score = max(0, 10 * (1 - min(1, fps_result / 0.001)))
            perception_score += fps_score

        # Testing coverage
        coverage_points = 0
        if perception_coverage == 100.0:
            coverage_points = 3  # +10% of 30% weight = 3 points
            perception_tests_passed += 1
        elif perception_coverage < 80.0:
            coverage_points = -6  # -20% of 30% weight = 6 points
        else:
            # Proportional for coverage between 80-100%
            coverage_points = max(-6, min(3, (perception_coverage - 80) / 20 * 3))
        perception_score += coverage_points

        # Ensure perception score doesn't exceed 30 or go below 0
        perception_score = max(0, min(30, perception_score))

        category_scores["Perception"] = {
            "raw_score": perception_tests_passed/perception_total_tests,
            "weighted_score": perception_score,
            "tests_passed": perception_tests_passed,
            "total_tests": perception_total_tests,
            "sub_scores": {
                "accuracy": perception_result,
                "latency": latency_result,
                "false_positive_rate": fps_result,
                "testing_coverage": perception_coverage
            }
        }

        # Localization (15% of total score)
        pos_accuracy_result = post_change_results["localization"]["positional_accuracy"][0]
        fusion_result = post_change_results["localization"]["sensor_fusion_robustness"][0]
        localization_coverage = post_change_results["localization"]["testing_coverage"][0]

        localization_score = 0
        localization_tests_passed = 0
        localization_total_tests = 3

        # Positional accuracy (target: <= 1.0m)
        if pos_accuracy_result <= 1.0:
            localization_score += 8
            localization_tests_passed += 1
        else:
            # Proportional scoring for positional accuracy
            pos_score = max(0, 8 * (1 - min(1, pos_accuracy_result / 3.0)))
            localization_score += pos_score

        # Sensor fusion robustness (target: >= 0.95)
        if fusion_result >= 0.95:
            localization_score += 7
            localization_tests_passed += 1
        else:
            # Proportional scoring for fusion robustness
            fusion_score = max(0, 7 * (fusion_result / 0.95))
            localization_score += fusion_score

        # Testing coverage
        coverage_points = 0
        if localization_coverage == 100.0:
            coverage_points = 1.5  # +10% of 15% weight = 1.5 points
            localization_tests_passed += 1
        elif localization_coverage < 80.0:
            coverage_points = -3  # -20% of 15% weight = 3 points
        else:
            # Proportional for coverage between 80-100%
            coverage_points = max(-3, min(1.5, (localization_coverage - 80) / 20 * 1.5))
        localization_score += coverage_points

        # Ensure localization score doesn't exceed 15 or go below 0
        localization_score = max(0, min(15, localization_score))

        category_scores["Localization"] = {
            "raw_score": localization_tests_passed/localization_total_tests,
            "weighted_score": localization_score,
            "tests_passed": localization_tests_passed,
            "total_tests": localization_total_tests,
            "sub_scores": {
                "positional_accuracy": pos_accuracy_result,
                "sensor_fusion_robustness": fusion_result,
                "testing_coverage": localization_coverage
            }
        }

        # Path Planning (20% of total score)
        route_completion_result = post_change_results["path_planning"]["route_completion_rate"][0]
        trajectory_result = post_change_results["path_planning"]["trajectory_smoothness"][0]
        obstacle_result = post_change_results["path_planning"]["obstacle_avoidance_success"][0]
        path_planning_coverage = post_change_results["path_planning"]["testing_coverage"][0]

        path_planning_score = 0
        path_planning_tests_passed = 0
        path_planning_total_tests = 4

        # Route completion rate (target: >= 0.98)
        if route_completion_result >= 0.98:
            path_planning_score += 7
            path_planning_tests_passed += 1
        else:
            # Proportional scoring for route completion
            route_score = max(0, 7 * (route_completion_result / 0.98))
            path_planning_score += route_score

        # Trajectory smoothness (target: <= 3.0 m/s³)
        if trajectory_result <= 3.0:
            path_planning_score += 7
            path_planning_tests_passed += 1
        else:
            # Inverse proportional for trajectory smoothness
            traj_score = max(0, 7 * (1 - min(1, max(0, trajectory_result - 3.0) / 3.0)))
            path_planning_score += traj_score

        # Obstacle avoidance success (target: >= 0.99)
        if obstacle_result >= 0.99:
            path_planning_score += 6
            path_planning_tests_passed += 1
        else:
            # Proportional scoring for obstacle avoidance
            obstacle_score = max(0, 6 * (obstacle_result / 0.99))
            path_planning_score += obstacle_score

        # Testing coverage
        coverage_points = 0
        if path_planning_coverage == 100.0:
            coverage_points = 2  # +10% of 20% weight = 2 points
            path_planning_tests_passed += 1
        elif path_planning_coverage < 80.0:
            coverage_points = -4  # -20% of 20% weight = 4 points
        else:
            # Proportional for coverage between 80-100%
            coverage_points = max(-4, min(2, (path_planning_coverage - 80) / 20 * 2))
        path_planning_score += coverage_points

        # Ensure path planning score doesn't exceed 20 or go below 0
        path_planning_score = max(0, min(20, path_planning_score))

        category_scores["Path Planning"] = {
            "raw_score": path_planning_tests_passed/path_planning_total_tests,
            "weighted_score": path_planning_score,
            "tests_passed": path_planning_tests_passed,
            "total_tests": path_planning_total_tests,
            "sub_scores": {
                "route_completion_rate": route_completion_result,
                "trajectory_smoothness": trajectory_result,
                "obstacle_avoidance_success": obstacle_result,
                "testing_coverage": path_planning_coverage
            }
        }

        # Control System (15% of total score)
        latency_result = post_change_results["control_system"]["steering_braking_latency"][0]
        safety_result = post_change_results["control_system"]["safety_margin_compliance"][0]
        failsafe_result = post_change_results["control_system"]["fail_safe_behavior"][0]
        control_coverage = post_change_results["control_system"]["testing_coverage"][0]

        control_score = 0
        control_tests_passed = 0
        control_total_tests = 4

        # Steering/braking latency (target: <= 30ms)
        if latency_result <= 30:
            control_score += 6
            control_tests_passed += 1
        else:
            # Inverse proportional for latency
            latency_score = max(0, 6 * (1 - min(1, max(0, latency_result - 30) / 50)))
            control_score += latency_score

        # Safety margin compliance (target: >= 0.99)
        if safety_result >= 0.99:
            control_score += 5
            control_tests_passed += 1
        else:
            # Proportional scoring for safety compliance
            safety_score = max(0, 5 * (safety_result / 0.99))
            control_score += safety_score

        # Fail-safe behavior (target: >= 0.95)
        if failsafe_result >= 0.95:
            control_score += 4
            control_tests_passed += 1
        else:
            # Proportional scoring for fail-safe
            failsafe_score = max(0, 4 * (failsafe_result / 0.95))
            control_score += failsafe_score

        # Testing coverage
        coverage_points = 0
        if control_coverage == 100.0:
            coverage_points = 1.5  # +10% of 15% weight = 1.5 points
            control_tests_passed += 1
        elif control_coverage < 80.0:
            coverage_points = -3  # -20% of 15% weight = 3 points
        else:
            # Proportional for coverage between 80-100%
            coverage_points = max(-3, min(1.5, (control_coverage - 80) / 20 * 1.5))
        control_score += coverage_points

        # Apply critical safety failure cap
        if safety_result < 0.99:  # Critical safety failure
            control_score = min(control_score, 7.5)  # Cap at 50% of 15% max

        # Ensure control score doesn't exceed 15 or go below 0
        control_score = max(0, min(15, control_score))

        category_scores["Control System"] = {
            "raw_score": control_tests_passed/control_total_tests,
            "weighted_score": control_score,
            "tests_passed": control_tests_passed,
            "total_tests": control_total_tests,
            "sub_scores": {
                "steering_braking_latency": latency_result,
                "safety_margin_compliance": safety_result,
                "fail_safe_behavior": failsafe_result,
                "testing_coverage": control_coverage
            }
        }

        # Traffic Signal Handling (10% of total score)
        dec_result = post_change_results["traffic_signals"]["dec_module_accuracy"][0]
        false_stop_result = post_change_results["traffic_signals"]["false_stop_rate"][0]
        traffic_coverage = post_change_results["traffic_signals"]["testing_coverage"][0]

        traffic_score = 0
        traffic_tests_passed = 0
        traffic_total_tests = 3

        # DEC module accuracy (target: >= 0.995)
        if dec_result >= 0.995:
            traffic_score += 6
            traffic_tests_passed += 1
        else:
            # Proportional scoring for DEC accuracy
            dec_score = max(0, 6 * (dec_result / 0.995))
            traffic_score += dec_score

        # False stop rate (target: <= 0.0001)
        if false_stop_result <= 0.0001:
            traffic_score += 4
            traffic_tests_passed += 1
        else:
            # Inverse proportional for false stop rate
            stop_score = max(0, 4 * (1 - min(1, false_stop_result / 0.0001)))
            traffic_score += stop_score

        # Testing coverage
        coverage_points = 0
        if traffic_coverage == 100.0:
            coverage_points = 1  # +10% of 10% weight = 1 point
            traffic_tests_passed += 1
        elif traffic_coverage < 80.0:
            coverage_points = -2  # -20% of 10% weight = 2 points
        else:
            # Proportional for coverage between 80-100%
            coverage_points = max(-2, min(1, (traffic_coverage - 80) / 20 * 1))
        traffic_score += coverage_points

        # Apply critical safety failure cap
        if false_stop_result > 0.0001:  # Critical safety failure
            traffic_score = min(traffic_score, 5.0)  # Cap at 50% of 10% max

        # Ensure traffic score doesn't exceed 10 or go below 0
        traffic_score = max(0, min(10, traffic_score))

        category_scores["Traffic Signal Handling"] = {
            "raw_score": traffic_tests_passed/traffic_total_tests,
            "weighted_score": traffic_score,
            "tests_passed": traffic_tests_passed,
            "total_tests": traffic_total_tests,
            "sub_scores": {
                "dec_module_accuracy": dec_result,
                "false_stop_rate": false_stop_result,
                "testing_coverage": traffic_coverage
            }
        }

        # Hardware Optimization (10% of total score)
        cpu_result = post_change_results["hardware_optimization"]["cpu_usage"][0]
        ram_result = post_change_results["hardware_optimization"]["ram_usage"][0]
        power_result = post_change_results["hardware_optimization"]["power_draw"][0]

        hardware_score = 0
        hardware_tests_passed = 0
        hardware_total_tests = 3

        # CPU usage (target: < 35%)
        if cpu_result < 35.0:
            hardware_score += 4
            hardware_tests_passed += 1
        else:
            # Inverse proportional for CPU usage
            cpu_score = max(0, 4 * (1 - min(1, max(0, cpu_result - 35.0) / 35.0)))
            hardware_score += cpu_score

        # RAM usage (target: < 1433.6 MB)
        if ram_result < 1433.6:
            hardware_score += 3
            hardware_tests_passed += 1
        else:
            # Inverse proportional for RAM usage
            ram_score = max(0, 3 * (1 - min(1, max(0, ram_result - 1433.6) / 576.4)))
            hardware_score += ram_score

        # Power draw (target: < 8.0 W)
        if power_result < 8.0:
            hardware_score += 3
            hardware_tests_passed += 1
        else:
            # Inverse proportional for power draw
            power_score = max(0, 3 * (1 - min(1, max(0, power_result - 8.0) / 2.0)))
            hardware_score += power_score

        category_scores["Hardware Optimization"] = {
            "raw_score": hardware_tests_passed/hardware_total_tests,
            "weighted_score": (hardware_score/10) * 10,  # 10% weight
            "tests_passed": hardware_tests_passed,
            "total_tests": hardware_total_tests,
            "sub_scores": {
                "cpu_usage_percent": cpu_result,
                "ram_usage_mb": ram_result,
                "power_draw_watts": power_result
            }
        }

        # Calculate total weighted score
        total_weighted_score = sum(cat["weighted_score"] for cat in category_scores.values())

        # Apply resource violation deduction
        resource_violation = False
        if cpu_result >= 35.0:
            resource_violation = True
        if ram_result >= 1433.6:
            resource_violation = True
        if power_result >= 8.0:
            resource_violation = True

        if resource_violation:
            deduction = total_weighted_score * 0.15
            total_weighted_score -= deduction

        return {
            "timestamp": time.time(),
            "initial_results": initial_results,
            "post_change_results": post_change_results,
            "category_scores": category_scores,
            "total_weighted_score": total_weighted_score,
            "improvements": improvements,
            "detailed_metrics": post_change_results
        }
    
    def _check_completion_criteria(self, post_change_results: Dict[str, Any]) -> Dict[str, bool]:
        """Check if all required criteria from Step 4 are met."""
        category_scores = self._calculate_category_scores(post_change_results)
        
        # All categories score ≥85%
        categories_85 = all(score["weighted_score"] >= 85.0 for score in category_scores.values())
        
        # Safety-critical issues resolved (no false negatives for stop signs/traffic lights)
        # Check for safety-critical failures
        false_stop_rate = post_change_results["traffic_signals"]["false_stop_rate"][0]
        safety_margin_compliance = post_change_results["control_system"]["safety_margin_compliance"][0]
        
        safety_resolved = (false_stop_rate <= 0.0001) and (safety_margin_compliance >= 0.99)
        
        # Overall grade ≥95%
        overall_95 = self._calculate_total_score(post_change_results) >= 95.0
        
        all_met = categories_85 and safety_resolved and overall_95
        
        return {
            "all_met": all_met,
            "categories_85": categories_85,
            "safety_resolved": safety_resolved,
            "overall_95": overall_95
        }
    
    def _calculate_category_scores(self, results: Dict[str, Any]):
        """Calculate category scores for criteria checking."""
        # This is a simplified version that calculates scores similar to the validator
        category_scores = {}
        
        # Perception score calculation
        perception_result = results["perception"]["object_detection_accuracy"][0]
        latency_result = results["perception"]["frame_processing_latency"][0]
        fps_result = results["perception"]["false_positive_rate"][0]

        perception_score = 0
        if perception_result >= 0.95:
            perception_score += 10
        else:
            perception_score += min(10, (perception_result / 0.95) * 10)

        if latency_result <= 50:
            perception_score += 10
        else:
            latency_score = max(0, 10 * (1 - max(0, latency_result - 50) / 100))
            perception_score += latency_score

        if fps_result <= 0.001:
            perception_score += 10
        else:
            fps_score = max(0, 10 * (1 - min(1, fps_result / 0.001)))
            perception_score += fps_score

        # Ensure perception score doesn't exceed 30 or go below 0
        perception_score = max(0, min(30, perception_score))
        
        category_scores["Perception"] = {"weighted_score": perception_score}

        return category_scores
    
    def _calculate_total_score(self, results: Dict[str, Any]) -> float:
        """Calculate total weighted score."""
        category_scores = self._calculate_category_scores(results)
        total_weighted_score = sum(cat["weighted_score"] for cat in category_scores.values())
        
        # Apply resource violation deduction
        cpu_result = results["hardware_optimization"]["cpu_usage"][0]
        resource_violation = cpu_result >= 35.0
        
        if resource_violation:
            deduction = total_weighted_score * 0.15
            total_weighted_score -= deduction
            
        return total_weighted_score


def main():
    """Run the post-change grading validation."""
    print("Sunnypilot Post-Change Grading")
    print("==============================")
    print("Step 4: Post-Change Grading Validation")
    print()
    
    grader = PostChangeGrader()
    report = grader.run_post_change_analysis()
    
    print(f"\nPost-change grading completed!")
    print(f"Overall score: {report['total_weighted_score']:.1f}%")
    
    return report


if __name__ == "__main__":
    main()