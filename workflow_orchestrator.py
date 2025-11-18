"""
Complete Sunnypilot Improvement Workflow
This orchestrates the full analysis-execution-validation cycle as specified in the original prompt.
"""

import time
import json
from typing import Dict, Any

from pareto_analysis import ParetoAnalyzer
from implementation_execution import ImplementationExecutor
from post_change_grading import PostChangeGrader
from comprehensive_analysis import main as run_comprehensive_analysis # Import the main function

# Import configuration settings
from config import (
    ASSESSMENT_CATEGORIES_SCORE_THRESHOLD,
    ASSESSMENT_SAFETY_RESOLVED_REQUIRED,
    ASSESSMENT_OVERALL_GRADE_THRESHOLD
)


class SunnypilotImprovementWorkflow:
    """
    Orchestrates the complete improvement workflow:
    Step 1: Codebase Analysis & Grading (already available in comprehensive_analysis.py)
    Step 2: Pareto-Optimal Improvement Plan (implemented in pareto_analysis.py)
    Step 3: Execution (implemented in implementation_execution.py)
    Step 4: Post-Change Grading (implemented in post_change_grading.py)
    """

    def __init__(self):
        self.analyzer = ParetoAnalyzer()
        self.executor = ImplementationExecutor()
        self.grader = PostChangeGrader()
        self.workflow_results = {}

    def run_complete_workflow(self) -> Dict[str, Any]:
        """Run the complete improvement workflow."""
        print("SUNNYPILLOT COMPLETE IMPROVEMENT WORKFLOW")
        print("========================================")
        print("Following the exact workflow from the original prompt")
        print()

        # Step 1: Codebase Analysis & Grading (uses existing comprehensive_analysis.py)
        print("STEP 1: Codebase Analysis & Grading")
        print("-" * 35)
        print("Running comprehensive analysis framework...")
        try:
            analysis_results = run_comprehensive_analysis()
            print("✓ Comprehensive analysis framework executed.")
            step_1_status = "Completed"
        except Exception as e:
            print(f"ERROR in Step 1 (Codebase Analysis): {e}")
            analysis_results = {"error": str(e)}
            step_1_status = "Failed"

        # Step 2: Pareto-Optimal Improvement Plan
        print("\nSTEP 2: Pareto-Optimal Improvement Plan")
        print("-" * 40)
        print("Analyzing current state and identifying top 3 issues...")
        try:
            improvement_plan = self.analyzer.generate_improvement_plan()
            print("✓ Improvement plan generated with top 3 issues")
            step_2_status = "Completed"
        except Exception as e:
            print(f"ERROR in Step 2 (Improvement Plan): {e}")
            improvement_plan = {"error": str(e)}
            step_2_status = "Failed"

        # Step 3: Execution
        print("\nSTEP 3: Execution")
        print("-" * 17)
        print("Executing improvements based on analysis...")
        try:
            execution_results = self.executor.execute_improvements()
            print("✓ Improvements executed according to plan")
            step_3_status = "Completed"
        except Exception as e:
            print(f"ERROR in Step 3 (Execution): {e}")
            execution_results = {"error": str(e)}
            step_3_status = "Failed"

        # Step 4: Post-Change Grading
        print("\nSTEP 4: Post-Change Grading")
        print("-" * 27)
        print("Validating results after improvements...")
        try:
            grading_results = self.grader.run_post_change_analysis()
            print("✓ Post-change grading completed")
            step_4_status = "Completed"
        except Exception as e:
            print(f"ERROR in Step 4 (Post-Change Grading): {e}")
            grading_results = {"error": str(e)}
            step_4_status = "Failed"

        # Compile complete workflow results
        self.workflow_results = {
            "timestamp": time.time(),
            "step_1_analysis": {"status": step_1_status, "results": analysis_results},
            "step_2_improvement_plan": {"status": step_2_status, "results": improvement_plan},
            "step_3_execution_results": {"status": step_3_status, "results": execution_results},
            "step_4_post_change_grading": {"status": step_4_status, "results": grading_results},
            "workflow_summary": self._generate_workflow_summary(grading_results)
        }

        # Save complete results
        filename = f"complete_workflow_results_{int(time.time())}.json"
        with open(filename, 'w') as f:
            json.dump(self.workflow_results, f, indent=2, default=str)

        print(f"\nComplete workflow results saved to: {filename}")

        # Final assessment
        self._final_assessment(grading_results)

        return self.workflow_results

    def _generate_workflow_summary(self, grading_results: Dict[str, Any]) -> Dict[str, Any]:
        """Generate a summary of the complete workflow."""
        criteria_met = grading_results.get('criteria_met', {})

        return {
            "total_score": grading_results.get('total_weighted_score', 0),
            "improvement_success": criteria_met.get('all_met', False),
            "categories_met": criteria_met.get('categories_85', False),
            "safety_resolved": criteria_met.get('safety_resolved', False),
            "overall_grade_target_met": criteria_met.get('overall_95', False),
            "workflow_completed": True
        }

    def _final_assessment(self, grading_results: Dict[str, Any]):
        """Provide final assessment of the workflow."""
        criteria_met = grading_results.get('criteria_met', {})

        print("\n" + "=" * 60)
        print("FINAL WORKFLOW ASSESSMENT")
        print("=" * 60)

        print(f"Overall Score: {grading_results.get('total_weighted_score', 0):.1f}%")

        all_criteria_met = criteria_met.get('all_met', False)

        if all_criteria_met:
            print("🎉 SUCCESS: All criteria met! System is ready for deployment.")
            print(f"   - All categories score ≥{ASSESSMENT_CATEGORIES_SCORE_THRESHOLD}%")
            print(f"   - Safety-critical issues resolved: {ASSESSMENT_SAFETY_RESOLVED_REQUIRED}")
            print(f"   - Overall grade ≥{ASSESSMENT_OVERALL_GRADE_THRESHOLD}%")
            print()
            print("The sunnypilot system now meets the requirements for")
            print("full point-to-point autonomous driving on comma three hardware.")
        else:
            print("⚠️  IMPROVEMENT NEEDED: Not all criteria met")

            if not criteria_met.get('categories_85', True):
                print(f"   - Some categories have scores below {ASSESSMENT_CATEGORIES_SCORE_THRESHOLD}%")
            if ASSESSMENT_SAFETY_RESOLVED_REQUIRED and not criteria_met.get('safety_resolved', True):
                print("   - Safety-critical issues remain unresolved")
            if not criteria_met.get('overall_95', True):
                print(f"   - Overall grade is below {ASSESSMENT_OVERALL_GRADE_THRESHOLD}%")

            print()
            print("Recommendation: Return to Step 1 for further analysis and improvements.")


def main():
    """Run the complete sunnypilot improvement workflow."""
    print("Starting Complete Sunnypilot Improvement Workflow...")
    print()

    workflow = SunnypilotImprovementWorkflow()
    results = workflow.run_complete_workflow()

    print(f"\nWorkflow completed successfully!")
    print(f"Total workflow time: {time.time() - results['timestamp']:.2f} seconds")

    return results


if __name__ == "__main__":
    main()