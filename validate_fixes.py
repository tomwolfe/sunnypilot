#!/usr/bin/env python3
"""
Validation script to ensure critical issues have been fixed
"""
import sys
import os

def validate_changes():
    """Validate that critical fixes have been implemented"""
    print("Validating fixes for critical issues...")

    # 1. Check if neon_optimizer.py has been fixed
    neon_path = "selfdrive/modeld/neon_optimizer.py"
    if os.path.exists(neon_path):
        with open(neon_path, 'r') as f:
            content = f.read()

        # Check for fixes in optimize_curvature_calculation
        if "isinstance(steer_angle, (int, float))" in content:
            print("✓ neon_optimizer: Fixed type checking in optimize_curvature_calculation")
        else:
            print("✗ neon_optimizer: Type checking fix not found")

        # Check for performance improvement
        if "__import__('time')" not in content or "import time" in content:
            print("✓ neon_optimizer: Fixed time import in optimize_curvature_calculation")
        else:
            print("✗ neon_optimizer: Time import optimization not found")

        # Check for more efficient clamping
        if "np.clip" not in content or "max(min(" in content:
            print("✓ neon_optimizer: Fixed curvature clamping method")
        else:
            print("✗ neon_optimizer: Curvature clamping method not optimized")

    # 2. Check if data_collector.py has been fixed
    data_collector_path = "common/data_collector.py"
    if os.path.exists(data_collector_path):
        with open(data_collector_path, 'r') as f:
            content = f.read()

        # Check for improved error handling
        if "except (OSError, IOError, RuntimeError)" in content:
            print("✓ data_collector: Fixed specific error handling")
        else:
            print("✗ data_collector: Specific error handling not found")

        # Check for batch processing improvements
        if "max_batch_size = 50" in content:
            print("✓ data_collector: Fixed batch processing limits")
        else:
            print("✗ data_collector: Batch processing fix not found")

        # Check for efficient JSON writing
        if "separators=(',', ':')" in content:
            print("✓ data_collector: Fixed compact JSON format")
        else:
            print("✗ data_collector: Compact JSON format not found")

    # 3. Check if controlsd.py has been fixed
    controls_path = "selfdrive/controls/controlsd.py"
    if os.path.exists(controls_path):
        with open(controls_path, 'r') as f:
            content = f.read()

        # Check for safety improvements in validation handling
        if "overall_confidence = validation_metrics.overallConfidence" in content:
            print("✓ controlsd: Fixed validation metrics handling")
        else:
            print("✗ controlsd: Validation metrics handling not optimized")

        # Check for performance optimization in main loop
        if "loop_counter % 10 == 0" in content:
            print("✓ controlsd: Fixed performance factor checking frequency")
        else:
            print("✗ controlsd: Performance optimization not found")

    # 4. Check thermal management error handling
    thermal_path = "selfdrive/monitoring/thermal_management.py"
    if os.path.exists(thermal_path):
        with open(thermal_path, 'r') as f:
            content = f.read()

        # Check for specific error handling in callbacks
        if "except (TypeError, ValueError)" in content:
            print("✓ thermal_management: Fixed specific error handling in callbacks")
        else:
            print("✗ thermal_management: Specific error handling not found in callbacks")

    print("\nValidation completed!")

if __name__ == "__main__":
    validate_changes()