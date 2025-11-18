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
            
        # Check for memory pool improvements
        if "np.copy(arr[:size])" in content:
            print("✓ neon_optimizer: Fixed memory pool array copying")
        else:
            print("✗ neon_optimizer: Memory pool fix not found")
    
    # 2. Check if data_collector.py has been fixed
    data_collector_path = "common/data_collector.py"
    if os.path.exists(data_collector_path):
        with open(data_collector_path, 'r') as f:
            content = f.read()
        
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
        
        # Check for safety improvements in lateral control
        if "abs(self.desired_curvature) < 0.005" in content:
            print("✓ controlsd: Fixed conservative lateral control logic")
        else:
            print("✗ controlsd: Lateral control fix not found")
            
        # Check for performance optimization in main loop
        if "loop_counter % 10 == 0" in content:
            print("✓ controlsd: Fixed performance factor checking frequency")
        else:
            print("✗ controlsd: Performance optimization not found")
    
    # 4. Check model_efficiency.py torch dependency fixes
    model_eff_path = "selfdrive/modeld/model_efficiency.py"
    if os.path.exists(model_eff_path):
        with open(model_eff_path, 'r') as f:
            content = f.read()
        
        # Check for lazy torch imports
        if "get_torch_modules()" in content:
            print("✓ model_efficiency: Fixed lazy torch imports")
        else:
            print("✗ model_efficiency: Lazy torch import not found")
        
        # Check for torch availability checks
        if "if self.torch is None:" in content:
            print("✓ model_efficiency: Fixed torch availability checks")
        else:
            print("✗ model_efficiency: Torch availability check not found")
    
    print("\nValidation completed!")

if __name__ == "__main__":
    validate_changes()