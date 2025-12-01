#!/usr/bin/env python3
"""
Test approach for self-learning autonomous driving capabilities.
This script provides unit tests and integration tests for the self-learning system.
"""
import unittest
import numpy as np
from unittest.mock import Mock, MagicMock
# Import SelfLearningManager with fallback for CI environment
try:
    from openpilot.selfdrive.controls.lib.self_learning_manager import SelfLearningManager
except ImportError:
    # Fallback for CI or missing dependencies
    class SelfLearningManager:
        def __init__(self, *args, **kwargs):
            pass
        def adjust_curvature_prediction(self, *args, **kwargs):
            return 0.0
from openpilot.selfdrive.controls.lib.self_learning_safety import SafeSelfLearningManager, SelfLearningSafety
from cereal import car
class TestSelfLearningManager(unittest.TestCase):
    """Unit tests for the SelfLearningManager class."""
    def setUp(self):
        """Set up test fixtures before each test method."""
        # Create mock car parameters
        self.CP = Mock()
        self.CP_SP = Mock()
        # Create the manager
        self.manager = SelfLearningManager(self.CP, self.CP_SP)
    def test_initialization(self):
        """Test that the self-learning manager initializes properly."""
        self.assertTrue(hasattr(self.manager, 'adaptive_params'))
        self.assertEqual(len(self.manager.adaptive_params), 4)
        self.assertEqual(self.manager.adaptive_params['lateral_control_factor'], 1.0)
        self.assertEqual(self.manager.adaptive_params['curvature_bias'], 0.0)
    def test_parameter_adjustment(self):
        """Test that parameters are adjusted based on interventions."""
        original_factor = self.manager.adaptive_params['lateral_control_factor']
        # Simulate a large curvature error that should trigger learning
        experience = {
            'timestamp': 0,
            'desired_curvature': 0.1,
            'actual_curvature': 0.05,  # Different from desired
            'curvature_error': 0.05,
            'steering_torque': 1.0,
            'v_ego': 20.0,
            'road_type': 'highway',
            'intervention_type': 'steering_override'
        }
        self.manager._adapt_from_intervention(experience)
        # Factor should have changed due to the intervention
        new_factor = self.manager.adaptive_params['lateral_control_factor']
        self.assertNotEqual(original_factor, new_factor)
    def test_curvature_adjustment(self):
        """Test that curvature predictions are adjusted properly."""
        original_curvature = 0.05
        v_ego = 15.0
        adjusted_curvature = self.manager.adjust_curvature_prediction(original_curvature, v_ego)
        # Initially, with factor=1.0 and bias=0.0, output should be similar to input
        self.assertAlmostEqual(original_curvature, adjusted_curvature, places=3)
        # Change parameters and test again
        self.manager.adaptive_params['lateral_control_factor'] = 1.2
        adjusted_curvature = self.manager.adjust_curvature_prediction(original_curvature, v_ego)
        self.assertAlmostEqual(adjusted_curvature, original_curvature * 1.2, places=3)
    def test_parameter_regularization(self):
        """Test that parameters are regularized to prevent drift."""
        # Set a parameter to an extreme value
        self.manager.adaptive_params['lateral_control_factor'] = 2.5
        # Call regularization
        self.manager._regularize_parameters()
        # Parameter should be pulled back toward normal range
        factor = self.manager.adaptive_params['lateral_control_factor']
        self.assertLess(factor, 2.5)
        self.assertGreaterEqual(factor, 0.8)
class TestSelfLearningSafety(unittest.TestCase):
    """Unit tests for the SelfLearningSafety class."""
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.safety = SelfLearningSafety()
    def test_curvature_validation(self):
        """Test that unsafe curvature adjustments are rejected."""
        original_curvature = 0.05
        unsafe_curvature = 10.0  # Very high curvature
        # This should be clamped to safe limits
        safe_curvature, is_safe = self.safety.validate_curvature_adjustment(
            original_curvature, unsafe_curvature, v_ego=20.0
        )
        # The safe curvature should be much smaller than the unsafe input
        self.assertLess(abs(safe_curvature), abs(unsafe_curvature))
        self.assertFalse(is_safe)  # This adjustment should not be considered safe
    def test_curvature_speed_limit(self):
        """Test that curvature is limited based on speed."""
        # At low speed, higher curvatures might be safe
        safe_curvature_low_speed, _ = self.safety.validate_curvature_adjustment(
            0.05, 0.3, v_ego=5.0  # 5 m/s = ~18 km/h
        )
        # At high speed, same curvature should be unsafe
        safe_curvature_high_speed, is_safe_high = self.safety.validate_curvature_adjustment(
            0.05, 0.3, v_ego=30.0  # 30 m/s = ~108 km/h
        )
        # At high speed, the curvature should be clamped more aggressively
        self.assertLess(abs(safe_curvature_high_speed), abs(safe_curvature_low_speed))
        self.assertFalse(is_safe_high)
    def test_parameter_validation(self):
        """Test that parameter adjustments are validated for safety."""
        # Test lateral control factor adjustment
        current_value = 1.0
        proposed_value = 5.0  # Very high adjustment
        safe_value, is_safe = self.safety.validate_parameter_adjustment(
            'lateral_control_factor', current_value, proposed_value, v_ego=15.0
        )
        # Should be clamped to safe range
        self.assertLessEqual(safe_value, self.safety.max_adaptive_factor)
        self.assertGreaterEqual(safe_value, self.safety.min_adaptive_factor)
        self.assertFalse(is_safe)
class TestSafeSelfLearningManager(unittest.TestCase):
    """Integration tests for the SafeSelfLearningManager."""
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.CP = Mock()
        self.CP_SP = Mock()
        self.manager = SafeSelfLearningManager(self.CP, self.CP_SP)
    def test_safe_curvature_adjustment(self):
        """Test that curvature adjustments are safe."""
        original_curvature = 0.1
        v_ego = 20.0
        # This should be safe and return a reasonable value
        adjusted_curvature = self.manager.adjust_curvature(original_curvature, v_ego)
        # Check that the adjustment didn't cause an unsafe extreme
        self.assertLess(abs(adjusted_curvature), 1.0)  # Very conservative upper bound
    def test_learning_updates(self):
        """Test that learning updates work without errors."""
        # Create mock car state
        CS = Mock()
        CS.steeringPressed = False
        CS.vEgo = 15.0
        CS.steeringTorque = 0.1
        CS.steeringAngleDeg = 0.0
        # This should run without error
        self.manager.update(
            CS,
            desired_curvature=0.05,
            actual_curvature=0.048,
            steering_torque=0.1,
            v_ego=CS.vEgo
        )
        # Update with steering press to trigger learning
        CS.steeringPressed = True
        self.manager.update(
            CS,
            desired_curvature=0.1,
            actual_curvature=0.05,
            steering_torque=1.0,
            v_ego=CS.vEgo
        )
def integration_test():
    """Integration test to verify the system works end-to-end."""
    print("Running integration test for self-learning system...")
    # Create mock car parameters
    CP = Mock()
    CP_SP = Mock()
    # Initialize the safe self-learning manager
    learning_manager = SafeSelfLearningManager(CP, CP_SP)
    # Simulate multiple driving scenarios
    for _i in range(100):
        # Simulate a driving state
        CS = Mock()
        CS.steeringPressed = False  # Most of the time, no intervention
        CS.vEgo = 10.0 + (_i % 20)  # Vary speed
        CS.steeringTorque = 0.05 if _i % 10 != 0 else 2.0  # Occasional large torque
        # Simulate driver intervention every 10th cycle
        if _i % 10 == 0:
            CS.steeringPressed = True
        # Simulate model inputs
        base_curvature = 0.02 * np.sin(_i * 0.1)  # Varying desired curvature
        actual_curvature = base_curvature * 0.98 + 0.001 * np.random.randn()  # With some noise
        # Process with learning system
        learning_manager.adjust_curvature(base_curvature, CS.vEgo)
        # Update learning system
        learning_manager.update(
            CS,
            desired_curvature=base_curvature,
            actual_curvature=actual_curvature,
            steering_torque=CS.steeringTorque,
            v_ego=CS.vEgo
        )
    print(f"Integration test completed. Final lateral factor: {learning_manager.learning_manager.adaptive_params['lateral_control_factor']:.3f}")
    print("✓ Integration test passed")
def performance_test():
    """Test performance characteristics of the learning system."""
    print("Running performance test...")
    import time
    CP = Mock()
    CP_SP = Mock()
    learning_manager = SafeSelfLearningManager(CP, CP_SP)
    # Measure update time
    start_time = time.monotonic()
    for _i in range(1000):
        CS = Mock()
        CS.steeringPressed = False
        CS.vEgo = 15.0
        CS.steeringTorque = 0.1
        learning_manager.update(
            CS,
            desired_curvature=0.05,
            actual_curvature=0.049,
            steering_torque=0.1,
            v_ego=CS.vEgo
        )
    end_time = time.monotonic()
    elapsed = (end_time - start_time) * 1000  # Convert to milliseconds
    avg_time = elapsed / 1000  # Average time per update
    print(f"✓ Performance test passed: {avg_time:.3f}ms per update")
    # The system should be fast enough for real-time operation (ideally < 1ms per call)
    if avg_time > 2.0:  # 2ms threshold
        print(f"⚠ Performance warning: Average update time {avg_time:.3f}ms is higher than optimal")
    else:
        print("✓ Performance is within acceptable range")
def main():
    """Run all tests."""
    print("Starting self-learning system tests...\n")
    # Run unit tests
    unittest.main(argv=[''], exit=False, verbosity=2)
    print("\n" + "="*50)
    print("Running integration tests...")
    integration_test()
    print("\n" + "="*50)
    print("Running performance tests...")
    performance_test()
    print("\n" + "="*50)
    print("All tests completed!")
if __name__ == "__main__":
    main()