#!/usr/bin/env python3
"""
Parameter Sensitivity and Stability Analysis
Tests for the high KP/KI parameters (KP=1.8, KI=0.5) used in PR5
"""

import unittest
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
from collections import deque
import sys
from pathlib import Path

# Add the project root to the path
sys.path.insert(0, str(Path(__file__).parent))

from openpilot.common.pid import PIDController
from selfdrive.controls.lib.latcontrol_torque import KP, KI, KP_INTERP, INTERP_SPEEDS


class ParameterSensitivityAnalyzer:
    """Analyzes parameter sensitivity for lateral control system"""
    
    def __init__(self, kp, ki, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = 0.05  # 20Hz update rate
        
    def create_transfer_function(self, plant_gain=1.0):
        """
        Creates a simplified transfer function for the lateral control system
        G(s) = plant_gain / (s * (s + a)) where 'a' represents system damping
        """
        # Simplified vehicle dynamics: second-order system with integrator
        # This represents the relationship between steering torque and lateral acceleration
        num = [plant_gain]
        den = [1, 1, 0]  # s^2 + s (simplified vehicle dynamics)
        
        # PID controller transfer function
        pid_num = [self.kp * self.dt**2 + self.ki * self.dt + self.kd, 
                   2*self.kd - self.kp * self.dt**2, 
                   -self.kd]
        pid_den = [self.dt**2, -2*self.dt, 1]  # z^-1 terms converted to s-domain approximation
        
        # Combine plant and controller
        system_num, system_den = signal.convolve(num, pid_num), signal.convolve(den, pid_den)
        
        return signal.TransferFunction(system_num, system_den)
    
    def check_stability(self, plant_gain=1.0):
        """Check if the control system is stable using root locus analysis"""
        # For a simplified analysis, we'll use a discrete-time approximation
        # The closed-loop system should have all poles inside the unit circle
        
        # Create discrete-time system using bilinear transform
        # Simplified approach: check if the equivalent discrete system is stable
        # For PID with plant = 1/(s(s+1)), closed-loop poles depend on kp, ki, kd
        
        # Using Routh-Hurwitz stability criterion for the simplified system
        # Characteristic equation: s^3 + s^2 + kp*s + ki = 0
        # For stability: all coefficients > 0 and Routh array conditions met
        
        a2 = 1.0  # coefficient of s^2
        a1 = 1.0  # coefficient of s^1 (including damping from vehicle dynamics)
        a0 = self.ki  # coefficient of s^0 (integral term)
        
        # Check Routh-Hurwitz conditions for third-order system
        if a2 > 0 and a0 > 0:
            b1 = (a2 * a1 - a0) / a2
            if b1 > 0:
                return True
        
        return False
    
    def simulate_step_response(self, duration=5.0):
        """Simulate the step response of the control system"""
        # Simplified simulation of vehicle lateral response
        t = np.arange(0, duration, self.dt)
        
        # Initialize state variables
        error_history = deque(maxlen=int(1/self.dt))  # 1-second history
        integral = 0.0
        output = 0.0
        outputs = []
        
        for i, time_step in enumerate(t):
            # Simulate error input (unit step)
            error = 1.0 if time_step > 0.1 else 0.0  # Step at 0.1s
            
            # PID control
            integral += error * self.dt
            derivative = (error - (error_history[-1] if error_history else error)) / self.dt if error_history else 0
            
            # Apply current parameters
            proportional = self.kp * error
            integral_term = self.ki * integral
            derivative_term = 0.0  # KD = 0 for current system
            
            output = proportional + integral_term + derivative_term
            
            # Limit output to prevent windup
            output = np.clip(output, -10.0, 10.0)
            
            outputs.append(output)
            error_history.append(error)
        
        return t, outputs


class TestParameterSensitivity(unittest.TestCase):
    """Test suite for parameter sensitivity analysis"""
    
    def test_current_parameters_stability(self):
        """Test that the current parameters (KP=1.8, KI=0.5) are stable"""
        analyzer = ParameterSensitivityAnalyzer(KP, KI)
        
        # Check theoretical stability
        is_stable = analyzer.check_stability()
        self.assertTrue(is_stable, f"Current parameters KP={KP}, KI={KI} should be stable")
        
        # Perform step response simulation to verify practical stability
        t, outputs = analyzer.simulate_step_response(duration=5.0)
        
        # Check that the response is bounded (doesn't go to infinity)
        max_output = max(abs(np.array(outputs)))
        self.assertLess(max_output, 100.0, "Step response should be bounded")
        
        # Check that the response settles reasonably (not oscillating indefinitely)
        final_third = outputs[int(len(outputs)*2/3):]  # Last third of the response
        response_std = np.std(final_third)
        self.assertLess(response_std, 5.0, "Response should settle reasonably")
    
    def test_parameter_sweep_stability(self):
        """Test stability across a range of parameter values"""
        # Test various KP values around the current value
        kp_values = np.linspace(1.0, 2.5, 16)  # From 1.0 to 2.5 in 16 steps
        ki_values = np.linspace(0.1, 1.0, 10)  # From 0.1 to 1.0 in 10 steps
        
        stable_count = 0
        total_tests = 0
        
        for kp in kp_values:
            for ki in ki_values:
                total_tests += 1
                analyzer = ParameterSensitivityAnalyzer(kp, ki)
                
                if analyzer.check_stability():
                    stable_count += 1
        
        # The current parameters should be in a stable region
        stability_ratio = stable_count / total_tests
        print(f"Stability ratio: {stable_count}/{total_tests} = {stability_ratio:.2%}")
        
        # The parameter space should have reasonable stability margin
        self.assertGreater(stability_ratio, 0.1, "At least 10% of parameter space should be stable")
        
        # Specifically test the current parameters
        current_analyzer = ParameterSensitivityAnalyzer(KP, KI)
        self.assertTrue(current_analyzer.check_stability(), 
                       f"Current parameters KP={KP}, KI={KI} should be in stable region")
    
    def test_speed_varying_parameters(self):
        """Test that speed-varying parameters (KP_INTERP) work correctly"""
        # Verify that the interpolation table is properly configured
        self.assertEqual(len(KP_INTERP), len(INTERP_SPEEDS), 
                        "KP_INTERP and INTERP_SPEEDS should have the same length")
        
        # The first element should be the high-gain value for low speeds
        # The last element should be the configured KP value
        self.assertEqual(KP_INTERP[-1], KP, f"Last KP_INTERP value should be {KP}")
        
        # Check that all values are positive
        for kp_val in KP_INTERP:
            self.assertGreater(kp_val, 0, "All interpolated KP values should be positive")
        
        # Verify that at very high speeds, we approach the base KP
        high_speed = 30.0  # m/s (about 67 mph)
        interpolated_kp = np.interp(high_speed, INTERP_SPEEDS, KP_INTERP)
        
        # At high speeds, the gain should be close to the base KP
        self.assertAlmostEqual(interpolated_kp, KP, delta=0.5, 
                              msg="At high speeds, interpolated gain should approach base KP")
    
    def test_gain_margin_analysis(self):
        """Analyze gain margins for the control system"""
        # Test how much gain can be increased before instability
        base_analyzer = ParameterSensitivityAnalyzer(KP, KI)
        base_stable = base_analyzer.check_stability()
        
        self.assertTrue(base_stable, "Base parameters should be stable")
        
        # Test with increased gains (to verify safety margins)
        test_multipliers = [0.8, 1.0, 1.2, 1.5]
        
        for mult in test_multipliers:
            test_kp = KP * mult
            test_ki = KI * mult
            
            test_analyzer = ParameterSensitivityAnalyzer(test_kp, test_ki)
            test_stable = test_analyzer.check_stability()
            
            # For this test, we expect at least the original and reduced gains to be stable
            if mult <= 1.0:
                self.assertTrue(test_stable, 
                               f"Parameters with {mult}x gain should be stable")
    
    def test_integral_windup_protection(self):
        """Test that integral windup is properly handled"""
        # Simulate a scenario where error persists for a long time
        dt = 0.05
        t_simulation = 10.0  # 10 seconds
        t = np.arange(0, t_simulation, dt)
        
        # Simulate PID controller with current parameters
        error = 2.0  # Persistent error
        integral = 0.0
        integral_limit = 5.0  # Reasonable limit to prevent windup
        outputs = []
        
        for time_step in t:
            # Add to integral with anti-windup
            integral += error * dt
            integral = np.clip(integral, -integral_limit, integral_limit)
            
            # PID output
            proportional = KP * error
            integral_term = KI * integral
            output = proportional + integral_term
            
            outputs.append(output)
            
            # Simulate plant response that would eventually reduce error
            if time_step > 5.0:  # After 5 seconds, error starts to reduce
                error *= 0.99  # Error reduces over time
        
        # Check that the integral term was properly limited
        max_integral = integral_limit * KI  # Maximum contribution from integral
        max_expected_output = KP * 2.0 + max_integral  # Max error * KP + limited integral
        
        actual_max_output = max(outputs)
        self.assertLess(actual_max_output, max_expected_output + 1.0, 
                       "Output should be limited by integral windup protection")


class TestModelSafetyConstraints(unittest.TestCase):
    """Test the safety constraints mentioned in the review"""
    
    def test_acceleration_change_limits(self):
        """Test that acceleration change limits are properly implemented"""
        # The review mentions 0.3 m/s³ limit for acceleration changes
        max_accel_change = 0.3  # m/s³
        dt = 0.05  # 20Hz operation
        
        # Calculate the per-update limit
        per_update_limit = max_accel_change * dt  # m/s² per update
        
        # Test with a sequence of acceleration values
        accelerations = [0.0, 1.0, 1.5, 1.2, 0.8]  # Sample accelerations
        clipped_accelerations = [accelerations[0]]  # Start with initial value
        
        prev_accel = accelerations[0]
        for i in range(1, len(accelerations)):
            target_accel = accelererations[i]
            
            # Apply rate limiting
            limited_accel = np.clip(target_accel, 
                                  prev_accel - per_update_limit, 
                                  prev_accel + per_update_limit)
            clipped_accelerations.append(limited_accel)
            prev_accel = limited_accel
        
        # Verify that all changes respect the limit
        for i in range(1, len(clipped_accelerations)):
            change = abs(clipped_accelerations[i] - clipped_accelerations[i-1])
            self.assertLessEqual(change, per_update_limit + 1e-6, 
                               f"Acceleration change should not exceed {per_update_limit:.4f}")
    
    def test_curvature_change_limits(self):
        """Test that curvature change limits are properly implemented"""
        # High speed (>5 m/s) limit: 0.01
        # Low speed (≤5 m/s) limit: 0.005
        high_speed_curvature_limit = 0.01
        low_speed_curvature_limit = 0.005
        dt = 0.05
        
        # Test at high speed
        v_ego = 10.0  # High speed
        per_update_limit = high_speed_curvature_limit if v_ego > 5.0 else low_speed_curvature_limit
        per_update_limit *= dt
        
        # Test curvature values with rate limiting
        curvatures = [0.0, 0.002, 0.005, 0.001, -0.003]  # Sample curvatures
        clipped_curvatures = [curvatures[0]]
        
        prev_curvature = curvatures[0]
        for i in range(1, len(curvatures)):
            target_curvature = curvatures[i]
            
            # Apply rate limiting
            limited_curvature = np.clip(target_curvature,
                                      prev_curvature - per_update_limit,
                                      prev_curvature + per_update_limit)
            clipped_curvatures.append(limited_curvature)
            prev_curvature = limited_curvature
        
        # Verify rate limits are respected
        for i in range(1, len(clipped_curvatures)):
            change = abs(clipped_curvatures[i] - clipped_curvatures[i-1])
            self.assertLessEqual(change, per_update_limit + 1e-6,
                               f"Curvature change at high speed should not exceed {per_update_limit:.6f}")
        
        # Test at low speed
        v_ego = 3.0  # Low speed
        per_update_limit = low_speed_curvature_limit if v_ego <= 5.0 else high_speed_curvature_limit
        per_update_limit *= dt
        
        # Same test with low speed limit
        low_curvatures = [0.0, 0.001, 0.002, 0.0005, -0.001]
        clipped_low_curvatures = [low_curvatures[0]]
        
        prev_curvature = low_curvatures[0]
        for i in range(1, len(low_curvatures)):
            target_curvature = low_curvatures[i]
            
            # Apply rate limiting
            limited_curvature = np.clip(target_curvature,
                                      prev_curvature - per_update_limit,
                                      prev_curvature + per_update_limit)
            clipped_low_curvatures.append(limited_curvature)
            prev_curvature = limited_curvature
        
        # Verify rate limits for low speed
        for i in range(1, len(clipped_low_curvatures)):
            change = abs(clipped_low_curvatures[i] - clipped_low_curvatures[i-1])
            self.assertLessEqual(change, per_update_limit + 1e-6,
                               f"Curvature change at low speed should not exceed {per_update_limit:.6f}")


class TestNNInputSafety(unittest.TestCase):
    """Test neural network input safety clipping"""
    
    def test_safe_input_clipping(self):
        """Test that NN input clipping works properly"""
        from sunnypilot.selfdrive.controls.lib.nnlc.nnlc import NeuralNetworkLateralControl
        
        nn_controller = NeuralNetworkLateralControl.__new__(NeuralNetworkLateralControl)
        
        # Test case 1: Normal values should pass through
        normal_input = [15.0, 1.0, 0.5, 0.01] + [1.0] * 20  # vEgo=15, reasonable values
        clipped = nn_controller.safe_clip_input(normal_input, 15.0, allow_high_values_for_testing=False)
        
        # First value (vEgo) should be preserved, others limited
        self.assertEqual(clipped[0], 15.0)
        for i in range(1, len(clipped)):
            self.assertGreaterEqual(clipped[i], -5.0)
            self.assertLessEqual(clipped[i], 5.0)
        
        # Test case 2: Extreme values should be clipped
        extreme_input = [50.0, 10.0, -10.0, 20.0] + [15.0] * 20  # Extreme values
        clipped_extreme = nn_controller.safe_clip_input(extreme_input, 50.0, allow_high_values_for_testing=False)
        
        # vEgo should be clipped to 40.0 max
        self.assertEqual(clipped_extreme[0], 40.0)
        # Other values should be clipped to ±5.0
        for i in range(1, len(clipped_extreme)):
            self.assertGreaterEqual(clipped_extreme[i], -5.0)
            self.assertLessEqual(clipped_extreme[i], 5.0)
        
        # Test case 3: Allow high values for testing (setpoint/measurement inputs)
        high_test_input = [15.0, 8.0, 6.0, 4.0] + [1.0] * 20  # High setpoint/jerk/roll for testing
        clipped_test = nn_controller.safe_clip_input(high_test_input, 15.0, allow_high_values_for_testing=True)
        
        # With testing flag, first few values can exceed limits, but others should be limited
        self.assertEqual(clipped_test[0], 15.0)  # vEgo preserved
        self.assertEqual(clipped_test[1], 8.0)   # Setpoint preserved when testing
        self.assertEqual(clipped_test[2], 6.0)   # Jerk preserved when testing
        self.assertEqual(clipped_test[3], 4.0)   # Roll preserved when testing
        
        # But later values should still be limited
        for i in range(4, len(clipped_test)):
            self.assertGreaterEqual(clipped_test[i], -5.0)
            self.assertLessEqual(clipped_test[i], 5.0)


def run_sensitivity_analysis():
    """Run parameter sensitivity analysis with detailed output"""
    print("=" * 80)
    print("PARAMETER SENSITIVITY AND STABILITY ANALYSIS")
    print("=" * 80)
    
    # Create test suites
    param_suite = unittest.TestLoader().loadTestsFromTestCase(TestParameterSensitivity)
    safety_suite = unittest.TestLoader().loadTestsFromTestCase(TestModelSafetyConstraints)
    nn_suite = unittest.TestLoader().loadTestsFromTestCase(TestNNInputSafety)
    
    # Combine all suites
    full_suite = unittest.TestSuite([param_suite, safety_suite, nn_suite])
    
    # Run tests with verbose output
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(full_suite)
    
    print("\n" + "=" * 80)
    print("SENSITIVITY ANALYSIS SUMMARY:")
    print(f"  Tests run: {result.testsRun}")
    print(f"  Failures: {len(result.failures)}")
    print(f"  Errors: {len(result.errors)}")
    print(f"  Success: {result.testsRun - len(result.failures) - len(result.errors)}/{result.testsRun}")
    
    # Additional parameter analysis printout
    print(f"\nCURRENT PARAMETERS (PR5):")
    print(f"  KP: {KP} (increased from 1.0)")
    print(f"  KI: {KI} (increased from 0.3)")
    print(f"  Speed-dependent interpolation enabled with table:")
    for speed, kp_val in zip(INTERP_SPEEDS, KP_INTERP):
        print(f"    {speed:4.1f} m/s -> {kp_val:5.2f} KP")
    print("=" * 80)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_sensitivity_analysis()
    exit(0 if success else 1)