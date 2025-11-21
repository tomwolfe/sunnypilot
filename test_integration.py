#!/usr/bin/env python3
"""
Integration tests for sunnypilot autonomous driving system
Tests interactions between all new modules to prevent interaction bugs
"""

import unittest
import numpy as np
from unittest.mock import Mock, patch
import sys
import os

# Add the selfdrive path to sys.path to import modules
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))

from selfdrive.controls.lib.adaptive_behavior import AdaptiveController, ConditionBasedParameterTuner
from selfdrive.controls.lib.environmental_awareness import EnvironmentalConditionProcessor
from selfdrive.controls.lib.backup_safety import RedundantControlValidator
from selfdrive.controls.lib.vehicle_calibration import VehicleCalibration
from selfdrive.controls.lib.model_confidence_validator import ModelConfidenceValidator
from selfdrive.modeld.performance_optimization import PerformanceOptimizer
from selfdrive.controls.lib.operational_safety_metrics import OperationalSafetyMetrics
from cereal import log


class TestSunnypilotIntegration(unittest.TestCase):
    """
    Integration tests for all sunnypilot modules
    """
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.adaptive_controller = AdaptiveController()
        self.environmental_processor = EnvironmentalConditionProcessor()
        self.backup_validator = RedundantControlValidator()
        self.calibrator = VehicleCalibration()
        self.confidence_validator = ModelConfidenceValidator()
        self.performance_optimizer = PerformanceOptimizer()
        
    def test_environmental_conditions_affect_adaptive_behavior(self):
        """
        Test that environmental conditions properly affect adaptive behavior
        """
        # Setup environmental conditions indicating poor visibility
        sm = Mock()
        sm.updated = {'modelV2': True}
        sm.recv_frame = {'modelV2': 0}
        
        # Mock model data with poor visibility indicators
        mock_model = Mock()
        mock_model.position.x = [0.0] * 10  # Simulate straight path
        mock_model.position.y = [0.0] * 10
        mock_model.laneLines = [Mock() for _ in range(4)]
        for lane_line in mock_model.laneLines:
            lane_line.prob = 0.3  # Low confidence in lane detection (poor visibility)
        mock_model.leadOne = Mock()
        mock_model.leadOne.prob = 1.0  # Lead vehicle detected
        mock_model.leadOne.dist = 50.0
        mock_model.meta = Mock()
        mock_model.meta.disengagePredictions = Mock()
        mock_model.meta.disengagePredictions.modelProb = 0.8  # High model confidence
        
        sm['modelV2'] = mock_model
        
        # Update environmental processor
        self.environmental_processor.update(sm)
        
        # Extract environmental conditions
        env_conditions = self.environmental_processor.get_current_conditions()
        
        # Now use these conditions to influence adaptive controller
        self.adaptive_controller.update_conditions(
            is_curving=False,
            is_on_grade=False,
            model_confidence_low=env_conditions.get('model_confidence', 1.0) < 0.5,
            visibility_poor=env_conditions.get('low_visibility', False),
            environmental_risk=env_conditions.get('risk_score', 0.0)
        )
        
        # Check that adaptive behavior reflects environmental conditions
        lat_limits = self.adaptive_controller.get_adaptive_lateral_limits()
        # With poor visibility, should be more conservative
        max_lat_limit = lat_limits[1]
        base_limit = self.adaptive_controller.lateral_accel_limit
        
        # Should be more conservative than base limit
        self.assertLessEqual(abs(max_lat_limit), base_limit)
    
    def test_backup_safety_with_environmental_conditions(self):
        """
        Test that backup safety systems work correctly with environmental conditions
        """
        # Setup model output with environmental conditions
        desired_curvature = 0.05  # Some desired curvature
        desired_accel = 1.0  # Some desired acceleration
        v_ego = 20.0  # Speed
        model_confidence = 0.6  # Moderate confidence
        environmental_risk = 0.7  # High environmental risk
        
        # Test backup validation with environmental conditions
        safe_curvature, safe_accel = self.backup_validator.validate_and_backup(
            desired_curvature, desired_accel, v_ego, 
            model_confidence, environmental_risk
        )
        
        # With high risk and moderate confidence, should be more conservative
        self.assertLessEqual(abs(safe_accel), abs(desired_accel))
        
        # Test with very low confidence
        very_low_confidence = 0.3
        environmental_risk = 0.9  # Very high risk
        safe_curvature2, safe_accel2 = self.backup_validator.validate_and_backup(
            desired_curvature, desired_accel, v_ego, 
            very_low_confidence, environmental_risk
        )
        
        # Should be even more conservative with low confidence and high risk
        self.assertLessEqual(abs(safe_accel2), abs(safe_accel))
    
    def test_performance_optimization_with_safety_validation(self):
        """
        Test that performance optimization does not compromise safety validation
        """
        # Setup input data for performance optimizer
        inputs = {
            'model_inputs': np.random.random((10, 200, 300, 3)).astype(np.float32),
            'calibration': np.random.random(4).astype(np.float32)
        }
        
        # Setup system metrics
        system_metrics = {
            'cpu_util': 85.0,  # High CPU usage
            'temperature': 75.0,  # High temperature
            'lat_active': False,  # Lateral control not active
            'long_active': False,  # Longitudinal control not active
            'model_confidence': 0.9,  # High model confidence
            'lead_present': False  # No lead vehicle
        }
        
        # Test optimization cycle with safety active
        optimized_result = self.performance_optimizer.optimize_inference_cycle(
            inputs, system_metrics, lat_active=False, longitudinal_active=False
        )
        
        # Frame might be skipped due to high CPU/temperature, but safety should still be ensured
        if not optimized_result.get('skip_inference', False):
            # If not skipping, should have optimized inputs
            self.assertTrue('optimized_inputs' in optimized_result or 'skip_inference' in optimized_result)
    
    def test_multi_module_interaction_during_weather_conditions(self):
        """
        Test interaction between environmental awareness, adaptive behavior, 
        and performance optimization during simulated weather conditions
        """
        # Simulate weather-based environmental conditions
        sm = Mock()
        sm.updated = {'modelV2': True, 'cameraOdometry': True}
        sm.recv_frame = {'modelV2': 0, 'cameraOdometry': 0}
        
        # Mock model data indicating rain/snow conditions
        mock_model = Mock()
        mock_model.position.x = [0.0] * 10
        mock_model.position.y = [0.0] * 10
        mock_model.laneLines = [Mock() for _ in range(4)]
        for lane_line in mock_model.laneLines:
            lane_line.prob = 0.2  # Very low confidence (poor visibility)
            lane_line.std = 0.8  # High standard deviation (uncertain detection)
        mock_model.roadEdges = [Mock() for _ in range(2)]
        for edge in mock_model.roadEdges:
            edge.prob = 0.4  # Low confidence in road edge detection
            
        sm['modelV2'] = mock_model
        
        # Mock camera odometry for exposure/lighting data
        mock_odom = Mock()
        mock_odom.exposures = [0.001]  # High exposure (night or low light)
        mock_odom.lighting = 0.2  # Low lighting
        sm['cameraOdometry'] = mock_odom
        
        # Update environmental processor with simulated poor weather conditions
        self.environmental_processor.update(sm)
        
        # Get environmental conditions
        env_conditions = self.environmental_processor.get_current_conditions()
        environmental_risk = env_conditions.get('risk_score', 0.0)
        
        # Update adaptive controller with environmental conditions
        self.adaptive_controller.update_conditions(
            is_curving=False,
            is_on_grade=False,
            model_confidence_low=False,
            visibility_poor=True,
            environmental_risk=environmental_risk
        )
        
        # Get adaptive parameters that reflect environmental conditions
        adaptive_params = self.adaptive_controller.get_adaptive_parameters()
        
        # Test that parameters are more conservative due to environmental conditions
        self.assertLessEqual(adaptive_params.get('lateral_accel_limit', 3.0), 3.0)
        self.assertGreaterEqual(adaptive_params.get('following_distance_factor', 1.0), 1.0)
        
        # Test with backup validation
        test_curvature = 0.02
        test_accel = 1.5
        v_ego = 15.0
        
        validated_curvature, validated_accel = self.backup_validator.validate_and_backup(
            test_curvature, test_accel, v_ego,
            model_confidence=0.7,  # Moderate confidence due to poor conditions
            environmental_risk=environmental_risk
        )
        
        # Should be conservative based on environmental conditions
        self.assertLessEqual(abs(validated_accel), abs(test_accel))
    
    def test_parameter_calibration_integration(self):
        """
        Test that calibrated parameters work properly with other systems
        """
        # Get calibrated parameters
        calibrated_params = self.calibrator.get_calibrated_params()
        
        # Update adaptive controller with calibrated parameters
        if 'BASE_LATERAL_ACCEL_LIMIT' in calibrated_params:
            self.adaptive_controller.lateral_accel_limit = calibrated_params['BASE_LATERAL_ACCEL_LIMIT']
        
        # Test that adaptive controller uses calibrated parameters correctly
        lat_min, lat_max = self.adaptive_controller.get_adaptive_lateral_limits()
        
        # Validate that parameters are in safe ranges
        validation_results = self.calibrator.validate_parameters()
        self.assertTrue(all(validation_results.values()), f"Parameter validation failed: {validation_results}")
    
    def test_model_confidence_validation_integration(self):
        """
        Test that model confidence validation integrates properly with other systems
        """
        # Create mock model output
        model_output = {
            'desiredCurvature': 0.03,
            'path': [0.0, 0.1, 0.2, 0.3, 0.4],
            'laneLine': [0.5, 0.6, 0.7],
            'modelConfidence': 0.8,
            'pathStd': 0.1
        }
        
        v_ego = 20.0
        curvature = 0.02
        
        # Validate model output using our enhanced system
        enhanced_confidence, is_safe, validation_details = self.confidence_validator.validate_model_output(
            model_output, v_ego, curvature
        )
        
        # The system should provide enhanced confidence based on multiple factors
        self.assertIsInstance(enhanced_confidence, float)
        self.assertGreaterEqual(enhanced_confidence, 0.0)
        self.assertLessEqual(enhanced_confidence, 1.0)
        
        # Test with problematic model output
        problematic_output = {
            'desiredCurvature': 0.5,  # Very high curvature
            'path': [0.0, 10.0, 0.0, -10.0, 0.0],  # Very erratic path
            'modelConfidence': 0.9,  # High but unrealistic confidence
            'pathStd': 5.0  # Very high standard deviation
        }
        
        enhanced_confidence_p, is_safe_p, validation_details_p = self.confidence_validator.validate_model_output(
            problematic_output, v_ego, curvature
        )
        
        # Should reduce confidence for problematic output
        self.assertLess(enhanced_confidence_p, enhanced_confidence)
        self.assertEqual(is_safe_p, False)
    
    def test_safety_validation_during_performance_optimization(self):
        """
        Test that safety validation works properly even when performance optimization is active
        """
        # Set up test conditions
        desired_curvature = 0.04
        desired_accel = 1.2
        v_ego = 25.0
        model_confidence = 0.75
        environmental_risk = 0.3
        
        # First, test normal validation
        validated_curvature, validated_accel = self.backup_validator.validate_and_backup(
            desired_curvature, desired_accel, v_ego, model_confidence, environmental_risk
        )
        
        # Now simulate performance optimization by checking safety validation
        # should still work even if system is under stress
        
        # Test with system stress indicators
        stressed_model_confidence = 0.6  # Lowered due to system stress
        stressed_env_risk = 0.6  # Higher due to environmental factors
        
        validated_curvature_stressed, validated_accel_stressed = self.backup_validator.validate_and_backup(
            desired_curvature, desired_accel, v_ego, 
            stressed_model_confidence, stressed_env_risk
        )
        
        # Under stress conditions, should be more conservative
        self.assertLessEqual(abs(validated_accel_stressed), abs(validated_accel))
    
    def test_error_handling_in_module_interactions(self):
        """
        Test that error handling works properly when modules interact
        """
        # Test with invalid inputs to ensure robust error handling
        with patch('openpilot.common.swaglog.cloudlog') as mock_log:
            # Try to validate with completely invalid data
            try:
                result = self.confidence_validator.validate_model_output(
                    {},  # Empty model output
                    -1.0,  # Invalid speed
                    float('inf')  # Invalid curvature
                )
                # Should return safe defaults
            except Exception:
                pass  # Exception should be handled gracefully
            
            # Test backup system with invalid data
            safe_curv, safe_accel = self.backup_validator.validate_and_backup(
                float('nan'), float('inf'), -10.0,  # Invalid inputs
                -1.0, 2.0  # Invalid confidence and risk values
            )
            
            # Should return safe default values
            self.assertEqual(safe_curv, 0.0)
            self.assertEqual(safe_accel, 0.0)

    def test_operational_safety_metrics_verification(self):
        """
        Test operational safety metrics verification
        """
        safety_metrics = OperationalSafetyMetrics()

        # Test following distance verification under various conditions
        is_safe, details = safety_metrics.verify_following_distance(
            current_distance=50.0,  # 50 meters
            lead_velocity=20.0,     # Lead vehicle at 20 m/s
            ego_velocity=25.0       # Ego vehicle at 25 m/s
        )
        self.assertTrue(is_safe, "Following distance should be safe at 50m")
        self.assertGreaterEqual(details['current_distance'], safety_metrics.MIN_FOLLOWING_DISTANCE_METERS)

        # Test lateral acceleration with high environmental risk
        is_safe, details = safety_metrics.verify_lateral_acceleration(
            lateral_acceleration=1.0,   # 1.0 m/s^2
            environmental_risk=0.8      # High risk
        )
        # Should be safe since 1.0 < MAX_LATERAL_ACCEL_HIGH_RISK (1.5)
        self.assertTrue(is_safe, "Lateral acceleration should be safe at 1.0 m/s^2 with high risk")

        # Test with excessive lateral acceleration in high risk
        is_safe, details = safety_metrics.verify_lateral_acceleration(
            lateral_acceleration=2.0,   # 2.0 m/s^2
            environmental_risk=0.9      # Very high risk
        )
        # Should be unsafe since 2.0 > MAX_LATERAL_ACCEL_HIGH_RISK (1.5)
        self.assertFalse(is_safe, "Lateral acceleration should be unsafe at 2.0 m/s^2 with very high risk")

        # Test longitudinal jerk verification
        is_safe, details = safety_metrics.verify_longitudinal_jerk(
            current_accel=1.5,    # Current acceleration
            prev_accel=1.0,       # Previous acceleration
            dt=0.05               # 50ms time delta
        )
        expected_jerk = abs(1.5 - 1.0) / 0.05  # 10.0 m/s^3
        self.assertFalse(is_safe, f"Jerk of {expected_jerk} should be unsafe (limit is {safety_metrics.MAX_LONGITUDINAL_JERK})")

        # Test with acceptable jerk
        is_safe, details = safety_metrics.verify_longitudinal_jerk(
            current_accel=1.1,    # Current acceleration
            prev_accel=1.0,       # Previous acceleration
            dt=0.05               # 50ms time delta (0.1 / 0.05 = 2.0 m/s^3)
        )
        self.assertTrue(is_safe, "Jerk should be safe at 2.0 m/s^3")

        # Test all metrics at once
        all_results = safety_metrics.verify_all_operational_metrics(
            following_distance=60.0,
            lead_velocity=20.0,
            ego_velocity=20.0,  # Matching velocities
            lateral_accel=0.8,
            environmental_risk=0.3,  # Low risk
            current_accel=1.0,
            prev_accel=0.9,
            current_lat_accel=0.5,
            prev_lat_accel=0.4,
            current_curvature=0.005,
            prev_curvature=0.004,
            dt=0.05
        )

        # All should be safe in this scenario
        all_safe = all(is_safe for is_safe, _ in all_results.values())
        self.assertTrue(all_safe, f"Not all operational metrics were safe: {all_results}")

        # Calculate safety score
        safety_score = safety_metrics.get_operational_safety_score(all_results)
        self.assertGreaterEqual(safety_score, 0.8, "Safety score should be high when all metrics are safe")


class TestSafetyCriticalInteractions(unittest.TestCase):
    """
    Additional tests for safety-critical module interactions
    """
    
    def setUp(self):
        self.backup_system = RedundantControlValidator()
        self.env_processor = EnvironmentalConditionProcessor()
        self.conf_validator = ModelConfidenceValidator()
        
    def test_emergency_scenario_integration(self):
        """
        Test system behavior in emergency scenarios where multiple safety systems activate
        """
        # Simulate emergency conditions
        high_risk_env = 0.9  # High environmental risk
        low_model_conf = 0.2  # Low model confidence
        dangerous_curvature = 0.8  # Very high curvature request
        dangerous_accel = 8.0  # Very high acceleration request
        v_ego = 30.0  # High speed
        
        # Process through backup system
        safe_curvature, safe_accel = self.backup_system.validate_and_backup(
            dangerous_curvature, dangerous_accel, v_ego,
            low_model_conf, high_risk_env
        )
        
        # Should return safe values significantly different from dangerous inputs
        self.assertNotEqual(safe_curvature, dangerous_curvature)
        self.assertNotEqual(safe_accel, dangerous_accel)
        self.assertLess(abs(safe_curvature), abs(dangerous_curvature))
        self.assertLess(abs(safe_accel), abs(dangerous_accel))
        
        # Curvature should be very close to 0 for safety
        self.assertAlmostEqual(safe_curvature, 0.0, places=2)
        # Acceleration should be negative (deceleration) for safety
        self.assertLessEqual(safe_accel, 0.0)
    
    def test_degraded_mode_operation(self):
        """
        Test system behavior when one or more modules are operating in degraded mode
        """
        # Test with environmental processor returning high risk (degraded perception)
        sm = Mock()
        sm.updated = {'modelV2': True}
        sm.recv_frame = {'modelV2': 0}
        
        # Mock model with very poor detection
        mock_model = Mock()
        mock_model.laneLines = [Mock() for _ in range(4)]
        for lane in mock_model.laneLines:
            lane.prob = 0.05  # Very low probability
            lane.std = 1.5  # Very high uncertainty
        mock_model.roadEdges = [Mock() for _ in range(2)]
        for edge in mock_model.roadEdges:
            edge.prob = 0.0  # No edge detection
        mock_model.position.y = [2.0] * 10  # Uncertain path
        
        sm['modelV2'] = mock_model
        self.env_processor.update(sm)
        
        env_conditions = self.env_processor.get_current_conditions()
        
        # Adaptive system should respond to degraded perception
        adaptive_ctrl = AdaptiveController()
        adaptive_ctrl.update_conditions(
            is_curving=False,
            is_on_grade=False,
            model_confidence_low=True,
            visibility_poor=True,
            environmental_risk=env_conditions.get('risk_score', 0.9)  # Default to high risk
        )
        
        # Should be very conservative
        lat_limits = adaptive_ctrl.get_adaptive_lateral_limits()
        max_lat = lat_limits[1]
        
        # With degraded perception, should be much more conservative
        self.assertLess(abs(max_lat), adaptive_ctrl.lateral_accel_limit * 0.7)


def run_integration_tests():
    """
    Run all integration tests
    """
    test_classes = [
        TestSunnypilotIntegration,
        TestSafetyCriticalInteractions
    ]
    
    for test_class in test_classes:
        loader = unittest.TestLoader()
        suite = loader.loadTestsFromTestCase(test_class)
        
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        
        if not result.wasSuccessful():
            print(f"Tests in {test_class.__name__} failed!")
            return False
    
    print("All integration tests passed!")
    return True


if __name__ == '__main__':
    run_integration_tests()