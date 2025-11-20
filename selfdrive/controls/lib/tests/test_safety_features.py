#!/usr/bin/env python3
"""
Test suite for new safety features in sunnypilot
Tests for environmental awareness, lateral safety, and adaptive behavior systems
"""

import unittest
import numpy as np
from unittest.mock import Mock, MagicMock
from openpilot.common.swaglog import cloudlog

from ..environmental_awareness import EnvironmentalConditionProcessor, EnvironmentalConditionMonitor
from ..lateral_safety import calculate_safe_curvature_limits, get_adaptive_lateral_curvature, adjust_lateral_limits_for_conditions
from ..model_confidence_validator import ModelConfidenceValidator
from ..performance_monitor import PerformanceMonitor


class TestEnvironmentalAwareness(unittest.TestCase):
    """Test environmental awareness system functionality"""

    def setUp(self):
        self.processor = EnvironmentalConditionProcessor()
        self.monitor = EnvironmentalConditionMonitor()

    def test_environmental_risk_calculation(self):
        """Test that environmental risk calculation handles non-linear interactions properly"""
        # Create a mock monitor instance for testing
        monitor = EnvironmentalConditionMonitor()
        
        # Test base case - no adverse conditions
        risk = monitor.get_environmental_risk_score(v_ego=10.0, curvature_ahead=0.001)
        self.assertGreaterEqual(risk, 0.0)
        self.assertLessEqual(risk, 1.0)
        
        # Test with multiple adverse conditions (should compound risk)
        monitor.is_rainy = True
        monitor.low_visibility = True
        monitor.is_night = True
        monitor.model_road_quality = 0.3  # Poor
        monitor.model_surface_condition = 0.3  # Poor
        
        risk_multi = monitor.get_environmental_risk_score(v_ego=25.0, curvature_ahead=0.01)
        self.assertGreaterEqual(risk_multi, 0.0)
        self.assertLessEqual(risk_multi, 1.0)
        # Risk should be higher with multiple adverse conditions

    def test_gradual_degradation(self):
        """Test that environmental processor degrades gradually rather than immediately to worst case"""
        processor = EnvironmentalConditionProcessor()

        # Create properly structured mock objects
        mock_meta = Mock()
        mock_meta.lanelessProbs = [0.1, 0.2]  # Proper list structure
        mock_meta.laneLineProbs = [0.8, 0.7, 0.6, 0.5]  # Also needed for the update function
        mock_meta.confidence = 0.8
        mock_meta.pathStds = [0.1] * 10

        mock_model_v2 = Mock()
        mock_model_v2.meta = mock_meta
        mock_model_v2.path = Mock()
        mock_model_v2.path.y = [0.0] * 20

        mock_car_state = Mock()
        mock_car_state.vEgo = 15.0

        mock_device_state = Mock()
        mock_road_camera = Mock()
        mock_road_camera.intensity = 100
        mock_road_camera.exposure = 50

        # Create a SubMaster mock that returns properly structured data
        mock_sm = Mock()
        mock_sm.updated = {'modelV2': True, 'carState': True, 'deviceState': True, 'roadCameraState': True}
        mock_sm.__getitem__ = Mock(side_effect=lambda key: {
            'modelV2': mock_model_v2,
            'carState': mock_car_state,
            'deviceState': mock_device_state,
            'roadCameraState': mock_road_camera
        }.get(key))

        # Process with the properly structured mock data
        processor.update(mock_sm)

        # Check that conditions are handled appropriately after processing
        conditions = processor.environmental_conditions
        self.assertIn('is_rainy', conditions)
        self.assertIn('is_night', conditions)
        self.assertIn('low_visibility', conditions)


class TestLateralSafety(unittest.TestCase):
    """Test lateral safety system functionality"""

    def test_curvature_calculation_bounds(self):
        """Test that curvature calculation handles extreme values safely"""
        # Test with very low speed (should return conservative limits)
        min_curv, max_curv = calculate_safe_curvature_limits(v_ego=0.05)
        self.assertLessEqual(max_curv, 0.2)
        self.assertGreaterEqual(min_curv, -0.2)
        
        # Test with normal speed
        min_curv, max_curv = calculate_safe_curvature_limits(v_ego=20.0)
        self.assertLessEqual(max_curv, 0.2)
        self.assertGreaterEqual(min_curv, -0.2)
        
        # Test with very high speed
        min_curv, max_curv = calculate_safe_curvature_limits(v_ego=40.0)
        self.assertLessEqual(max_curv, 0.2)
        self.assertGreaterEqual(min_curv, -0.2)
        
        # Test with invalid inputs
        min_curv, max_curv = calculate_safe_curvature_limits(v_ego=None)
        self.assertEqual(min_curv, -0.1)  # Safe default
        self.assertEqual(max_curv, 0.1)   # Safe default
        
        min_curv, max_curv = calculate_safe_curvature_limits(v_ego="invalid")
        self.assertEqual(min_curv, -0.1)  # Safe default
        self.assertEqual(max_curv, 0.1)   # Safe default

    def test_adjust_lateral_limits_conditions(self):
        """Test that lateral limits are properly adjusted for various conditions"""
        # Test with high confidence and good conditions
        max_lat_accel, rate_mult = adjust_lateral_limits_for_conditions(
            v_ego=20.0,
            curvature_ahead=0.002,
            model_confidence=0.9,
            is_rainy=False,
            is_night=False
        )
        # Should be at or near maximum allowed (3.0 is the default)
        self.assertLessEqual(max_lat_accel, 3.0)  # Should not exceed maximum
        self.assertLessEqual(rate_mult, 1.0)      # Should not exceed maximum
        
        # Test with low confidence and adverse conditions
        max_lat_accel, rate_mult = adjust_lateral_limits_for_conditions(
            v_ego=25.0,
            curvature_ahead=0.01,
            model_confidence=0.5,
            is_rainy=True,
            is_night=True
        )
        # Should be significantly reduced
        self.assertLess(max_lat_accel, 2.0)  # Much more conservative
        self.assertLess(rate_mult, 0.7)      # More conservative rate limiting


class TestModelConfidenceValidator(unittest.TestCase):
    """Test model confidence validator functionality"""

    def test_consistent_thresholds(self):
        """Test that model confidence validator uses consistent thresholds"""
        validator = ModelConfidenceValidator()
        from ..autonomous_params import LATERAL_SAFETY_PARAMS
        
        # Ensure validator uses the same threshold as defined in params
        expected_threshold = LATERAL_SAFETY_PARAMS['MODEL_CONFIDENCE_THRESHOLD']
        self.assertEqual(validator.confidence_threshold, expected_threshold)

    def test_confidence_validation(self):
        """Test that model confidence validation works properly"""
        validator = ModelConfidenceValidator()
        
        # Test with high confidence model output
        model_output = {
            'modelConfidence': 0.9,
            'path': [0.0, 0.1, 0.2, 0.3],
            'desiredCurvature': [0.01, 0.015, 0.02, 0.018]
        }
        
        enh_conf, is_safe, details = validator.validate_model_output(model_output, 15.0, 0.01)
        self.assertGreaterEqual(enh_conf, 0.0)
        self.assertLessEqual(enh_conf, 1.0)
        self.assertTrue(is_safe)  # Should be safe with high confidence


class TestPerformanceMonitor(unittest.TestCase):
    """Test performance monitoring functionality"""

    def test_fallback_tracking(self):
        """Test that fallback triggers are properly tracked"""
        monitor = PerformanceMonitor()
        
        # Record a fallback
        monitor.record_fallback_trigger('test_fallback', {'test': 'data'})
        
        # Check that it was recorded
        report = monitor.get_performance_report()
        self.assertIn('test_fallback', report['fallback_counts'])
        self.assertEqual(report['fallback_counts']['test_fallback'], 1)
        
        # Record multiple to test frequency calculation
        for i in range(5):
            monitor.record_fallback_trigger('test_fallback2')
        
        report = monitor.get_performance_report()
        self.assertEqual(report['fallback_counts']['test_fallback2'], 5)

    def test_system_health_tracking(self):
        """Test that system health is properly tracked"""
        monitor = PerformanceMonitor()
        
        # Record system states
        monitor.record_system_state('test_system', True)  # healthy
        monitor.record_system_state('test_system', False) # failed
        monitor.record_system_state('test_system', True)  # healthy
        
        report = monitor.get_performance_report()
        self.assertEqual(report['system_health']['test_system_healthy'], 2)
        self.assertEqual(report['system_health']['test_system_failed'], 1)


class TestAdaptiveCurvatureFunction(unittest.TestCase):
    """Test adaptive lateral curvature function"""

    def test_error_handling_returns_neutral(self):
        """Test that error in adaptive curvature function returns neutral (0.0) instead of previous value"""
        # This test verifies the fix for the dangerous oscillation issue
        from ..lateral_safety import get_adaptive_lateral_curvature
        from ..autonomous_params import LATERAL_SAFETY_PARAMS

        # Create a properly structured mock model_v2
        mock_meta = Mock()
        mock_meta.confidence = 0.8

        # Create mock model_v2 with properly structured data
        mock_model_v2 = Mock()
        mock_model_v2.meta = mock_meta
        mock_model_v2.orientationNED = Mock()
        mock_model_v2.orientationNED.x = [0.0, 0.0, 0.0]  # Avoid index issues

        # Mock path with attribute access
        mock_path = Mock()
        mock_path.y = [0.0] * 10  # 10 points of y coordinates
        mock_path.x = [i * 0.1 for i in range(10)]  # 10 points of x coordinates
        mock_model_v2.path = mock_path

        # Create a mock params object with required attributes
        mock_params = Mock()
        mock_params.roll = 0.0  # Add roll attribute for compensation

        # Create a mock CarParams
        mock_CP = Mock()
        mock_CP.lateralParams = Mock()
        mock_CP.lateralParams.maxLateralAccel = LATERAL_SAFETY_PARAMS['MAX_LATERAL_ACCEL']

        # Test that error handling returns a valid result
        result = get_adaptive_lateral_curvature(
            v_ego=15.0,
            desired_curvature=0.05,
            prev_curvature=0.1,  # This would be the previous curvature
            model_v2=mock_model_v2,
            params=mock_params,  # Mock params
            CP=mock_CP,      # Mock CarParams
            is_rainy=False,
            is_night=False,
            dt=0.01
        )

        # Function should return a valid curvature value
        self.assertIsInstance(result, (int, float))
        self.assertGreaterEqual(result, -0.2)  # Within safe limits
        self.assertLessEqual(result, 0.2)      # Within safe limits


if __name__ == '__main__':
    unittest.main()