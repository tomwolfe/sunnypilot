"""
Unit tests for SafetyMonitor and related components
"""

import unittest
from unittest.mock import Mock, patch
import numpy as np
from selfdrive.common.safety_redundancy import SafetyMonitor, SafetySystemStatus, FallbackLevel, ValidationRedundancy


class TestSafetyMonitor(unittest.TestCase):
    """Test cases for SafetyMonitor"""

    def setUp(self):
        """Set up test fixtures"""
        self.safety_monitor = SafetyMonitor()

    def test_initialization(self):
        """Test that SafetyMonitor initializes correctly"""
        self.assertIsNotNone(self.safety_monitor)
        self.assertIsNotNone(self.safety_monitor.redundancy_manager)
        self.assertIsNotNone(self.safety_monitor.validation_redundancy)
        self.assertEqual(self.safety_monitor.safety_limits['max_lat_accel'], 2.5)

    def test_register_systems(self):
        """Test that register_systems works correctly"""
        self.safety_monitor.register_systems()
        
        # Check that systems were registered
        registered_systems = self.safety_monitor.redundancy_manager.systems
        expected_systems = [
            'perception', 'planning', 'control', 'validation',
            'communication', 'power_management'
        ]
        
        for system in expected_systems:
            self.assertIn(system, registered_systems)

    def test_check_emergency_conditions_no_emergency(self):
        """Test emergency condition checking with no emergencies"""
        car_state = {'vEgo': 15.0}
        model_output = {
            'leads_v3': [
                Mock(dRel=50.0, prob=0.9)  # Safe distance
            ]
        }
        env_context = {}

        emergency = self.safety_monitor.check_emergency_conditions(car_state, model_output, env_context)

        self.assertFalse(emergency)

    def test_validate_model_output_basic(self):
        """Test basic model output validation"""
        model_output = {
            'lateral_control': {'acceleration': 1.0},
            'longitudinal_control': {'acceleration': 1.0}
        }
        car_state = {
            'vEgo': 15.0,
            'max_acceleration': 3.0,
            'min_acceleration': -5.0
        }

        validation_passed, validation_result = self.safety_monitor.validate_model_output(
            model_output, car_state
        )

        self.assertIsInstance(validation_passed, bool)
        self.assertIsInstance(validation_result, dict)
        self.assertIn('passed', validation_result)
        self.assertIn('confidence', validation_result)
        self.assertIn('messages', validation_result)

    def test_get_safety_recommendation_normal(self):
        """Test safety recommendation in normal conditions"""
        car_state = {
            'vEgo': 15.0,
            'max_acceleration': 3.0,
            'min_acceleration': -5.0
        }
        model_output = {
            'leads_v3': [Mock(dRel=50.0, prob=0.9)],
            'lateral_control': {'acceleration': 0.5},
            'longitudinal_control': {'acceleration': 1.0}
        }

        recommendation = self.safety_monitor.get_safety_recommendation(car_state, model_output)

        # The function returns (FallbackLevel, str), check that first element is a FallbackLevel
        fallback_level, reason = recommendation
        self.assertIsInstance(fallback_level, FallbackLevel)
        self.assertIsInstance(reason, str)

    def test_validation_redundancy_basic(self):
        """Test basic validation redundancy functionality"""
        validator = ValidationRedundancy()
        
        model_output = {
            'leads_v3': [Mock(dRel=50.0, prob=0.9)]
        }
        car_state = {'vEgo': 15.0}
        
        result = validator.validate_comprehensive(model_output, car_state)
        
        self.assertEqual(len(result), 3)  # Should return (passed, score, messages)
        validation_passed, confidence, messages = result
        self.assertIsInstance(validation_passed, bool)
        self.assertIsInstance(confidence, float)
        self.assertIsInstance(messages, list)


class TestValidationRedundancy(unittest.TestCase):
    """Test cases for ValidationRedundancy"""

    def setUp(self):
        """Set up test fixtures"""
        self.validator = ValidationRedundancy()

    def test_model_output_validation_valid(self):
        """Test model output validation with valid data"""
        model_output = {
            'leads_v3': [Mock(dRel=50.0, prob=0.9)]
        }
        
        result = self.validator._model_output_validation(model_output)
        passed, score, message = result
        
        self.assertIsInstance(passed, bool)
        self.assertIsInstance(score, float)
        self.assertIsInstance(message, str)

    def test_model_output_validation_with_nan(self):
        """Test model output validation with NaN values"""
        # Create an array with NaN
        nan_array = np.array([1.0, np.nan, 3.0])
        model_output = {
            'test_array': nan_array,
            'leads_v3': [Mock(dRel=50.0, prob=0.9)]
        }
        
        result = self.validator._model_output_validation(model_output)
        passed, score, message = result
        
        # Should fail due to NaN values
        self.assertLessEqual(score, 0.2)  # Should be low due to NaN


if __name__ == '__main__':
    unittest.main()