"""
Unit tests for PredictiveThermalManager
"""

import unittest
from unittest.mock import Mock, patch
import numpy as np
from selfdrive.common.thermal_management import PredictiveThermalManager


class TestPredictiveThermalManager(unittest.TestCase):
    """Test cases for PredictiveThermalManager"""

    def setUp(self):
        """Set up test fixtures"""
        self.thermal_manager = PredictiveThermalManager()

    def test_initialization(self):
        """Test that PredictiveThermalManager initializes correctly"""
        self.assertIsNotNone(self.thermal_manager)
        self.assertEqual(self.thermal_manager.performance_scaling, 1.0)
        self.assertEqual(len(self.thermal_manager.thermal_history), 0)

    def test_calculate_component_score_normal_range(self):
        """Test component score calculation in normal range"""
        thresholds = {'min': 30, 'max': 80, 'critical': 95}
        # Value in good range (between min and max)
        score = self.thermal_manager._calculate_component_score(50, thresholds)
        self.assertGreater(score, 0.5)
        self.assertLessEqual(score, 1.0)

        # Value at min threshold
        score_min = self.thermal_manager._calculate_component_score(30, thresholds)
        self.assertEqual(score_min, 1.0)

        # Value at max threshold
        score_max = self.thermal_manager._calculate_component_score(80, thresholds)
        self.assertEqual(score_max, 0.0)

    def test_calculate_component_score_critical_range(self):
        """Test component score calculation in critical range"""
        thresholds = {'min': 30, 'max': 80, 'critical': 95}
        # Value in critical range (above max, below critical)
        score = self.thermal_manager._calculate_component_score(90, thresholds)
        self.assertGreaterEqual(score, 0.0)
        self.assertLess(score, 0.3)

        # Value at critical threshold
        score_critical = self.thermal_manager._calculate_component_score(95, thresholds)
        self.assertEqual(score_critical, 0.0)

    def test_calculate_component_score_edge_cases(self):
        """Test component score calculation edge cases"""
        # Value below minimum
        thresholds = {'min': 30, 'max': 80, 'critical': 95}
        score_below = self.thermal_manager._calculate_component_score(20, thresholds)
        self.assertEqual(score_below, 1.0)

        # Invalid thresholds (max <= min)
        invalid_thresholds = {'min': 80, 'max': 70, 'critical': 95}
        with patch('selfdrive.common.thermal_management.cloudlog') as mock_cloudlog:
            score = self.thermal_manager._calculate_component_score(75, invalid_thresholds)
            self.assertEqual(score, 1.0)
            mock_cloudlog.error.assert_called()

        # Invalid thresholds (critical <= max)
        invalid_thresholds2 = {'min': 30, 'max': 80, 'critical': 75}
        with patch('selfdrive.common.thermal_management.cloudlog') as mock_cloudlog:
            score = self.thermal_manager._calculate_component_score(75, invalid_thresholds2)
            self.assertEqual(score, 1.0)
            mock_cloudlog.error.assert_called()

    def test_calculate_thermal_metrics_basic(self):
        """Test thermal metrics calculation with valid inputs"""
        current_data = {
            'cpu_temp': 65,
            'gpu_temp': 55,
            'memory_percent': 60,
            'cpu_percent': 45,
            'thermal_status': 1
        }
        predicted_data = {
            'cpu_temp': 70,
            'gpu_temp': 60,
            'trend': 'heating'
        }
        
        metrics = self.thermal_manager._calculate_thermal_metrics(current_data, predicted_data)
        
        self.assertIsInstance(metrics, dict)
        self.assertIn('thermal_score', metrics)
        self.assertIn('system_thermal_state', metrics)
        self.assertGreaterEqual(metrics['thermal_score'], 0.0)
        self.assertLessEqual(metrics['thermal_score'], 1.0)


if __name__ == '__main__':
    unittest.main()