"""
Unit tests for ValidationMetricsPublisher
"""

import unittest
from unittest.mock import Mock, patch
import numpy as np
from selfdrive.common.validation_publisher import ValidationMetricsPublisher


class TestValidationMetricsPublisher(unittest.TestCase):
    """Test cases for ValidationMetricsPublisher"""

    def setUp(self):
        """Set up test fixtures"""
        self.publisher = ValidationMetricsPublisher()

    def test_initialization(self):
        """Test that ValidationMetricsPublisher initializes correctly"""
        self.assertIsNotNone(self.publisher)
        self.assertIsNotNone(self.publisher.params)

    def test_create_basic_metrics(self):
        """Test creation of basic metrics from model output"""
        model_output = {
            'lead_confidence_avg': 0.8,
            'lane_confidence_avg': 0.75,
            'overall_confidence': 0.85,
            'temporal_consistency': 0.9
        }

        metrics = self.publisher.create_basic_metrics(model_output)

        self.assertEqual(metrics['base_confidence'], 0.85)
        self.assertEqual(metrics['situation_factor'], 1.0)
        self.assertTrue(metrics['system_safe'])
        self.assertTrue(metrics['lead_confidence_ok'])

    def test_create_basic_metrics_defaults(self):
        """Test creation of basic metrics with default values"""
        model_output = {}  # Empty dict

        metrics = self.publisher.create_basic_metrics(model_output)

        self.assertEqual(metrics['base_confidence'], 0.0)
        self.assertEqual(metrics['situation_factor'], 1.0)
        self.assertFalse(metrics['system_safe'])
        self.assertFalse(metrics['lead_confidence_ok'])
        self.assertFalse(metrics['lane_confidence_ok'])
        self.assertFalse(metrics['overall_confidence_ok'])

    @patch('selfdrive.common.validation_publisher.cloudlog')
    def test_publish_metrics_with_none_result(self, mock_cloudlog):
        """Test publishing metrics with None result"""
        self.publisher.publish_metrics(None)
        mock_cloudlog.warning.assert_called_once_with("No validation result to publish")

    @patch('selfdrive.common.validation_publisher.cloudlog')
    @patch('selfdrive.common.validation_publisher.messaging')
    def test_publish_metrics_with_invalid_result(self, mock_messaging, mock_cloudlog):
        """Test publishing metrics with invalid result"""
        # Set up mock message
        mock_msg = Mock()
        mock_validation_metrics = Mock()
        mock_msg.validationMetrics = mock_validation_metrics
        mock_messaging.new_message.return_value = mock_msg
        
        if self.publisher.pm:
            self.publisher.pm = Mock()
            self.publisher.pm.send = Mock()

        # Test with result that has wrong data types
        invalid_result = {
            'base_confidence': 'not_a_number',
            'situation_adjusted_confidence': 'not_a_number'
        }
        
        # This should not crash, due to the type conversion fixes
        self.publisher.publish_metrics(invalid_result)


if __name__ == '__main__':
    unittest.main()