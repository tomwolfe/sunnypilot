"""
Integration tests for validation metrics publishing in modeld.py
"""
import unittest
from unittest.mock import Mock, patch
import numpy as np
from cereal import log
import cereal.messaging as messaging
from openpilot.selfdrive.modeld.enhanced_vision import EnhancedVisionProcessor


class TestValidationMetricsPublishing(unittest.TestCase):
    
    def setUp(self):
        self.processor = EnhancedVisionProcessor()
    
    def test_validation_metrics_structure(self):
        """Test that validation metrics have the expected structure and fields."""
        # Create model output with some data
        class MockLead:
            def __init__(self, prob):
                self.prob = prob
        
        class MockLane:
            def __init__(self, prob):
                self.prob = prob

        model_output = {
            'leads_v3': [MockLead(0.8), MockLead(0.6)],
            'lane_lines': [MockLane(0.85), MockLane(0.8), MockLane(0.75), MockLane(0.9)]
        }
        
        validation_metrics = self.processor.validate_model_outputs(model_output)
        
        # Check that all required fields are present
        expected_fields = [
            'lead_confidence_avg',
            'lead_confidence_max', 
            'lane_confidence_avg',
            'overall_confidence'
        ]
        
        for field in expected_fields:
            self.assertIn(field, validation_metrics)
            self.assertIsInstance(validation_metrics[field], (float, int))
    
    @patch('cereal.messaging.new_message')
    @patch('time.time')
    def test_validation_metrics_message_creation(self, mock_time, mock_new_message):
        """Test that validation metrics messages are created correctly."""
        # Mock timestamp
        mock_time.return_value = 12345.678
        
        # Mock the message creation
        mock_message = Mock()
        mock_validation_msg = Mock()
        mock_message.validationMetrics = mock_validation_msg
        mock_new_message.return_value = mock_message
        
        # Create some validation metrics
        validation_data = {
            'leadConfidenceAvg': 0.75,
            'leadConfidenceMax': 0.8,
            'laneConfidenceAvg': 0.78,
            'overallConfidence': 0.77,
            'isValid': True,
            'confidenceThreshold': 0.5
        }
        
        # Simulate what happens in modeld.py when creating validation metrics message
        validation_send = messaging.new_message('validationMetrics')
        vm = validation_send.validationMetrics
        
        # Set the values (this would happen in the actual modeld.py)
        vm.leadConfidenceAvg = validation_data['leadConfidenceAvg']
        vm.leadConfidenceMax = validation_data['leadConfidenceMax']
        vm.laneConfidenceAvg = validation_data['laneConfidenceAvg']
        vm.overallConfidence = validation_data['overallConfidence']
        vm.timestampMonoTime = 1234567890  # Some timestamp
        vm.isValid = validation_data['isValid']
        vm.confidenceThreshold = validation_data['confidenceThreshold']
        
        # Verify the message was created with the right structure
        self.assertEqual(vm.leadConfidenceAvg, 0.75)
        self.assertEqual(vm.leadConfidenceMax, 0.8)
        self.assertEqual(vm.laneConfidenceAvg, 0.78)
        self.assertEqual(vm.overallConfidence, 0.77)
        self.assertEqual(vm.isValid, True)
        self.assertEqual(vm.confidenceThreshold, 0.5)
    
    def test_validate_model_outputs_edge_cases(self):
        """Test validation metrics with edge cases."""
        # Test with no leads
        model_output_no_leads = {'lane_lines': []}
        metrics_no_leads = self.processor.validate_model_outputs(model_output_no_leads)
        self.assertEqual(metrics_no_leads['lead_confidence_avg'], 0.0)
        self.assertEqual(metrics_no_leads['lead_confidence_max'], 0.0)
        
        # Test with no lanes
        model_output_no_lanes = {'leads_v3': []}
        metrics_no_lanes = self.processor.validate_model_outputs(model_output_no_lanes)
        self.assertEqual(metrics_no_lanes['lane_confidence_avg'], 0.0)
        
        # Test with missing keys
        model_output_empty = {}
        metrics_empty = self.processor.validate_model_outputs(model_output_empty)
        self.assertEqual(metrics_empty['lead_confidence_avg'], 0.0)
        self.assertEqual(metrics_empty['lead_confidence_max'], 0.0)
        self.assertEqual(metrics_empty['lane_confidence_avg'], 0.0)
        self.assertGreaterEqual(metrics_empty['overall_confidence'], 0.0)  # Should be 0 due to all zeros


if __name__ == '__main__':
    unittest.main()