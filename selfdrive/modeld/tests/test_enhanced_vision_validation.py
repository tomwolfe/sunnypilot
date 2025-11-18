"""
Unit tests for EnhancedVisionProcessor validation metrics
"""
import unittest
import numpy as np
from cereal import log
from openpilot.selfdrive.modeld.enhanced_vision import EnhancedVisionProcessor


class TestEnhancedVisionValidationMetrics(unittest.TestCase):
    
    def setUp(self):
        self.processor = EnhancedVisionProcessor()
    
    def test_validation_metrics_empty_input(self):
        """Test validation metrics with empty or minimal model output."""
        model_output = {}
        
        validation_metrics = self.processor.validate_model_outputs(model_output)
        
        self.assertIn('lead_confidence_avg', validation_metrics)
        self.assertIn('lead_confidence_max', validation_metrics)
        self.assertIn('lane_confidence_avg', validation_metrics)
        self.assertIn('overall_confidence', validation_metrics)
        
        # Should have default values for missing data
        self.assertEqual(validation_metrics['lead_confidence_avg'], 0.0)
        self.assertEqual(validation_metrics['lead_confidence_max'], 0.0)
        self.assertEqual(validation_metrics['lane_confidence_avg'], 0.0)
    
    def test_validation_metrics_lead_detection(self):
        """Test validation metrics with simulated lead detection data."""
        # Create mock lead data with different confidence levels
        class MockLead:
            def __init__(self, prob):
                self.prob = prob

        model_output = {
            'leads_v3': [
                MockLead(0.8),  # High confidence lead
                MockLead(0.6)   # Medium confidence lead
            ]
        }
        
        validation_metrics = self.processor.validate_model_outputs(model_output)
        
        self.assertAlmostEqual(validation_metrics['lead_confidence_avg'], 0.7, places=2)  # (0.8 + 0.6) / 2
        self.assertEqual(validation_metrics['lead_confidence_max'], 0.8)
    
    def test_validation_metrics_lane_detection(self):
        """Test validation metrics with simulated lane detection data."""
        # Create mock lane data
        class MockLane:
            def __init__(self, prob):
                self.prob = prob

        model_output = {
            'lane_lines': [
                MockLane(0.9),  # Left lane
                MockLane(0.85), # Left-right lane
                MockLane(0.8),  # Right-left lane
                MockLane(0.95)  # Right lane
            ]
        }
        
        validation_metrics = self.processor.validate_model_outputs(model_output)
        
        expected_avg = (0.9 + 0.85 + 0.8 + 0.95) / 4
        self.assertAlmostEqual(validation_metrics['lane_confidence_avg'], expected_avg, places=2)
    
    def test_overall_confidence_calculation(self):
        """Test the overall confidence calculation formula."""
        # Create a mock model output with known values
        class MockLead:
            def __init__(self, prob):
                self.prob = prob
        
        class MockLane:
            def __init__(self, prob):
                self.prob = prob

        model_output = {
            'leads_v3': [MockLead(0.8), MockLead(0.7)],
            'lane_lines': [MockLane(0.85), MockLane(0.8), MockLane(0.75), MockLane(0.9)]
        }
        
        validation_metrics = self.processor.validate_model_outputs(model_output)
        
        # Formula from the EnhancedVisionProcessor:
        # overall_confidence = (
        #     lead_confidence_avg * 0.4 +
        #     lane_confidence_avg * 0.3 +
        #     lead_confidence_max * 0.3
        # )
        lead_conf_avg = (0.8 + 0.7) / 2  # 0.75
        lane_conf_avg = (0.85 + 0.8 + 0.75 + 0.9) / 4  # 0.825
        lead_conf_max = 0.8
        
        expected_overall = lead_conf_avg * 0.4 + lane_conf_avg * 0.3 + lead_conf_max * 0.3
        expected_overall = 0.75 * 0.4 + 0.825 * 0.3 + 0.8 * 0.3
        expected_overall = 0.3 + 0.2475 + 0.24  # = 0.7875
        
        self.assertAlmostEqual(validation_metrics['overall_confidence'], expected_overall, places=3)
    
    def test_enhance_feature_extraction_with_depth(self):
        """Test feature enhancement with depth information."""
        # Create mock vision output
        vision_output = {
            'leads_v3': []
        }
        
        # Add some mock leads
        class MockLead:
            def __init__(self, x_vals, y_vals, prob):
                self.x = x_vals
                self.y = y_vals
                self.prob = prob
        
        vision_output['leads_v3'] = [MockLead([50.0], [0.0], 0.8)]
        
        # Create mock depth map
        depth_map = np.zeros((100, 100), dtype=np.float32)
        depth_map[50, 50] = 50.0  # Set depth at position where lead is detected
        
        # This should enhance the features based on depth consistency
        enhanced_output = self.processor.enhance_feature_extraction(vision_output, depth_map)
        
        # The lead probability might be adjusted based on depth consistency
        self.assertEqual(len(enhanced_output['leads_v3']), 1)
        # Note: The actual enhancement logic depends on the position matching
    
    def test_estimate_depth_from_stereo(self):
        """Test depth estimation functionality."""
        # Create simple test images
        road_img = np.ones((100, 100, 3), dtype=np.uint8) * 128
        wide_road_img = np.ones((100, 100, 3), dtype=np.uint8) * 128
        
        transforms = {}
        calib_params = np.array([0.0, 0.0, 0.0], dtype=np.float32)  # No calibration offset
        
        depth_map = self.processor.estimate_depth_from_stereo(
            road_img, wide_road_img, transforms, calib_params
        )
        
        # Should return a depth map or None if computation fails
        self.assertTrue(depth_map is None or isinstance(depth_map, np.ndarray))
        
        if depth_map is not None:
            self.assertEqual(depth_map.shape, road_img.shape[:2])
            self.assertEqual(depth_map.dtype, np.float32)


if __name__ == '__main__':
    unittest.main()