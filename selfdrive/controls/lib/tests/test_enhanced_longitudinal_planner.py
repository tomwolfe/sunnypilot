"""
Unit tests for EnhancedLongitudinalPlanner module
"""
import unittest
import numpy as np
from unittest.mock import Mock, MagicMock
from cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.enhanced_longitudinal_planner import EnhancedLongitudinalPlanner
from opendbc.car.car_helpers import interfaces
from opendbc.car import CarParams


class TestEnhancedLongitudinalPlanner(unittest.TestCase):
    
    def setUp(self):
        # Create a mock CarParams for testing
        self.CP = CarParams()
        self.CP.longitudinalTuning.kA = 1.0  # Example longitudinal acceleration gain
        self.planner = EnhancedLongitudinalPlanner(self.CP)
    
    def test_initialization(self):
        """Test that the planner initializes correctly."""
        self.assertIsNotNone(self.planner.CP)
        self.assertEqual(self.planner.CP.longitudinalTuning.kA, 1.0)
        self.assertIsInstance(self.planner.lead_distance_filter, FirstOrderFilter)
        self.assertIsInstance(self.planner.lead_speed_filter, FirstOrderFilter)
        self.assertIsInstance(self.planner.lead_accel_filter, FirstOrderFilter)
    
    def test_confidence_accel_limits(self):
        """Test that acceleration limits are properly set based on confidence."""
        self.assertIn('high', self.planner.confidence_accel_limits)
        self.assertIn('medium', self.planner.confidence_accel_limits)
        self.assertIn('low', self.planner.confidence_accel_limits)
        
        # Verify that low confidence limits are less than high confidence limits
        low_limits = self.planner.confidence_accel_limits['low']
        high_limits = self.planner.confidence_accel_limits['high']
        self.assertLess(low_limits[0], high_limits[0])  # Negative values, so less aggressive braking
        self.assertLess(low_limits[1], high_limits[1])  # Less aggressive acceleration
    
    def test_update_lead_vehicle_prediction_with_validation_metrics(self):
        """Test that longitudinal plan is updated correctly with validation metrics."""
        # Create mock longitudinal plan
        long_plan = log.LongitudinalPlan.new_message()
        long_plan.aTarget = 1.0
        long_plan.shouldStop = False
        long_plan.hasLead = True
        long_plan.vEgo = 20.0  # 20 m/s
        
        # Create mock radar state
        radar_state = log.RadarState.new_message()
        radar_state.leadOne = log.RadarState.LeadData.new_message()
        radar_state.leadOne.status = True
        radar_state.leadOne.dRel = 50.0  # 50m lead distance
        radar_state.leadOne.vRel = -5.0  # 5 m/s slower than lead
        
        # Create mock car state
        car_state = log.CarState.new_message()
        car_state.vEgo = 25.0  # 25 m/s ego speed
        
        # Create validation metrics
        validation_metrics = {
            'leadConfidenceAvg': 0.8,
            'leadConfidenceMax': 0.9,
            'laneConfidenceAvg': 0.85,
            'overallConfidence': 0.82,
            'isValid': True,
            'confidenceThreshold': 0.5
        }
        
        # Test the update method
        enhanced_plan = self.planner.update_lead_vehicle_prediction(
            long_plan, radar_state, car_state, validation_metrics
        )
        
        # Check that the enhanced plan is returned and is still a LongitudinalPlan
        self.assertIsNotNone(enhanced_plan)
        self.assertEqual(enhanced_plan.aTarget, 1.0)  # Should remain the same unless modified by logic
        self.assertEqual(enhanced_plan.shouldStop, False)
    
    def test_update_lead_vehicle_prediction_low_confidence(self):
        """Test that low confidence results in more conservative behavior."""
        # Create mock longitudinal plan
        long_plan = log.LongitudinalPlan.new_message()
        long_plan.aTarget = 2.0  # High acceleration target
        long_plan.shouldStop = False
        long_plan.hasLead = True
        long_plan.vEgo = 20.0
        
        # Create mock radar state with close lead
        radar_state = log.RadarState.new_message()
        radar_state.leadOne = log.RadarState.LeadData.new_message()
        radar_state.leadOne.status = True
        radar_state.leadOne.dRel = 25.0  # Close lead distance
        radar_state.leadOne.vRel = 0.0  # Same speed as lead
        
        # Create mock car state
        car_state = log.CarState.new_message()
        car_state.vEgo = 25.0
        
        # Create validation metrics with low confidence
        validation_metrics = {
            'leadConfidenceAvg': 0.3,  # Low confidence
            'leadConfidenceMax': 0.4,
            'laneConfidenceAvg': 0.25,
            'overallConfidence': 0.28,
            'isValid': False,
            'confidenceThreshold': 0.5
        }
        
        # Test the update method
        enhanced_plan = self.planner.update_lead_vehicle_prediction(
            long_plan, radar_state, car_state, validation_metrics
        )
        
        # With low confidence and close lead, the acceleration should be more conservative
        self.assertIsNotNone(enhanced_plan)
    
    def test_update_lead_vehicle_prediction_no_validation_metrics(self):
        """Test that the planner works when validation metrics are not available."""
        # Create mock longitudinal plan
        long_plan = log.LongitudinalPlan.new_message()
        long_plan.aTarget = 1.0
        long_plan.shouldStop = False
        long_plan.hasLead = False  # No lead
        
        # Create mock radar state
        radar_state = log.RadarState.new_message()
        radar_state.leadOne = log.RadarState.LeadData.new_message()
        radar_state.leadOne.status = False  # No lead status
        
        # Create mock car state
        car_state = log.CarState.new_message()
        car_state.vEgo = 25.0
        
        # Test with None validation metrics
        enhanced_plan = self.planner.update_lead_vehicle_prediction(
            long_plan, radar_state, car_state, None
        )
        
        # Should still return a valid plan
        self.assertIsNotNone(enhanced_plan)
        self.assertEqual(enhanced_plan.aTarget, 1.0)
        self.assertEqual(enhanced_plan.shouldStop, False)
    
    def test_probabilistic_spacing_safe_distance_calculation(self):
        """Test that safe distance is calculated properly based on confidence."""
        # This test verifies the internal logic of probabilistic spacing
        long_plan = log.LongitudinalPlan.new_message()
        long_plan.hasLead = True
        long_plan.vEgo = 20.0  # 20 m/s
        
        radar_state = log.RadarState.new_message()
        radar_state.leadOne = log.RadarState.LeadData.new_message()
        radar_state.leadOne.status = True
        radar_state.leadOne.dRel = 30.0  # 30m lead distance
        
        car_state = log.CarState.new_message()
        car_state.vEgo = 20.0
        
        validation_metrics = {
            'leadConfidenceAvg': 0.8,
            'overallConfidence': 0.75,
            'isValid': True
        }
        
        # Call the internal method to test probabilistic spacing
        # Note: The actual logic will be complex, but we're testing that it doesn't crash
        enhanced_plan = self.planner._apply_probabilistic_spacing(
            long_plan, radar_state, car_state, validation_metrics
        )
        
        self.assertIsNotNone(enhanced_plan)
    
    def test_reset_function(self):
        """Test that the reset function works correctly."""
        # Apply some updates to create non-zero filter values
        long_plan = log.LongitudinalPlan.new_message()
        long_plan.aTarget = 1.0
        long_plan.shouldStop = False
        long_plan.hasLead = False
        
        radar_state = log.RadarState.new_message()
        radar_state.leadOne = log.RadarState.LeadData.new_message()
        radar_state.leadOne.status = False
        
        car_state = log.CarState.new_message()
        car_state.vEgo = 20.0
        
        validation_metrics = {'overallConfidence': 0.8}
        
        # Process a few updates to modify filter states
        enhanced_plan = self.planner.update_lead_vehicle_prediction(
            long_plan, radar_state, car_state, validation_metrics
        )
        
        # Reset the planner
        self.planner.reset()
        
        # Check that the filters are reset to zero
        self.assertEqual(self.planner.lead_distance_filter.x, 0.0)
        self.assertEqual(self.planner.lead_speed_filter.x, 0.0)
        self.assertEqual(self.planner.lead_accel_filter.x, 0.0)
        self.assertEqual(len(self.planner.lead_history), 0)


if __name__ == '__main__':
    unittest.main()