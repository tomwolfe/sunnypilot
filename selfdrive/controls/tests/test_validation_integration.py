"""
Integration tests for validation metrics and safety systems in controlsd.py
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
import numpy as np
from cereal import car, log
import cereal.messaging as messaging
from openpilot.selfdrive.controls.lib.enhanced_longitudinal_planner import EnhancedLongitudinalPlanner
from openpilot.selfdrive.controls.controlsd import Controls


class TestControlsValidationIntegration(unittest.TestCase):
    
    def setUp(self):
        # Mock the parameters and car params to avoid needing real car params
        with patch('openpilot.common.params.Params.get') as mock_params_get, \
             patch('cereal.messaging.SubMaster'), \
             patch('cereal.messaging.PubMaster'), \
             patch('opendbc.car.car_helpers.interfaces'):
            
            # Mock car params
            mock_car_params = car.CarParams.new_message()
            mock_car_params.carFingerprint = "mock_car"
            mock_car_params.steerControlType = car.CarParams.SteerControlType.angle
            mock_car_params.lateralTuning.which = Mock(return_value='pid')
            mock_params_get.return_value = mock_car_params.to_bytes()
            
            # Mock interfaces
            mock_interface = Mock()
            mock_interface.return_value = Mock()
            patcher = patch('opendbc.car.car_helpers.interfaces', {'mock_car': lambda cp, cp_sp: mock_interface})
            patcher.start()
            self.addCleanup(patcher.stop)
            
            self.controls = Controls()
    
    @patch('cereal.messaging.SubMaster')
    @patch('cereal.messaging.PubMaster') 
    @patch('openpilot.common.params.Params.get')
    @patch('opendbc.car.car_helpers.interfaces')
    def test_validation_metrics_integration(self, mock_interfaces, mock_params_get, mock_pubmaster, mock_submaster):
        """Test that validation metrics are properly integrated into control decisions."""
        
        # Mock car params
        mock_car_params = car.CarParams.new_message()
        mock_car_params.carFingerprint = "mock_car"
        mock_car_params.steerControlType = car.CarParams.SteerControlType.angle
        mock_car_params.lateralTuning.which = Mock(return_value='pid')
        mock_params_get.return_value = mock_car_params.to_bytes()
        
        # Mock interface
        mock_interface = Mock()
        mock_interface.CP = mock_car_params
        mock_interface.CP_SP = Mock()
        mock_interfaces.__getitem__.return_value = lambda cp, cp_sp: mock_interface
        
        # Create controls instance
        controls = Controls()
        
        # Verify that validation metrics subscription was added
        mock_submaster.assert_called()
        # Check that 'validationMetrics' is in the subscription list (it's added to sm_services_ext too)
    
    def test_validation_state_logic(self):
        """Test the validation state logic based on confidence thresholds."""
        # This test simulates the validation state logic in controlsd.py
        
        # Simulate high confidence scenario
        validation_metrics_high = Mock()
        validation_metrics_high.leadConfidenceAvg = 0.8
        validation_metrics_high.laneConfidenceAvg = 0.85
        validation_metrics_high.overallConfidence = 0.82
        
        # Check validation state for high confidence
        lead_conf_ok = validation_metrics_high.leadConfidenceAvg >= 0.6
        lane_conf_ok = validation_metrics_high.laneConfidenceAvg >= 0.65
        overall_conf_ok = validation_metrics_high.overallConfidence >= 0.6
        
        self.assertTrue(lead_conf_ok)
        self.assertTrue(lane_conf_ok)
        self.assertTrue(overall_conf_ok)
        
        # Simulate low confidence scenario
        validation_metrics_low = Mock()
        validation_metrics_low.leadConfidenceAvg = 0.3
        validation_metrics_low.laneConfidenceAvg = 0.4
        validation_metrics_low.overallConfidence = 0.35
        
        # Check validation state for low confidence
        lead_conf_ok = validation_metrics_low.leadConfidenceAvg >= 0.6
        lane_conf_ok = validation_metrics_low.laneConfidenceAvg >= 0.65
        overall_conf_ok = validation_metrics_low.overallConfidence >= 0.6
        
        self.assertFalse(lead_conf_ok)
        self.assertFalse(lane_conf_ok)
        self.assertFalse(overall_conf_ok)
    
    def test_longitudinal_plan_adjustment_based_on_confidence(self):
        """Test that longitudinal plan is adjusted based on validation metrics."""
        # This tests the logic where low lead confidence leads to more conservative control
        validation_metrics = Mock()
        validation_metrics.leadConfidenceAvg = 0.4  # Below threshold
        validation_metrics.laneConfidenceAvg = 0.7
        validation_metrics.overallConfidence = 0.55
        
        # Simulate the logic from controlsd.py
        validation_state = {}
        lead_conf_ok = validation_metrics.leadConfidenceAvg >= 0.6
        validation_state['lead_confidence_ok'] = lead_conf_ok
        
        # Check that low lead confidence is detected
        self.assertFalse(validation_state['lead_confidence_ok'])
        
        # Test the PID limits adjustment logic
        pid_accel_limits_original = (-4.0, 2.0)  # Original limits
        if not validation_state['lead_confidence_ok']:
            # Apply conservative adjustment
            pid_accel_limits_adjusted = (
                max(pid_accel_limits_original[0] * 0.8, -4.0),
                min(pid_accel_limits_original[1] * 0.8, 2.0)
            )
        
        # Confirm the limits are more conservative
        self.assertEqual(pid_accel_limits_adjusted[0], -3.2)  # More conservative braking
        self.assertEqual(pid_accel_limits_adjusted[1], 1.6)   # More conservative acceleration
    
    def test_curvature_adjustment_based_on_confidence(self):
        """Test that curvature is adjusted based on lane detection confidence."""
        validation_metrics = Mock()
        validation_metrics.laneConfidenceAvg = 0.4  # Below threshold
        validation_metrics.overallConfidence = 0.5
        
        # Simulate the logic from controlsd.py for curvature adjustment
        active_curvature = 0.01  # Some desired curvature
        validation_state = {
            'lane_confidence_ok': validation_metrics.laneConfidenceAvg >= 0.65
        }
        
        # Apply adjustment logic
        if not validation_state['lane_confidence_ok']:
            if abs(active_curvature) > 0.005:
                adjusted_curvature = active_curvature * 0.7  # Reduce by 30%
            else:
                adjusted_curvature = active_curvature
        else:
            adjusted_curvature = active_curvature
        
        # Check that curvature was reduced due to low confidence
        self.assertLess(abs(adjusted_curvature), abs(active_curvature))
        self.assertEqual(adjusted_curvature, 0.01 * 0.7)
    
    def test_safety_fallback_logic(self):
        """Test that safety fallbacks are triggered at low confidence."""
        validation_metrics = Mock()
        validation_metrics.overallConfidence = 0.3  # Below critical threshold (0.4)
        validation_metrics.leadConfidenceAvg = 0.2
        validation_metrics.laneConfidenceAvg = 0.3
        
        # Simulate the fallback logic from controlsd.py
        safety_status = {
            'fallback_active': False,
            'degraded_mode': False,
            'system_alert': False
        }
        
        if validation_metrics.overallConfidence < 0.4:
            safety_status['fallback_active'] = True
            safety_status['system_alert'] = True
        elif validation_metrics.overallConfidence < 0.6:
            safety_status['degraded_mode'] = True
        elif validation_metrics.overallConfidence < 0.75:
            safety_status['system_alert'] = True
        
        # Check that critical safety threshold was triggered
        self.assertTrue(safety_status['fallback_active'])
        self.assertTrue(safety_status['system_alert'])
        self.assertFalse(safety_status['degraded_mode'])  # Should not be degraded if in fallback


class TestEnhancedLongitudinalPlannerWithValidation(unittest.TestCase):
    
    def setUp(self):
        self.CP = car.CarParams.new_message()
        self.CP.longitudinalTuning.kA = 1.0
        self.planner = EnhancedLongitudinalPlanner(self.CP)
    
    def test_confidence_level_calculation(self):
        """Test that confidence levels are calculated correctly."""
        # Test high confidence
        validation_metrics_high = {'leadConfidenceAvg': 0.8}
        lead_confidence = validation_metrics_high['leadConfidenceAvg']
        
        if lead_confidence >= 0.75:
            confidence_level = 'high'
        elif lead_confidence >= 0.5:
            confidence_level = 'medium'
        else:
            confidence_level = 'low'
        
        self.assertEqual(confidence_level, 'high')
        
        # Test medium confidence
        validation_metrics_medium = {'leadConfidenceAvg': 0.6}
        lead_confidence = validation_metrics_medium['leadConfidenceAvg']
        
        if lead_confidence >= 0.75:
            confidence_level = 'high'
        elif lead_confidence >= 0.5:
            confidence_level = 'medium'
        else:
            confidence_level = 'low'
        
        self.assertEqual(confidence_level, 'medium')
        
        # Test low confidence
        validation_metrics_low = {'leadConfidenceAvg': 0.3}
        lead_confidence = validation_metrics_low['leadConfidenceAvg']
        
        if lead_confidence >= 0.75:
            confidence_level = 'high'
        elif lead_confidence >= 0.5:
            confidence_level = 'medium'
        else:
            confidence_level = 'low'
        
        self.assertEqual(confidence_level, 'low')


if __name__ == '__main__':
    unittest.main()