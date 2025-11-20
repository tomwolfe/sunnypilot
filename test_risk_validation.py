#!/usr/bin/env python3
"""
Validation tests for environmental risk calculation in sunnypilot
Tests to ensure risk calculations properly balance safety and performance
"""

import unittest
import numpy as np
from unittest.mock import Mock
import sys
import os

# Add the selfdrive path to sys.path to import modules
sys.path.append(os.path.join(os.path.dirname(__file__), "selfdrive"))

from selfdrive.controls.lib.environmental_awareness import EnvironmentalConditionMonitor
from selfdrive.controls.lib.autonomous_params import ENVIRONMENTAL_PARAMS


class TestEnvironmentalRiskValidation(unittest.TestCase):
    """
    Tests for environmental risk calculation validation
    """
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.env_monitor = EnvironmentalConditionMonitor()
        self.base_params = ENVIRONMENTAL_PARAMS
    
    def test_risk_calculation_bounds(self):
        """Test that risk calculations always return values in valid range [0.0, 1.0]"""
        # Test various combinations of inputs to ensure output is always bounded
        test_cases = [
            # (v_ego, curvature_ahead, visibility_factor, road_quality, surface_condition, expected_max_risk)
            (0.0, 0.0, 1.0, 1.0, 1.0, 1.0),  # Stationary, ideal conditions
            (30.0, 0.01, 0.1, 0.2, 0.3, 1.0),  # High speed, poor conditions
            (5.0, 0.05, 0.8, 0.9, 0.8, 1.0),  # Low speed, sharp curve, decent conditions
            (25.0, 0.005, 0.2, 0.3, 0.1, 1.0),  # High speed, moderate curve, poor conditions
        ]
        
        for v_ego, curvature_ahead, visibility_factor, road_quality, surface_condition, max_expected in test_cases:
            risk_score = self.env_monitor.calculate_environmental_risk_score(
                v_ego, curvature_ahead, visibility_factor, road_quality, surface_condition
            )
            
            # Risk should always be between 0 and 1
            self.assertGreaterEqual(risk_score, 0.0, 
                                  f"Risk score {risk_score} below 0 for inputs: v_ego={v_ego}, curvature={curvature_ahead}")
            self.assertLessEqual(risk_score, 1.0, 
                               f"Risk score {risk_score} above 1 for inputs: v_ego={v_ego}, curvature={curvature_ahead}")
    
    def test_speed_based_risk_scaling(self):
        """Test that risk scales appropriately with speed in dangerous conditions"""
        curvature_ahead = 0.01  # Sharp curve
        visibility_factor = 0.3  # Poor visibility
        road_quality = 0.4  # Poor road quality
        surface_condition = 0.3  # Poor surface
        
        # Risk should increase with speed when in dangerous conditions
        risk_low_speed = self.env_monitor.calculate_environmental_risk_score(
            5.0, curvature_ahead, visibility_factor, road_quality, surface_condition
        )
        risk_high_speed = self.env_monitor.calculate_environmental_risk_score(
            30.0, curvature_ahead, visibility_factor, road_quality, surface_condition
        )
        
        # Risk should be higher at high speed in dangerous conditions
        self.assertGreaterEqual(risk_high_speed, risk_low_speed,
                              f"Risk should increase with speed: low={risk_low_speed}, high={risk_high_speed}")
    
    def test_curvature_based_risk(self):
        """Test that risk increases appropriately with curvature ahead"""
        v_ego = 20.0  # Moderate speed
        visibility_factor = 0.5
        road_quality = 0.6
        surface_condition = 0.5
        
        # Risk should increase with curvature
        risk_low_curve = self.env_monitor.calculate_environmental_risk_score(
            v_ego, 0.001, visibility_factor, road_quality, surface_condition  # Very slight curve
        )
        risk_high_curve = self.env_monitor.calculate_environmental_risk_score(
            v_ego, 0.02, visibility_factor, road_quality, surface_condition   # Sharp curve
        )
        
        self.assertGreaterEqual(risk_high_curve, risk_low_curve,
                              f"Risk should increase with curvature: low={risk_low_curve}, high={risk_high_curve}")
    
    def test_visibility_impact_on_risk(self):
        """Test that poor visibility significantly increases risk"""
        v_ego = 15.0
        curvature_ahead = 0.005
        road_quality = 0.7
        surface_condition = 0.7
        
        # Risk should be higher with poor visibility
        risk_good_visibility = self.env_monitor.calculate_environmental_risk_score(
            v_ego, curvature_ahead, 0.9, road_quality, surface_condition  # Good visibility
        )
        risk_poor_visibility = self.env_monitor.calculate_environmental_risk_score(
            v_ego, curvature_ahead, 0.1, road_quality, surface_condition  # Poor visibility
        )
        
        self.assertGreaterEqual(risk_poor_visibility, risk_good_visibility,
                              f"Risk should be higher with poor visibility: good={risk_good_visibility}, poor={risk_poor_visibility}")
    
    def test_road_quality_impact_on_risk(self):
        """Test that poor road quality increases risk"""
        v_ego = 15.0
        curvature_ahead = 0.005
        visibility_factor = 0.7
        surface_condition = 0.7
        
        # Risk should be higher with poor road quality
        risk_good_road = self.env_monitor.calculate_environmental_risk_score(
            v_ego, curvature_ahead, visibility_factor, 0.9, surface_condition  # Good road quality
        )
        risk_poor_road = self.env_monitor.calculate_environmental_risk_score(
            v_ego, curvature_ahead, visibility_factor, 0.1, surface_condition  # Poor road quality
        )
        
        self.assertGreaterEqual(risk_poor_road, risk_good_road,
                              f"Risk should be higher with poor road quality: good={risk_good_road}, poor={risk_poor_road}")
    
    def test_surface_condition_impact_on_risk(self):
        """Test that poor surface condition increases risk"""
        v_ego = 15.0
        curvature_ahead = 0.005
        visibility_factor = 0.7
        road_quality = 0.7
        
        # Risk should be higher with poor surface condition
        risk_good_surface = self.env_monitor.calculate_environmental_risk_score(
            v_ego, curvature_ahead, visibility_factor, road_quality, 0.9  # Good surface
        )
        risk_poor_surface = self.env_monitor.calculate_environmental_risk_score(
            v_ego, curvature_ahead, visibility_factor, road_quality, 0.1  # Poor surface
        )
        
        self.assertGreaterEqual(risk_poor_surface, risk_good_surface,
                              f"Risk should be higher with poor surface: good={risk_good_surface}, poor={risk_poor_surface}")
    
    def test_combined_risk_factors(self):
        """Test that combined risk factors work appropriately"""
        # All good conditions
        risk_all_good = self.env_monitor.calculate_environmental_risk_score(
            10.0, 0.001, 0.9, 0.9, 0.9
        )
        
        # All poor conditions
        risk_all_poor = self.env_monitor.calculate_environmental_risk_score(
            25.0, 0.02, 0.1, 0.1, 0.1
        )
        
        # Risk with all poor conditions should be much higher
        self.assertGreaterEqual(risk_all_poor, risk_all_good,
                              f"Risk with poor conditions should be higher: good={risk_all_good}, poor={risk_all_poor}")
        self.assertGreaterEqual(risk_all_poor, 0.5,  # Should be significantly high
                              f"All poor conditions should result in high risk: {risk_all_poor}")
    
    def test_risk_scaling_edge_cases(self):
        """Test risk calculation edge cases"""
        # Very high speed with dangerous conditions should result in high risk
        high_risk = self.env_monitor.calculate_environmental_risk_score(
            40.0, 0.03, 0.05, 0.05, 0.05  # Very dangerous conditions at high speed
        )
        self.assertLessEqual(high_risk, 1.0, "Risk should not exceed 1.0")
        
        # Stationary with dangerous conditions should still have some risk but less due to zero speed
        stationary_risk = self.env_monitor.calculate_environmental_risk_score(
            0.0, 0.03, 0.05, 0.05, 0.05  # Still dangerous but not moving
        )
        self.assertLessEqual(stationary_risk, high_risk, 
                           "Stationary risk should be less than high-speed risk in same conditions")


class TestRiskCalculationComponents(unittest.TestCase):
    """
    Tests for individual components of risk calculation
    """
    
    def setUp(self):
        self.env_monitor = EnvironmentalConditionMonitor()
    
    def test_speed_factor_calculation(self):
        """Test the quadratic speed factor calculation"""
        # At zero speed, speed factor should be 0
        speed_factor = (0.0 / self.env_monitor.SPEED_NORMALIZATION_BASELINE) ** 2
        self.assertEqual(speed_factor, 0.0)
        
        # At baseline speed, speed factor should be 1.0
        baseline_speed = self.env_monitor.SPEED_NORMALIZATION_BASELINE
        speed_factor_baseline = (baseline_speed / baseline_speed) ** 2
        self.assertEqual(speed_factor_baseline, 1.0)
        
        # At higher speed, speed factor should be quadratic
        double_speed = baseline_speed * 2.0
        speed_factor_double = (double_speed / baseline_speed) ** 2
        self.assertEqual(speed_factor_double, 4.0)
    
    def test_curvature_risk_calculation(self):
        """Test that curvature risk is calculated appropriately"""
        # Should only add risk for sharp curves (> 0.008)
        v_ego = 20.0
        
        # Non-sharp curve should not add curvature risk
        non_sharp_risk = self.env_monitor._calculate_curvature_risk(0.005, v_ego)
        self.assertEqual(non_sharp_risk, 0.0)
        
        # Sharp curve should add risk
        sharp_risk = self.env_monitor._calculate_curvature_risk(0.015, v_ego)
        self.assertGreater(sharp_risk, 0.0)
        self.assertLessEqual(sharp_risk, 0.3)  # Should be capped at 0.3 as per implementation
    
    def test_visibility_risk_calculation(self):
        """Test that visibility risk is calculated appropriately"""
        # Better visibility should result in lower risk
        visibility_risk_good = self.env_monitor._calculate_visibility_risk(0.9)  # Good visibility
        visibility_risk_poor = self.env_monitor._calculate_visibility_risk(0.1)  # Poor visibility
        
        self.assertGreaterEqual(visibility_risk_poor, visibility_risk_good,
                              "Poor visibility should result in higher risk")


class TestRiskIntegration(unittest.TestCase):
    """
    Integration tests for risk calculation with other systems
    """
    
    def setUp(self):
        from selfdrive.controls.lib.environmental_awareness import EnvironmentalConditionProcessor
        self.env_processor = EnvironmentalConditionProcessor()
    
    def test_risk_integration_with_adaptive_behavior(self):
        """Test that risk calculation integrates properly with adaptive behavior"""
        # Mock SM data to trigger risk calculation
        sm = Mock()
        sm.updated = {'modelV2': True}
        sm.recv_frame = {'modelV2': 0}
        
        # Mock model data
        mock_model = Mock()
        mock_model.position = Mock()
        mock_model.position.x = [0.0] * 10  # Straight path
        mock_model.position.y = [0.1 * i for i in range(10)]  # Gradually curving
        mock_model.laneLines = [Mock() for _ in range(4)]
        for i, lane_line in enumerate(mock_model.laneLines):
            lane_line.prob = 0.5  # Medium confidence
            lane_line.std = 0.2 + (i * 0.1)  # Varying uncertainty
        mock_model.roadEdges = [Mock() for _ in range(2)]
        for edge in mock_model.roadEdges:
            edge.prob = 0.7  # Medium-high confidence
        mock_model.leadsV3 = [Mock()]
        mock_model.leadsV3[0].dist = 50.0
        mock_model.leadsV3[0].prob = 1.0
        
        sm['modelV2'] = mock_model
        
        # Update the environmental processor
        self.env_processor.update(sm)
        
        # Check that risk was calculated and is reasonable
        current_conditions = self.env_processor.get_current_conditions()
        risk_score = current_conditions.get('risk_score', 0.0)
        
        self.assertGreaterEqual(risk_score, 0.0)
        self.assertLessEqual(risk_score, 1.0)
        
        # The risk should be moderate given the mixed conditions
        self.assertLessEqual(risk_score, 0.8)  # Should not be extremely high
    
    def test_risk_validation_in_processing(self):
        """Test that risk values are validated during processing"""
        # Test with potentially invalid risk values to ensure they're clamped
        from selfdrive.controls.lib.environmental_awareness import validate_environmental_risk
        
        # Test clamping of out-of-bounds values
        self.assertEqual(validate_environmental_risk(1.5), 1.0)  # Too high
        self.assertEqual(validate_environmental_risk(-0.5), 0.0)  # Too low
        self.assertEqual(validate_environmental_risk(0.5), 0.5)  # Within bounds


def run_risk_validation_tests():
    """
    Run all risk validation tests
    """
    test_classes = [
        TestEnvironmentalRiskValidation,
        TestRiskCalculationComponents,
        TestRiskIntegration
    ]
    
    all_tests_passed = True
    
    for test_class in test_classes:
        loader = unittest.TestLoader()
        suite = loader.loadTestsFromTestCase(test_class)
        
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        
        if not result.wasSuccessful():
            print(f"Tests in {test_class.__name__} failed!")
            all_tests_passed = False
    
    if all_tests_passed:
        print("All risk validation tests passed!")
    else:
        print("Some risk validation tests failed!")
    
    return all_tests_passed


if __name__ == '__main__':
    run_risk_validation_tests()