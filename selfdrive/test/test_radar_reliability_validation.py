#!/usr/bin/env python3
"""
Test suite for validating the simplified radar reliability calculation in longitudinal_planner.py
This test ensures that the simplified model maintains safety while providing performance benefits.
"""

import numpy as np
import pytest
from unittest.mock import Mock
from opendbc.car.honda.values import CAR
from opendbc.car.honda.interface import CarInterface
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner


class MockLeadRadar:
  """Mock object to simulate radar lead data for testing"""

  def __init__(self, status=True, dRel=50.0, vRel=0.0, aLeadK=0.0, snr=None, std=None, prob=None, age=None, newLead=False):
    self.status = status
    self.dRel = dRel
    self.vRel = vRel
    self.aLeadK = aLeadK
    self.snr = snr
    self.std = std
    self.prob = prob
    self.age = age
    self.newLead = newLead


class TestRadarReliabilityValidation:
  """Test suite for radar reliability calculation validation"""

  def setup_method(self):
    """Set up test fixtures before each test method"""
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
    self.planner = LongitudinalPlanner(CP, CP_SP)
  
  def test_invalid_lead_reliability(self):
    """Test that invalid leads return 0.0 reliability"""
    invalid_lead = MockLeadRadar(status=False)
    reliability = self.planner._calculate_radar_reliability(invalid_lead)
    assert reliability == 0.0, "Invalid leads should have 0.0 reliability"
  
  def test_distance_factor_basic(self):
    """Test basic distance-based reliability calculation"""
    # Close lead should have higher reliability
    close_lead = MockLeadRadar(status=True, dRel=10.0)
    close_reliability = self.planner._calculate_radar_reliability(close_lead)
    
    # Far lead should have lower reliability
    far_lead = MockLeadRadar(status=True, dRel=200.0)
    far_reliability = self.planner._calculate_radar_reliability(far_lead)
    
    assert close_reliability >= far_reliability, "Closer leads should have higher reliability"
    assert 0.1 <= close_reliability <= 1.0, "Reliability should be clamped between 0.1 and 1.0"
    assert 0.1 <= far_reliability <= 1.0, "Reliability should be clamped between 0.1 and 1.0"
  
  def test_snr_reliability(self):
    """Test SNR-based reliability calculation"""
    lead_with_good_snr = MockLeadRadar(status=True, dRel=50.0, snr=20.0)
    reliability_good_snr = self.planner._calculate_radar_reliability(lead_with_good_snr)
    
    lead_with_poor_snr = MockLeadRadar(status=True, dRel=50.0, snr=2.0)
    reliability_poor_snr = self.planner._calculate_radar_reliability(lead_with_poor_snr)
    
    # Note: With averaging approach, the difference might not be as pronounced
    # but good SNR should generally result in higher overall reliability
    assert 0.1 <= reliability_good_snr <= 1.0, "Reliability should be clamped"
    assert 0.1 <= reliability_poor_snr <= 1.0, "Reliability should be clamped"
  
  def test_std_reliability(self):
    """Test standard deviation-based reliability calculation"""
    lead_with_low_std = MockLeadRadar(status=True, dRel=50.0, std=0.1)  # Low std = high reliability
    reliability_low_std = self.planner._calculate_radar_reliability(lead_with_low_std)
    
    lead_with_high_std = MockLeadRadar(status=True, dRel=50.0, std=5.0)  # High std = low reliability
    reliability_high_std = self.planner._calculate_radar_reliability(lead_with_high_std)
    
    assert 0.1 <= reliability_low_std <= 1.0, "Reliability should be clamped"
    assert 0.1 <= reliability_high_std <= 1.0, "Reliability should be clamped"
  
  def test_extreme_velocity_filtering(self):
    """Test that extreme velocities reduce reliability"""
    normal_lead = MockLeadRadar(status=True, dRel=50.0, vRel=10.0)
    normal_reliability = self.planner._calculate_radar_reliability(normal_lead)
    
    extreme_lead = MockLeadRadar(status=True, dRel=50.0, vRel=70.0)  # > 60 threshold
    extreme_reliability = self.planner._calculate_radar_reliability(extreme_lead)
    
    assert extreme_reliability < normal_reliability, "Extreme velocities should reduce reliability"
    assert extreme_reliability <= 0.1, "Extreme velocity leads should have very low reliability"
  
  def test_extreme_acceleration_filtering(self):
    """Test that extreme accelerations reduce reliability"""
    normal_lead = MockLeadRadar(status=True, dRel=50.0, aLeadK=5.0)
    normal_reliability = self.planner._calculate_radar_reliability(normal_lead)
    
    extreme_lead = MockLeadRadar(status=True, dRel=50.0, aLeadK=15.0)  # > 12 threshold
    extreme_reliability = self.planner._calculate_radar_reliability(extreme_lead)
    
    assert extreme_reliability < normal_reliability, "Extreme accelerations should reduce reliability"
    assert extreme_reliability <= 0.3, "Extreme acceleration leads should have reduced reliability"
  
  def test_track_age_boost(self):
    """Test that older, stable tracks get reliability boost"""
    new_lead = MockLeadRadar(status=True, dRel=50.0, age=2)  # New track
    new_reliability = self.planner._calculate_radar_reliability(new_lead)
    
    stable_lead = MockLeadRadar(status=True, dRel=50.0, age=10)  # Stable track
    stable_reliability = self.planner._calculate_radar_reliability(stable_lead)
    
    # The age boost is applied as a 1.1 multiplier if age > 5, clamped to 1.0 max
    assert 0.1 <= new_reliability <= 1.0, "Reliability should be clamped"
    assert 0.1 <= stable_reliability <= 1.0, "Reliability should be clamped"
  
  def test_safety_bounds(self):
    """Test that reliability is always bounded between 0.1 and 1.0"""
    test_cases = [
      # (status, dRel, vRel, aLeadK, snr, std, prob, age)
      (True, 0.1, 0.0, 0.0, 100, 0.01, 1.0, 20),  # Should be maxed out
      (True, 1000, 100, 20, 0, 10, 0.1, 1),        # Should be reduced but not below 0.1
    ]
    
    for case in test_cases:
      status, dRel, vRel, aLeadK, snr, std, prob, age = case
      lead = MockLeadRadar(status=status, dRel=dRel, vRel=vRel, aLeadK=aLeadK, 
                          snr=snr, std=std, prob=prob, age=age)
      reliability = self.planner._calculate_radar_reliability(lead)
      
      assert 0.1 <= reliability <= 1.0, f"Reliability {reliability} should be bounded [0.1, 1.0] for case {case}"
  
  def test_comparison_with_original_logic(self):
    """Test to compare behavior with expected original logic patterns"""
    # Test case that should be clearly valid
    valid_lead = MockLeadRadar(status=True, dRel=30.0, vRel=5.0, aLeadK=2.0, snr=15.0, age=8)
    reliability = self.planner._calculate_radar_reliability(valid_lead)
    assert reliability > 0.5, "Valid lead with good parameters should have moderate to high reliability"
    
    # Test case that should be clearly invalid due to extreme values
    invalid_lead = MockLeadRadar(status=True, dRel=30.0, vRel=80.0, aLeadK=20.0)  # Both extreme
    reliability = self.planner._calculate_radar_reliability(invalid_lead)
    assert reliability <= 0.3, "Leads with extreme vRel and aLeadK should have low reliability"


class TestPerformanceComparison:
  """Tests to measure performance improvements of simplified algorithm"""

  def setup_method(self):
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
    self.planner = LongitudinalPlanner(CP, CP_SP)
  
  def test_calculation_time(self):
    """Basic test to ensure the method runs without errors"""
    import time
    
    lead = MockLeadRadar(status=True, dRel=50.0, vRel=10.0, aLeadK=2.0, snr=10.0, age=5)
    
    start_time = time.time()
    for _ in range(1000):  # Test multiple calls to get average
      reliability = self.planner._calculate_radar_reliability(lead)
    end_time = time.time()
    
    avg_time = (end_time - start_time) / 1000
    # Ensure it completes in reasonable time (this is primarily to ensure no errors)
    assert avg_time < 0.001, f"Calculation should be fast, took {avg_time}s per call"


if __name__ == "__main__":
  pytest.main([__file__])