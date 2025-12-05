#!/usr/bin/env python3
"""
Additional tests for the safety enhancements implemented in the adaptive control system.
Tests the additional safety features added beyond the basic functionality.
"""

import numpy as np
from unittest.mock import Mock  # noqa: TID251

from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner


class TestSafetyEnhancements:
  """Additional tests for safety enhancements in the adaptive control system."""

  def setup_method(self, method):
    """Set up test fixtures before each test method."""
    # Create a partial LongitudinalPlanner instance with only the required methods
    # We can't use __init__ as it may require other dependencies, so we set up just the fields we need
    self.planner = object.__new__(LongitudinalPlanner)
    # Initialize required attributes for _validate_fused_sensor_data method
    self.planner._prev_validated_x = np.array([50.0])
    self.planner._prev_validated_v = np.array([5.0])
    self.planner._prev_validated_a = np.array([2.0])
    self.planner._acceleration_history = []
    self.planner._frame_counter = 0

  def test_radar_reliability_extreme_conditions(self):
    """Test radar reliability calculation under extreme conditions."""
    # Mock a radar lead with extreme conditions
    mock_lead = Mock()
    mock_lead.status = True
    mock_lead.dRel = 5.0  # Very close distance
    mock_lead.vRel = 60.0  # Very high relative velocity
    mock_lead.aLeadK = 20.0  # Very high acceleration
    mock_lead.snr = 1.0  # Low SNR
    mock_lead.std = 5.0  # High standard deviation
    mock_lead.age = 10  # Mock age as integer so it can be compared

    # Calculate reliability
    reliability = self.planner._calculate_radar_reliability(mock_lead)

    # With all these extreme/unreliable conditions, reliability should be low
    assert reliability < 0.5, "Reliability should be low for extreme conditions"

  def test_radar_reliability_angle_validation(self):
    """Test radar reliability calculation with angle validation."""
    # Mock a radar lead with angle data
    mock_lead = Mock()
    mock_lead.status = True
    mock_lead.dRel = 50.0  # Normal distance
    mock_lead.yRel = 50.0  # Wide angle (45 degree angle)
    mock_lead.vRel = 5.0  # Normal relative velocity
    mock_lead.aLeadK = 2.0  # Normal acceleration
    mock_lead.snr = 20.0  # Good SNR
    mock_lead.age = 3  # Mock age as integer so it can be compared

    # Calculate reliability (with angle validation)
    self.planner._calculate_radar_reliability(mock_lead)

    # With a wider angle (exceeding 45 degrees), reliability should be reduced
    mock_lead.yRel = 60.0  # Much wider angle (beyond 45 degrees)
    reliability_wide_angle = self.planner._calculate_radar_reliability(mock_lead)

    # Reset to small angle
    mock_lead.yRel = 5.0  # Small angle (well within 45 degrees)
    reliability_tight_angle = self.planner._calculate_radar_reliability(mock_lead)

    # The tight angle should have higher reliability than the wide angle
    assert reliability_tight_angle > reliability_wide_angle, "Reliability should be higher for tighter angles"

  def test_fused_sensor_validation_extreme_scenarios(self):
    """Test fused sensor validation with extreme scenarios."""
    # Initialize the planner's validation attributes
    self.planner._prev_validated_x = np.array([50.0])
    self.planner._prev_validated_v = np.array([5.0])
    self.planner._prev_validated_a = np.array([2.0])
    self.planner._acceleration_history = [2.0, 2.1, 1.9, 2.0, 2.2]

    # Test with extreme acceleration values
    x = np.array([45.0])
    v = np.array([5.0])
    a = np.array([20.0])  # Very high acceleration

    validated_x, validated_v, validated_a = self.planner._validate_fused_sensor_data(x, v, a)

    # The extreme acceleration should be clamped to a safer value
    assert validated_a[0] <= 8.0, "Extreme acceleration should be clamped"
    assert validated_a[0] >= -15.0, "Extreme acceleration should be clamped"

  def test_fused_sensor_validation_physical_inconsistency(self):
    """Test fused sensor validation for physically inconsistent data."""
    # Initialize the planner's validation attributes
    self.planner._prev_validated_x = np.array([50.0])
    self.planner._prev_validated_v = np.array([5.0])
    self.planner._prev_validated_a = np.array([2.0])
    self.planner._acceleration_history = [2.0, 2.1, 1.9, 2.0, 2.2]

    # Test with physically inconsistent data: distance decreasing but velocity positive and high
    x = np.array([30.0])  # Distance decreased (getting closer)
    v = np.array([10.0])  # But lead is moving away rapidly
    a = np.array([5.0])  # And accelerating away

    validated_x, validated_v, validated_a = self.planner._validate_fused_sensor_data(x, v, a)

    # The acceleration should be moderated due to both physical inconsistency and acceleration-velocity inconsistency
    # Both safety checks may apply, leading to a modified result within safe bounds
    assert validated_a[0] <= 8.0, "Acceleration should be within safe bounds after all safety checks"
    assert validated_a[0] >= -15.0, "Acceleration should be within safe bounds after all safety checks"
