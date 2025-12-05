#!/usr/bin/env python3
"""
Test suite for monitoring, safety checks, and fallback mechanisms
for the optimized algorithms in sunnypilot.
"""

import pytest
from unittest.mock import Mock, patch  # noqa: TID251
from opendbc.car.honda.values import CAR
from opendbc.car.honda.interface import CarInterface
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.lib.thermal_manager import ThermalManager


class TestSafetyMonitoring:
  """Test safety monitoring and anomaly detection"""

  def test_radar_reliability_anomaly_detection(self):
    """Test for detecting anomalies in radar reliability calculations"""
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
    planner = LongitudinalPlanner(CP, CP_SP)

    # Test for extremely inconsistent reliability values
    # In normal operation, similar leads should have similar reliability
    lead1 = Mock()
    lead1.status = True
    lead1.dRel = 50.0
    lead1.vRel = 0.0
    lead1.aLeadK = 0.0
    lead1.snr = 10.0
    lead1.age = 10

    reliability1 = planner._calculate_radar_reliability(lead1)

    # Same lead with tiny variation should have similar reliability
    lead2 = Mock()
    lead2.status = True
    lead2.dRel = 50.1  # Very small difference
    lead2.vRel = 0.1
    lead2.aLeadK = 0.1
    lead2.snr = 10.1
    lead2.age = 10

    reliability2 = planner._calculate_radar_reliability(lead2)

    # The difference should not be too drastic for similar inputs
    assert abs(reliability1 - reliability2) < 0.5, "Similar leads should have reasonably similar reliability values"

  def test_thermal_management_safety_validation(self):
    """Test validation of thermal management decisions"""
    thermal_manager = ThermalManager()

    # Verify that thermal status bounds are respected
    from cereal import log

    # Test all thermal status levels to ensure appropriate action
    test_statuses = [
        (log.DeviceState.ThermalStatus.green, "safe"),
        (log.DeviceState.ThermalStatus.yellow, "conservative"),
        (log.DeviceState.ThermalStatus.red, "conservative"),
        (log.DeviceState.ThermalStatus.danger, "very_conservative")
    ]

    # Just verify we can handle all statuses without errors
    for status, _expected_behavior in test_statuses:
      device_state = Mock()
      device_state.thermalStatus = status
      device_state.gpuTemp = 60.0  # Set a proper numeric value, not a Mock

      sm = Mock()
      sm.recv_frame = {'deviceState': 1}
      sm.updated = {'deviceState': True}
      sm.get = Mock(return_value=device_state)  # Use .get method like in other tests
      sm.__getitem__ = Mock(return_value=device_state)
      CS = Mock()

      # Should not raise an exception
      try:
        thermal_manager.apply_gpu_management(sm, CS)
      except Exception as e:
        pytest.fail(f"Thermal management failed for status {status}: {e}")


class TestFallbackMechanisms:
  """Test fallback mechanisms for safety-critical systems"""

  def test_radar_reliability_fallback(self):
    """Test fallback when radar data is invalid or missing"""
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
    planner = LongitudinalPlanner(CP, CP_SP)

    # Test with completely missing attributes (should handle gracefully)
    invalid_lead = Mock()
    invalid_lead.status = False  # This should make it return 0.0 immediately
    # The simplified method should handle invalid status gracefully

    try:
      reliability = planner._calculate_radar_reliability(invalid_lead)
      # Invalid leads should return 0.0
      assert reliability == 0.0, f"Invalid leads should return 0.0, got {reliability}"
    except Exception as e:
      pytest.fail(f"Radar reliability calculation should handle invalid leads gracefully: {e}")


  def test_thermal_management_error_handling(self):
    """Test thermal management error handling"""
    thermal_manager = ThermalManager()

    # Test with None device state values
    device_state = Mock()
    device_state.thermalStatus = None
    device_state.gpuTemp = None

    sm = Mock()
    sm.recv_frame = {'deviceState': 1}
    sm.updated = {'deviceState': True}
    sm.get = Mock(return_value=device_state)  # Use .get method like in other tests
    sm.__getitem__ = Mock(return_value=device_state)
    CS = Mock()

    # Should handle None values gracefully
    try:
      thermal_manager.apply_gpu_management(sm, CS)
    except Exception as e:
      pytest.fail(f"Thermal management should handle None values gracefully: {e}")


class TestPerformanceMonitoring:
  """Test performance monitoring capabilities"""

  def test_calculation_performance_tracking(self):
    """Test that we can track performance improvements"""
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
    planner = LongitudinalPlanner(CP, CP_SP)

    import time

    # Create a standard test lead
    test_lead = Mock()
    test_lead.status = True
    test_lead.dRel = 50.0
    test_lead.vRel = 5.0
    test_lead.aLeadK = 1.0
    test_lead.snr = 15.0
    test_lead.age = 8

    # Time the calculation
    start_time = time.perf_counter()
    for _ in range(1000):
      planner._calculate_radar_reliability(test_lead)
    end_time = time.perf_counter()

    total_time = end_time - start_time
    avg_time = total_time / 1000

    # Should complete in reasonable time (performance test)
    # This is more of a regression test to ensure it doesn't become too slow
    assert avg_time < 0.001, f"Calculation should be fast (<1ms), took {avg_time*1000:.2f}ms"

  def test_thermal_prediction_tracking(self):
    """Test thermal prediction accuracy tracking"""
    thermal_manager = ThermalManager()

    # Test that we can calculate temperature trends
    # Initialize previous temperature
    thermal_manager._prev_gpu_temp = 60.0

    # Create mock device state with current temperature
    device_state = Mock()
    device_state.thermalStatus = 0  # green
    device_state.gpuTemp = 65.0

    sm = Mock()
    sm.recv_frame = {'deviceState': 1}
    sm.updated = {'deviceState': True}
    sm.get = Mock(return_value=device_state)  # Use .get method like in other tests
    sm.__getitem__ = Mock(return_value=device_state)
    CS = Mock()

    # Apply thermal management to trigger trend calculation
    thermal_manager.apply_gpu_management(sm, CS)

    # Verify that the trend logic works (internal state updated)
    assert thermal_manager._prev_gpu_temp == 65.0


class TestSystemIntegration:
  """Test integration between different optimized components"""

  def test_combined_system_behavior(self):
    """Test that all systems work together safely"""
    # This test verifies that the two main optimized systems can work together

    # 1. Test radar reliability calculation
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
    planner = LongitudinalPlanner(CP, CP_SP)
    lead = Mock()
    lead.status = True
    lead.dRel = 50.0
    lead.vRel = 5.0
    lead.aLeadK = 1.0
    lead.snr = 10.0
    lead.age = 5

    reliability = planner._calculate_radar_reliability(lead)
    assert 0.1 <= reliability <= 1.0, "Reliability should be bounded"

    # 2. Test thermal management
    thermal_manager = ThermalManager()
    device_state = Mock()
    device_state.thermalStatus = 0  # green
    device_state.gpuTemp = 60.0

    sm = Mock()
    sm.recv_frame = {'deviceState': 1}
    sm.updated = {'deviceState': True}
    sm.get = Mock(return_value=device_state)  # Use .get method like in other tests
    sm.__getitem__ = Mock(return_value=device_state)
    CS = Mock()

    # Should not fail when applying thermal management
    try:
      thermal_manager.apply_gpu_management(sm, CS)
    except Exception as e:
      pytest.fail(f"Thermal management failed: {e}")

  def test_safety_consistency_across_systems(self):
    """Test that safety behavior is consistent across systems"""
    # All systems should default to safe behavior when uncertain

    # Radar: invalid lead -> 0.0 reliability (safe)
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
    planner = LongitudinalPlanner(CP, CP_SP)
    invalid_lead = Mock(status=False)
    reliability = planner._calculate_radar_reliability(invalid_lead)
    assert reliability == 0.0, "Invalid leads should have 0 reliability"

    # Thermal: dangerous conditions -> conservative mode (safe)
    thermal_manager = ThermalManager()
    device_state_danger = Mock()
    device_state_danger.thermalStatus = 3  # danger level
    device_state_danger.gpuTemp = 80.0  # Set a proper numeric value, not a Mock

    sm = Mock()
    sm.recv_frame = {'deviceState': 1}
    sm.updated = {'deviceState': True}
    sm.get = Mock(return_value=device_state_danger)
    sm.__getitem__ = Mock(return_value=device_state_danger)
    CS = Mock()

    # Should apply conservative thermal management
    with patch.object(thermal_manager, '_apply_gpu_ondemand_mode') as mock_ondemand:
      thermal_manager.apply_gpu_management(sm, CS)
      # In danger state, ondemand should be called for safety
      mock_ondemand.assert_called_once()




if __name__ == "__main__":
  raise RuntimeError("pytest.main is banned, run with `pytest {__file__}` instead")
