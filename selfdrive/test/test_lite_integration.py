import pytest
import numpy as np


# Using builtin object as a simple mock class since unittest import is banned
class Mock:
  def __init__(self, **kwargs):
    self.__dict__.update(kwargs)

  def __getattr__(self, name):
    # Return a new Mock for any attribute that doesn't exist
    attr = Mock()
    setattr(self, name, attr)
    return attr


import time

from openpilot.selfdrive.monitoring.lite_monitoring import LightweightSystemMonitor, LightweightSafetyChecker
from openpilot.selfdrive.controls.lib.lite_control import LightweightAdaptiveGainScheduler


# Mock FirstOrderFilter for predictable behavior in tests
class MockFirstOrderFilter:
  def __init__(self, x0, k, dt):
    self.x = x0
    self.k = k
    self.dt = dt

  def update(self, new_val):
    self.x = new_val


class TestLiteIntegration:
  @pytest.fixture
  def mock_car_params(self):
    CP = Mock()
    # Mocking longitudinal tuning for interpolation
    CP.longitudinalTuning.kpBP = [0.0, 10.0, 20.0]
    CP.longitudinalTuning.kpV = [0.1, 0.5, 1.0]
    CP.longitudinalTuning.kiBP = [0.0, 10.0, 20.0]
    CP.longitudinalTuning.kiV = [0.01, 0.03, 0.05]
    CP.longitudinalTuning.kf = 0.05
    # Mocking lateral tuning for interpolation
    CP.lateralTuning.pid.kpBP = [0.0, 10.0, 20.0]
    CP.lateralTuning.pid.kpV = [0.2, 0.8, 1.5]
    CP.lateralTuning.pid.kiBP = [0.0, 10.0, 20.0]
    CP.lateralTuning.pid.kiV = [0.02, 0.04, 0.06]
    CP.lateralTuning.pid.kdV = [0.0, 0.1, 0.2]
    CP.lateralTuning.pid.kdBP = [0.0, 10.0, 20.0]
    CP.lateralTuning.pid.kf = 0.1
    CP.carFingerprint = "TOYOTA_COROLLA"
    CP.stopAccel = -3.5  # Example stopAccel for testing
    CP.steerMax = 1.5  # Example steerMax for safety_checker

    return CP

  @pytest.fixture
  def mock_device_state(self):
    ds = Mock()
    ds.cpuTempC = [60.0]
    ds.gpuTempC = 55.0
    ds.cpuUsagePercent = 30.0
    ds.memoryUsagePercent = 40.0
    ds.freeSpacePercent = 80.0
    ds.canMonoTimes = [int(time.monotonic() * 1e9 - 1e8)]  # 0.1 seconds ago
    return ds

  @pytest.fixture
  def mock_car_state(self):
    cs = Mock()
    cs.vEgo = 15.0
    return cs

  @pytest.fixture
  def mock_actuators(self):
    actuators = Mock()
    actuators.accel = 0.0
    actuators.steer = 0.0
    actuators.curvature = 0.0
    return actuators

  @pytest.fixture
  def mock_radar_state(self):
    rs = Mock()
    rs.leadOne.status = False
    return rs

  @pytest.fixture
  def setup_modules(self, mock_car_params, mocker):
    # Patch FirstOrderFilter for LightweightAdaptiveGainScheduler
    mocker.patch('openpilot.selfdrive.controls.lib.lite_control.FirstOrderFilter', new=MockFirstOrderFilter)

    monitor = LightweightSystemMonitor()
    scheduler = LightweightAdaptiveGainScheduler(mock_car_params)
    safety_checker = LightweightSafetyChecker(mock_car_params)
    return monitor, scheduler, safety_checker

  def test_thermal_state_impacts_gains_and_safety(self, setup_modules, mock_device_state, mock_car_state, mock_actuators, mock_radar_state, mock_car_params):
    monitor, scheduler, safety_checker = setup_modules

    # --- Scenario 1: Normal operation, no thermal stress ---
    mock_device_state.cpuTempC = [60.0]  # Thermal state should be low (0.0 to 0.5 range)
    thermal_state = monitor.calculate_thermal_state(mock_device_state)
    # Based on interp logic: 40C=0, 70C=0.5. 60C is 2/3 of the way, so 0.5 * (2/3) = 0.333
    assert np.isclose(thermal_state, 0.3333333333333333, rtol=1e-6)

    gains_normal = scheduler.get_adaptive_gains(mock_car_state.vEgo, thermal_state)
    # Verify gains are not significantly reduced due to thermal state
    # For kf: base is 0.05. thermal factor is max(0.85, 1.0 - 0.333*0.15) = max(0.85, 1.0 - 0.05) = 0.95
    # speed_factor at 15m/s (TOYOTA_COROLLA long_speed_gain_factor=35): 1.0 + (15/35) = 1.428. Capped at 1.6. So 1.428
    # Final kf = 0.05 * (1.0 + 15/35) * 0.95 = 0.05 * 1.4285714285714286 * 0.95 = 0.06785714285714287
    assert np.isclose(gains_normal['longitudinal']['kf'], 0.06785714285714287, rtol=1e-6)

    # No safety violations
    mock_actuators.accel = 0.5
    mock_actuators.steer = 0.1

    # Initialize safety_checker's previous state to reflect current actuators for the first check
    safety_checker._prev_steer = mock_actuators.steer
    safety_checker._prev_time = time.monotonic()

    safety_report_normal = safety_checker.validate_outputs(mock_actuators, mock_car_state, mock_radar_state)
    assert safety_report_normal['safe'] is True
    assert safety_report_normal['recommended_action'] == 'continue'

    # --- Scenario 2: High thermal stress ---
    mock_device_state.cpuTempC = [90.0]  # Critical thermal state
    mock_device_state.gpuTempC = 90.0
    thermal_state_critical = monitor.calculate_thermal_state(mock_device_state)
    assert np.isclose(thermal_state_critical, 1.0, rtol=1e-6)

    gains_thermal = scheduler.get_adaptive_gains(mock_car_state.vEgo, thermal_state_critical)
    # Verify gains are reduced due to thermal state
    # Thermal factor for long: max(0.85, 1.0 - 1.0 * 0.15) = 0.85
    # Thermal factor for lat: max(0.9, 1.0 - 1.0 * 0.1) = 0.9
    # Final kf = 0.05 * 1.428 * 0.85 = 0.06071428571428572
    assert np.isclose(gains_thermal['longitudinal']['kf'], 0.06071428571428572, rtol=1e-6)
    # For lateral, base_lat_kp at 15m/s (0.2, 0.8, 1.5) -> interp(15, [0,10,20], [0.2,0.8,1.5]) = 1.15
    # lat_speed_factor (TOYOTA_COROLLA lat_speed_gain_factor=55): 1.0 + (15/55) = 1.2727. Capped at 1.4. So 1.2727
    # Final lat_kp = 1.15 * 1.2727 * 0.9 = 1.3172727272727272
    assert np.isclose(gains_thermal['lateral']['kp'], 1.3172727272727272, rtol=1e-6)

    # --- Scenario 3: Safety check triggers deceleration due to high accel ---
    mock_actuators.accel = 4.0  # Exceeds max_long_accel (3.0)
    safety_report_accel_violation = safety_checker.validate_outputs(mock_actuators, mock_car_state, mock_radar_state)
    assert safety_report_accel_violation['safe'] is False
    assert safety_report_accel_violation['recommended_action'] == 'decelerate'

    fail_safe_cmd = safety_checker.trigger_fail_safe(safety_report_accel_violation, mock_car_state)
    assert fail_safe_cmd['enabled'] is True
    assert fail_safe_cmd['acceleration'] == mock_car_params.stopAccel  # Should use CP.stopAccel

  def test_stale_device_state_impacts_system_monitor_health(self, setup_modules, mock_device_state, mock_car_state, mocker):
    monitor, scheduler, safety_checker = setup_modules

    # Test the direct check for can_bus_ok
    mocker.patch('time.monotonic', return_value=100.0)
    mock_device_state.canMonoTimes = [int(96.0 * 1e9)]  # 4 seconds ago, stale
    health_report = monitor.check_system_health(mock_device_state, mock_car_state)
    assert health_report['can_bus_ok'] is False

    # Let's ensure the monitor still calculates thermal state correctly when valid data is provided
    # This doesn't directly link to can_bus_ok in thermal_state calculation, but ensures it still works.
    mock_device_state.cpuTempC = [85.0]
    thermal_state_warm = monitor.calculate_thermal_state(mock_device_state)
    # Based on interp logic: 70C=0.5, 90C=1.0. 85C is 3/4 of the way, so 0.5 + 0.5 * (3/4) = 0.875
    assert np.isclose(thermal_state_warm, 0.875)
