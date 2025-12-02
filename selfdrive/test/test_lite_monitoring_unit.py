import pytest
import numpy as np
import time
from unittest.mock import Mock, patch

from openpilot.selfdrive.monitoring.lite_monitoring import LightweightSystemMonitor, LightweightSafetyChecker

class TestLightweightSystemMonitor:
    @pytest.fixture
    def monitor(self):
        return LightweightSystemMonitor()

    def test_calculate_thermal_state_cold(self, monitor):
        mock_device_state = Mock()
        mock_device_state.cpuTempC = [30.0]
        mock_device_state.gpuTempC = 30.0
        thermal_state = monitor.calculate_thermal_state(mock_device_state)
        assert thermal_state == 0.0

    def test_calculate_thermal_state_warm_cpu(self, monitor):
        mock_device_state = Mock()
        mock_device_state.cpuTempC = [50.0]
        mock_device_state.gpuTempC = 40.0
        thermal_state = monitor.calculate_thermal_state(mock_device_state)
        # np.interp([0, 40, 70, 90], [0, 0, 0.5, 1.0]) for 50.0 -> 0.0 + (50-40)/(70-40) * (0.5-0.0) = 10/30 * 0.5 = 0.1666
        assert np.isclose(thermal_state, 0.16666666666666666)

    def test_calculate_thermal_state_warm_gpu(self, monitor):
        mock_device_state = Mock()
        mock_device_state.cpuTempC = [40.0]
        mock_device_state.gpuTempC = 50.0
        thermal_state = monitor.calculate_thermal_state(mock_device_state)
        assert np.isclose(thermal_state, 0.16666666666666666)

    def test_calculate_thermal_state_hot(self, monitor):
        mock_device_state = Mock()
        mock_device_state.cpuTempC = [80.0]
        mock_device_state.gpuTempC = 75.0
        thermal_state = monitor.calculate_thermal_state(mock_device_state)
        # np.interp([0, 40, 70, 90], [0, 0, 0.5, 1.0]) for 80.0 -> 0.5 + (80-70)/(90-70) * (1.0-0.5) = 10/20 * 0.5 = 0.75
        assert np.isclose(thermal_state, 0.75)

    def test_calculate_thermal_state_critical(self, monitor):
        mock_device_state = Mock()
        mock_device_state.cpuTempC = [95.0]
        mock_device_state.gpuTempC = 90.0
        thermal_state = monitor.calculate_thermal_state(mock_device_state)
        assert thermal_state == 1.0

    def test_calculate_thermal_state_no_temps(self, monitor):
        mock_device_state = Mock()
        mock_device_state.cpuTempC = []
        mock_device_state.gpuTempC = None
        thermal_state = monitor.calculate_thermal_state(mock_device_state)
        assert thermal_state == 0.0

    def _create_mock_device_state(self, canMonoTimes=None, freeSpacePercent=50.0):
        mock_ds = Mock()
        mock_ds.cpuTempC = [50.0]
        mock_ds.gpuTempC = 50.0
        mock_ds.cpuUsagePercent = 50.0
        mock_ds.memoryUsagePercent = 50.0
        mock_ds.freeSpacePercent = freeSpacePercent
        mock_ds.canMonoTimes = canMonoTimes if canMonoTimes is not None else []
        return mock_ds

    def test_check_system_health_can_bus_ok(self, monitor):
        mock_device_state = self._create_mock_device_state(canMonoTimes=[int(time.monotonic() * 1e9 - 1e8)]) # 0.1 seconds ago
        mock_car_state = Mock()
        with patch('time.monotonic', return_value=time.monotonic()):
            report = monitor.check_system_health(mock_device_state, mock_car_state)
            assert report['can_bus_ok'] is True

    def test_check_system_health_can_bus_stale(self, monitor):
        mock_device_state = self._create_mock_device_state(canMonoTimes=[int(time.monotonic() * 1e9 - 3e9)]) # 3 seconds ago
        mock_car_state = Mock()
        with patch('time.monotonic', return_value=time.monotonic()):
            report = monitor.check_system_health(mock_device_state, mock_car_state)
            assert report['can_bus_ok'] is False

    def test_check_system_health_can_bus_no_messages(self, monitor):
        mock_device_state = self._create_mock_device_state(canMonoTimes=[])
        mock_car_state = Mock()
        with patch('time.monotonic', return_value=time.monotonic()):
            report = monitor.check_system_health(mock_device_state, mock_car_state)
            assert report['can_bus_ok'] is False

    def test_check_system_health_disk_space_ok(self, monitor):
        mock_device_state = self._create_mock_device_state(freeSpacePercent=10.0) # 10% free, which is > 5%
        mock_car_state = Mock()
        with patch('time.monotonic', return_value=time.monotonic()):
            report = monitor.check_system_health(mock_device_state, mock_car_state)
            assert report['disk_space_ok'] is True

    def test_check_system_health_disk_space_low(self, monitor):
        mock_device_state = self._create_mock_device_state(freeSpacePercent=3.0) # 3% free, which is < 5%
        mock_car_state = Mock()
        with patch('time.monotonic', return_value=time.monotonic()):
            report = monitor.check_system_health(mock_device_state, mock_car_state)
            assert report['disk_space_ok'] is False

class TestLightweightSafetyChecker:
    @pytest.fixture
    def safety_checker(self):
        # Create a mock CarParams object that has the necessary attributes
        mock_CP = Mock()
        mock_CP.steerMax = 1.5 # Example steerMax for testing (radians)

        # If any other CP attributes are accessed in _get_vehicle_specific_thresholds,
        # they need to be mocked here too. For now, only steerMax is directly used.
        return LightweightSafetyChecker(mock_CP)

    @pytest.fixture
    def mock_actuators(self):
        actuators = Mock()
        actuators.accel = 0.0
        actuators.steer = 0.0
        actuators.curvature = 0.0
        return actuators

    @pytest.fixture
    def mock_car_state(self):
        car_state = Mock()
        car_state.vEgo = 10.0
        return car_state

    @pytest.fixture
    def mock_radar_state(self):
        radar_state = Mock()
        radar_state.leadOne.status = False
        return radar_state

    def test_steering_rate_no_violation(self, safety_checker, mock_actuators, mock_car_state, mock_radar_state):
        # Initial time for the first call
        initial_monotonic_time = 100.0 # A fixed starting time

        # Patch time.monotonic for the initial setup
        with patch('time.monotonic', return_value=initial_monotonic_time):
            safety_checker.validate_outputs(mock_actuators, mock_car_state, mock_radar_state)

        # Time for the second call, ensuring a specific time_delta
        second_monotonic_time = initial_monotonic_time + 0.1 # 0.1s later

        # Ensure no other violations are possible
        mock_actuators.accel = 0.0
        mock_actuators.curvature = 0.0
        mock_radar_state.leadOne.status = False

        # Simulate a small steering change over a short time that should NOT trigger a violation
        mock_actuators.steer = 0.04
        with patch('time.monotonic', return_value=second_monotonic_time): # Only patch for this one call
            report = safety_checker.validate_outputs(mock_actuators, mock_car_state, mock_radar_state)
            assert report['safe'] is True
            assert 'steering_rate_limit_exceeded' not in report['violations']

    def test_steering_rate_violation(self, safety_checker, mock_actuators, mock_car_state, mock_radar_state):
        initial_monotonic_time = 100.0
        with patch('time.monotonic', return_value=initial_monotonic_time):
            safety_checker.validate_outputs(mock_actuators, mock_car_state, mock_radar_state)

        # Simulate a rapid steering change that SHOULD trigger a violation
        second_monotonic_time = initial_monotonic_time + 0.01 # 0.01s time delta
        mock_actuators.steer = 0.5 # A large change
        with patch('time.monotonic', return_value=second_monotonic_time):
            report = safety_checker.validate_outputs(mock_actuators, mock_car_state, mock_radar_state)
            assert report['safe'] is False
            assert 'steering_rate_limit_exceeded' in report['violations']
            assert report['recommended_action'] == 'disengage'

    def test_steering_rate_race_condition_fix(self, safety_checker, mock_actuators, mock_car_state, mock_radar_state):
        initial_monotonic_time = 100.0
        with patch('time.monotonic', return_value=initial_monotonic_time):
            mock_actuators.steer = 0.1
            safety_checker.validate_outputs(mock_actuators, mock_car_state, mock_radar_state)

        # Simulate second call immediately after, with a slightly different steering that SHOULD trigger a violation
        second_monotonic_time = initial_monotonic_time + 0.001 # 1 ms later
        mock_actuators.steer = 0.105 # Changed from 0.101 to ensure rate > 0.45
        with patch('time.monotonic', return_value=second_monotonic_time):
            report = safety_checker.validate_outputs(mock_actuators, mock_car_state, mock_radar_state)
            # steering_rate = abs(0.105 - 0.1) / max(0.001, 0.01) = 0.005 / 0.01 = 0.5
            # max_steering_rate = 0.45
            assert report['safe'] is False # Should now flag violation
            assert 'steering_rate_limit_exceeded' in report['violations']

    def test_accel_limit_exceeded(self, safety_checker, mock_actuators, mock_car_state, mock_radar_state):
        mock_actuators.accel = 4.0 # Exceeds default max_long_accel (3.0)
        report = safety_checker.validate_outputs(mock_actuators, mock_car_state, mock_radar_state)
        assert report['safe'] is False
        assert 'long_accel_limit_exceeded' in report['violations']
        assert report['recommended_action'] == 'decelerate'

    def test_lat_accel_limit_exceeded(self, safety_checker, mock_actuators, mock_car_state, mock_radar_state):
        mock_actuators.curvature = 0.05 # Example curvature
        mock_car_state.vEgo = 20.0 # Example speed, lat_accel = 0.05 * 20^2 = 20
        # Exceeds default max_lat_accel (2.5)
        report = safety_checker.validate_outputs(mock_actuators, mock_car_state, mock_radar_state)
        assert report['safe'] is False
        assert 'lat_accel_limit_exceeded' in report['violations']
        assert report['recommended_action'] == 'decelerate'

    def test_steering_angle_limit_exceeded(self, safety_checker, mock_actuators, mock_car_state, mock_radar_state):
        mock_actuators.steer = 2.0 # Exceeds max_steering_angle (0.95 * steerMax = 0.95 * 1.5 = 1.425)
        report = safety_checker.validate_outputs(mock_actuators, mock_car_state, mock_radar_state)
        assert report['safe'] is False
        assert 'steering_angle_limit_exceeded' in report['violations']
        assert report['recommended_action'] == 'disengage'

    def test_forward_collision_imminent(self, safety_checker, mock_actuators, mock_car_state, mock_radar_state):
        mock_radar_state.leadOne.status = True
        mock_radar_state.leadOne.dRel = 10.0 # meters
        mock_radar_state.leadOne.vRel = -5.0 # m/s, lead is slower by 5m/s
        mock_car_state.vEgo = 15.0 # m/s
        # ttc = 10.0 / (15.0 - (-5.0)) = 10.0 / 20.0 = 0.5 s
        # min_time_gap = 2.0. ttc < min_time_gap (0.5 < 2.0)
        report = safety_checker.validate_outputs(mock_actuators, mock_car_state, mock_radar_state)
        assert report['safe'] is False
        assert 'forward_collision_imminent' in report['violations']
        assert report['recommended_action'] == 'decelerate'

    def test_multiple_violations_accel_and_collision(self, safety_checker):
        mock_actuators = Mock()
        mock_actuators.accel = 4.0  # Exceeds max_long_accel (3.0)
        mock_actuators.steer = 0.0
        mock_actuators.curvature = 0.0

        mock_car_state = Mock()
        mock_car_state.vEgo = 15.0

        mock_radar_state = Mock()
        mock_radar_state.leadOne.status = True
        mock_radar_state.leadOne.dRel = 10.0
        mock_radar_state.leadOne.vRel = -5.0

        report = safety_checker.validate_outputs(mock_actuators, mock_car_state, mock_radar_state)

        assert report['safe'] is False
        assert 'long_accel_limit_exceeded' in report['violations']
        assert 'forward_collision_imminent' in report['violations']
        assert report['recommended_action'] == 'decelerate' # Deceleration is prioritized for these

    def test_multiple_violations_steering_rate_and_collision(self, safety_checker):
        mock_actuators = Mock()
        mock_actuators.accel = 0.0
        mock_actuators.steer = 0.1 # Will be changed to simulate rate
        mock_actuators.curvature = 0.0

        mock_car_state = Mock()
        mock_car_state.vEgo = 15.0

        mock_radar_state = Mock()
        mock_radar_state.leadOne.status = True
        mock_radar_state.leadOne.dRel = 10.0
        mock_radar_state.leadOne.vRel = -5.0

        initial_monotonic_time = 100.0
        with patch('time.monotonic', return_value=initial_monotonic_time):
            # First call to set initial steer and time for rate calculation
            safety_checker.validate_outputs(mock_actuators, mock_car_state, mock_radar_state)

        # Simulate rapid steering change AND forward collision
        second_monotonic_time = initial_monotonic_time + 0.01 # 0.01s time delta
        mock_actuators.steer = 0.5 # A large change to trigger steering rate violation
        with patch('time.monotonic', return_value=second_monotonic_time):
            report = safety_checker.validate_outputs(mock_actuators, mock_car_state, mock_radar_state)

            assert report['safe'] is False
            assert 'steering_rate_limit_exceeded' in report['violations']
            assert 'forward_collision_imminent' in report['violations']
            assert report['recommended_action'] == 'disengage' # Disengage is prioritized over decelerate

    def test_trigger_fail_safe_disengage(self, safety_checker, mock_car_state):
        safety_report = {
            'safe': False,
            'violations': ['steering_angle_limit_exceeded'],
            'recommended_action': 'disengage'
        }
        fail_safe_cmd = safety_checker.trigger_fail_safe(safety_report, mock_car_state)
        assert fail_safe_cmd['enabled'] is False
        assert fail_safe_cmd['acceleration'] == 0.0
        assert fail_safe_cmd['steering'] == 0.0

    def test_trigger_fail_safe_decelerate(self, safety_checker, mock_car_state):
        safety_report = {
            'safe': False,
            'violations': ['long_accel_limit_exceeded'],
            'recommended_action': 'decelerate'
        }
        mock_car_state.vEgo = 10.0 # m/s
        fail_safe_cmd = safety_checker.trigger_fail_safe(safety_report, mock_car_state)
        assert fail_safe_cmd['enabled'] is True
        assert fail_safe_cmd['acceleration'] == -3.0 # np.interp(10.0, [0.0, 10.0, 30.0], [-2.0, -3.0, -2.5])
        assert fail_safe_cmd['steering'] == 0.0

    def test_trigger_fail_safe_continue(self, safety_checker, mock_car_state):
        safety_report = {
            'safe': True,
            'violations': [],
            'recommended_action': 'continue'
        }
        fail_safe_cmd = safety_checker.trigger_fail_safe(safety_report, mock_car_state)
        assert fail_safe_cmd['enabled'] is True
        assert fail_safe_cmd['acceleration'] is None
        assert fail_safe_cmd['steering'] is None
