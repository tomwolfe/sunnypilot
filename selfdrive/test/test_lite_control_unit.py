import pytest
import numpy as np
from unittest.mock import Mock


from openpilot.selfdrive.controls.lib.lite_control import LightweightAdaptiveGainScheduler, LightweightComfortOptimizer

# Mock FirstOrderFilter for predictable behavior in tests
class MockFirstOrderFilter:
    def __init__(self, x0, k, dt):
        self.x = x0 # Initialize with x0
        self.k = k
        self.dt = dt

    def update(self, new_val):
        self.x = new_val # For testing, make it instant filter

class TestLightweightAdaptiveGainScheduler: # Removed @patch decorator here
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
        CP.lateralTuning.pid.kdV = [0.0, 0.1, 0.2] # Example for kdV
        CP.lateralTuning.pid.kdBP = [0.0, 10.0, 20.0]
        CP.lateralTuning.pid.kf = 0.1

        # Mock car fingerprint
        CP.carFingerprint = "TOYOTA_COROLLA" # Use a car with specific tuning

        return CP

    @pytest.fixture
    def scheduler(self, mock_car_params, mocker): # Add mocker fixture
        # Patch FirstOrderFilter before LightweightAdaptiveGainScheduler is instantiated
        mocker.patch('openpilot.selfdrive.controls.lib.lite_control.FirstOrderFilter', new=MockFirstOrderFilter)
        sched = LightweightAdaptiveGainScheduler(mock_car_params)
        return sched

    def test_init(self, scheduler, mock_car_params):
        # Filters are now MockFirstOrderFilter instances, but their update behavior is mocked.
        # Check initial x value as set by MockFirstOrderFilter.__init__
        assert isinstance(scheduler.long_speed_filter, MockFirstOrderFilter)
        assert scheduler.long_speed_filter.x == 0.1 # Based on FirstOrderFilter(0.1, ...) init
        assert isinstance(scheduler.lat_speed_filter, MockFirstOrderFilter)
        assert scheduler.lat_speed_filter.x == 0.1
        assert isinstance(scheduler.long_thermal_filter, MockFirstOrderFilter)
        assert scheduler.long_thermal_filter.x == 0.2
        assert isinstance(scheduler.lat_thermal_filter, MockFirstOrderFilter)
        assert scheduler.lat_thermal_filter.x == 0.2

        assert scheduler.CP == mock_car_params
        assert scheduler.vehicle_tuning['long_speed_gain_factor'] == 35.0 # From TOYOTA_COROLLA specific tuning

    def test_get_adaptive_gains_base_values(self, scheduler):
        v_ego = 5.0 # m/s
        thermal_state = 0.0 # No thermal reduction

        gains = scheduler.get_adaptive_gains(v_ego, thermal_state)

        # Expected base longitudinal gains (interpolated at v_ego=5.0)


        # Expected base lateral gains (interpolated at v_ego=5.0)


        # With mock filters that instantly update their .x to the input value (factor)
        # And since thermal_state=0.0 -> long_thermal_factor = 1.0, lat_thermal_factor = 1.0
        # And long_speed_factor and lat_speed_factor will be calculated.




        # Calculate expected precise values using exact calculations


        assert np.isclose(gains['longitudinal']['kp'], 0.34285714285714286, rtol=1e-6)
        assert np.isclose(gains['longitudinal']['ki'], 0.022857142857142856, rtol=1e-6)
        assert np.isclose(gains['longitudinal']['kf'], 0.05714285714285714, rtol=1e-6)
        assert np.isclose(gains['lateral']['kp'], 0.5454545454545454, rtol=1e-6)
        assert np.isclose(gains['lateral']['ki'], 0.032727272727272724, rtol=1e-6)
        assert np.isclose(gains['lateral']['kd'], 0.05454545454545454, rtol=1e-6)
        assert np.isclose(gains['lateral']['kf'], 0.10909090909090909, rtol=1e-6)

    def test_get_adaptive_gains_speed_scaling(self, scheduler):
        v_ego = 20.0 # High speed
        thermal_state = 0.0

        gains = scheduler.get_adaptive_gains(v_ego, thermal_state)

        long_speed_factor_expected = max(
            scheduler.vehicle_tuning['min_long_gain_multiplier'],
            min(
                scheduler.vehicle_tuning['max_long_gain_multiplier'],
                1.0 + (v_ego / scheduler.vehicle_tuning['long_speed_gain_factor'])
            )
        )
        lat_speed_factor_expected = max(
            scheduler.vehicle_tuning['min_lat_gain_multiplier'],
            min(
                scheduler.vehicle_tuning['max_lat_gain_multiplier'],
                1.0 + (v_ego / scheduler.vehicle_tuning['lat_speed_gain_factor'])
            )
        )

        # Since filters are mocked to instantly update their 'x' to the input
        expected_long_kp = np.interp(v_ego, scheduler.CP.longitudinalTuning.kpBP, scheduler.CP.longitudinalTuning.kpV)
        expected_long_ki = np.interp(v_ego, scheduler.CP.longitudinalTuning.kiBP, scheduler.CP.longitudinalTuning.kiV)
        expected_long_kf = scheduler.CP.longitudinalTuning.kf

        expected_lat_kp = np.interp(v_ego, scheduler.CP.lateralTuning.pid.kpBP, scheduler.CP.lateralTuning.pid.kpV)
        expected_lat_ki = np.interp(v_ego, scheduler.CP.lateralTuning.pid.kiBP, scheduler.CP.lateralTuning.pid.kiV)
        expected_lat_kd = np.interp(v_ego, scheduler.CP.lateralTuning.pid.kdBP, scheduler.CP.lateralTuning.pid.kdV)
        expected_lat_kf = scheduler.CP.lateralTuning.pid.kf

        # Calculate expected precise values using exact calculations


        assert np.isclose(gains['longitudinal']['kp'], 1.5714285714285714, rtol=1e-6)
        assert np.isclose(gains['longitudinal']['ki'], 0.07857142857142857, rtol=1e-6)
        assert np.isclose(gains['longitudinal']['kf'], 0.07857142857142857, rtol=1e-6)
        assert np.isclose(gains['lateral']['kp'], 2.0454545454545454, rtol=1e-6)
        assert np.isclose(gains['lateral']['ki'], 0.08181818181818182, rtol=1e-6)
        assert np.isclose(gains['lateral']['kd'], 0.2727272727272727, rtol=1e-6)
        assert np.isclose(gains['lateral']['kf'], 0.13636363636363635, rtol=1e-6)


    def test_get_adaptive_gains_thermal_scaling(self, scheduler):
        v_ego = 10.0
        thermal_state = 0.5 # 50% thermal state

        gains = scheduler.get_adaptive_gains(v_ego, thermal_state)

        long_thermal_factor_expected = max(0.85, 1.0 - thermal_state * 0.15)
        lat_thermal_factor_expected = max(0.9, 1.0 - thermal_state * 0.1)

        # Calculate speed factors as they also influence the final gain
        long_speed_factor_expected = max(
            scheduler.vehicle_tuning['min_long_gain_multiplier'],
            min(
                scheduler.vehicle_tuning['max_long_gain_multiplier'],
                1.0 + (v_ego / scheduler.vehicle_tuning['long_speed_gain_factor'])
            )
        )
        lat_speed_factor_expected = max(
            scheduler.vehicle_tuning['min_lat_gain_multiplier'],
            min(
                scheduler.vehicle_tuning['max_lat_gain_multiplier'],
                1.0 + (v_ego / scheduler.vehicle_tuning['lat_speed_gain_factor'])
            )
        )

        # Expected base longitudinal gains (interpolated at v_ego=10.0)
        expected_long_kp_base = np.interp(v_ego, scheduler.CP.longitudinalTuning.kpBP, scheduler.CP.longitudinalTuning.kpV)
        expected_long_ki_base = np.interp(v_ego, scheduler.CP.longitudinalTuning.kiBP, scheduler.CP.longitudinalTuning.kiV)
        expected_long_kf_base = scheduler.CP.longitudinalTuning.kf

        # Expected base lateral gains (interpolated at v_ego=10.0)
        expected_lat_kp_base = np.interp(v_ego, scheduler.CP.lateralTuning.pid.kpBP, scheduler.CP.lateralTuning.pid.kpV)
        expected_lat_ki_base = np.interp(v_ego, scheduler.CP.lateralTuning.pid.kiBP, scheduler.CP.lateralTuning.pid.kiV)
        expected_lat_kd_base = np.interp(v_ego, scheduler.CP.lateralTuning.pid.kdBP, scheduler.CP.lateralTuning.pid.kdV)
        expected_lat_kf_base = scheduler.CP.lateralTuning.pid.kf


        # Calculate expected precise values using exact calculations


        assert np.isclose(gains['longitudinal']['kp'], 0.5946428571428571, rtol=1e-6)
        assert np.isclose(gains['longitudinal']['ki'], 0.035678571428571426, rtol=1e-6)
        assert np.isclose(gains['longitudinal']['kf'], 0.05946428571428571, rtol=1e-6)
        assert np.isclose(gains['lateral']['kp'], 0.8981818181818182, rtol=1e-6)
        assert np.isclose(gains['lateral']['ki'], 0.04490909090909091, rtol=1e-6)
        assert np.isclose(gains['lateral']['kd'], 0.11227272727272727, rtol=1e-6)
        assert np.isclose(gains['lateral']['kf'], 0.11227272727272727, rtol=1e-6)




class TestLightweightComfortOptimizer:
    @pytest.fixture
    def optimizer(self):
        return LightweightComfortOptimizer()

    def test_optimize_for_comfort_no_jerk_limit(self, optimizer):
        desired_acceleration = 1.0
        v_ego = 10.0
        # First call to set initial prev_acceleration and prev_time
        optimizer.optimize_for_comfort(0.0, v_ego) # set prev_acceleration to 0.0

        # Simulate a small acceleration change that doesn't exceed jerk limit
        optimizer.prev_acceleration = 0.0
        optimizer.prev_time = 100.0 # Fixed time
        
        # Patch time.monotonic to control dt
        with mocker.patch('time.monotonic', return_value=100.1): # dt = 0.1
            optimized_accel = optimizer.optimize_for_comfort(desired_acceleration, v_ego)
            # adaptive_jerk_limit for v_ego=10.0: speed_factor = 1.0 - (10.0/30.0) = 2/3. adaptive_limit = 1.5 * (2/3) = 1.0
            # desired_jerk = (1.0 - 0.0) / 0.1 = 10.0
            # Since 10.0 > 1.0, jerk limit should apply.
            # max_delta_a = 1.0 * 0.1 = 0.1
            # new_acceleration = 0.0 + 0.1 * sign(10.0) = 0.1
            assert np.isclose(optimized_accel, 0.1, rtol=1e-6)

    def test_optimize_for_comfort_jerk_limit_active(self, optimizer):
        desired_acceleration = 2.0
        v_ego = 20.0
        optimizer.prev_acceleration = 0.0
        optimizer.prev_time = 100.0

        with mocker.patch('time.monotonic', return_value=100.1): # dt = 0.1
            optimized_accel = optimizer.optimize_for_comfort(desired_acceleration, v_ego)
            # adaptive_jerk_limit for v_ego=20.0: speed_factor = 1.0 - (20.0/30.0) = 1/3. adaptive_limit = 1.5 * (1/3) = 0.5
            # desired_jerk = (2.0 - 0.0) / 0.1 = 20.0
            # Since 20.0 > 0.5, jerk limit should apply.
            # max_delta_a = 0.5 * 0.1 = 0.05
            # new_acceleration = 0.0 + 0.05 * sign(20.0) = 0.05
            assert np.isclose(optimized_accel, 0.05, rtol=1e-6) # Corrected expected value

    def test_calculate_adaptive_jerk_limit(self, optimizer):
        # Test at low speed
        assert np.isclose(optimizer._calculate_adaptive_jerk_limit(v_ego=0.0), 1.5, rtol=1e-6) # max(0.5, 1.5 * (1.0 - 0/30)) = 1.5
        # Test at medium speed
        assert np.isclose(optimizer._calculate_adaptive_jerk_limit(v_ego=15.0), 0.75, rtol=1e-6) # max(0.5, 1.5 * (1.0 - 15/30)) = 0.75
        # Test at high speed (should cap at 0.5)
        assert np.isclose(optimizer._calculate_adaptive_jerk_limit(v_ego=40.0), 0.5, rtol=1e-6) \
          # Corrected expected value: max(0.5, 1.5 * (1.0 - 40/30)) = max(0.5, -0.5) = 0.5
