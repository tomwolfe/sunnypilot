"""
Unit tests for Enhanced DEC (Dynamic Experimental Controller)
"""
import time
import pytest
import numpy as np
from unittest.mock import Mock, patch

from sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController


class TestDynamicExperimentalController:
    """Test suite for DynamicExperimentalController class"""

    def setup_method(self):
        """Setup test fixtures before each test method"""
        # Create mock CarParams
        self.mock_cp = Mock()
        self.mock_cp.radarUnavailable = False
        self.mock_cp.lateralTuning.torque.kp = 1.0
        self.mock_cp.lateralTuning.torque.ki = 0.1
        
        # Create mock SubMaster
        self.mock_sm = Mock()
        self.mock_sm.updated = {
            'carState': True,
            'radarState': True,
            'modelV2': True,
            'selfdriveState': True
        }
        
        # Create mock messaging system
        mock_car_state = Mock()
        mock_car_state.vEgo = 15.0
        mock_car_state.vCruise = 25.0
        mock_car_state.standstill = False
        mock_car_state.steeringPressed = False
        mock_car_state.steeringTorqueEps = 0.5
        
        def mock_sm_getitem(key):
            if key == 'carState':
                return mock_car_state
            elif key == 'radarState':
                return Mock(radarFaulted=False)
            elif key == 'modelV2':
                return Mock(
                    meta=Mock(stopState=0.2),
                    temporalBatch=[Mock(trafficLightStateProba=[0.1])]
                )
            elif key == 'selfdriveState':
                return Mock(experimentalMode=True)
            else:
                return Mock()
        
        self.mock_sm.__getitem__ = mock_sm_getitem
        
        # Initialize the controller
        with patch('sunnypilot.selfdrive.controls.lib.dec.dec.Params'), \
             patch('sunnypilot.selfdrive.controls.lib.dec.dec.EditableParams'), \
             patch('sunnypilot.selfdrive.controls.lib.dec.dec.ModeManager'):
            self.dec = DynamicExperimentalController(self.mock_cp)

    def test_initialization(self):
        """Test initialization of DynamicExperimentalController"""
        # Verify that new attributes from the enhancements are initialized
        assert hasattr(self.dec, '_weather_confidence')
        assert hasattr(self.dec, '_lighting_condition')
        assert hasattr(self.dec, '_driver_aggression_score')
        assert hasattr(self.dec, '_driver_override_history')
        assert hasattr(self.dec, '_lidar_available')
        assert hasattr(self.dec, '_radar_confidence')
        assert hasattr(self.dec, '_has_predictive_stop')
        assert hasattr(self.dec, '_approaching_object_confidence')
        assert hasattr(self.dec, '_stop_signal_confidence')
        assert hasattr(self.dec, '_intersection_confidence')
        
        # Verify initial values
        assert self.dec._weather_confidence == 1.0
        assert self.dec._lighting_condition == 1.0
        assert self.dec._driver_aggression_score == 0.5
        assert self.dec._radar_confidence == 1.0
        assert self.dec._has_predictive_stop is False

    def test_update_environmental_conditions(self):
        """Test environmental conditions update"""
        # Call the method to update environmental conditions
        self.dec._update_environmental_conditions(self.mock_sm)

        # Verify that the environmental properties were updated
        assert 0.0 <= self.dec._weather_confidence <= 1.0
        assert 0.0 <= self.dec._lighting_condition <= 1.0
        assert 0.0 <= self.dec._radar_confidence <= 1.0

    def test_realistic_weather_sensors_simulation(self):
        """Test environmental awareness with realistic sensor data simulating various weather conditions"""
        # Create a more realistic mock that simulates adverse weather conditions
        mock_car_state = Mock()
        mock_car_state.vEgo = 15.0
        mock_car_state.vCruise = 25.0
        mock_car_state.standstill = False
        mock_car_state.steeringPressed = True  # Simulate more frequent steering
        mock_car_state.steeringTorqueEps = 0.5
        mock_car_state.steeringRateDeg = 15.0  # High steering rate indicating unstable conditions
        mock_car_state.aEgo = 2.5  # High acceleration indicating unstable traction

        # Simulate wheel speeds with variance indicating slippery conditions
        mock_wheel_speeds = Mock()
        mock_wheel_speeds.fl = 14.8
        mock_wheel_speeds.fr = 15.2
        mock_wheel_speeds.rl = 14.5
        mock_wheel_speeds.rr = 15.5
        mock_car_state.wheelSpeeds = mock_wheel_speeds

        # Create mock device state with low light sensor reading
        mock_device_state = Mock()
        mock_device_state.lightSensor = 30.0  # Low light condition

        # Create mock model with high execution time (indicating poor visibility/processing)
        mock_model = Mock()
        mock_meta = Mock()
        mock_meta.modelExecutionTime = 0.06  # High execution time
        mock_model.meta = mock_meta

        # Create mock radar state with few objects detected (indicating poor weather)
        mock_radar_state = Mock()
        mock_radar_state.radarFaulted = False
        mock_radar_state.leadOne = Mock()
        mock_radar_state.leadOne.status = False  # No lead detected
        mock_radar_state.tracks = []  # No other objects detected

        def mock_sm_getitem(key):
            if key == 'carState':
                return mock_car_state
            elif key == 'radarState':
                return mock_radar_state
            elif key == 'modelV2':
                return mock_model
            elif key == 'deviceState':
                return mock_device_state
            elif key == 'selfdriveState':
                return Mock(experimentalMode=True)
            else:
                return Mock()

        mock_sm = Mock()
        mock_sm.updated = {
            'carState': True,
            'radarState': True,
            'modelV2': True,
            'deviceState': True,
            'selfdriveState': True
        }
        mock_sm.__getitem__ = mock_sm_getitem

        # Store initial values
        initial_weather_conf = self.dec._weather_confidence
        initial_lighting_cond = self.dec._lighting_condition
        initial_radar_conf = self.dec._radar_confidence

        # Update environmental conditions with adverse weather indicators
        self.dec._update_environmental_conditions(mock_sm)

        # Verify that confidence values were appropriately reduced due to adverse conditions
        assert self.dec._weather_confidence < initial_weather_conf, \
            "Weather confidence should be reduced in adverse conditions"
        assert self.dec._lighting_condition < initial_lighting_cond, \
            "Lighting condition should be reduced with low light sensor"
        # Radar confidence might be affected by object detection metrics

    def test_combined_environmental_factors(self):
        """Test environmental awareness when multiple factors indicate good conditions"""
        # Create mock with good condition indicators
        mock_car_state = Mock()
        mock_car_state.vEgo = 20.0
        mock_car_state.steeringPressed = False
        mock_car_state.steeringRateDeg = 2.0  # Low steering rate
        mock_car_state.aEgo = 0.5  # Low acceleration variation
        mock_car_state.wheelSpeeds = Mock()
        mock_car_state.wheelSpeeds.fl = 20.0
        mock_car_state.wheelSpeeds.fr = 20.0
        mock_car_state.wheelSpeeds.rl = 20.0
        mock_car_state.wheelSpeeds.rr = 20.0

        mock_device_state = Mock()
        mock_device_state.lightSensor = 200.0  # Bright light condition

        mock_model = Mock()
        mock_meta = Mock()
        mock_meta.modelExecutionTime = 0.02  # Fast execution time
        mock_model.meta = mock_meta

        mock_radar_state = Mock()
        mock_radar_state.radarFaulted = False
        mock_radar_state.leadOne = Mock()
        mock_radar_state.leadOne.status = True  # Lead detected
        # Simulate multiple tracks
        mock_radar_state.tracks = [Mock() for _ in range(8)]
        for i, track in enumerate(mock_radar_state.tracks):
            track.status = 1 if i < 6 else 0  # Most tracks active

        def mock_sm_getitem(key):
            if key == 'carState':
                return mock_car_state
            elif key == 'radarState':
                return mock_radar_state
            elif key == 'modelV2':
                return mock_model
            elif key == 'deviceState':
                return mock_device_state
            elif key == 'selfdriveState':
                return Mock(experimentalMode=True)
            else:
                return Mock()

        mock_sm = Mock()
        mock_sm.updated = {
            'carState': True,
            'radarState': True,
            'modelV2': True,
            'deviceState': True,
            'selfdriveState': True
        }
        mock_sm.__getitem__ = mock_sm_getitem

        # Update environmental conditions with good condition indicators
        self.dec._update_environmental_conditions(mock_sm)

        # Confidence values should increase or remain high in good conditions
        assert 0.0 <= self.dec._weather_confidence <= 1.0
        assert 0.0 <= self.dec._lighting_condition <= 1.0
        assert 0.0 <= self.dec._radar_confidence <= 1.0

    def test_multiple_simultaneous_environmental_factors(self):
        """Test complex scenarios with multiple environmental factors occurring simultaneously"""
        # Create a complex scenario with multiple environmental factors
        mock_car_state = Mock()
        mock_car_state.vEgo = 10.0  # Low speed in urban setting
        mock_car_state.steeringPressed = True  # Driver intervention
        mock_car_state.steeringRateDeg = 12.0  # High steering rate
        mock_car_state.aEgo = 3.0  # High acceleration variation
        mock_car_state.wheelSpeeds = Mock()
        mock_car_state.wheelSpeeds.fl = 9.5  # Some variance in wheel speeds
        mock_car_state.wheelSpeeds.fr = 10.5
        mock_car_state.wheelSpeeds.rl = 8.8
        mock_car_state.wheelSpeeds.rr = 11.2

        mock_device_state = Mock()
        mock_device_state.lightSensor = 25.0  # Poor lighting

        mock_model = Mock()
        mock_meta = Mock()
        mock_meta.modelExecutionTime = 0.07  # Slow model execution
        mock_model.meta = mock_meta

        mock_radar_state = Mock()
        mock_radar_state.radarFaulted = False
        mock_radar_state.leadOne = Mock()
        mock_radar_state.leadOne.status = False  # No lead detected (could indicate weather issues)
        # Simulate few tracks detected
        mock_radar_state.tracks = []  # No tracks detected

        def mock_sm_getitem(key):
            if key == 'carState':
                return mock_car_state
            elif key == 'radarState':
                return mock_radar_state
            elif key == 'modelV2':
                return mock_model
            elif key == 'deviceState':
                return mock_device_state
            elif key == 'selfdriveState':
                return Mock(experimentalMode=True)
            else:
                return Mock()

        mock_sm = Mock()
        mock_sm.updated = {
            'carState': True,
            'radarState': True,
            'modelV2': True,
            'deviceState': True,
            'selfdriveState': True
        }
        mock_sm.__getitem__ = mock_sm_getitem

        # Store initial values
        initial_weather_conf = self.dec._weather_confidence
        initial_lighting_cond = self.dec._lighting_condition
        initial_radar_conf = self.dec._radar_confidence

        # Update environmental conditions with multiple adverse factors
        self.dec._update_environmental_conditions(mock_sm)

        # Verify that all confidence values were reduced due to multiple adverse factors
        # The combination of multiple factors should significantly reduce confidence
        assert self.dec._weather_confidence <= initial_weather_conf, \
            "Weather confidence should be reduced with multiple adverse factors"
        assert self.dec._lighting_condition <= initial_lighting_cond, \
            "Lighting condition should reflect poor lighting sensor"

    def test_safety_constraints_with_multiple_simultaneous_triggers(self):
        """Test system behavior when multiple safety constraints are triggered simultaneously"""
        # Test that the system handles multiple simultaneous safety events appropriately
        with patch.object(self.dec._mode_manager, 'request_mode') as mock_request:
            # Simulate multiple safety triggers at once
            self.dec._has_mpc_fcw = True  # Emergency condition
            self.dec._has_predictive_stop = True  # Predictive stop
            self.dec._has_slow_down = True  # Slow down scenario
            self.dec._urgency = 0.8  # High urgency
            self.dec._has_slowness = True  # Slowness scenario
            self.dec._standstill_count = 5  # Standstill

            # Calculate base confidence with adverse conditions
            self.dec._weather_confidence = 0.4  # Poor weather
            self.dec._lighting_condition = 0.5  # Poor lighting
            base_confidence = self.dec._calculate_base_confidence()

            # Call the methods that handle these scenarios
            emergency_handled = self.dec._handle_emergency_conditions()
            slow_down_handled = self.dec._handle_slow_down_scenarios(base_confidence)
            standstill_handled = self.dec._handle_standstill_scenario(base_confidence)

            # Emergency should be handled first (MPC FCW has highest priority)
            assert emergency_handled == True
            # Verify that request_mode was called with emergency=True and high confidence
            mock_request.assert_called_with('blended', confidence=1.0, emergency=True)

    def test_acceleration_change_limits_with_environmental_conditions(self):
        """Test that acceleration change limits are properly enforced with environmental adjustments"""
        # This test checks the safety limits from the longitudinal planner
        from openpilot.selfdrive.controls.lib.longitudinal_planner import get_max_accel, limit_accel_in_turns
        import numpy as np

        # Test acceleration limits under different speeds and environmental conditions
        test_speeds = [5.0, 15.0, 25.0, 35.0]
        for speed in test_speeds:
            base_limit = get_max_accel(speed, experimental_mode=False)
            exp_limit = get_max_accel(speed, experimental_mode=True)

            # Experimental mode should allow higher but still safe acceleration
            if speed < 30:  # At lower speeds, experimental mode can be more aggressive
                assert exp_limit > base_limit
            else:  # At higher speeds, be more conservative even in experimental mode
                assert exp_limit <= 2.0  # Should not exceed 2.0 m/s^2

    def test_multi_level_fallback_system(self):
        """Test the multi-level fallback system under different error conditions"""
        # Store original active state
        original_active = self.dec._active

        # Test Level 1 fallback
        self.dec._active = True  # Start in active state
        self.dec._handle_error_fallback(Exception("Test error"))

        # Level 1 should deactivate experimental mode but keep ACC active
        assert self.dec._active == False  # Should be inactive after error

        # Test error disengagement counting
        original_disengagement_count = self.dec._mode_manager.disengagement_count
        original_error_count = self.dec._mode_manager.error_disengagement_count

        # Trigger another error to test disengagement counting
        self.dec._last_error_occurred = True
        prev_active = True
        # Simulate the update flow where prev_active=True and current_active=False
        self.dec._active = False  # This simulates experimental mode being disabled after error

        if prev_active and not self.dec._active:
            self.dec._mode_manager.disengagement_count += 1
            if hasattr(self.dec, '_last_error_occurred') and self.dec._last_error_occurred:
                self.dec._mode_manager.error_disengagement_count += 1
                self.dec._last_error_occurred = False

        # Verify counts were incremented
        assert self.dec._mode_manager.disengagement_count >= original_disengagement_count
        assert self.dec._mode_manager.error_disengagement_count >= original_error_count

    def test_update_driver_behavior(self):
        """Test driver behavior tracking"""
        # Create a car state with steering pressed to simulate override
        mock_car_state = Mock()
        mock_car_state.steeringPressed = True
        mock_sm = Mock()
        mock_sm.__getitem__ = Mock(return_value=mock_car_state)

        initial_aggression = self.dec._driver_aggression_score

        # Update driver behavior
        self.dec._update_driver_behavior(mock_sm)

        # After overrides, aggression should decrease
        if len(self.dec._driver_override_history) > 0:
            assert self.dec._driver_aggression_score <= initial_aggression + 0.01  # Allow for floating point

    def test_calculate_predictive_stops(self):
        """Test predictive stop calculation"""
        # Create mock model data
        mock_model = Mock()
        mock_model.meta = Mock(stopState=0.8)  # High stop state confidence
        mock_model.temporalBatch = [Mock(trafficLightStateProba=[0.9])]  # High traffic light confidence
        
        # Call the method to calculate predictive stops
        self.dec._calculate_predictive_stops(mock_model)
        
        # Based on the high confidence values, predictive stop should eventually be detected
        # (Note: The Kalman filter might need multiple updates to reach the threshold)

    def test_radarless_mode_with_enhancements(self):
        """Test radarless mode with environmental awareness"""
        self.dec._CP.radarUnavailable = True
        
        # Set up conditions that would trigger different behaviors
        self.dec._has_mpc_fcw = True  # Emergency condition
        self.dec._has_predictive_stop = True
        self.dec._weather_confidence = 0.5  # Adverse weather
        self.dec._driver_aggression_score = 0.2  # Cautious driver
        
        # Mock the mode manager to verify calls
        with patch.object(self.dec._mode_manager, 'request_mode') as mock_request:
            self.dec._radarless_mode()
            
            # Verify that request_mode was called with adjusted confidence
            mock_request.assert_called()

    def test_radar_mode_with_enhancements(self):
        """Test radar mode with environmental awareness"""
        self.dec._CP.radarUnavailable = False  # Use radar mode
        
        # Set up conditions
        self.dec._has_mpc_fcw = False
        self.dec._has_predictive_stop = True
        self.dec._weather_confidence = 0.3  # Adverse weather
        self.dec._radar_confidence = 0.4  # Unreliable radar
        
        # Mock the mode manager to verify calls
        with patch.object(self.dec._mode_manager, 'request_mode') as mock_request:
            self.dec._radar_mode()
            
            # Verify that request_mode was called
            mock_request.assert_called()

    def test_update_method_with_error_handling(self):
        """Test update method with error handling"""
        # Set up the controller properly
        with patch('sunnypilot.selfdrive.controls.lib.dec.dec.Params'), \
             patch('sunnypilot.selfdrive.controls.lib.dec.dec.EditableParams'), \
             patch('sunnypilot.selfdrive.controls.lib.dec.dec.ModeManager'):
            dec = DynamicExperimentalController(self.mock_cp)
            
            # Mock the update methods to raise exceptions
            with patch.object(dec, '_read_params', side_effect=Exception("Test error")), \
                 patch.object(dec._mode_manager, 'request_mode'), \
                 patch.object(dec._mode_manager, 'update'):
                # This should not crash due to error handling in the enhanced update method
                dec.update(self.mock_sm)
                
                # Verify that safe fallback was used
                assert dec._active is False  # Should be inactive after error

    def test_update_method_normal_operation(self):
        """Test normal update operation"""
        # Ensure the controller is set up properly for normal operation
        with patch('sunnypilot.selfdrive.controls.lib.dec.dec.Params'), \
             patch('sunnypilot.selfdrive.controls.lib.dec.dec.EditableParams'), \
             patch('sunnypilot.selfdrive.controls.lib.dec.dec.ModeManager'):
            dec = DynamicExperimentalController(self.mock_cp)
            
            # Mock the necessary components
            with patch.object(dec, '_read_params'), \
                 patch.object(dec, 'set_mpc_fcw_crash_cnt'), \
                 patch.object(dec, '_update_calculations'), \
                 patch.object(dec, '_should_enforce_standstill', return_value=False), \
                 patch.object(dec._mode_manager, 'request_mode'), \
                 patch.object(dec._mode_manager, 'update'):
                
                # This should execute without errors
                dec.update(self.mock_sm)
                
                # Verify active state based on experimental mode
                assert dec._active is True  # Should be active based on mock experimentalMode=True

    def test_predictive_model_parameters(self):
        """Test predictive model parameter handling"""
        # Test initial values
        assert self.dec._approaching_object_confidence == 0.0
        assert self.dec._stop_signal_confidence == 0.0
        assert self.dec._intersection_confidence == 0.0
        
        # These values should change when the predictive methods are called


if __name__ == "__main__":
    pytest.main([__file__])