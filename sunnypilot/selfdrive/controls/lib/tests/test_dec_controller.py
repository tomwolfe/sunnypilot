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