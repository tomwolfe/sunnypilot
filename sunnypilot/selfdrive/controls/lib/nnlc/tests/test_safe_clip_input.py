"""
Unit tests for the safe_clip_input functionality in NeuralNetworkLateralControl
These tests specifically address the critical review regarding testing complex dependencies
by focusing on the safe clipping functionality that was added
"""
import pytest
import numpy as np
from unittest.mock import Mock, patch

from sunnypilot.selfdrive.controls.lib.nnlc.nnlc import NeuralNetworkLateralControl


class TestSafeClipInput:
    """Test suite specifically for the safe_clip_input method"""

    def setup_method(self):
        """Setup test fixtures before each test method"""
        # Create mock CarParams
        self.mock_cp = Mock()
        self.mock_cp.steerRatio = 15.0
        self.mock_cp.steerActuatorDelay = 0.1
        self.mock_cp.lateralTuning.torque.kp = 1.0
        self.mock_cp.lateralTuning.torque.ki = 0.1
        self.mock_cp.lateralTuning.torque.kf = 0.01
        self.mock_cp.mass = 1500.0
        self.mock_cp.wheelbase = 2.7
        self.mock_cp.centerToFront = 1.5
        self.mock_cp.steerLimitTimer = 0.8
        self.mock_cp.steerMaxBP = [5.0, 20.0, 40.0]
        self.mock_cp.steerMaxV = [0.5, 0.3, 0.25]

        # Create mock VehicleModel
        self.mock_vm = Mock()

        # We need lac_torque, CP, CP_SP, CI for the actual constructor
        self.mock_lac_torque = Mock()
        self.mock_lac_torque.steer_max = 1.0
        self.mock_lac_torque.torque_params = Mock()
        self.mock_lac_torque.torque_params.STEER_MAX = 2.0
        self.mock_lac_torque.torque_params.STEER_DELTA_UP = 0.5
        self.mock_lac_torque.torque_params.STEER_DELTA_DOWN = 0.5
        self.mock_lac_torque.torque_params.STEER_ERROR_MAX = 0.5
        self.mock_lac_torque.torque_params.TORQUE_MEASURED_SCALAR = 1000.0
        self.mock_lac_torque.torque_params.TORQUE_OFFSET = 0.0
        self.mock_lac_torque.torque_params.TORQUE_POSITION_SCALAR = 1000.0

        # Create mock CP_SP (SunPyPilot CarParams)
        self.mock_cp_sp = Mock()
        self.mock_nnlc_sp = Mock()
        self.mock_model_path = Mock()
        self.mock_model_path.path = "test_path"  # This will be ignored due to mocking
        self.mock_nnlc_sp.model = self.mock_model_path
        self.mock_cp_sp.neuralNetworkLateralControl = self.mock_nnlc_sp

        # Create mock CI (Car Interface)
        self.mock_ci = Mock()

        # Mock the model and params at the right place
        with patch('sunnypilot.selfdrive.controls.lib.nnlc.nnlc.NNTorqueModelTinygrad') as MockModelClass, \
             patch('openpilot.common.params.Params') as MockParamsClass:

            # Mock the model instance that will be created
            mock_model_instance = Mock()
            mock_model_instance.friction_override = False
            MockModelClass.return_value = mock_model_instance

            # Mock the params instance that will be created
            mock_params_instance = Mock()
            mock_params_instance.get_bool.return_value = True
            MockParamsClass.return_value = mock_params_instance

            self.nnlc = NeuralNetworkLateralControl(self.mock_lac_torque, self.mock_cp, self.mock_cp_sp, self.mock_ci)

    def test_v_ego_boundary_clipping(self):
        """Test that vEgo is always clipped to [0.0, 40.0] range"""
        test_cases = [
            (-5.0, 0.0),      # Below minimum
            (0.0, 0.0),       # At minimum
            (20.0, 20.0),     # Within range
            (40.0, 40.0),     # At maximum
            (50.0, 40.0),     # Above maximum
        ]

        for input_val, expected in test_cases:
            input_list = [input_val, 1.0, 2.0, 3.0]
            result = self.nnlc.safe_clip_input(input_list, input_val, False)
            assert result[0] == expected, f"Input {input_val} should be clipped to {expected}"

    def test_normal_clipping_with_testing_disabled(self):
        """Test that all non-vEgo parameters are clipped to [-5.0, 5.0] when testing is disabled"""
        extreme_input = [15.0, 10.0, 20.0, -15.0, 30.0, -25.0]
        
        result = self.nnlc.safe_clip_input(extreme_input, 15.0, False)
        
        # First element (vEgo) should be unchanged if within bounds
        assert result[0] == 15.0
        
        # All other elements should be clipped to [-5.0, 5.0]
        for i in range(1, len(result)):
            assert -5.0 <= result[i] <= 5.0, f"Element {i} should be in range [-5.0, 5.0], got {result[i]}"

    def test_testing_mode_preserves_indices_1_2_3(self):
        """Test that testing mode preserves indices 1, 2, 3 while clipping others"""
        test_input = [20.0, 8.0, 6.0, 4.0, 10.0, -10.0, 15.0]
        
        result = self.nnlc.safe_clip_input(test_input, 20.0, True)
        
        # Index 0 (vEgo) preserved
        assert result[0] == 20.0
        
        # Indices 1, 2, 3 preserved when testing mode is enabled
        assert result[1] == 8.0  # setpoint
        assert result[2] == 6.0  # jerk
        assert result[3] == 4.0  # roll
        
        # Indices 4+ should be clipped to [-5.0, 5.0]
        assert result[4] == 5.0   # clipped from 10.0
        assert result[5] == -5.0  # clipped from -10.0
        assert result[6] == 5.0   # clipped from 15.0

    def test_normal_mode_clips_all_indices_1_plus(self):
        """Test that normal mode clips all indices 1+ regardless of position"""
        test_input = [20.0, 8.0, 6.0, 4.0, 10.0, -10.0, 15.0]
        
        result = self.nnlc.safe_clip_input(test_input, 20.0, False)
        
        # Index 0 (vEgo) preserved if within bounds
        assert result[0] == 20.0
        
        # All other indices should be clipped
        assert result[1] == 5.0   # clipped from 8.0
        assert result[2] == 5.0   # clipped from 6.0
        assert result[3] == 4.0   # stays at 4.0 as it's within [-5.0, 5.0]
        assert result[4] == 5.0   # clipped from 10.0
        assert result[5] == -5.0  # clipped from -10.0
        assert result[6] == 5.0   # clipped from 15.0

    def test_edge_case_boundary_values(self):
        """Test boundary conditions"""
        # Test values exactly at the boundary
        boundary_input = [40.0, 5.0, -5.0, 5.0, -5.0]
        
        result = self.nnlc.safe_clip_input(boundary_input, 40.0, False)
        
        # All values at boundary should remain unchanged if within range
        assert result[0] == 40.0  # vEgo at upper bound
        assert result[1] == 5.0   # at +5 bound
        assert result[2] == -5.0  # at -5 bound
        assert result[3] == 5.0   # at +5 bound
        assert result[4] == -5.0  # at -5 bound

    def test_empty_and_single_element_lists(self):
        """Test edge cases with minimal input lists"""
        # Single element (just vEgo)
        single_input = [50.0]
        result = self.nnlc.safe_clip_input(single_input, 50.0, False)
        assert result[0] == 40.0  # vEgo should be clipped
        
        # Two elements
        two_input = [25.0, 10.0]
        result = self.nnlc.safe_clip_input(two_input, 25.0, False)
        assert result[0] == 25.0  # vEgo within bounds
        assert result[1] == 5.0   # second element clipped

    def test_non_numeric_values(self):
        """Test that non-numeric values are handled gracefully"""
        # Mix of numeric and non-numeric values
        mixed_input = [20.0, "string", None, 8.0, 10.0]
        
        result = self.nnlc.safe_clip_input(mixed_input, 20.0, False)
        
        # vEgo (index 0) should be clipped
        assert result[0] == 20.0  # within bounds, so unchanged
        
        # Non-numeric values should remain unchanged
        assert result[1] == "string"
        assert result[2] is None
        
        # Numeric values should be clipped
        assert result[3] == 5.0   # 8.0 clipped to 5.0
        assert result[4] == 5.0   # 10.0 clipped to 5.0

    def test_saturation_behavior_preservation(self):
        """Test that saturation behavior is preserved with input clipping during testing"""
        # This test specifically addresses the review comment about preserving
        # saturation behavior during testing
        test_setpoint_input = [25.0, 10.0, 8.0, 6.0] + [1.0] * 10  # High setpoint/measurement values
        
        result = self.nnlc.safe_clip_input(test_setpoint_input, 25.0, allow_high_values_for_testing=True)
        
        # The first few values (vEgo, setpoint, jerk, roll) should maintain their relative relationships
        # for proper saturation behavior during testing
        assert result[0] == 25.0  # vEgo preserved
        assert result[1] == 10.0  # setpoint preserved during testing
        assert result[2] == 8.0   # jerk preserved during testing
        assert result[3] == 6.0   # roll preserved during testing
        
        # Other values should be clipped to [-5.0, 5.0]
        for i in range(4, len(result)):
            assert -5.0 <= result[i] <= 5.0

    def test_property_preservation_v_ego_bounds(self):
        """Property test: vEgo is always in [0.0, 40.0] range regardless of input"""
        test_values = [-100.0, -10.0, -1.0, 0.0, 10.0, 20.0, 40.0, 50.0, 100.0, 1000.0]
        
        for v_ego_input in test_values:
            input_list = [v_ego_input, 1.0, 2.0, 3.0]
            result = self.nnlc.safe_clip_input(input_list, v_ego_input, False)
            assert 0.0 <= result[0] <= 40.0, f"vEgo {v_ego_input} resulted in {result[0]}, which is outside [0.0, 40.0]"

    def test_property_preservation_other_bounds_when_testing_disabled(self):
        """Property test: when testing is disabled, all non-vEgo values are in [-5.0, 5.0]"""
        import random
        
        # Run multiple random test cases to ensure the property holds
        for _ in range(100):
            # Generate random input with extreme values
            test_input = [random.uniform(0, 50)]  # vEgo
            for _ in range(10):  # Other parameters
                test_input.append(random.uniform(-50, 50))
            
            result = self.nnlc.safe_clip_input(test_input, test_input[0], False)
            
            # Check that vEgo is in range
            assert 0.0 <= result[0] <= 40.0
            
            # Check that all other values are in [-5.0, 5.0]
            for val in result[1:]:
                if isinstance(val, (int, float)):
                    assert -5.0 <= val <= 5.0, f"Non-vEgo value {val} is outside [-5.0, 5.0] when testing disabled"

    def test_property_preservation_testing_mode_behavior(self):
        """Property test: when testing enabled, indices 1,2,3 preserved, others clipped"""
        import random
        
        for _ in range(50):
            # Generate random input with extreme values
            v_ego = random.uniform(0, 40)
            val1, val2, val3 = random.uniform(-20, 20), random.uniform(-20, 20), random.uniform(-20, 20)
            val4_plus = [random.uniform(-20, 20) for _ in range(8)]  # 8 additional values
            
            test_input = [v_ego, val1, val2, val3] + val4_plus
            result = self.nnlc.safe_clip_input(test_input, v_ego, True)
            
            # vEgo preserved (if within bounds)
            assert 0.0 <= result[0] <= 40.0
            
            # Indices 1, 2, 3 preserved
            assert result[1] == val1
            assert result[2] == val2
            assert result[3] == val3
            
            # Values from index 4 onwards should be clipped to [-5.0, 5.0]
            for i in range(4, len(result)):
                if isinstance(result[i], (int, float)):
                    assert -5.0 <= result[i] <= 5.0


def test_safe_clip_input_comprehensive():
    """Comprehensive test of safe_clip_input functionality"""
    # This is similar to the test in the review but with proper class usage
    from unittest.mock import Mock, patch

    # Create mock objects - we need lac_torque, CP, CP_SP, CI
    mock_lac_torque = Mock()
    mock_lac_torque.steer_max = 1.0
    mock_lac_torque.torque_params = Mock()
    mock_lac_torque.torque_params.STEER_MAX = 2.0
    mock_lac_torque.torque_params.STEER_DELTA_UP = 0.5
    mock_lac_torque.torque_params.STEER_DELTA_DOWN = 0.5
    mock_lac_torque.torque_params.STEER_ERROR_MAX = 0.5
    mock_lac_torque.torque_params.TORQUE_MEASURED_SCALAR = 1000.0
    mock_lac_torque.torque_params.TORQUE_OFFSET = 0.0
    mock_lac_torque.torque_params.TORQUE_POSITION_SCALAR = 1000.0

    mock_cp = Mock()
    mock_cp.steerRatio = 15.0
    mock_cp.steerActuatorDelay = 0.1
    mock_cp.lateralTuning.torque.kp = 1.0
    mock_cp.lateralTuning.torque.ki = 0.1
    mock_cp.lateralTuning.torque.kf = 0.01
    mock_cp.mass = 1500.0
    mock_cp.wheelbase = 2.7
    mock_cp.centerToFront = 1.5
    mock_cp.steerLimitTimer = 0.8
    mock_cp.steerMaxBP = [5.0, 20.0, 40.0]
    mock_cp.steerMaxV = [0.5, 0.3, 0.25]

    # Create mock CP_SP (SunPyPilot CarParams)
    # Need to mock the attribute path properly
    mock_cp_sp = Mock()
    mock_nnlc_sp = Mock()
    mock_model_path = Mock()
    mock_model_path.path = "test_path"  # This will be ignored due to mocking
    mock_nnlc_sp.model = mock_model_path
    mock_cp_sp.neuralNetworkLateralControl = mock_nnlc_sp

    # Create mock CI (Car Interface)
    mock_ci = Mock()

    # Mock the entire model before it gets instantiated
    # We need to patch where it's used, not where it's defined
    with patch('sunnypilot.selfdrive.controls.lib.nnlc.nnlc.NNTorqueModelTinygrad') as MockModelClass, \
         patch('openpilot.common.params.Params') as MockParamsClass:

        # Mock the model instance that will be created
        mock_model_instance = Mock()
        mock_model_instance.friction_override = False
        MockModelClass.return_value = mock_model_instance

        # Mock the params instance that will be created
        mock_params_instance = Mock()
        mock_params_instance.get_bool.return_value = True
        MockParamsClass.return_value = mock_params_instance

        nnlc = NeuralNetworkLateralControl(mock_lac_torque, mock_cp, mock_cp_sp, mock_ci)

        # Test normal operation
        normal_input = [20.0, 1.0, 0.5, 0.2, 2.0, -3.0]
        clipped_normal = nnlc.safe_clip_input(normal_input, 20.0, False)
        assert clipped_normal[0] == 20.0  # vEgo preserved
        for val in clipped_normal[1:]:
            assert -5.0 <= val <= 5.0  # Others clipped

        # Test with testing mode enabled
        test_input = [20.0, 8.0, 6.0, 4.0, 10.0, -10.0]
        clipped_test = nnlc.safe_clip_input(test_input, 20.0, True)
        assert clipped_test[0] == 20.0  # vEgo preserved
        assert clipped_test[1] == 8.0   # setpoint preserved during testing
        assert clipped_test[2] == 6.0   # jerk preserved during testing
        assert clipped_test[3] == 4.0   # roll preserved during testing
        assert clipped_test[4] == 5.0   # others clipped
        assert clipped_test[5] == -5.0  # others clipped

        # Test with testing disabled (all clipped except vEgo)
        clipped_normal = nnlc.safe_clip_input(normal_input, 20.0, False)
        assert clipped_normal[0] == 20.0  # vEgo preserved
        for val in clipped_normal[1:]:
            assert -5.0 <= val <= 5.0  # All others clipped

    print("All safe_clip_input tests passed!")


if __name__ == "__main__":
    test_safe_clip_input_comprehensive()
    pytest.main([__file__])