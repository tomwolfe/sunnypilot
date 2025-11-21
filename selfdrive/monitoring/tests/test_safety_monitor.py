import pytest
import numpy as np
from unittest.mock import Mock, patch

from cereal import log
from openpilot.common.params import Params, UnknownKeyName
from openpilot.sunnypilot.selfdrive.monitoring.safety_monitor import SafetyMonitor

# Mock the Params class to control parameter values in tests
class MockParams(Params):
    def __init__(self, params_dict=None):
        self._params = params_dict if params_dict is not None else {}

    def get(self, key, encoding='utf8', block=False):
        if key in self._params:
            return str(self._params[key]).encode(encoding)
        # Simulate UnknownKeyName if parameter is not found
        raise UnknownKeyName(f"Parameter {key} not found")

    def put(self, key, value, encoding='utf8'):
        self._params[key] = value

# Fixture to provide a SafetyMonitor instance with mocked Params
@pytest.fixture
def safety_monitor_instance(mock_params_data=None):
    # This fixture needs to be adapted to actually use the mock_params_data
    # For now, it defaults to no params_data, which means SafetyMonitor will use default values
    with patch('openpilot.common.params.Params', MockParams):
        monitor = SafetyMonitor()
        monitor.params = MockParams(mock_params_data) # Ensure the instance also has the mock
        yield monitor

# Fixture for a SafetyMonitor instance initialized with default parameters
@pytest.fixture
def default_safety_monitor_instance():
    # Use a specific params_dict for default initialization, where all keys exist but are default
    default_params = {
        "ModelConfidenceThreshold": 0.7,
        "RadarConfidenceThreshold": 0.6,
        "LaneDeviationThreshold": 0.8,
        "LowSpeedSafetyThreshold": 0.4,
        "ModelConfidenceThresholdMultiplier": 0.8,
        "MaxLateralAcceleration": 2.0,
        "CurveDetectionThreshold": 0.5,
        "MaxRadarDistanceForConfidence": 150.0,
        "VelocityConfidenceScale": 30.0,  # Changed from 100.0 to 30.0
        "SafetyCriticalThreshold": 0.3,   # Added new parameter
    }
    with patch('openpilot.common.params.Params', MockParams):
        # Override the Params.get method for precise control over return values
        with patch.object(MockParams, 'get', side_effect=lambda key, **kwargs: (
            str(default_params[key]).encode('utf8') if key in default_params else
            UnknownKeyName(f"Parameter {key} not found")
        )):
            monitor = SafetyMonitor()
            monitor.params = MockParams(default_params) # Ensure the instance also has the mock
            yield monitor

# Helper function to create mock messages
def create_mock_model_v2_msg(meta_confidence=1.0, path_x=None, path_y=None, lane_width=3.5, d_path=None, dangerous_exec_time=False):
    msg = Mock()
    msg.meta.confidence = meta_confidence
    msg.path.x = path_x if path_x is not None else [float(i) for i in range(100)]
    msg.path.y = path_y if path_y is not None else [0.0] * 100
    msg.lateralPlan.laneWidth = lane_width
    msg.lateralPlan.dPath = d_path if d_path is not None else [0.0] # Default to no deviation
    msg.meta.dangerousModelExecutionTime = dangerous_exec_time
    return msg

def create_mock_radar_state_msg(lead_status=True, d_rel=50.0, v_rel=0.0, a_rel=0.0, lead_one_has_a_rel=True):
    msg = Mock()
    msg.leadOne.status = lead_status
    msg.leadOne.dRel = d_rel
    msg.leadOne.vRel = v_rel
    if lead_one_has_a_rel:
        msg.leadOne.aRel = a_rel
    else:
        # Simulate aRel not existing
        if hasattr(msg.leadOne, 'aRel'):
            del msg.leadOne.aRel
    return msg

def create_mock_car_state_msg(v_ego=10.0, steering_angle_deg=0.0):
    msg = Mock()
    msg.vEgo = v_ego
    msg.steeringAngleDeg = steering_angle_deg
    return msg

def create_mock_car_control_msg():
    msg = Mock()
    return msg

def create_mock_live_pose_msg(orientation_ned_available=True, orientation_ned_vals=[0.0, 0.0, 0.0]):
    msg = Mock()
    if orientation_ned_available:
        msg.orientationNED = orientation_ned_vals # roll, pitch, yaw
    else:
        msg.orientationNED = None # Simulate not available
    return msg

# --- Test Cases ---

def test_safety_monitor_initialization(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    assert monitor.model_confidence_threshold == 0.7
    assert monitor.radar_confidence_threshold == 0.6
    assert monitor.lane_deviation_threshold == 0.8
    assert monitor.low_speed_safety_threshold == 0.4
    assert monitor.model_confidence_threshold_multiplier == 0.8
    assert monitor.max_lat_accel == 2.0
    assert monitor.curve_detection_threshold == 0.5
    assert monitor.max_radar_distance_for_confidence == 150.0
    assert monitor.velocity_confidence_scale == 30.0
    assert monitor.safety_critical_threshold == 0.3
    assert monitor.lighting_condition == "normal"
    assert monitor.weather_condition == "clear"
    assert monitor.road_condition == "dry"
    assert monitor.in_tunnel == False
    assert monitor.overall_safety_score == 1.0
    assert not monitor.requires_intervention
    assert not monitor.confidence_degraded

def test_model_confidence_threshold_multiplier_validation():
    # Test with valid multiplier
    params_data = {"ModelConfidenceThresholdMultiplier": 0.7}
    monitor = SafetyMonitor()
    monitor.params = MockParams(params_data)
    # Re-initialize to apply new params and run validation
    monitor.__init__()
    assert monitor.model_confidence_threshold_multiplier == 0.7

    # Test with invalid multiplier (low)
    params_data = {"ModelConfidenceThresholdMultiplier": 0.3}
    with patch('logging.warning') as mock_warning:
        monitor = SafetyMonitor()
        monitor.params = MockParams(params_data)
        monitor.__init__()
        mock_warning.assert_called_with("ModelConfidenceThresholdMultiplier 0.3 out of valid range [0.5, 1.0]. Using default 0.8")
        assert monitor.model_confidence_threshold_multiplier == 0.8

    # Test with invalid multiplier (high)
    params_data = {"ModelConfidenceThresholdMultiplier": 1.2}
    with patch('logging.warning') as mock_warning:
        monitor = SafetyMonitor()
        monitor.params = MockParams(params_data)
        monitor.__init__()
        mock_warning.assert_called_with("ModelConfidenceThresholdMultiplier 1.2 out of valid range [0.5, 1.0]. Using default 0.8")
        assert monitor.model_confidence_threshold_multiplier == 0.8

# Test for environmental conditions detection (placeholder for now)
def test_environmental_detection_defaults(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    model_v2_msg = create_mock_model_v2_msg()
    car_state_msg = create_mock_car_state_msg()
    car_control_msg = create_mock_car_control_msg()
    live_pose_msg = create_mock_live_pose_msg()

    with patch('logging.warning') as mock_warning:
        monitor.detect_environmental_conditions(model_v2_msg, car_state_msg, car_control_msg, live_pose_msg)
        mock_warning.assert_any_call("Luminance data for lighting condition detection is not available. Defaulting to 'normal' lighting condition.")
        assert monitor.lighting_condition == "normal"
        assert monitor.weather_condition == "clear"
        assert monitor.road_condition == "dry"
        assert not monitor.in_tunnel

def test_environmental_detection_no_orientation(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    model_v2_msg = create_mock_model_v2_msg()
    car_state_msg = create_mock_car_state_msg()
    car_control_msg = create_mock_car_control_msg()
    live_pose_msg = create_mock_live_pose_msg(orientation_ned_available=False)

    with patch('logging.warning') as mock_warning:
        monitor.detect_environmental_conditions(model_v2_msg, car_state_msg, car_control_msg, live_pose_msg)
        mock_warning.assert_any_call("Luminance data for lighting condition detection is not available. Defaulting to 'normal' lighting condition.")
        mock_warning.assert_any_call("IMU data for weather/road condition detection is not available. Maintaining previous environmental conditions.")
        assert monitor.lighting_condition == "normal"
        assert monitor.weather_condition == "clear"
        assert monitor.road_condition == "dry"
        assert not monitor.in_tunnel

# Test radar confidence with and without aRel
def test_radar_confidence_with_a_rel(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    # With aRel
    radar_state_msg = create_mock_radar_state_msg(lead_status=True, d_rel=25.0, v_rel=-5.0, a_rel=-1.0)
    monitor.update_radar_confidence(radar_state_msg)
    # Expected: distance_confidence = 1 - (25/150) = 0.8333...
    #           velocity_confidence = (30.0 - abs(-5.0)) / 30.0 = 0.8333... (since velocity_confidence_scale is now 30.0)
    #           acceleration_confidence = (5.0 - |-1.0|) / 5.0 = 0.8
    #           radar_confidence = (0.8333 * 0.5 + 0.8333 * 0.3 + 0.8 * 0.2) = 0.41665 + 0.25 + 0.16 = 0.82665
    assert monitor.radar_confidence == pytest.approx(0.82665, abs=1e-5)

def test_radar_confidence_without_a_rel(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    # Without aRel (should default to 0.5 for acceleration_confidence)
    radar_state_msg = create_mock_radar_state_msg(lead_status=True, d_rel=25.0, v_rel=-5.0, lead_one_has_a_rel=False)
    with patch('logging.warning') as mock_warning:
        monitor.update_radar_confidence(radar_state_msg)
        mock_warning.assert_any_call("aRel not available for radar acceleration confidence. Defaulting to a conservative 0.5.")
        # Expected: distance_confidence = 1 - (25/150) = 0.8333...
        #           velocity_confidence = (30.0 - abs(-5.0)) / 30.0 = 0.8333...
        #           acceleration_confidence = 0.5 (fallback)
        #           radar_confidence = (0.8333 * 0.5 + 0.8333 * 0.3 + 0.5 * 0.2) = 0.41665 + 0.25 + 0.1 = 0.76665
        assert monitor.radar_confidence == pytest.approx(0.76665, abs=1e-5)

def test_radar_confidence_no_lead(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    radar_state_msg = create_mock_radar_state_msg(lead_status=False)
    monitor.update_radar_confidence(radar_state_msg)
    assert monitor.radar_confidence == pytest.approx(0.3)

# Test curvature calculation edge cases
def test_curve_anticipation_straight_path(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    model_v2_msg = create_mock_model_v2_msg(path_y=[0.0] * 100) # Straight path
    car_state_msg = create_mock_car_state_msg(v_ego=20.0)
    monitor.detect_curve_anticipation(model_v2_msg, car_state_msg)
    assert monitor.curve_anticipation_active == False
    assert monitor.curve_anticipation_score == pytest.approx(0.0)

def test_curve_anticipation_sharp_turn(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    # Simulate a sharp turn: y-coords changing rapidly
    path_x_sharp = [float(i) for i in range(100)]
    path_y_sharp = [0.0] * 10 + [float(i)**2 / 100 for i in range(10)] + [float(100 - i)**2 / 100 for i in range(10, 100)]
    model_v2_msg = create_mock_model_v2_msg(path_x=path_x_sharp, path_y=path_y_sharp)
    car_state_msg = create_mock_car_state_msg(v_ego=20.0)

    monitor.detect_curve_anticipation(model_v2_msg, car_state_msg)

    assert monitor.curve_anticipation_active == True
    assert monitor.curve_anticipation_score > 0.0

def test_curve_anticipation_short_path(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    # path needs at least 3 points for gradient calculations for second derivative
    model_v2_msg = create_mock_model_v2_msg(path_x=[0.0, 0.5, 1.0], path_y=[0.0, 0.1, 0.2]) # Very short path
    car_state_msg = create_mock_car_state_msg(v_ego=20.0)

    with patch('logging.warning') as mock_warning:
        monitor.detect_curve_anticipation(model_v2_msg, car_state_msg)
        mock_warning.assert_called_with("Path is shorter than expected for 200.0m anticipation. Actual: 3 points, Expected: 400 points.")
        assert monitor.curve_anticipation_active == False # Should not activate on such short path

# Test overall safety score calculation
def test_overall_safety_score_low_model_confidence(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    # Set low model confidence
    monitor.raw_model_confidence = 0.3
    monitor.model_confidence_threshold = 0.7
    monitor.model_confidence_threshold_multiplier = 0.8 # adjusted threshold = 0.56
    
    car_state_msg = create_mock_car_state_msg()
    safety_score = monitor.calculate_overall_safety_score(car_state_msg)
    
    # Expect score to be reduced due to low model confidence
    assert safety_score < 0.5

def test_overall_safety_score_high_lane_deviation(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    # Simulate high lane deviation
    monitor.lane_deviation_filter.update(1.5) # Above default threshold 0.8
    monitor.lane_deviation_threshold = 0.8
    
    car_state_msg = create_mock_car_state_msg()
    safety_score = monitor.calculate_overall_safety_score(car_state_msg)
    
    assert safety_score < 1.0 # Should be reduced from 1.0
    # Recalculate expected value for precision
    deviation_penalty = max(0.1, 1.0 - (1.5 / 0.8)) # max(0.1, 1.0 - 1.875) = max(0.1, -0.875) = 0.1
    # Assuming other confidences are 1.0, and weights are default
    # base safety score = (1.0*0.4 + 1.0*0.3 + 1.0*0.2 + 1.0*0.1) = 1.0
    # final safety score = 1.0 * deviation_penalty = 1.0 * 0.1 = 0.1
    assert safety_score == pytest.approx(0.1, abs=1e-5)

def test_update_model_confidence_missing_data(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    model_v2_msg = create_mock_model_v2_msg(meta_confidence=None) # Simulate missing confidence
    
    with patch('logging.warning') as mock_warning:
        monitor.update_model_confidence(model_v2_msg)
        mock_warning.assert_any_call("modelV2 data not available. Defaulting model confidence to a conservative 0.1.")
        assert monitor.model_confidence == pytest.approx(0.1)
        assert monitor.raw_model_confidence == pytest.approx(0.1)
    
# Test intervention logic
def test_intervention_low_safety_score(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    car_state_msg = create_mock_car_state_msg(v_ego=20.0)
    
    # Simulate low overall safety score
    intervention = monitor.evaluate_safety_intervention_needed(0.2, car_state_msg)
    assert intervention == True

def test_intervention_low_speed_low_safety_score(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    car_state_msg = create_mock_car_state_msg(v_ego=3.0) # Low speed
    monitor.low_speed_safety_threshold = 0.4
    
    # Simulate low overall safety score at low speed
    intervention = monitor.evaluate_safety_intervention_needed(0.3, car_state_msg) # 0.3 < 0.4
    assert intervention == True

def test_intervention_no_issue(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    car_state_msg = create_mock_car_state_msg(v_ego=20.0)
    
    # Simulate no issues
    monitor.model_confidence = 0.9
    monitor.raw_model_confidence = 0.9
    monitor.lane_deviation_filter.update(0.1)
    monitor.curve_anticipation_active = False
    
    intervention = monitor.evaluate_safety_intervention_needed(0.9, car_state_msg)
    assert intervention == False

def test_missing_model_v2_data(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    # Simulate missing modelV2 message completely
    model_v2_msg = None
    radar_state_msg = create_mock_radar_state_msg()
    car_state_msg = create_mock_car_state_msg()
    car_control_msg = create_mock_car_control_msg()
    live_pose_msg = create_mock_live_pose_msg()

    with patch('logging.error') as mock_error, patch('logging.warning') as mock_warning:
        safety_score, intervention_needed, safety_report = monitor.update(
            model_v2_msg, radar_state_msg, car_state_msg, car_control_msg, live_pose_msg
        )
        # Check that errors were logged for model_v2 related updates
        # update_model_confidence now logs a warning for missing data
        mock_warning.assert_any_call("modelV2 data not available. Defaulting model confidence to a conservative 0.1.")
        mock_error.assert_any_call(f"Error in update_camera_confidence: 'NoneType' object has no attribute 'lateralPlan'")
        mock_error.assert_any_call(f"Error in detect_curve_anticipation: 'NoneType' object has no attribute 'path'")
        
        # Expect a lower safety score and intervention
        # With model confidence at 0.1, the overall score should be significantly low.
        # Using conservative lower bound for assertion.
        assert safety_score < 0.3 # Significantly reduced due to critical model data missing
        assert intervention_needed
        assert safety_report['error_occurred']

def test_missing_radar_state_data(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    # Simulate missing radarState message completely
    model_v2_msg = create_mock_model_v2_msg()
    radar_state_msg = None
    car_state_msg = create_mock_car_state_msg()
    car_control_msg = create_mock_car_control_msg()
    live_pose_msg = create_mock_live_pose_msg()

    with patch('logging.error') as mock_error:
        safety_score, intervention_needed, safety_report = monitor.update(
            model_v2_msg, radar_state_msg, car_state_msg, car_control_msg, live_pose_msg
        )
        mock_error.assert_called_with(f"Error in update_radar_confidence: 'NoneType' object has no attribute 'leadOne'")
        assert safety_score < 1.0
        assert intervention_needed
        assert safety_report['error_occurred']

def test_missing_live_pose_data(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    # Simulate missing livePose message completely
    model_v2_msg = create_mock_model_v2_msg()
    radar_state_msg = create_mock_radar_state_msg()
    car_state_msg = create_mock_car_state_msg()
    car_control_msg = create_mock_car_control_msg()
    live_pose_msg = None

    with patch('logging.error') as mock_error:
        safety_score, intervention_needed, safety_report = monitor.update(
            model_v2_msg, radar_state_msg, car_state_msg, car_control_msg, live_pose_msg
        )
        mock_error.assert_called_with(f"Error in detect_environmental_conditions: 'NoneType' object has no attribute 'orientationNED'")
        # Default safety score should still be high if other factors are good
        assert safety_score == pytest.approx(1.0) # Environmental conditions don't directly penalize score
        assert not intervention_needed
        assert safety_report['error_occurred'] # But an error did occur in environmental conditions
        assert safety_report['lighting_condition'] == "normal"
        assert safety_report['weather_condition'] == "clear"
        assert safety_report['road_condition'] == "dry"

def test_environmental_conditions_affect_weights(default_safety_monitor_instance):
    monitor = default_safety_monitor_instance
    car_state_msg = create_mock_car_state_msg()
    
    # Simulate night conditions
    monitor.lighting_condition = "night"
    
    # Update internal confidences (assuming they are all 1.0 for simplicity)
    monitor.model_confidence = 1.0
    monitor.camera_confidence = 1.0
    monitor.radar_confidence = 1.0
    monitor.imu_confidence = 1.0
    
    safety_score = monitor.calculate_overall_safety_score(car_state_msg)
    
    # Expected: (1.0 * 0.4 * 0.8) + (1.0 * 0.3) + (1.0 * 0.2 * 1.1) + (1.0 * 0.1)
    #           = 0.32 + 0.3 + 0.22 + 0.1 = 0.94
    assert safety_score == pytest.approx(0.94)

    # Simulate rain conditions
    monitor.lighting_condition = "normal" # Reset
    monitor.weather_condition = "rain"
    safety_score_rain = monitor.calculate_overall_safety_score(car_state_msg)
    
    # Expected: (1.0 * 0.4 * 0.7) + (1.0 * 0.3 * 0.6) + (1.0 * 0.2 * 1.2) + (1.0 * 0.1)
    #           = 0.28 + 0.18 + 0.24 + 0.1 = 0.8
    assert safety_score_rain == pytest.approx(0.8)

