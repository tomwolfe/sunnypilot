import pytest
import numpy as np
import time

from openpilot.sunnypilot.selfdrive.controls.lib.traffic_light_validation import (
    create_traffic_validator, TrafficSignType, TrafficSignData
)
from openpilot.sunnypilot.selfdrive.controls.lib.traffic_sign_detection import create_traffic_sign_handler


def test_traffic_validator_add_sign():
    validator = create_traffic_validator()
    initial_time = time.time()

    # Add a traffic sign
    sign_data = TrafficSignData(
        sign_type=TrafficSignType.STOP_SIGN,
        distance=50.0,
        confidence=0.95,
        position=np.array([50.0, 0.0, 0.0])
    )
    validator.add_traffic_sign_data(sign_data)

    assert len(validator.traffic_signs) == 1
    assert validator.traffic_signs[0].sign_type == TrafficSignType.STOP_SIGN
    assert validator.traffic_signs[0].distance == 50.0
    assert validator.traffic_signs[0].confidence == 0.95
    assert np.array_equal(validator.traffic_signs[0].position, np.array([50.0, 0.0, 0.0]))
    assert validator.traffic_signs[0].timestamp >= initial_time # Check that timestamp was set


def test_traffic_validator_expiration():
    validator = create_traffic_validator()
    
    # Add a sign that is technically expired when added (timestamp is in the past)
    # It will still be added initially because filtering happens *before* adding the new sign.
    expired_sign_data = TrafficSignData(
        sign_type=TrafficSignType.STOP_SIGN,
        distance=50.0,
        confidence=0.9,
        position=np.array([50.0, 0.0, 0.0]),
        timestamp=time.time() - validator.validity_threshold - 1.0 # Ensures it's expired
    )
    validator.add_traffic_sign_data(expired_sign_data)
    # At this point, the list will contain the expired_sign_data because filtering occurs *before* adding.
    assert len(validator.traffic_signs) == 1

    # Add a new, valid sign. This call will trigger the cleanup of the previous expired sign.
    valid_sign_data = TrafficSignData(
        sign_type=TrafficSignType.TRAFFIC_LIGHT_GREEN,
        distance=20.0,
        confidence=0.85,
        position=np.array([20.0, 0.0, 0.0])
    )
    validator.add_traffic_sign_data(valid_sign_data)

    # Now, only the currently valid sign should remain
    assert len(validator.traffic_signs) == 1
    assert validator.traffic_signs[0].sign_type == TrafficSignType.TRAFFIC_LIGHT_GREEN
    assert validator.traffic_signs[0].distance == 20.0


def test_traffic_validator_red_light_violation():
    validator = create_traffic_validator()
    red_light = TrafficSignData(
        sign_type=TrafficSignType.TRAFFIC_LIGHT_RED,
        distance=30.0,
        confidence=0.9,
        position=np.array([30.0, 0.0, 0.0])
    )
    validator.add_traffic_sign_data(red_light)

    # Approaching red light too fast
    position = np.array([10.0, 0.0, 0.0]) # 10m from light
    velocity = 20.0 # m/s
    is_safe, violations, action = validator.validate_traffic_lights(position, velocity)
    assert not is_safe
    assert "APPROACHING_RED_LIGHT_TOO_FAST" in violations
    assert action == "BRAKE_IMMEDIATELY"

    # Approaching red light safely (stopped)
    position = np.array([5.0, 0.0, 0.0]) # 5m from light
    velocity = 0.0 # m/s
    is_safe, violations, action = validator.validate_traffic_lights(position, velocity)
    assert is_safe
    assert not violations
    assert action == "NONE"

    # Red light far away, speed doesn't matter
    position = np.array([100.0, 0.0, 0.0]) # 100m from light
    velocity = 20.0 # m/s
    is_safe, violations, action = validator.validate_traffic_lights(position, velocity)
    assert is_safe
    assert not violations
    assert action == "NONE"


def test_traffic_validator_stop_sign_violation():
    validator = create_traffic_validator()
    stop_sign = TrafficSignData(
        sign_type=TrafficSignType.STOP_SIGN,
        distance=30.0,
        confidence=0.9,
        position=np.array([30.0, 0.0, 0.0])
    )
    validator.add_traffic_sign_data(stop_sign)

    # Approaching stop sign too fast
    position = np.array([10.0, 0.0, 0.0]) # 10m from sign
    velocity = 5.0 # m/s (above 2.0)
    is_safe, violations, action = validator.validate_stop_signs(position, velocity)
    assert not is_safe
    assert "APPROACHING_STOP_SIGN_TOO_FAST" in violations
    assert action == "BRAKE_FOR_STOP"

    # Approaching stop sign safely (slowly)
    position = np.array([5.0, 0.0, 0.0]) # 5m from sign
    velocity = 1.0 # m/s (below 2.0)
    is_safe, violations, action = validator.validate_stop_signs(position, velocity)
    assert is_safe
    assert not violations
    assert action == "NONE"

    # Stop sign far away, speed doesn't matter
    position = np.array([100.0, 0.0, 0.0]) # 100m from sign
    velocity = 20.0 # m/s
    is_safe, violations, action = validator.validate_stop_signs(position, velocity)
    assert is_safe
    assert not violations
    assert action == "NONE"


def test_traffic_sign_detection_handler():
    handler = create_traffic_sign_handler()
    validator = create_traffic_validator()

    # Mock log.ModelDataV2 with objects
    class MockObject:
        def __init__(self, confidence, classType, x, y, z):
            self.confidence = confidence
            self.classType = classType
            self.x = x
            self.y = y
            self.z = z

    class MockModelDataV2:
        def __init__(self, objects):
            self.objects = objects

    # Test processing model output with a stop sign
    model_data_stop = MockModelDataV2(objects=[
        MockObject(confidence=0.9, classType=13, x=20.0, y=0.0, z=0.0) # 13: STOP_SIGN
    ])
    detected_signs_stop = handler.process_modeld_output(model_data_stop)
    assert len(detected_signs_stop) == 1
    assert detected_signs_stop[0].sign_type == TrafficSignType.STOP_SIGN
    assert detected_signs_stop[0].distance == 20.0

    handler.integrate_with_traffic_validator(validator, model_data_stop)
    assert len(validator.traffic_signs) == 1
    assert validator.traffic_signs[0].sign_type == TrafficSignType.STOP_SIGN

    # Test processing model output with a red light
    model_data_red_light = MockModelDataV2(objects=[
        MockObject(confidence=0.85, classType=14, x=30.0, y=0.0, z=0.0) # 14: TRAFFIC_LIGHT_RED
    ])
    detected_signs_red_light = handler.process_modeld_output(model_data_red_light)
    assert len(detected_signs_red_light) == 1
    assert detected_signs_red_light[0].sign_type == TrafficSignType.TRAFFIC_LIGHT_RED

    handler.integrate_with_traffic_validator(validator, model_data_red_light)
    assert len(validator.traffic_signs) == 2 # One stop sign, one red light

    # Test custom model output
    class MockCustomSign:
        def __init__(self, type_id, distance, confidence, x, y, z, validityTime=None, size=None, color=None, value=None):
            self.type = type_id
            self.distance = distance
            self.confidence = confidence
            self.x = x
            self.y = y
            self.z = z
            self.validityTime = validityTime
            self.size = size
            self.color = color
            self.value = value

    class MockCustomModelData:
        def __init__(self, traffic_signs):
            self.trafficSigns = traffic_signs

    custom_model_data = MockCustomModelData(traffic_signs=[
        MockCustomSign(type_id=5, distance=40.0, confidence=0.99, x=40.0, y=0.0, z=0.0, value=60) # 5: SPEED_LIMIT
    ])
    detected_custom_signs = handler.process_custom_model_output(custom_model_data)
    assert len(detected_custom_signs) == 1
    assert detected_custom_signs[0].sign_type == TrafficSignType.SPEED_LIMIT
    assert detected_custom_signs[0].additional_data['value'] == 60
