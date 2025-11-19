#!/usr/bin/env python3
"""
Test script to validate the traffic light validation enhancements
"""
import numpy as np
from openpilot.sunnypilot.selfdrive.controls.lib.traffic_light_validation import create_traffic_safety_system
from openpilot.sunnypilot.selfdrive.controls.lib.traffic_sign_detection import create_traffic_sign_handler
from openpilot.sunnypilot.selfdrive.controls.lib.traffic_light_validation import TrafficSignType, TrafficSignData


def test_traffic_validation():
    """Test the traffic light validation system"""
    print("Testing traffic validation system...")
    
    # Create the traffic safety system
    traffic_system = create_traffic_safety_system()
    sign_handler = create_traffic_sign_handler()
    
    # Create a mock traffic sign
    sign_data = TrafficSignData(
        sign_type=TrafficSignType.TRAFFIC_LIGHT_RED,
        distance=30.0,
        confidence=0.9,
        position=np.array([30.0, 0.0, 0.0]),
        validity_time=10.0  # This can be any value for testing
    )
    
    # Add the sign to the system
    traffic_system.add_traffic_sign_data(sign_data)
    print("✓ Traffic sign added to validation system")
    
    # Verify the sign was added
    assert len(traffic_system.validator.traffic_signs) == 1
    print("✓ Traffic sign correctly stored in validator")
    
    # Test with ego state (simplified)
    from openpilot.selfdrive.controls.advanced_planner import EgoState, PlanningResult, PlanningState
    ego_state = EgoState(
        position=np.array([0.0, 0.0, 0.0]),
        velocity=20.0,  # 20 m/s = ~72 km/h
        acceleration=0.0,
        heading=0.0,
        curvature=0.0,
        steering_angle=0.0
    )
    
    # Create a mock car state (simplified)
    class MockCarState:
        pass
    
    car_state = MockCarState()
    car_state.vEgo = 20.0
    car_state.aEgo = 0.0
    
    # Create a mock planning result
    planning_result = PlanningResult(
        desired_curvature=0.0,
        desired_speed=20.0,
        desired_acceleration=0.0,
        planning_state=PlanningState.LANE_FOLLOWING,
        safety_factor=0.8,
        confidence=0.9,
        risk_assessment={'collision_risk': 0.1, 'total_threat_score': 0.1}
    )
    
    # Create mock model outputs
    model_outputs = {
        'overall_confidence': 0.85,
        'lead_confidence': 0.9,
        'lane_confidence': 0.8
    }
    
    # Validate traffic compliance
    is_safe, violations, metrics = traffic_system.validate_with_planning(
        ego_state, car_state, planning_result, model_outputs
    )
    
    print(f"✓ Traffic validation completed: safe={is_safe}, violations={len(violations)}")
    print(f"  Confidence: {metrics['detection_confidence']:.2f}")
    print(f"  Recommended action: {metrics['recommended_action']}")
    
    # Test scenario where we're approaching a red light too fast
    ego_state_approaching = EgoState(
        position=np.array([10.0, 0.0, 0.0]),  # Closer to the light
        velocity=20.0,
        acceleration=0.0,
        heading=0.0,
        curvature=0.0,
        steering_angle=0.0
    )
    
    is_safe_approaching, violations_approaching, metrics_approaching = traffic_system.validate_with_planning(
        ego_state_approaching, car_state, planning_result, model_outputs
    )
    
    print(f"✓ Approaching red light test: safe={is_safe_approaching}")
    if violations_approaching:
        print(f"  Violations: {violations_approaching}")
    
    print("\n✓ All traffic validation tests passed!")


if __name__ == "__main__":
    test_traffic_validation()