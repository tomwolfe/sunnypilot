"""
Basic tests for the lightweight enhancement components.
"""

import numpy as np
from sunnypilot.lightweight.scene_detection.detector import (
    create_scene_change_detector
)
from sunnypilot.lightweight.coordinated_control.controller import (
    create_coordinated_controller
)
from sunnypilot.lightweight.edge_case_detection.detector import (
    create_edge_case_detector
)
from sunnypilot.lightweight.integration import (
    create_light_integrator
)


def test_scene_change_detection():
    """Test the lightweight scene change detector."""
    print("Testing Scene Change Detection...")

    detector = create_scene_change_detector()

    # Create two identical frames (static scene)
    static_frame1 = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
    static_frame2 = static_frame1.copy()

    # First frame should always process
    should_process1, motion1 = detector.detect_change(static_frame1)
    print(f"  First frame - Process: {should_process1}, Motion: {motion1:.3f}")

    # Second identical frame should be detected as static
    should_process2, motion2 = detector.detect_change(static_frame2)
    print(f"  Second frame (static) - Process: {should_process2}, Motion: {motion2:.3f}")

    # Create a different frame (motion)
    motion_frame = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
    should_process3, motion3 = detector.detect_change(motion_frame)
    print(f"  Third frame (motion) - Process: {should_process3}, Motion: {motion3:.3f}")

    print("  ✓ Scene change detection test completed\n")


def test_coordinated_control():
    """Test the simplified coordinated controller."""
    print("Testing Coordinated Control...")

    controller, safety_limiter = create_coordinated_controller()

    # Test with low lateral demand (should not change acceleration much)
    base_accel = 1.0
    low_lateral_demand = 0.1
    speed = 20.0
    curvature = 0.001

    adjusted_accel1 = controller.adjust_acceleration(base_accel, low_lateral_demand, speed, curvature)
    print(f"  Low lateral demand - Base: {base_accel}, Adjusted: {adjusted_accel1:.3f}")

    # Test with high lateral demand (should reduce acceleration)
    high_lateral_demand = 2.0
    adjusted_accel2 = controller.adjust_acceleration(base_accel, high_lateral_demand, speed, curvature)
    print(f"  High lateral demand - Base: {base_accel}, Adjusted: {adjusted_accel2:.3f}")

    # Test safety limiting
    final_accel = safety_limiter.apply_safety_limits(
        adjusted_accel2, base_accel, high_lateral_demand, speed,
        lead_distance=10.0, lead_velocity=0.0
    )
    print(f"  With safety limits - Final: {final_accel:.3f}")

    print("  ✓ Coordinated control test completed\n")


def test_edge_case_detection():
    """Test the basic edge case detector."""
    print("Testing Edge Case Detection...")

    detector = create_edge_case_detector()

    # Create test data simulating stopped traffic
    radar_data = {
        'leadOne': type('obj', (object,), {
            'status': True,
            'dRel': 30.0,  # 30m away
            'vRel': 0.1    # Very slow relative velocity (stopped traffic)
        })(),
        'leadTwo': type('obj', (object,), {
            'status': False,
            'dRel': 0.0,
            'vRel': 0.0
        })()
    }

    vision_data = {}
    car_state = {'vEgo': 15.0}  # Going at 15 m/s

    result = detector.detect_edge_cases(radar_data, vision_data, car_state)

    print(f"  Detected {len(result.edge_cases)} edge cases")
    print(f"  Safe speed multiplier: {result.safe_speed_multiplier:.2f}")
    print(f"  Required action: {result.required_action}")

    for case in result.edge_cases:
        print(f"    - {case.case_type.value}: confidence={case.confidence:.2f}, severity={case.severity}")

    print("  ✓ Edge case detection test completed\n")


def test_integration():
    """Test the light enhancement integrator."""
    print("Testing Light Enhancement Integrator...")

    integrator = create_light_integrator()

    # Simulate a static scene
    static_frame1 = np.full((100, 100), 128, dtype=np.uint8)  # Gray frame
    static_frame2 = static_frame1.copy()

    # Process frames
    process1, motion1 = integrator.should_process_frame(static_frame1)
    process2, motion2 = integrator.should_process_frame(static_frame2)

    print(f"  Frame 1 - Process: {process1}, Motion: {motion1:.3f}")
    print(f"  Frame 2 - Process: {process2}, Motion: {motion2:.3f}")

    # Simulate adjustment with coordination
    adjusted_accel = integrator.adjust_acceleration_for_lateral_demand(
        base_acceleration=1.0,
        lateral_demand=0.5,
        speed=20.0,
        curvature=0.005,
        radar_data={
            'leadOne': type('obj', (object,), {
                'status': True,
                'dRel': 50.0,
                'vRel': 1.0
            })()
        },
        car_state={'vEgo': 20.0}
    )

    print(f"  Acceleration adjustment - Original: 1.0, Adjusted: {adjusted_accel:.3f}")

    # Get performance stats
    stats = integrator.get_performance_stats()
    print(f"  Performance - Total frames: {stats['total_frames']}, Skipped: {stats['frames_skipped']}, Skip rate: {stats['frame_skip_rate']:.2%}")

    print("  ✓ Integration test completed\n")


if __name__ == "__main__":
    print("Running Basic Tests for Lightweight Enhancement Components\n")

    test_scene_change_detection()
    test_coordinated_control()
    test_edge_case_detection()
    test_integration()

    print("All tests completed successfully! ✓")
