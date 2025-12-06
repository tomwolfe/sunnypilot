"""
Comprehensive validation tests for the lightweight enhancement components.
This serves as the beginning of Phase 2: Validation and Testing.
"""

import numpy as np
import time
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


def test_scene_change_detection_comprehensive():
    """Comprehensive test of scene change detection with performance metrics."""
    print("=== Comprehensive Scene Change Detection Test ===")

    # Create separate detector for static test to avoid state interference
    static_detector = create_scene_change_detector()

    # Test with completely static scene (should skip many frames)
    static_frames = [np.full((200, 200), 128, dtype=np.uint8) for _ in range(10)]

    static_process_count = 0
    static_motion_levels = []

    start_time = time.monotonic()
    for _i, frame in enumerate(static_frames):
        should_process, motion_level = static_detector.detect_change(frame)
        if not should_process:  # Note: should_process refers to whether to skip, so if it's False, we process
            static_process_count += 1
        static_motion_levels.append(motion_level)

    static_time = time.monotonic() - start_time
    print(f"  Static scene: {static_process_count}/{len(static_frames)} frames processed ({static_process_count/len(static_frames)*100:.1f}%)")
    print(f"  Static scene: Motion levels = {[f'{x:.3f}' for x in static_motion_levels[:5]]}... (first 5)")
    print(f"  Static scene: Processing time = {static_time:.4f}s")

    # Create separate detector for dynamic test to avoid state interference
    dynamic_detector = create_scene_change_detector()

    # Test with dynamic scene (should process most frames)
    dynamic_frames = [np.random.randint(0, 255, (200, 200), dtype=np.uint8) for _ in range(10)]

    dynamic_process_count = 0
    dynamic_motion_levels = []

    start_time = time.monotonic()
    for _i, frame in enumerate(dynamic_frames):
        should_process, motion_level = dynamic_detector.detect_change(frame)
        if not should_process:  # Note: should_process refers to whether to skip, so if it's False, we process
            dynamic_process_count += 1
        dynamic_motion_levels.append(motion_level)

    dynamic_time = time.monotonic() - start_time
    print(f"  Dynamic scene: {dynamic_process_count}/{len(dynamic_frames)} frames processed ({dynamic_process_count/len(dynamic_frames)*100:.1f}%)")
    print(f"  Dynamic scene: Motion levels = {[f'{x:.3f}' for x in dynamic_motion_levels[:5]]}... (first 5)")
    print(f"  Dynamic scene: Processing time = {dynamic_time:.4f}s")

    # Validate that dynamic scene had higher motion levels on average
    avg_static_motion = np.mean(static_motion_levels)
    avg_dynamic_motion = np.mean(dynamic_motion_levels)
    print(f"  Average motion - Static: {avg_static_motion:.3f}, Dynamic: {avg_dynamic_motion:.3f}")

    # For the scene detection to work properly:
    # 1. Dynamic scene should have higher average motion than static
    # 2. In a truly dynamic scene, most/all frames should be processed (should_skip = False)
    # 3. In a static scene, fewer frames should be processed after initial ones (should_skip becomes more True)
    dynamic_higher_motion = avg_dynamic_motion > avg_static_motion
    dynamic_mostly_processed = dynamic_process_count >= len(dynamic_frames) * 0.8  # At least 80% processed

    success = (
        dynamic_higher_motion and
        dynamic_mostly_processed and
        avg_static_motion < 0.1  # Static scene should have very low motion
    )
    print(f"  ✓ Scene detection test: {'PASSED' if success else 'FAILED'}")
    print(f"    (Motion higher: {dynamic_higher_motion}, Dynamic processed: {dynamic_mostly_processed}, Static low motion: {avg_static_motion < 0.1})")
    print()

    return success


def test_coordinated_control_comprehensive():
    """Comprehensive test of coordinated control with safety validation."""
    print("=== Comprehensive Coordinated Control Test ===")

    controller, safety_limiter = create_coordinated_controller()

    # Test 1: Very low lateral demand (should not change acceleration much)
    base_acc = 1.0
    adjusted_no_lat = controller.adjust_acceleration(base_acc, 0.05, 15.0, 0.0001)  # Very small curvature
    print(f"  Very low lateral demand: {base_acc} -> {adjusted_no_lat:.3f}")

    # Test 2: Medium lateral demand (should reduce acceleration slightly)
    adjusted_med_lat = controller.adjust_acceleration(base_acc, 1.0, 15.0, 0.05)
    print(f"  Medium lateral demand: {base_acc} -> {adjusted_med_lat:.3f}")

    # Test 3: High lateral demand (should reduce acceleration significantly)
    adjusted_high_lat = controller.adjust_acceleration(base_acc, 2.0, 20.0, 0.1)
    print(f"  High lateral demand: {base_acc} -> {adjusted_high_lat:.3f}")

    # Test 4: Safety limiting with close lead vehicle
    final_acc = safety_limiter.apply_safety_limits(
        adjusted_high_lat, base_acc, 2.0, 20.0,
        lead_distance=10.0,  # Close lead
        lead_velocity=0.0    # Lead is stopped
    )
    print(f"  With safety limits (close lead): {adjusted_high_lat:.3f} -> {final_acc:.3f}")

    # Test 5: Safety limiting in sharp curve at high speed
    final_acc_curve = safety_limiter.apply_safety_limits(
        adjusted_high_lat, base_acc, 4.0, 25.0,  # High lateral accel at high speed
        lead_distance=50.0,  # No close lead
        lead_velocity=20.0
    )
    print(f"  With safety limits (sharp curve): {adjusted_high_lat:.3f} -> {final_acc_curve:.3f}")

    success = (
        abs(adjusted_no_lat - base_acc) < 0.5 and  # Very low demand should not change much
        adjusted_high_lat <= adjusted_med_lat <= adjusted_no_lat and  # Coordination should reduce acceleration
        final_acc <= -1.0  # Safety limiting should apply hard braking
    )
    print(f"  ✓ Coordinated control test: {'PASSED' if success else 'FAILED'}")
    print()

    return success


def test_edge_case_detection_comprehensive():
    """Comprehensive test of edge case detection with various scenarios."""
    print("=== Comprehensive Edge Case Detection Test ===")

    detector = create_edge_case_detector()

    # Test 1: Normal conditions (should detect no edge cases)
    normal_radar = {
        'leadOne': type('obj', (object,), {
            'status': True,
            'dRel': 100.0,  # Far away
            'vRel': 1.0     # Moving at similar speed
        })()
    }
    normal_result = detector.detect_edge_cases(normal_radar, {}, {'vEgo': 25.0})
    print(f"  Normal conditions: {len(normal_result.edge_cases)} cases detected")
    print(f"  Normal conditions: Speed multiplier: {normal_result.safe_speed_multiplier:.2f}")

    # Test 2: Stopped traffic detection
    stopped_radar = {
        'leadOne': type('obj', (object,), {
            'status': True,
            'dRel': 20.0,   # Close
            'vRel': 0.1     # Almost stopped relative to us
        })()
    }
    stopped_result = detector.detect_edge_cases(stopped_radar, {}, {'vEgo': 15.0})
    print(f"  Stopped traffic: {len(stopped_result.edge_cases)} cases detected")
    if stopped_result.edge_cases:
        case = stopped_result.edge_cases[0]
        print(f"    - {case.case_type.value}, confidence: {case.confidence:.2f}, severity: {case.severity}")
    print(f"  Stopped traffic: Speed multiplier: {stopped_result.safe_speed_multiplier:.2f}")
    print(f"  Stopped traffic: Action: {stopped_result.required_action}")

    # Test 3: Potential construction zone
    construction_radar = {
        'leadOne': type('obj', (object,), {
            'status': True,
            'dRel': 30.0,
            'vRel': 1.0  # Slow moving
        })(),
        'leadTwo': type('obj', (object,), {
            'status': True,
            'dRel': 60.0,
            'vRel': 1.5  # Also slow moving
        })()
    }
    construction_result = detector.detect_edge_cases(construction_radar, {}, {'vEgo': 8.0})
    print(f"  Construction zone: {len(construction_result.edge_cases)} cases detected")
    if construction_result.edge_cases:
        case = construction_result.edge_cases[0]
        print(f"    - {case.case_type.value}, confidence: {case.confidence:.2f}, severity: {case.severity}")
    print(f"  Construction zone: Speed multiplier: {construction_result.safe_speed_multiplier:.2f}")
    print(f"  Construction zone: Action: {construction_result.required_action}")

    # Validate results
    success = (
        len(normal_result.edge_cases) == 0 and  # No cases in normal conditions
        len(stopped_result.edge_cases) > 0 and  # Should detect stopped traffic
        stopped_result.safe_speed_multiplier < 1.0 and  # Should reduce speed
        stopped_result.required_action != "CONTINUE_NORMAL"  # Should recommend action
    )
    print(f"  ✓ Edge case detection test: {'PASSED' if success else 'FAILED'}")
    print()

    return success


def test_integration_comprehensive():
    """Comprehensive test of the integration system."""
    print("=== Comprehensive Integration Test ===")

    integrator = create_light_integrator()

    # Simulate a sequence of frames and control decisions
    frames = [
        np.full((100, 100), 128, dtype=np.uint8),  # Static frame
        np.full((100, 100), 130, dtype=np.uint8),  # Similar static frame
        np.random.randint(0, 255, (100, 100), dtype=np.uint8),  # Motion frame
        np.random.randint(0, 255, (100, 100), dtype=np.uint8),  # Motion frame
    ]

    print("  Processing frame sequence...")
    for i, frame in enumerate(frames):
        should_process, motion_level = integrator.should_process_frame(frame)
        print(f"    Frame {i+1}: Process={should_process}, Motion={motion_level:.3f}")

    # Test coordinated control through integrator
    adjusted_acc = integrator.adjust_acceleration_for_lateral_demand(
        base_acceleration=1.5,
        lateral_demand=1.2,
        speed=18.0,
        curvature=0.08,
        radar_data={
            'leadOne': type('obj', (object,), {
                'status': True,
                'dRel': 40.0,
                'vRel': 2.0
            })()
        },
        car_state={'vEgo': 18.0}
    )
    print(f"  Coordinated control through integrator: 1.5 -> {adjusted_acc:.3f}")

    # Test edge case detection through integrator
    edge_result = integrator.detect_edge_cases(
        radar_data={
            'leadOne': type('obj', (object,), {
                'status': True,
                'dRel': 25.0,
                'vRel': 0.5
            })()
        },
        vision_data={},
        car_state={'vEgo': 12.0}
    )
    print(f"  Edge case detection through integrator: {len(edge_result.edge_cases)} cases")

    # Get performance stats
    stats = integrator.get_performance_stats()
    print(f"  Performance stats: {stats['total_frames']} total, {stats['frames_skipped']} skipped, {stats['frame_skip_rate']:.1%} skip rate")

    success = (
        stats['total_frames'] == len(frames) and
        adjusted_acc < 1.5  # Should be reduced due to coordination
    )
    print(f"  ✓ Integration test: {'PASSED' if success else 'FAILED'}")
    print()

    return success


def validate_performance_requirements():
    """Validate that components meet performance requirements for Snapdragon 845."""
    print("=== Performance Requirements Validation ===")

    # Test timing for scene detection (should be fast)
    detector = create_scene_change_detector()
    test_frame = np.random.randint(0, 255, (200, 200, 3), dtype=np.uint8)  # Simulate camera frame

    # Time several iterations to get average
    times = []
    for _ in range(10):
        start = time.perf_counter()
        detector.detect_change(test_frame)
        end = time.perf_counter()
        times.append((end - start) * 1000)  # Convert to milliseconds

    avg_time_ms = np.mean(times)
    print(f"  Scene detection average time: {avg_time_ms:.3f}ms per frame")
    print(f"  Scene detection time range: {np.min(times):.3f}ms - {np.max(times):.3f}ms")

    # Test timing for coordinated control
    controller, safety_limiter = create_coordinated_controller()

    times = []
    for _ in range(10):
        start = time.perf_counter()
        acc = controller.adjust_acceleration(1.0, 1.0, 15.0, 0.05)
        safety_limiter.apply_safety_limits(acc, 1.0, 1.0, 15.0, 30.0, 10.0)
        end = time.perf_counter()
        times.append((end - start) * 1000)

    avg_control_time_ms = np.mean(times)
    print(f"  Coordinated control average time: {avg_control_time_ms:.3f}ms per calculation")
    print(f"  Coordinated control time range: {np.min(times):.3f}ms - {np.max(times):.3f}ms")

    # Performance validation
    scene_perf_ok = avg_time_ms < 5.0  # Should be well under 5ms for scene detection
    control_perf_ok = avg_control_time_ms < 2.0  # Should be well under 2ms for control

    print(f"  Scene detection performance OK: {'YES' if scene_perf_ok else 'NO'} (target: <5ms)")
    print(f"  Coordinated control performance OK: {'YES' if control_perf_ok else 'NO'} (target: <2ms)")

    overall_perf_ok = scene_perf_ok and control_perf_ok
    print(f"  ✓ Performance validation: {'PASSED' if overall_perf_ok else 'FAILED'}")
    print()

    return overall_perf_ok


def run_all_tests():
    """Run all comprehensive validation tests."""
    print("Running Comprehensive Validation Tests for Lightweight Enhancement Components\n")

    results = []
    results.append(test_scene_change_detection_comprehensive())
    results.append(test_coordinated_control_comprehensive())
    results.append(test_edge_case_detection_comprehensive())
    results.append(test_integration_comprehensive())
    results.append(validate_performance_requirements())

    print("=== Test Summary ===")
    test_names = [
        "Scene Change Detection",
        "Coordinated Control",
        "Edge Case Detection",
        "Integration",
        "Performance Requirements"
    ]

    all_passed = True
    for _i, (name, result) in enumerate(zip(test_names, results, strict=True)):
        status = "PASSED" if result else "FAILED"
        print(f"  {name}: {status}")
        all_passed = all_passed and result

    print(f"\nOverall: {'ALL TESTS PASSED ✓' if all_passed else 'SOME TESTS FAILED ✗'}")
    return all_passed


if __name__ == "__main__":
    run_all_tests()
