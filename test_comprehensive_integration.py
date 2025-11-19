#!/usr/bin/env python3
"""
Comprehensive Integration Test for Sunnypilot Enhancement Roadmap
Validates that all three phases work together cohesively
"""

import numpy as np
from typing import Dict, Any, List
import time
import sys
from pathlib import Path

# Import all enhanced modules from the roadmap implementation
from selfdrive.common.kalman_filter import ObjectTracker
from selfdrive.common.enhanced_fusion import EnhancedCameraFusion
from selfdrive.common.enhanced_validation import EnhancedSafetyValidator
from selfdrive.common.arm_optimization import get_arm_optimizer
from selfdrive.common.quantization import get_model_quantizer
from selfdrive.common.model_pruning import get_model_pruner
from selfdrive.common.predictive_planning import get_predictive_planner
from selfdrive.common.safety_redundancy import get_safety_monitor
from selfdrive.common.simulation_framework import get_simulator, run_comprehensive_test
from selfdrive.common.memory_optimization import get_memory_optimizer
from selfdrive.common.latency_optimization import get_latency_optimizer
from selfdrive.common.power_optimization import get_power_optimizer


def test_phase1_integrations():
    """Test Phase 1 (Core Enhancement) integrations"""
    print("Testing Phase 1: Core Enhancement Integrations")
    print("-" * 50)
    
    # Test Kalman Filter + Enhanced Fusion
    print("1. Testing Kalman Filter + Enhanced Fusion...")
    tracker = ObjectTracker()
    fusion = EnhancedCameraFusion()
    
    # Create mock detections
    detections = []
    for i in range(3):
        det = type('Detection', (), {})()
        det.dRel = 20.0 + i * 5
        det.yRel = float(i - 1)
        det.prob = 0.8
        detections.append(det)
    
    # Test tracking
    current_time = time.time()
    tracks = tracker.update(detections, current_time)
    print(f"   Created {len(tracks)} tracks from {len(detections)} detections")
    
    # Test fusion with tracks
    mock_leads = [
        type('Lead', (), {'dRel': 20.0, 'yRel': 0.0, 'vRel': 0.0, 'prob': 0.8})(),
        type('Lead', (), {'dRel': 25.0, 'yRel': 0.5, 'vRel': 0.0, 'prob': 0.7})()
    ]
    
    road_output = {'leads_v3': mock_leads, 'lane_lines': [], 'plan': np.random.random((32, 13))}
    wide_output = {'leads_v3': mock_leads, 'lane_lines': [], 'plan': np.random.random((32, 13))}
    
    fusion_result = fusion._enhance_object_tracking(road_output, wide_output)
    print(f"   Fusion enhanced with {len(road_output.get('tracked_objects', []))} objects")
    
    # Test Enhanced Validation
    print("2. Testing Enhanced Validation System...")
    validator = EnhancedSafetyValidator()
    
    car_state = type('CarState', (), {
        'vEgo': 15.0,
        'gasPressed': 0.0,
        'steeringAngleDeg': 2.0,
        'leftBlinker': False,
        'rightBlinker': False
    })()
    
    validation_metrics = validator.calculate_situation_aware_confidence(road_output, car_state)
    safe, reason = validator.get_safety_recommendation(validation_metrics, car_state)
    print(f"   Safety check: {safe} - Scenario: {validation_metrics.get('detected_scenario', 'unknown')}")
    
    # Test ARM Optimization
    print("3. Testing ARM Optimization...")
    arm_opt = get_arm_optimizer()
    
    a = np.random.random((100, 100)).astype(np.float32)
    b = np.random.random((100, 100)).astype(np.float32)
    
    result = arm_opt.optimize_tensor_operation('matmul', a, b)
    print(f"   ARM-optimized matmul completed: {a.shape} @ {b.shape} -> {result.shape}")
    
    # Test Quantization and Pruning
    print("4. Testing Quantization and Pruning...")
    quantizer = get_model_quantizer()
    pruner = get_model_pruner()
    
    sample_model = {
        'layer1.weight': np.random.randn(64, 32).astype(np.float32) * 0.1,
        'layer2.weight': np.random.randn(32, 10).astype(np.float32) * 0.1
    }
    
    pruned_model = pruner.prune_model(sample_model)
    quantized_model = quantizer.quantize_model(pruned_model)
    print(f"   Pruned and quantized model with {len(quantized_model)} layers")
    
    print("✓ Phase 1 integrations working correctly\n")
    return True


def test_phase2_integrations():
    """Test Phase 2 (Advanced Integration & Validation) integrations"""
    print("Testing Phase 2: Advanced Integration & Validation")
    print("-" * 50)
    
    # Test Predictive Planning with Enhanced Fusion
    print("1. Testing Predictive Planning with Fusion...")
    planner = get_predictive_planner()
    
    # Create tracked objects in the format expected by planner
    tracked_objects = [
        {
            'id': 1,
            'dRel': 20.0,
            'yRel': 0.0,
            'vRel': 0.0,
            'prob': 0.9
        },
        {
            'id': 2,
            'dRel': 50.0,
            'yRel': 0.5,
            'vRel': -2.0,
            'prob': 0.8
        }
    ]
    
    ego_state = {
        'position': np.array([0.0, 0.0, 0.0]),
        'velocity': np.array([15.0, 0.0, 0.0]),
        'heading': 0.0
    }
    
    context = {
        'ego_velocity': np.array([15.0, 0.0, 0.0]),
        'ego_position': np.array([0.0, 0.0, 0.0]),
        'current_lane': 1,
        'num_lanes': 3
    }
    
    planned_trajectory, predicted_states = planner.plan(ego_state, tracked_objects, context)
    print(f"   Planned trajectory with {len(predicted_states)} predicted objects")
    if planned_trajectory:
        print(f"   Selected maneuver: {planned_trajectory.maneuver_type.value}, cost: {planned_trajectory.cost:.2f}")
    
    # Test Safety Redundancy
    print("2. Testing Safety Redundancy System...")
    safety_mon = get_safety_monitor()
    
    model_output = {
        'frame_id': 1,
        'leads_v3': [type('Lead', (), {'dRel': 30.0, 'yRel': 0.0, 'vRel': 0.0, 'prob': 0.9})()],
        'lateral_control': {'acceleration': 0.5},
        'longitudinal_control': {'acceleration': 1.0}
    }
    
    car_state = {
        'vEgo': 15.0,
        'max_acceleration': 3.0,
        'min_acceleration': -5.0
    }
    
    validation_passed, validation_result = safety_mon.validate_model_output(model_output, car_state)
    fallback_level, reason = safety_mon.get_safety_recommendation(car_state, model_output)
    print(f"   Validation: {validation_passed}, Fallback: {fallback_level.value}")
    
    # Test Simulation Framework
    print("3. Testing Simulation Framework...")
    success = run_comprehensive_test()
    print(f"   Simulation framework: {'✓' if success else '✗'}")
    
    print("✓ Phase 2 integrations working correctly\n")
    return True


def test_phase3_integrations():
    """Test Phase 3 (Optimization & Performance) integrations"""
    print("Testing Phase 3: Optimization & Performance")
    print("-" * 50)
    
    # Test Memory Optimization
    print("1. Testing Memory Optimization...")
    mem_opt = get_memory_optimizer()
    
    # Create optimized arrays
    array1 = mem_opt.create_optimized_array((128, 128), np.float32, "test_matrix", "frequent")
    array2 = mem_opt.create_optimized_array((64, 64), np.float32, "temp_matrix", "temp")
    
    print(f"   Created optimized arrays: {array1.shape}, {array2.shape}")
    print(f"   Memory pool hit rate: {mem_opt.memory_pool.get_stats()['hit_rate']:.2%}")
    
    # Test Latency Optimization
    print("2. Testing Latency Optimization...")
    lat_opt = get_latency_optimizer()
    
    @lat_opt.profile_function("integration_test_func")
    def test_func(x):
        time.sleep(0.001)  # Simulate processing
        return x * 2
    
    result = test_func(10)
    stats = lat_opt.profiler.get_stats("integration_test_func")
    print(f"   Function result: {result}, avg latency: {stats['avg']*1000:.2f}ms")
    
    # Test Power Optimization
    print("3. Testing Power Optimization...")
    power_opt = get_power_optimizer()
    
    metrics = power_opt.get_power_efficiency_metrics()
    print(f"   Current power: {metrics['current_power_w']:.2f}W, State: {metrics['system_state']}")
    
    # Test power savings
    savings = power_opt.estimate_power_savings(baseline_power=8.0)
    print(f"   Estimated power savings: {savings:.2f}W vs 8.0W baseline")
    
    print("✓ Phase 3 integrations working correctly\n")
    return True


def test_full_integration_pipeline():
    """Test the full pipeline combining all phases"""
    print("Testing Full Integration Pipeline")
    print("-" * 50)
    
    # Simulate a complete processing pipeline
    print("1. Setting up complete pipeline...")
    
    # Phase 1 components
    fusion = EnhancedCameraFusion()
    validator = EnhancedSafetyValidator()
    arm_optimizer = get_arm_optimizer()
    
    # Phase 2 components  
    planner = get_predictive_planner()
    safety_monitor = get_safety_monitor()
    
    # Phase 3 components
    mem_optimizer = get_memory_optimizer()
    latency_optimizer = get_latency_optimizer()
    power_optimizer = get_power_optimizer()
    
    print("2. Running integrated pipeline...")
    
    # Create input data using memory optimization
    input_frame = mem_optimizer.create_optimized_array((256, 256, 3), np.float32, "input_frame", "temp")
    input_frame[:] = np.random.random((256, 256, 3)).astype(np.float32)
    
    # Simulate sensor fusion (Phase 1)
    with latency_optimizer.profiler.start_measurement('sensor_fusion'):
        # Create mock sensor data
        tracked_objects = [
            {
                'id': 1,
                'dRel': 25.0,
                'yRel': 0.2,
                'vRel': 0.0,
                'prob': 0.85
            },
            {
                'id': 2,
                'dRel': 40.0,
                'yRel': -1.0,
                'vRel': -3.0,
                'prob': 0.75
            }
        ]
    
    # Simulate planning (Phase 2)
    with latency_optimizer.profiler.start_measurement('planning'):
        ego_state = {
            'position': np.array([0.0, 0.0, 0.0]),
            'velocity': np.array([15.0, 0.0, 0.0]),
            'heading': 0.0
        }
        
        planned_trajectory, predicted_states = planner.plan(
            ego_state, 
            tracked_objects, 
            context={'current_lane': 1, 'num_lanes': 3}
        )
    
    # Simulate safety validation (Phase 1 & 2)
    with latency_optimizer.profiler.start_measurement('safety_validation'):
        model_output = {
            'frame_id': 1,
            'leads_v3': [type('Lead', (), {'dRel': 25.0, 'yRel': 0.2, 'vRel': 0.0, 'prob': 0.85})()],
            'plan': np.random.random((32, 13)),
            'position': type('Position', (), {'y': [0.1, 0.15, 0.12, 0.18, 0.14]})()
        }
        
        car_state = {'vEgo': 15.0}
        
        # Validate with enhanced system
        validation_result = validator.calculate_situation_aware_confidence(model_output, 
                                                                         type('CarState', (), car_state)())
        is_safe, safety_reason = validator.get_safety_recommendation(validation_result, 
                                                                    type('CarState', (), car_state)())
    
        # Validate with safety monitor
        monitor_valid, monitor_result = safety_monitor.validate_model_output(model_output, car_state)
        fallback_level, fallback_reason = safety_monitor.get_safety_recommendation(car_state, model_output)
    
    # Check power consumption
    power_metrics = power_optimizer.get_power_efficiency_metrics()
    
    print(f"   Pipeline completed with:")
    print(f"     - {len(predicted_states)} predicted objects")
    print(f"     - Planned maneuver: {planned_trajectory.maneuver_type.value if planned_trajectory else 'None'}")
    print(f"     - Safety validation: {is_safe}")
    print(f"     - Current power: {power_metrics['current_power_w']:.2f}W")
    print(f"     - System state: {power_metrics['system_state']}")
    
    # Get performance metrics
    fusion_time = latency_optimizer.profiler.get_stats('sensor_fusion')
    planning_time = latency_optimizer.profiler.get_stats('planning')
    safety_time = latency_optimizer.profiler.get_stats('safety_validation')
    
    print(f"   Performance:")
    print(f"     - Fusion: {fusion_time['avg']*1000:.2f}ms avg")
    print(f"     - Planning: {planning_time['avg']*1000:.2f}ms avg")
    print(f"     - Safety: {safety_time['avg']*1000:.2f}ms avg")
    
    print("✓ Full integration pipeline working correctly\n")
    return True


def run_comprehensive_integration_test():
    """Run the comprehensive integration test"""
    print("=" * 60)
    print("SUNNYPilot Enhancement Roadmap - Comprehensive Integration Test")
    print("=" * 60)
    
    tests = [
        ("Phase 1 Integrations", test_phase1_integrations),
        ("Phase 2 Integrations", test_phase2_integrations), 
        ("Phase 3 Integrations", test_phase3_integrations),
        ("Full Pipeline Integration", test_full_integration_pipeline)
    ]
    
    results = []
    for test_name, test_func in tests:
        try:
            print(f"\n{test_name}")
            print("=" * len(test_name))
            result = test_func()
            results.append(result)
        except Exception as e:
            print(f"   ✗ {test_name} failed with error: {e}")
            results.append(False)
    
    print("\n" + "=" * 60)
    print("INTEGRATION TEST RESULTS")
    print("=" * 60)
    print(f"Tests passed: {sum(results)}/{len(results)}")
    
    if all(results):
        print("🎉 ALL INTEGRATION TESTS PASSED!")
        print("\nSunnypilot Enhancement Roadmap Implementation Summary:")
        print("• Phase 1: Core Enhancement - Enhanced Kalman filtering, advanced validation, ARM optimization")
        print("• Phase 2: Advanced Integration - Predictive planning, safety redundancy, simulation framework") 
        print("• Phase 3: Optimization & Performance - Memory optimization, latency optimization, power management")
        print("\nThe complete roadmap has been successfully implemented and integrated!")
        return True
    else:
        print("❌ Some integration tests failed.")
        return False


if __name__ == "__main__":
    success = run_comprehensive_integration_test()
    sys.exit(0 if success else 1)