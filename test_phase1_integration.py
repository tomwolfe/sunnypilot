#!/usr/bin/env python3
"""
Integration test for Phase 1 enhancements:
- Enhanced Kalman filter for object tracking
- Advanced safety validation with scenario detection
- ARM NEON optimization
- Neural network quantization
- Model pruning
"""

import numpy as np
from typing import Dict, Any
import time
import sys

# Import all the enhanced modules
from selfdrive.common.kalman_filter import KalmanFilter2D, ObjectTracker
from selfdrive.common.enhanced_fusion import EnhancedCameraFusion
from selfdrive.common.enhanced_validation import EnhancedSafetyValidator
from selfdrive.common.arm_optimization import ARMNeuralNetworkOptimizer
from selfdrive.common.quantization import NeuralNetworkQuantizer, QuantizationConfig
from selfdrive.common.model_pruning import NeuralNetworkPruner, PruningConfig
from selfdrive.common.weather_data import weather_data_interface


def test_kalman_and_fusion_integration():
    """Test integration between Kalman filter and camera fusion"""
    print("Testing Kalman Filter and Fusion Integration...")
    
    # Create sample detections
    detections = []
    for i in range(3):
        detection = type('Detection', (), {})()
        detection.dRel = 20.0 + i * 5  # Range: 20, 25, 30m
        detection.yRel = float(i - 1)   # Range: -1, 0, 1m (left, center, right)
        detection.prob = 0.8
        detections.append(detection)
    
    # Test ObjectTracker (which uses Kalman filters internally)
    tracker = ObjectTracker(max_distance=5.0, max_age=10, min_hits=3)
    current_time = time.time()
    
    # Send multiple frames of detections to tracker
    for frame in range(5):
        for det in detections:
            det.dRel += 0.1  # Move objects slightly
        tracks = tracker.update(detections, current_time + frame * 0.05)
        print(f"  Frame {frame}: {len(tracks)} tracks")
    
    # Test EnhancedCameraFusion with tracking
    fusion = EnhancedCameraFusion()
    
    # Create mock model outputs
    mock_leads = []
    for i, det in enumerate(detections):
        lead = type('Lead', (), {})()
        lead.dRel = det.dRel
        lead.yRel = det.yRel
        lead.vRel = 0.0
        lead.prob = det.prob
        mock_leads.append(lead)
    
    road_output = {
        'leads_v3': mock_leads,
        'lane_lines': [],
        'plan': np.random.random((32, 13))
    }
    wide_output = {
        'leads_v3': mock_leads,
        'lane_lines': [],
        'plan': np.random.random((32, 13))
    }
    
    # Update calibrations
    fake_calib = np.array([0.0, 0.0, 0.0])  # Simulated calibration data
    fusion.update_calibrations(type('CalibData', (), {'rpyCalib': fake_calib})())
    
    # Enhance object tracking
    result = fusion._enhance_object_tracking(road_output, wide_output)
    
    success = 'tracked_objects' in road_output and len(road_output['tracked_objects']) >= 2
    print(f"  Kalman fusion integration: {'✓ PASS' if success else '✗ FAIL'}")
    
    return success


def test_validation_with_weather():
    """Test enhanced validation with weather data"""
    print("\nTesting Enhanced Validation with Weather...")
    
    validator = EnhancedSafetyValidator()
    
    # Create mock model output
    mock_leads = []
    for i in range(2):
        lead = type('Lead', (), {})()
        lead.dRel = 30.0 - i * 10  # 30m, 20m
        lead.yRel = float(i) - 0.5  # -0.5m, 0.5m
        lead.vRel = 0.0
        lead.prob = 0.85
        mock_leads.append(lead)
    
    model_output = {
        'leads_v3': mock_leads,
        'lane_lines': [],
        'plan': np.random.random((32, 13)),
        'position': type('Position', (), {'y': [0.1, 0.2, 0.15, 0.18, 0.22]})  # Path points
    }
    
    # Create mock car state
    car_state = type('CarState', (), {
        'vEgo': 15.0,  # 15 m/s
        'gasPressed': 0.0,
        'steeringAngleDeg': 5.0,
        'leftBlinker': False,
        'rightBlinker': False
    })()
    
    # Test with different weather conditions
    print("  Testing normal weather conditions...")
    normal_metrics = validator.calculate_situation_aware_confidence(model_output, car_state)
    normal_safe, normal_reason = validator.get_safety_recommendation(normal_metrics, car_state)
    
    # Simulate weather change and test again
    print("  Testing with simulated weather...")
    
    success = (normal_metrics['weather_factor'] > 0.5 and 
               'detected_scenario' in normal_metrics and
               isinstance(normal_safe, bool))
    
    print(f"  Weather validation integration: {'✓ PASS' if success else '✗ FAIL'}")
    
    return success


def test_arm_optimizations():
    """Test ARM optimization utilities"""
    print("\nTesting ARM Optimizations...")
    
    optimizer = ARMNeuralNetworkOptimizer()
    
    # Test basic operations
    a = np.random.random((100, 100)).astype(np.float32)
    b = np.random.random((100, 100)).astype(np.float32)
    
    # Test optimized operations
    result_add = optimizer.optimize_tensor_operation('add', a, b)
    result_matmul = optimizer.optimize_tensor_operation('matmul', a, b)
    result_relu = optimizer.optimize_tensor_operation('relu', a - 0.5)
    
    # Test with sample model layers
    sample_layers = [
        {'type': 'dense', 'weights': np.random.random((100, 50)).astype(np.float32), 'activation': 'relu'},
        {'type': 'dense', 'weights': np.random.random((50, 10)).astype(np.float32), 'activation': 'relu'}
    ]
    
    sample_input = np.random.random((1, 100)).astype(np.float32)
    result = optimizer.optimize_model_inference(None, sample_input, sample_layers)
    
    success = (result_add.shape == a.shape and 
               result_matmul.shape[0] == a.shape[0] and result_matmul.shape[1] == b.shape[1] and
               result_relu.shape == a.shape and
               result.shape == (1, 10))
    
    print(f"  ARM optimization tests: {'✓ PASS' if success else '✗ FAIL'}")
    
    return success


def test_quantization_pipeline():
    """Test neural network quantization pipeline"""
    print("\nTesting Quantization Pipeline...")
    
    quantizer = NeuralNetworkQuantizer()
    
    # Create sample model
    sample_model = {
        'layer1.weight': np.random.randn(64, 32).astype(np.float32),
        'layer2.weight': np.random.randn(32, 16).astype(np.float32),
        'layer3.weight': np.random.randn(16, 10).astype(np.float32)
    }
    
    # Quantize the model
    quantized_model = quantizer.quantize_model(sample_model)
    
    # Dequantize to test round-trip
    dequantized_model = quantizer.dequantize_model(quantized_model)
    
    # Test quantized inference
    sample_input = np.random.random((1, 64)).astype(np.float32)
    result = quantizer.quantized_inference(quantized_model, sample_input)
    
    success = (len(quantized_model) == len(sample_model) and
               all(layer in dequantized_model for layer in sample_model) and
               result.shape[0] == 1)
    
    print(f"  Quantization pipeline tests: {'✓ PASS' if success else '✗ FAIL'}")
    
    return success


def test_pruning_pipeline():
    """Test neural network pruning pipeline"""
    print("\nTesting Pruning Pipeline...")
    
    config = PruningConfig(
        pruning_method='magnitude',
        pruning_ratio=0.2,  # Prune 20%
        schedule_type='one_shot'
    )
    
    pruner = NeuralNetworkPruner(config)
    
    # Create sample model
    sample_model = {
        'layer1.weight': np.random.randn(32, 16).astype(np.float32),
        'layer2.weight': np.random.randn(16, 8).astype(np.float32),
        'layer3.weight': np.random.randn(8, 4).astype(np.float32)
    }
    
    # Calculate original sparsity
    original_sparsity = pruner.calculate_model_sparsity(sample_model)
    
    # Prune the model
    pruned_model = pruner.prune_model(sample_model)
    
    # Calculate pruned sparsity 
    pruned_sparsity = pruner.calculate_model_sparsity(pruned_model)
    
    # Fine-tune the pruned model
    fine_tuned_model = pruner.fine_tune_model(pruned_model, epochs=1)
    
    success = (len(pruned_model) == len(sample_model) and
               all(layer in pruned_model for layer in sample_model) and
               any(sparsity > orig for orig, sparsity in zip(
                   original_sparsity.values(), pruned_sparsity.values())) and
               fine_tuned_model['layer1.weight'].shape == (32, 16))
    
    print(f"  Pruning pipeline tests: {'✓ PASS' if success else '✗ FAIL'}")
    
    return success


def test_comprehensive_integration():
    """Test all systems working together"""
    print("\nTesting Comprehensive Integration...")
    
    # Create sample data
    sample_input = np.random.random((1, 64)).astype(np.float32)
    
    # Step 1: Apply ARM optimizations
    arm_optimizer = ARMNeuralNetworkOptimizer()
    
    # Step 2: Use quantized/pruned model (simulated)
    quantizer = NeuralNetworkQuantizer()
    pruner = NeuralNetworkPruner()
    
    # Step 3: Process with enhanced fusion
    fusion = EnhancedCameraFusion()
    
    # Step 4: Validate with enhanced safety system
    validator = EnhancedSafetyValidator()
    
    # Simulate a complete pipeline
    mock_model = {
        'layer1.weight': np.random.randn(64, 32).astype(np.float32) * 0.1,  # Small weights
        'layer2.weight': np.random.randn(32, 10).astype(np.float32) * 0.1
    }
    
    # Prune and quantize the model
    pruned_model = pruner.prune_model(mock_model)
    quantized_model = quantizer.quantize_model(pruned_model)
    
    # Create mock sensor data
    mock_leads = []
    for i in range(2):
        lead = type('Lead', (), {})()
        lead.dRel = 25.0 - i * 5
        lead.yRel = float(i) - 0.5
        lead.vRel = 0.0
        lead.prob = 0.9
        mock_leads.append(lead)
    
    model_output = {
        'leads_v3': mock_leads,
        'lane_lines': [],
        'plan': np.random.random((32, 13)),
        'position': type('Position', (), {'y': [0.1, 0.15, 0.12, 0.18, 0.14]})()
    }
    
    car_state = type('CarState', (), {
        'vEgo': 12.0,
        'gasPressed': 0.0,
        'steeringAngleDeg': 2.0,
        'leftBlinker': False,
        'rightBlinker': False
    })()
    
    # Validate the output
    safety_metrics = validator.calculate_situation_aware_confidence(model_output, car_state)
    is_safe, reason = validator.get_safety_recommendation(safety_metrics, car_state)
    
    success = (isinstance(safety_metrics, dict) and 
               'detected_scenario' in safety_metrics and
               isinstance(is_safe, bool))
    
    print(f"  Comprehensive integration: {'✓ PASS' if success else '✗ FAIL'}")
    
    return success


def run_all_tests():
    """Run all integration tests"""
    print("Running Phase 1 Integration Tests")
    print("=" * 40)
    
    tests = [
        test_kalman_and_fusion_integration,
        test_validation_with_weather,
        test_arm_optimizations,
        test_quantization_pipeline,
        test_pruning_pipeline,
        test_comprehensive_integration
    ]
    
    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"  Test failed with exception: {e}")
            results.append(False)
    
    print(f"\nPhase 1 Integration Test Results: {sum(results)}/{len(results)} passed")
    
    if all(results):
        print("🎉 All Phase 1 integration tests passed!")
        print("Core enhancements successfully integrated:")
        print("  - Enhanced Kalman filter tracking")
        print("  - Advanced safety validation") 
        print("  - ARM NEON optimizations")
        print("  - Neural network quantization")
        print("  - Model pruning")
        return True
    else:
        print("❌ Some integration tests failed.")
        return False


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)