#!/usr/bin/env python3
"""
Validation tests for the enhanced Kalman filter fusion system
Tests tracking accuracy, consistency, and performance under various conditions
"""
import numpy as np
import time
import matplotlib.pyplot as plt
from typing import List, Dict, Any
from selfdrive.common.kalman_filter import KalmanFilter2D, ObjectTracker
from selfdrive.common.enhanced_fusion import EnhancedCameraFusion


def test_kalman_filter_accuracy():
    """Test the accuracy of the 2D Kalman filter"""
    print("Testing Kalman Filter Accuracy...")
    
    # Create a known trajectory (moving object)
    dt = 0.05  # 20 Hz
    time_steps = 100
    true_positions = []
    measurements = []
    
    # Generate true positions (constant velocity motion with some acceleration)
    pos = np.array([0.0, 0.0])  # Initial position
    vel = np.array([10.0, 2.0])  # Initial velocity (10 m/s in x, 2 m/s in y)
    
    for t in range(time_steps):
        # Update true position
        pos = pos + vel * dt
        # Add small acceleration
        vel = vel + np.array([0.1, 0.02]) * dt
        
        true_positions.append(pos.copy())
        
        # Add noise to measurements
        noise = np.random.normal(0, 0.5, size=2)  # 0.5m std dev
        measurements.append(pos + noise)
    
    # Test Kalman filter
    kf = KalmanFilter2D(dt=dt, process_noise=0.1, measurement_noise=0.25)
    
    # Initialize with first measurement
    kf.x[:2] = measurements[0]
    
    estimated_positions = []
    covariances = []
    
    for measurement in measurements:
        kf.predict()
        kf.update(measurement)
        estimated_positions.append(kf.get_position().copy())
        covariances.append(kf.P.diagonal().copy())
    
    # Calculate errors
    true_positions = np.array(true_positions)
    estimated_positions = np.array(estimated_positions)
    measurements = np.array(measurements)
    
    position_errors = np.linalg.norm(estimated_positions - true_positions, axis=1)
    measurement_errors = np.linalg.norm(measurements - true_positions, axis=1)
    
    avg_kf_error = np.mean(position_errors)
    avg_measurement_error = np.mean(measurement_errors)
    
    print(f"  Average Kalman filter position error: {avg_kf_error:.3f}m")
    print(f"  Average measurement error: {avg_measurement_error:.3f}m")
    print(f"  Improvement: {(avg_measurement_error - avg_kf_error) / avg_measurement_error * 100:.1f}%")
    
    # Check if Kalman filter is working properly
    success = avg_kf_error < avg_measurement_error * 0.8  # Should be significantly better
    print(f"  Kalman filter test: {'✓ PASS' if success else '✗ FAIL'}")
    
    return success


def test_object_tracker():
    """Test the multi-object tracker"""
    print("\nTesting Object Tracker...")
    
    tracker = ObjectTracker(max_distance=5.0, max_age=10, min_hits=3)
    
    # Create a scenario with 2 objects moving at different speeds
    dt = 0.05
    time_steps = 100
    
    # Object 1: Moving right
    obj1_pos = []
    # Object 2: Moving right but faster
    obj2_pos = []
    
    for t in range(time_steps):
        # Object 1: slower
        obj1_pos.append([t * 5, 10.0])  # Moving at 5 m/s in x
        # Object 2: faster
        obj2_pos.append([t * 8, -5.0])  # Moving at 8 m/s in x, y = -5m
    
    # Simulate detection with some noise
    detection_sets = []
    for t in range(time_steps):
        detections = []
        
        # Add detections with probability 0.9
        if np.random.random() > 0.1:  # 90% detection rate
            # Object 1 with noise
            noise = np.random.normal(0, 0.5, size=2)
            detection1 = type('Detection', (), {})()
            detection1.dRel = obj1_pos[t][0] + noise[0]
            detection1.yRel = obj1_pos[t][1] + noise[1]
            detection1.prob = 0.8
            detections.append(detection1)
        
        if np.random.random() > 0.1:  # 90% detection rate  
            # Object 2 with noise
            noise = np.random.normal(0, 0.5, size=2)
            detection2 = type('Detection', (), {})()
            detection2.dRel = obj2_pos[t][0] + noise[0]
            detection2.yRel = obj2_pos[t][1] + noise[1]
            detection2.prob = 0.8
            detections.append(detection2)
        
        detection_sets.append(detections)
    
    # Run tracking
    track_histories = []
    for t, detections in enumerate(detection_sets):
        current_time = t * dt
        confirmed_tracks = tracker.update(detections, current_time)
        track_histories.append(confirmed_tracks.copy())
    
    # Check if tracking was stable
    avg_tracks = np.mean([len(tracks) for tracks in track_histories[20:]])  # After initial settling
    
    print(f"  Average number of confirmed tracks (steady state): {avg_tracks:.1f}")
    
    # Should have ~2 tracks most of the time after initialization
    success = avg_tracks > 1.5  # At least 1.5 on average means mostly tracking both
    print(f"  Object tracker test: {'✓ PASS' if success else '✗ FAIL'}")
    
    return success


def test_enhanced_fusion_integration():
    """Test the integration of Kalman filter with EnhancedCameraFusion"""
    print("\nTesting Enhanced Fusion Integration...")
    
    fusion = EnhancedCameraFusion()
    
    # Create simulated model outputs with leads
    def create_mock_leads(dRel, yRel, prob):
        """Create mock lead objects"""
        lead = type('Lead', (), {})()
        lead.dRel = dRel
        lead.yRel = yRel
        lead.vRel = 0.0
        lead.prob = prob
        return [lead]
    
    # Simulate multiple frames of data
    success_count = 0
    total_tests = 50
    
    for i in range(total_tests):
        # Create mock outputs simulating different scenarios
        road_output = {
            'leads_v3': create_mock_leads(20.0 + i*0.1, 1.0, 0.8),
            'lane_lines': [],
            'plan': np.random.random((32, 13))  # Mock plan data
        }
        
        wide_output = {
            'leads_v3': create_mock_leads(20.2 + i*0.1, 0.8, 0.7),  # Slightly different detection
            'lane_lines': [],
            'plan': np.random.random((32, 13))  # Mock plan data
        }
        
        try:
            # Test the enhanced fusion method
            fusion_output = fusion._enhance_object_tracking(road_output, wide_output)
            
            # Check if tracking info was added
            has_tracking = 'tracked_objects' in road_output
            has_quality = 'tracking_quality' in road_output
            has_confidence = 'track_confidence_avg' in road_output
            
            if has_tracking and has_quality and has_confidence:
                success_count += 1
        except Exception as e:
            print(f"    Error in fusion test {i}: {e}")
    
    success_rate = success_count / total_tests
    print(f"  Fusion integration success rate: {success_rate:.1%}")
    
    success = success_rate > 0.95  # 95% success rate
    print(f"  Integration test: {'✓ PASS' if success else '✗ FAIL'}")
    
    return success


def test_temporal_consistency():
    """Test the improved temporal consistency calculation"""
    print("\nTesting Temporal Consistency...")
    
    fusion = EnhancedCameraFusion()
    
    # Test with tracking data
    test_output_with_tracking = {
        'plan': np.ones((32, 13)) * 0.5,
        'lane_lines': [],  # Mock lane lines
        'track_count': 2,
        'track_confidence_avg': 0.85,
        'track_confidence_max': 0.95
    }
    
    # Add mock lane lines with points for consistency calculation
    for i in range(4):
        mock_line = type('LaneLine', (), {})()
        mock_line.points = list(np.linspace(0, 10, 10))  # 10 points
        mock_line.prob = 0.9
        test_output_with_tracking['lane_lines'].append(mock_line)
    
    # Test consistency without previous data
    consistency1 = fusion._calculate_temporal_consistency(test_output_with_tracking)
    print(f"  Consistency with tracking: {consistency1:.3f}")
    
    # Add to buffer to test comparison logic
    fusion.temporal_consistency_buffer.append(test_output_with_tracking)
    
    # Create slightly different output
    test_output_different = test_output_with_tracking.copy()
    test_output_different['plan'] = np.ones((32, 13)) * 0.55  # Slightly different
    
    consistency2 = fusion._calculate_temporal_consistency(test_output_different)
    print(f"  Consistency with comparison: {consistency2:.3f}")
    
    # Both should be reasonable values
    success = 0.0 < consistency1 <= 1.0 and 0.0 < consistency2 <= 1.0
    print(f"  Temporal consistency test: {'✓ PASS' if success else '✗ FAIL'}")
    
    return success


def run_all_tests():
    """Run all validation tests"""
    print("Running Kalman Filter and Fusion Validation Tests")
    print("=" * 55)
    
    tests = [
        test_kalman_filter_accuracy,
        test_object_tracker, 
        test_enhanced_fusion_integration,
        test_temporal_consistency
    ]
    
    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"  Test failed with exception: {e}")
            results.append(False)
    
    print(f"\nTest Results: {sum(results)}/{len(results)} passed")
    
    if all(results):
        print("🎉 All validation tests passed!")
        return True
    else:
        print("❌ Some validation tests failed.")
        return False


if __name__ == "__main__":
    success = run_all_tests()
    exit(0 if success else 1)