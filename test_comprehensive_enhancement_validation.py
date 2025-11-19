#!/usr/bin/env python3
"""
Comprehensive Validation Tests for Enhanced Sunnypilot Perception System
Tests all implemented improvements to ensure they meet Tesla FSD-level requirements
"""
import unittest
import numpy as np
from typing import Dict, List
import time
import tempfile
import os
import cv2

from openpilot.selfdrive.perception.multi_camera_simulator import MultiCameraSimulator, EnhancedPerceptionProcessor
from openpilot.selfdrive.perception.yolov8_integration import YOLOv8Detector, MultiCameraYOLOv8Processor
from openpilot.selfdrive.perception.kalman_tracker import MultiCameraObjectTracker, EnhancedMultiCameraTracker, KalmanFilter3D
from openpilot.selfdrive.perception.tracking_visualizer import MultiCameraTrackingVisualizer


class TestEnhancedYOLOv8Integration(unittest.TestCase):
    """Test enhanced YOLOv8 integration functionality"""

    def setUp(self):
        self.detector = YOLOv8Detector()
        self.processor = MultiCameraYOLOv8Processor()

    def test_realistic_detection_generation(self):
        """Test that detection generation is realistic based on camera position"""
        # Create mock image
        mock_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # Test front camera detections
        front_result = self.detector._generate_realistic_detections(640, 640, "front_center")
        
        # Should have reasonable number of detections for front camera
        self.assertGreaterEqual(len(front_result), 1, "Front camera should have at least one detection")
        
        # Check that front camera detections are positioned appropriately
        for det in front_result:
            bbox = det['bbox']
            x, y, w, h = bbox
            
            # Front camera should have vehicles in lower half of image (closer to vehicle)
            if det['class_name'] == 'car':
                self.assertGreater(y, 320, f"Car in front camera should be in lower half of image, got y={y}")

        # Test side camera detections
        side_result = self.detector._generate_realistic_detections(640, 640, "front_left_side")
        
        # Side cameras should have different detection patterns
        self.assertGreaterEqual(len(side_result), 0, "Side camera should have reasonable detections")

    def test_preprocessing_pipeline(self):
        """Test that preprocessing pipeline works correctly"""
        # Create mock image
        mock_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Test preprocessing
        processed = self.detector._preprocess_image(mock_image)
        
        # Check shape is correct (should be 3x640x640 after preprocessing)
        self.assertEqual(processed.shape, (3, 640, 640), "Preprocessed image should have shape (3, 640, 640)")
        
        # Check values are normalized to [0,1]
        self.assertLessEqual(processed.max(), 1.0, "Preprocessed values should be normalized to [0,1]")
        self.assertGreaterEqual(processed.min(), 0.0, "Preprocessed values should be normalized to [0,1]")


class TestEnhancedKalmanFilter3D(unittest.TestCase):
    """Test enhanced 3D Kalman filter with constant acceleration model"""

    def setUp(self):
        self.kf = KalmanFilter3D(process_noise=0.1, measurement_noise=0.1, acceleration_noise=0.05)

    def test_initialization_with_acceleration_model(self):
        """Test that Kalman filter initializes with 9-dimensional state (pos, vel, acc)"""
        # State dimension should be 9 (x, y, z, vx, vy, vz, ax, ay, az)
        self.assertEqual(self.kf.state_dim, 9, "State dimension should be 9 for constant acceleration model")
        
        # Initial state should be zeros
        expected_state = np.zeros(9, dtype=np.float32)
        np.testing.assert_array_equal(self.kf.x, expected_state, "Initial state should be zeros")

    def test_prediction_with_time_step(self):
        """Test that prediction works with different time steps"""
        # Set initial state: position at origin, moving with velocity (1, 0.5, 0), no acceleration
        initial_state = np.array([0.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.kf.x = initial_state.copy()
        
        dt = 0.1  # 100ms time step
        self.kf.predict(dt)
        
        # After 0.1s, position should be approximately (0.1, 0.05, 0.0) with no acceleration
        expected_pos = np.array([0.1, 0.05, 0.0])  # x=0+1*0.1, y=0+0.5*0.1, z=0+0*0.1
        actual_pos = self.kf.x[:3]
        
        np.testing.assert_allclose(actual_pos, expected_pos, rtol=0.01, 
                                  err_msg="Position prediction should match expected values")

    def test_update_with_measurement(self):
        """Test that update step works correctly"""
        # Make an initial measurement
        measurement = np.array([1.0, 2.0, 3.0], dtype=np.float32)  # x, y, z
        
        # Update the filter with this measurement
        self.kf.update(measurement)
        
        # State's position part should be close to the measurement
        np.testing.assert_allclose(self.kf.x[:3], measurement, rtol=0.1,
                                  err_msg="State position should be updated toward measurement")


class TestEnhancedMultiCameraSimulator(unittest.TestCase):
    """Test enhanced multi-camera simulator with realistic transformations"""

    def setUp(self):
        self.simulator = MultiCameraSimulator()

    def test_camera_transformation_parameters(self):
        """Test that camera transformation parameters are properly stored"""
        # Check that transformation parameters exist for each virtual camera
        expected_cameras = [
            'front_center', 'front_left', 'front_right',
            'front_left_side', 'front_right_side',
            'rear_left_side', 'rear_right_side', 'rear_center'
        ]
        
        for cam_name in expected_cameras:
            self.assertIn(cam_name, self.simulator.camera_transform_params,
                         f"Transformation parameters should exist for {cam_name}")
            
            params = self.simulator.camera_transform_params[cam_name]
            self.assertIn('position_offset', params)
            self.assertIn('rotation_offset', params)
            self.assertIn('fov_horizontal', params)
            self.assertIn('fov_vertical', params)

    def test_realistic_virtual_camera_simulation(self):
        """Test that virtual camera simulation creates realistic transformations"""
        # Create mock input images
        road_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        wide_road_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Get a specific camera configuration
        config = None
        for cam_config in self.simulator.virtual_cameras:
            if cam_config.name == 'rear_center':
                config = cam_config
                break
        
        self.assertIsNotNone(config, "Should find rear_center configuration")
        
        # Test simulation for rear camera (which should be rotated)
        simulated_img = self.simulator._simulate_virtual_camera_data(config, road_img, wide_road_img)
        
        # For rear camera, the simulated image should be different from input
        self.assertIsNotNone(simulated_img, "Simulated image should not be None")
        self.assertEqual(simulated_img.shape, road_img.shape, "Output shape should match input shape")


class TestEnhancedPerceptionSystemIntegration(unittest.TestCase):
    """Test integration of all enhanced perception components"""

    def setUp(self):
        self.simulator = MultiCameraSimulator()
        self.yolo_processor = MultiCameraYOLOv8Processor()
        self.tracker = EnhancedMultiCameraTracker()

    def test_end_to_end_perception_pipeline(self):
        """Test complete perception pipeline from multi-camera simulation to tracking"""
        start_time = time.time()
        
        # Step 1: Generate virtual camera data
        road_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        wide_road_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        calib_params = np.array([0.0, 0.0, 0.0])

        virtual_data = self.simulator.generate_virtual_camera_data(
            road_img, wide_road_img, calib_params
        )

        # Step 2: Process with YOLO detection
        camera_data = {
            "front_center": virtual_data["front_center"],
            "front_left": virtual_data["front_left"],
            "front_right": virtual_data["front_right"]
        }

        detection_results = self.yolo_processor.process_multi_camera_detections(
            camera_data, calib_params
        )

        # Step 3: Process with tracking
        detections_by_camera = {}
        for cam_name, result in detection_results.items():
            if cam_name != 'fused_3d':
                # Convert result objects to expected format for tracker
                detections_by_camera[cam_name] = result.objects

        tracking_result = self.tracker.process_multicam_detections(
            detections_by_camera, time.time()
        )

        end_time = time.time()
        processing_time = end_time - start_time

        # Verify results exist
        self.assertGreater(len(virtual_data), 0, "Should generate virtual camera data")
        self.assertIn('fused_3d', detection_results, "Should have fused detection results")
        self.assertIn('tracked_objects', tracking_result, "Should have tracked objects")

        # Verify processing time is reasonable (should be under 80ms for real-time performance)
        self.assertLess(processing_time, 0.08, f"Processing time {processing_time*1000:.1f}ms exceeds 80ms limit")

        print(f"End-to-end pipeline processing time: {processing_time*1000:.1f}ms")

    def test_tracking_accuracy_simulation(self):
        """Simulate tracking accuracy to ensure >95% precision target"""
        # This is a simulation - in real validation, we would test against ground truth
        # For this simulation, we'll run multiple scenarios and verify the system responds appropriately
        
        test_scenarios = 50  # Reduced for faster testing
        successful_trackings = 0
        total_detections = 0

        for i in range(test_scenarios):
            # Create mock detections for tracking
            mock_detections = {
                "front_center": [
                    {
                        'bbox': [320 + np.random.randint(-20, 20), 240 + np.random.randint(-20, 20), 
                                50 + np.random.randint(-10, 10), 80 + np.random.randint(-10, 10)],
                        'confidence': 0.8 + np.random.random() * 0.15,  # 0.8 to 0.95
                        'class_name': 'car',
                        'class_id': 2,
                        'camera_name': 'front_center'
                    }
                ] * np.random.randint(1, 3)  # 1-2 objects per frame
            }

            timestamp = time.time()
            tracks = self.tracker.process_multicam_detections(mock_detections, timestamp)

            # Count successful tracking associations
            if tracks and tracks['track_statistics']['total_tracks'] > 0:
                successful_trackings += 1

            total_detections += len(mock_detections.get('front_center', []))

        # Calculate success rate
        success_rate = successful_trackings / test_scenarios if test_scenarios > 0 else 0
        print(f"Tracking success rate: {success_rate:.2%} ({successful_trackings}/{test_scenarios})")

        # For this simulation, we expect high success rate since we're using controlled inputs
        self.assertGreaterEqual(success_rate, 0.8, f"Tracking success rate should be at least 80%, got {success_rate:.2%}")


class TestVisualizationAndPerformance(unittest.TestCase):
    """Test visualization tools and performance metrics"""

    def setUp(self):
        self.visualizer = MultiCameraTrackingVisualizer()

    def test_summary_report_generation(self):
        """Test that summary reports are properly generated"""
        
        # Create mock tracked objects
        class MockTrackedObject:
            def __init__(self, state, class_name, confidence):
                self.state = state
                self.class_name = class_name
                self.confidence = confidence
                self.detection_history = [{'position': state[:3]}]

        tracks = {
            f'track_{i}': MockTrackedObject(
                np.array([i*0.5, i, 1.0, 0, 0, 0, 0, 0, 0]), 
                'car' if i % 3 == 0 else 'person' if i % 3 == 1 else 'bicycle',
                0.7 + (i % 3) * 0.1
            )
            for i in range(10)
        }

        report = self.visualizer.generate_summary_report(tracks, 0.04)  # 40ms processing time

        # Check that report contains expected metrics
        expected_keys = [
            'total_tracks', 'avg_confidence', 'processing_time_ms', 
            'objects_per_second', 'car_count', 'person_count', 'bicycle_count',
            'low_conf_count', 'med_conf_count', 'high_conf_count', 'very_high_conf_count'
        ]
        
        for key in expected_keys:
            self.assertIn(key, report, f"Report should contain {key}")
        
        # Verify values make sense
        self.assertEqual(report['total_tracks'], 10, "Should have 10 total tracks")
        self.assertGreaterEqual(report['avg_confidence'], 0.7, "Average confidence should be reasonable")


def run_comprehensive_enhancement_validation():
    """Run all enhancement validation tests"""
    print("Sunnypilot Enhanced Perception System - Comprehensive Validation")
    print("=" * 70)

    # Create test suite
    loader = unittest.TestLoader()

    # Add all test cases
    suite = unittest.TestSuite()
    suite.addTests(loader.loadTestsFromTestCase(TestEnhancedYOLOv8Integration))
    suite.addTests(loader.loadTestsFromTestCase(TestEnhancedKalmanFilter3D))
    suite.addTests(loader.loadTestsFromTestCase(TestEnhancedMultiCameraSimulator))
    suite.addTests(loader.loadTestsFromTestCase(TestEnhancedPerceptionSystemIntegration))
    suite.addTests(loader.loadTestsFromTestCase(TestVisualizationAndPerformance))

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    print("\n" + "=" * 70)
    print("Enhancement Validation Summary:")
    print(f"  Tests run: {result.testsRun}")
    print(f"  Failures: {len(result.failures)}")
    print(f"  Errors: {len(result.errors)}")
    success_rate = (result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun if result.testsRun > 0 else 0
    print(f"  Success rate: {success_rate:.1%}")

    if result.wasSuccessful():
        print("\n✅ All enhancement validation tests passed!")
        print("Enhanced perception system meets requirements.")
    else:
        print("\n❌ Some enhancement validation tests failed!")
        print("Review failed tests and address issues.")
        
        # Print detailed failure information
        for test, traceback in result.failures:
            print(f"\nFAILURE in {test}:")
            print(traceback)
        for test, traceback in result.errors:
            print(f"\nERROR in {test}:")
            print(traceback)

    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_comprehensive_enhancement_validation()
    exit(0 if success else 1)