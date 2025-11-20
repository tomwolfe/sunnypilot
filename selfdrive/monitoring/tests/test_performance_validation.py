"""
Unit tests for performance impact validation
These tests verify that monitoring overhead is limited to ≤20ms per operation
"""
import time
import pytest
import numpy as np
from unittest.mock import Mock, patch
import threading
import queue

from selfdrive.monitoring.driving_monitor import AutonomousDrivingMonitor
from selfdrive.monitoring.autonomous_metrics import AutonomousMetricsCollector


class TestPerformanceImpactValidation:
    """Test suite for performance validation of monitoring system"""

    def setup_method(self):
        """Setup test fixtures before each test method"""
        self.monitor = AutonomousDrivingMonitor()
        self.metrics_collector = AutonomousMetricsCollector()

    def test_monitoring_overhead_under_normal_conditions(self):
        """Test that monitoring overhead is ≤20ms under normal system load"""
        # Mock the SubMaster with realistic data
        mock_sm = Mock()
        mock_sm.update = Mock(return_value=None)
        mock_sm.updated = {
            'carState': True,
            'modelV2': True,
            'controlsState': True,
            'deviceState': True,
            'radarState': True
        }
        
        # Create realistic mock data for each service
        mock_car_state = Mock()
        mock_car_state.vEgo = 15.0
        mock_car_state.aEgo = 0.5
        mock_car_state.steeringAngleDeg = 2.0
        mock_car_state.steeringRateDeg = 1.5
        mock_car_state.steeringPressed = False
        mock_car_state.wheelSpeeds = Mock()
        mock_car_state.wheelSpeeds.fl = 15.0
        mock_car_state.wheelSpeeds.fr = 15.1
        mock_car_state.wheelSpeeds.rl = 14.9
        mock_car_state.wheelSpeeds.rr = 15.0
        
        mock_model_v2 = Mock()
        mock_model_v2.position = Mock()
        mock_model_v2.position.x = [float(i) for i in range(33)]  # TRAJECTORY_SIZE
        mock_model_v2.orientation = Mock()
        mock_model_v2.orientation.x = [0.0] * 33
        mock_model_v2.orientation.y = [0.0] * 33
        mock_model_v2.path = Mock()
        mock_model_v2.path.y = [0.001] * 33
        mock_model_v2.meta = Mock()
        mock_model_v2.meta.stopState = 0.1
        mock_model_v2.meta.modelExecutionTime = 0.02  # 20ms execution time
        
        mock_controls_state = Mock()
        mock_controls_state.lateralJerk = 0.8
        mock_controls_state.longitudinalJerk = 0.5
        mock_controls_state.experimentalMode = True
        
        mock_device_state = Mock()
        mock_device_state.cpuUsagePercent = [45.0, 42.0, 48.0, 44.0]
        mock_device_state.memoryUsagePercent = 65.0
        mock_device_state.cpuTempC = [65.0, 66.0, 64.0, 67.0]
        mock_device_state.gpuTempC = [55.0, 56.0, 54.0, 57.0]
        mock_device_state.lightSensor = 150.0
        
        mock_radar_state = Mock()
        mock_radar_state.fcw = False
        mock_radar_state.leadOne = Mock()
        mock_radar_state.leadOne.status = True
        mock_radar_state.leadOne.vRel = -2.0
        mock_radar_state.leadOne.dRel = 50.0
        
        def mock_sm_getitem(key):
            mapping = {
                'carState': mock_car_state,
                'modelV2': mock_model_v2,
                'controlsState': mock_controls_state,
                'deviceState': mock_device_state,
                'radarState': mock_radar_state
            }
            return mapping.get(key, Mock())
        
        mock_sm.__getitem__ = mock_sm_getitem
        
        # Initialize the metrics collector with the mock
        self.metrics_collector.initialize_submaster(mock_sm)
        
        # Measure the time taken for metrics collection
        start_time = time.perf_counter()
        collected_metrics = self.metrics_collector.collect_metrics()
        end_time = time.perf_counter()
        
        execution_time_ms = (end_time - start_time) * 1000  # Convert to milliseconds
        
        # The critical requirement: monitoring overhead should be ≤20ms per operation
        assert execution_time_ms <= 20.0, f"Metrics collection took {execution_time_ms:.2f}ms, exceeding 20ms limit"
        
        print(f"Monitoring overhead: {execution_time_ms:.2f}ms (≤20ms requirement: {'PASS' if execution_time_ms <= 20.0 else 'FAIL'})")

    def test_monitoring_overhead_under_high_load_conditions(self):
        """Test monitoring overhead under high system load conditions"""
        # Create mock data that simulates high system load conditions
        mock_sm = Mock()
        mock_sm.update = Mock(return_value=None)
        mock_sm.updated = {
            'carState': True,
            'modelV2': True,
            'controlsState': True,
            'deviceState': True,
            'radarState': True
        }
        
        # Simulate high load conditions
        mock_car_state = Mock()
        mock_car_state.vEgo = 25.0
        mock_car_state.aEgo = 1.2
        mock_car_state.steeringAngleDeg = 8.0  # Higher steering angle
        mock_car_state.steeringRateDeg = 8.0  # Higher steering rate
        mock_car_state.steeringPressed = True  # More driver interaction
        mock_car_state.wheelSpeeds = Mock()
        mock_car_state.wheelSpeeds.fl = 24.5
        mock_car_state.wheelSpeeds.fr = 25.5
        mock_car_state.wheelSpeeds.rl = 24.0
        mock_car_state.wheelSpeeds.rr = 26.0  # Higher variance
        
        # Simulate complex model data that might take longer to process
        mock_model_v2 = Mock()
        mock_model_v2.position = Mock()
        mock_model_v2.position.x = [float(i) + np.random.normal(0, 0.1) for i in range(33)]  # Add noise
        mock_model_v2.orientation = Mock()
        mock_model_v2.orientation.x = [np.random.normal(0, 0.01) for _ in range(33)]
        mock_model_v2.orientation.y = [np.random.normal(0, 0.01) for _ in range(33)]
        mock_model_v2.path = Mock()
        mock_model_v2.path.y = [np.random.normal(0.001, 0.002) for _ in range(33)]
        mock_model_v2.meta = Mock()
        mock_model_v2.meta.stopState = 0.7  # Higher stop probability
        mock_model_v2.meta.modelExecutionTime = 0.04  # Higher execution time
        
        mock_controls_state = Mock()
        mock_controls_state.lateralJerk = 2.5  # Higher jerk values
        mock_controls_state.longitudinalJerk = 1.8
        mock_controls_state.experimentalMode = True
        
        # High system load indicators
        mock_device_state = Mock()
        mock_device_state.cpuUsagePercent = [85.0, 88.0, 90.0, 87.0]  # High CPU usage
        mock_device_state.memoryUsagePercent = 85.0  # High memory usage
        mock_device_state.cpuTempC = [80.0, 82.0, 85.0, 83.0]  # High temperature
        mock_device_state.gpuTempC = [75.0, 78.0, 80.0, 77.0]
        mock_device_state.lightSensor = 30.0  # Low light
        
        mock_radar_state = Mock()
        mock_radar_state.fcw = True  # FCW triggered
        mock_radar_state.leadOne = Mock()
        mock_radar_state.leadOne.status = True
        mock_radar_state.leadOne.vRel = 1.0  # Close lead
        mock_radar_state.leadOne.dRel = 15.0  # Short distance
        
        def mock_sm_getitem(key):
            mapping = {
                'carState': mock_car_state,
                'modelV2': mock_model_v2,
                'controlsState': mock_controls_state,
                'deviceState': mock_device_state,
                'radarState': mock_radar_state
            }
            return mapping.get(key, Mock())
        
        mock_sm.__getitem__ = mock_sm_getitem
        
        # Initialize the metrics collector with the mock
        self.metrics_collector.initialize_submaster(mock_sm)
        
        # Measure the time taken for metrics collection under high load
        start_time = time.perf_counter()
        collected_metrics = self.metrics_collector.collect_metrics()
        end_time = time.perf_counter()
        
        execution_time_ms = (end_time - start_time) * 1000  # Convert to milliseconds
        
        # Even under high load, monitoring overhead should be ≤20ms
        assert execution_time_ms <= 20.0, f"High load metrics collection took {execution_time_ms:.2f}ms, exceeding 20ms limit"
        
        print(f"High load monitoring overhead: {execution_time_ms:.2f}ms (≤20ms requirement: {'PASS' if execution_time_ms <= 20.0 else 'FAIL'})")

    def test_continuous_monitoring_performance(self):
        """Test performance over extended monitoring periods"""
        # Test that the monitoring system maintains performance over time without memory leaks
        mock_sm = Mock()
        mock_sm.update = Mock(return_value=None)
        mock_sm.updated = {
            'carState': True,
            'modelV2': True,
            'controlsState': True,
            'deviceState': True,
            'radarState': True
        }
        
        # Create simple mock data
        mock_car_state = Mock()
        mock_car_state.vEgo = 10.0
        mock_car_state.aEgo = 0.2
        mock_car_state.steeringAngleDeg = 1.0
        mock_car_state.steeringRateDeg = 0.5
        mock_car_state.steeringPressed = False
        
        mock_model_v2 = Mock()
        mock_model_v2.position = Mock()
        mock_model_v2.position.x = [float(i) for i in range(33)]
        mock_model_v2.orientation = Mock()
        mock_model_v2.orientation.x = [0.0] * 33
        mock_model_v2.path = Mock()
        mock_model_v2.path.y = [0.001] * 33
        mock_model_v2.meta = Mock()
        mock_model_v2.meta.stopState = 0.1
        
        mock_controls_state = Mock()
        mock_controls_state.lateralJerk = 0.5
        mock_controls_state.longitudinalJerk = 0.3
        mock_controls_state.experimentalMode = True
        
        mock_device_state = Mock()
        mock_device_state.cpuUsagePercent = [50.0, 48.0, 52.0, 49.0]
        mock_device_state.memoryUsagePercent = 70.0
        mock_device_state.cpuTempC = [68.0, 69.0, 67.0, 70.0]
        
        mock_radar_state = Mock()
        mock_radar_state.fcw = False
        mock_radar_state.leadOne = Mock()
        mock_radar_state.leadOne.status = True
        
        def mock_sm_getitem(key):
            mapping = {
                'carState': mock_car_state,
                'modelV2': mock_model_v2,
                'controlsState': mock_controls_state,
                'deviceState': mock_device_state,
                'radarState': mock_radar_state
            }
            return mapping.get(key, Mock())
        
        mock_sm.__getitem__ = mock_sm_getitem
        
        # Initialize the metrics collector
        self.metrics_collector.initialize_submaster(mock_sm)
        
        # Run multiple iterations to check for performance degradation
        execution_times = []
        iterations = 50  # Test over 50 iterations
        
        for i in range(iterations):
            start_time = time.perf_counter()
            collected_metrics = self.metrics_collector.collect_metrics()
            end_time = time.perf_counter()
            
            execution_time_ms = (end_time - start_time) * 1000
            execution_times.append(execution_time_ms)
        
        # Calculate statistics
        avg_time = np.mean(execution_times)
        max_time = np.max(execution_times)
        min_time = np.min(execution_times)
        std_time = np.std(execution_times)
        
        # Verify performance requirements
        assert max_time <= 20.0, f"Maximum execution time {max_time:.2f}ms exceeds 20ms limit"
        assert avg_time <= 20.0, f"Average execution time {avg_time:.2f}ms exceeds 20ms limit"
        
        print(f"Continuous monitoring performance over {iterations} iterations:")
        print(f"  Avg: {avg_time:.2f}ms, Max: {max_time:.2f}ms, Min: {min_time:.2f}ms, Std: {std_time:.2f}ms")
        print(f"  All measurements ≤20ms: {'PASS' if max_time <= 20.0 else 'FAIL'}")

    def test_memory_usage_stability(self):
        """Test that memory usage remains stable during extended operation"""
        # This test should check for memory leaks during extended operation
        mock_sm = Mock()
        mock_sm.update = Mock(return_value=None)
        mock_sm.updated = {'carState': True}
        
        # Create simple mock data
        mock_car_state = Mock()
        mock_car_state.vEgo = 15.0
        mock_car_state.aEgo = 0.0
        mock_car_state.steeringAngleDeg = 0.0
        mock_car_state.steeringPressed = False
        
        mock_sm.__getitem__ = Mock(return_value=mock_car_state)
        
        # Initialize the metrics collector
        self.metrics_collector.initialize_submaster(mock_sm)
        
        # Get initial memory-related buffer sizes
        initial_cpu_buffer_size = len(self.metrics_collector.cpu_usage_buffer) if hasattr(self.metrics_collector, 'cpu_usage_buffer') else 0
        initial_mem_buffer_size = len(self.metrics_collector.memory_usage_buffer) if hasattr(self.metrics_collector, 'memory_usage_buffer') else 0
        initial_temp_buffer_size = len(self.metrics_collector.temperature_buffer) if hasattr(self.metrics_collector, 'temperature_buffer') else 0
        
        # Run multiple collection cycles
        for i in range(100):  # More iterations to test buffer management
            self.metrics_collector.collect_metrics()
        
        # Check that buffer sizes are stable (not growing indefinitely)
        final_cpu_buffer_size = len(self.metrics_collector.cpu_usage_buffer) if hasattr(self.metrics_collector, 'cpu_usage_buffer') else 0
        final_mem_buffer_size = len(self.metrics_collector.memory_usage_buffer) if hasattr(self.metrics_collector, 'memory_usage_buffer') else 0
        final_temp_buffer_size = len(self.metrics_collector.temperature_buffer) if hasattr(self.metrics_collector, 'temperature_buffer') else 0
        
        # Deque with max length should keep size stable
        assert final_cpu_buffer_size <= 100, f"CPU buffer grew to {final_cpu_buffer_size}, should be capped"
        assert final_mem_buffer_size <= 100, f"Memory buffer grew to {final_mem_buffer_size}, should be capped"  
        assert final_temp_buffer_size <= 100, f"Temperature buffer grew to {final_temp_buffer_size}, should be capped"

    def test_performance_under_error_conditions(self):
        """Test performance when metrics collection encounters errors"""
        # Create a mock that will cause some errors during collection
        mock_sm = Mock()
        mock_sm.updated = {
            'carState': True,
            'modelV2': True,
            'controlsState': True,
            'deviceState': True,
            'radarState': True
        }
        
        # Create mock that will have some attributes missing to trigger error handling
        mock_car_state = Mock()
        mock_car_state.vEgo = 15.0
        # Deliberately not setting some attributes that the collector looks for
        
        # Mock model with missing attributes
        mock_model_v2 = Mock()
        mock_model_v2.position = Mock()
        mock_model_v2.position.x = []  # Empty array to trigger condition check
        mock_model_v2.path = Mock()
        mock_model_v2.path.y = []  # Empty array
        
        mock_controls_state = Mock()
        # Missing lateralJerk and longitudinalJerk to trigger error handling
        
        mock_device_state = Mock()
        # Missing some temperature attributes
        
        mock_radar_state = Mock()
        # Missing FCW attribute
        
        def mock_sm_getitem(key):
            mapping = {
                'carState': mock_car_state,
                'modelV2': mock_model_v2,
                'controlsState': mock_controls_state,
                'deviceState': mock_device_state,
                'radarState': mock_radar_state
            }
            return mapping.get(key, Mock())
        
        mock_sm.__getitem__ = mock_sm_getitem
        
        # Initialize the metrics collector with the mock
        self.metrics_collector.initialize_submaster(mock_sm)
        
        # Even with missing attributes (error conditions), collection should be fast
        start_time = time.perf_counter()
        collected_metrics = self.metrics_collector.collect_metrics()
        end_time = time.perf_counter()
        
        execution_time_ms = (end_time - start_time) * 1000
        
        # Should still be fast even when handling errors
        assert execution_time_ms <= 20.0, f"Error condition metrics collection took {execution_time_ms:.2f}ms, exceeding 20ms limit"
        
        print(f"Error condition monitoring overhead: {execution_time_ms:.2f}ms")


if __name__ == "__main__":
    pytest.main([__file__])