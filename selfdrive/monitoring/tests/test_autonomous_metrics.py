"""
Unit tests for Autonomous Metrics Collector module
"""
import time
import pytest
import numpy as np
from unittest.mock import Mock, patch

from selfdrive.monitoring.autonomous_metrics import AutonomousMetricsCollector


class TestAutonomousMetricsCollector:
    """Test suite for AutonomousMetricsCollector class"""

    def setup_method(self):
        """Setup test fixtures before each test method"""
        self.metrics_collector = AutonomousMetricsCollector()

    def test_initialization(self):
        """Test initialization of AutonomousMetricsCollector"""
        assert self.metrics_collector.frame_count == 0
        assert len(self.metrics_collector.lateral_jerk_buffer) == 0
        assert len(self.metrics_collector.longitudinal_jerk_buffer) == 0
        assert len(self.metrics_collector.steering_angles) == 0
        assert len(self.metrics_collector.accelerations) == 0
        assert len(self.metrics_collector.velocities) == 0
        
        # Check initial state
        assert self.metrics_collector.fcw_events == 0
        assert self.metrics_collector.driver_interventions == 0
        assert self.metrics_collector.mode_switches == 0
        assert not self.metrics_collector.initialized

    def test_initialize_submaster(self):
        """Test SubMaster initialization"""
        mock_sm = Mock()
        self.metrics_collector.initialize_submaster(mock_sm)
        
        assert self.metrics_collector.initialized
        assert self.metrics_collector.sm == mock_sm

    def test_collect_metrics_before_initialization(self):
        """Test collect_metrics behavior before initialization"""
        # Should return self without errors
        collector = self.metrics_collector.collect_metrics()
        assert collector == self.metrics_collector
        # Should have logged a warning (we can't easily test this without patching)

    def test_collect_metrics_with_mock_submaster(self):
        """Test metrics collection with mocked SubMaster data"""
        # Create a mock SubMaster with realistic data
        mock_sm = Mock()
        mock_sm.updated = {
            'carState': True,
            'controlsState': True,
            'deviceState': True,
            'radarState': True,
            'modelV2': True
        }
        
        # Mock carState
        mock_car_state = Mock()
        mock_car_state.steeringAngleDeg = 2.5
        mock_car_state.aEgo = 15.0
        mock_car_state.vEgo = 25.0
        mock_car_state.steeringPressed = False
        mock_car_state.steeringRateDeg = 0.5
        mock_sm.__getitem__ = Mock(side_effect=lambda key: {
            'carState': mock_car_state,
            'controlsState': Mock(
                lateralJerk=0.2,
                longitudinalJerk=0.1,
                experimentalMode=True,
                lateralControlState=Mock(error=0.01)
            )
        }[key] if key in ['carState', 'controlsState'] else (
            Mock(cpuUsagePercent=[25.0, 30.0, 20.0], 
                 memoryUsagePercent=40.0, 
                 cpuTempC=[65.0, 60.0, 62.0],
                 gpuTempC=[])
            if key == 'deviceState' else
            Mock(fcw=False, leadOne=Mock(status=False))
            if key == 'radarState' else
            Mock(position=Mock(x=[]), path=Mock(y=[]))
        ))
        
        self.metrics_collector.initialize_submaster(mock_sm)
        
        # Collect metrics once
        collector = self.metrics_collector.collect_metrics()
        
        # Check that metrics were collected
        assert collector.frame_count == 1
        assert len(collector.steering_angles) == 1
        assert collector.steering_angles[0] == 2.5
        
        # Collect metrics again to build up data
        for _ in range(5):
            self.metrics_collector.collect_metrics()
        
        assert len(self.metrics_collector.steering_angles) <= 6  # Limited by buffer size
        assert len(self.metrics_collector.accelerations) <= 6

    def test_collect_metrics_error_handling(self):
        """Test that metrics collection continues despite errors in specific message types"""
        mock_sm = Mock()
        mock_sm.updated = {'carState': True}
        mock_sm.__getitem__ = Mock(side_effect=AttributeError("No such attribute"))
        
        self.metrics_collector.initialize_submaster(mock_sm)
        
        # This should not raise an exception despite the error in __getitem__
        collector = self.metrics_collector.collect_metrics()
        assert collector.frame_count == 1

    def test_get_performance_report(self):
        """Test performance report generation"""
        # Add some test data to buffers
        for i in range(10):
            self.metrics_collector.lateral_jerk_buffer.append(0.1 * i)
            self.metrics_collector.longitudinal_jerk_buffer.append(0.05 * i)
            self.metrics_collector.steering_angles.append(1.0 * i)
            self.metrics_collector.accelerations.append(2.0 * i)
            self.metrics_collector.velocities.append(10.0 + i)
        
        report = self.metrics_collector.get_performance_report()
        
        # Verify report structure and content
        assert 'avg_lateral_jerk' in report
        assert 'avg_longitudinal_jerk' in report
        assert 'avg_steering_angle' in report
        assert 'avg_acceleration' in report
        assert 'avg_velocity' in report
        
        # Verify calculated values make sense
        assert report['avg_lateral_jerk'] >= 0
        assert report['avg_longitudinal_jerk'] >= 0
        assert report['avg_steering_angle'] >= 0
        assert report['avg_acceleration'] >= 0
        assert report['avg_velocity'] >= 0

    def test_get_system_health(self):
        """Test system health assessment"""
        # Add some test data
        for i in range(10):
            self.metrics_collector.cpu_usage_buffer.append(20.0 + i)
            self.metrics_collector.memory_usage_buffer.append(40.0 + i)
            self.metrics_collector.temperature_buffer.append(60.0 + i)
        
        health = self.metrics_collector.get_system_health()
        
        assert 'status' in health
        assert 'avg_cpu_util' in health
        assert 'avg_memory_util' in health
        assert 'avg_temperature' in health
        assert 'cpu_peaks' in health
        assert 'thermal_issue_count' in health
        assert 'lateral_smoothness' in health
        assert 'longitudinal_smoothness' in health
        
        assert health['status'] in ['healthy', 'warning', 'critical']

    def test_get_baseline_performance(self):
        """Test baseline performance establishment"""
        # Add some data to the buffers
        for i in range(50):
            self.metrics_collector.lateral_jerk_buffer.append(0.5 + 0.1 * (i % 5))
            self.metrics_collector.longitudinal_jerk_buffer.append(0.3 + 0.05 * (i % 5))
            self.metrics_collector.cpu_usage_buffer.append(30.0 + 2.0 * (i % 3))
            self.metrics_collector.temperature_buffer.append(65.0 + 1.0 * (i % 4))
        
        baseline = self.metrics_collector.get_baseline_performance()
        
        assert 'avg_lateral_jerk' in baseline
        assert 'avg_longitudinal_jerk' in baseline
        assert 'cpu_utilization' in baseline
        assert 'system_stability' in baseline
        
        # Test that baseline values are reasonable
        assert baseline['avg_lateral_jerk'] > 0
        assert baseline['avg_longitudinal_jerk'] > 0
        assert 0 <= baseline['cpu_utilization'] <= 100
        assert 0 <= baseline['system_stability'] <= 1

    def test_compare_with_baseline(self):
        """Test comparison with baseline performance"""
        # Set up some baseline data
        baseline_data = {
            'avg_lateral_jerk': 1.0,
            'avg_longitudinal_jerk': 0.8,
            'cpu_utilization': 50.0,
            'system_stability': 0.7
        }
        
        # Add some current performance data
        for i in range(20):
            self.metrics_collector.lateral_jerk_buffer.append(0.8 + 0.01 * i)
            self.metrics_collector.longitudinal_jerk_buffer.append(0.6 + 0.01 * i)
            self.metrics_collector.cpu_usage_buffer.append(45.0 + 0.1 * i)
            self.metrics_collector.temperature_buffer.append(60.0 + 0.05 * i)
        
        comparison = self.metrics_collector.compare_with_baseline(baseline_data)
        
        assert 'lateral_jerk_improvement' in comparison
        assert 'longitudinal_jerk_improvement' in comparison
        assert 'cpu_efficiency_improvement' in comparison
        assert 'stability_improvement' in comparison
        assert 'overall_improvement_score' in comparison

    def test_buffer_size_constraints(self):
        """Test that buffers maintain correct maximum sizes"""
        # Add more data than buffer capacity
        for i in range(300):  # Much more than buffer size of 100-200
            self.metrics_collector.lateral_jerk_buffer.append(0.1 * i)
            self.metrics_collector.longitudinal_jerk_buffer.append(0.05 * i)
            self.metrics_collector.steering_angles.append(1.0 * i)
            self.metrics_collector.accelerations.append(2.0 * i)
            self.metrics_collector.velocities.append(10.0 * i)
        
        # Verify buffers are still within expected size limits
        assert len(self.metrics_collector.lateral_jerk_buffer) == 200  # Max size
        assert len(self.metrics_collector.longitudinal_jerk_buffer) == 200  # Max size
        assert len(self.metrics_collector.steering_angles) == 200  # Max size
        assert len(self.metrics_collector.accelerations) == 200  # Max size
        assert len(self.metrics_collector.velocities) == 200  # Max size


if __name__ == "__main__":
    pytest.main([__file__])