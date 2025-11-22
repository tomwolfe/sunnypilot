"""
Unit tests for Driving Monitor module
"""
import time
import pytest
import numpy as np
from unittest.mock import Mock, patch

from selfdrive.monitoring.driving_monitor import AutonomousDrivingMonitor


class TestAutonomousDrivingMonitor:
    """Test suite for AutonomousDrivingMonitor class"""

    def setup_method(self):
        """Setup test fixtures before each test method"""
        self.driving_monitor = AutonomousDrivingMonitor()

    def test_initialization(self):
        """Test initialization of AutonomousDrivingMonitor"""
        assert hasattr(self.driving_monitor, 'frame_count')
        assert hasattr(self.driving_monitor, 'params')
        assert hasattr(self.driving_monitor, 'metrics_collector')

    def test_collect_and_monitor_method_exists(self):
        """Test that the main monitoring method exists"""
        assert hasattr(self.driving_monitor, 'collect_and_monitor')

    def test_initialize_dec_system_method_exists(self):
        """Test that DEC system initialization method exists"""
        assert hasattr(self.driving_monitor, 'initialize_dec_system')

    def test_run_monitoring_cycle_method_exists(self):
        """Test that the main run method exists"""
        assert hasattr(self.driving_monitor, 'run_monitoring_cycle')

    def test_collect_and_monitor_with_mock_data(self):
        """Test full monitoring collection with mock data"""
        # Create mock SubMaster and metrics collector
        mock_sm = Mock()
        mock_metrics = Mock()
        mock_metrics.collect_metrics.return_value = Mock()
        mock_metrics.collect_metrics.return_value.__dict__ = {}
        mock_metrics.get_performance_report.return_value = {
            'duration': 5.0,
            'avg_cpu_util': 50.0,
            'avg_lateral_jerk': 1.2,
            'driver_interventions': 0,
        }
        mock_metrics.get_system_health.return_value = {
            'status': 'healthy',
            'safety_score': 1.0,
            'stability_score': 1.0,
            'smoothness_score': 1.0,
        }

        # Mock the metrics collector
        with patch('selfdrive.monitoring.driving_monitor.get_metrics_collector') as mock_get_collector:
            mock_get_collector.return_value = mock_metrics

            # Initialize and run collection
            driving_monitor = AutonomousDrivingMonitor()

            # Update monitoring
            result = driving_monitor.collect_and_monitor()

            # Verify results structure
            assert 'system_health' in result
            assert 'current_metrics' in result
            assert 'cycle_time_ms' in result

    def test_monitoring_cycle_execution(self):
        """Test the continuous monitoring cycle"""
        # Mock the metrics collector
        mock_metrics = Mock()
        mock_metrics.collect_metrics.return_value = Mock()
        mock_metrics.collect_metrics.return_value.__dict__ = {}
        mock_metrics.get_performance_report.return_value = {
            'duration': 5.0,
            'avg_cpu_util': 50.0,
            'avg_lateral_jerk': 1.2,
            'driver_interventions': 0,
        }
        mock_metrics.get_system_health.return_value = {
            'status': 'healthy',
            'safety_score': 1.0,
            'stability_score': 1.0,
            'smoothness_score': 1.0,
        }

        with patch('selfdrive.monitoring.driving_monitor.get_metrics_collector') as mock_get_collector:
            mock_get_collector.return_value = mock_metrics

            driving_monitor = AutonomousDrivingMonitor()

            # Test short monitoring period with mocked sleep
            with patch('time.sleep', return_value=None):  # Mock sleep to avoid actual sleep
                # Immediately break the loop after one iteration by patching the condition
                with patch.object(driving_monitor, 'shutdown', True):
                    driving_monitor.run_monitoring_cycle(duration=0.1)

            # The frame count should be accessible
            assert hasattr(driving_monitor, 'frame_count')

    def test_error_handling_in_collection(self):
        """Test error handling in monitoring collection"""
        mock_metrics = Mock()
        mock_metrics.collect_metrics.side_effect = Exception("Test error")
        mock_metrics.get_performance_report.side_effect = Exception("Test error")
        mock_metrics.get_system_health.side_effect = Exception("Test error")

        with patch('selfdrive.monitoring.driving_monitor.get_metrics_collector') as mock_get_collector:
            mock_get_collector.return_value = mock_metrics

            driving_monitor = AutonomousDrivingMonitor()

            # This should not raise an exception despite the errors in metrics collection
            result = driving_monitor.collect_and_monitor()

            # Should have default or error values
            assert 'system_health' in result
            assert 'current_metrics' in result
            assert 'cycle_time_ms' in result


if __name__ == "__main__":
    pytest.main([__file__])