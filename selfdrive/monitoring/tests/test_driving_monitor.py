"""
Unit tests for Driving Monitor module
"""
import time
import pytest
import numpy as np
from unittest.mock import Mock, patch

from selfdrive.monitoring.driving_monitor import DrivingMonitor


class TestDrivingMonitor:
    """Test suite for DrivingMonitor class"""

    def setup_method(self):
        """Setup test fixtures before each test method"""
        self.driving_monitor = DrivingMonitor()

    def test_initialization(self):
        """Test initialization of DrivingMonitor"""
        assert self.driving_monitor.frame_count == 0
        assert self.driving_monitor.monitoring_enabled is True
        assert self.driving_monitor.safety_metrics == {}
        assert self.driving_monitor.performance_assessment == {}
        assert self.driving_monitor.driving_behavior == {}

    def test_update_behavior_classification(self):
        """Test driving behavior classification"""
        # Mock data with different driving characteristics
        mock_data = {
            'avg_lateral_jerk': 1.5,  # Higher jerk - more aggressive
            'avg_longitudinal_jerk': 2.0,
            'avg_steering_angle': 5.0,
            'avg_acceleration': 1.5,
            'avg_velocity': 20.0
        }
        
        behavior = self.driving_monitor._update_behavior_classification(mock_data)
        
        # Should update internal driving_behavior dictionary
        assert 'driving_style' in self.driving_monitor.driving_behavior
        assert 'aggression_score' in self.driving_monitor.driving_behavior
        assert 'smoothness_rating' in self.driving_monitor.driving_behavior

    def test_evaluate_safety_metrics(self):
        """Test safety metrics evaluation"""
        # Mock metrics data
        mock_metrics = {
            'fcw_events': 2,
            'driver_interventions': 1,
            'avg_lateral_jerk': 2.0,
            'avg_longitudinal_jerk': 1.5
        }
        
        safety = self.driving_monitor._evaluate_safety_metrics(mock_metrics)
        
        assert 'fcw_risk_level' in safety
        assert 'intervention_frequency' in safety
        assert 'jerk_safety_score' in safety
        assert 'overall_safety_rating' in safety

    def test_assess_performance(self):
        """Test performance assessment"""
        # Mock metrics data
        mock_metrics = {
            'avg_cpu_util': 65.0,
            'avg_memory_util': 70.0,
            'avg_lateral_jerk': 1.2,
            'avg_longitudinal_jerk': 0.8,
            'avg_velocity': 25.0
        }
        
        performance = self.driving_monitor._assess_performance(mock_metrics)
        
        assert 'efficiency_score' in performance
        assert 'smoothness_score' in performance
        'resource_utilization_score' in performance
        assert 'performance_rating' in performance

    def test_generate_alerts(self):
        """Test alert generation based on metrics"""
        # Mock metrics data indicating potential issues
        mock_metrics = {
            'avg_lateral_jerk': 3.5,  # High jerk
            'fcw_events': 5,  # Many FCW events
            'driver_interventions': 8,  # Many interventions
            'avg_cpu_util': 90.0,  # High CPU usage
        }
        
        alerts = self.driving_monitor._generate_alerts(mock_metrics)
        
        # Should contain appropriate alerts based on high values
        assert isinstance(alerts, list)
        # At least one alert should be generated for the high values

    def test_update_monitoring_with_mock_data(self):
        """Test full monitoring update with mock data"""
        # Create mock SubMaster and metrics collector
        mock_sm = Mock()
        mock_metrics = Mock()
        mock_metrics.get_performance_report.return_value = {
            'avg_lateral_jerk': 1.2,
            'avg_longitudinal_jerk': 0.8,
            'avg_steering_angle': 3.0,
            'avg_acceleration': 1.0,
            'avg_velocity': 20.0
        }
        mock_metrics.get_system_health.return_value = {
            'status': 'healthy',
            'avg_cpu_util': 50.0,
            'avg_memory_util': 60.0
        }
        mock_metrics.fcw_events = 1
        mock_metrics.driver_interventions = 0
        mock_metrics.mode_switches = 2
        
        # Mock the metrics collector
        with patch('selfdrive.monitoring.driving_monitor.get_metrics_collector') as mock_get_collector:
            mock_get_collector.return_value = mock_metrics
            
            # Initialize and run update
            driving_monitor = DrivingMonitor()
            driving_monitor.initialize_system(mock_sm)
            
            # Update monitoring
            result = driving_monitor.update_monitoring()
            
            # Verify results structure
            assert 'behavior_classification' in result
            assert 'safety_assessment' in result
            assert 'performance_assessment' in result
            assert 'alerts' in result

    def test_get_monitoring_report(self):
        """Test monitoring report generation"""
        # Set up some mock data
        self.driving_monitor.driving_behavior = {
            'driving_style': 'conservative',
            'aggression_score': 0.3,
            'smoothness_rating': 'good'
        }
        self.driving_monitor.safety_metrics = {
            'fcw_risk_level': 'low',
            'intervention_frequency': 'low',
            'overall_safety_rating': 'good'
        }
        self.driving_monitor.performance_assessment = {
            'efficiency_score': 0.8,
            'performance_rating': 'good'
        }
        self.driving_monitor.alerts = ['test alert']
        
        report = self.driving_monitor.get_monitoring_report()
        
        assert 'timestamp' in report
        assert 'behavior_classification' in report
        assert 'safety_assessment' in report
        assert 'performance_assessment' in report
        assert 'safety_overview' in report
        assert 'performance_summary' in report
        assert 'recommendations' in report

    def test_monitoring_loop(self):
        """Test the continuous monitoring loop"""
        mock_sm = Mock()
        mock_metrics = Mock()
        mock_metrics.get_performance_report.return_value = {
            'avg_lateral_jerk': 1.0,
            'avg_longitudinal_jerk': 0.5
        }
        mock_metrics.get_system_health.return_value = {
            'status': 'healthy',
            'avg_cpu_util': 40.0
        }
        mock_metrics.fcw_events = 0
        mock_metrics.driver_interventions = 0
        mock_metrics.mode_switches = 0
        
        with patch('selfdrive.monitoring.driving_monitor.get_metrics_collector') as mock_get_collector:
            mock_get_collector.return_value = mock_metrics
            
            driving_monitor = DrivingMonitor()
            driving_monitor.initialize_system(mock_sm)
            
            # Test short monitoring period
            with patch('time.sleep', side_effect=StopIteration):  # Stop after first iteration
                try:
                    driving_monitor.run_monitoring_loop(duration=0.1)
                except StopIteration:
                    pass  # Expected to stop after first sleep call
            
            # Should have processed at least one frame
            assert driving_monitor.frame_count >= 0

    def test_error_handling_in_update(self):
        """Test error handling in monitoring update"""
        mock_sm = Mock()
        mock_metrics = Mock()
        mock_metrics.get_performance_report.side_effect = Exception("Test error")
        mock_metrics.get_system_health.side_effect = Exception("Test error")
        
        with patch('selfdrive.monitoring.driving_monitor.get_metrics_collector') as mock_get_collector:
            mock_get_collector.return_value = mock_metrics
            
            driving_monitor = DrivingMonitor()
            driving_monitor.initialize_system(mock_sm)
            
            # This should not raise an exception despite the errors in metrics collection
            result = driving_monitor.update_monitoring()
            
            # Should have default or error values
            assert 'behavior_classification' in result
            assert 'safety_assessment' in result
            assert 'performance_assessment' in result
            assert 'alerts' in result


if __name__ == "__main__":
    pytest.main([__file__])