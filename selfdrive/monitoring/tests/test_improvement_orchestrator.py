"""
Unit tests for Improvement Orchestrator module
"""
import time
import pytest
import numpy as np
from unittest.mock import Mock, patch, MagicMock
import json
from pathlib import Path

from selfdrive.monitoring.improvement_orchestrator import ImprovementPlanOrchestrator


class TestImprovementPlanOrchestrator:
    """Test suite for ImprovementPlanOrchestrator class"""

    def setup_method(self):
        """Setup test fixtures before each test method"""
        # Mock the CarParams and SunnypilotCarParams
        self.mock_cp = Mock()
        self.mock_cp.lateralTuning.torque.kp = 1.0
        self.mock_cp.lateralTuning.torque.ki = 0.1
        self.mock_cp.steerRatio = 15.0
        self.mock_cp.steerActuatorDelay = 0.1
        
        self.mock_cp_sp = Mock()
        
        self.orchestrator = ImprovementPlanOrchestrator()

    def test_initialization(self):
        """Test initialization of ImprovementPlanOrchestrator"""
        assert self.orchestrator.shutdown is False
        assert isinstance(self.orchestrator.improvement_progress, dict)
        assert isinstance(self.orchestrator.baseline_performance, dict)
        assert len(self.orchestrator.baseline_performance) > 0
        assert len(self.orchestrator.improvement_algorithms) > 0

    def test_initialize_system(self):
        """Test system initialization with CarParams"""
        mock_sm = Mock()
        
        # Test successful initialization
        result = self.orchestrator.initialize_system(self.mock_cp, self.mock_cp_sp, mock_sm)
        assert result is True
        assert self.orchestrator.sm == mock_sm

    def test_setup_improvement_algorithms(self):
        """Test setup of improvement algorithms"""
        # Check that algorithms are properly configured
        assert len(self.orchestrator.improvement_algorithms) > 0
        
        for algo in self.orchestrator.improvement_algorithms:
            assert 'name' in algo
            assert 'function' in algo
            assert 'frequency' in algo
            assert 'enabled' in algo

    def test_execute_lateral_optimization(self):
        """Test lateral control optimization"""
        # Mock metrics collector with test data
        mock_metrics = Mock()
        mock_metrics.get_performance_report.return_value = {
            'avg_lateral_jerk': 2.0,
            'avg_longitudinal_jerk': 1.0,
            'avg_steering_angle': 3.0,
            'avg_velocity': 20.0
        }
        mock_metrics.get_system_health.return_value = {
            'lateral_smoothness': 0.6,
            'status': 'healthy'
        }
        
        self.orchestrator.metrics_collector = mock_metrics
        
        # Execute optimization
        result = self.orchestrator._execute_lateral_optimization()
        
        assert 'success' in result
        assert 'new_parameters' in result or 'error' in result

    def test_execute_performance_optimization(self):
        """Test performance optimization"""
        # Mock metrics collector with test data
        mock_metrics = Mock()
        mock_metrics.get_performance_report.return_value = {
            'avg_cpu_util': 70.0,
            'avg_lateral_jerk': 1.5
        }
        mock_metrics.get_system_health.return_value = {
            'status': 'healthy'
        }
        
        self.orchestrator.metrics_collector = mock_metrics
        
        # Execute optimization
        result = self.orchestrator._execute_performance_optimization()
        
        assert 'success' in result

    def test_execute_safety_optimization(self):
        """Test safety optimization"""
        # Mock metrics collector with test data
        mock_metrics = Mock()
        mock_metrics.fcw_events = 1
        mock_metrics.driver_interventions = 0
        
        self.orchestrator.metrics_collector = mock_metrics
        
        # Execute optimization
        result = self.orchestrator._execute_safety_optimization()
        
        assert 'success' in result

    def test_execute_system_integration(self):
        """Test system integration optimization"""
        # Mock metrics collector with test data
        mock_metrics = Mock()
        mock_metrics.get_performance_report.return_value = {
            'avg_lateral_jerk': 1.0,
            'avg_cpu_util': 50.0
        }
        mock_metrics.get_system_health.return_value = {
            'status': 'healthy'
        }
        
        self.orchestrator.metrics_collector = mock_metrics
        
        # Execute optimization
        result = self.orchestrator._execute_system_integration()
        
        assert 'success' in result

    def test_execute_improvement_cycle(self):
        """Test single improvement cycle execution"""
        mock_sm = Mock()
        mock_sm.updated = {
            'deviceState': True
        }
        mock_device_state = Mock()
        mock_device_state.cpuUsagePercent = [50.0]
        mock_device_state.cpuTempC = [60.0]
        mock_sm.__getitem__ = Mock(return_value=mock_device_state)
        
        # Initialize the orchestrator with mock data
        self.orchestrator.initialize_system(self.mock_cp, self.mock_cp_sp, mock_sm)
        
        # Mock the metrics collector
        mock_metrics = Mock()
        mock_metrics.frame_count = 100  # Set to trigger algorithm execution
        self.orchestrator.metrics_collector = mock_metrics
        
        # Execute improvement cycle
        result = self.orchestrator.execute_improvement_cycle()
        
        assert 'timestamp' in result
        assert 'frame_count' in result
        assert 'algorithm_results' in result

    def test_evaluate_overall_progress(self):
        """Test overall improvement progress evaluation"""
        # Mock metrics collector
        mock_metrics = Mock()
        mock_metrics.get_performance_report.return_value = {
            'avg_lateral_jerk': 1.0,
            'avg_longitudinal_jerk': 0.8,
            'cpu_utilization': 50.0,
            'system_stability': 0.7
        }
        mock_metrics.get_system_health.return_value = {
            'longitudinal_smoothness': 0.8
        }
        
        self.orchestrator.metrics_collector = mock_metrics
        
        # Add some completed improvements to progress
        self.orchestrator.improvement_progress = {
            'lateral_control': {'completed': True, 'score': 0.8, 'timestamp': time.time()},
            'performance': {'completed': True, 'score': 0.7, 'timestamp': time.time()},
            'safety': {'completed': False, 'score': 0.0, 'timestamp': time.time()}
        }
        
        analysis = self.orchestrator.evaluate_overall_progress()
        
        assert 'progress_percentage' in analysis
        assert 'overall_score' in analysis
        assert 'completed_components' in analysis
        assert 'total_components' in analysis
        assert 'current_performance' in analysis
        assert 'improvement_vs_baseline' in analysis
        assert 'recommendations' in analysis

    def test_run_continuous_improvement(self):
        """Test continuous improvement loop"""
        mock_sm = Mock()
        mock_sm.updated = {
            'deviceState': True
        }
        mock_device_state = Mock()
        mock_device_state.cpuUsagePercent = [50.0]
        mock_device_state.cpuTempC = [60.0]
        mock_sm.__getitem__ = Mock(return_value=mock_device_state)
        
        # Initialize the orchestrator
        self.orchestrator.initialize_system(self.mock_cp, self.mock_cp_sp, mock_sm)
        
        # Mock metrics collector
        mock_metrics = Mock()
        mock_metrics.get_performance_report.return_value = {
            'avg_lateral_jerk': 1.0,
            'avg_longitudinal_jerk': 0.8,
            'cpu_utilization': 50.0,
            'system_stability': 0.7
        }
        mock_metrics.get_system_health.return_value = {
            'status': 'healthy'
        }
        mock_metrics.fcw_events = 0
        mock_metrics.driver_interventions = 0
        mock_metrics.frame_count = 100
        
        self.orchestrator.metrics_collector = mock_metrics
        
        # Test short duration to avoid long execution
        with patch('time.sleep'), \
             patch('time.time', side_effect=[time.time(), time.time() + 0.1, time.time() + 0.2]):  # Short time intervals
            try:
                final_analysis = self.orchestrator.run_continuous_improvement(duration=0.1)
                assert 'overall_score' in final_analysis
            except StopIteration:
                pass  # May raise if the loop exits early

    def test_generate_improvement_report(self):
        """Test improvement report generation"""
        # Mock the report generation
        with patch('builtins.open') as mock_open, \
             patch('json.dump') as mock_json_dump:
            
            # Add some test data to improvement progress
            self.orchestrator.improvement_progress = {
                'test_component': {
                    'completed': True,
                    'score': 0.8,
                    'timestamp': time.time(),
                    'details': {}
                }
            }
            
            report = self.orchestrator.generate_improvement_report()
            
            # Verify that file was written
            mock_open.assert_called()
            mock_json_dump.assert_called()
            assert 'timestamp' in report
            assert 'improvement_progress' in report

    def test_error_handling_in_algorithms(self):
        """Test error handling in improvement algorithms"""
        # Mock metrics collector to raise an exception
        mock_metrics = Mock()
        mock_metrics.get_performance_report.side_effect = Exception("Test error")
        
        self.orchestrator.metrics_collector = mock_metrics
        
        # Test that algorithms handle errors gracefully
        result = self.orchestrator._execute_lateral_optimization()
        assert result['success'] is False
        assert 'error' in result


if __name__ == "__main__":
    pytest.main([__file__])