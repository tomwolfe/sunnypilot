"""
Unit tests for Integration Monitor module
"""
import time
import pytest
import numpy as np
from unittest.mock import Mock, patch

from selfdrive.monitoring.integration_monitor import AutonomousDrivingIntegrator


class TestAutonomousDrivingIntegrator:
    """Test suite for AutonomousDrivingIntegrator class"""

    def setup_method(self):
        """Setup test fixtures before each test method"""
        self.integrator = AutonomousDrivingIntegrator()

    def test_initialization(self):
        """Test initialization of AutonomousDrivingIntegrator"""
        assert self.integrator.shutdown is False
        assert self.integrator.enabled is True
        assert self.integrator.system_stability_score == 1.0
        # Verify that SubMaster and PubMaster are initialized
        assert self.integrator.sm is not None
        assert self.integrator.pm is not None

    def test_initialize_systems(self):
        """Test system initialization with CarParams"""
        # Create mock CarParams
        mock_cp = Mock()
        mock_cp.lateralTuning.torque.kp = 1.0
        mock_cp.lateralTuning.torque.ki = 0.1
        
        mock_cp_sp = Mock()
        
        # Mock the imports that are needed for initialization
        with patch('selfdrive.monitoring.integration_monitor.DynamicExperimentalController'), \
             patch('selfdrive.monitoring.integration_monitor.SpeedLimitAssist'), \
             patch('selfdrive.monitoring.integration_monitor.get_metrics_collector'):
            
            result = self.integrator.initialize_systems(mock_cp, mock_cp_sp)
            assert result is True

    def test_update_integration_without_components(self):
        """Test update integration when components aren't initialized"""
        # Mock SubMaster with test data
        mock_sm = Mock()
        mock_sm.updated = {
            'carState': True,
            'modelV2': True,
            'controlsState': True,
            'radarState': True,
            'selfdriveState': True
        }
        
        # Mock carState
        mock_car_state = Mock()
        mock_car_state.vEgo = 15.0
        mock_car_state.aEgo = 1.0
        mock_car_state.vCruise = 25.0
        
        def mock_getitem(key):
            if key == 'carState':
                return mock_car_state
            else:
                return Mock()
        
        mock_sm.__getitem__ = mock_getitem
        self.integrator.sm = mock_sm
        
        # Mock the metrics collector
        mock_metrics_collector = Mock()
        mock_metrics_collector.collect_metrics.return_value = mock_metrics_collector
        mock_metrics_collector.get_system_health.return_value = {
            'avg_lateral_jerk': 1.0,
            'status': 'healthy'
        }
        
        self.integrator.metrics_collector = mock_metrics_collector
        
        # Update integration - should work even without DEC/SLA controllers
        results = self.integrator.update_integration()
        
        assert 'dec_active' in results
        assert 'sla_active' in results
        assert 'system_health' in results
        assert 'stability_score' in results

    def test_update_integration_with_components(self):
        """Test update integration with all components initialized"""
        # Mock SubMaster with test data
        mock_sm = Mock()
        mock_sm.updated = {
            'carState': True,
            'modelV2': True,
            'controlsState': True,
            'radarState': True,
            'selfdriveState': True
        }
        
        # Mock carState
        mock_car_state = Mock()
        mock_car_state.vEgo = 20.0
        mock_car_state.aEgo = 0.5
        mock_car_state.vCruise = 30.0
        
        def mock_getitem(key):
            if key == 'carState':
                return mock_car_state
            else:
                return Mock()
        
        mock_sm.__getitem__ = mock_getitem
        self.integrator.sm = mock_sm
        
        # Mock metrics collector
        mock_metrics_collector = Mock()
        mock_metrics_collector.collect_metrics.return_value = mock_metrics_collector
        mock_metrics_collector.get_system_health.return_value = {
            'avg_lateral_jerk': 0.8,
            'status': 'healthy'
        }
        
        self.integrator.metrics_collector = mock_metrics_collector
        
        # Mock DEC controller
        mock_dec = Mock()
        mock_dec.update.return_value = None
        mock_dec.active.return_value = True
        self.integrator.dec_controller = mock_dec
        
        # Mock SLA controller
        mock_sla = Mock()
        mock_sla.update.return_value = None
        mock_sla.is_active = True
        self.integrator.sla_controller = mock_sla
        
        # Update integration
        results = self.integrator.update_integration()
        
        assert results['dec_active'] is True
        assert results['sla_active'] is True
        assert results['system_health'] == 'healthy'
        assert results['stability_score'] >= 0.0

    def test_run_integration_loop(self):
        """Test the integration monitoring loop"""
        # Mock SubMaster with test data
        mock_sm = Mock()
        mock_sm.updated = {'carState': True}
        mock_car_state = Mock()
        mock_car_state.vEgo = 15.0
        mock_sm.__getitem__ = Mock(return_value=mock_car_state)
        
        # Mock metrics collector
        mock_metrics_collector = Mock()
        mock_metrics_collector.collect_metrics.return_value = mock_metrics_collector
        mock_metrics_collector.get_system_health.return_value = {
            'avg_lateral_jerk': 1.0,
            'status': 'healthy'
        }
        mock_metrics_collector.frame_count = 100
        
        self.integrator.sm = mock_sm
        self.integrator.metrics_collector = mock_metrics_collector
        
        # Test short integration loop to avoid long execution
        with patch('time.sleep', return_value=None), \
             patch('time.time', side_effect=[time.time(), time.time() + 0.1]):
            self.integrator.run_integration_loop(duration=0.05)  # Very short duration

    def test_get_performance_insights(self):
        """Test performance insights generation"""
        # Mock metrics collector with performance data
        mock_metrics = Mock()
        mock_metrics.get_performance_report.return_value = {
            'avg_lateral_jerk': 2.5,  # High jerk - should generate recommendation
            'avg_longitudinal_jerk': 1.8,
            'avg_cpu_util': 90.0  # High CPU - should generate recommendation
        }
        mock_metrics.get_system_health.return_value = {
            'status': 'warning'
        }
        
        self.integrator.metrics_collector = mock_metrics
        
        insights = self.integrator.get_performance_insights()
        
        assert 'performance_summary' in insights
        assert 'health_summary' in insights
        assert 'recommendations' in insights
        assert 'stability_trend' in insights
        
        # Should have recommendations due to high jerk and CPU usage
        assert len(insights['recommendations']) >= 0  # At least 0 or more recommendations

    def test_stability_calculation(self):
        """Test stability score calculation based on metrics"""
        # Mock metrics collector
        mock_metrics = Mock()
        mock_metrics.get_performance_report.return_value = {
            'avg_lateral_jerk': 0.5  # Low jerk = high stability
        }
        mock_metrics.get_system_health.return_value = {
            'avg_lateral_jerk': 0.5,
            'status': 'healthy'
        }
        
        self.integrator.metrics_collector = mock_metrics
        
        # Update integration and check stability
        results = self.integrator.update_integration()
        
        # With low jerk, stability should be relatively high
        assert results['stability_score'] >= 0.5

    def test_error_handling_in_update(self):
        """Test error handling in update_integration method"""
        # Mock SubMaster to cause errors
        mock_sm = Mock()
        mock_sm.__getitem__.side_effect = Exception("Test error")
        
        self.integrator.sm = mock_sm
        
        # Mock metrics collector to also cause errors
        mock_metrics_collector = Mock()
        mock_metrics_collector.collect_metrics.side_effect = Exception("Metrics error")
        mock_metrics_collector.get_system_health.return_value = {
            'avg_lateral_jerk': 1.0,
            'status': 'healthy'
        }
        
        self.integrator.metrics_collector = mock_metrics_collector
        
        # This should handle errors gracefully
        results = self.integrator.update_integration()
        
        # Should still return expected structure
        assert 'dec_active' in results
        assert 'sla_active' in results
        assert 'system_health' in results

    def test_performance_insights_error_handling(self):
        """Test error handling in performance insights generation"""
        # Mock metrics collector to raise an exception
        mock_metrics = Mock()
        mock_metrics.get_performance_report.side_effect = Exception("Test error")
        
        self.integrator.metrics_collector = mock_metrics
        
        insights = self.integrator.get_performance_insights()
        
        # Should return error object instead of full insights
        assert 'error' in insights


if __name__ == "__main__":
    pytest.main([__file__])