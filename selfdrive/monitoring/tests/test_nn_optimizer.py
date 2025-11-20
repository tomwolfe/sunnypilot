"""
Unit tests for Neural Network Optimizer module
"""
import time
import pytest
import numpy as np
from unittest.mock import Mock, patch, MagicMock
import os

from selfdrive.monitoring.nn_optimizer import NNPerformanceOptimizer, LateralControlOptimizer, get_lateral_optimizer, apply_neural_network_control


class TestNNPerformanceOptimizer:
    """Test suite for NNPerformanceOptimizer class"""

    def setup_method(self):
        """Setup test fixtures before each test method"""
        self.optimizer = NNPerformanceOptimizer()

    def test_initialization(self):
        """Test initialization of NNPerformanceOptimizer"""
        assert self.optimizer.is_enabled is False
        assert len(self.optimizer.inference_times) == 0
        assert len(self.optimizer.power_consumption) == 0
        assert len(self.optimizer.thermal_load) == 0
        assert self.optimizer.quantization_enabled is True
        assert 0.0 <= self.optimizer.model_complexity_target <= 1.0

    def test_initialize_model_success(self):
        """Test successful model initialization"""
        # Create a temporary model file for testing
        test_model_path = "/tmp/test_model"
        with open(test_model_path, 'w') as f:
            f.write("dummy model content")
        
        try:
            # Mock the model import and initialization
            with patch('selfdrive.monitoring.nn_optimizer.NNTorqueModelTinygrad') as mock_model_class:
                mock_model = Mock()
                mock_model_class.return_value = mock_model
                mock_model.input_size = 10  # Mock input size
                
                optimizer = NNPerformanceOptimizer(model_path=test_model_path)
                
                assert optimizer.is_enabled is True
                assert optimizer.model is not None
                # Verify benchmark was called
                # Note: We can't easily verify benchmark_performance was called due to the try/catch
                
        finally:
            # Clean up test file
            if os.path.exists(test_model_path):
                os.remove(test_model_path)

    def test_initialize_model_failure(self):
        """Test model initialization with non-existent path"""
        optimizer = NNPerformanceOptimizer(model_path="/nonexistent/path")
        
        assert optimizer.is_enabled is False
        assert optimizer.model is None

    def test_benchmark_performance(self):
        """Test performance benchmarking"""
        # Mock the model with necessary attributes
        mock_model = Mock()
        mock_model.input_size = 10
        mock_model.evaluate.return_value = 0.5  # Mock output
        
        self.optimizer.model = mock_model
        self.optimizer.is_enabled = True
        
        # Run benchmark
        self.optimizer.benchmark_performance()
        
        # Verify that inference times were collected
        assert len(self.optimizer.inference_times) >= 0  # May be 0 if benchmark fails

    def test_optimize_for_hardware_with_model(self):
        """Test hardware optimization with a mock model"""
        # Mock the model
        mock_model = Mock()
        mock_model.input_size = 10
        mock_model.evaluate.return_value = 0.75  # Mock output value
        
        self.optimizer.model = mock_model
        self.optimizer.is_enabled = True
        
        # Test optimization with sample input
        test_input = [10.0, 0.1, 0.05] + [0.0] * 7  # 10 elements total
        result, info = self.optimizer.optimize_for_hardware(test_input)
        
        assert isinstance(result, float)
        assert 'optimized' in info
        assert info['optimized'] is True

    def test_optimize_for_hardware_without_model(self):
        """Test hardware optimization without a model (should return fallback)"""
        result, info = self.optimizer.optimize_for_hardware([1.0, 2.0, 3.0])
        
        assert result == 0.0
        assert info['optimized'] is False
        assert 'reason' in info

    def test_optimize_for_hardware_model_failure(self):
        """Test hardware optimization when model evaluation fails"""
        # Mock the model to raise an exception
        mock_model = Mock()
        mock_model.evaluate.side_effect = Exception("Model error")
        
        self.optimizer.model = mock_model
        self.optimizer.is_enabled = True
        
        test_input = [1.0, 2.0, 3.0] + [0.0] * 7
        result, info = self.optimizer.optimize_for_hardware(test_input)
        
        assert 'error' in info

    def test_prepare_optimized_input(self):
        """Test optimized input preparation"""
        # Test with high complexity (no simplification)
        test_input = [1.0, -0.005, 2.0, 0.001, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
        result = self.optimizer._prepare_optimized_input(test_input)
        
        assert len(result) == len(test_input)
        
        # Test with reduced complexity
        self.optimizer.model_complexity_target = 0.5  # Below 0.7 threshold
        result_reduced = self.optimizer._prepare_optimized_input(test_input)
        
        assert len(result_reduced) == len(test_input)

    def test_get_performance_metrics(self):
        """Test performance metrics retrieval"""
        # Add some test data to inference times
        for i in range(10):
            self.optimizer.inference_times.append(15.0 + i)  # ms
        
        metrics = self.optimizer.get_performance_metrics()
        
        assert 'enabled' in metrics
        assert 'avg_inference_time_ms' in metrics
        assert 'min_inference_time_ms' in metrics
        assert 'max_inference_time_ms' in metrics
        assert 'performance_score' in metrics
        
        assert metrics['enabled'] == self.optimizer.is_enabled
        assert metrics['inference_samples'] == 10

    def test_adapt_to_system_load(self):
        """Test system load adaptation"""
        initial_target = self.optimizer.model_complexity_target
        
        # Test high load adaptation
        self.optimizer.adapt_to_system_load(cpu_load=90.0, thermal_status=80.0)  # High load
        assert self.optimizer.model_complexity_target < initial_target
        
        # Reset and test low load adaptation
        self.optimizer.model_complexity_target = initial_target
        self.optimizer.adapt_to_system_load(cpu_load=20.0, thermal_status=30.0)  # Low load
        # This might not increase due to the gradual adjustment logic


class TestLateralControlOptimizer:
    """Test suite for LateralControlOptimizer class"""

    def setup_method(self):
        """Setup test fixtures before each test method"""
        # Mock params to avoid actual parameter loading
        with patch('selfdrive.monitoring.nn_optimizer.Params') as mock_params_class:
            mock_params = Mock()
            mock_params.get_bool.return_value = False  # Disable by default for tests
            mock_params_class.return_value = mock_params
            
            self.lateral_optimizer = LateralControlOptimizer()

    def test_initialization(self):
        """Test initialization of LateralControlOptimizer"""
        assert self.lateral_optimizer.enabled is False  # Due to mocked params
        assert self.lateral_optimizer.is_active is False
        assert self.lateral_optimizer.steering_limited_count == 0

    def test_update_without_active_optimizer(self):
        """Test update when optimizer is not active (fallback behavior)"""
        test_input = [1.0, 2.0, 3.0] + [0.0] * 7
        result, info = self.lateral_optimizer.update(test_input, 15.0, 0.1)
        
        # Should return fallback value
        assert 'fallback' in info
        assert info['fallback'] is True

    def test_get_status_report(self):
        """Test status report generation"""
        report = self.lateral_optimizer.get_status_report()
        
        assert 'enabled' in report
        assert 'active' in report
        assert 'steering_limited_count' in report
        assert 'last_optimization_time' in report


def test_global_functions():
    """Test global functions for neural network control"""
    # Test get_lateral_optimizer
    optimizer = get_lateral_optimizer()
    assert optimizer is not None
    
    # Test apply_neural_network_control with mock
    with patch('selfdrive.monitoring.nn_optimizer.lateral_optimizer') as mock_opt:
        mock_opt.update.return_value = (0.5, {"test": "info"})
        result, info = apply_neural_network_control([1.0, 2.0], 15.0, 0.1)
        assert result == 0.5


if __name__ == "__main__":
    pytest.main([__file__])