"""
Tests for Perfect Grade E2E Enhancements
=========================================

This module contains tests for all 5 Perfect Grade improvements:
1. Unified World Model (Longitudinal + Lateral)
2. Uncertainty Quantification with Trajectory Variance
3. Temporal Multi-Frame Vision (10-frame buffer)
4. Cost-to-Go Policy Integration
5. Hardware Latency Optimization
"""

import numpy as np
import pytest
from unittest.mock import Mock
import time


class TestUnifiedPolicy:
    """Tests for Unified Policy Transformer"""
    
    def test_unified_policy_initialization(self):
        """Test unified policy transformer initializes correctly"""
        from sunnypilot.modeld_v2.e2e_torque.unified_policy import UnifiedPolicyTransformer
        
        policy = UnifiedPolicyTransformer(
            hidden_dim=256,
            num_heads=8,
            lateral_modes=3,
            longitudinal_modes=3,
        )
        
        assert policy.hidden_dim == 256
        assert policy.num_heads == 8
        assert policy.lateral_modes == 3
        assert policy.longitudinal_modes == 3
    
    def test_unified_policy_forward(self):
        """Test unified policy forward pass"""
        from sunnypilot.modeld_v2.e2e_torque.unified_policy import UnifiedPolicyTransformer
        
        policy = UnifiedPolicyTransformer(hidden_dim=128, num_heads=4)
        
        # Create dummy vision features
        vision_features = np.random.randn(10, 128).astype(np.float32)
        
        # Run forward pass
        output = policy.forward(
            vision_features,
            vehicle_state={'v_ego': 10.0, 'a_ego': 0.0},
            context=None
        )
        
        # Verify outputs
        assert output.steering_torque != 0.0 or output.throttle_command != 0.0
        assert output.confidence > 0.0
        assert output.confidence <= 1.0
        assert output.is_valid
    
    def test_cross_attention_coordination(self):
        """Test cross-attention module for lateral-longitudinal coordination"""
        from sunnypilot.modeld_v2.e2e_torque.unified_policy import CrossAttentionModule
        
        cross_attn = CrossAttentionModule(hidden_dim=128, num_heads=4)
        
        lateral_latent = np.random.randn(1, 128).astype(np.float32)
        longitudinal_latent = np.random.randn(1, 128).astype(np.float32)
        
        updated_lateral, updated_longitudinal = cross_attn(
            lateral_latent, longitudinal_latent
        )
        
        # Verify outputs have correct shape
        assert updated_lateral.shape == lateral_latent.shape
        assert updated_longitudinal.shape == longitudinal_latent.shape
        
        # Verify residual connection
        assert not np.allclose(updated_lateral, lateral_latent)
        assert not np.allclose(updated_longitudinal, longitudinal_latent)


class TestUncertaintyQuantification:
    """Tests for Uncertainty Quantification with Trajectory Variance"""
    
    def test_trajectory_variance_predictor(self):
        """Test trajectory variance prediction"""
        from sunnypilot.modeld_v2.e2e_torque.uncertainty_estimation import (
            TrajectoryVariancePredictor,
            TrajectoryVariance,
        )
        
        predictor = TrajectoryVariancePredictor(
            horizon_steps=50,
            dt=0.1,
        )
        
        # Create dummy trajectory features
        trajectory_features = np.random.randn(50, 256).astype(np.float32)
        
        # Predict variance
        variance = predictor.predict_trajectory_variance(
            trajectory_features,
            context={'objects': [{'distance': 20.0}]}
        )
        
        # Verify outputs
        assert isinstance(variance, TrajectoryVariance)
        assert variance.position_variance.shape[0] == 50
        assert variance.position_variance.shape[1] == 2
        assert variance.path_uncertainty >= 0.0
        assert variance.endpoint_uncertainty >= 0.0
    
    def test_uncertainty_growth_modeling(self):
        """Test that uncertainty grows over trajectory horizon"""
        from sunnypilot.modeld_v2.e2e_torque.uncertainty_estimation import TrajectoryVariancePredictor
        
        predictor = TrajectoryVariancePredictor(horizon_steps=50, dt=0.1)
        
        trajectory_features = np.random.randn(50, 256).astype(np.float32)
        variance = predictor.predict_trajectory_variance(trajectory_features)
        
        # Early uncertainty should be lower than late uncertainty
        early_uncertainty = np.mean(np.sum(variance.position_variance[:5], axis=1))
        late_uncertainty = np.mean(np.sum(variance.position_variance[-5:], axis=1))
        
        assert late_uncertainty >= early_uncertainty
    
    def test_uncertainty_aware_controller(self):
        """Test uncertainty-aware controller"""
        from sunnypilot.modeld_v2.e2e_torque.uncertainty_estimation import (
            UncertaintyAwareController,
            UncertaintyOutput,
        )
        
        controller = UncertaintyAwareController(
            method='mc_dropout',
            num_samples=5,
            enable_trajectory_variance=True,
        )
        
        # Dummy forward pass
        def forward_fn(features, mask=None):
            return np.sum(features)
        
        input_features = np.random.randn(256).astype(np.float32)
        trajectory_features = np.random.randn(50, 256).astype(np.float32)
        
        output = controller.estimate(
            forward_fn,
            input_features,
            trajectory_features=trajectory_features,
        )
        
        assert isinstance(output, UncertaintyOutput)
        assert output.confidence_score >= 0.0
        assert output.confidence_score <= 1.0
        assert output.trajectory_variance is not None


class TestTemporalBuffer:
    """Tests for Enhanced Temporal Multi-Frame Vision"""
    
    def test_enhanced_temporal_buffer_initialization(self):
        """Test enhanced temporal buffer initialization"""
        from sunnypilot.modeld_v2.enhanced_temporal_buffer import EnhancedTemporalBuffer
        
        buffer = EnhancedTemporalBuffer(
            num_frames=10,
            frame_height=256,
            frame_width=512,
            enable_motion_features=True,
        )
        
        assert buffer.num_frames == 10
        assert buffer.frame_height == 256
        assert buffer.frame_width == 512
        assert buffer.enable_motion_features
    
    def test_temporal_buffer_frame_addition(self):
        """Test adding frames to temporal buffer"""
        from sunnypilot.modeld_v2.enhanced_temporal_buffer import EnhancedTemporalBuffer
        
        buffer = EnhancedTemporalBuffer(num_frames=10)
        
        # Add frames
        for i in range(15):
            frame = np.random.randn(256, 512, 3).astype(np.float32)
            state = buffer.add_frame(frame, time.time())
            
            assert state.frame_count == min(i + 1, 10)
        
        # Buffer should be full
        assert buffer.is_full
        assert state.frame_count == 10
    
    def test_temporal_attention_weights(self):
        """Test temporal attention weighting"""
        from sunnypilot.modeld_v2.enhanced_temporal_buffer import EnhancedTemporalBuffer
        
        buffer = EnhancedTemporalBuffer(num_frames=10, enable_temporal_attention=True)
        
        # Add frames
        for i in range(10):
            frame = np.random.randn(256, 512, 3).astype(np.float32)
            buffer.add_frame(frame, time.time())
        
        # Check attention weights (recent frames should have higher weights)
        weights = buffer.importance_weights
        assert len(weights) == 10
        assert np.isclose(np.sum(weights), 1.0, atol=1e-5)
        
        # Recent frames should have higher weights
        assert weights[-1] > weights[0]
    
    def test_motion_feature_extraction(self):
        """Test motion feature extraction between frames"""
        from sunnypilot.modeld_v2.enhanced_temporal_buffer import EnhancedTemporalBuffer
        
        buffer = EnhancedTemporalBuffer(
            num_frames=10,
            enable_motion_features=True,
        )
        
        # Add two different frames
        frame1 = np.random.randn(256, 512, 3).astype(np.float32)
        buffer.add_frame(frame1, time.time())
        
        frame2 = frame1 + 0.1  # Add some motion
        buffer.add_frame(frame2, time.time())
        
        # Check motion features
        assert buffer.motion_features is not None
        assert buffer.motion_features.shape[0] >= 1


class TestCostToGo:
    """Tests for Cost-to-Go Policy Integration"""
    
    def test_cost_to_go_predictor(self):
        """Test Cost-to-Go prediction"""
        from sunnypilot.modeld_v2.e2e_torque.cost_to_go import (
            CostToGoPredictor,
            CostToGoOutput,
        )
        
        predictor = CostToGoPredictor(hidden_dim=128)
        
        state_features = np.random.randn(128).astype(np.float32)
        candidate_actions = [
            np.array([0.0, 0.3, 0.0, 0.0], dtype=np.float32),
            np.array([0.2, 0.5, 0.0, 0.0], dtype=np.float32),
            np.array([-0.2, 0.3, 0.1, 0.0], dtype=np.float32),
        ]
        
        outputs = predictor.predict_cost_to_go(state_features, candidate_actions)
        
        assert len(outputs) == 3
        for output in outputs:
            assert isinstance(output, CostToGoOutput)
            assert output.immediate_cost >= 0.0
            assert output.short_term_cost >= 0.0
            assert output.long_term_cost >= 0.0
    
    def test_cost_breakdown(self):
        """Test cost breakdown by category"""
        from sunnypilot.modeld_v2.e2e_torque.cost_to_go import CostToGoPredictor
        
        predictor = CostToGoPredictor()
        
        state_features = np.random.randn(256).astype(np.float32)
        action = np.array([0.0, 0.3, 0.0, 0.0], dtype=np.float32)
        
        outputs = predictor.predict_cost_to_go(state_features, [action])
        output = outputs[0]
        
        # Check cost breakdown
        assert output.safety_cost >= 0.0
        assert output.comfort_cost >= 0.0
        assert output.progress_cost >= 0.0
        assert output.legality_cost >= 0.0
    
    def test_personality_conditioning(self):
        """Test personality conditioning (chill vs sporty)"""
        from sunnypilot.modeld_v2.e2e_torque.cost_to_go import CostToGoPlanner
        
        planner = CostToGoPlanner()
        
        # Test chill personality
        planner.cost_predictor.set_personality('chill')
        assert planner.cost_predictor.current_weights.comfort_weight > 0.5
        
        # Test sporty personality
        planner.cost_predictor.set_personality('sporty')
        assert planner.cost_predictor.current_weights.comfort_weight < 0.5
        assert planner.cost_predictor.current_weights.progress_weight > 0.3
    
    def test_cost_gradient_computation(self):
        """Test cost gradient computation for optimization"""
        from sunnypilot.modeld_v2.e2e_torque.cost_to_go import CostToGoPredictor
        
        predictor = CostToGoPredictor()
        
        state_features = np.random.randn(256).astype(np.float32)
        action = np.array([0.0, 0.3, 0.0, 0.0], dtype=np.float32)
        
        gradient = predictor._compute_cost_gradient(state_features, action)
        
        assert gradient.shape == (4,)
        assert np.all(np.isfinite(gradient))


class TestLatencyOptimization:
    """Tests for Hardware Latency Optimization"""
    
    def test_shared_memory_optimizer(self):
        """Test shared memory buffer optimization"""
        from sunnypilot.modeld_v2.e2e_torque.latency_optimization import (
            SharedMemoryOptimizer,
        )
        
        optimizer = SharedMemoryOptimizer(
            num_buffers=4,
            buffer_size_mb=16,
            enable_zero_copy=True,
        )
        
        # Acquire buffer
        buffer_idx = optimizer.acquire_buffer(owner="cpu")
        assert buffer_idx is not None
        assert 0 <= buffer_idx < 4
        
        # Release buffer
        optimizer.release_buffer(buffer_idx)
        assert len(optimizer._available_buffers) == 4
    
    def test_fused_preprocessing_kernel(self):
        """Test fused preprocessing kernel"""
        from sunnypilot.modeld_v2.e2e_torque.latency_optimization import (
            FusedPreprocessingKernel,
        )
        
        kernel = FusedPreprocessingKernel(
            input_width=1928,
            input_height=1208,
            output_width=512,
            output_height=256,
        )
        
        # Create dummy buffers
        yuv_buffer = np.random.randn(1208, 1928, 3).astype(np.float32)
        output_buffer = np.zeros((256, 512, 3), dtype=np.float32)
        
        # Process frame
        processing_time = kernel.process_frame(yuv_buffer, output_buffer)
        
        assert processing_time > 0.0
        assert processing_time < 10.0  # Should be fast
    
    def test_async_pipeline_parallelism(self):
        """Test async pipeline parallelism"""
        from sunnypilot.modeld_v2.e2e_torque.latency_optimization import (
            AsyncPipelineParallelism,
        )
        
        pipeline = AsyncPipelineParallelism(
            num_stages=4,
            queue_size=4,
        )
        
        # Set dummy stage processors
        for i in range(4):
            pipeline.set_stage_processor(i, lambda x: x)
        
        # Submit frames
        for i in range(5):
            frame = np.random.randn(256, 512, 3).astype(np.float32)
            success = pipeline.submit_frame(frame, time.time())
            # May fail if pipeline full
        
        # Process pipeline
        output = pipeline.process_pipeline()
        
        # Check statistics
        stats = pipeline.get_stats()
        assert 'frames_processed' in stats
        assert 'throughput_fps' in stats
    
    def test_latency_monitor(self):
        """Test latency monitoring"""
        from sunnypilot.modeld_v2.e2e_torque.latency_optimization import LatencyMonitor
        
        monitor = LatencyMonitor(window_size=100)
        
        # Mark frame start
        monitor.mark_frame_start(0)
        
        # Mark stages
        monitor.mark_stage_complete(0, 'camera_capture', 2.0)
        monitor.mark_stage_complete(0, 'preprocessing', 1.0)
        monitor.mark_stage_complete(0, 'inference', 5.0)
        
        # Mark frame complete
        total_latency = monitor.mark_frame_complete(0)
        
        assert total_latency > 0.0
        
        # Get statistics
        stats = monitor.get_stats()
        assert stats.total_latency_ms > 0.0
        assert stats.fps > 0.0


class TestPerfectGradeIntegration:
    """Tests for integrated Perfect Grade E2E system"""
    
    def test_perfect_grade_controller_initialization(self):
        """Test Perfect Grade controller initializes all components"""
        from sunnypilot.modeld_v2.e2e_torque.perfect_grade_integration import (
            PerfectGradeE2EController,
        )
        
        controller = PerfectGradeE2EController(
            enable_unified_policy=True,
            enable_uncertainty=True,
            enable_temporal_buffer=True,
            enable_cost_to_go=True,
            enable_world_model=True,
            enable_mcts=True,
            enable_latency_opt=True,
        )
        
        assert controller.unified_policy is not None
        assert controller.uncertainty_controller is not None
        assert controller.temporal_buffer is not None
        assert controller.cost_planner is not None
        assert controller.world_model is not None
        assert controller.shared_memory_optimizer is not None
    
    def test_perfect_grade_frame_processing(self):
        """Test end-to-end frame processing"""
        from sunnypilot.modeld_v2.e2e_torque.perfect_grade_integration import (
            PerfectGradeE2EController,
        )
        
        controller = PerfectGradeE2EController(
            enable_unified_policy=True,
            enable_uncertainty=True,
            enable_temporal_buffer=True,
            enable_cost_to_go=True,
            enable_world_model=False,  # Disable for faster test
            enable_mcts=False,
            enable_latency_opt=False,
        )
        
        # Create dummy frame
        frame = np.random.randn(256, 512, 3).astype(np.float32)
        
        # Process frame
        output = controller.process_frame(
            frame,
            time.time(),
            vehicle_state={'v_ego': 10.0},
            context=None
        )
        
        # Verify outputs
        assert output.steering_torque is not None
        assert output.throttle_command is not None
        assert output.brake_command is not None
        assert output.uncertainty is not None
        assert output.latency_ms > 0.0
        assert output.fps > 0.0
    
    def test_personality_switching(self):
        """Test personality switching (replaces binary mode toggle)"""
        from sunnypilot.modeld_v2.e2e_torque.perfect_grade_integration import (
            PerfectGradeE2EController,
        )
        
        controller = PerfectGradeE2EController(
            enable_cost_to_go=True,
        )
        
        # Test switching personalities
        controller.set_personality('chill')
        assert controller._personality == 'chill'
        
        controller.set_personality('sporty')
        assert controller._personality == 'sporty'
        
        controller.set_personality('balanced')
        assert controller._personality == 'balanced'
    
    def test_statistics_reporting(self):
        """Test statistics reporting"""
        from sunnypilot.modeld_v2.e2e_torque.perfect_grade_integration import (
            PerfectGradeE2EController,
        )
        
        controller = PerfectGradeE2EController(
            enable_latency_opt=True,
        )
        
        stats = controller.get_stats()
        
        assert 'frames_processed' in stats
        assert 'avg_latency_ms' in stats
        assert 'avg_fps' in stats


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
