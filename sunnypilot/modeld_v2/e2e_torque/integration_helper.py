"""
E2E A+ Integration Helper
===========================

This module provides integration between the four A+ enhancement modules
and the existing sunnypilot modeld infrastructure.

Usage:
------
1. Pure E2E Torque: Use E2ETorquePredictor in latcontrol_neural.py
2. Multi-Modal Fusion: Use MultiModalLatentFusion in modeld.py
3. World Model: Use WorldModel for action validation in plannerd.py
4. Dynamic Delay: Use DynamicDelayPredictor in drive_helpers.py

Example Integration:
-------------------

# In modeld.py - Add radar/map fusion:
from openpilot.sunnypilot.modeld_v2.e2e_torque import (
    E2EController, MultiModalLatentFusion, LatentInjectionBuffer
)

# Initialize
fusion = MultiModalLatentFusion()
latent_buffer = LatentInjectionBuffer()

# In model loop:
radar = fusion.process_radar_state(sm['radarState'])
map_data = fusion.process_map_state(sm['liveMapDataSP'])
latent_buffer.add_observation(radar, map_data)
fused_result = fusion.fuse(vision_features, radar, map_data)

# Use fused_result.fused_latent as input to policy model


# In latcontrol_neural.py - Add pure E2E torque:
from openpilot.sunnypilot.modeld_v2.e2e_torque import E2ETorquePredictor

torque_predictor = E2ETorquePredictor()

# In update():
torque_output = torque_predictor.process_raw_output(
    model_output.get('torque'),
    model_output.get('torque_uncertainty')
)

# Blend with fallback
final_torque = torque_predictor.blend_with_fallback(
    torque_output.torque,
    fallback_torque,
    torque_output.confidence
)


# In drive_helpers.py - Use dynamic delay:
from openpilot.sunnypilot.modeld_v2.e2e_torque import DynamicDelayPredictor

delay_predictor = DynamicDelayPredictor()

# In update:
delay_predictor.update(steering_angle, curvature, lateral_accel, speed, timestamp)
delay_pred = delay_predictor.predict_delay(curvature, speed, desired_curvature)

# Use delay_pred.total_delay in get_lag_adjusted_curvature


# In plannerd.py - Add world model simulation:
from openpilot.sunnypilot.modeld_v2.e2e_torque import WorldModel, WorldState

world_model = WorldModel()

# Before executing action:
current_state = WorldState(
    position=np.array([x, y, z]),
    velocity=np.array([vx, vy, vz]),
    acceleration=np.array([ax, ay, az]),
    heading=heading,
    lane_position=lane_offset,
    objects=detected_objects
)

proposed_actions = [action1, action2, action3]
result = world_model.simulate(current_state, proposed_actions, context)

if result.is_safe:
    execute_action(result.chosen_trajectory_idx)
else:
    use_fallback_planning()
"""

from openpilot.sunnypilot.modeld_v2.e2e_torque import (
    E2EController,
    E2ETorquePredictor,
    E2ETorqueOutput,
    E2ETorqueSafety,
    MultiModalLatentFusion,
    RadarState,
    MapState,
    FusionOutput,
    LatentInjectionBuffer,
    WorldModel,
    WorldState,
    TrajectoryPrediction,
    SimulationResult,
    ImaginationBuffer,
    DynamicDelayPredictor,
    DelayPrediction,
    AdaptiveDelayFilter,
    # MCTS Planning (Recommendation #1)
    ContinuousMCTSPlanner,
    MCTSIntegrationHelper,
    MCTSNode,
    MCTSResult,
    # Quantization (Recommendation #2)
    INT8Quantizer,
    ModelQuantizer,
    QuantizedLayer,
    QuantizationConfig,
    # Temporal Memory (Recommendation #3)
    TemporalMemoryModule,
    TransformerXLMemory,
    StateSpaceMemory,
    # Neural Observer (Recommendation #4)
    NeuralObserver,
    NeuralDelayObserver,
    VehicleDynamicsObserver,
    # Disengagement Analysis (Recommendation #5)
    DisengagementAnalyzer,
    DisengagementEvent,
    TelemetryRecorder
)


__all__ = [
    'E2EController',
    'E2ETorquePredictor',
    'E2ETorqueOutput',
    'E2ETorqueSafety',
    'MultiModalLatentFusion',
    'RadarState',
    'MapState',
    'FusionOutput',
    'LatentInjectionBuffer',
    'WorldModel',
    'WorldState',
    'TrajectoryPrediction',
    'SimulationResult',
    'ImaginationBuffer',
    'DynamicDelayPredictor',
    'DelayPrediction',
    'AdaptiveDelayFilter',
    # MCTS Planning (Recommendation #1)
    'ContinuousMCTSPlanner',
    'MCTSIntegrationHelper',
    'MCTSNode',
    'MCTSResult',
    # Quantization (Recommendation #2)
    'INT8Quantizer',
    'ModelQuantizer',
    'QuantizedLayer',
    'QuantizationConfig',
    # Temporal Memory (Recommendation #3)
    'TemporalMemoryModule',
    'TransformerXLMemory',
    'StateSpaceMemory',
    # Neural Observer (Recommendation #4)
    'NeuralObserver',
    'NeuralDelayObserver',
    'VehicleDynamicsObserver',
    # Disengagement Analysis (Recommendation #5)
    'DisengagementAnalyzer',
    'DisengagementEvent',
    'TelemetryRecorder',
]
