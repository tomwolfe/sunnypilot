"""
E2E Torque Module for sunnypilot
==================================

This module provides Pure E2E Torque Prediction, integrating:
1. Direct torque prediction from neural network
2. Multi-modal latent fusion (radar + map) 
3. World model for closed-loop simulation
4. Dynamic load-based delay adjustment

These four A+ enhancements work together to provide world-class E2E driving.
"""

from openpilot.sunnypilot.modeld_v2.e2e_torque.e2e_torque_predictor import (
    E2ETorquePredictor,
    E2ETorqueOutput,
    E2ETorqueSafety
)

from openpilot.sunnypilot.modeld_v2.e2e_torque.multimodal_latent_fusion import (
    MultiModalLatentFusion,
    RadarState,
    MapState,
    FusionOutput,
    LatentInjectionBuffer
)

from openpilot.sunnypilot.modeld_v2.e2e_torque.world_model import (
    WorldModel,
    WorldState,
    TrajectoryPrediction,
    SimulationResult,
    ImaginationBuffer
)

from openpilot.sunnypilot.modeld_v2.e2e_torque.dynamic_delay import (
    DynamicDelayPredictor,
    DelayPrediction,
    AdaptiveDelayFilter
)


class E2EController:
    """
    Unified E2E Controller
    
    Combines all four A+ enhancements into a single integrated controller:
    1. Pure E2E Torque Prediction
    2. Multi-Modal Latent Fusion  
    3. World Model Simulation
    4. Dynamic Delay Prediction
    """
    
    def __init__(self,
                 enable_torque: bool = True,
                 enable_fusion: bool = True,
                 enable_world_model: bool = True,
                 enable_dynamic_delay: bool = True):
        
        self.enable_torque = enable_torque
        self.enable_fusion = enable_fusion
        self.enable_world_model = enable_world_model
        self.enable_dynamic_delay = enable_dynamic_delay
        
        self.torque_predictor = E2ETorquePredictor() if enable_torque else None
        self.torque_safety = E2ETorqueSafety() if enable_torque else None
        
        self.fusion = MultiModalLatentFusion() if enable_fusion else None
        self.latent_buffer = LatentInjectionBuffer() if enable_fusion else None
        
        self.world_model = WorldModel() if enable_world_model else None
        self.imagination_buffer = ImaginationBuffer() if enable_world_model else None
        
        self.delay_predictor = DynamicDelayPredictor() if enable_dynamic_delay else None
        self.delay_filter = AdaptiveDelayFilter() if enable_dynamic_delay else None
        
    def update(self,
              model_outputs: dict,
              radar_state: dict = None,
              map_state: dict = None,
              vehicle_state: dict = None):
        """
        Update all components with new data
        
        Args:
            model_outputs: Raw neural network outputs
            radar_state: Radar sensor data
            map_state: Map navigation data
            vehicle_state: Current vehicle state (speed, steering, etc.)
        """
        if self.enable_fusion and radar_state and map_state:
            radar = self.fusion.process_radar_state(radar_state)
            map_data = self.fusion.process_map_state(map_state)
            self.latent_buffer.add_observation(radar, map_data)
            
        if self.enable_dynamic_delay and vehicle_state:
            self.delay_predictor.update(
                steering_angle=vehicle_state.get('steeringAngle', 0),
                curvature=vehicle_state.get('curvature', 0),
                lateral_accel=vehicle_state.get('lateralAccel', 0),
                speed=vehicle_state.get('speed', 0),
                timestamp=vehicle_state.get('timestamp', 0)
            )
            
    def get_fused_latent(self, vision_latent: np.ndarray) -> FusionOutput:
        """Get multi-modal fused latent representation"""
        if not self.enable_fusion or not self.fusion:
            return FusionOutput(fused_latent=vision_latent)
            
        radar = self.latent_buffer.get_recent_radar(1)[0] if self.latent_buffer else None
        map_data = self.latent_buffer.get_recent_map(1)[0] if self.latent_buffer else None
        
        return self.fusion.fuse(vision_latent, radar, map_data)
        
    def predict_torque(self, 
                       torque_output: np.ndarray,
                       uncertainty_output: np.ndarray = None) -> E2ETorqueOutput:
        """Predict torque from model outputs"""
        if not self.enable_torque or not self.torque_predictor:
            return E2ETorqueOutput(
                torque=0.0, torque_steering=0.0, torque_drive=0.0,
                uncertainty=1.0, confidence=0.0, is_valid=False
            )
            
        return self.torque_predictor.process_raw_output(
            torque_output, uncertainty_output
        )
        
    def validate_torque(self, torque: float, curvature: float = 0.0, 
                       speed: float = 0.0) -> tuple[bool, str]:
        """Validate torque for safety"""
        if not self.enable_torque or not self.torque_safety:
            return True, "ok"
            
        return self.torque_safety.validate_torque(torque, curvature, speed)
        
    def simulate_action(self,
                       current_state: WorldState,
                       proposed_actions: list,
                       context: dict = None) -> SimulationResult:
        """Run world model simulation for action"""
        if not self.enable_world_model or not self.world_model:
            return None
            
        return self.world_model.simulate(current_state, proposed_actions, context)
        
    def should_execute(self,
                      current_state: WorldState,
                      proposed_action: np.ndarray,
                      context: dict = None) -> tuple[bool, str]:
        """Determine if action should be executed"""
        if not self.enable_world_model or not self.world_model:
            return True, "ok"
            
        return self.world_model.should_execute_action(current_state, proposed_action, context)
        
    def predict_delay(self,
                      current_curvature: float,
                      speed: float,
                      desired_curvature: float = 0.0) -> DelayPrediction:
        """Predict effective steering delay"""
        if not self.enable_dynamic_delay or not self.delay_predictor:
            return DelayPrediction(
                base_delay=0.15,
                surface_delay=0.0,
                grip_delay=0.0,
                total_delay=0.15,
                confidence=0.5,
                surface_condition="unknown"
            )
            
        return self.delay_predictor.predict_delay(current_curvature, speed, desired_curvature)
        
    def get_adjusted_delay(self, base_delay: float, predicted_delay: float) -> float:
        """Get filtered delay value"""
        if not self.enable_dynamic_delay or not self.delay_filter:
            return base_delay
            
        return self.delay_filter.update(predicted_delay)
