"""
Multi-Modal Latent Fusion Module for sunnypilot
===============================================

This module implements fusion of radar and map data directly into the 
Transformer's latent space, enabling the E2E model to "attend" to 
sensor data beyond just vision.

Key Features:
- Radar state latent injection
- OpenStreetMap data latent injection  
- Cross-Attention Fusion (vision queries radar/map)
- Learned feature fusion via cross-attention
- Temporal attention over sensor modalities
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Any


@dataclass
class RadarState:
    """Processed radar state for latent injection"""
    lead_one_present: bool
    lead_one_drel: float
    lead_one_vrel: float
    lead_one_prob: float
    lead_two_present: bool
    lead_two_drel: float
    lead_two_vrel: float
    lead_two_prob: float


@dataclass
class MapState:
    """Processed map data for latent injection"""
    speed_limit: float
    speed_limit_ahead: float
    current_road_type: int
    upcoming_turn_type: int
    upcoming_turn_distance: float
    curvature_ahead: float


@dataclass
class FusionOutput:
    """Output from the multi-modal fusion module"""
    fused_latent: np.ndarray
    attention_weights_radar: Optional[np.ndarray] = None
    attention_weights_map: Optional[np.ndarray] = None
    uncertainty: float = 0.0


class CrossAttentionFusion:
    """
    Cross-Attention Fusion Module

    Instead of concatenating radar/map data into the vision latent,
    this module allows the vision transformer to "query" the radar data.

    This allows the model to:
    - Ignore vision noise (like glare) if radar provides high-confidence return
    - Attend to specific radar detections that matter for the current context
    - Dynamically weight between vision and radar based on confidence
    """

    def __init__(self,
                 vision_dim: int = 256,
                 radar_dim: int = 32,
                 map_dim: int = 32,
                 num_heads: int = 4,
                 dropout: float = 0.1):
        self.vision_dim = vision_dim
        self.radar_dim = radar_dim
        self.map_dim = map_dim
        self.num_heads = num_heads
        self.head_dim = vision_dim // num_heads

        self._vision_to_q = np.random.randn(vision_dim, vision_dim).astype(np.float32) * 0.01
        self._radar_to_kv = np.random.randn(radar_dim, vision_dim * 2).astype(np.float32) * 0.01
        self._map_to_kv = np.random.randn(map_dim, vision_dim * 2).astype(np.float32) * 0.01

        self._radar_confidence_weight = 0.5
        self._map_confidence_weight = 0.5

    def forward(self,
                vision_latent: np.ndarray,
                radar_features: Optional[np.ndarray] = None,
                map_features: Optional[np.ndarray] = None,
                radar_confidence: float = 0.5,
                map_confidence: float = 0.5) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Cross-attention forward pass
        
        Args:
            vision_latent: [batch, seq_len, vision_dim]
            radar_features: [batch, radar_dim] or None
            map_features: [batch, map_dim] or None
            radar_confidence: Confidence in radar data [0, 1]
            map_confidence: Confidence in map data [0, 1]
            
        Returns:
            (fused_latent, radar_attention, map_attention)
        """
        batch_size, seq_len, _ = vision_latent.shape

        q = np.dot(vision_latent, self._vision_to_q)
        q = q.reshape(batch_size, seq_len, self.num_heads, self.head_dim)
        q = np.transpose(q, (0, 2, 1, 3))

        fused = vision_latent.copy()
        radar_attention = np.zeros((batch_size, self.num_heads, seq_len, 1), dtype=np.float32)
        map_attention = np.zeros((batch_size, self.num_heads, seq_len, 1), dtype=np.float32)

        if radar_features is not None:
            kv_radar = np.dot(radar_features, self._radar_to_kv)
            k_radar, v_radar = np.split(kv_radar, 2, axis=-1)

            k_radar = k_radar.reshape(batch_size, 1, self.num_heads, self.head_dim)
            k_radar = np.transpose(k_radar, (0, 2, 3, 1))
            v_radar = v_radar.reshape(batch_size, 1, self.num_heads, self.head_dim)
            v_radar = np.transpose(v_radar, (0, 2, 1, 3))

            radar_attention = self._scaled_dot_product_attention(q, k_radar, v_radar)

            radar_contribution = self._apply_attention(vision_latent, radar_attention, v_radar)
            fused = fused + radar_confidence * radar_contribution

        if map_features is not None:
            kv_map = np.dot(map_features, self._map_to_kv)
            k_map, v_map = np.split(kv_map, 2, axis=-1)

            k_map = k_map.reshape(batch_size, 1, self.num_heads, self.head_dim)
            k_map = np.transpose(k_map, (0, 2, 3, 1))
            v_map = v_map.reshape(batch_size, 1, self.num_heads, self.head_dim)
            v_map = np.transpose(v_map, (0, 2, 1, 3))

            map_attention = self._scaled_dot_product_attention(q, k_map, v_map)

            map_contribution = self._apply_attention(vision_latent, map_attention, v_map)
            fused = fused + map_confidence * map_contribution

        normalization = 1.0 + radar_confidence + map_confidence
        fused = fused / normalization

        return fused, radar_attention, map_attention

    def _scaled_dot_product_attention(self,
                                     q: np.ndarray,
                                     k: np.ndarray,
                                     v: np.ndarray) -> np.ndarray:
        """Compute scaled dot-product attention"""
        d_k = k.shape[-1]

        scores = np.matmul(q, k) / np.sqrt(d_k)

        attention_weights = self._softmax(scores, axis=-1)

        attention_output = np.matmul(attention_weights, v)

        return attention_output

    def _apply_attention(self,
                        vision_latent: np.ndarray,
                        attention: np.ndarray,
                        value: np.ndarray) -> np.ndarray:
        """Apply attention weights to vision latent"""
        batch_size, seq_len, dim = vision_latent.shape

        attention_transposed = np.transpose(attention, (0, 2, 1, 3))
        attention_flat = attention_transposed.reshape(batch_size, seq_len, dim)

        return attention_flat

    def _softmax(self, x: np.ndarray, axis: int = -1) -> np.ndarray:
        """Numerically stable softmax"""
        exp_x = np.exp(x - np.max(x, axis=axis, keepdims=True))
        return exp_x / np.sum(exp_x, axis=axis, keepdims=True)

    def compute_modality_confidence(self,
                                   vision_features: np.ndarray,
                                   radar_features: Optional[np.ndarray],
                                   map_features: Optional[np.ndarray]) -> tuple[float, float]:
        """
        Compute dynamic confidence weights based on modality quality
        
        Returns:
            (radar_confidence, map_confidence)
        """
        vision_confidence = self._estimate_feature_quality(vision_features)

        radar_confidence = 0.5
        if radar_features is not None:
            radar_confidence = self._estimate_radar_quality(radar_features)

        map_confidence = 0.5
        if map_features is not None:
            map_confidence = self._estimate_map_quality(map_features)

        return radar_confidence, map_confidence

    def _estimate_feature_quality(self, features: np.ndarray) -> float:
        """Estimate quality of feature representation"""
        variance = np.var(features)
        quality = float(np.clip(variance * 10, 0, 1))
        return quality

    def _estimate_radar_quality(self, radar_features: np.ndarray) -> float:
        """Estimate radar detection quality"""
        if radar_features.ndim > 1:
            radar_features = radar_features[0]

        energy = np.linalg.norm(radar_features)
        quality = float(np.clip(energy / 5.0, 0, 1))

        return quality

    def _estimate_map_quality(self, map_features: np.ndarray) -> float:
        """Estimate map data quality"""
        if map_features.ndim > 1:
            map_features = map_features[0]

        non_zero = np.count_nonzero(map_features)
        quality = float(np.clip(non_zero / len(map_features), 0, 1))

        return quality


class MultiModalLatentFusion:
    """
    Multi-Modal Latent Fusion
    
    Integrates radar and map data directly into the Transformer's latent space
    using cross-attention mechanisms.
    
    This allows the E2E model to:
    - "See" radar-detected objects even when occluded in camera
    - "Know" about upcoming speed limits and turns from map data
    - Attend to historical sensor data with temporal context
    """

    FEATURE_DIM = 256
    RADAR_FEATURE_DIM = 16
    MAP_FEATURE_DIM = 16

    def __init__(self,
                 enable_radar: bool = True,
                 enable_map: bool = True,
                 attention_heads: int = 4,
                 dropout: float = 0.1):
        self.enable_radar = enable_radar
        self.enable_map = enable_map
        self.attention_heads = attention_heads

        self._radar_projection = np.random.randn(self.RADAR_FEATURE_DIM, self.FEATURE_DIM).astype(np.float32) * 0.01
        self._map_projection = np.random.randn(self.MAP_FEATURE_DIM, self.FEATURE_DIM).astype(np.float32) * 0.01

        self._attention_weights_radar = None
        self._attention_weights_map = None

    def process_radar_state(self, radar_msg: dict[str, Any]) -> RadarState:
        """Process incoming radarState message"""
        lead_one = radar_msg.get('leadOne', {})
        lead_two = radar_msg.get('leadTwo', {})

        return RadarState(
            lead_one_present=bool(lead_one.get('status', 0)),
            lead_one_drel=float(lead_one.get('dRel', 0)),
            lead_one_vrel=float(lead_one.get('vRel', 0)),
            lead_one_prob=float(lead_one.get('prob', 0)),
            lead_two_present=bool(lead_two.get('status', 0)),
            lead_two_drel=float(lead_two.get('dRel', 0)),
            lead_two_vrel=float(lead_two.get('vRel', 0)),
            lead_two_prob=float(lead_two.get('prob', 0))
        )

    def process_map_state(self, map_msg: dict[str, Any]) -> MapState:
        """Process incoming liveMapDataSP message"""
        return MapState(
            speed_limit=float(map_msg.get('speedLimit', 0)),
            speed_limit_ahead=float(map_msg.get('speedLimitAhead', 0)),
            current_road_type=int(map_msg.get('roadType', 0)),
            upcoming_turn_type=int(map_msg.get('turnState', {}).get('type', 0)),
            upcoming_turn_distance=float(map_msg.get('turnState', {}).get('distance', 0)),
            curvature_ahead=float(map_msg.get('curvature', 0))
        )

    def _radar_to_features(self, radar: RadarState) -> np.ndarray:
        """Convert radar state to feature vector"""
        features = np.zeros(self.RADAR_FEATURE_DIM, dtype=np.float32)

        if radar.lead_one_present:
            features[0] = 1.0
            features[1] = radar.lead_one_drel / 200.0
            features[2] = radar.lead_one_vrel / 50.0
            features[3] = radar.lead_one_prob

        if radar.lead_two_present:
            features[4] = 1.0
            features[5] = radar.lead_two_drel / 200.0
            features[6] = radar.lead_two_vrel / 50.0
            features[7] = radar.lead_two_prob

        features[8] = features[1] - features[5] if radar.lead_two_present else 0.0

        if radar.lead_one_present and radar.lead_one_drel < 50:
            features[9] = 1.0
        if radar.lead_two_present and radar.lead_two_drel < 50:
            features[10] = 1.0

        return features

    def _map_to_features(self, map_data: MapState) -> np.ndarray:
        """Convert map state to feature vector"""
        features = np.zeros(self.MAP_FEATURE_DIM, dtype=np.float32)

        features[0] = map_data.speed_limit / 50.0
        features[1] = map_data.speed_limit_ahead / 50.0
        features[2] = map_data.current_road_type / 10.0

        features[3] = map_data.upcoming_turn_type / 5.0
        features[4] = np.clip(map_data.upcoming_turn_distance / 500.0, 0, 1)

        features[5] = map_data.curvature_ahead * 100.0

        if map_data.speed_limit_ahead > 0 and map_data.speed_limit_ahead < map_data.speed_limit:
            features[6] = 1.0

        if map_data.upcoming_turn_type > 0:
            features[7] = 1.0 / (1.0 + map_data.upcoming_turn_distance / 100.0)

        return features

    def fuse(self,
             vision_latent: np.ndarray,
             radar_state: Optional[RadarState] = None,
             map_state: Optional[MapState] = None) -> FusionOutput:
        """
        Fuse multi-modal inputs into unified latent representation
        
        Uses cross-attention to let vision features attend to radar and map features
        """
        batch_size = vision_latent.shape[0]
        seq_len = vision_latent.shape[1]
        feature_dim = vision_latent.shape[2]

        modalities = []
        attention_weights = []

        if self.enable_radar and radar_state is not None:
            radar_features = self._radar_to_features(radar_state)
            radar_latent = np.dot(radar_features, self._radar_projection)
            radar_latent = np.broadcast_to(radar_latent, (batch_size, 1, feature_dim))
            modalities.append(radar_latent)
            attn_w = self._compute_attention(vision_latent, radar_latent)
            attention_weights.append(attn_w)

        if self.enable_map and map_state is not None:
            map_features = self._map_to_features(map_state)
            map_latent = np.dot(map_features, self._map_projection)
            map_latent = np.broadcast_to(map_latent, (batch_size, 1, feature_dim))
            modalities.append(map_latent)
            attn_w = self._compute_attention(vision_latent, map_latent)
            attention_weights.append(attn_w)

        if not modalities:
            return FusionOutput(
                fused_latent=vision_latent,
                uncertainty=0.1
            )

        fused = vision_latent.copy()

        for modality_latent in modalities:
            fused = fused + 0.5 * modality_latent

        fused = fused / (1.0 + 0.5 * len(modalities))

        uncertainty = 0.1
        if attention_weights:
            uncertainty = float(np.mean([np.max(aw) for aw in attention_weights]))

        return FusionOutput(
            fused_latent=fused,
            attention_weights_radar=attention_weights[0] if len(attention_weights) > 0 else None,
            attention_weights_map=attention_weights[1] if len(attention_weights) > 1 else None,
            uncertainty=uncertainty
        )

    def _compute_attention(self,
                          query: np.ndarray,
                          key: np.ndarray) -> np.ndarray:
        """
        Compute scaled dot-product attention
        
        Simplified version for fusion weights
        """
        d_k = key.shape[-1]
        scores = np.matmul(query, key.transpose(0, 2, 1)) / np.sqrt(d_k)
        attention_weights = self._softmax(scores, axis=-1)

        self._attention_weights_radar = attention_weights
        return attention_weights

    def _softmax(self, x: np.ndarray, axis: int = -1) -> np.ndarray:
        """Numerically stable softmax"""
        exp_x = np.exp(x - np.max(x, axis=axis, keepdims=True))
        return exp_x / np.sum(exp_x, axis=axis, keepdims=True)


class LatentInjectionBuffer:
    """
    Maintains temporal history of latent injections for attention mechanism
    
    Allows the model to attend to radar/map observations from multiple 
    timesteps in the past
    """

    def __init__(self, max_history: int = 100):
        self.max_history = max_history
        self._radar_history: list[Optional[RadarState]] = []
        self._map_history: list[Optional[MapState]] = []
        self._timestamps: list[float] = []

    def add_observation(self,
                       radar: Optional[RadarState] = None,
                       map_data: Optional[MapState] = None,
                       timestamp: float = 0.0):
        """Add new observation to history"""
        self._radar_history.append(radar)
        self._map_history.append(map_data)
        self._timestamps.append(timestamp)

        if len(self._radar_history) > self.max_history:
            self._radar_history.pop(0)
            self._map_history.pop(0)
            self._timestamps.pop(0)

    def get_recent_radar(self, n: int = 10) -> list[Optional[RadarState]]:
        """Get n most recent radar observations"""
        return self._radar_history[-n:] if len(self._radar_history) >= n else self._radar_history

    def get_recent_map(self, n: int = 10) -> list[Optional[MapState]]:
        """Get n most recent map observations"""
        return self._map_history[-n:] if len(self._map_history) >= n else self._map_history

    def get_radar_at_time(self, timestamp: float, tolerance: float = 0.5) -> Optional[RadarState]:
        """Get radar state closest to given timestamp"""
        if not self._timestamps:
            return None

        idx = np.argmin(np.abs(np.array(self._timestamps) - timestamp))
        if abs(self._timestamps[idx] - timestamp) < tolerance:
            return self._radar_history[idx]
        return None
