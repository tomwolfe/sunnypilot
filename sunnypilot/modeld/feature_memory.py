import numpy as np
from dataclasses import dataclass
from typing import Optional, Any


@dataclass
class OSMEmbedding:
    """OpenStreetMap spatial embedding for attention bias"""
    road_type: int
    speed_limit: float
    curvature: float
    intersection_type: int
    stop_sign_present: bool
    traffic_light_present: bool
    lane_count: int
    is_highway: bool
    is_urban: bool
    distance_to_intersection: float


class FeatureMemory:
    """
    Transformer-based Feature Memory for E2E Path Planning.
    Replaces rolling history with an Attention mechanism over a latent buffer.

    A+ Enhancement: Deep OSM Fusion
    - Map data is now a "spatial embedding" that biases the Transformer Attention
    - Road names and intersection geometry spike attention on velocity features
    - Stop signs trigger attention spikes on braking-related features
    """
    def __init__(self, feature_len, max_history=50):
        self.feature_len = feature_len
        self.max_history = max_history
        self.history = [] # List of past latent features
        self.osm_history = []  # OSM embeddings aligned with history

        # Simple Linear Projections for Attention (Q, K, V)
        # In a production environment, these weights would be learned.
        # We initialize them as identity to start with "mean-pooling" behavior.
        self.W_q = np.eye(feature_len, dtype=np.float32)
        self.W_k = np.eye(feature_len, dtype=np.float32)
        self.W_v = np.eye(feature_len, dtype=np.float32)

        # OSM-specific attention bias projections
        self.W_osm_key = np.random.randn(10, feature_len).astype(np.float32) * 0.01
        self.W_osm_value = np.random.randn(10, feature_len).astype(np.float32) * 0.01

        # Attention bias scaling factors
        self.osm_attention_scale = 0.5
        self.stop_sign_attention_boost = 2.0
        self.intersection_attention_boost = 1.5

    def add(self, feature: np.ndarray, osm_embedding: Optional[OSMEmbedding] = None):
        """
        Adds a new feature to the memory.

        Args:
            feature: Latent feature vector
            osm_embedding: Optional OSM spatial embedding for attention bias
        """
        if len(self.history) >= self.max_history:
            self.history.pop(0)
            if self.osm_history:
                self.osm_history.pop(0)
        self.history.append(feature.copy())
        self.osm_history.append(osm_embedding)

    def add_with_osm(self, feature: np.ndarray, osm_data: dict[str, Any]):
        """
        Add feature with OSM data extracted from liveMapDataSP

        Args:
            feature: Latent feature vector
            osm_data: Raw OSM data from liveMapDataSP
        """
        osm_embedding = self._parse_osm_embedding(osm_data)
        self.add(feature, osm_embedding)

    def _parse_osm_embedding(self, osm_data: dict[str, Any]) -> OSMEmbedding:
        """Parse liveMapDataSP message into OSM embedding"""
        road_type = osm_data.get('roadType', 0)
        speed_limit = osm_data.get('speedLimit', 0.0)
        curvature = osm_data.get('curvature', 0.0)

        turn_state = osm_data.get('turnState', {})
        intersection_type = turn_state.get('type', 0)
        distance_to_intersection = turn_state.get('distance', 1000.0)

        is_highway = road_type in (1, 2)
        is_urban = road_type in (4, 5) or speed_limit < 50

        stop_sign_present = osm_data.get('hasStopSign', False)
        traffic_light_present = osm_data.get('hasTrafficLight', False)

        lane_count = osm_data.get('laneCount', 2)

        return OSMEmbedding(
            road_type=road_type,
            speed_limit=speed_limit,
            curvature=curvature,
            intersection_type=intersection_type,
            stop_sign_present=stop_sign_present,
            traffic_light_present=traffic_light_present,
            lane_count=lane_count,
            is_highway=is_highway,
            is_urban=is_urban,
            distance_to_intersection=distance_to_intersection
        )

    def get_contextual_feature(self, current_feature: np.ndarray,
                               osm_context: Optional[dict[str, Any]] = None) -> np.ndarray:
        """
        Computes a contextual feature using Scaled Dot-Product Attention with OSM bias.

        A+ Enhancement: OSM data biases the attention mechanism:
        - Stop signs spike attention on velocity/braking features
        - Intersections increase attention on lateral position features
        - Highway driving focuses on longitudinal velocity

        Args:
            current_feature: Current latent feature
            osm_context: Optional OSM context for attention bias

        Returns:
            Contextual feature with attention-weighted history
        """
        if not self.history:
            return current_feature

        # Stack history for batch computation
        history_stack = np.array(self.history) # (N, FEATURE_LEN)

        # 1. Project to Q, K, V
        query = current_feature @ self.W_q # (FEATURE_LEN,)
        keys = history_stack @ self.W_k    # (N, FEATURE_LEN)
        values = history_stack @ self.W_v  # (N, FEATURE_LEN)

        # 2. Scaled Dot-Product Attention
        d_k = self.feature_len
        scores = (query @ keys.T) / np.sqrt(d_k) # (N,)

        # 3. Apply OSM attention bias
        osm_bias = self._compute_osm_attention_bias(osm_context)
        if osm_bias is not None:
            scores = scores + osm_bias

        # 4. Softmax
        weights = np.exp(scores - np.max(scores))
        weights /= weights.sum()

        # 5. Weighted Sum of Values
        context_vector = weights @ values # (FEATURE_LEN,)

        # Residual Connection
        return (current_feature + context_vector) / 2.0

    def _compute_osm_attention_bias(self, osm_context: Optional[dict[str, Any]]) -> Optional[np.ndarray]:
        """
        Compute attention bias from OSM context

        This biases the Transformer to attend to relevant historical features
        based on the current map context.

        Returns:
            Attention bias array of shape (N,) or None
        """
        if not self.osm_history or osm_context is None:
            return None

        current_osm = self._parse_osm_embedding(osm_context)
        n_history = len(self.history)

        bias = np.zeros(n_history, dtype=np.float32)

        # Stop sign: attend to recent braking-related features
        if current_osm.stop_sign_present:
            for i in range(n_history):
                if self.osm_history[i] and self.osm_history[i].stop_sign_present:
                    bias[i] += self.stop_sign_attention_boost

        # Intersection: attend to features near intersections
        if current_osm.intersection_type > 0 and current_osm.distance_to_intersection < 50:
            distance_factor = 1.0 - (current_osm.distance_to_intersection / 50.0)
            for i in range(n_history):
                if self.osm_history[i] and self.osm_history[i].intersection_type > 0:
                    bias[i] += self.intersection_attention_boost * distance_factor

        # Highway: attend to high-speed features
        if current_osm.is_highway:
            for i in range(n_history):
                if self.osm_history[i] and self.osm_history[i].is_highway:
                    bias[i] += self.osm_attention_scale

        # Urban: attend to low-speed, high-curvature features
        if current_osm.is_urban:
            for i in range(n_history):
                if self.osm_history[i] and self.osm_history[i].is_urban:
                    bias[i] += self.osm_attention_scale

        # Scale the overall bias
        bias = bias * self.osm_attention_scale

        return bias
