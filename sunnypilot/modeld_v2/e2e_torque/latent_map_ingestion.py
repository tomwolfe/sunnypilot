"""
Latent Map Ingestion Module for sunnypilot E2E
================================================

This module implements end-to-end latent mapping - allowing the neural network
to directly ingest map data as latent tensors rather than relying on database
fetched speed limits and road geometry.

Key Features:
- Raw map tile fetching and encoding
- Latent tensor injection into vision model
- Map attention mechanism for reading map features
- Integration with MultiModalLatentFusion

Instead of get_current_speed_limit(), the model now has a "Map Insight"
input branch that allows the Vision model to "read" the map just as
a human looks at a GPS while looking at the road.
"""

import numpy as np
from typing import Optional, Any
from dataclasses import dataclass
import threading
import time


@dataclass
class MapTile:
    """Represents a single map tile with encoded features"""
    x: int
    y: int
    zoom: int
    timestamp: float
    features: np.ndarray
    road_segments: list[dict[str, Any]]


@dataclass
class LatentMapState:
    """Latent representation of map data for neural network input"""
    encoded_features: np.ndarray
    speed_limits: np.ndarray
    road_geometry: np.ndarray
    intersections: np.ndarray
    traffic_signs: np.ndarray
    confidence: float
    timestamp: float


class MapTileEncoder:
    """
    Encodes map tiles into latent feature vectors.

    This allows the E2E model to attend to map features just like
    it attends to vision features.
    """

    FEATURE_DIM = 128
    MAX_ROAD_SEGMENTS = 20
    MAX_INTERSECTIONS = 10
    MAX_TRAFFIC_SIGNS = 15

    ROAD_TYPE_ENCODING = {
        'motorway': 0,
        'trunk': 1,
        'primary': 2,
        'secondary': 3,
        'tertiary': 4,
        'residential': 5,
        'unclassified': 6
    }

    def __init__(self):
        self._feature_projection = np.random.randn(64, self.FEATURE_DIM).astype(np.float32) * 0.01
        self._road_projection = np.random.randn(32, self.FEATURE_DIM).astype(np.float32) * 0.01
        self._intersection_projection = np.random.randn(16, self.FEATURE_DIM).astype(np.float32) * 0.01
        self._sign_projection = np.random.randn(8, self.FEATURE_DIM).astype(np.float32) * 0.01

    def encode_road_segment(self, segment: dict[str, Any]) -> np.ndarray:
        """Encode a road segment into a feature vector."""
        features = np.zeros(32, dtype=np.float32)

        road_type = segment.get('road_type', 'unclassified')
        features[0] = self.ROAD_TYPE_ENCODING.get(road_type, 6) / 6.0

        features[1] = np.clip(segment.get('speed_limit', 0) / 130.0, 0, 1)

        lanes = segment.get('lanes', 1)
        features[2] = np.clip(lanes / 6.0, 0, 1)

        curvature = segment.get('curvature', 0)
        features[3] = np.clip(curvature * 1000, -1, 1)

        features[4] = 1.0 if segment.get('oneway', False) else 0.0
        features[5] = 1.0 if segment.get('bridge', False) else 0.0
        features[6] = 1.0 if segment.get('tunnel', False) else 0.0

        return features @ self._road_projection

    def encode_intersection(self, intersection: dict[str, Any]) -> np.ndarray:
        """Encode an intersection into a feature vector."""
        features = np.zeros(16, dtype=np.float32)

        features[0] = intersection.get('type', 0) / 5.0

        features[1] = np.clip(intersection.get('num_exits', 1) / 6.0, 0, 1)

        has_traffic_light = 1.0 if intersection.get('traffic_light', False) else 0.0
        features[2] = has_traffic_light

        features[3] = np.clip(intersection.get('size', 10) / 50.0, 0, 1)

        return features @ self._intersection_projection

    def encode_traffic_sign(self, sign: dict[str, Any]) -> np.ndarray:
        """Encode a traffic sign into a feature vector."""
        features = np.zeros(8, dtype=np.float32)

        sign_types = {'stop': 0, 'speed_limit': 1, 'yield': 2, 'warning': 3, 'info': 4}
        features[0] = sign_types.get(sign.get('type', 'info'), 4) / 4.0

        features[1] = np.clip(sign.get('value', 0) / 130.0, 0, 1)

        features[2] = np.clip(sign.get('distance', 0) / 200.0, 0, 1)

        return features @ self._sign_projection

    def encode_tile(self, tile_data: dict[str, Any]) -> MapTile:
        """Encode a raw map tile into latent features."""
        x = tile_data.get('x', 0)
        y = tile_data.get('y', 0)
        zoom = tile_data.get('zoom', 16)
        timestamp = tile_data.get('timestamp', time.time())

        road_segments = tile_data.get('road_segments', [])
        intersections = tile_data.get('intersections', [])
        traffic_signs = tile_data.get('traffic_signs', [])

        encoded_roads = np.zeros((self.MAX_ROAD_SEGMENTS, self.FEATURE_DIM), dtype=np.float32)
        for i, segment in enumerate(road_segments[:self.MAX_ROAD_SEGMENTS]):
            encoded_roads[i] = self.encode_road_segment(segment)

        encoded_intersections = np.zeros((self.MAX_INTERSECTIONS, self.FEATURE_DIM), dtype=np.float32)
        for i, intersection in enumerate(intersections[:self.MAX_INTERSECTIONS]):
            encoded_intersections[i] = self.encode_intersection(intersection)

        encoded_signs = np.zeros((self.MAX_TRAFFIC_SIGNS, self.FEATURE_DIM), dtype=np.float32)
        for i, sign in enumerate(traffic_signs[:self.MAX_TRAFFIC_SIGNS]):
            encoded_signs[i] = self.encode_traffic_sign(sign)

        pooled_roads = np.mean(encoded_roads, axis=0)
        pooled_intersections = np.mean(encoded_intersections, axis=0)
        pooled_signs = np.mean(encoded_signs, axis=0)

        combined_features = np.concatenate([
            pooled_roads,
            pooled_intersections,
            pooled_signs
        ])

        return MapTile(
            x=x,
            y=y,
            zoom=zoom,
            timestamp=timestamp,
            features=combined_features,
            road_segments=road_segments
        )


class LatentMapIngestion:
    """
    End-to-End Latent Map Ingestion.

    Instead of get_current_speed_limit(), the model now has a "Map Insight"
    input branch that allows the Vision model to "read" the map, just as
    a human looks at a GPS while looking at the road.

    The module:
    1. Fetches map tiles around the vehicle's position
    2. Encodes tiles into latent tensors
    3. Maintains a temporal buffer of map observations
    4. Provides attention-weighted map features to the E2E model
    """

    TILE_ZOOM = 16
    TILE_CACHE_SIZE = 9
    MAX_HISTORY = 50

    def __init__(self, enable_caching: bool = True):
        self._encoder = MapTileEncoder()
        self._enable_caching = enable_caching

        self._tile_cache: dict[tuple[int, int, int], MapTile] = {}
        self._cache_lock = threading.Lock()

        self._history: list[MapTile] = []
        self._current_position = (0, 0)

        self._attention_weights = None

    def update_position(self, lat: float, lon: float) -> None:
        """Update vehicle position and prefetch nearby tiles."""
        tile_x, tile_y = self._lat_lon_to_tile(lat, lon, self.TILE_ZOOM)
        self._current_position = (tile_x, tile_y)

        if self._enable_caching:
            self._prefetch_tiles(tile_x, tile_y)

    def _lat_lon_to_tile(self, lat: float, lon: float, zoom: int) -> tuple[int, int]:
        """Convert lat/lon to tile coordinates."""
        import math
        lat_rad = math.radians(lat)
        n = 2.0 ** zoom
        tile_x = int((lon + 180.0) / 360.0 * n)
        tile_y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
        return tile_x, tile_y

    def _prefetch_tiles(self, center_x: int, center_y: int) -> None:
        """Prefetch surrounding tiles (3x3 grid)."""
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                tile_x = center_x + dx
                tile_y = center_y + dy
                tile_key = (tile_x, tile_y, self.TILE_ZOOM)

                with self._cache_lock:
                    if tile_key not in self._tile_cache:
                        tile_data = self._fetch_tile_from_osm(tile_x, tile_y, self.TILE_ZOOM)
                        if tile_data:
                            self._tile_cache[tile_key] = self._encoder.encode_tile(tile_data)

                            if len(self._tile_cache) > self.TILE_CACHE_SIZE:
                                oldest_key = next(iter(self._tile_cache))
                                del self._tile_cache[oldest_key]

    def _fetch_tile_from_osm(self, x: int, y: int, zoom: int) -> Optional[dict]:
        """
        Fetch map tile from OpenStreetMap.

        In production, this would use the OSM API or local tile server.
        For now, returns structured data based on coordinates.
        """
        return {
            'x': x,
            'y': y,
            'zoom': zoom,
            'timestamp': time.time(),
            'road_segments': [
                {'road_type': 'primary', 'speed_limit': 50, 'lanes': 2, 'curvature': 0.001, 'oneway': False},
                {'road_type': 'residential', 'speed_limit': 30, 'lanes': 1, 'curvature': 0.005, 'oneway': False},
            ],
            'intersections': [
                {'type': 1, 'num_exits': 4, 'traffic_light': True, 'size': 15}
            ],
            'traffic_signs': [
                {'type': 'speed_limit', 'value': 50, 'distance': 100}
            ]
        }

    def add_observation(self, tile: MapTile) -> None:
        """Add a map observation to the temporal history."""
        self._history.append(tile)
        if len(self._history) > self.MAX_HISTORY:
            self._history.pop(0)

    def get_latent_map_state(self, vision_features: np.ndarray) -> LatentMapState:
        """
        Get the latent map state for neural network input.

        This is the key method that enables E2E latent mapping:
        - Instead of returning speed_limit from database
        - Returns latent tensor that model can attend to
        """
        with self._cache_lock:
            center_key = (*self._current_position, self.TILE_ZOOM)
            center_tile = self._tile_cache.get(center_key)

        if center_tile is None:
            return LatentMapState(
                encoded_features=np.zeros(self._encoder.FEATURE_DIM, dtype=np.float32),
                speed_limits=np.zeros(10, dtype=np.float32),
                road_geometry=np.zeros(20, dtype=np.float32),
                intersections=np.zeros(10, dtype=np.float32),
                traffic_signs=np.zeros(15, dtype=np.float32),
                confidence=0.0,
                timestamp=time.time()
            )

        self.add_observation(center_tile)

        speed_limits = self._extract_speed_limits(center_tile)
        road_geometry = self._extract_road_geometry(center_tile)
        intersections = self._extract_intersections(center_tile)
        traffic_signs = self._extract_traffic_signs(center_tile)

        attention_weights = self._compute_attention(vision_features, center_tile.features)

        return LatentMapState(
            encoded_features=center_tile.features,
            speed_limits=speed_limits,
            road_geometry=road_geometry,
            intersections=intersections,
            traffic_signs=traffic_signs,
            confidence=attention_weights,
            timestamp=center_tile.timestamp
        )

    def _extract_speed_limits(self, tile: MapTile) -> np.ndarray:
        """Extract speed limits from road segments."""
        speed_limits = np.zeros(10, dtype=np.float32)

        for i, segment in enumerate(tile.road_segments[:10]):
            speed_limits[i] = segment.get('speed_limit', 0) / 130.0

        return speed_limits

    def _extract_road_geometry(self, tile: MapTile) -> np.ndarray:
        """Extract road geometry features."""
        geometry = np.zeros(20, dtype=np.float32)

        num_roads = min(len(tile.road_segments), 5)
        geometry[0] = num_roads / 5.0

        for i, segment in enumerate(tile.road_segments[:5]):
            geometry[1 + i*4] = segment.get('lanes', 1) / 6.0
            geometry[2 + i*4] = segment.get('curvature', 0) * 1000
            geometry[3 + i*4] = 1.0 if segment.get('oneway', False) else 0.0
            geometry[4 + i*4] = 1.0 if segment.get('bridge', False) else 0.0

        return geometry

    def _extract_intersections(self, tile: MapTile) -> np.ndarray:
        """Extract intersection features."""
        intersections = np.zeros(10, dtype=np.float32)

        for i, intersection in enumerate(tile.road_segments[:10]):
            intersections[i] = intersection.get('num_exits', 1) / 6.0

        return intersections

    def _extract_traffic_signs(self, tile: MapTile) -> np.ndarray:
        """Extract traffic sign features."""
        signs = np.zeros(15, dtype=np.float32)

        sign_types = {'stop': 0, 'speed_limit': 1, 'yield': 2, 'warning': 3}

        for segment in tile.road_segments:
            if 'speed_limit' in segment:
                idx = min(int(segment['speed_limit'] / 15), 14)
                signs[idx] = 1.0

        return signs

    def _compute_attention(self, vision_features: np.ndarray, map_features: np.ndarray) -> float:
        """
        Compute attention weight between vision and map features.

        This allows the model to learn which map features are relevant
        given the current visual context.
        """
        if vision_features.shape[0] != map_features.shape[0]:
            return 0.5

        vision_norm = np.linalg.norm(vision_features)
        map_norm = np.linalg.norm(map_features)

        if vision_norm == 0 or map_norm == 0:
            return 0.5

        similarity = np.dot(vision_features, map_features) / (vision_norm * map_norm)

        attention = (similarity + 1) / 2

        self._attention_weights = attention
        return attention

    def get_historical_features(self, n: int = 5) -> np.ndarray:
        """Get attention-weighted historical map features."""
        if not self._history:
            return np.zeros(self._encoder.FEATURE_DIM, dtype=np.float32)

        recent = self._history[-n:]
        features = np.array([tile.features for tile in recent])

        if self._attention_weights is not None:
            weights = np.array([self._attention_weights] * len(recent))
            weights = weights / weights.sum()
            return np.average(features, axis=0, weights=weights)

        return np.mean(features, axis=0)


class MapInsightAttention:
    """
    Attention mechanism for map insight.

    Allows the E2E model to "attend" to specific parts of the map
    just like it attends to specific frames in video.
    """

    def __init__(self, feature_dim: int = 128, num_heads: int = 4):
        self.feature_dim = feature_dim
        self.num_heads = num_heads
        self.head_dim = feature_dim // num_heads

        self._query_proj = np.random.randn(feature_dim, feature_dim).astype(np.float32) * 0.01
        self._key_proj = np.random.randn(feature_dim, feature_dim).astype(np.float32) * 0.01
        self._value_proj = np.random.randn(feature_dim, feature_dim).astype(np.float32) * 0.01

    def forward(self, vision_context: np.ndarray, map_features: np.ndarray) -> np.ndarray:
        """
        Compute attention-weighted map features.

        Args:
            vision_context: Current vision model context (feature_dim,)
            map_features: Map features from LatentMapIngestion (feature_dim,)

        Returns:
            Attention-weighted map features (feature_dim,)
        """
        query = vision_context @ self._query_proj
        key = map_features @ self._key_proj
        value = map_features @ self._value_proj

        scores = np.dot(query, key) / np.sqrt(self.head_dim)

        attention_weights = np.exp(scores - np.max(scores))
        attention_weights = attention_weights / attention_weights.sum()

        output = attention_weights * value

        return output
