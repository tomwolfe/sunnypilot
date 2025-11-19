"""
Simple Navigation Routing for Sunnypilot
Basic routing functions
"""
from typing import List, Dict, Tuple


def calculate_basic_route(start: Tuple[float, float], end: Tuple[float, float]) -> List[Dict]:
  """Calculate a basic route between two points"""
  # Simplified routing - in a real implementation would use OpenStreetMap or other routing service
  distance = _haversine_distance(start[0], start[1], end[0], end[1])
  
  return [{
    "start": start,
    "end": end,
    "distance": distance,
    "instructions": ["Head toward destination"]
  }]


def _haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  """Calculate distance between two points using haversine formula (simplified)"""
  # Simplified calculation
  import math
  lat_diff = abs(lat2 - lat1)
  lon_diff = abs(lon2 - lon1)
  return math.sqrt(lat_diff**2 + lon_diff**2) * 111.0  # Rough conversion to km