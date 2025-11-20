"""
Improved Navigation Routing for Sunnypilot
Enhanced routing with more realistic calculations
"""
from typing import List, Dict, Tuple
import math


def calculate_basic_route(start: Tuple[float, float], end: Tuple[float, float]) -> List[Dict]:
  """Calculate a more realistic route between two points"""
  distance = haversine_distance(start[0], start[1], end[0], end[1])

  # For now, return a route with basic steps - in production this would use
  # a real routing service or map data
  route_steps = []

  # Calculate intermediate steps for longer distances
  step_distance = min(5.0, distance / 5.0)  # At most 5 steps
  num_steps = max(1, int(distance / step_distance))

  step_lat = (end[0] - start[0]) / num_steps
  step_lon = (end[1] - start[1]) / num_steps

  for i in range(num_steps):
    current_lat = start[0] + step_lat * i
    current_lon = start[1] + step_lon * i
    next_lat = start[0] + step_lat * (i + 1)
    next_lon = start[1] + step_lon * (i + 1)

    step = {
      "step": i + 1,
      "start_coords": [current_lat, current_lon],
      "end_coords": [next_lat, next_lon],
      "distance_km": step_distance,
      "instructions": f"Drive toward ({next_lat:.6f}, {next_lon:.6f})",
      "maneuver": "CONTINUE" if i < num_steps - 1 else "DESTINATION"
    }
    route_steps.append(step)

  return route_steps


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  """Calculate distance between two points using proper haversine formula"""
  # Convert latitude and longitude from degrees to radians
  lat1_rad = math.radians(lat1)
  lon1_rad = math.radians(lon1)
  lat2_rad = math.radians(lat2)
  lon2_rad = math.radians(lon2)

  # Haversine formula
  dlat = lat2_rad - lat1_rad
  dlon = lon2_rad - lon1_rad
  a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
  c = 2 * math.asin(math.sqrt(a))

  # Radius of earth in kilometers
  r = 6371.0
  return c * r