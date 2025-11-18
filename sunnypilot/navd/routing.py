"""
Basic routing implementation for sunnypilot navigation system.
This provides actual route calculation instead of the placeholder implementation.
"""
import time
import math
from typing import List, Tuple, Optional
from dataclasses import dataclass
from openpilot.sunnypilot.navd.helpers import Coordinate
from openpilot.selfdrive.common.metrics import Metrics, record_metric

@dataclass
class RouteSegment:
    """Represents a segment of a route with geometry and maneuver information."""
    start: Coordinate
    end: Coordinate
    distance: float  # in meters
    bearing: float   # in degrees
    maneuver_type: str  # turn, straight, arrive, etc.
    estimated_time: float  # in seconds

@dataclass
class Route:
    """Complete route with segments and metadata."""
    destination: Coordinate
    segments: List[RouteSegment]
    total_distance: float
    total_time: float
    estimated_arrival: float  # timestamp

class BasicRouter:
    """A basic routing engine that calculates routes between coordinates."""
    
    def __init__(self):
        self.last_route_calculation_time = 0
        self.route_calculation_count = 0
        
    def calculate_route(self, start: Coordinate, destination: Coordinate) -> Optional[Route]:
        """
        Calculate a basic route from start to destination.
        In a real implementation, this would use actual map data and routing algorithms.
        For this implementation, we'll create a simple straight-line route with possible waypoints.
        """
        start_time = time.time()
        
        try:
            # Calculate straight-line distance
            total_distance = start.distance_to(destination)
            
            if total_distance < 10:  # Less than 10 meters, consider as arrived
                return None
                
            # Create route segments - for simplicity, we'll create segments of ~100m each
            segments = []
            current_pos = start
            
            # Calculate bearing from start to destination
            bearing = self._calculate_bearing(start, destination)
            
            # Estimate time based on average speed (15 m/s = 54 km/h)
            avg_speed = 15.0  # m/s
            total_time = total_distance / avg_speed if avg_speed > 0 else 0
            
            # For longer routes, add intermediate waypoints
            num_segments = min(max(int(total_distance / 100), 1), 50)  # Limit segments to prevent too many
            
            segment_distance = total_distance / num_segments if num_segments > 0 else 0
            
            for i in range(num_segments):
                # Calculate intermediate coordinate
                if i == num_segments - 1:  # Last segment goes directly to destination
                    segment_end = destination
                    segment_dist = current_pos.distance_to(destination)
                else:
                    # Calculate intermediate point
                    fraction = (i + 1) / num_segments
                    lat_diff = destination.latitude - start.latitude
                    lon_diff = destination.longitude - start.longitude
                    segment_end = Coordinate(
                        start.latitude + (lat_diff * fraction),
                        start.longitude + (lon_diff * fraction)
                    )
                    segment_dist = current_pos.distance_to(segment_end)
                
                # Determine maneuver type
                if i == num_segments - 1:
                    maneuver_type = "arrive"
                elif abs(self._calculate_bearing(current_pos, segment_end) - bearing) > 10:
                    # Significant bearing change indicates a turn
                    maneuver_type = "turn"
                else:
                    maneuver_type = "straight"
                
                # Calculate estimated time for this segment
                segment_time = segment_dist / avg_speed if avg_speed > 0 else 0
                
                segment = RouteSegment(
                    start=current_pos,
                    end=segment_end,
                    distance=segment_dist,
                    bearing=self._calculate_bearing(current_pos, segment_end),
                    maneuver_type=maneuver_type,
                    estimated_time=segment_time
                )
                
                segments.append(segment)
                current_pos = segment_end
            
            # Create the route
            route = Route(
                destination=destination,
                segments=segments,
                total_distance=total_distance,
                total_time=total_time,
                estimated_arrival=time.time() + total_time
            )
            
            # Record metrics for route calculation
            calculation_time = time.time() - start_time
            self.route_calculation_count += 1
            self.last_route_calculation_time = time.time()
            
            record_metric(Metrics.NAVIGATION_LATENCY_MS, calculation_time * 1000, {
                "operation": "route_calculation",
                "distance": total_distance,
                "segments": len(segments),
                "calculation_time": calculation_time,
                "start": [start.latitude, start.longitude],
                "destination": [destination.latitude, destination.longitude]
            })
            
            record_metric("navigation.route_calculation_time", calculation_time, {
                "distance": total_distance,
                "segments": len(segments)
            })
            
            return route
            
        except Exception as e:
            print(f"Error calculating route: {e}")
            return None
    
    def _calculate_bearing(self, start: Coordinate, end: Coordinate) -> float:
        """
        Calculate the initial bearing from start to end coordinate.
        Formula: θ = atan2(sin(Δlong).cos(lat2), cos(lat1).sin(lat2) − sin(lat1).cos(lat2).cos(Δlong))
        """
        lat1 = math.radians(start.latitude)
        lat2 = math.radians(end.latitude)
        diff_long = math.radians(end.longitude - start.longitude)
        
        x = math.sin(diff_long) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diff_long))
        
        initial_bearing = math.atan2(x, y)
        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360
        
        return compass_bearing

class EnhancedRouteManager:
    """Enhanced route manager that uses actual routing instead of placeholder."""
    
    def __init__(self):
        self.router = BasicRouter()
        self.current_route: Optional[Route] = None
        self.destination: Optional[Coordinate] = None
        self.active = False
        self.current_segment_index = 0
        self.route_start_time = None
        
    def set_destination(self, coordinate: Coordinate, current_pos: Coordinate) -> bool:
        """Calculate and set a new destination with actual route."""
        start_time = time.time()
        
        # Calculate the route
        route = self.router.calculate_route(current_pos, coordinate)
        if route is None:
            return False
            
        self.current_route = route
        self.destination = coordinate
        self.active = True
        self.current_segment_index = 0
        self.route_start_time = time.time()
        
        # Record metrics
        route_calc_time = time.time() - start_time
        record_metric(Metrics.NAVIGATION_LATENCY_MS, route_calc_time * 1000, {
            "operation": "enhanced_set_destination",
            "route_distance": route.total_distance,
            "route_segments": len(route.segments),
            "destination": [coordinate.latitude, coordinate.longitude]
        })
        
        record_metric(Metrics.ROUTE_COMPLETION_RATE, 0.0, {  # Starting a new route
            "operation": "route_started",
            "distance": route.total_distance,
            "estimated_time": route.total_time
        })
        
        return True
    
    def update_route(self, current_pos: Coordinate) -> bool:
        """Update route progress based on current position."""
        if not self.active or not self.current_route:
            return False
            
        start_time = time.time()
        
        # Find the closest segment to current position
        closest_segment_idx = 0
        min_distance = float('inf')
        
        for i, segment in enumerate(self.current_route.segments):
            dist_to_start = current_pos.distance_to(segment.start)
            dist_to_end = current_pos.distance_to(segment.end)
            dist = min(dist_to_start, dist_to_end)
            
            if dist < min_distance:
                min_distance = dist
                closest_segment_idx = i
        
        self.current_segment_index = closest_segment_idx
        
        # Calculate remaining distance and time
        remaining_distance = 0
        remaining_time = 0
        
        for i in range(closest_segment_idx, len(self.current_route.segments)):
            remaining_distance += self.current_route.segments[i].distance
            remaining_time += self.current_route.segments[i].estimated_time
        
        # Update route information
        self.current_route.total_distance = self.current_route.destination.distance_to(current_pos)
        time_elapsed = time.time() - self.route_start_time if self.route_start_time else 0
        
        # Record metrics
        update_time = time.time() - start_time
        record_metric(Metrics.NAVIGATION_LATENCY_MS, update_time * 1000, {
            "operation": "enhanced_route_update",
            "remaining_distance": remaining_distance,
            "remaining_time": remaining_time,
            "closest_segment": closest_segment_idx
        })
        
        record_metric(Metrics.NAVIGATION_ACCURACY, min_distance, {
            "operation": "route_tracking_accuracy",
            "current_pos": [current_pos.latitude, current_pos.longitude],
            "min_distance_to_route": min_distance
        })
        
        return True
    
    def get_maneuver_info(self, current_pos: Coordinate) -> Tuple[str, float, float]:
        """Get maneuver information for the current position."""
        if not self.active or not self.current_route or self.current_segment_index >= len(self.current_route.segments):
            return "none", 0.0, 0.0
            
        current_segment = self.current_route.segments[self.current_segment_index]
        
        # Calculate distance to the next maneuver (end of current segment)
        distance_to_maneuver = current_pos.distance_to(current_segment.end)
        
        # Get maneuver type and angle
        maneuver_type = current_segment.maneuver_type
        bearing = current_segment.bearing
        
        # Record maneuver metrics
        record_metric(Metrics.MANEUVER_SUCCESS_RATE, 1.0 if maneuver_type != "none" else 0.0, {
            "operation": "maneuver_info_query",
            "maneuver_type": maneuver_type,
            "distance_to_maneuver": distance_to_maneuver
        })
        
        return maneuver_type, distance_to_maneuver, bearing
    
    def check_route_completion(self, current_pos: Coordinate, tolerance: float = 50.0) -> bool:
        """Check if we have reached the destination."""
        if not self.active or not self.destination:
            return False
            
        distance = current_pos.distance_to(self.destination)
        reached = distance <= tolerance
        
        if reached:
            self.active = False
            route_duration = time.time() - self.route_start_time if self.route_start_time else 0
            
            # Record successful completion metrics
            record_metric(Metrics.ROUTE_COMPLETION_RATE, 1.0, {
                "operation": "route_completed",
                "route_duration": route_duration,
                "final_distance": distance,
                "tolerance": tolerance,
                "success": True
            })
        else:
            # Record progress metrics
            progress = 1 - (distance / self.current_route.total_distance if self.current_route and self.current_route.total_distance > 0 else 1)
            record_metric(Metrics.ROUTE_COMPLETION_RATE, progress, {
                "operation": "route_progress",
                "distance_to_destination": distance,
                "tolerance": tolerance,
                "progress": progress
            })
        
        return reached
    
    def clear_route(self):
        """Clear the current route."""
        self.current_route = None
        self.destination = None
        self.active = False
        self.current_segment_index = 0
        self.route_start_time = None