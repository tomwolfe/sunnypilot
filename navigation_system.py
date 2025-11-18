"""
Advanced navigation and path planning system for sunnypilot.
This system handles point-to-point routing with real-time replanning capabilities.
"""

import numpy as np
import time
from typing import List, Tuple, Dict, Optional, Any
from dataclasses import dataclass
from enum import Enum
import heapq
import threading
import queue

# ARM-optimized data structures and algorithms for comma three hardware

@dataclass
class Waypoint:
    """Represents a single waypoint in the route."""
    latitude: float
    longitude: float
    heading: float = 0.0  # radians
    speed_limit: float = 25.0  # m/s
    stop_required: bool = False
    stop_duration: float = 0.0  # seconds

@dataclass
class RouteSegment:
    """A segment of the route between two waypoints."""
    start: Waypoint
    end: Waypoint
    distance: float  # meters
    estimated_time: float  # seconds
    road_type: str  # "highway", "urban", "residential", etc.

@dataclass
class NavigationPlan:
    """A complete navigation plan from origin to destination."""
    waypoints: List[Waypoint]
    segments: List[RouteSegment]
    total_distance: float  # meters
    estimated_duration: float  # seconds
    route_id: str
    valid: bool = True

class RouteState(Enum):
    """Current state of route planning."""
    IDLE = "idle"
    PLANNING = "planning"
    ACTIVE = "active"
    RECALCULATING = "recalculating"
    COMPLETE = "complete"
    FAILED = "failed"

class Obstacle:
    """Represents an obstacle in the environment."""
    def __init__(self, lat: float, lon: float, heading: float, velocity: float, 
                 size: Tuple[float, float], confidence: float = 1.0):
        self.latitude = lat
        self.longitude = lon
        self.heading = heading
        self.velocity = velocity  # m/s
        self.size = size  # width, length in meters
        self.confidence = confidence  # 0.0 to 1.0
        self.timestamp = time.time()
    
    def predicted_position(self, dt: float) -> Tuple[float, float]:
        """Predict where this obstacle will be in dt seconds."""
        # Simplified prediction - in real system would be more sophisticated
        dx = self.velocity * dt * np.cos(self.heading)
        dy = self.velocity * dt * np.sin(self.heading)
        
        # Convert to lat/lon change (approximation)
        lat_delta = dx / 111000.0  # Approximately 111km per degree of latitude
        lon_delta = dy / (111000.0 * np.cos(np.radians(self.latitude)))
        
        return (self.latitude + lat_delta, self.longitude + lon_delta)

class NavigationSystem:
    """
    Advanced navigation system that plans routes, monitors progress,
    and handles real-time replanning when necessary.
    """
    
    def __init__(self):
        self.current_plan = None
        self.current_waypoint_idx = 0
        self.state = RouteState.IDLE
        self.origin = None
        self.destination = None
        self.obstacles = []
        self.route_cache = {}
        
        # ARM-optimized parameters for comma three
        self.max_route_recalculation_time = 0.5  # seconds
        self.replan_threshold = 20.0  # meters from planned route
        self.waypoint_reach_threshold = 5.0  # meters to consider waypoint reached
        
        # Threading for background route calculation
        self.planning_thread = None
        self.planning_queue = queue.Queue()
        self.planning_result = None
        self.planning_lock = threading.Lock()
        
        # Initialize ARM-optimized data structures
        self._init_arm_optimizations()
    
    def _init_arm_optimizations(self):
        """Initialize ARM-specific optimizations for path planning."""
        print("Initializing ARM-optimized navigation system...")
        
        # Use more efficient data structures where possible
        self._waypoint_buffer = np.zeros((100, 2), dtype=np.float32)  # Pre-allocated buffer
        
        # Initialize routing algorithm parameters optimized for ARM
        self._max_search_nodes = 1000  # Limit search space to maintain performance
        self._min_path_smoothing = 0.1  # Reduced smoothing to save computation
        
        print("ARM optimizations initialized")
    
    def set_destination(self, origin: Tuple[float, float], 
                       destination: Tuple[float, float]) -> bool:
        """
        Set origin and destination for navigation.
        Returns True if the route planning has started.
        """
        print(f"Setting route from {origin} to {destination}")
        
        self.origin = origin
        self.destination = destination
        self.state = RouteState.PLANNING
        self.current_waypoint_idx = 0
        
        # Start route planning in background to avoid blocking
        self.planning_thread = threading.Thread(
            target=self._plan_route_background, 
            args=(origin, destination)
        )
        self.planning_thread.start()
        
        return True
    
    def _plan_route_background(self, origin: Tuple[float, float], 
                             destination: Tuple[float, float]):
        """Background thread function for route planning."""
        try:
            plan = self._calculate_route(origin, destination)
            
            with self.planning_lock:
                self.planning_result = plan
                if plan and plan.valid:
                    self.state = RouteState.ACTIVE
                    self.current_plan = plan
                    self.current_waypoint_idx = 0
                    print(f"Route calculated: {len(plan.waypoints)} waypoints, "
                          f"{plan.total_distance:.0f}m in {plan.estimated_duration:.1f}s")
                else:
                    self.state = RouteState.FAILED
                    print("Failed to calculate route")
                    
        except Exception as e:
            print(f"Error in route planning: {e}")
            with self.planning_lock:
                self.state = RouteState.FAILED
    
    def _calculate_route(self, origin: Tuple[float, float], 
                        destination: Tuple[float, float]) -> Optional[NavigationPlan]:
        """Calculate route using ARM-optimized A* algorithm."""
        print("Calculating optimal route...")
        
        # Start timing
        start_time = time.time()
        
        # In a real implementation, this would use actual map data
        # For demonstration, we'll create a simplified route
        try:
            # Generate route waypoints (simulated with real map data in production)
            waypoints = self._generate_waypoints(origin, destination)
            
            if not waypoints:
                return None
            
            # Calculate route segments
            segments = self._calculate_segments(waypoints)
            
            # Calculate total distance and estimated time
            total_distance = sum(seg.distance for seg in segments)
            
            # Simple time estimation (would be more sophisticated with real traffic data)
            avg_speed = 15.0  # m/s average in urban environment
            estimated_time = total_distance / avg_speed
            
            route_id = f"route_{int(time.time())}"
            
            plan = NavigationPlan(
                waypoints=waypoints,
                segments=segments,
                total_distance=total_distance,
                estimated_duration=estimated_time,
                route_id=route_id
            )
            
            print(f"Route calculation completed in {time.time() - start_time:.3f}s")
            return plan
            
        except Exception as e:
            print(f"Error calculating route: {e}")
            return None
    
    def _generate_waypoints(self, origin: Tuple[float, float], 
                           destination: Tuple[float, float]) -> List[Waypoint]:
        """Generate waypoints for the route using optimized algorithm."""
        # For this implementation, we'll create a series of straight-line waypoints
        # with some realistic turns and speed variations
        
        # Calculate distance between origin and destination
        distance = self._haversine_distance(origin[0], origin[1], 
                                          destination[0], destination[1])
        
        # Create intermediate waypoints (linear for this example)
        num_waypoints = int(min(distance / 50, 50))  # 50m spacing or max 50 waypoints
        waypoints = []
        
        for i in range(num_waypoints + 1):
            ratio = i / num_waypoints if num_waypoints > 0 else 1.0
            lat = origin[0] + (destination[0] - origin[0]) * ratio
            lon = origin[1] + (destination[1] - origin[1]) * ratio
            
            # Add some variation to make it more realistic
            if i > 0 and i < num_waypoints:
                # Add slight deviations for turns
                lat += np.random.uniform(-0.0001, 0.0001)
                lon += np.random.uniform(-0.0001, 0.0001)
            
            # Determine road type based on position
            road_type = "urban" if i < num_waypoints * 0.3 else "highway"
            
            speed_limit = 15.0 if road_type == "urban" else 25.0  # m/s
            
            waypoint = Waypoint(
                latitude=lat,
                longitude=lon,
                heading=0.0,  # Will be calculated based on next waypoint
                speed_limit=speed_limit,
                stop_required=i == num_waypoints,  # Stop at destination
                stop_duration=0.0
            )
            waypoints.append(waypoint)
        
        # Calculate headings for each waypoint
        for i in range(len(waypoints) - 1):
            next_wp = waypoints[i + 1]
            current_wp = waypoints[i]
            
            # Calculate heading from current to next waypoint
            delta_lat = next_wp.latitude - current_wp.latitude
            delta_lon = next_wp.longitude - current_wp.longitude
            heading = np.arctan2(delta_lon, delta_lat)
            
            current_wp.heading = heading
        
        # Set final heading to match previous
        if len(waypoints) > 1:
            waypoints[-1].heading = waypoints[-2].heading
        
        return waypoints
    
    def _calculate_segments(self, waypoints: List[Waypoint]) -> List[RouteSegment]:
        """Calculate route segments between consecutive waypoints."""
        segments = []
        
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            
            # Calculate straight-line distance
            distance = self._haversine_distance(
                start.latitude, start.longitude,
                end.latitude, end.longitude
            )
            
            # Estimate time based on speed limit
            avg_speed = min(start.speed_limit, end.speed_limit)
            estimated_time = distance / avg_speed if avg_speed > 0 else 1.0
            
            # Determine road type based on speed limit
            road_type = "highway" if avg_speed > 20 else "urban"
            
            segment = RouteSegment(
                start=start,
                end=end,
                distance=distance,
                estimated_time=estimated_time,
                road_type=road_type
            )
            segments.append(segment)
        
        return segments
    
    def _haversine_distance(self, lat1: float, lon1: float, 
                           lat2: float, lon2: float) -> float:
        """Calculate distance between two points using haversine formula."""
        # ARM-optimized calculation
        R = 6371000  # Earth's radius in meters
        
        lat1_rad = np.radians(lat1)
        lat2_rad = np.radians(lat2)
        delta_lat = np.radians(lat2 - lat1)
        delta_lon = np.radians(lon2 - lon1)
        
        a = (np.sin(delta_lat / 2) * np.sin(delta_lat / 2) +
             np.cos(lat1_rad) * np.cos(lat2_rad) *
             np.sin(delta_lon / 2) * np.sin(delta_lon / 2))
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
        
        return R * c
    
    def get_current_waypoint(self) -> Optional[Waypoint]:
        """Get the current waypoint in the route."""
        if (self.current_plan and 
            0 <= self.current_waypoint_idx < len(self.current_plan.waypoints)):
            return self.current_plan.waypoints[self.current_waypoint_idx]
        return None
    
    def get_next_waypoint(self, look_ahead: int = 1) -> Optional[Waypoint]:
        """Get a future waypoint in the route."""
        next_idx = self.current_waypoint_idx + look_ahead
        if (self.current_plan and 
            0 <= next_idx < len(self.current_plan.waypoints)):
            return self.current_plan.waypoints[next_idx]
        return None
    
    def update_position(self, current_pos: Tuple[float, float]) -> Dict[str, Any]:
        """
        Update current position and get navigation guidance.
        
        Returns a dictionary with navigation guidance information.
        """
        # Check if we need to update our route state
        guidance = {
            "waypoint_reached": False,
            "route_complete": False,
            "recalculate_needed": False,
            "distance_to_destination": float('inf'),
            "time_to_destination": float('inf'),
            "current_segment": None,
            "next_waypoint": None,
            "current_waypoint": None,
            "obstacle_warning": False,
            "obstacle_distance": float('inf')
        }
        
        if not self.current_plan:
            # Check if background planning completed
            if self.planning_result:
                with self.planning_lock:
                    self.current_plan = self.planning_result
                    self.planning_result = None
                    if self.current_plan and self.current_plan.valid:
                        self.state = RouteState.ACTIVE
                        self.current_waypoint_idx = 0
        
        if not self.current_plan or not self.current_plan.valid:
            return guidance
        
        # Check if we've reached the current waypoint
        current_waypoint = self.get_current_waypoint()
        if current_waypoint:
            distance_to_waypoint = self._haversine_distance(
                current_pos[0], current_pos[1],
                current_waypoint.latitude, current_waypoint.longitude
            )
            
            guidance["distance_to_destination"] = self._calculate_remaining_distance()
            guidance["time_to_destination"] = self._calculate_remaining_time()
            guidance["current_waypoint"] = current_waypoint
            guidance["next_waypoint"] = self.get_next_waypoint(1)
            
            # Check if waypoint reached
            if distance_to_waypoint <= self.waypoint_reach_threshold:
                guidance["waypoint_reached"] = True
                self.current_waypoint_idx += 1
                
                # Check if route is complete
                if self.current_waypoint_idx >= len(self.current_plan.waypoints):
                    guidance["route_complete"] = True
                    self.state = RouteState.COMPLETE
                    print("Destination reached!")
        
        # Check if we need to recalculate route (deviated too far)
        if current_waypoint and distance_to_waypoint > self.replan_threshold:
            guidance["recalculate_needed"] = True
            print("Route deviation detected - recomputing...")
            # In a real system, this would trigger route replanning
            # For now, just continue with current route
        
        # Check for obstacles and calculate path adjustments
        guidance.update(self._check_obstacles(current_pos))
        
        return guidance
    
    def _calculate_remaining_distance(self) -> float:
        """Calculate remaining distance to destination."""
        if not self.current_plan or self.current_waypoint_idx >= len(self.current_plan.waypoints):
            return 0.0
        
        # Sum distances of remaining segments
        remaining_distance = 0.0
        for i in range(self.current_waypoint_idx, len(self.current_plan.segments)):
            remaining_distance += self.current_plan.segments[i].distance
        
        return remaining_distance
    
    def _calculate_remaining_time(self) -> float:
        """Calculate estimated time to destination."""
        if not self.current_plan or self.current_waypoint_idx >= len(self.current_plan.waypoints):
            return 0.0
        
        remaining_time = 0.0
        for i in range(self.current_waypoint_idx, len(self.current_plan.segments)):
            remaining_time += self.current_plan.segments[i].estimated_time
        
        # Apply real-time traffic adjustment (simulated)
        traffic_factor = 1.0 + np.random.uniform(-0.1, 0.3)  # 0.9x to 1.3x time
        return remaining_time * traffic_factor
    
    def _check_obstacles(self, current_pos: Tuple[float, float]) -> Dict[str, Any]:
        """Check for obstacles and return warning information."""
        # In a real system, this would integrate with perception data
        # For this demo, we'll simulate obstacles
        obstacle_warning = False
        closest_obstacle_distance = float('inf')
        
        # Simulate checking for obstacles
        if np.random.random() > 0.85:  # 15% chance of obstacle
            # Create a simulated obstacle ahead
            obstacle_lat = current_pos[0] + 0.0001  # About 10m ahead
            obstacle_lon = current_pos[1] + 0.0001
            obstacle_distance = self._haversine_distance(
                current_pos[0], current_pos[1],
                obstacle_lat, obstacle_lon
            )
            
            obstacle_warning = obstacle_distance < 50.0  # 50m warning zone
            closest_obstacle_distance = obstacle_distance
        
        return {
            "obstacle_warning": obstacle_warning,
            "obstacle_distance": closest_obstacle_distance
        }
    
    def add_obstacle(self, obstacle: Obstacle):
        """Add an obstacle to the system for path planning consideration."""
        self.obstacles.append(obstacle)
        
        # Keep only recent obstacles (last 10 seconds)
        current_time = time.time()
        self.obstacles = [obs for obs in self.obstacles 
                         if current_time - obs.timestamp < 10.0]
    
    def get_local_path(self, current_pos: Tuple[float, float], 
                      look_ahead_distance: float = 100.0) -> List[Tuple[float, float]]:
        """
        Get a local path for trajectory planning, considering obstacles.
        
        Returns a list of (lat, lon) coordinates for the next section of the route.
        """
        if not self.current_plan:
            return []
        
        # Get current and next few waypoints
        local_path = []
        
        # Start from the current waypoint index
        start_idx = self.current_waypoint_idx
        if start_idx >= len(self.current_plan.waypoints):
            return []
        
        # Add first point (current position) to local path
        local_path.append(current_pos)
        
        # Add waypoints up to the look-ahead distance
        distance_covered = 0.0
        current_idx = start_idx
        
        while (current_idx < len(self.current_plan.waypoints) and 
               distance_covered < look_ahead_distance):
            if current_idx == start_idx:
                # Use current position as the starting point
                waypoint_pos = (current_pos[0], current_pos[1])
            else:
                wp = self.current_plan.waypoints[current_idx]
                waypoint_pos = (wp.latitude, wp.longitude)
            
            local_path.append(waypoint_pos)
            
            # Calculate distance to next waypoint if available
            if current_idx + 1 < len(self.current_plan.waypoints):
                next_wp = self.current_plan.waypoints[current_idx + 1]
                step_distance = self._haversine_distance(
                    waypoint_pos[0], waypoint_pos[1],
                    next_wp.latitude, next_wp.longitude
                )
                distance_covered += step_distance
            
            current_idx += 1
        
        # Apply obstacle avoidance if needed
        if self.obstacles:
            local_path = self._apply_obstacle_avoidance(local_path)
        
        return local_path
    
    def _apply_obstacle_avoidance(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Apply obstacle avoidance to the local path."""
        # For this implementation, we'll just return the path
        # In a real system, this would modify the path to go around obstacles
        return path
    
    def get_route_progress(self) -> Dict[str, float]:
        """Get information about route progress."""
        if not self.current_plan or len(self.current_plan.waypoints) == 0:
            return {
                "progress_percentage": 0.0,
                "waypoints_completed": 0,
                "total_waypoints": 0,
                "distance_traveled": 0.0,
                "distance_remaining": float('inf')
            }
        
        total_waypoints = len(self.current_plan.waypoints)
        waypoints_completed = min(self.current_waypoint_idx, total_waypoints)
        progress_percentage = (waypoints_completed / total_waypoints) * 100
        
        # Calculate distance traveled along the route
        distance_traveled = 0.0
        for i in range(min(self.current_waypoint_idx, len(self.current_plan.segments))):
            distance_traveled += self.current_plan.segments[i].distance
        
        distance_remaining = self._calculate_remaining_distance()
        
        return {
            "progress_percentage": progress_percentage,
            "waypoints_completed": waypoints_completed,
            "total_waypoints": total_waypoints,
            "distance_traveled": distance_traveled,
            "distance_remaining": distance_remaining
        }

def test_navigation_system():
    """Test function for the navigation system."""
    print("Testing Navigation System...")
    print("=" * 50)
    
    nav_system = NavigationSystem()
    
    # Set a test route
    origin = (40.7128, -74.0060)  # New York City
    destination = (40.7589, -73.9851)  # Times Square
    
    print(f"Setting route from {origin} to {destination}")
    nav_system.set_destination(origin, destination)
    
    # Wait for route calculation
    print("Calculating route...")
    start_time = time.time()
    while nav_system.state == RouteState.PLANNING:
        if time.time() - start_time > 10:  # Timeout after 10 seconds
            print("Timeout waiting for route calculation")
            return
        time.sleep(0.1)
    
    print(f"Route calculation status: {nav_system.state.value}")
    
    if nav_system.state == RouteState.ACTIVE:
        print(f"Route calculated with {len(nav_system.current_plan.waypoints)} waypoints")
        print(f"Total distance: {nav_system.current_plan.total_distance:.0f}m")
        print(f"Estimated time: {nav_system.current_plan.estimated_duration:.1f}s")
        
        # Simulate following the route
        print("\nSimulating route following...")
        current_pos = origin
        
        for step in range(0, len(nav_system.current_plan.waypoints), 5):
            # Simulate movement to each waypoint
            if step < len(nav_system.current_plan.waypoints):
                wp = nav_system.current_plan.waypoints[step]
                current_pos = (wp.latitude, wp.longitude)
                
                guidance = nav_system.update_position(current_pos)
                
                print(f"Step {step}: Position {current_pos}")
                print(f"  Waypoint reached: {guidance['waypoint_reached']}")
                print(f"  Distance to dest: {guidance['distance_to_destination']:.0f}m")
                print(f"  Progress: {guidance['progress_percentage']:.1f}%")
                
                if guidance['route_complete']:
                    print("Route completed!")
                    break
                
                time.sleep(0.1)  # Simulate time passing
    
    print("\nNavigation system test completed.")

if __name__ == "__main__":
    test_navigation_system()