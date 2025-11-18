"""
Integrated control system for sunnypilot autonomous driving.
This system combines perception, planning, and control for safe autonomous driving.
"""

import numpy as np
import time
import threading
from typing import Dict, Tuple, Optional, Any
from dataclasses import dataclass
import queue

from core_autonomous_system import AutonomousDrivingSystem, Destination, VehicleState
from navigation_system import NavigationSystem
from perception_system import PerceptionSystem, PerceptionOutput


@dataclass
class ControlOutput:
    """Output from the integrated control system."""
    steering_angle: float
    gas_pedal: float  # 0.0 to 1.0
    brake_pedal: float  # 0.0 to 1.0
    timestamp: float
    valid: bool = True


@dataclass
class SystemState:
    """Overall state of the autonomous driving system."""
    vehicle_state: VehicleState
    perception_data: Optional[PerceptionOutput]
    navigation_guidance: Dict[str, Any]
    control_output: Optional[ControlOutput]
    system_status: str  # "idle", "planning", "driving", "stopping", "emergency"
    timestamp: float


class IntegratedControlSystem:
    """
    Main integrated control system that manages perception, planning, and control
    to achieve safe autonomous driving.
    """
    
    def __init__(self):
        # System components
        self.navigation_system = NavigationSystem()
        self.perception_system = PerceptionSystem()
        self.vehicle_state = VehicleState(
            speed=0.0,
            acceleration=0.0,
            heading=0.0,
            position=(40.7128, -74.0060),  # Initial position (New York City)
            steering_angle=0.0
        )
        
        # Control parameters
        self.steering_controller = SteeringController()
        self.speed_controller = SpeedController()
        self.safety_monitor = SafetyMonitor()
        
        # System state
        self.destination = None
        self.system_state = "idle"
        self.emergency_stop_active = False
        self.control_output = None
        self.last_control_time = 0.0
        
        # Threading and synchronization
        self.running = False
        self.main_thread = None
        self.control_thread = None
        self.update_rate = 0.05  # 20 Hz for control updates
        
        # ARM-optimized parameters for comma three
        self._init_hardware_optimizations()
    
    def _init_hardware_optimizations(self):
        """Initialize hardware-specific optimizations for comma three."""
        print("Initializing ARM-optimized control system...")
        
        # Pre-allocate arrays to minimize memory allocation
        self._control_buffer = np.zeros(3, dtype=np.float32)  # [steering, gas, brake]
        self._sensor_buffer = np.zeros(10, dtype=np.float32)  # Sensor readings
        
        # Set up efficient data structures
        self._control_history = queue.deque(maxlen=10)  # Last 10 control outputs
        
        # Initialize control algorithms optimized for ARM
        self._max_control_computation_time = 0.04  # Must compute within 40ms for 20Hz
        self._control_stability_factor = 0.9  # For smoothing control outputs
        
        print("Control system hardware optimizations initialized")
    
    def set_destination(self, destination: Destination) -> bool:
        """Set destination for point-to-point navigation."""
        if self.navigation_system.set_destination(
            self.vehicle_state.position, 
            (destination.latitude, destination.longitude)
        ):
            self.destination = destination
            self.system_state = "planning"
            print(f"Destination set: {destination.name}")
            return True
        else:
            print("Failed to set destination")
            return False
    
    def start_system(self) -> bool:
        """Start the integrated control system."""
        print("Starting integrated control system...")
        
        # Start perception system
        self.perception_system.start()
        
        # Start main control loop
        self.running = True
        self.main_thread = threading.Thread(target=self._main_loop)
        self.main_thread.start()
        
        print("Integrated control system started")
        return True
    
    def stop_system(self):
        """Stop the integrated control system."""
        print("Stopping integrated control system...")
        self.running = False
        
        if self.main_thread:
            self.main_thread.join()
        
        # Stop perception system
        self.perception_system.stop()
        
        print("Integrated control system stopped")
    
    def _main_loop(self):
        """Main control loop that integrates perception, planning, and control."""
        print("Starting integrated control loop...")
        
        while self.running:
            try:
                loop_start_time = time.time()
                
                # Update vehicle state (in simulation, based on control output)
                self._update_vehicle_state()
                
                # Get perception data
                perception_data = self.perception_system.get_latest_perception()
                
                # Update navigation guidance
                nav_guidance = self.navigation_system.update_position(self.vehicle_state.position)
                
                # Determine necessary actions based on perception, navigation, and safety
                if perception_data and self.destination:
                    # Process control cycle
                    control_output = self._process_control_cycle(
                        perception_data, nav_guidance)
                    
                    # Apply output to simulated control
                    self._apply_control_output(control_output)
                    
                    # Update system state for monitoring
                    self.control_output = control_output
                    self._update_system_state(perception_data, nav_guidance, control_output)
                
                # Monitor system safety
                if self._check_safety(perception_data, nav_guidance):
                    # If safety issue detected, trigger emergency procedures
                    self._handle_safety_issue(perception_data)
                
                # Update system status
                self._update_system_status(nav_guidance)
                
                # Throttle loop to update_rate
                elapsed = time.time() - loop_start_time
                sleep_time = max(0, self.update_rate - elapsed)
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
            except Exception as e:
                print(f"Error in main control loop: {e}")
                time.sleep(0.1)  # Brief pause on error
    
    def _update_vehicle_state(self):
        """Update the simulated vehicle state based on control inputs."""
        # This simulates the physical response of the vehicle to control inputs
        # In a real system, this would read from actual sensors
        if self.control_output and not self.emergency_stop_active:
            dt = self.update_rate
            
            # Update speed based on gas/brake inputs
            acceleration = (self.control_output.gas_pedal * 3.0 +  # Max acceleration 3 m/s²
                           self.control_output.brake_pedal * -5.0)  # Max deceleration 5 m/s²
            self.vehicle_state.acceleration = acceleration
            self.vehicle_state.speed = max(0, self.vehicle_state.speed + acceleration * dt)
            
            # Update steering angle
            self.vehicle_state.steering_angle = self.control_output.steering_angle
            
            # Update position based on current speed and heading
            # This is a simplified model - in reality, would use localization data
            distance = self.vehicle_state.speed * dt
            self.vehicle_state.position = (
                self.vehicle_state.position[0] + distance * 0.00001 * np.cos(self.vehicle_state.heading),
                self.vehicle_state.position[1] + distance * 0.00001 * np.sin(self.vehicle_state.heading)
            )
    
    def _process_control_cycle(self, perception: PerceptionOutput, 
                             navigation: Dict[str, Any]) -> ControlOutput:
        """Process one control cycle."""
        start_time = time.time()
        
        # Get desired steering from navigation system
        desired_steering = self._calculate_desired_steering(navigation)
        
        # Adjust for obstacles detected by perception
        desired_steering = self._adjust_steering_for_obstacles(
            desired_steering, perception, navigation)
        
        # Get desired speed from navigation
        desired_speed = self._calculate_desired_speed(navigation)
        
        # Adjust for safety conditions
        desired_speed = self._adjust_speed_for_safety(desired_speed, perception)
        
        # Calculate control outputs
        steering_output = self.steering_controller.compute(
            desired_steering, self.vehicle_state.steering_angle, self.vehicle_state.speed)
        
        # Limit steering rate to prevent jerky movements
        max_steering_rate = 0.2  # radians per control cycle
        if self.control_output:
            rate_limit = max_steering_rate * self.update_rate
            steering_change = steering_output - self.control_output.steering_angle
            steering_output = self.control_output.steering_angle + np.clip(
                steering_change, -rate_limit, rate_limit)
        
        # Calculate speed control
        speed_output = self.speed_controller.compute(
            desired_speed, self.vehicle_state.speed, self.vehicle_state.acceleration)
        
        # Determine gas and brake from acceleration
        gas_output, brake_output = self._acceleration_to_pedals(speed_output)
        
        # Apply safety limits
        gas_output = max(0.0, min(1.0, gas_output))
        brake_output = max(0.0, min(1.0, brake_output))
        steering_output = max(-1.0, min(1.0, steering_output))
        
        # Check computation time for performance monitoring
        computation_time = time.time() - start_time
        if computation_time > self._max_control_computation_time:
            print(f"WARNING: Control computation took {computation_time*1000:.1f}ms, "
                  f"exceeds limit of {self._max_control_computation_time*1000:.0f}ms")
        
        # Add to control history for stability calculations
        self._control_history.append((steering_output, gas_output, brake_output))
        
        control_output = ControlOutput(
            steering_angle=steering_output,
            gas_pedal=gas_output,
            brake_pedal=brake_output,
            timestamp=time.time(),
            valid=True
        )
        
        return control_output
    
    def _calculate_desired_steering(self, navigation: Dict[str, Any]) -> float:
        """Calculate desired steering angle based on navigation guidance."""
        # Get current and next waypoints
        current_waypoint = navigation.get("current_waypoint")
        next_waypoint = navigation.get("next_waypoint")
        
        if not current_waypoint or not next_waypoint:
            return self.vehicle_state.steering_angle  # Maintain current steering
        
        # Calculate the required steering to head toward the next waypoint
        # Simple proportional control
        lat_diff = next_waypoint.latitude - self.vehicle_state.position[0]
        lon_diff = next_waypoint.longitude - self.vehicle_state.position[1]
        
        # Calculate heading error
        desired_heading = np.arctan2(lon_diff, lat_diff)
        heading_error = desired_heading - self.vehicle_state.heading
        
        # Normalize angle to -π to π
        while heading_error > np.pi:
            heading_error -= 2 * np.pi
        while heading_error < -np.pi:
            heading_error += 2 * np.pi
        
        # Convert heading error to steering angle (simplified)
        # This should be replaced with more sophisticated lateral control
        max_steering = 0.5  # Maximum steering angle in radians
        desired_steering = np.clip(heading_error * 0.5, -max_steering, max_steering)
        
        return desired_steering
    
    def _adjust_steering_for_obstacles(self, original_steering: float, 
                                     perception: PerceptionOutput, 
                                     navigation: Dict[str, Any]) -> float:
        """Adjust steering to avoid obstacles while following the route."""
        if not perception or not perception.objects:
            return original_steering
        
        # Check for obstacles in the path
        for obj in perception.objects:
            if obj.obj_type in ["car", "pedestrian", "bicycle"] and obj.confidence > 0.7:
                # Calculate if obstacle is in the vehicle's path
                # Simplified approach: if object is ahead and in the ego lane
                if (obj.center[1] < 320 and obj.center[1] > 200 and  # In the front half of camera view
                    obj.center[0] > 100 and obj.center[0] < 540 and  # In the central area
                    obj.distance and obj.distance < 50.0):  # Within 50m
                
                    # Adjust steering to avoid obstacle
                    # For now, simple adjustment based on object position in image
                    image_center_x = 320
                    obj_offset = (obj.center[0] - image_center_x) / image_center_x  # Normalize to [-1, 1]
                    
                    # If object is to the right, steer left; if object is to the left, steer right
                    avoidance_steering = -obj_offset * 0.3  # Limited avoidance steering
                    original_steering += avoidance_steering
                
                    # Also reduce speed when avoiding obstacles
                    break  # Only adjust for the first significant obstacle
    
        return np.clip(original_steering, -0.5, 0.5)  # Limit steering adjustment
    
    def _calculate_desired_speed(self, navigation: Dict[str, Any]) -> float:
        """Calculate desired speed based on navigation guidance."""
        # Get current waypoint to determine speed limit
        current_waypoint = navigation.get("current_waypoint")
        
        # Base desired speed on route information
        if current_waypoint:
            speed_limit = current_waypoint.speed_limit
        else:
            speed_limit = 15.0  # Default speed limit (about 54 km/h)
        
        # Adjust for upcoming turns or traffic
        distance_to_destination = navigation.get("distance_to_destination", float('inf'))
        if distance_to_destination < 100.0:  # Approaching destination
            speed_limit = min(speed_limit, 5.0)  # Slow down near destination
        
        # Check if next waypoint requires slowing down
        next_waypoint = navigation.get("next_waypoint")
        if next_waypoint and next_waypoint.stop_required:
            distance_to_next = 10.0 * (next_waypoint.stop_duration + 5)  # Estimate distance
            if distance_to_next < 30.0:
                speed_limit = min(speed_limit, 3.0)  # Slow down for stops
        
        # Adjust for traffic conditions
        if navigation.get("obstacle_warning", False):
            obstacle_distance = navigation.get("obstacle_distance", float('inf'))
            if obstacle_distance < 50.0:
                speed_limit = min(speed_limit, 10.0)  # Slow down for obstacles
        
        return speed_limit
    
    def _adjust_speed_for_safety(self, desired_speed: float, 
                               perception: PerceptionOutput) -> float:
        """Adjust speed based on safety considerations."""
        if not perception:
            return desired_speed
        
        # Adjust for traffic lights
        for light in perception.traffic_lights:
            if light.state == "red" and light.distance and light.distance < 100.0:
                # Calculate stopping distance based on current speed
                stopping_distance = self._calculate_stopping_distance(
                    self.vehicle_state.speed)
                
                if light.distance < stopping_distance + 10.0:  # 10m safety margin
                    # Need to stop for red light
                    max_safe_speed = min(desired_speed, 2.0)  # Crawl to light
                    desired_speed = max_safe_speed
                    break
        
        # Adjust for detected objects
        for obj in perception.objects:
            if obj.obj_type in ["car", "truck"] and obj.confidence > 0.7:
                if obj.distance and obj.distance < 50.0:
                    # Maintain safe following distance
                    safe_distance = self.vehicle_state.speed * 2.0 + 20.0  # 2-second rule + 20m
                    if obj.distance < safe_distance:
                        # Adjust speed based on relative velocity if available
                        if obj.velocity:
                            relative_speed = self.vehicle_state.speed - obj.velocity[0]
                            if relative_speed > 0:
                                desired_speed = max(0.0, obj.velocity[0] + 5.0)  # Follow with safety gap
                        else:
                            desired_speed = min(desired_speed, self.vehicle_state.speed * 0.7)
        
        # Limit maximum speed
        desired_speed = min(desired_speed, 25.0)  # 90 km/h max
        
        return max(0.0, desired_speed)  # No negative speeds
    
    def _calculate_stopping_distance(self, speed: float) -> float:
        """Calculate stopping distance based on current speed."""
        # Simplified stopping distance calculation
        # Reaction time: 1.5 seconds + braking distance
        reaction_distance = speed * 1.5
        braking_distance = (speed ** 2) / (2 * 4.5)  # Using 4.5 m/s² deceleration
        return reaction_distance + braking_distance
    
    def _acceleration_to_pedals(self, acceleration: float) -> Tuple[float, float]:
        """Convert desired acceleration to gas and brake pedal positions."""
        # Convert acceleration to pedal inputs
        if acceleration > 0:
            # Positive acceleration (speed up)
            gas = min(acceleration / 3.0, 1.0)  # Max 3 m/s² acceleration
            brake = 0.0
        elif acceleration < 0:
            # Negative acceleration (slow down)
            gas = 0.0
            brake = min(abs(acceleration) / 5.0, 1.0)  # Max 5 m/s² braking
        else:
            # Maintain speed
            gas = 0.1  # Small gas to maintain momentum on level ground
            brake = 0.0
        
        return gas, brake
    
    def _apply_control_output(self, control_output: ControlOutput):
        """Apply control output to the vehicle (simulation only)."""
        # In a real system, this would send commands to the vehicle
        # For simulation purposes, we just update the internal state
        pass
    
    def _update_system_state(self, perception: PerceptionOutput, 
                           navigation: Dict[str, Any], 
                           control: ControlOutput):
        """Update internal system state for monitoring."""
        # Store in internal state for potential external use
        pass
    
    def _check_safety(self, perception: Optional[PerceptionOutput], 
                     navigation: Dict[str, Any]) -> bool:
        """Check for safety issues and return True if emergency action needed."""
        if not perception:
            return False
        
        # Emergency stop conditions
        # 1. Imminent collision
        for obj in perception.objects:
            if (obj.distance and obj.distance < 5.0 and 
                obj.obj_type in ["car", "pedestrian", "bicycle"] and 
                obj.confidence > 0.8):
                print(f"EMERGENCY: Close object detected at {obj.distance:.1f}m")
                return True
        
        # 2. Red light with insufficient distance to stop
        for light in perception.traffic_lights:
            if (light.state == "red" and light.distance and 
                light.distance < self._calculate_stopping_distance(self.vehicle_state.speed)):
                print(f"EMERGENCY: Red light ahead, insufficient stopping distance")
                return True
        
        # 3. System deviation from route
        if (navigation.get("recalculate_needed", False) and 
            self.vehicle_state.speed > 3.0):  # Only if moving significantly
            print("EMERGENCY: Significant deviation from route")
            return True
        
        # 4. Critical system failures
        if not perception.valid:
            print("EMERGENCY: Perception system failure")
            return True
        
        return False
    
    def _handle_safety_issue(self, perception: Optional[PerceptionOutput]):
        """Handle an identified safety issue."""
        print("Handling safety issue...")
        self.emergency_stop_active = True
        
        # Apply maximum braking
        emergency_brake = ControlOutput(
            steering_angle=self.vehicle_state.steering_angle,  # Maintain current steering
            gas_pedal=0.0,
            brake_pedal=1.0,  # Full brake
            timestamp=time.time(),
            valid=True
        )
        
        self._apply_control_output(emergency_brake)
        self.control_output = emergency_brake
        
        # Wait a bit before attempting to resume
        time.sleep(2.0)
        
        # Clear emergency state
        self.emergency_stop_active = False
        print("Safety issue handled, resuming normal operation")
    
    def _update_system_status(self, navigation: Dict[str, Any]):
        """Update system status based on current state."""
        if navigation.get("route_complete", False):
            self.system_state = "destination_reached"
        elif self.emergency_stop_active:
            self.system_state = "emergency_stop"
        elif self.system_state == "planning":
            if navigation.get("current_waypoint") is not None:
                self.system_state = "driving"
        elif self.system_state == "driving":
            if navigation.get("recalculate_needed", False):
                self.system_state = "recalculating"
    
    def get_system_status(self) -> Dict[str, Any]:
        """Get current system status for monitoring."""
        return {
            "system_state": self.system_state,
            "vehicle_speed": self.vehicle_state.speed,
            "vehicle_position": self.vehicle_state.position,
            "destination": self.destination.name if self.destination else None,
            "distance_to_destination": self.navigation_system.get_route_progress()["distance_remaining"] 
                                       if hasattr(self.navigation_system, 'get_route_progress') else float('inf'),
            "emergency_active": self.emergency_stop_active,
            "perception_valid": self.perception_system.get_latest_perception() is not None,
            "control_output": self.control_output.__dict__ if self.control_output else None
        }


class SteeringController:
    """Steering controller using PID control."""
    
    def __init__(self):
        # PID parameters optimized for vehicle steering
        self.kp = 0.8  # Proportional gain
        self.ki = 0.0  # Integral gain (minimal to prevent accumulation)
        self.kd = 0.1  # Derivative gain (for stability)
        
        self.prev_error = 0.0
        self.integral = 0.0
    
    def compute(self, desired_steering: float, current_steering: float, speed: float) -> float:
        """Compute steering output based on desired and current values."""
        # Calculate error
        error = desired_steering - current_steering
        
        # Update integral with anti-windup
        self.integral = np.clip(self.integral + error * 0.05, -0.5, 0.5)  # 0.05 = dt
        
        # Calculate derivative
        derivative = (error - self.prev_error) / 0.05
        
        # Calculate output
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        # Update previous error
        self.prev_error = error
        
        # Adjust for speed (higher speeds require less steering)
        if speed > 5.0:  # If moving faster than walking speed
            speed_factor = np.clip(1.0 - (speed - 5.0) / 20.0, 0.3, 1.0)
            output *= speed_factor
        
        # Limit output
        return np.clip(output, -1.0, 1.0)


class SpeedController:
    """Speed controller using PID control."""
    
    def __init__(self):
        # PID parameters optimized for vehicle speed control
        self.kp = 0.5  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.05  # Derivative gain
        
        self.prev_error = 0.0
        self.integral = 0.0
    
    def compute(self, desired_speed: float, current_speed: float, acceleration: float) -> float:
        """Compute acceleration output based on desired and current values."""
        # Calculate error
        error = desired_speed - current_speed
        
        # Update integral with anti-windup
        self.integral = np.clip(self.integral + error * 0.05, -2.0, 2.0)  # 0.05 = dt
        
        # Calculate derivative
        derivative = (error - self.prev_error) / 0.05
        
        # Calculate output (acceleration)
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        # Update previous error
        self.prev_error = error
        
        return output


class SafetyMonitor:
    """Monitor system for safety checks and validation."""
    
    def __init__(self):
        self.safety_checks_enabled = True
        self.last_safety_check = time.time()
        self.safety_check_interval = 1.0  # seconds
    
    def validate_safe_operation(self, perception: PerceptionOutput, 
                              control_output: ControlOutput, 
                              vehicle_state: VehicleState) -> Tuple[bool, str]:
        """Validate that the system is operating safely."""
        if not perception.valid:
            return False, "Perception system not valid"
        
        if control_output.brake_pedal > 0.8 and vehicle_state.speed > 5.0:
            # Hard braking check
            if time.time() - self.last_safety_check < 0.1:
                # Frequent hard braking may indicate issues
                pass  # For now, just continue
        
        # Check that outputs are in valid ranges
        if (not 0.0 <= control_output.gas_pedal <= 1.0 or
            not 0.0 <= control_output.brake_pedal <= 1.0 or
            not -1.0 <= control_output.steering_angle <= 1.0):
            return False, "Control output out of bounds"
        
        # Future: Add more sophisticated safety checks
        return True, "Operating within safety parameters"


def main():
    """Main function to run the integrated control system."""
    print("Sunnypilot Integrated Control System")
    print("====================================")
    
    # Create the integrated control system
    control_system = IntegratedControlSystem()
    
    # Define a destination
    destination = Destination(
        latitude=40.7589,  # Times Square, NYC
        longitude=-73.9851,
        name="Times Square",
        arrival_radius=10.0
    )
    
    # Set the destination
    if control_system.set_destination(destination):
        print(f"Destination '{destination.name}' set successfully")
        
        # Start the system
        if control_system.start_system():
            print("Integrated control system started. Press Ctrl+C to stop.")
            
            try:
                # Run for 30 seconds for demonstration
                start_time = time.time()
                while time.time() - start_time < 30:
                    # Print system status periodically
                    if int(time.time() - start_time) % 5 == 0:
                        status = control_system.get_system_status()
                        print(f"\nStatus: {status['system_state']}")
                        print(f"  Speed: {status['vehicle_speed']:.2f} m/s")
                        print(f"  Position: {status['vehicle_position']}")
                        print(f"  Emergency: {status['emergency_active']}")
                        print(f"  Distance to destination: {status['distance_to_destination']:.0f}m")
                    
                    time.sleep(1)
            
            except KeyboardInterrupt:
                print("\nStopping integrated control system...")
            
            finally:
                control_system.stop_system()
                print("System stopped.")
        else:
            print("Failed to start control system")
    else:
        print("Failed to set destination")


if __name__ == "__main__":
    main()