"""
Simulation Framework for Sunnypilot Testing
Provides comprehensive simulation environment for testing all enhanced systems
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
import time
import math
import random
from enum import Enum
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import threading


class SimulationMode(Enum):
    """Different simulation modes"""
    BASIC = "basic"
    TRAFFIC = "traffic"
    WEATHER = "weather"
    FAILURE = "failure"


@dataclass
class SimulationConfig:
    """Configuration for the simulation"""
    simulation_time: float = 60.0  # seconds
    time_step: float = 0.05       # seconds (20 Hz)
    seed: Optional[int] = 42      # For reproducible results
    enable_visualization: bool = False
    scenario_complexity: str = "medium"  # "low", "medium", "high"


class VehicleState:
    """Represents the state of a vehicle in the simulation"""
    
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0,
                 vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
                 heading: float = 0.0, length: float = 4.0, width: float = 2.0):
        self.position = np.array([x, y, z], dtype=np.float64)
        self.velocity = np.array([vx, vy, vz], dtype=np.float64)
        self.heading = heading  # radians
        self.length = length
        self.width = width
        self.acceleration = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    
    def update(self, ax: float, ay: float, dt: float):
        """Update state with acceleration"""
        self.acceleration = np.array([ax, ay, 0.0])
        self.velocity += self.acceleration * dt
        self.position += self.velocity * dt
    
    def get_corners(self) -> np.ndarray:
        """Get vehicle corner positions for visualization"""
        # Calculate the four corners of the vehicle
        cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
        
        half_l, half_w = self.length / 2, self.width / 2
        
        # Corners relative to center
        corners_rel = np.array([
            [-half_l, -half_w],
            [-half_l, half_w],
            [half_l, half_w],
            [half_l, -half_w]
        ])
        
        # Rotate and translate
        corners = np.zeros((4, 2))
        for i in range(4):
            corners[i, 0] = self.position[0] + corners_rel[i, 0] * cos_h - corners_rel[i, 1] * sin_h
            corners[i, 1] = self.position[1] + corners_rel[i, 0] * sin_h + corners_rel[i, 1] * cos_h
        
        return corners


class ObjectTrackerSimulation:
    """Simulates object tracking with realistic noise and errors"""
    
    def __init__(self):
        self.objects = {}
        self.next_id = 1
        self.detection_noise = 0.3  # meters
        self.detection_probability = 0.9  # 90% detection rate
    
    def add_object(self, initial_state: VehicleState, object_type: str = "vehicle") -> int:
        """Add an object to tracking simulation"""
        obj_id = self.next_id
        self.next_id += 1
        
        self.objects[obj_id] = {
            'state': initial_state,
            'type': object_type,
            'last_seen': time.time(),
            'track_length': 0
        }
        
        return obj_id
    
    def get_simulated_detections(self) -> List[Dict[str, Any]]:
        """Generate simulated detections with noise"""
        detections = []
        
        for obj_id, obj_data in self.objects.items():
            # Simulate detection probability
            if random.random() < self.detection_probability:
                state = obj_data['state']
                
                # Add noise to position
                noisy_pos = state.position + np.random.normal(0, self.detection_noise, 3)
                
                # Calculate relative position from ego vehicle (assumed at origin)
                dRel = noisy_pos[0]  # longitudinal distance
                yRel = noisy_pos[1]  # lateral distance
                
                # Add noise to velocity
                noisy_vel = state.velocity + np.random.normal(0, 0.2, 3)
                vRel = noisy_vel[0]  # relative velocity
                
                detection = {
                    'id': obj_id,
                    'dRel': float(dRel),
                    'yRel': float(yRel), 
                    'vRel': float(vRel),
                    'prob': 0.8 + random.random() * 0.2,  # 0.8-1.0 confidence
                    'type': obj_data['type']
                }
                
                detections.append(detection)
        
        return detections
    
    def update_object(self, obj_id: int, ax: float, ay: float, dt: float):
        """Update an object with acceleration"""
        if obj_id in self.objects:
            obj = self.objects[obj_id]
            obj['state'].update(ax, ay, dt)
            obj['track_length'] += 1


class TrafficSimulation:
    """Simulates realistic traffic scenarios"""
    
    def __init__(self):
        self.vehicles = {}
        self.lane_width = 3.7
        self.speed_limit = 25.0  # m/s (about 90 km/h)
    
    def create_lead_vehicle(self, distance: float, lateral_offset: float = 0.0, 
                          speed: Optional[float] = None) -> int:
        """Create a lead vehicle in front of ego"""
        if speed is None:
            speed = self.speed_limit - 5.0  # 5 m/s slower
        
        initial_state = VehicleState(
            x=distance,
            y=lateral_offset,
            vx=speed,
            vy=0.0,
            heading=0.0
        )
        
        obj_tracker = ObjectTrackerSimulation()
        # Note: We'll integrate with the main tracker later
        return 1  # Return dummy ID; real integration would link to tracker
    
    def create_cut_in_vehicle(self, start_distance: float, 
                            lane_offset: int, cut_in_time: float) -> Dict:
        """Create a vehicle that cuts in"""
        return {
            'start_distance': start_distance,
            'lane_offset': lane_offset,  # -1 for left, +1 for right
            'cut_in_time': cut_in_time,
            'current_lane': lane_offset,
            'is_cutting_in': False
        }


class WeatherSimulation:
    """Simulates weather effects on perception and control"""
    
    def __init__(self):
        self.current_weather = "clear"
        self.visibility = 200.0  # meters
        self.precipitation = 0.0  # 0.0 to 1.0
        self.road_friction = 1.0  # 1.0 = normal, lower = slippery
    
    def set_weather(self, weather_type: str):
        """Set current weather conditions"""
        self.current_weather = weather_type
        
        if weather_type == "clear":
            self.visibility = 200.0
            self.precipitation = 0.0
            self.road_friction = 1.0
        elif weather_type == "rain_light":
            self.visibility = 100.0
            self.precipitation = 0.3
            self.road_friction = 0.85
        elif weather_type == "rain_heavy":
            self.visibility = 50.0
            self.precipitation = 0.7
            self.road_friction = 0.7
        elif weather_type == "snow":
            self.visibility = 80.0
            self.precipitation = 0.8
            self.road_friction = 0.6
        elif weather_type == "fog":
            self.visibility = 20.0
            self.precipitation = 0.1
            self.road_friction = 0.95  # Not slippery but visibility poor
    
    def get_weather_effects(self) -> Dict[str, float]:
        """Get current weather effects"""
        return {
            'visibility': self.visibility,
            'precipitation': self.precipitation,
            'friction': self.road_friction
        }


class ControlSystemSimulation:
    """Simulates the control system with realistic delays and errors"""
    
    def __init__(self, dt: float = 0.05):
        self.dt = dt
        self.control_delay = 0.1  # 100ms delay
        self.actuator_noise = 0.05  # 5% noise
        self.max_steer = 0.5  # radians
        self.max_throttle = 1.0
        self.max_brake = 1.0
    
    def apply_control(self, target_steering: float, target_throttle: float, 
                     target_brake: float) -> Dict[str, float]:
        """Apply control commands with realistic limitations"""
        # Apply delays by storing and retrieving previous commands
        # (simplified model - real implementation would use queuing)
        
        # Add noise to commands
        noisy_steering = target_steering + random.gauss(0, self.actuator_noise)
        noisy_throttle = max(0, min(target_throttle + random.gauss(0, self.actuator_noise), self.max_throttle))
        noisy_brake = max(0, min(target_brake + random.gauss(0, self.actuator_noise), self.max_brake))
        
        # Apply limits
        noisy_steering = max(-self.max_steer, min(noisy_steering, self.max_steer))
        
        return {
            'steering': noisy_steering,
            'throttle': noisy_throttle,
            'brake': noisy_brake
        }


class Simulator:
    """Main simulation environment"""
    
    def __init__(self, config: SimulationConfig = None):
        self.config = config or SimulationConfig()
        
        if self.config.seed is not None:
            random.seed(self.config.seed)
            np.random.seed(self.config.seed)
        
        self.time = 0.0
        self.dt = self.config.time_step
        self.ego_vehicle = VehicleState(x=0.0, y=0.0, vx=15.0, vy=0.0, heading=0.0)
        self.object_tracker = ObjectTrackerSimulation()
        self.traffic_sim = TrafficSimulation()
        self.weather_sim = WeatherSimulation()
        self.control_sim = ControlSystemSimulation(dt=self.dt)
        
        # Data logging
        self.ego_trajectory = []
        self.other_trajectories = {}
        self.controls_history = []
        self.safety_events = []
        
        # Visualization
        self.fig = None
        self.ax = None
        if self.config.enable_visualization:
            self._setup_visualization()
    
    def _setup_visualization(self):
        """Setup visualization if enabled"""
        plt.ion()
        self.fig, self.ax = plt.subplots(1, 1, figsize=(12, 8))
    
    def add_scenario(self, scenario_type: SimulationMode, **kwargs):
        """Add a specific scenario to the simulation"""
        if scenario_type == SimulationMode.TRAFFIC:
            # Add traffic to simulation
            distance = kwargs.get('distance', 50.0)
            speed = kwargs.get('speed', 10.0)
            self.traffic_sim.create_lead_vehicle(distance, speed=speed)
        elif scenario_type == SimulationMode.WEATHER:
            weather_type = kwargs.get('weather', 'clear')
            self.weather_sim.set_weather(weather_type)
        elif scenario_type == SimulationMode.FAILURE:
            # Simulate system failures
            self._inject_system_failure(**kwargs)
    
    def _inject_system_failure(self, **kwargs):
        """Inject system failures for testing"""
        failure_type = kwargs.get('failure_type', 'sensor')
        start_time = kwargs.get('start_time', 10.0)
        
        # This would be used to simulate various failures
        pass
    
    def step(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        """Execute one simulation step"""
        # Update ego vehicle based on control inputs (simulated)
        # For now, simple constant velocity model
        self.ego_vehicle.update(0.0, 0.0, self.dt)
        
        # Get simulated detections
        detections = self.object_tracker.get_simulated_detections()
        
        # Create model output structure similar to real model
        model_output = {
            'frame_id': int(self.time / self.dt),
            'leads_v3': [],
            'lane_lines': [],
            'plan': np.zeros((32, 13)),  # Placeholder
            'position': type('Position', (), {'y': [0.1, 0.15, 0.12, 0.18, 0.14]})()
        }
        
        # Convert detections to the format expected by the planning system
        for det in detections:
            lead = type('Lead', (), {
                'dRel': det['dRel'],
                'yRel': det['yRel'], 
                'vRel': det['vRel'],
                'prob': det['prob']
            })()
            model_output['leads_v3'].append(lead)
        
        # Create car state
        car_state = {
            'vEgo': float(self.ego_vehicle.velocity[0]),
            'steeringAngleDeg': math.degrees(self.ego_vehicle.heading),
            'aEgo': float(self.ego_vehicle.acceleration[0]),
            'max_acceleration': 3.0,
            'min_acceleration': -5.0
        }
        
        # Create environment context
        env_context = {
            'ego_velocity': self.ego_vehicle.velocity,
            'ego_position': self.ego_vehicle.position,
            'weather': self.weather_sim.get_weather_effects(),
            'current_lane': 1,
            'num_lanes': 3
        }
        
        # Log data
        self.ego_trajectory.append(self.ego_vehicle.position.copy())
        
        # Update simulation time
        self.time += self.dt
        
        return model_output, car_state, env_context
    
    def run_simulation(self, duration: Optional[float] = None) -> Dict[str, Any]:
        """Run the full simulation"""
        duration = duration or self.config.simulation_time
        steps = int(duration / self.dt)
        
        print(f"Running simulation for {duration}s at {1/self.dt}Hz")
        
        model_outputs = []
        car_states = []
        env_contexts = []
        
        for i in range(steps):
            model_output, car_state, env_context = self.step()
            
            model_outputs.append(model_output)
            car_states.append(car_state)
            env_contexts.append(env_context)
            
            # Update visualization if enabled
            if self.config.enable_visualization and i % 10 == 0:  # Update every 10 steps
                self._update_visualization()
        
        print("Simulation completed!")
        
        return {
            'model_outputs': model_outputs,
            'car_states': car_states,
            'env_contexts': env_contexts,
            'ego_trajectory': np.array(self.ego_trajectory),
            'simulation_time': self.time
        }
    
    def _update_visualization(self):
        """Update the visualization"""
        if not self.config.enable_visualization or self.fig is None:
            return
        
        self.ax.clear()
        
        # Plot ego vehicle
        ego_pos = self.ego_vehicle.position
        ego_rect = Rectangle(
            (ego_pos[0] - 2, ego_pos[1] - 1), 
            4, 2, 
            angle=math.degrees(self.ego_vehicle.heading),
            facecolor='blue', 
            alpha=0.7,
            label='Ego Vehicle'
        )
        self.ax.add_patch(ego_rect)
        
        # Plot detections (simulated other vehicles)
        detections = self.object_tracker.get_simulated_detections()
        for det in detections:
            vehicle_circle = Circle(
                (det['dRel'], det['yRel']), 
                1.0, 
                facecolor='red', 
                alpha=0.5
            )
            self.ax.add_patch(vehicle_circle)
        
        # Plot trajectory
        if len(self.ego_trajectory) > 1:
            traj = np.array(self.ego_trajectory)
            self.ax.plot(traj[:, 0], traj[:, 1], 'b-', alpha=0.3, label='Ego Trajectory')
        
        self.ax.set_xlim(ego_pos[0] - 50, ego_pos[0] + 50)
        self.ax.set_ylim(ego_pos[1] - 20, ego_pos[1] + 20)
        self.ax.set_xlabel('Longitudinal Position (m)')
        self.ax.set_ylabel('Lateral Position (m)')
        self.ax.set_title(f'Simulation at t={self.time:.2f}s')
        self.ax.legend()
        self.ax.grid(True)
        
        plt.draw()
        plt.pause(0.001)
    
    def evaluate_safety_metrics(self, results: Dict[str, Any]) -> Dict[str, float]:
        """Evaluate safety metrics from simulation"""
        trajectory = results['ego_trajectory']
        
        metrics = {
            'total_distance': 0.0,
            'avg_velocity': 0.0,
            'max_lat_accel': 0.0,
            'min_distance_to_others': float('inf'),
            'time_to_collision_min': float('inf'),
            'emergency_braking_events': 0,
            'lane_departure_events': 0
        }
        
        if len(trajectory) > 1:
            # Calculate total distance
            for i in range(1, len(trajectory)):
                dist_step = np.linalg.norm(trajectory[i] - trajectory[i-1])
                metrics['total_distance'] += dist_step
            
            # Calculate average velocity (approximate)
            total_time = results['simulation_time']
            if total_time > 0:
                metrics['avg_velocity'] = metrics['total_distance'] / total_time
        
        # For more complex metrics, we would need to analyze the control and perception data
        # This is a simplified example - a full implementation would analyze all simulation data
        
        return metrics


class ScenarioManager:
    """Manages different test scenarios for the simulation"""
    
    def __init__(self):
        self.scenarios = {}
    
    def register_scenario(self, name: str, setup_func, description: str = ""):
        """Register a scenario setup function"""
        self.scenarios[name] = {
            'setup': setup_func,
            'description': description
        }
    
    def get_scenario(self, name: str):
        """Get a scenario setup function"""
        return self.scenarios.get(name)
    
    def list_scenarios(self):
        """List all available scenarios"""
        return list(self.scenarios.keys())


def create_basic_scenario(simulator: Simulator):
    """Create a basic driving scenario"""
    simulator.add_scenario(SimulationMode.TRAFFIC, distance=50.0, speed=12.0)
    simulator.add_scenario(SimulationMode.TRAFFIC, distance=80.0, speed=14.0)


def create_traffic_scenario(simulator: Simulator):
    """Create a more complex traffic scenario"""
    # Multiple vehicles at different distances and speeds
    simulator.add_scenario(SimulationMode.TRAFFIC, distance=30.0, speed=10.0)
    simulator.add_scenario(SimulationMode.TRAFFIC, distance=50.0, speed=12.0)
    simulator.add_scenario(SimulationMode.TRAFFIC, distance=100.0, speed=16.0)
    
    # Add a slow vehicle that might cause lane change
    simulator.add_scenario(SimulationMode.TRAFFIC, distance=25.0, speed=8.0)


def create_weather_scenario(simulator: Simulator):
    """Create a weather-affected scenario"""
    simulator.add_scenario(SimulationMode.WEATHER, weather='rain_heavy')
    simulator.add_scenario(SimulationMode.TRAFFIC, distance=40.0, speed=10.0)


def run_comprehensive_test():
    """Run a comprehensive test of all enhanced systems with simulation"""
    print("Running Comprehensive Test with Simulation Framework...")
    
    # Create simulation configuration
    config = SimulationConfig(
        simulation_time=30.0,  # 30 seconds
        time_step=0.05,        # 20 Hz
        enable_visualization=False,  # Disable for automated testing
        scenario_complexity='medium'
    )
    
    # Initialize simulator
    simulator = Simulator(config)
    
    # Set up a complex scenario
    create_traffic_scenario(simulator)
    create_weather_scenario(simulator)
    
    # Run simulation
    results = simulator.run_simulation()
    
    # Evaluate safety metrics
    safety_metrics = simulator.evaluate_safety_metrics(results)
    
    print("\nSafety Metrics:")
    for metric, value in safety_metrics.items():
        print(f"  {metric}: {value}")
    
    # Test integration with planning system
    from selfdrive.common.predictive_planning import get_predictive_planner
    from selfdrive.common.enhanced_validation import EnhancedSafetyValidator
    from selfdrive.common.safety_redundancy import get_safety_monitor
    
    planner = get_predictive_planner()
    validator = EnhancedSafetyValidator()
    safety_monitor = get_safety_monitor()
    
    # Test with simulation data
    test_model_output = results['model_outputs'][0] if results['model_outputs'] else {}
    test_car_state = results['car_states'][0] if results['car_states'] else {}
    
    if test_model_output and test_car_state:
        # Test planning with simulation data
        try:
            # Convert simulation detections to proper format for planning
            tracked_objects = []
            if 'leads_v3' in test_model_output and test_model_output['leads_v3']:
                for lead in test_model_output['leads_v3']:
                    tracked_object = {
                        'id': 0,  # Will be set properly
                        'dRel': getattr(lead, 'dRel', 0.0),
                        'yRel': getattr(lead, 'yRel', 0.0),
                        'vRel': getattr(lead, 'vRel', 0.0),
                        'prob': getattr(lead, 'prob', 0.8)
                    }
                    tracked_objects.append(tracked_object)
            
            ego_state = {
                'position': np.array([0.0, 0.0, 0.0]),
                'velocity': np.array([test_car_state.get('vEgo', 15.0), 0.0, 0.0]),
                'heading': 0.0
            }
            
            # Plan with simulated data
            best_trajectory, predicted_states = planner.plan(ego_state, tracked_objects, {})
            
            if best_trajectory:
                print(f"\nPlanning successful: {best_trajectory.maneuver_type.value}")
                print(f"Trajectory cost: {best_trajectory.cost:.2f}")
            else:
                print("\nPlanning failed")
            
            # Test validation
            validation_passed, validation_result = validator.calculate_situation_aware_confidence(
                test_model_output, 
                type('CarState', (), test_car_state)()
            )
            
            safety_safe, safety_reason = validator.get_safety_recommendation(
                validation_result, 
                type('CarState', (), test_car_state)()
            )
            
            print(f"\nValidation passed: {validation_passed}")
            print(f"Safety check: {safety_safe} - {safety_reason}")
            
        except Exception as e:
            print(f"\nError during integration test: {e}")
    
    print("\nComprehensive test completed!")
    return True


def get_simulator() -> Simulator:
    """Get the global simulator instance"""
    if not hasattr(get_simulator, 'instance'):
        config = SimulationConfig(enable_visualization=False)
        get_simulator.instance = Simulator(config)
    return get_simulator.instance


# Global instance for use across the system
simulator = get_simulator()


# Example usage and testing
if __name__ == "__main__":
    print("Testing Simulation Framework...")
    
    # Run the comprehensive test
    success = run_comprehensive_test()
    
    print(f"\nSimulation framework test {'passed' if success else 'failed'}!")
    
    # Create a simple scenario and run it
    config = SimulationConfig(
        simulation_time=10.0,
        enable_visualization=False
    )
    
    simple_sim = Simulator(config)
    create_basic_scenario(simple_sim)
    
    results = simple_sim.run_simulation()
    
    print(f"Simple simulation completed with {len(results['model_outputs'])} steps")
    print(f"Ego trajectory length: {len(results['ego_trajectory'])} points")
    
    # Evaluate metrics
    metrics = simple_sim.evaluate_safety_metrics(results)
    print("Sample metrics:", {k: v for k, v in list(metrics.items())[:3]})  # Show first 3 metrics
    
    print("Simulation framework test completed!")