"""
Adaptive control parameter system for Sunnypilot
Real-time tuning of control parameters based on conditions and environment
"""

import numpy as np
from typing import Dict, Any, Optional
from cereal import log
import cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.common.weather_data import weather_data_interface


class AdaptiveControlParameters:
    """
    Adaptive control system that adjusts parameters based on driving conditions,
    weather, road surface, and vehicle dynamics
    """
    
    def __init__(self):
        self.params = Params()
        
        # Default PID and control parameters
        self.default_pid_params = {
            'kp': 0.05,  # Proportional gain
            'ki': 0.01,  # Integral gain
            'kd': 0.1,   # Derivative gain
        }
        
        # Lateral control parameters
        self.default_lat_params = {
            'steer_ratio': 15.0,    # Default steer ratio
            'stiffness_factor': 1.0, # Stiffness factor
            'angle_offset': 0.0,    # Steering angle offset
            'max_steer': 0.5,       # Maximum steering command
        }
        
        # Longitudinal control parameters
        self.default_long_params = {
            'max_accel': 2.0,       # Maximum acceleration
            'min_accel': -3.5,      # Maximum deceleration
            'jerk_limit': 1.0,      # Jerk limit
        }
        
        # Current active parameters
        self.current_pid_params = self.default_pid_params.copy()
        self.current_lat_params = self.default_lat_params.copy()
        self.current_long_params = self.default_long_params.copy()
        
        # Environmental condition filters
        self.weather_condition_filter = FirstOrderFilter(0.5, 5.0, 0.1)
        self.road_surface_filter = FirstOrderFilter(0.3, 10.0, 0.1)
        
        # Historical data for adaptation
        self.steering_history = []
        self.acceleration_history = []
        self.max_history = 100
        
        # Initialize messaging
        try:
            self.sm = messaging.SubMaster([
                'carState', 'liveParameters', 'liveTorqueParameters',
                'modelV2', 'radarState', 'weatherData'  # Note: weatherData may not exist yet
            ], ignore_alive=True)
        except Exception as e:
            cloudlog.warning(f"Could not initialize SubMaster in AdaptiveControlParameters: {e}")
            self.sm = None
    
    def adjust_for_conditions(self, car_state: log.CarState, 
                            environment_data: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Adjust control parameters based on current conditions
        
        Args:
            car_state: Current car state
            environment_data: Optional environmental data
        
        Returns:
            Dictionary of adjusted control parameters
        """
        # Start with default parameters
        self.current_pid_params = self.default_pid_params.copy()
        self.current_lat_params = self.default_lat_params.copy()
        self.current_long_params = self.default_long_params.copy()
        
        # Adjust for vehicle-specific parameters if available
        if self.sm and 'liveParameters' in self.sm:
            live_params = self.sm['liveParameters']
            self.current_lat_params['steer_ratio'] = getattr(live_params, 'steerRatio', 15.0)
            self.current_lat_params['stiffness_factor'] = getattr(live_params, 'stiffnessFactor', 1.0)
            self.current_lat_params['angle_offset'] = getattr(live_params, 'angleOffsetDeg', 0.0)
        
        # Adjust for live torque parameters if available
        if self.sm and 'liveTorqueParameters' in self.sm:
            torque_params = self.sm['liveTorqueParameters']
            if getattr(torque_params, 'useParams', False):
                # Adjust lateral control parameters based on torque parameters
                self.current_pid_params['kp'] *= torque_params.latAccelFactorFiltered
                self.current_pid_params['ki'] *= torque_params.latAccelFactorFiltered
                self.current_pid_params['kd'] *= torque_params.latAccelFactorFiltered
        
        # Adjust for environmental conditions
        self._adjust_for_weather(car_state, environment_data)
        self._adjust_for_road_surface(car_state, environment_data)
        self._adjust_for_speed(car_state)
        self._adjust_for_traffic(car_state)
        self._adjust_for_road_curvature(car_state)
        
        # Update historical data
        self._update_historical_data(car_state)
        
        # Return current parameters
        return {
            'pid': self.current_pid_params,
            'lateral': self.current_lat_params,
            'longitudinal': self.current_long_params,
            'active': True
        }
    
    def _adjust_for_weather(self, car_state: log.CarState,
                          environment_data: Optional[Dict[str, Any]]) -> None:
        """Adjust parameters based on weather conditions"""
        weather_factor = 1.0

        # First, try to get weather data from the interface
        try:
            current_weather = weather_data_interface.get_weather_data()
            weather_factor = weather_data_interface.get_precipitation_factor()

            # Additional adjustments based on other weather factors
            if current_weather['visibility'] < 100:  # Poor visibility
                weather_factor *= 0.85
            elif current_weather['visibility'] < 50:
                weather_factor *= 0.7

            if current_weather['wind_speed'] > 15:  # High winds
                weather_factor *= 0.9
            elif current_weather['wind_speed'] > 25:  # Very high winds
                weather_factor *= 0.75

            # Road surface condition adjustments
            road_surface = current_weather.get('road_surface_condition', 'dry')
            if road_surface == 'wet':
                weather_factor *= 0.85
            elif road_surface in ['icy', 'snowy']:
                weather_factor *= 0.7
        except Exception as e:
            cloudlog.warning(f"Could not get weather data for adaptive control: {e}")

        # If environment data is also available, use it to further adjust
        if environment_data:
            # Example: rain, snow, fog conditions from environment data
            if environment_data.get('precipitation_type') == 'rain':
                weather_factor = min(weather_factor, 0.7)  # More conservative in rain
            elif environment_data.get('precipitation_type') == 'snow':
                weather_factor = min(weather_factor, 0.5)  # Much more conservative in snow
            elif environment_data.get('visibility') < 50:  # Low visibility
                weather_factor = min(weather_factor, 0.8)

        # Apply weather adjustments
        self.current_pid_params['kp'] *= weather_factor
        self.current_pid_params['ki'] *= weather_factor
        self.current_pid_params['kd'] *= weather_factor
        self.current_lat_params['max_steer'] *= weather_factor
        self.current_long_params['max_accel'] *= weather_factor * 0.8  # Even more conservative for acceleration
        self.current_long_params['min_accel'] = max(-2.5, self.current_long_params['min_accel'] * weather_factor)  # Less braking in bad weather

    def _adjust_for_road_surface(self, car_state: log.CarState, 
                               environment_data: Optional[Dict[str, Any]]) -> None:
        """Adjust parameters based on road surface conditions"""
        surface_factor = 1.0
        
        # Estimate road surface based on acceleration patterns and steering behavior
        if len(self.steering_history) > 10:
            # Look for signs of slippery conditions (high steering with low response)
            recent_steering_changes = np.diff(self.steering_history[-10:])
            avg_change = np.mean(np.abs(recent_steering_changes))
            
            # High steering activity with low car response might indicate slippery surface
            if avg_change > 0.1 and car_state.vEgo > 5:  # If moving and steering a lot
                surface_factor = 0.85  # More conservative
        
        # Apply surface adjustments
        self.current_pid_params['kp'] *= surface_factor * 0.9  # More conservative for unknown surfaces
        self.current_lat_params['max_steer'] *= surface_factor * 0.85
        # Reduce longitudinal aggressiveness on potentially slippery surfaces
        self.current_long_params['max_accel'] *= surface_factor * 0.8
        self.current_long_params['min_accel'] = max(-2.0, self.current_long_params['min_accel'] * surface_factor * 0.8)

    def _adjust_for_speed(self, car_state: log.CarState) -> None:
        """Adjust parameters based on vehicle speed"""
        v_ego = car_state.vEgo
        
        if v_ego < 5:  # Very low speed
            speed_factor = 1.2  # More responsive at low speed
        elif v_ego < 15:  # Moderate speed (~54 km/h)
            speed_factor = 1.0
        elif v_ego < 30:  # Higher speed (~108 km/h)
            speed_factor = 0.9  # More conservative
        else:  # High speed
            speed_factor = 0.8  # Most conservative
            
        # Apply speed-based adjustments
        self.current_pid_params['kp'] *= speed_factor
        self.current_pid_params['ki'] *= speed_factor
        self.current_lat_params['max_steer'] *= speed_factor
        # Adjust longitudinal parameters based on speed too
        if v_ego > 25:  # Above ~90 km/h
            self.current_long_params['max_accel'] *= 0.7  # More conservative acceleration
            self.current_long_params['min_accel'] = max(-3.0, self.current_long_params['min_accel'] * 0.8)

    def _adjust_for_traffic(self, car_state: log.CarState) -> None:
        """Adjust parameters based on traffic conditions"""
        traffic_factor = 1.0
        
        # If we have radar state data, use it for traffic awareness
        if self.sm and 'radarState' in self.sm:
            radar_state = self.sm['radarState']
            
            # Check for close lead vehicles
            if radar_state.leadOne.status and radar_state.leadOne.dRel < 50:  # Lead within 50m
                if radar_state.leadOne.dRel < 20:  # Very close
                    traffic_factor = 0.6  # Very conservative
                elif radar_state.leadOne.dRel < 35:  # Close
                    traffic_factor = 0.8  # More conservative
        
        # Apply traffic adjustments
        self.current_long_params['max_accel'] *= traffic_factor
        self.current_long_params['min_accel'] = max(
            -2.5, self.current_long_params['min_accel'] * traffic_factor
        )
        # Be more conservative laterally when close to other vehicles
        self.current_lat_params['max_steer'] *= traffic_factor * 0.9

    def _adjust_for_road_curvature(self, car_state: log.CarState) -> None:
        """Adjust parameters based on road curvature from model outputs"""
        curvature_factor = 1.0
        
        # If we have model outputs, check for high curvature sections
        if self.sm and 'modelV2' in self.sm:
            model_v2 = self.sm['modelV2']
            # Check if we're approaching a sharp curve
            if model_v2.position.y[10] > 5 or model_v2.position.y[20] > 8:  # Offsets at 1s and 2s
                curvature_factor = 0.8  # More conservative on sharp curves
            
        # Apply curvature adjustments
        self.current_lat_params['max_steer'] *= curvature_factor
        # Reduce longitudinal acceleration when on curves to maintain lateral grip
        self.current_long_params['max_accel'] *= curvature_factor * 0.9
        
    def _update_historical_data(self, car_state: log.CarState) -> None:
        """Update historical data for adaptive algorithms"""
        # Update steering history
        self.steering_history.append(car_state.steeringAngleDeg)
        if len(self.steering_history) > self.max_history:
            self.steering_history.pop(0)
        
        # Update acceleration history
        self.acceleration_history.append(car_state.aEgo)
        if len(self.acceleration_history) > self.max_history:
            self.acceleration_history.pop(0)
    
    def get_adaptive_parameters(self) -> Dict[str, Any]:
        """Get the currently adapted parameters"""
        return {
            'pid': self.current_pid_params,
            'lateral': self.current_lat_params,
            'longitudinal': self.current_long_params
        }
    
    def reset_to_defaults(self) -> None:
        """Reset all parameters to defaults"""
        self.current_pid_params = self.default_pid_params.copy()
        self.current_lat_params = self.default_lat_params.copy() 
        self.current_long_params = self.default_long_params.copy()
        self.steering_history = []
        self.acceleration_history = []


class AdaptiveLatControlTorque:
    """
    Enhanced torque-based lateral control with adaptive parameters
    """
    
    def __init__(self):
        self.adaptive_params = AdaptiveControlParameters()
        self.current_gains = self.adaptive_params.default_pid_params.copy()
        
    def update_gains_for_conditions(self, car_state: log.CarState, 
                                  environment_data: Optional[Dict[str, Any]] = None) -> None:
        """Update control gains based on current conditions"""
        params = self.adaptive_params.adjust_for_conditions(car_state, environment_data)
        self.current_gains = params['pid']
        
        # Apply the new gains to the controller
        # In a real implementation, this would update the actual controller gains
        cloudlog.debug(f"Updated lateral control gains: {self.current_gains}")
    
    def get_current_gains(self) -> Dict[str, float]:
        """Get current control gains"""
        return self.current_gains.copy()


# Global instances for use across the system
adaptive_control = AdaptiveControlParameters()
adaptive_lat_control = AdaptiveLatControlTorque()