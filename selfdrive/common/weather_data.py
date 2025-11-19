"""
Weather data interface for Sunnypilot
Provides weather information for adaptive control and safety systems
"""

import time
from typing import Dict, Any, Optional
from cereal import log
import cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params


class WeatherDataInterface:
    """
    Weather data interface that provides weather information for adaptive control
    In a real implementation, this would connect to weather APIs or sensors
    """
    
    def __init__(self):
        self.params = Params()
        self.last_update_time = 0
        self.weather_data = {
            'precipitation_type': 'none',  # 'none', 'rain', 'snow', 'sleet'
            'precipitation_intensity': 0.0,  # 0.0 to 1.0
            'temperature': 20.0,  # Celsius
            'visibility': 200.0,  # Meters
            'wind_speed': 5.0,  # m/s
            'road_surface_condition': 'dry',  # 'dry', 'wet', 'icy', 'snowy'
            'timestamp': 0
        }
        
        # Initialize messaging
        try:
            self.sm = messaging.SubMaster(['deviceState', 'carState'], ignore_alive=True)
        except Exception as e:
            cloudlog.warning(f"Could not initialize SubMaster in WeatherDataInterface: {e}")
            self.sm = None
    
    def get_weather_data(self, latitude: float = None, longitude: float = None) -> Dict[str, Any]:
        """
        Get current weather data (simulated in this implementation)
        In a real implementation, this would fetch from weather APIs
        """
        current_time = time.time()
        
        # Simulate weather changes based on conditions or time
        if current_time - self.last_update_time > 30:  # Update every 30 seconds
            self._simulate_weather_update(latitude, longitude)
            self.last_update_time = current_time
        
        return self.weather_data.copy()
    
    def _simulate_weather_update(self, latitude: float = None, longitude: float = None) -> None:
        """
        Simulate weather updates (in real implementation, this would fetch actual data)
        """
        # In a real implementation, this would fetch from weather APIs
        # For now, simulate based on car state or time
        if self.sm and 'deviceState' in self.sm:
            # Use device state to potentially determine location-based weather simulation
            pass
            
        # Update timestamp
        self.weather_data['timestamp'] = time.time()
        
        # Simulate some weather changes based on season/time of day concepts
        # This is just a placeholder for demonstration
        if time.time() % 300 < 10:  # Randomly simulate weather changes
            self.weather_data['precipitation_type'] = 'rain'
            self.weather_data['precipitation_intensity'] = 0.3
            self.weather_data['visibility'] = 100.0
            self.weather_data['road_surface_condition'] = 'wet'
        else:
            self.weather_data['precipitation_type'] = 'none'
            self.weather_data['precipitation_intensity'] = 0.0
            self.weather_data['visibility'] = 200.0
            self.weather_data['road_surface_condition'] = 'dry'
    
    def get_precipitation_factor(self) -> float:
        """Get a factor based on precipitation for control adjustments"""
        try:
            if self.weather_data['precipitation_type'] == 'rain':
                factor = 0.7 - (self.weather_data['precipitation_intensity'] * 0.3)  # More conservative in rain
                return max(0.3, factor)  # Ensure factor doesn't go too low
            elif self.weather_data['precipitation_type'] == 'snow':
                factor = 0.4 - (self.weather_data['precipitation_intensity'] * 0.3)  # Much more conservative in snow
                return max(0.1, factor)  # Ensure factor doesn't go too low
            else:
                return 1.0  # Normal conditions
        except (KeyError, TypeError):
            cloudlog.warning("Error accessing weather data, returning normal conditions factor")
            return 1.0  # Return normal conditions as fallback


# Global instance
weather_data_interface = WeatherDataInterface()