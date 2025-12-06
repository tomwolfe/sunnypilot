"""
Lightweight Integration Module for Snapdragon 845
Clean, modular integration of lightweight enhancements
"""

from typing import Dict, Any, Tuple
from sunnypilot.lightweight.enhancement_suite import (
    LightweightEnhancementSuite, create_light_enhancement_suite
)
import time


class LightweightIntegrator:
    """
    Clean integration class for lightweight enhancements only.
    Designed specifically for Snapdragon 845 hardware constraints.
    """
    
    def __init__(self):
        self.controller = create_light_enhancement_suite()
        self.last_update_time = time.time()
        
    def should_process_frame(self, frame):
        """
        Determine if the current frame should be processed based on scene detection.
        
        Args:
            frame: Current camera frame
            
        Returns:
            Tuple of (should_process, motion_level)
        """
        return self.controller.should_process_frame(frame)
    
    def adjust_acceleration_for_lateral_demand(self, 
                                             base_acceleration: float,
                                             lateral_demand: float,
                                             speed: float,
                                             curvature: float,
                                             radar_data: Dict[str, Any],
                                             car_state: Dict[str, Any]) -> float:
        """
        Adjust longitudinal acceleration based on lateral demand with safety limits.
        """
        return self.controller.adjust_acceleration_for_lateral_demand(
            base_acceleration, lateral_demand, speed, curvature, radar_data, car_state
        )
    
    def detect_edge_cases(self, radar_data: Dict[str, Any], 
                         vision_data: Dict[str, Any], 
                         car_state: Dict[str, Any]) -> Any:
        """
        Detect critical edge cases that require special handling.
        """
        return self.controller.detect_edge_cases(radar_data, vision_data, car_state)
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """
        Get performance statistics for the enhancement suite.
        """
        return self.controller.get_performance_stats()


def create_light_integrator() -> LightweightIntegrator:
    """
    Factory function to create the lightweight integrator.
    This is the recommended integration point for Snapdragon 845 systems.
    """
    return LightweightIntegrator()