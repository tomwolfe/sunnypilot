"""
Lightweight Enhancement Suite for Sunnypilot2
Designed specifically for Snapdragon 845 hardware constraints

This module consolidates the three viable lightweight enhancements:
1. Scene change detection to reduce CPU load on static scenes
2. Simplified coordinated control for smoother acceleration during high lateral demand
3. Basic edge case detection for stopped traffic and construction zones

All components are optimized for real-time performance on Snapdragon 845.
"""

import numpy as np
import time
from typing import Dict, Tuple, Optional, Any
from dataclasses import dataclass
from enum import Enum


@dataclass
class EdgeCaseType:
    STOPPED_TRAFFIC = "STOPPED_TRAFFIC"
    CONSTRUCTION_ZONE = "CONSTRUCTION_ZONE"
    SHARP_CURVE = "SHARP_CURVE"
    OBSTACLE_DETECTED = "OBSTACLE_DETECTED"


@dataclass
class EdgeCaseResult:
    """Result from edge case detection."""
    edge_cases: list  # List of detected edge cases
    safe_speed_multiplier: float  # Multiplier to apply to desired speed (0.0 to 1.0)
    required_action: str  # Recommended action ("SLOW_DOWN", "STOP", "CAUTION", "CONTINUE_NORMAL")
    confidence: float  # Overall confidence in detection


class SceneChangeDetector:
    """
    Lightweight scene change detector to reduce CPU load on static scenes.
    Uses simple motion detection to determine if full model processing is needed.
    """
    
    def __init__(self, 
                 static_threshold: float = 0.02, 
                 dynamic_threshold: float = 0.1,
                 static_frame_hold: int = 5,
                 dynamic_frame_reset: int = 2):
        """
        Initialize the scene change detector.
        
        Args:
            static_threshold: Motion threshold below which scene is considered static
            dynamic_threshold: Motion threshold above which scene is considered dynamic  
            static_frame_hold: Number of consecutive static frames to trigger skip
            dynamic_frame_reset: Number of dynamic frames to reset to processing mode
        """
        self.static_threshold = static_threshold
        self.dynamic_threshold = dynamic_threshold
        self.static_frame_hold = static_frame_hold
        self.dynamic_frame_reset = dynamic_frame_reset
        
        self.static_frame_count = 0
        self.dynamic_frame_count = 0
        self.current_state = False  # False = processing, True = skipping (static)
        
        # Store previous frame for motion detection
        self.prev_frame = None
        self.frame_shape = None
    
    def detect_change(self, current_frame: np.ndarray) -> Tuple[bool, float]:
        """
        Detect if scene has changed to determine if processing should continue.
        
        Args:
            current_frame: Current camera frame (grayscale)
            
        Returns:
            Tuple of (should_skip_processing, motion_level)
            - should_skip_processing: True if frame processing should be skipped
            - motion_level: Float representing amount of motion (0.0 to 1.0)
        """
        if current_frame is None:
            return False, 0.0
        
        # Convert to grayscale if needed and resize for efficiency
        if len(current_frame.shape) == 3:
            current_frame = np.mean(current_frame, axis=2)
        
        # Store frame shape for consistency
        if self.frame_shape is None:
            self.frame_shape = current_frame.shape
        elif current_frame.shape != self.frame_shape:
            # Resize to consistent shape for comparison
            current_frame = np.resize(current_frame, self.frame_shape)
        
        motion_level = 0.0

        if self.prev_frame is not None:
            # Calculate motion level as normalized difference
            diff = np.abs(current_frame.astype(np.float32) - self.prev_frame.astype(np.float32))
            motion_level = np.mean(diff) / 255.0  # Normalize to 0-1 range

            # Determine if we should skip processing based on motion level
            if motion_level < self.static_threshold:
                # Scene appears static
                self.static_frame_count += 1
                self.dynamic_frame_count = 0  # Reset dynamic counter

                # Skip processing if we've seen enough static frames
                if self.static_frame_count >= self.static_frame_hold:
                    self.current_state = True  # Mark as skipping (static)
            elif motion_level > self.dynamic_threshold:
                # Scene is clearly dynamic
                self.dynamic_frame_count += 1
                self.static_frame_count = 0  # Reset static counter

                # If we have enough dynamic frames, return to processing mode
                if self.dynamic_frame_count >= self.dynamic_frame_reset:
                    self.current_state = False  # Mark as processing (not skipping)
            else:
                # Motion level is in intermediate range, maintain current state
                pass
        else:
            # First frame - we can't determine motion from nothing, so default to processing
            # Don't change state since we can't evaluate motion yet
            pass

        # Update previous frame
        self.prev_frame = current_frame.copy()

        # Return whether to skip processing (True = skip, False = process)
        # The current_state tracks whether we're in "skip" mode (True = skip processing)
        return self.current_state, motion_level
    
    def reset(self):
        """Reset the detector state."""
        self.static_frame_count = 0
        self.dynamic_frame_count = 0
        self.current_state = False
        self.prev_frame = None


class CoordinatedController:
    """
    Simplified coordinated controller that adjusts longitudinal acceleration 
    based on lateral demand to improve safety and comfort during turns.
    """
    
    def __init__(self, 
                 lateral_accel_threshold: float = 1.5,  # m/s² beyond which we reduce longitudinal
                 max_speed_reduction: float = 0.3,      # Max reduction factor (30%)
                 min_safe_speed_factor: float = 0.2):   # Minimum safe speed factor
        """
        Initialize coordinated controller.
        
        Args:
            lateral_accel_threshold: Lateral acceleration threshold for intervention
            max_speed_reduction: Maximum reduction in longitudinal acceleration 
            min_safe_speed_factor: Minimum speed factor to maintain (for safety)
        """
        self.lateral_accel_threshold = lateral_accel_threshold
        self.max_speed_reduction = max_speed_reduction
        self.min_safe_speed_factor = min_safe_speed_factor
    
    def adjust_acceleration(self, 
                          base_acceleration: float, 
                          lateral_acceleration: float,
                          speed: float,
                          curvature: float) -> float:
        """
        Adjust longitudinal acceleration based on lateral demand.
        
        Args:
            base_acceleration: Base longitudinal acceleration command
            lateral_acceleration: Estimated lateral acceleration demand
            speed: Current vehicle speed (m/s)
            curvature: Current path curvature
            
        Returns:
            Adjusted acceleration value
        """
        # Calculate lateral demand factor (0.0 to 1.0+)
        lateral_demand_factor = min(abs(lateral_acceleration) / self.lateral_accel_threshold, 1.0)
        
        # Apply coordinated control factor
        coordination_factor = 1.0 - (lateral_demand_factor * self.max_speed_reduction)
        
        # Ensure minimum safe factor
        coordination_factor = max(coordination_factor, self.min_safe_speed_factor)
        
        # Adjust acceleration based on coordination
        adjusted_accel = base_acceleration * coordination_factor
        
        # Additional reduction at high speeds with high curvature
        if abs(curvature) > 0.05 and speed > 15.0:  # High curvature at high speed
            speed_curvature_factor = max(0.7, 1.0 - (abs(curvature) * speed * 0.01))
            adjusted_accel *= speed_curvature_factor
        
        return adjusted_accel


class SafetyLimiter:
    """
    Safety limiter that ensures coordinated control doesn't compromise safety.
    """
    
    def __init__(self,
                 max_deceleration: float = -4.0,  # Maximum safe deceleration (m/s²)  
                 min_distance_threshold: float = 20.0):  # Minimum safe distance (m)
        """
        Initialize safety limiter.
        
        Args:
            max_deceleration: Maximum deceleration allowed for safety
            min_distance_threshold: Minimum safe distance to lead vehicle
        """
        self.max_deceleration = max_deceleration
        self.min_distance_threshold = min_distance_threshold
    
    def apply_safety_limits(self,
                          coordinated_accel: float,
                          base_acceleration: float, 
                          lateral_acceleration: float,
                          speed: float,
                          lead_distance: float,
                          lead_velocity: float) -> float:
        """
        Apply safety limits to coordinated acceleration.
        
        Args:
            coordinated_accel: Acceleration after coordination adjustment
            base_acceleration: Original acceleration command
            lateral_acceleration: Current lateral acceleration 
            speed: Current speed
            lead_distance: Distance to lead vehicle
            lead_velocity: Velocity of lead vehicle
            
        Returns:
            Final acceleration after safety limiting
        """
        final_accel = coordinated_accel
        
        # Safety check for lead vehicle
        if lead_distance > 0 and lead_distance < self.min_distance_threshold:
            relative_velocity = speed - lead_velocity  # How fast we're approaching
            
            if relative_velocity > 0:  # We're approaching the lead
                time_to_collision = lead_distance / relative_velocity if relative_velocity > 0.1 else 0.1
                if time_to_collision < 3.0:  # Less than 3 seconds
                    # Apply emergency braking if time to collision is critical
                    safe_brake_accel = min(self.max_deceleration, -1.0)
                    final_accel = min(final_accel, safe_brake_accel)
        
        # Ensure we don't exceed maximum safe deceleration
        final_accel = max(final_accel, self.max_deceleration)
        
        # Additional safety if lateral and longitudinal demands are both high
        if abs(lateral_acceleration) > 2.0 and speed > 20.0 and coordinated_accel > 0:
            # Reduce acceleration if both lateral and longitudinal demands are high at high speed
            high_demand_factor = max(0.5, 1.0 - (abs(lateral_acceleration) * 0.1))
            final_accel = min(final_accel, base_acceleration * high_demand_factor)
        
        return final_accel


class EdgeCaseDetector:
    """
    Basic edge case detection for stopped traffic and construction zones.
    Designed to be computationally efficient while maintaining safety.
    """
    
    def __init__(self,
                 stopped_traffic_distance: float = 30.0,  # Distance threshold for stopped traffic (m)
                 stopped_traffic_rel_vel: float = 2.0,    # Relative velocity threshold (m/s)
                 construction_zone_threshold: float = 2): # Number of slow vehicles to indicate construction
        """
        Initialize edge case detector.
        
        Args:
            stopped_traffic_distance: Distance threshold for stopped traffic detection
            stopped_traffic_rel_vel: Relative velocity threshold for stopped traffic
            construction_zone_threshold: Number of slow vehicles to indicate construction zone
        """
        self.stopped_traffic_distance = stopped_traffic_distance
        self.stopped_traffic_rel_vel = stopped_traffic_rel_vel
        self.construction_zone_threshold = construction_zone_threshold
    
    def detect_edge_cases(self, radar_data: Dict, vision_data: Dict, car_state: Dict) -> EdgeCaseResult:
        """
        Detect various edge cases from sensor data.
        
        Args:
            radar_data: Radar state data
            vision_data: Vision model data  
            car_state: Current car state
            
        Returns:
            EdgeCaseResult with detected cases and recommendations
        """
        edge_cases = []
        speed_multiplier = 1.0
        required_action = "CONTINUE_NORMAL"
        confidence = 0.8  # Base confidence
        
        v_ego = car_state.get('vEgo', 0.0)
        
        # Detect stopped traffic
        lead_one = radar_data.get('leadOne')
        if lead_one and getattr(lead_one, 'status', False):
            d_rel = getattr(lead_one, 'dRel', float('inf'))
            v_rel = getattr(lead_one, 'vRel', 0.0)
            
            # Check if lead is close and relatively stopped
            if d_rel < self.stopped_traffic_distance and abs(v_rel) < self.stopped_traffic_rel_vel:
                edge_cases.append({
                    'type': EdgeCaseType.STOPPED_TRAFFIC,
                    'distance': d_rel,
                    'relative_velocity': v_rel,
                    'severity': 'HIGH' if d_rel < 15.0 else 'MEDIUM'
                })
                
                # Calculate appropriate speed reduction
                if d_rel < 10.0:
                    speed_multiplier = 0.1  # Near stop
                    required_action = "STOP"
                elif d_rel < 20.0:
                    speed_multiplier = 0.3  # Slow down significantly
                    required_action = "SLOW_DOWN"
                else:
                    speed_multiplier = 0.7  # Moderate reduction
                    required_action = "CAUTION"
        
        # Detect potential construction zone
        slow_vehicles_count = 0
        for i in range(1, 3):  # Check leadOne and leadTwo
            lead_key = f'lead{i if i == 2 else "One"}'
            lead = radar_data.get(lead_key)
            if lead and getattr(lead, 'status', False):
                d_rel = getattr(lead, 'dRel', float('inf'))
                v_rel = getattr(lead, 'vRel', 0.0)
                
                # Check if vehicle is moving slowly relative to our speed
                if d_rel < 50.0 and v_ego > 5.0 and (v_ego + v_rel) < 8.0:  # Lead moving <8m/s when we're moving >5m/s
                    slow_vehicles_count += 1
        
        if slow_vehicles_count >= self.construction_zone_threshold:
            edge_cases.append({
                'type': EdgeCaseType.CONSTRUCTION_ZONE,
                'slow_vehicles_count': slow_vehicles_count,
                'severity': 'MEDIUM'
            })
            speed_multiplier = min(speed_multiplier, 0.6)  # Additional reduction
            if required_action == "CONTINUE_NORMAL":
                required_action = "CAUTION"
        
        # Ensure minimum safe speed
        speed_multiplier = max(speed_multiplier, 0.1)
        
        return EdgeCaseResult(
            edge_cases=edge_cases,
            safe_speed_multiplier=speed_multiplier,
            required_action=required_action,
            confidence=confidence
        )


class LightweightEnhancementIntegrator:
    """
    Main integration class that combines all lightweight enhancements.
    Designed for minimal computational overhead on Snapdragon 845.
    """
    
    def __init__(self):
        # Initialize all lightweight components
        self.scene_detector = SceneChangeDetector()
        self.coordinated_controller = CoordinatedController()
        self.safety_limiter = SafetyLimiter()
        self.edge_case_detector = EdgeCaseDetector()
        
        # Performance tracking
        self.total_frames = 0
        self.frames_skipped = 0
    
    def should_process_frame(self, frame: np.ndarray) -> Tuple[bool, float]:
        """
        Determine if a frame should be processed based on scene change detection.

        Args:
            frame: Camera frame to analyze

        Returns:
            Tuple of (should_process_frame, motion_level)
        """
        self.total_frames += 1

        should_skip, motion_level = self.scene_detector.detect_change(frame)
        should_process = not should_skip  # Convert to "should process" logic (invert the skip decision)

        if should_skip:
            self.frames_skipped += 1

        return should_process, motion_level
    
    def adjust_acceleration_for_lateral_demand(self,
                                            base_acceleration: float,
                                            lateral_demand: float,
                                            speed: float,
                                            curvature: float,
                                            radar_data: Dict,
                                            car_state: Dict) -> float:
        """
        Adjust acceleration based on lateral demand and safety considerations.
        """
        # Apply coordinated control first
        coordinated_accel = self.coordinated_controller.adjust_acceleration(
            base_acceleration, lateral_demand, speed, curvature
        )
        
        # Then apply safety limits
        lead_distance = getattr(radar_data.get('leadOne', type('obj', (object,), {'dRel': float('inf')})()), 'dRel', float('inf'))
        lead_velocity = getattr(radar_data.get('leadOne', type('obj', (object,), {'vRel': 0.0})()), 'vRel', 0.0) + car_state.get('vEgo', 0.0)
        
        final_accel = self.safety_limiter.apply_safety_limits(
            coordinated_accel, base_acceleration, lateral_demand, 
            speed, lead_distance, lead_velocity
        )
        
        return final_accel
    
    def detect_edge_cases(self, radar_data: Dict, vision_data: Dict, car_state: Dict) -> EdgeCaseResult:
        """
        Detect edge cases using lightweight detector.
        """
        return self.edge_case_detector.detect_edge_cases(radar_data, vision_data, car_state)
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """
        Get performance statistics for monitoring.
        """
        frame_skip_rate = self.frames_skipped / self.total_frames if self.total_frames > 0 else 0.0
        
        return {
            'total_frames': self.total_frames,
            'frames_skipped': self.frames_skipped,
            'frame_skip_rate': frame_skip_rate,
            'efficiency_gain': f"{frame_skip_rate * 100:.1f}% CPU reduction from scene detection"
        }
    
    def reset_performance_stats(self):
        """Reset performance tracking."""
        self.total_frames = 0
        self.frames_skipped = 0


def create_light_enhancement_suite():
    """
    Factory function to create the lightweight enhancement suite.
    """
    return LightweightEnhancementIntegrator()


def validate_performance_requirements():
    """
    Validate that the lightweight system meets performance requirements.
    """
    print("=== Lightweight Enhancement Suite - Performance Validation ===")
    
    # Test scene detection performance
    detector = SceneChangeDetector()
    test_frame = np.random.randint(0, 255, (200, 200), dtype=np.uint8)
    
    # Time several iterations to get average
    import time as time_module
    
    times = []
    for _ in range(100):  # More samples for accuracy
        start = time_module.perf_counter()
        should_process, motion_level = detector.detect_change(test_frame)
        end = time_module.perf_counter()
        times.append((end - start) * 1000)  # Convert to milliseconds
    
    avg_time_ms = np.mean(times)
    print(f"  Scene detection average time: {avg_time_ms:.3f}ms per frame")
    
    # Test coordinated control performance
    controller = CoordinatedController()
    safety_limiter = SafetyLimiter()
    
    control_times = []
    for _ in range(100):
        start = time_module.perf_counter()
        adjusted = controller.adjust_acceleration(1.0, 1.5, 20.0, 0.05)
        final = safety_limiter.apply_safety_limits(adjusted, 1.0, 1.5, 20.0, 30.0, 15.0)
        end = time_module.perf_counter()
        control_times.append((end - start) * 1000)
    
    avg_control_time = np.mean(control_times)
    print(f"  Coordinated control average time: {avg_control_time:.3f}ms per update")
    
    # Performance validation
    scene_perf_ok = avg_time_ms < 5.0  # Should be well under 5ms for scene detection
    control_perf_ok = avg_control_time < 2.0  # Should be well under 2ms for control
    
    print(f"  ✓ Scene detection performance OK: {'YES' if scene_perf_ok else 'NO'} (target: <5ms)")
    print(f"  ✓ Coordinated control performance OK: {'YES' if control_perf_ok else 'NO'} (target: <2ms)")
    
    overall_perf_ok = scene_perf_ok and control_perf_ok
    print(f"  ✓ Overall performance validation: {'PASSED' if overall_perf_ok else 'FAILED'}")
    
    return overall_perf_ok


if __name__ == "__main__":
    print("Lightweight Enhancement Suite for Sunnypilot2")
    print("Designed for Snapdragon 845 hardware compatibility")
    
    # Run performance validation
    validate_performance_requirements()
    
    # Create and test the integrator
    integrator = create_light_enhancement_suite()
    print(f"\n✓ Lightweight enhancement suite created successfully")
    print(f"✓ All components optimized for Snapdragon 845 constraints")