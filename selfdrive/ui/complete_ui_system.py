"""
Complete Sunnypilot UI System with Real-time Data Integration
Combines all components into a cohesive UI system for the Comma Three device
"""
import pyray as rl
import time
from typing import Dict, Any, Optional, Callable
from dataclasses import dataclass

from cereal import messaging
from openpilot.system.ui.lib.application import gui_app, DEFAULT_FPS
from openpilot.selfdrive.ui.sunnypilot_ui import SunnypilotUI, DrivingState, Theme
from openpilot.selfdrive.ui.raylib_ui_system import RaylibUI, UIConfig
from openpilot.selfdrive.ui.data_integration import (
    DataIntegrationManager, 
    RealtimeMetricsCollector,
    SafetyDataProcessor,
    PerceptionDataProcessor,
    NavigationDataProcessor
)
from openpilot.selfdrive.ui.ui_state import ui_state


@dataclass
class SystemPerformance:
    """Container for system performance metrics"""
    cpu_usage: float = 0.0
    ram_usage: float = 0.0
    gpu_usage: float = 0.0
    power_draw: float = 0.0
    fps: float = 0.0
    render_time_ms: float = 0.0
    thermal_status: int = 0
    latency_ms: float = 0.0


class CompleteSunnypilotUISystem:
    """
    Complete UI system integrating all components with real-time data
    Optimized for the Comma Three hardware (2GB RAM, 4-core ARM CPU, 10W power budget)
    """
    
    def __init__(self):
        # Initialize configuration for Comma Three
        self.config = UIConfig(
            target_fps=30,  # Balance between smoothness and efficiency
            max_cpu_usage=5.0,
            resource_budget=50.0,  # MB
            screen_width=1280,
            screen_height=720
        )
        
        # Initialize data integration
        self.data_integration = DataIntegrationManager()
        self.metrics_collector = RealtimeMetricsCollector(self.data_integration)
        self.safety_processor = SafetyDataProcessor(self.data_integration)
        self.perception_processor = PerceptionDataProcessor(self.data_integration)
        self.navigation_processor = NavigationDataProcessor(self.data_integration)
        
        # Initialize UI system
        self.raylib_ui = RaylibUI(self.config)
        
        # Performance tracking
        self.performance = SystemPerformance()
        self.last_update_time = 0.0
        self.frame_count = 0
        self.last_fps_update = time.time()
        
        # Resource management
        self.cpu_usage_history = []
        self.max_cpu_history = 50  # Keep last 50 readings
        
        print("Complete Sunnypilot UI System initialized")
    
    def update(self):
        """Update the entire UI system with real-time data"""
        current_time = time.time()
        start_time = time.time()
        
        # Update data integration
        self.data_integration.update()
        
        # Update all data processors
        self.metrics_collector.update_metrics()
        safety_data = self.safety_processor.update_safety_data()
        perception_data = self.perception_processor.update_perception_data()
        navigation_data = self.navigation_processor.update_navigation_data()
        
        # Update performance metrics
        self._update_performance_metrics()
        
        # Update UI system with processed data
        self.raylib_ui.update()
        
        # Update frame time
        frame_time = time.time() - start_time
        self.metrics_collector.update_fps(frame_time)
        
        # Check resource usage and adjust if needed
        self._check_resource_usage()
        
        # Update FPS counter
        self.frame_count += 1
        if current_time - self.last_fps_update > 1.0:
            self.performance.fps = self.frame_count / (current_time - self.last_fps_update)
            self.frame_count = 0
            self.last_fps_update = current_time
    
    def _update_performance_metrics(self):
        """Update performance metrics from data integration"""
        current_metrics = self.metrics_collector.get_current_metrics()
        
        self.performance.cpu_usage = current_metrics.get('cpu', 0)
        self.performance.ram_usage = current_metrics.get('ram', 0)
        self.performance.power_draw = current_metrics.get('power', 0)
        self.performance.render_time_ms = current_metrics.get('render_time_ms', 0)
        self.performance.latency_ms = current_metrics.get('latency', 0) * 1000  # Convert to ms
        
        # Add to CPU history for trend analysis
        self.cpu_usage_history.append(self.performance.cpu_usage)
        if len(self.cpu_usage_history) > self.max_cpu_history:
            self.cpu_usage_history.pop(0)
    
    def _check_resource_usage(self):
        """Check resource usage and take corrective actions if needed"""
        # Check if we're approaching resource limits
        if self.performance.cpu_usage > 80:
            # Activate compact mode to save resources
            self.raylib_ui.toggle_compact_mode()
        elif self.performance.cpu_usage < 40 and self.raylib_ui.state_manager.compact_mode:
            # Go back to normal mode if resources allow
            self.raylib_ui.toggle_compact_mode()
    
    def render(self, rect: rl.Rectangle):
        """Render the complete UI system"""
        start_time = time.time()
        
        # Render UI system
        self.raylib_ui.render(rect)
        
        # Calculate render time
        render_time = time.time() - start_time
        self.performance.render_time_ms = render_time * 1000  # Convert to milliseconds
        
        # Add performance overlay if needed
        self._render_performance_overlay(rect)
    
    def _render_performance_overlay(self, rect: rl.Rectangle):
        """Render performance metrics overlay"""
        # Only show if debug mode is enabled
        if not self.raylib_ui.state_manager.debug_mode:
            return
        
        # Draw performance info in top-left corner
        x, y = 10, 10
        line_height = 20
        text_size = 14
        
        # Background for performance overlay
        bg_color = rl.Color(0, 0, 0, 180)
        border_color = rl.Color(100, 100, 120, 200)
        
        # Calculate height based on number of metrics
        num_metrics = 6
        height = num_metrics * line_height + 20
        rl.draw_rectangle(int(x), int(y), 220, int(height), bg_color)
        rl.draw_rectangle_lines(int(x), int(y), 220, int(height), border_color)
        
        # Draw metrics
        metrics = [
            f"CPU: {self.performance.cpu_usage:.1f}%",
            f"RAM: {self.performance.ram_usage:.1f}%",
            f"Power: {self.performance.power_draw:.2f}W",
            f"FPS: {self.performance.fps:.1f}",
            f"Render: {self.performance.render_time_ms:.1f}ms",
            f"Latency: {self.performance.latency_ms:.1f}ms"
        ]
        
        text_color = rl.Color(200, 220, 255, 255)
        for i, metric in enumerate(metrics):
            rl.draw_text(metric, int(x + 10), int(y + 10 + i * line_height), text_size, text_color)
    
    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system status"""
        return {
            'performance': self.performance,
            'safety': self.safety_processor.update_safety_data(),
            'perception': self.perception_processor.update_perception_data(),
            'navigation': self.navigation_processor.update_navigation_data(),
            'connectivity': self.data_integration.get_service_connectivity(),
            'data_freshness': self.data_integration.get_data_freshness()
        }
    
    def is_safe_to_engage(self) -> bool:
        """Check if it's safe to engage autonomous driving"""
        safety_data = self.safety_processor.update_safety_data()
        return safety_data.get('allow_engage', False) and safety_data.get('safety_validated', False)
    
    def get_driving_recommendation(self) -> str:
        """Get driving recommendation based on current data"""
        safety_data = self.safety_processor.update_safety_data()
        perception_data = self.perception_processor.update_perception_data()
        
        if not safety_data.get('allow_engage', False):
            return "SAFETY ISSUE: Do not engage"
        elif not perception_data.get('free_path_available', True):
            return "OBSTACLE DETECTED: Check surroundings"
        elif perception_data.get('closest_object_distance', float('inf')) < 30.0:  # 30m
            return "CAUTION: Object ahead"
        elif safety_data.get('safety_level', 5) < 3:
            return "SAFETY LEVEL LOW: Exercise caution"
        else:
            return "SAFE TO DRIVE"


def run_ui_system():
    """Run the complete UI system"""
    print("Starting Complete Sunnypilot UI System...")
    print("="*50)
    print(f"Resolution: {1280}x{720}")
    print(f"Target FPS: {30}")
    print(f"Hardware: Comma Three (2GB RAM, 4-core ARM)")
    print("Press ESC to exit, D for debug mode, C for compact mode")
    print("="*50)
    
    # Initialize the complete system
    ui_system = CompleteSunnypilotUISystem()
    
    # Initialize raylib window
    rl.init_window(1280, 720, "Sunnypilot UI System")
    rl.set_target_fps(30)
    
    try:
        while not rl.window_should_close():
            # Update the UI system
            ui_system.update()
            
            # Begin drawing
            rl.begin_drawing()
            rl.clear_background(rl.BLACK)
            
            # Render the UI
            screen_rect = rl.Rectangle(0, 0, 1280, 720)
            ui_system.render(screen_rect)
            
            # Handle user input
            if rl.is_key_pressed(rl.KeyboardKey.KEY_ESCAPE):
                break
            elif rl.is_key_pressed(rl.KeyboardKey.KEY_D):
                ui_system.raylib_ui.enable_debug_mode(
                    not ui_system.raylib_ui.state_manager.debug_mode
                )
            elif rl.is_key_pressed(rl.KeyboardKey.KEY_C):
                ui_system.raylib_ui.toggle_compact_mode()
            
            # End drawing
            rl.end_drawing()
    
    except KeyboardInterrupt:
        print("\nUI System shutdown requested")
    finally:
        # Cleanup
        rl.close_window()
        print("Complete Sunnypilot UI System shutdown")


def benchmark_ui_system():
    """Benchmark the UI system performance"""
    print("Benchmarking Sunnypilot UI System...")
    
    ui_system = CompleteSunnypilotUISystem()
    start_time = time.time()
    
    # Run for 10 seconds
    run_duration = 10.0
    frame_count = 0
    
    while time.time() - start_time < run_duration:
        ui_system.update()
        
        # Create a dummy rectangle for rendering test
        dummy_rect = rl.Rectangle(0, 0, 1280, 720)
        ui_system.render(dummy_rect)
        
        frame_count += 1
    
    elapsed_time = time.time() - start_time
    avg_fps = frame_count / elapsed_time
    
    print(f"Performance Results:")
    print(f"  Duration: {elapsed_time:.2f}s")
    print(f"  Frames: {frame_count}")
    print(f"  Average FPS: {avg_fps:.2f}")
    print(f"  Target FPS: 30")
    print(f"  Performance: {'GOOD' if avg_fps >= 25 else 'POOR'}")
    
    # Check resource usage
    status = ui_system.get_system_status()
    perf = status['performance']
    print(f"Resource Usage:")
    print(f"  CPU: {perf.cpu_usage:.1f}%")
    print(f"  Power: {perf.power_draw:.2f}W")
    print(f"  Max Render Time: {perf.render_time_ms:.1f}ms")
    print(f"  Resource Usage: {'ACCEPTABLE' if perf.cpu_usage < 5 else 'HIGH'}")


def main():
    """Main entry point"""
    print("Sunnypilot Complete UI System")
    print("1. Run UI System")
    print("2. Benchmark Performance")
    print("3. Check System Status")
    choice = input("Select option (1-3): ").strip()
    
    if choice == "1":
        run_ui_system()
    elif choice == "2":
        benchmark_ui_system()
    elif choice == "3":
        # Check system status
        ui_system = CompleteSunnypilotUISystem()
        ui_system.update()  # Update once to get current data
        
        status = ui_system.get_system_status()
        perf = status['performance']
        safety = status['safety']
        
        print("\nSystem Status:")
        print(f"  Performance - CPU: {perf.cpu_usage:.1f}%, FPS: {perf.fps:.1f}")
        print(f"  Safety Level: {safety.get('safety_level', 0)}/5")
        print(f"  Safety Validated: {safety.get('safety_validated', False)}")
        print(f"  Safe to Engage: {ui_system.is_safe_to_engage()}")
        print(f"  Driving Recommendation: {ui_system.get_driving_recommendation()}")
        print(f"  Connected Services: {sum(1 for v in status['connectivity'].values() if v)}/{len(status['connectivity'])}")
    else:
        print("Invalid choice")


if __name__ == "__main__":
    main()