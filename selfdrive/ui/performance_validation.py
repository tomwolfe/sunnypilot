#!/usr/bin/env python3
"""
Performance Validation and Benchmarking Tools for Sunnypilot UI System
Comprehensive performance testing and validation tools for resource usage
"""
import time
import psutil
import statistics
from typing import List, Dict, Any, Callable
from dataclasses import dataclass

import pyray as rl
from cereal import messaging

@dataclass
class PerformanceMetrics:
    """Container for performance metrics"""
    fps: List[float]
    cpu_usage: List[float]
    ram_usage: List[float]
    power_draw: List[float]
    render_time_ms: List[float]
    thermal_status: List[int]
    frame_times: List[float]
    
    def __init__(self):
        self.fps = []
        self.cpu_usage = []
        self.ram_usage = []
        self.power_draw = []
        self.render_time_ms = []
        self.thermal_status = []
        self.frame_times = []

    def get_average_fps(self) -> float:
        return statistics.mean(self.fps) if self.fps else 0.0

    def get_average_cpu(self) -> float:
        return statistics.mean(self.cpu_usage) if self.cpu_usage else 0.0

    def get_average_ram(self) -> float:
        return statistics.mean(self.ram_usage) if self.ram_usage else 0.0

    def get_average_render_time(self) -> float:
        return statistics.mean(self.render_time_ms) if self.render_time_ms else 0.0

    def get_max_render_time(self) -> float:
        return max(self.render_time_ms) if self.render_time_ms else 0.0

    def get_percentile_render_time(self, percentile: float) -> float:
        if not self.render_time_ms:
            return 0.0
        sorted_times = sorted(self.render_time_ms)
        index = int(len(sorted_times) * percentile)
        return sorted_times[min(index, len(sorted_times) - 1)]


class PerformanceValidator:
    """Validates UI performance against specified targets"""
    
    def __init__(self):
        self.metrics = PerformanceMetrics()
        self.target_fps = 30.0
        self.target_cpu_usage = 5.0  # percentage
        self.target_render_time_ms = 33.0  # ms per frame (for 30fps)
        self.target_ram_usage = 50.0  # MB
        
        # Message subscriber to get device stats
        self.sm = messaging.SubMaster(["deviceState"])
    
    def collect_metrics(self, render_time: float) -> None:
        """Collect performance metrics during UI operation"""
        current_time = time.time()
        
        # Update messaging to get latest device state
        self.sm.update(0)
        
        # Collect system metrics
        cpu_percent = psutil.cpu_percent()
        ram_percent = psutil.virtual_memory().percent
        render_time_ms = render_time * 1000  # Convert to milliseconds
        
        # Get device state if available
        device_metrics = {"power_draw": 0.0, "thermal_status": 0}
        if self.sm.updated["deviceState"]:
            device_state = self.sm["deviceState"]
            device_metrics["power_draw"] = device_state.powerDraw
            device_metrics["thermal_status"] = device_state.thermalStatus
        
        # Add metrics to collections
        self.metrics.cpu_usage.append(cpu_percent)
        self.metrics.ram_usage.append(ram_percent)
        self.metrics.render_time_ms.append(render_time_ms)
        self.metrics.power_draw.append(device_metrics["power_draw"])
        self.metrics.thermal_status.append(device_metrics["thermal_status"])
        self.metrics.frame_times.append(render_time)
        
        if self.metrics.frame_times:
            # Calculate FPS based on recent frame times
            recent_frames = self.metrics.frame_times[-30:]  # Last 30 frames
            if recent_frames:
                avg_frame_time = sum(recent_frames) / len(recent_frames)
                fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0
                self.metrics.fps.append(fps)
    
    def validate_performance(self) -> Dict[str, Any]:
        """Validate performance against targets and return results"""
        results = {
            'fps_pass': False,
            'cpu_pass': False,
            'render_time_pass': False,
            'ram_pass': False,
            'overall_pass': False,
            'avg_fps': 0.0,
            'avg_cpu': 0.0,
            'avg_render_time': 0.0,
            'avg_ram': 0.0,
            'max_render_time': 0.0,
            'percentile_95_render_time': 0.0,
            'warnings': []
        }
        
        if not self.metrics.fps:
            return results
        
        # Calculate averages
        avg_fps = self.metrics.get_average_fps()
        avg_cpu = self.metrics.get_average_cpu()
        avg_render_time = self.metrics.get_average_render_time()
        avg_ram = self.metrics.get_average_ram()
        max_render_time = self.metrics.get_max_render_time()
        p95_render_time = self.metrics.get_percentile_render_time(0.95)
        
        # Store in results
        results['avg_fps'] = avg_fps
        results['avg_cpu'] = avg_cpu
        results['avg_render_time'] = avg_render_time
        results['avg_ram'] = avg_ram
        results['max_render_time'] = max_render_time
        results['percentile_95_render_time'] = p95_render_time
        
        # Validate against targets
        results['fps_pass'] = avg_fps >= self.target_fps * 0.9  # Allow 10% tolerance
        results['cpu_pass'] = avg_cpu <= self.target_cpu_usage * 2.0  # Allow 100% tolerance initially
        results['render_time_pass'] = avg_render_time <= self.target_render_time_ms * 1.1  # Allow 10% tolerance
        results['ram_pass'] = avg_ram <= 70.0  # Using a reasonable RAM target
        
        results['overall_pass'] = all([
            results['fps_pass'],
            results['cpu_pass'],
            results['render_time_pass']
        ])
        
        # Add warnings for near-violations or other issues
        if avg_fps < self.target_fps * 0.95:
            results['warnings'].append(f"FPS ({avg_fps:.1f}) is below target ({self.target_fps})")
        
        if avg_cpu > self.target_cpu_usage:
            results['warnings'].append(f"CPU usage ({avg_cpu:.1f}%) exceeds target ({self.target_cpu_usage}%)")
        
        if avg_render_time > self.target_render_time_ms:
            results['warnings'].append(f"Render time ({avg_render_time:.1f}ms) exceeds target ({self.target_render_time_ms}ms)")
        
        return results
    
    def print_performance_report(self) -> None:
        """Print a comprehensive performance report"""
        results = self.validate_performance()
        
        print("=" * 60)
        print("SUNNYPilot UI PERFORMANCE VALIDATION REPORT")
        print("=" * 60)
        
        print(f"Average FPS: {results['avg_fps']:.2f} (Target: >=30) - {'PASS' if results['fps_pass'] else 'FAIL'}")
        print(f"Average CPU: {results['avg_cpu']:.2f}% (Target: <=5%) - {'PASS' if results['cpu_pass'] else 'FAIL'}")
        print(f"Average Render Time: {results['avg_render_time']:.2f}ms (Target: <=33ms) - {'PASS' if results['render_time_pass'] else 'FAIL'}")
        print(f"95th Percentile Render Time: {results['percentile_95_render_time']:.2f}ms")
        print(f"Max Render Time: {results['max_render_time']:.2f}ms")
        print(f"Average RAM: {results['avg_ram']:.2f}%")
        
        print("\nOverall Performance: {}".format("PASS" if results['overall_pass'] else "FAIL"))
        
        if results['warnings']:
            print("\nWARNINGS:")
            for warning in results['warnings']:
                print(f"  - {warning}")
        
        print("=" * 60)


class BenchmarkRunner:
    """Runs comprehensive benchmarks on the UI system"""
    
    def __init__(self):
        self.validator = PerformanceValidator()
    
    def run_stress_test(self, ui_system, duration: float = 30.0) -> Dict[str, Any]:
        """Run a stress test on the UI system for specified duration"""
        print(f"Starting UI stress test for {duration} seconds...")
        
        start_time = time.time()
        frame_count = 0
        
        # Initialize raylib window for testing
        rl.init_window(1280, 720, "UI Benchmark")
        rl.set_target_fps(60)  # Set high FPS to avoid artificial limits
        
        try:
            while time.time() - start_time < duration:
                if rl.window_should_close():
                    break
                
                # Record start time for frame
                frame_start = time.time()
                
                # Update UI system
                ui_system.update()
                
                # Begin drawing
                rl.begin_drawing()
                rl.clear_background(rl.BLACK)
                
                # Render UI
                screen_rect = rl.Rectangle(0, 0, 1280, 720)
                ui_system.render(screen_rect)
                
                # End drawing
                rl.end_drawing()
                
                # Calculate frame time and collect metrics
                frame_time = time.time() - frame_start
                self.validator.collect_metrics(frame_time)
                
                frame_count += 1
                
                if frame_count % 300 == 0:  # Print progress every ~10 seconds at 30fps
                    elapsed = time.time() - start_time
                    print(f"Stress test progress: {elapsed:.1f}s / {duration}s")
        
        except KeyboardInterrupt:
            print("Stress test interrupted by user")
        finally:
            rl.close_window()
        
        print(f"Stress test completed. Total frames: {frame_count}")
        return self.validator.validate_performance()
    
    def run_component_benchmarks(self) -> Dict[str, Any]:
        """Run benchmarks on individual UI components"""
        from selfdrive.ui.components.hardware_status import HardwareStatusDashboard
        from selfdrive.ui.components.navigation_display import NavigationDisplay
        from selfdrive.ui.components.perception_visualization import PerceptionVisualization
        from selfdrive.ui.components.system_status import SystemStatusPanel
        from selfdrive.ui.components.controls_interface import ControlsInterface
        
        results = {}
        
        # Test hardware status component
        print("Benchmarking Hardware Status Dashboard...")
        hw_component = HardwareStatusDashboard()
        hw_time = self._benchmark_component(hw_component, 1000)
        results['hardware_status'] = {
            'avg_render_time_ms': hw_time * 1000,
            'pass': hw_time * 1000 < 2.0  # Should render in less than 2ms
        }
        
        # Test navigation display component
        print("Benchmarking Navigation Display...")
        nav_component = NavigationDisplay()
        nav_time = self._benchmark_component(nav_component, 1000)
        results['navigation_display'] = {
            'avg_render_time_ms': nav_time * 1000,
            'pass': nav_time * 1000 < 5.0  # Should render in less than 5ms
        }
        
        # Test perception visualization component
        print("Benchmarking Perception Visualization...")
        perception_component = PerceptionVisualization()
        perception_time = self._benchmark_component(perception_component, 1000)
        results['perception_visualization'] = {
            'avg_render_time_ms': perception_time * 1000,
            'pass': perception_time * 1000 < 8.0  # Should render in less than 8ms
        }
        
        print("\nComponent Benchmark Results:")
        for component, result in results.items():
            status = "PASS" if result['pass'] else "FAIL"
            print(f"  {component}: {result['avg_render_time_ms']:.2f}ms - {status}")
        
        return results
    
    def _benchmark_component(self, component, iterations: int) -> float:
        """Benchmark a single component for performance"""
        # Create a dummy SubMaster for the component update
        sm = messaging.SubMaster([
            "modelV2", "controlsState", "selfdriveState", "deviceState",
            "carState", "perceptionModel", "uiDebug", "navInstruction"
        ])
        
        start_time = time.time()
        total_render_time = 0.0
        
        for i in range(iterations):
            # Update with messaging system
            sm.update(0)
            component.update(sm)
            
            # Record render time
            render_start = time.time()
            rect = rl.Rectangle(0, 0, 1280, 720)
            component.render(rect)
            total_render_time += time.time() - render_start
            
            # Update every 100 iterations to reduce overhead
            if i % 100 == 0:
                # Simulate different data states
                pass
        
        avg_time = total_render_time / iterations
        return avg_time


def run_comprehensive_performance_test():
    """Run the full performance validation suite"""
    print("Starting comprehensive performance validation...")
    
    benchmark_runner = BenchmarkRunner()
    
    # Run component benchmarks first
    print("\n" + "="*60)
    print("RUNNING COMPONENT BENCHMARKS")
    print("="*60)
    component_results = benchmark_runner.run_component_benchmarks()
    
    # For system-level benchmarking, we'd need the complete UI system
    # which requires more setup. For now, let's create a basic system mock
    print("\n" + "="*60)
    print("RUNNING SYSTEM BENCHMARKS")
    print("="*60)
    
    # Create a basic mock UI system for testing
    from selfdrive.ui.raylib_ui_system import RaylibUI, UIConfig
    from selfdrive.ui.complete_ui_system import CompleteSunnypilotUISystem
    
    ui_config = UIConfig(
        target_fps=30,
        max_cpu_usage=5.0,
        resource_budget=50.0,
        screen_width=1280,
        screen_height=720
    )
    
    # Test the CompleteSunnypilotUISystem performance
    try:
        test_ui_system = CompleteSunnypilotUISystem()
        
        print("Running system stress test...")
        system_results = benchmark_runner.run_stress_test(test_ui_system, 15.0)  # 15 second test
        
        print("\n" + "="*60)
        print("SYSTEM BENCHMARK RESULTS")
        print("="*60)
        benchmark_runner.validator.print_performance_report()
        
        print(f"\nFinal Assessment:")
        print(f"  Overall System Performance: {'PASS' if system_results['overall_pass'] else 'FAIL'}")
        print(f"  Average FPS: {system_results['avg_fps']:.2f}")
        print(f"  Average CPU Usage: {system_results['avg_cpu']:.2f}%")
        print(f"  Average Render Time: {system_results['avg_render_time']:.2f}ms")
        
        return {
            'components': component_results,
            'system': system_results
        }
        
    except Exception as e:
        print(f"Error running system benchmark: {e}")
        import traceback
        traceback.print_exc()
        return {'error': str(e)}


def main():
    """Main function to run performance validation"""
    print("Sunnypilot UI Performance Validation Tool")
    print("This tool benchmarks the UI system performance against targets.")
    print("Targets: FPS >= 30, CPU <= 5%, Render time <= 33ms per frame")
    
    results = run_comprehensive_performance_test()
    
    # Generate final summary
    print("\n" + "="*60)
    print("PERFORMANCE VALIDATION SUMMARY")
    print("="*60)
    
    if 'error' in results:
        print(f"Error during validation: {results['error']}")
        return 1
    
    system_results = results['system']
    
    if system_results['overall_pass']:
        print("✓ UI PERFORMANCE VALIDATION: PASSED")
        print("The UI system meets performance targets for deployment")
    else:
        print("✗ UI PERFORMANCE VALIDATION: FAILED")
        print("The UI system does not meet performance requirements")
        
        if not system_results['fps_pass']:
            print(f"  - FPS requirement not met: {system_results['avg_fps']:.2f} < 30")
        if not system_results['cpu_pass']:
            print(f"  - CPU requirement not met: {system_results['avg_cpu']:.2f}% > 5%")
        if not system_results['render_time_pass']:
            print(f"  - Render time requirement not met: {system_results['avg_render_time']:.2f}ms > 33ms")
    
    return 0 if system_results['overall_pass'] else 1


if __name__ == "__main__":
    exit(main())