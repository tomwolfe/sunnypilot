#!/usr/bin/env python3
"""
Performance profiling tools for sunnypilot autonomous driving system.
This implements Step 2 by identifying performance bottlenecks using profiling tools.
"""

import cProfile
import pstats
import subprocess
import sys
import os
import time
import threading
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import io

class ProfilerManager:
    """Manages different profiling tools to identify performance bottlenecks."""

    def __init__(self, target_modules: List[str] = None):
        self.target_modules = target_modules or [
            "selfdrive.controls.lib.lateral_planner",
            "selfdrive.controls.lib.longitudinal_mpc_lib",
            "selfdrive.controls.lib.alertmanager",
            "selfdrive.locationd.kalman",
            "sunnypilot.navd.navigation",
            "selfdrive.common.hardware_monitor",
            "selfdrive.common.metrics"
        ]
        self.profiles = {}

    def run_cprofile_on_module(self, module_path: str, output_file: str = None) -> str:
        """Profile a specific module using Python's cProfile."""
        print(f"Profiling module: {module_path}")

        # Create a temporary script to profile the module
        temp_script = f"""
import cProfile
import sys
sys.path.insert(0, '.')
import {module_path}

def profile_target():
    # Import the module to trigger its initialization if needed
    pass

if __name__ == "__main__":
    cProfile.run('profile_target()', '{output_file}')
"""

        temp_script_path = f"temp_profile_{module_path.replace('.', '_')}.py"
        with open(temp_script_path, 'w') as f:
            f.write(temp_script)

        try:
            # Run the profiling
            result = subprocess.run([sys.executable, temp_script_path],
                                  capture_output=True, text=True, timeout=30)
            if result.returncode != 0:
                print(f"Error profiling {module_path}: {result.stderr}")

        except subprocess.TimeoutExpired:
            print(f"Profiling timed out for {module_path}")
        finally:
            # Clean up temporary file
            if os.path.exists(temp_script_path):
                os.remove(temp_script_path)

        return output_file

    def analyze_cprofile_results(self, profile_file: str) -> List[Tuple[str, float, int]]:
        """Analyze cProfile results and return top functions by cumulative time."""
        if not os.path.exists(profile_file):
            return []

        try:
            stats = pstats.Stats(profile_file)
            # Sort by cumulative time and get top 10
            stats.sort_stats('cumulative')
            top_functions = []

            # Get the top 10 functions
            for func, (cc, nc, tt, ct, callers) in list(stats.stats.items())[:10]:
                top_functions.append((f"{func[0]}:{func[1]}({func[2]})", ct, cc))

            return top_functions
        except Exception as e:
            print(f"Error analyzing profile results: {e}")
            return []

    def run_memory_profiling(self, module_path: str) -> Dict[str, any]:
        """Analyze memory usage of a specific module."""
        try:
            # Try to import memory_profiler if available
            from memory_profiler import profile, memory_usage
            import importlib

            # Import the module to profile
            module = importlib.import_module(module_path)

            # Get memory usage
            mem_usage = memory_usage((lambda: module, (), {}), interval=0.1, timeout=5)

            return {
                "module": module_path,
                "max_memory_mb": max(mem_usage) if mem_usage else 0,
                "avg_memory_mb": sum(mem_usage)/len(mem_usage) if mem_usage else 0,
                "memory_usage_trace": mem_usage
            }
        except ImportError:
            print("memory_profiler not available. Install with: pip install memory-profiler")
            return {"error": "memory_profiler not available"}
        except Exception as e:
            print(f"Error profiling memory for {module_path}: {e}")
            return {"error": str(e)}

    def run_system_profiling(self) -> Dict[str, any]:
        """Run system-level profiling to identify bottlenecks."""
        print("Running system-level profiling...")

        # Get current system status
        import psutil
        import platform

        system_info = {
            "platform": platform.platform(),
            "cpu_count": psutil.cpu_count(),
            "cpu_percent": psutil.cpu_percent(interval=1),
            "memory_gb": psutil.virtual_memory().total / (1024**3),
            "memory_percent": psutil.virtual_memory().percent,
            "disk_usage_percent": psutil.disk_usage('/').percent,
            "load_average": os.getloadavg() if os.name != 'nt' else "N/A"
        }

        # Get process-specific information
        process = psutil.Process()
        process_info = {
            "pid": process.pid,
            "process_cpu_percent": process.cpu_percent(),
            "process_memory_mb": process.memory_info().rss / (1024 * 1024),
            "num_threads": process.num_threads(),
            "open_files": len(process.open_files()),
            "connections": len(process.connections())
        }

        # Get top CPU-consuming processes
        top_processes = []
        for proc in sorted(psutil.process_iter(['pid', 'name', 'cpu_percent']),
                          key=lambda p: p.info['cpu_percent'], reverse=True)[:5]:
            top_processes.append(proc.info)

        return {
            "system_info": system_info,
            "process_info": process_info,
            "top_processes": top_processes
        }

    def identify_performance_bottlenecks(self) -> Dict[str, any]:
        """Identify performance bottlenecks across the system."""
        print("Starting performance analysis...")

        analysis_results = {
            "system_profiling": self.run_system_profiling(),
            "module_profiling": {},
            "memory_analysis": {},
            "recommendations": []
        }

        for module in self.target_modules[:3]:  # Limit to first 3 to avoid long execution
            print(f"Analyzing module: {module}")

            # Create profile file path
            profile_file = f"profile_{module.replace('.', '_')}.prof"

            # Run cProfile on the module
            profile_file = self.run_cprofile_on_module(module, profile_file)

            # Analyze results
            top_functions = self.analyze_cprofile_results(profile_file)
            analysis_results["module_profiling"][module] = {
                "top_functions": top_functions,
                "profile_file": profile_file
            }

            # Run memory profiling
            mem_analysis = self.run_memory_profiling(module)
            analysis_results["memory_analysis"][module] = mem_analysis

            # Clean up profile file
            if os.path.exists(profile_file):
                os.remove(profile_file)

        # Generate recommendations based on findings
        analysis_results["recommendations"] = self._generate_recommendations(analysis_results)

        return analysis_results

    def _generate_recommendations(self, analysis_results: Dict) -> List[str]:
        """Generate optimization recommendations based on profiling results."""
        recommendations = []

        # Check system-level metrics
        sys_info = analysis_results["system_profiling"]["system_info"]
        if sys_info["cpu_percent"] > 50:
            recommendations.append(f"High CPU usage detected: {sys_info['cpu_percent']:.1f}% - investigate CPU-intensive operations")

        if sys_info["memory_percent"] > 70:
            recommendations.append(f"High memory usage detected: {sys_info['memory_percent']:.1f}% - consider memory optimization")

        # Check module-specific profiles
        for module, profiling_data in analysis_results["module_profiling"].items():
            top_functions = profiling_data.get("top_functions", [])
            if top_functions:
                # Look for functions taking significant time
                for func_name, cum_time, call_count in top_functions[:3]:
                    if cum_time > 0.1:  # More than 100ms in cumulative time
                        recommendations.append(f"In {module}: {func_name} takes {cum_time:.3f}s cumulative time - potential optimization target")

        # Check memory usage
        for module, mem_data in analysis_results["memory_analysis"].items():
            if "max_memory_mb" in mem_data:
                if mem_data["max_memory_mb"] > 100:  # More than 100MB
                    recommendations.append(f"{module} uses {mem_data['max_memory_mb']:.1f}MB - consider memory optimization")

        return recommendations

def run_performance_profiling():
    """Run comprehensive performance profiling of sunnypilot system."""
    print("Sunnypilot Performance Profiling Tool")
    print("=====================================")

    profiler = ProfilerManager()

    # Run comprehensive analysis
    results = profiler.identify_performance_bottlenecks()

    # Print results
    print("\n" + "="*80)
    print("PERFORMANCE PROFILING RESULTS")
    print("="*80)

    print("\nSystem Profiling:")
    print("-" * 40)
    sys_info = results["system_profiling"]["system_info"]
    print(f"  CPU Usage: {sys_info['cpu_percent']:.1f}%")
    print(f"  Memory Usage: {sys_info['memory_percent']:.1f}% ({sys_info['memory_gb']:.1f}GB total)")
    print(f"  Disk Usage: {sys_info['disk_usage_percent']:.1f}%")

    print("\nProcess Information:")
    print("-" * 40)
    proc_info = results["system_profiling"]["process_info"]
    print(f"  Process Memory: {proc_info['process_memory_mb']:.1f}MB")
    print(f"  Process CPU: {proc_info['process_cpu_percent']:.1f}%")
    print(f"  Threads: {proc_info['num_threads']}")

    print("\nModule Profiling Results:")
    print("-" * 40)
    for module, data in results["module_profiling"].items():
        print(f"\nModule: {module}")
        top_funcs = data.get("top_functions", [])
        if top_funcs:
            for i, (func, time_taken, calls) in enumerate(top_funcs[:3]):
                print(f"  {i+1}. {func}: {time_taken:.3f}s ({calls} calls)")
        else:
            print("  No profiling data available")

    print("\nMemory Analysis:")
    print("-" * 40)
    for module, mem_data in results["memory_analysis"].items():
        if "max_memory_mb" in mem_data:
            print(f"  {module}: Max {mem_data['max_memory_mb']:.1f}MB, Avg {mem_data['avg_memory_mb']:.1f}MB")
        elif "error" in mem_data:
            print(f"  {module}: {mem_data['error']}")

    print("\nOptimization Recommendations:")
    print("-" * 40)
    for i, rec in enumerate(results["recommendations"], 1):
        print(f"  {i}. {rec}")

    # Save results to file
    import json
    timestamp = int(time.time())
    filename = f"performance_analysis_{timestamp}.json"

    # Convert any non-serializable objects for JSON
    serializable_results = {}
    for key, value in results.items():
        if key == "memory_analysis":
            # Process memory analysis to make it serializable
            serializable_results[key] = {}
            for mod, data in value.items():
                clean_data = {}
                for k, v in data.items():
                    if isinstance(v, list) and len(v) > 10:  # Truncate large lists
                        clean_data[k] = v[:10] + [f"... and {len(v)-10} more"]
                    else:
                        clean_data[k] = v
                serializable_results[key][mod] = clean_data
        else:
            serializable_results[key] = value

    with open(filename, 'w') as f:
        json.dump(serializable_results, f, indent=2, default=str)

    print(f"\nDetailed results saved to: {filename}")

    return results

if __name__ == "__main__":
    run_performance_profiling()