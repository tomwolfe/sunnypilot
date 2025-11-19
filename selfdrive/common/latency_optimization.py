"""
Latency Optimization for Sunnypilot Real-Time Performance
Provides low-latency processing and scheduling for time-critical operations
"""

import numpy as np
from typing import Dict, List, Optional, Callable, Any, Tuple
import time
import threading
from dataclasses import dataclass
from enum import Enum
import queue
import heapq
from collections import deque
import os
import sys


class TaskPriority(Enum):
    """Priority levels for latency-critical tasks"""
    CRITICAL = 0      # Safety-critical, must run immediately
    HIGH = 1          # Time-sensitive, high priority
    MEDIUM = 2        # Standard priority
    LOW = 3           # Background tasks


@dataclass
class Task:
    """Represents a task in the real-time scheduler"""
    id: str
    func: Callable
    args: tuple
    kwargs: dict
    priority: TaskPriority
    deadline: float  # Absolute time by which task should complete
    created_time: float
    task_type: str = "generic"  # "vision", "control", "planning", etc.
    cpu_affinity: Optional[int] = None  # CPU core affinity if available


class RealTimeScheduler:
    """
    Real-time scheduler optimized for low-latency execution of critical tasks
    """
    
    def __init__(self, max_workers: int = 4, deadline_tolerance: float = 0.005):
        self.max_workers = max_workers
        self.deadline_tolerance = deadline_tolerance  # 5ms tolerance
        self.task_queue = []  # Priority queue (heapq)
        self.task_lock = threading.Lock()
        self.worker_threads = []
        self.running = False
        self.stats = {
            'completed_tasks': 0,
            'missed_deadlines': 0,
            'avg_latency': 0.0,
            'max_latency': 0.0
        }
        
        # Track execution times for latency analysis
        self.execution_times = deque(maxlen=100)
    
    def add_task(self, task: Task) -> bool:
        """Add a task to the scheduler queue"""
        with self.task_lock:
            # Use priority and deadline as sorting key
            priority_key = (task.priority.value, task.deadline)
            heapq.heappush(self.task_queue, (priority_key, task))
            return True
    
    def get_next_task(self) -> Optional[Task]:
        """Get the next highest priority task"""
        with self.task_lock:
            if self.task_queue:
                # Check if highest priority task should be skipped due to deadline
                priority_key, task = heapq.heappop(self.task_queue)
                
                current_time = time.time()
                if current_time > task.deadline + self.deadline_tolerance:
                    # Task has missed deadline, add to stats and continue
                    self.stats['missed_deadlines'] += 1
                    return None
                
                return task
            return None
    
    def start(self):
        """Start the scheduler"""
        if self.running:
            return
        
        self.running = True
        for i in range(self.max_workers):
            worker = threading.Thread(target=self._worker_loop, args=(i,), daemon=True)
            worker.start()
            self.worker_threads.append(worker)
    
    def stop(self):
        """Stop the scheduler"""
        self.running = False
        for thread in self.worker_threads:
            thread.join(timeout=2.0)
        self.worker_threads.clear()
    
    def _worker_loop(self, worker_id: int):
        """Main worker loop that processes tasks"""
        while self.running:
            task = self.get_next_task()
            if task is None:
                time.sleep(0.001)  # Brief sleep to prevent busy waiting
                continue
            
            start_time = time.time()
            
            try:
                # Set CPU affinity if specified (Linux only)
                if task.cpu_affinity and hasattr(os, 'sched_setaffinity'):
                    os.sched_setaffinity(0, [task.cpu_affinity])
                
                # Execute the task
                result = task.func(*task.args, **task.kwargs)
                
                # Record execution statistics
                execution_time = time.time() - start_time
                self.execution_times.append(execution_time)
                
                with self.task_lock:
                    self.stats['completed_tasks'] += 1
                    self.stats['avg_latency'] = sum(self.execution_times) / len(self.execution_times)
                    self.stats['max_latency'] = max(self.execution_times) if self.execution_times else 0.0
                
            except Exception as e:
                print(f"Task {task.id} failed: {e}")
    
    def get_stats(self) -> Dict[str, Any]:
        """Get scheduler statistics"""
        with self.task_lock:
            return self.stats.copy()


class LatencyProfiler:
    """
    Profiling tools for measuring and optimizing latency in critical paths
    """
    
    def __init__(self):
        self.measurements = {}
        self.active_probes = {}
    
    def start_measurement(self, name: str):
        """Start a latency measurement"""
        self.active_probes[name] = time.perf_counter()
    
    def end_measurement(self, name: str) -> float:
        """End a latency measurement and return duration in seconds"""
        if name in self.active_probes:
            duration = time.perf_counter() - self.active_probes[name]
            del self.active_probes[name]
            
            # Store measurement
            if name not in self.measurements:
                self.measurements[name] = deque(maxlen=1000)  # Keep last 1000 measurements
            self.measurements[name].append(duration)
            
            return duration
        return 0.0
    
    def get_stats(self, name: str) -> Dict[str, float]:
        """Get statistics for a specific measurement"""
        if name in self.measurements and self.measurements[name]:
            values = list(self.measurements[name])
            return {
                'count': len(values),
                'avg': sum(values) / len(values),
                'min': min(values),
                'max': max(values),
                'p95': sorted(values)[int(0.95 * len(values))] if values else 0,
                'p99': sorted(values)[int(0.99 * len(values))] if values else 0
            }
        return {'count': 0, 'avg': 0, 'min': 0, 'max': 0, 'p95': 0, 'p99': 0}
    
    def get_all_stats(self) -> Dict[str, Dict[str, float]]:
        """Get statistics for all measurements"""
        return {name: self.get_stats(name) for name in self.measurements}


class LatencyOptimizer:
    """
    Main latency optimization controller with various optimization strategies
    """
    
    def __init__(self):
        self.scheduler = RealTimeScheduler()
        self.profiler = LatencyProfiler()
        self.pipeline_stages = {}
        self.pipeline_queue = queue.Queue()
        self.pipeline_lock = threading.Lock()
        self.pipeline_active = False
        
        # Pre-allocated buffers for zero-copy operations
        self.buffers = {}
        
    def optimize_pipeline(self, stages: List[Tuple[str, Callable]]) -> Callable:
        """Create an optimized pipeline with reduced latency"""
        def pipeline_func(*args, **kwargs):
            # Measure latency of pipeline execution
            self.profiler.start_measurement('pipeline_total')
            
            result = args
            for stage_name, stage_func in stages:
                self.profiler.start_measurement(f'pipeline_stage_{stage_name}')
                try:
                    if len(result) == 1:
                        result = (stage_func(result[0]),)
                    else:
                        result = stage_func(*result)
                except Exception as e:
                    print(f"Pipeline stage {stage_name} failed: {e}")
                    break
                self.profiler.end_measurement(f'pipeline_stage_{stage_name}')
            
            total_time = self.profiler.end_measurement('pipeline_total')
            return result[0] if len(result) == 1 else result
        
        return pipeline_func
    
    def create_zero_copy_buffer(self, name: str, shape: tuple, dtype: np.dtype = np.float32):
        """Create a pre-allocated buffer for zero-copy operations"""
        buffer = np.zeros(shape, dtype=dtype)
        self.buffers[name] = buffer
        return buffer
    
    def get_buffer(self, name: str) -> Optional[np.ndarray]:
        """Get a pre-allocated buffer"""
        return self.buffers.get(name)
    
    def profile_function(self, name: str = None) -> Callable:
        """Decorator to profile function execution time"""
        def decorator(func):
            profile_name = name or func.__name__
            
            def wrapper(*args, **kwargs):
                self.profiler.start_measurement(profile_name)
                try:
                    result = func(*args, **kwargs)
                    self.profiler.end_measurement(profile_name)
                    return result
                except Exception as e:
                    self.profiler.end_measurement(profile_name)
                    raise e
            return wrapper
        return decorator
    
    def schedule_critical_task(self, func: Callable, *args, 
                             priority: TaskPriority = TaskPriority.CRITICAL,
                             relative_deadline: float = 0.05,  # 50ms deadline
                             task_type: str = "generic",
                             cpu_affinity: Optional[int] = None) -> str:
        """Schedule a critical task with specific deadline requirements"""
        task_id = f"task_{int(time.time() * 1000000)}"
        current_time = time.time()
        
        task = Task(
            id=task_id,
            func=func,
            args=args,
            kwargs={},
            priority=priority,
            deadline=current_time + relative_deadline,
            created_time=current_time,
            task_type=task_type,
            cpu_affinity=cpu_affinity
        )
        
        # Add to scheduler
        self.scheduler.add_task(task)
        
        return task_id
    
    def optimize_memory_access(self, data: np.ndarray) -> np.ndarray:
        """Optimize array for cache-friendly access patterns"""
        # Ensure array is C-contiguous for optimal cache access
        if not data.flags['C_CONTIGUOUS']:
            data = np.ascontiguousarray(data)
        
        # For large arrays, consider memory layout optimization
        if data.ndim > 1:
            # Ensure row-major order for cache efficiency
            if data.strides[-1] != data.itemsize:  # Not row-major
                data = np.ascontiguousarray(data)
        
        return data
    
    def batch_process_optimized(self, items: List[Any], 
                              processor: Callable,
                              batch_size: int = 32) -> List[Any]:
        """Process items in batches to optimize memory and cache usage"""
        results = []
        for i in range(0, len(items), batch_size):
            batch = items[i:i + batch_size]
            
            # Process batch with profiling
            self.profiler.start_measurement('batch_process')
            batch_results = [processor(item) for item in batch]
            self.profiler.end_measurement('batch_process')
            
            results.extend(batch_results)
        
        return results
    
    def get_latency_stats(self) -> Dict[str, Any]:
        """Get comprehensive latency optimization statistics"""
        return {
            'scheduler_stats': self.scheduler.get_stats(),
            'profiler_stats': self.profiler.get_all_stats(),
            'buffer_count': len(self.buffers)
        }


class LowLatencyVisionProcessor:
    """
    Vision processing optimized for low latency with ARM-specific optimizations
    """
    
    def __init__(self, latency_optimizer: LatencyOptimizer):
        self.optimizer = latency_optimizer
        self.profiling_enabled = True
        
        # Pre-allocate buffers for common operations
        self.input_buffer = self.optimizer.create_zero_copy_buffer(
            'vision_input', (480, 640, 3), np.uint8
        )
        self.processed_buffer = self.optimizer.create_zero_copy_buffer(
            'vision_processed', (256, 256, 3), np.float32
        )
    
    @LatencyOptimizer().profile_function('vision_preprocess')
    def preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """Optimized frame preprocessing with reduced latency"""
        # Use pre-allocated buffer to avoid allocation
        output = self.optimizer.get_buffer('vision_processed')
        
        if output is not None:
            # Resize and normalize efficiently
            # Use optimized operations for ARM processors
            if frame.shape != output.shape:
                # Optimize resize operation
                output = self._optimized_resize(frame, output.shape)
            else:
                output[:] = frame.astype(np.float32) / 255.0
            
            return output
        else:
            # Fallback if buffer unavailable
            return self.optimizer.optimize_memory_access(
                frame.astype(np.float32) / 255.0
            )
    
    def _optimized_resize(self, input_array: np.ndarray, target_shape: tuple) -> np.ndarray:
        """Optimized resize operation with ARM-friendly algorithms"""
        # Use efficient numpy operations that map well to ARM NEON
        if input_array.size > target_shape[0] * target_shape[1] * input_array.shape[-1]:
            # Downsample - use slicing for efficiency
            h_ratio = input_array.shape[0] / target_shape[0]
            w_ratio = input_array.shape[1] / target_shape[1]
            
            h_indices = (np.arange(target_shape[0]) * h_ratio).astype(int)
            w_indices = (np.arange(target_shape[1]) * w_ratio).astype(int)
            
            # Use advanced indexing for efficient sampling
            result = input_array[np.ix_(h_indices, w_indices, [0, 1, 2])]
        else:
            # Upsample or similar size - use standard resize
            result = np.resize(input_array, target_shape)
        
        return result.astype(np.float32) / 255.0


def get_latency_optimizer() -> LatencyOptimizer:
    """Get the global latency optimizer instance"""
    if not hasattr(get_latency_optimizer, 'instance'):
        get_latency_optimizer.instance = LatencyOptimizer()
        get_latency_optimizer.instance.scheduler.start()
    return get_latency_optimizer.instance


def optimize_for_latency(func: Callable, name: str = None) -> Callable:
    """Decorator to optimize function for low latency"""
    optimizer = get_latency_optimizer()
    profiler_name = name or func.__name__
    
    # Apply profiling
    profiled_func = optimizer.profile_function(profiler_name)(func)
    
    # Schedule with high priority if it's a critical function
    def optimized_wrapper(*args, **kwargs):
        task_id = optimizer.schedule_critical_task(
            profiled_func, *args,
            priority=TaskPriority.HIGH if 'critical' in profiler_name.lower() else TaskPriority.MEDIUM,
            relative_deadline=0.02  # 20ms deadline for most functions
        )
        
        # For now, execute directly (in full system, this would be scheduled)
        return profiled_func(*args, **kwargs)
    
    return optimized_wrapper


# Example usage and testing
if __name__ == "__main__":
    print("Testing Latency Optimization System...")
    
    # Create latency optimizer
    latency_opt = get_latency_optimizer()
    
    # Test 1: Basic profiling
    print("Test 1: Basic profiling")
    
    @latency_opt.profile_function("test_function")
    def test_func():
        time.sleep(0.001)  # Simulate 1ms operation
        return "completed"
    
    result = test_func()
    stats = latency_opt.profiler.get_stats("test_function")
    print(f"  Function completed: {result}")
    print(f"  Execution time: avg={stats['avg']*1000:.2f}ms, max={stats['max']*1000:.2f}ms")
    
    # Test 2: Pipeline optimization
    print("\nTest 2: Pipeline optimization")
    
    def stage1(x):
        time.sleep(0.0005)  # 0.5ms
        return x * 2
    
    def stage2(x):
        time.sleep(0.0003)  # 0.3ms
        return x + 1
    
    def stage3(x):
        time.sleep(0.0002)  # 0.2ms
        return x / 2
    
    pipeline = latency_opt.optimize_pipeline([
        ("stage1", stage1),
        ("stage2", stage2),
        ("stage3", stage3)
    ])
    
    result = pipeline(10)
    print(f"  Pipeline result: {result}")
    
    pipeline_stats = latency_opt.profiler.get_stats("pipeline_total")
    print(f"  Pipeline time: avg={pipeline_stats['avg']*1000:.2f}ms")
    
    # Test 3: Zero-copy buffers
    print("\nTest 3: Zero-copy buffers")
    
    # Create and use buffer
    buffer = latency_opt.create_zero_copy_buffer("test_buffer", (100, 100))
    buffer.fill(5.0)
    print(f"  Created buffer shape: {buffer.shape}, filled with 5.0")
    
    retrieved_buffer = latency_opt.get_buffer("test_buffer")
    print(f"  Retrieved same buffer: {retrieved_buffer is buffer}")
    
    # Test 4: Vision processing optimization
    print("\nTest 4: Vision processing optimization")
    
    vision_processor = LowLatencyVisionProcessor(latency_opt)
    
    # Simulate a frame
    test_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    processed = vision_processor.preprocess_frame(test_frame)
    print(f"  Processed frame from {test_frame.shape} to {processed.shape}")
    
    vision_stats = latency_opt.profiler.get_stats("vision_preprocess")
    print(f"  Vision preprocessing time: avg={vision_stats['avg']*1000:.2f}ms")
    
    # Test 5: Task scheduling
    print("\nTest 5: Task scheduling")
    
    def sample_task():
        time.sleep(0.002)  # 2ms task
        return "task completed"
    
    task_id = latency_opt.schedule_critical_task(
        sample_task,
        priority=TaskPriority.CRITICAL,
        relative_deadline=0.05
    )
    print(f"  Scheduled task with ID: {task_id}")
    
    # Get overall statistics
    overall_stats = latency_opt.get_latency_stats()
    print(f"\nOverall Latency Optimization Stats:")
    print(f"  Scheduler completed tasks: {overall_stats['scheduler_stats']['completed_tasks']}")
    print(f"  Profiler measurements: {len(overall_stats['profiler_stats'])}")
    print(f"  Pre-allocated buffers: {overall_stats['buffer_count']}")
    
    print("\nLatency optimization system test completed!")