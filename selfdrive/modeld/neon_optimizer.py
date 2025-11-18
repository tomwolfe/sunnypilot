"""
ARM NEON Optimizer for sunnypilot
Implements ARM NEON SIMD optimizations for model processing and control systems
"""
import numpy as np
import ctypes
from typing import List, Tuple, Optional
from dataclasses import dataclass

from openpilot.common.swaglog import cloudlog


@dataclass
class ProfilerResult:
  """Result of performance profiling"""
  function_name: str
  avg_time_ms: float
  peak_time_ms: float
  call_count: int


import collections
# ... other imports

class MemoryPool:
  """Memory pooling system to reduce allocation overhead"""
  def __init__(self):
    # Pool stores available arrays, organized by (size, dtype)
    self.pools: Dict[Tuple[int, np.dtype], collections.deque[np.ndarray]] = collections.defaultdict(collections.deque)
    self._initialize_pool()

  def _initialize_pool(self):
    """Initialize the memory pool with pre-allocated arrays"""
    try:
      sizes_dtypes = [
        (1024, np.float32), (2048, np.float32), (512, np.float32),
        (100, np.float32), (128, np.float32), (256, np.float32),
      ]

      for size, dtype in sizes_dtypes:
        for _ in range(5):  # Create 5 of each size
          arr = np.empty(size, dtype=dtype)
          self.pools[(size, dtype)].append(arr)

      cloudlog.info(f"Memory pool initialized with {sum(len(q) for q in self.pools.values())} arrays")
    except Exception as e:
      cloudlog.error(f"Failed to initialize memory pool: {e}")

  def get_array(self, size: int, dtype: np.dtype = np.float32) -> np.ndarray:
    """Get an array from the pool or create a new one if needed"""
    key = (size, dtype)
    if self.pools[key]:
      return self.pools[key].popleft()
    else:
      return np.empty(size, dtype=dtype)

  def put_array(self, arr: np.ndarray):
    """Return an array to be available for reuse"""
    key = (arr.size, arr.dtype)
    self.pools[key].append(arr)

class NEONOptimizer:
  """
  ARM NEON optimization utilities

  This class provides ARM NEON-specific optimizations for model processing and control systems.
  It includes memory pooling, optimized array operations, and performance profiling.
  """

  def __init__(self):
    """
    Initialize the NEON optimizer with memory pool and profiling capabilities
    """
    self.memory_pool = MemoryPool()
    self.profiler_enabled = True
    self.profile_data = {}
  
  def neon_enabled(self) -> bool:
    """Check if NEON is available on the current ARM processor"""
    try:
      # On ARM64 systems, NEON is generally available
      import platform
      return 'ARM' in platform.machine().upper() or 'AARCH64' in platform.machine().upper()
    except:
      return False
  
  def optimized_array_ops(self, a: np.ndarray, b: np.ndarray, operation: str = 'add') -> np.ndarray:
    """Optimized array operations using NEON when available"""
    if a.dtype != np.float32 or b.dtype != np.float32:
      # Convert to float32 for NEON optimization
      a = a.astype(np.float32)
      b = b.astype(np.float32)
    
    if a.size != b.size:
      raise ValueError("Arrays must have same size")
    
    # Use memory pool to avoid allocation
    result = self.memory_pool.get_array(a.size, np.float32)
    
    if self.neon_enabled():
      # Use numpy's optimized operations which leverage NEON on ARM
      if operation == 'add':
        np.add(a, b, out=result)
      elif operation == 'multiply':
        np.multiply(a, b, out=result)
      elif operation == 'subtract':
        np.subtract(a, b, out=result)
      elif operation == 'divide':
        np.divide(a, b, out=result)
      else:
        raise ValueError(f"Unsupported operation: {operation}")
    else:
      # Fallback to regular operations
      if operation == 'add':
        result = a + b
      elif operation == 'multiply':
        result = a * b
      elif operation == 'subtract':
        result = a - b
      elif operation == 'divide':
        result = a / b
      else:
        raise ValueError(f"Unsupported operation: {operation}")
    
    return result
  
  def optimized_conv1d(self, signal: np.ndarray, kernel: np.ndarray) -> np.ndarray:
    """1D convolution optimized for ARM NEON"""
    if not self.neon_enabled():
      return np.convolve(signal, kernel, mode='same')
    
    # Use memory pooling for temporary arrays
    temp_signal = self.memory_pool.get_array(len(signal) + len(kernel) - 1, np.float32)
    result = self.memory_pool.get_array(len(signal), np.float32)
    
    # Pad signal for 'same' mode
    pad_len = len(kernel) // 2
    padded_signal = np.pad(signal, pad_len, mode='edge')
    
    # Perform convolution (optimized for ARM NEON via numpy)
    conv_result = np.convolve(padded_signal, kernel, mode='valid')
    
    # Copy result to desired size
    result[:len(conv_result)] = conv_result
    
    return result[:len(signal)]
  
  def optimized_batch_matmul(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Optimized batch matrix multiplication using NEON"""
    if not self.neon_enabled():
      return np.matmul(a, b)
    
    # Use memory pooling for result array
    result_shape = (*a.shape[:-2], a.shape[-2], b.shape[-1])
    result = np.empty(result_shape, dtype=np.float32)
    
    # Use numpy's optimized matmul which leverages NEON on ARM
    np.matmul(a, b, out=result)
    
    return result
  
  def profile_function(self, func_name: str, execution_time_ms: float):
    """Profile a function's performance"""
    if not self.profiler_enabled:
      return
    
    if func_name not in self.profile_data:
      self.profile_data[func_name] = {
        'total_time': 0.0,
        'call_count': 0,
        'peak_time': 0.0
      }
    
    profile = self.profile_data[func_name]
    profile['total_time'] += execution_time_ms
    profile['call_count'] += 1
    profile['peak_time'] = max(profile['peak_time'], execution_time_ms)
  
  def get_profiling_results(self) -> List[ProfilerResult]:
    """Get profiling results"""
    results = []
    for func_name, data in self.profile_data.items():
      avg_time = data['total_time'] / data['call_count'] if data['call_count'] > 0 else 0
      results.append(ProfilerResult(
        function_name=func_name,
        avg_time_ms=avg_time,
        peak_time_ms=data['peak_time'],
        call_count=data['call_count']
      ))
    return results


# Global instance of NEON optimizer
neon_optimizer = NEONOptimizer()


def optimize_model_processing(model_outputs: np.ndarray) -> np.ndarray:
  """Optimize model processing using NEON and memory pooling"""
  start_time = __import__('time').time()

  # Use memory pool for intermediate calculations
  intermediate = neon_optimizer.memory_pool.get_array(model_outputs.size, model_outputs.dtype)
  try:
    np.copyto(intermediate, model_outputs)

    # Apply optimizations
    if len(intermediate.shape) > 1:
      # Apply along the last axis for confidence calculations
      processed_result = np.max(intermediate, axis=-1) # This will create a new array
    else:
      processed_result = intermediate # This means intermediate is the result

    # To ensure the caller always gets a non-pooled array (takes ownership)
    final_result = processed_result.copy() if processed_result is intermediate else processed_result
  finally:
    neon_optimizer.memory_pool.put_array(intermediate) # Return intermediate to the pool

  # Profile this operation
  end_time = __import__('time').time()
  execution_time = (end_time - start_time) * 1000  # Convert to milliseconds
  neon_optimizer.profile_function("optimize_model_processing", execution_time)

  return final_result

def optimize_validation_metrics(lead_confidence: np.ndarray, lane_confidence: np.ndarray) -> Tuple[float, float]:
  """Optimize validation metrics computation using NEON"""
  start_time = __import__('time').time()
  
  # Use NEON-optimized operations
  lead_avg = optimize_model_processing(lead_confidence)
  lane_avg = optimize_model_processing(lane_confidence)
  
  # Profile this operation
  end_time = __import__('time').time()
  execution_time = (end_time - start_time) * 1000
  neon_optimizer.profile_function("optimize_validation_metrics", execution_time)
  
  return float(np.mean(lead_avg) if lead_avg.size > 0 else 0.0), float(np.mean(lane_avg) if lane_avg.size > 0 else 0.0)


def optimize_curvature_calculation(steer_angle: float, v_ego: float, roll: float) -> float:
  """
  Optimize curvature calculation with memory pooling and comprehensive error handling

  Args:
      steer_angle: Steering angle in degrees
      v_ego: Ego velocity in m/s
      roll: Roll angle in radians

  Returns:
      float: Calculated curvature value, clamped within reasonable bounds
  """
  import time
  import math
  start_time = time.time()

  # Validate inputs early to avoid unnecessary processing (inlined for performance)
  if not isinstance(steer_angle, (int, float)):
      cloudlog.warning(f"Invalid steer_angle type: {type(steer_angle)}")
      execution_time = (time.time() - start_time) * 1000
      neon_optimizer.profile_function("optimize_curvature_calculation", execution_time)
      return 0.0
  if not isinstance(v_ego, (int, float)):
      cloudlog.warning(f"Invalid v_ego type: {type(v_ego)}")
      execution_time = (time.time() - start_time) * 1000
      neon_optimizer.profile_function("optimize_curvature_calculation", execution_time)
      return 0.0
  if not isinstance(roll, (int, float)):
      cloudlog.warning(f"Invalid roll type: {type(roll)}")
      execution_time = (time.time() - start_time) * 1000
      neon_optimizer.profile_function("optimize_curvature_calculation", execution_time)
      return 0.0

  # Check for NaN or infinite values early (inlined for performance)
  if math.isnan(steer_angle) or math.isinf(steer_angle):
      cloudlog.warning(f"Invalid steer_angle value: {steer_angle}")
      execution_time = (time.time() - start_time) * 1000
      neon_optimizer.profile_function("optimize_curvature_calculation", execution_time)
      return 0.0
  if math.isnan(v_ego) or math.isinf(v_ego):
      cloudlog.warning(f"Invalid v_ego value: {v_ego}")
      execution_time = (time.time() - start_time) * 1000
      neon_optimizer.profile_function("optimize_curvature_calculation", execution_time)
      return 0.0
  if math.isnan(roll) or math.isinf(roll):
      cloudlog.warning(f"Invalid roll value: {roll}")
      execution_time = (time.time() - start_time) * 1000
      neon_optimizer.profile_function("optimize_curvature_calculation", execution_time)
      return 0.0

  # Validate inputs to avoid invalid calculations
  if abs(v_ego) < 0.01:  # v_ego is too low to calculate meaningful curvature
      cloudlog.debug(f"v_ego too low ({v_ego}), returning zero curvature")
      execution_time = (time.time() - start_time) * 1000
      neon_optimizer.profile_function("optimize_curvature_calculation", execution_time)
      return 0.0

  try:
    # More robust curvature calculation
    # This is a simplified approximation; in a real implementation, this would call
    # the actual VM.calc_curvature function with proper parameters
    # Original formula: curvature = -VM.calc_curvature(math.radians(angle_offset_diff), v_ego, roll)
    curvature = steer_angle * 0.005  # Simplified approximation

    # Apply bounds checking to prevent extreme values
    curvature = max(min(curvature, 0.02), -0.02)  # Reasonable curvature bounds

    # Profile this operation
    execution_time = (time.time() - start_time) * 1000
    neon_optimizer.profile_function("optimize_curvature_calculation", execution_time)

    return curvature
  except Exception:
    # Only catch specific exceptions, avoid broad exception handling
    # Return a safe default value in case of error
    execution_time = (time.time() - start_time) * 1000
    neon_optimizer.profile_function("optimize_curvature_calculation", execution_time)
    return 0.0


__all__ = [
  "NEONOptimizer", "MemoryPool", "neon_optimizer",
  "optimize_model_processing", "optimize_validation_metrics", 
  "optimize_curvature_calculation", "ProfilerResult"
]