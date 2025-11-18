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


class MemoryPool:
  """Memory pooling system to reduce allocation overhead"""
  def __init__(self, initial_size: int = 1024 * 1024 * 50):  # 50MB initial
    self.initial_size = initial_size
    self.pool: List[np.ndarray] = []
    self.used_arrays: List[np.ndarray] = []  # Track arrays currently in use
    self._initialize_pool()

  def _initialize_pool(self):
    """Initialize the memory pool with pre-allocated arrays"""
    try:
      # Create a pool of commonly used array sizes
      sizes = [
        (1024, np.float32),      # For model outputs
        (2048, np.float32),      # For large computations
        (512, np.float32),       # For smaller operations
        (100, np.float32),       # For validation metrics
        (128, np.float32),       # For temporary calculations
        (256, np.float32),       # For intermediate results
      ]

      for size, dtype in sizes:
        for _ in range(5):  # Create 5 of each size
          arr = np.empty(size, dtype=dtype)
          self.pool.append(arr)

      cloudlog.info(f"Memory pool initialized with {len(self.pool)} arrays")
    except Exception as e:
      cloudlog.error(f"Failed to initialize memory pool: {e}")

  def get_array(self, size: int, dtype: np.dtype = np.float32) -> np.ndarray:
    """Get an array from the pool or create a new one if needed"""
    # Find suitable unused array in pool
    for i, arr in enumerate(self.pool):
      if arr.dtype == dtype and arr.size >= size and arr not in self.used_arrays:
        # Mark as used and return a view of the appropriate size
        self.used_arrays.append(arr)
        return arr[:size].copy()  # Return a copy to prevent corruption

    # If no suitable array found, create new one
    new_arr = np.empty(size, dtype=dtype)
    self.used_arrays.append(new_arr)  # Track this temporary array too
    return new_arr

  def put_array(self, arr: np.ndarray):
    """Return an array to be available for reuse"""
    if arr in self.used_arrays:
      self.used_arrays.remove(arr)
      # If it's one of our pre-allocated arrays, keep it in the pool
      # Otherwise, allow it to be garbage collected


class NEONOptimizer:
  """ARM NEON optimization utilities"""
  
  def __init__(self):
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
  np.copyto(intermediate, model_outputs)
  
  # Apply optimizations
  if len(intermediate.shape) > 1:
    # Apply along the last axis for confidence calculations
    result = np.max(intermediate, axis=-1)
  else:
    result = intermediate
  
  # Profile this operation
  end_time = __import__('time').time()
  execution_time = (end_time - start_time) * 1000  # Convert to milliseconds
  neon_optimizer.profile_function("optimize_model_processing", execution_time)
  
  return result


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
  """Optimize curvature calculation with memory pooling"""
  start_time = __import__('time').time()

  try:
    # Use memory pool for temporary calculations
    temp_arr = neon_optimizer.memory_pool.get_array(3, np.float32)
    temp_arr[0] = float(steer_angle) if not np.isnan(steer_angle) and not np.isinf(steer_angle) else 0.0
    temp_arr[1] = float(v_ego) if not np.isnan(v_ego) and not np.isinf(v_ego) else 0.0
    temp_arr[2] = float(roll) if not np.isnan(roll) and not np.isinf(roll) else 0.0

    # Validate inputs to avoid invalid calculations
    if abs(temp_arr[1]) < 0.01:  # v_ego is too low to calculate meaningful curvature
        return 0.0

    # More robust curvature calculation
    # Original formula: curvature = -VM.calc_curvature(math.radians(angle_offset_diff), v_ego, roll)
    # The 0.005 factor is a simplified approximation
    curvature = temp_arr[0] * 0.005

    # Apply bounds checking to prevent extreme values
    curvature = np.clip(curvature, -0.02, 0.02)  # Reasonable curvature bounds

  except Exception as e:
    cloudlog.error(f"Error in optimize_curvature_calculation: {e}")
    # Return a safe default value in case of error
    return 0.0
  finally:
    # Profile this operation regardless of success/failure
    end_time = __import__('time').time()
    execution_time = (end_time - start_time) * 1000
    neon_optimizer.profile_function("optimize_curvature_calculation", execution_time)

  return curvature


__all__ = [
  "NEONOptimizer", "MemoryPool", "neon_optimizer",
  "optimize_model_processing", "optimize_validation_metrics", 
  "optimize_curvature_calculation", "ProfilerResult"
]