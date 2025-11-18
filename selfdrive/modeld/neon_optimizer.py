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
  def __init__(self, max_pool_size: int = 10):
    """
    Initialize the memory pool

    Args:
        max_pool_size: Maximum number of arrays to keep in each size/type pool
    """
    # Pool stores available arrays, organized by (size, dtype)
    self.pools: Dict[Tuple[int, np.dtype], collections.deque[np.ndarray]] = collections.defaultdict(collections.deque)
    self.max_pool_size = max_pool_size
    self.pool_stats = {'get_count': 0, 'new_allocations': 0, 'put_count': 0}  # Track allocation statistics

  def get_array(self, size: int, dtype: np.dtype = np.float32) -> np.ndarray:
    """
    Get an array from the pool or create a new one if needed

    Args:
        size: Size of the array to get
        dtype: Data type of the array to get

    Returns:
        np.ndarray: Array of requested size and type
    """
    if size <= 0:
        raise ValueError(f"Size must be positive, got: {size}")

    key = (size, dtype)
    if self.pools[key]:
      self.pool_stats['get_count'] += 1
      return self.pools[key].popleft()
    else:
      self.pool_stats['new_allocations'] += 1
      return np.empty(size, dtype=dtype)

  def put_array(self, arr: np.ndarray):
    """
    Return an array to be available for reuse

    Args:
        arr: The array to return to the pool
    """
    if not isinstance(arr, np.ndarray):
        cloudlog.error(f"Invalid array type to return to pool: {type(arr)}")
        return

    # Limit pool size to prevent excessive memory usage
    key = (arr.size, arr.dtype)
    if len(self.pools[key]) < self.max_pool_size:  # Limit to max_pool_size arrays per size/type to prevent memory bloat
      self.pools[key].append(arr)
      self.pool_stats['put_count'] += 1
    else:
      # Pool is full, array will be garbage collected
      cloudlog.debug(f"Pool for size {key} full, array will be garbage collected")

  def get_stats(self) -> Dict[str, int]:
    """
    Get statistics about pool usage

    Returns:
        Dict containing pool statistics
    """
    return self.pool_stats.copy()

  def clear_pool(self):
    """Clear all arrays from the pool (useful for memory management)"""
    for key in list(self.pools.keys()):
      self.pools[key].clear()

class NEONOptimizer:
  """
  ARM NEON optimization utilities

  This class provides ARM NEON-specific optimizations for model processing and control systems.
  It includes memory pooling, optimized array operations, and performance profiling.
  """

  def __init__(self, max_pool_size: int = 15):
    """
    Initialize the NEON optimizer with memory pool and profiling capabilities

    Args:
        max_pool_size: Maximum number of arrays to keep in each size/type pool
    """
    self.memory_pool = MemoryPool(max_pool_size=max_pool_size)
    self.profiler_enabled = True
    self.profile_data = {}
    # Pre-allocated arrays for common operations
    self._temp_buffer = np.zeros(1024, dtype=np.float32)  # For temporary calculations
    self._initialized = True  # Flag to indicate proper initialization

  def neon_enabled(self) -> bool:
    """Check if NEON is available on the current ARM processor"""
    try:
      # On ARM64 systems, NEON is generally available
      import platform
      return 'ARM' in platform.machine().upper() or 'AARCH64' in platform.machine().upper()
    except:
      return False

  def optimized_array_operation_batch(self, operations: List[Tuple[np.ndarray, np.ndarray, str]]) -> List[np.ndarray]:
    """Perform batch array operations efficiently"""
    results = []
    for a, b, operation in operations:
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

      results.append(result)
    return results
  
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
  import math

  # Enhanced validation with better error handling
  if not isinstance(steer_angle, (int, float, np.number)):
    cloudlog.error(f"Invalid steer_angle type: {type(steer_angle)}")
    return 0.0

  if not isinstance(v_ego, (int, float, np.number)):
    cloudlog.error(f"Invalid v_ego type: {type(v_ego)}")
    return 0.0

  if not isinstance(roll, (int, float, np.number)):
    cloudlog.error(f"Invalid roll type: {type(roll)}")
    return 0.0

  # Check for NaN or infinite values
  try:
    if math.isnan(steer_angle) or math.isinf(steer_angle) or \
       math.isnan(v_ego) or math.isinf(v_ego) or \
       math.isnan(roll) or math.isinf(roll):
        cloudlog.warning(f"Invalid numeric values in curvature calc: steer={steer_angle}, v_ego={v_ego}, roll={roll}")
        return 0.0
  except Exception as e:
    cloudlog.error(f"Error checking for NaN/inf values: {e}")
    return 0.0

  # Early exit for low speeds - no meaningful curvature calculation possible
  if abs(v_ego) < 0.01:
      return 0.0

  # Optimized curvature calculation with more realistic physics-based model
  try:
    # More accurate curvature calculation: k = tan(steering_angle_rad) / wheelbase
    # Assuming a typical wheelbase of 2.7m and converting steering angle to radians
    # This is a simplified but more physically accurate model than the linear approximation
    wheelbase = 2.7  # meters, typical for many vehicles
    steering_ratio = 15.0  # typical steering ratio for the car being controlled
    # Only convert if we have reasonable steering angle values
    if abs(steer_angle) > 100:  # Check for possible sensor error
      cloudlog.warning(f"Unexpectedly large steering angle: {steer_angle}. Clipping to safe range.")
      steer_angle = max(min(steer_angle, 100), -100)  # Limit to reasonable range

    steer_angle_rad = (steer_angle / steering_ratio) * (math.pi / 180.0)  # Convert to radians and account for steering ratio

    # Calculate curvature using the bicycle model: curvature = tan(steering_angle) / wheelbase
    # Use a safer tan calculation to prevent overflow in extreme cases
    tan_value = math.tan(steer_angle_rad)
    if abs(tan_value) > 10:  # Prevent extremely large tan values
      tan_value = math.copysign(10, tan_value)  # Cap to ±10 with sign preserved

    curvature = tan_value / wheelbase

    # Apply vehicle-specific limits to prevent extreme values
    # Typical maximum curvature for a passenger vehicle is around 0.015 m^-1
    max_curvature = 0.015
    result = max(min(curvature, max_curvature), -max_curvature)

    # Log if we're hitting the limits frequently as it might indicate an issue
    if abs(result) >= max_curvature * 0.95:
      cloudlog.debug(f"Curvature is near max limit: {result}, indicating potential high steering demand")

    return result

  except ValueError as e:
    cloudlog.error(f"Value error in curvature calculation: {e}")
    return 0.0
  except Exception as e:
    cloudlog.error(f"Unexpected error in curvature calculation: {e}")
    # Return safe default value in case of error
    return 0.0


__all__ = [
  "NEONOptimizer", "MemoryPool", "neon_optimizer",
  "optimize_model_processing", "optimize_validation_metrics",
  "optimize_curvature_calculation", "ProfilerResult"
]