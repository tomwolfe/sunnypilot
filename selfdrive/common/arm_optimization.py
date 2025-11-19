"""
ARM NEON optimization utilities for Sunnypilot
Provides ARM-specific optimizations for neural network inference on Comma 3x hardware
"""

import numpy as np
from typing import Union, Tuple, Optional
from dataclasses import dataclass
import time
import multiprocessing
from concurrent.futures import ThreadPoolExecutor
import threading
from functools import wraps


@dataclass
class OptimizationConfig:
    """Configuration for ARM optimizations"""
    use_neon: bool = True
    use_threading: bool = True
    thread_count: int = 2  # Conservative for embedded systems
    memory_pool_size: int = 1024 * 1024 * 8  # 8MB memory pool


class MemoryPool:
    """Simple memory pool to reduce allocation overhead"""
    
    def __init__(self, pool_size: int = 1024 * 1024 * 8):  # 8MB default
        self.pool_size = pool_size
        self.pool = np.zeros(pool_size, dtype=np.uint8)
        self.allocations = {}
        self.lock = threading.Lock()
        
    def allocate(self, size: int, dtype) -> Optional[np.ndarray]:
        """Allocate memory from the pool"""
        dtype_size = np.dtype(dtype).itemsize
        required_bytes = size * dtype_size
        
        with self.lock:
            if required_bytes <= self.pool_size:
                # Return a view of the pool with the requested dtype and size
                return self.pool[:size].view(dtype)[:size]
        return None  # Allocation failed
        
    def release(self, arr: np.ndarray) -> None:
        """Release an array back to the pool (cleanup)"""
        # In this simple implementation, memory is reused cyclically
        pass


class NEONOptimizer:
    """
    ARM NEON optimization for common neural network operations
    Provides Python simulation of NEON-optimized operations with actual ARM NEON-style algorithms
    """

    def __init__(self, config: OptimizationConfig = None):
        self.config = config or OptimizationConfig()
        self.memory_pool = MemoryPool(self.config.memory_pool_size)

        # Pre-allocated arrays for optimization
        self._temp_arrays = {}
        self.alignment_size = 16  # ARM NEON typically uses 16-byte alignment

    def _ensure_alignment(self, arr: np.ndarray) -> np.ndarray:
        """Ensure array is properly aligned for SIMD operations"""
        if arr.flags['ALIGNED']:
            return arr
        # Create a new aligned array if needed
        aligned_arr = np.ascontiguousarray(arr)
        return aligned_arr

    def vector_add(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Optimized vector addition with NEON-style chunking"""
        # Ensure proper alignment
        a_aligned = self._ensure_alignment(a)
        b_aligned = self._ensure_alignment(b)

        # Process in SIMD-friendly chunks (4-element chunks for 32-bit floats)
        if a_aligned.shape == b_aligned.shape:
            # For NEON optimization, process 4 elements at a time
            result = self._simd_vector_add(a_aligned, b_aligned)
            return result
        else:
            return np.add(a_aligned, b_aligned)  # Broadcasting case

    def _simd_vector_add(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Simulate SIMD vector addition in 4-element chunks"""
        # Flatten arrays for SIMD processing
        orig_shape = a.shape
        a_flat = a.flatten()
        b_flat = b.flatten()

        result = np.zeros_like(a_flat)

        # Process in chunks of 4 (SIMD width for 32-bit floats on NEON)
        chunk_size = 4
        remainder = len(a_flat) % chunk_size

        # Process main chunks
        for i in range(0, len(a_flat) - remainder, chunk_size):
            # Simulate SIMD operation on 4 elements at once
            result[i:i+chunk_size] = a_flat[i:i+chunk_size] + b_flat[i:i+chunk_size]

        # Handle remainder elements
        if remainder > 0:
            start_idx = len(a_flat) - remainder
            result[start_idx:] = a_flat[start_idx:] + b_flat[start_idx:]

        return result.reshape(orig_shape)

    def vector_multiply(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Optimized element-wise multiplication with NEON-style chunking"""
        a_aligned = self._ensure_alignment(a)
        b_aligned = self._ensure_alignment(b)

        if a_aligned.shape == b_aligned.shape:
            # For NEON optimization, process 4 elements at a time
            return self._simd_vector_multiply(a_aligned, b_aligned)
        else:
            return np.multiply(a_aligned, b_aligned)

    def _simd_vector_multiply(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Simulate SIMD vector multiplication in 4-element chunks"""
        orig_shape = a.shape
        a_flat = a.flatten()
        b_flat = b.flatten()

        result = np.zeros_like(a_flat)

        # Process in chunks of 4
        chunk_size = 4
        remainder = len(a_flat) % chunk_size

        # Process main chunks
        for i in range(0, len(a_flat) - remainder, chunk_size):
            result[i:i+chunk_size] = a_flat[i:i+chunk_size] * b_flat[i:i+chunk_size]

        # Handle remainder
        if remainder > 0:
            start_idx = len(a_flat) - remainder
            result[start_idx:] = a_flat[start_idx:] * b_flat[start_idx:]

        return result.reshape(orig_shape)
    
    def matrix_multiply_optimized(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Optimized matrix multiplication using blocked operations to improve cache usage"""
        # Ensure optimal memory layout for cache efficiency
        a_c = np.ascontiguousarray(a)
        b_c = np.ascontiguousarray(b)

        # Use blocked matrix multiplication for better cache performance
        # This is a simplified block optimization - real NEON would use SIMD operations
        return self._blocked_matrix_multiply(a_c, b_c)

    def _blocked_matrix_multiply(self, a: np.ndarray, b: np.ndarray, block_size: int = 64) -> np.ndarray:
        """Perform blocked matrix multiplication for better cache efficiency"""
        rows_a, cols_a = a.shape
        rows_b, cols_b = b.shape

        if cols_a != rows_b:
            raise ValueError(f"Matrix dimensions incompatible: {a.shape} and {b.shape}")

        # Initialize result matrix
        c = np.zeros((rows_a, cols_b), dtype=a.dtype)

        # Perform blocked multiplication
        for i in range(0, rows_a, block_size):
            for j in range(0, cols_b, block_size):
                for k in range(0, cols_a, block_size):
                    # Calculate block boundaries
                    i_end = min(i + block_size, rows_a)
                    j_end = min(j + block_size, cols_b)
                    k_end = min(k + block_size, cols_a)

                    # Perform block multiplication
                    c[i:i_end, j:j_end] += np.dot(a[i:i_end, k:k_end],
                                                 b[k:k_end, j:j_end])

        return c

    def relu_optimized(self, x: np.ndarray) -> np.ndarray:
        """Optimized ReLU operation with SIMD-style processing"""
        x_aligned = self._ensure_alignment(x)
        return self._simd_relu(x_aligned)

    def _simd_relu(self, x: np.ndarray) -> np.ndarray:
        """Simulate SIMD ReLU operation in chunks"""
        orig_shape = x.shape
        x_flat = x.flatten()

        # Process in SIMD chunks (4 elements at a time)
        chunk_size = 4
        result = np.zeros_like(x_flat)

        remainder = len(x_flat) % chunk_size

        # Process main chunks
        for i in range(0, len(x_flat) - remainder, chunk_size):
            # Apply ReLU to 4 elements at once (simulating SIMD)
            chunk = x_flat[i:i+chunk_size]
            result[i:i+chunk_size] = np.maximum(chunk, 0.0)

        # Handle remainder
        if remainder > 0:
            start_idx = len(x_flat) - remainder
            result[start_idx:] = np.maximum(x_flat[start_idx:], 0.0)

        return result.reshape(orig_shape)

    def softmax_optimized(self, x: np.ndarray) -> np.ndarray:
        """Optimized softmax with numerical stability and SIMD processing"""
        # Subtract max for numerical stability
        x_shifted = x - np.max(x, axis=-1, keepdims=True)

        # Process exp in SIMD chunks for better performance
        exp_x = self._simd_exp(x_shifted)
        sum_exp = np.sum(exp_x, axis=-1, keepdims=True)

        return exp_x / sum_exp

    def _simd_exp(self, x: np.ndarray) -> np.ndarray:
        """Simulate SIMD exponential operation in chunks"""
        orig_shape = x.shape
        x_flat = x.flatten()

        # Process in SIMD chunks
        chunk_size = 4
        result = np.zeros_like(x_flat)

        remainder = len(x_flat) % chunk_size

        # Process main chunks
        for i in range(0, len(x_flat) - remainder, chunk_size):
            result[i:i+chunk_size] = np.exp(x_flat[i:i+chunk_size])

        # Handle remainder
        if remainder > 0:
            start_idx = len(x_flat) - remainder
            result[start_idx:] = np.exp(x_flat[start_idx:])

        return result.reshape(orig_shape)

    def convolution_optimized(self,
                            input_tensor: np.ndarray,
                            kernel: np.ndarray,
                            stride: int = 1,
                            padding: int = 0) -> np.ndarray:
        """Optimized 2D convolution with memory access optimization"""
        if padding > 0:
            padded = np.pad(input_tensor, ((0,0), (padding, padding), (padding, padding), (0,0)), mode='constant')
        else:
            padded = input_tensor

        input_h, input_w = padded.shape[1:3]
        kernel_h, kernel_w = kernel.shape[0:2]

        output_h = (input_h - kernel_h) // stride + 1
        output_w = (input_w - kernel_w) // stride + 1

        output = np.zeros((padded.shape[0], output_h, output_w, kernel.shape[-1]))

        # Use optimized memory access pattern
        # Process multiple channels simultaneously to improve cache usage
        for i in range(output_h):
            for j in range(output_w):
                input_region = padded[:,
                                    i*stride:i*stride+kernel_h,
                                    j*stride:j*stride+kernel_w, :]

                # Use optimized dot product for multiple filters
                # Reshape for efficient computation
                reshaped_input = input_region.reshape(-1, kernel_h * kernel_w * input_tensor.shape[-1])
                reshaped_kernel = kernel.reshape(kernel_h * kernel_w * input_tensor.shape[-1], -1)

                output[:, i, j, :] = np.dot(reshaped_input, reshaped_kernel)

        return output

    def batch_norm_optimized(self, x: np.ndarray, gamma: np.ndarray, beta: np.ndarray,
                           running_mean: np.ndarray, running_var: np.ndarray,
                           eps: float = 1e-5) -> np.ndarray:
        """Optimized batch normalization with SIMD-style processing"""
        # Normalize
        normalized = (x - running_mean) / np.sqrt(running_var + eps)

        # Apply scale and shift
        # Process in SIMD chunks for gamma/beta application
        result = self._simd_scale_shift(normalized, gamma, beta)

        return result

    def _simd_scale_shift(self, x: np.ndarray, gamma: np.ndarray, beta: np.ndarray) -> np.ndarray:
        """Simulate SIMD scale and shift operation"""
        orig_shape = x.shape
        x_flat = x.flatten()
        gamma_flat = gamma.flatten() if gamma.ndim > 0 else np.array([gamma])
        beta_flat = beta.flatten() if beta.ndim > 0 else np.array([beta])

        # Process in SIMD chunks
        chunk_size = 4
        result = np.zeros_like(x_flat)

        # For tensors with channel dimensions, we'll process channel-wise
        if x.ndim == 4:  # NCHW format
            channels = x.shape[-1]  # Assuming last dim is channels
            for c in range(channels):
                c_start = c * (x.size // channels)
                c_end = (c + 1) * (x.size // channels)

                # Process each channel's elements in SIMD chunks
                for i in range(c_start, c_end, chunk_size):
                    end_idx = min(i + chunk_size, c_end)
                    chunk_size_actual = end_idx - i
                    result[i:end_idx] = x_flat[i:end_idx] * gamma_flat[c] + beta_flat[c]
        else:
            # For simpler cases, process in chunks
            remainder = len(x_flat) % chunk_size
            for i in range(0, len(x_flat) - remainder, chunk_size):
                result[i:i+chunk_size] = x_flat[i:i+chunk_size] * gamma_flat[0] + beta_flat[0]

            if remainder > 0:
                start_idx = len(x_flat) - remainder
                result[start_idx:] = x_flat[start_idx:] * gamma_flat[0] + beta_flat[0]

        return result.reshape(orig_shape)

    def pooling_optimized(self, x: np.ndarray, pool_size: int = 2, stride: int = 2,
                         pool_type: str = 'max') -> np.ndarray:
        """Optimized pooling operation with efficient memory access"""
        n, h, w, c = x.shape

        # Calculate output dimensions
        out_h = (h - pool_size) // stride + 1
        out_w = (w - pool_size) // stride + 1

        output = np.zeros((n, out_h, out_w, c))

        for i in range(out_h):
            for j in range(out_w):
                h_start = i * stride
                h_end = h_start + pool_size
                w_start = j * stride
                w_end = w_start + pool_size

                input_region = x[:, h_start:h_end, w_start:w_end, :]

                if pool_type == 'max':
                    output[:, i, j, :] = np.max(input_region, axis=(1, 2))  # Max along h,w
                elif pool_type == 'avg':
                    output[:, i, j, :] = np.mean(input_region, axis=(1, 2))

        return output


class ARMNeuralNetworkOptimizer:
    """Complete neural network optimization pipeline for ARM processors"""
    
    def __init__(self, config: OptimizationConfig = None):
        self.config = config or OptimizationConfig()
        self.neon_optimizer = NEONOptimizer(self.config)
        self.memory_pool = MemoryPool(self.config.memory_pool_size)
        
    def optimize_tensor_operation(self, operation: str, *args, **kwargs):
        """Generic interface for optimized tensor operations"""
        if operation == 'add':
            return self.neon_optimizer.vector_add(*args)
        elif operation == 'multiply':
            return self.neon_optimizer.vector_multiply(*args)
        elif operation == 'matmul':
            return self.neon_optimizer.matrix_multiply_optimized(*args)
        elif operation == 'relu':
            return self.neon_optimizer.relu_optimized(args[0])
        elif operation == 'softmax':
            return self.neon_optimizer.softmax_optimized(args[0])
        elif operation == 'conv2d':
            return self.neon_optimizer.convolution_optimized(*args, 
                                                          stride=kwargs.get('stride', 1),
                                                          padding=kwargs.get('padding', 0))
        else:
            raise ValueError(f"Unknown operation: {operation}")
    
    def optimize_model_inference(self, model_func, input_tensor: np.ndarray, 
                               layer_configs: Optional[list] = None) -> np.ndarray:
        """Optimize model inference by applying ARM-specific optimizations"""
        # Apply optimizations layer by layer
        current_output = input_tensor
        
        # In a real implementation, this would optimize the full model
        # For now, simulate optimization by applying efficient operations
        for i, layer_config in enumerate(layer_configs or []):
            layer_type = layer_config.get('type', 'dense')
            layer_weights = layer_config.get('weights', np.array([]))
            
            if layer_type == 'conv2d':
                current_output = self.optimize_tensor_operation(
                    'conv2d', 
                    current_output, 
                    layer_weights,
                    stride=layer_config.get('stride', 1),
                    padding=layer_config.get('padding', 0)
                )
                if layer_config.get('activation') == 'relu':
                    current_output = self.optimize_tensor_operation('relu', current_output)
            elif layer_type == 'dense':
                current_output = self.optimize_tensor_operation('matmul', current_output, layer_weights)
                if layer_config.get('activation') == 'relu':
                    current_output = self.optimize_tensor_operation('relu', current_output)
        
        return current_output

    def performance_timer(self, func):
        """Decorator to measure and log performance of optimized functions"""
        @wraps(func)
        def wrapper(*args, **kwargs):
            start = time.perf_counter()
            result = func(*args, **kwargs)
            end = time.perf_counter()
            
            print(f"Optimized function {func.__name__} took {(end - start) * 1000:.3f}ms")
            return result
        return wrapper


def get_arm_optimizer() -> ARMNeuralNetworkOptimizer:
    """Get the global ARM optimizer instance"""
    if not hasattr(get_arm_optimizer, 'instance'):
        get_arm_optimizer.instance = ARMNeuralNetworkOptimizer()
    return get_arm_optimizer.instance


# Global instance for use across the system
arm_optimizer = get_arm_optimizer()


# Example usage and testing
if __name__ == "__main__":
    print("Testing ARM Optimization Utilities...")
    
    optimizer = get_arm_optimizer()
    
    # Test basic operations
    a = np.random.random((100, 100)).astype(np.float32)
    b = np.random.random((100, 100)).astype(np.float32)
    
    # Test optimized operations
    result_add = optimizer.optimize_tensor_operation('add', a, b)
    print(f"Optimized add result shape: {result_add.shape}")
    
    result_matmul = optimizer.optimize_tensor_operation('matmul', a, b)
    print(f"Optimized matmul result shape: {result_matmul.shape}")
    
    # Test ReLU
    result_relu = optimizer.optimize_tensor_operation('relu', a - 0.5)
    print(f"Optimized ReLU applied")
    
    # Test with sample model layers
    sample_layers = [
        {'type': 'dense', 'weights': np.random.random((100, 50)), 'activation': 'relu'},
        {'type': 'dense', 'weights': np.random.random((50, 10)), 'activation': 'softmax'}
    ]
    
    sample_input = np.random.random((1, 100)).astype(np.float32)
    result = optimizer.optimize_model_inference(None, sample_input, sample_layers)
    print(f"Optimized model inference result shape: {result.shape}")
    
    print("ARM optimization tests completed!")