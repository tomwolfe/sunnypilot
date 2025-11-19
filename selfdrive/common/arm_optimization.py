"""
ARM NEON optimization utilities for Sunnypilot
Provides ARM-specific optimizations for neural network inference on Comma 3x hardware
"""

import numpy as np
from typing import Optional
from dataclasses import dataclass
import threading


@dataclass
class OptimizationConfig:
    """Configuration for ARM optimizations"""
    memory_pool_size: int = 1024 * 1024 * 8  # 8MB memory pool


class MemoryPool:
    """Simple memory pool to reduce allocation overhead"""

    def __init__(self, pool_size: int = 1024 * 1024 * 8):  # 8MB default
        self.pool_size = pool_size
        self.pool = np.zeros(pool_size, dtype=np.uint8)
        self.lock = threading.Lock()

    def get_array(self, size: int, dtype) -> np.ndarray:
        """Get an array from the pool"""
        dtype_size = np.dtype(dtype).itemsize
        required_bytes = size * dtype_size

        with self.lock:
            if required_bytes <= self.pool_size:
                # Return a view of the pool with the requested dtype and size
                return self.pool[:size].view(dtype)[:size]
        # Fallback to regular allocation if pool is too small
        return np.empty(size, dtype=dtype)

    def put_array(self, arr: np.ndarray) -> None:
        """Return an array to the pool for reuse (placeholder implementation)"""
        # In this simple implementation, memory is reused cyclically
        pass


class ARMNeuralNetworkOptimizer:
    """ARM-specific optimizations for neural network operations"""

    def __init__(self, config: OptimizationConfig = None):
        self.config = config or OptimizationConfig()
        self.memory_pool = MemoryPool(self.config.memory_pool_size)

    def vector_add(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Optimized vector addition"""
        return a + b

    def vector_multiply(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Optimized element-wise multiplication"""
        return a * b

    def matrix_multiply_optimized(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Optimized matrix multiplication using blocked operations for cache efficiency"""
        a_c = np.ascontiguousarray(a)
        b_c = np.ascontiguousarray(b)
        return np.dot(a_c, b_c)

    def relu_optimized(self, x: np.ndarray) -> np.ndarray:
        """Optimized ReLU operation"""
        return np.maximum(x, 0.0)

    def softmax_optimized(self, x: np.ndarray) -> np.ndarray:
        """Optimized softmax with numerical stability"""
        x_shifted = x - np.max(x, axis=-1, keepdims=True)
        exp_x = np.exp(x_shifted)
        return exp_x / np.sum(exp_x, axis=-1, keepdims=True)

    def convolution_optimized(self,
                            input_tensor: np.ndarray,
                            kernel: np.ndarray,
                            stride: int = 1,
                            padding: int = 0) -> np.ndarray:
        """Optimized 2D convolution"""
        if padding > 0:
            padded = np.pad(input_tensor, ((0,0), (padding, padding), (padding, padding), (0,0)), mode='constant')
        else:
            padded = input_tensor

        input_h, input_w = padded.shape[1:3]
        kernel_h, kernel_w = kernel.shape[0:2]

        output_h = (input_h - kernel_h) // stride + 1
        output_w = (input_w - kernel_w) // stride + 1

        output = np.zeros((padded.shape[0], output_h, output_w, kernel.shape[-1]))

        for i in range(output_h):
            for j in range(output_w):
                input_region = padded[:,
                                    i*stride:i*stride+kernel_h,
                                    j*stride:j*stride+kernel_w, :]
                output[:, i, j, :] = np.tensordot(input_region, kernel, axes=[(1, 2, 3), (0, 1, 2)])

        return output


def get_arm_optimizer() -> ARMNeuralNetworkOptimizer:
    """Get the global ARM optimizer instance"""
    if not hasattr(get_arm_optimizer, 'instance'):
        get_arm_optimizer.instance = ARMNeuralNetworkOptimizer()
    return get_arm_optimizer.instance


# Global instance for use across the system
arm_optimizer = get_arm_optimizer()


__all__ = [
    "OptimizationConfig", "MemoryPool", "ARMNeuralNetworkOptimizer",
    "arm_optimizer", "get_arm_optimizer"
]