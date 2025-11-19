"""
Advanced Memory Management and Optimization for Sunnypilot
Provides efficient memory allocation, pooling, and management for ARM processors
"""

import numpy as np
from typing import Dict, Optional, Any, Callable, List
from dataclasses import dataclass
import threading
import time
import weakref
from collections import defaultdict, deque
import gc


@dataclass
class MemoryBlock:
    """Represents a memory block in the pool"""
    data: np.ndarray
    size: int
    dtype: np.dtype
    shape: tuple
    used: bool
    last_used: float
    reuse_count: int


class MemoryPool:
    """
    Advanced memory pool that manages reusable memory blocks to reduce allocations
    """
    
    def __init__(self, initial_pool_size: int = 1024*1024*16,  # 16MB initial
                 max_pool_size: int = 1024*1024*64,            # 64MB max
                 gc_threshold: int = 100):                     # GC every N allocations
        self.initial_pool_size = initial_pool_size
        self.max_pool_size = max_pool_size
        self.gc_threshold = gc_threshold
        
        # Memory pools organized by dtype and size ranges
        self.pools: Dict[str, List[MemoryBlock]] = defaultdict(list)
        self.lock = threading.Lock()
        self.allocation_count = 0
        self.hit_count = 0  # Successful pool hits
        self.miss_count = 0  # Pool misses requiring new allocation
        
        # Size ranges for efficient pooling
        self.size_ranges = [
            (0, 1024),      # Small allocations (0-1KB)
            (1024, 1024*8), # Medium allocations (1KB-8KB)
            (1024*8, 1024*64), # Large allocations (8KB-64KB)
            (1024*64, 1024*256), # Very large (64KB-256KB)
            (1024*256, float('inf'))  # Massive allocations
        ]
        
        # Initialize with some common blocks
        self._initialize_common_blocks()
    
    def _initialize_common_blocks(self):
        """Initialize the pool with commonly used array shapes"""
        common_shapes = [
            ((32, 13), np.float32),   # Model output
            ((64,), np.float32),      # Small vectors
            ((128,), np.float32),     # Medium vectors
            ((64, 64), np.float32),   # Small matrices
            ((128, 128), np.float32), # Medium matrices
            ((3, 3), np.float32),     # Transform matrices
            ((4, 4), np.float32),     # Transform matrices
            ((100,), np.int32),       # Integer arrays
        ]
        
        with self.lock:
            for shape, dtype in common_shapes:
                self._create_block(shape, dtype)
    
    def _get_size_range(self, size: int) -> str:
        """Get the appropriate size range key for a given size"""
        for min_size, max_size in self.size_ranges:
            if min_size <= size < max_size:
                return f"{min_size}-{max_size}"
        return f"{self.size_ranges[-1][0]}-inf"
    
    def _create_block(self, shape: tuple, dtype: np.dtype) -> MemoryBlock:
        """Create a new memory block"""
        size = int(np.prod(shape) * dtype.itemsize)
        array = np.zeros(shape, dtype=dtype)
        
        block = MemoryBlock(
            data=array,
            size=size,
            dtype=dtype,
            shape=shape,
            used=False,
            last_used=time.time(),
            reuse_count=0
        )
        
        size_range = self._get_size_range(size)
        self.pools[size_range].append(block)
        
        return block
    
    def allocate(self, shape: tuple, dtype: np.dtype = np.float32, 
                 fill_value: Optional[float] = None) -> Optional[np.ndarray]:
        """Allocate memory from the pool or create new if needed"""
        with self.lock:
            self.allocation_count += 1
            target_size = int(np.prod(shape) * dtype.itemsize)
            size_range = self._get_size_range(target_size)
            
            # Look for an available block in the appropriate size range
            for i, block in enumerate(self.pools[size_range]):
                if not block.used and block.shape == shape and block.dtype == dtype:
                    block.used = True
                    block.last_used = time.time()
                    block.reuse_count += 1
                    
                    self.hit_count += 1
                    
                    # Optionally fill the array
                    if fill_value is not None:
                        block.data.fill(fill_value)
                    
                    return block.data
            
            # No suitable block found in pool
            self.miss_count += 1
            
            # Check if we need to run cleanup
            if self.allocation_count % self.gc_threshold == 0:
                self._cleanup_unused_blocks()
            
            # Create new block if under limits
            if self._current_pool_size() < self.max_pool_size:
                new_block = self._create_block(shape, dtype)
                new_block.used = True
                new_block.reuse_count = 1
                
                if fill_value is not None:
                    new_block.data.fill(fill_value)
                
                return new_block.data
            else:
                # Pool is full, create temporary array (less efficient)
                result = np.zeros(shape, dtype=dtype)
                if fill_value is not None:
                    result.fill(fill_value)
                return result
    
    def release(self, array: np.ndarray) -> bool:
        """Release an array back to the pool"""
        with self.lock:
            # Find the corresponding block and mark as unused
            target_size = array.size * array.dtype.itemsize
            size_range = self._get_size_range(target_size)
            
            for block in self.pools[size_range]:
                if block.data is array and block.used:
                    block.used = False
                    block.last_used = time.time()
                    return True
            
            return False  # Array not found in pool
    
    def _cleanup_unused_blocks(self):
        """Remove old unused blocks to free memory"""
        current_time = time.time()
        cleanup_threshold = 10.0  # Remove unused blocks after 10 seconds
        
        for size_range, blocks in self.pools.items():
            # Keep only recent unused blocks
            remaining_blocks = []
            for block in blocks:
                if block.used or (current_time - block.last_used < cleanup_threshold):
                    remaining_blocks.append(block)
            
            self.pools[size_range] = remaining_blocks
    
    def _current_pool_size(self) -> int:
        """Get current total pool size"""
        total = 0
        for blocks in self.pools.values():
            for block in blocks:
                total += block.size
        return total
    
    def get_stats(self) -> Dict[str, Any]:
        """Get memory pool statistics"""
        with self.lock:
            total_blocks = sum(len(blocks) for blocks in self.pools.values())
            used_blocks = sum(1 for blocks in self.pools.values() 
                             for block in blocks if block.used)
            
            hit_rate = self.hit_count / max(self.allocation_count, 1)
            
            return {
                'allocation_count': self.allocation_count,
                'hit_count': self.hit_count,
                'miss_count': self.miss_count,
                'hit_rate': hit_rate,
                'total_blocks': total_blocks,
                'used_blocks': used_blocks,
                'free_blocks': total_blocks - used_blocks,
                'pool_size_bytes': self._current_pool_size(),
                'max_pool_size': self.max_pool_size
            }


class TensorManager:
    """
    Manages frequently used tensors with pre-allocation and reuse
    """
    
    def __init__(self, memory_pool: MemoryPool):
        self.memory_pool = memory_pool
        self.tensors: Dict[str, np.ndarray] = {}
        self.tensor_info: Dict[str, Dict[str, Any]] = {}
        self.lock = threading.Lock()
    
    def register_tensor(self, name: str, shape: tuple, dtype: np.dtype = np.float32,
                       initializer: Optional[Callable] = None) -> np.ndarray:
        """Register a tensor that will be reused frequently"""
        with self.lock:
            if name not in self.tensors:
                tensor = self.memory_pool.allocate(shape, dtype)
                if tensor is not None:
                    if initializer:
                        initializer(tensor)
                    self.tensors[name] = tensor
                    self.tensor_info[name] = {
                        'shape': shape,
                        'dtype': dtype,
                        'last_access': time.time(),
                        'access_count': 0
                    }
                else:
                    # Fallback if pool allocation fails
                    tensor = np.zeros(shape, dtype=dtype)
                    if initializer:
                        initializer(tensor)
                    self.tensors[name] = tensor
                    self.tensor_info[name] = {
                        'shape': shape,
                        'dtype': dtype,
                        'last_access': time.time(),
                        'access_count': 0,
                        'pooled': False
                    }
            
            self.tensor_info[name]['access_count'] += 1
            self.tensor_info[name]['last_access'] = time.time()
            
            return self.tensors[name]
    
    def get_tensor(self, name: str) -> Optional[np.ndarray]:
        """Get a registered tensor"""
        with self.lock:
            if name in self.tensors:
                self.tensor_info[name]['access_count'] += 1
                self.tensor_info[name]['last_access'] = time.time()
                return self.tensors[name]
            return None
    
    def clear_inactive_tensors(self, max_age: float = 30.0):
        """Clear tensors that haven't been accessed recently"""
        with self.lock:
            current_time = time.time()
            for name, info in list(self.tensor_info.items()):
                if current_time - info['last_access'] > max_age:
                    if self.memory_pool.release(self.tensors[name]):
                        # Successfully returned to pool
                        del self.tensors[name]
                        del self.tensor_info[name]
                    else:
                        # Not pooled, but clear anyway since inactive
                        del self.tensors[name]
                        del self.tensor_info[name]


class MemoryOptimizer:
    """
    Main memory optimization controller that combines pooling and tensor management
    """
    
    def __init__(self):
        self.memory_pool = MemoryPool()
        self.tensor_manager = TensorManager(self.memory_pool)
        self.optimization_stats = {
            'allocation_savings': 0,
            'reused_arrays': 0,
            'temporary_arrays': 0
        }
        self.active_tensors: Dict[str, weakref.ReferenceType] = {}
        self.lock = threading.Lock()
    
    def create_optimized_array(self, shape: tuple, dtype: np.dtype = np.float32,
                             name: Optional[str] = None, 
                             reuse_pattern: str = 'temp') -> np.ndarray:
        """
        Create an array with optimization based on reuse pattern
        
        Args:
            shape: Array shape
            dtype: Array data type
            name: Optional name for registered tensors
            reuse_pattern: 'temp', 'frequent', or 'persistent'
        """
        if reuse_pattern == 'frequent' and name:
            # Register as frequently reused tensor
            return self.tensor_manager.register_tensor(name, shape, dtype)
        elif reuse_pattern == 'persistent' and name:
            # Register as persistent tensor
            tensor = self.memory_pool.allocate(shape, dtype)
            if tensor is not None:
                self.active_tensors[name] = weakref.ref(tensor)
                return tensor
            else:
                tensor = np.zeros(shape, dtype=dtype)
                self.active_tensors[name] = weakref.ref(tensor)
                return tensor
        else:
            # Temporary allocation, try pool first
            pooled_array = self.memory_pool.allocate(shape, dtype)
            if pooled_array is not None:
                self.optimization_stats['reused_arrays'] += 1
                return pooled_array
            else:
                self.optimization_stats['temporary_arrays'] += 1
                # Fallback to regular allocation
                return np.zeros(shape, dtype=dtype)
    
    def release_array(self, array: np.ndarray) -> bool:
        """Release an array back to the pool"""
        return self.memory_pool.release(array)
    
    def register_frequent_tensor(self, name: str, shape: tuple, 
                               dtype: np.dtype = np.float32,
                               initializer: Optional[Callable] = None) -> np.ndarray:
        """Register a tensor that will be used frequently"""
        return self.tensor_manager.register_tensor(name, shape, dtype, initializer)
    
    def get_registered_tensor(self, name: str) -> Optional[np.ndarray]:
        """Get a registered tensor"""
        return self.tensor_manager.get_tensor(name)
    
    def get_memory_stats(self) -> Dict[str, Any]:
        """Get comprehensive memory optimization statistics"""
        pool_stats = self.memory_pool.get_stats()
        tensor_count = len(self.tensor_manager.tensors)
        
        return {
            **pool_stats,
            'registered_tensors': tensor_count,
            'optimization_stats': self.optimization_stats.copy()
        }
    
    def cleanup_inactive_tensors(self, max_age: float = 30.0):
        """Clean up inactive tensors"""
        self.tensor_manager.clear_inactive_tensors(max_age)
        
        # Also clear weak references to dead objects
        with self.lock:
            dead_refs = []
            for name, ref in self.active_tensors.items():
                if ref() is None:  # Object has been garbage collected
                    dead_refs.append(name)
            
            for name in dead_refs:
                del self.active_tensors[name]


# Global memory optimizer instance for use across the system
def get_memory_optimizer() -> MemoryOptimizer:
    """Get the global memory optimizer instance"""
    if not hasattr(get_memory_optimizer, 'instance'):
        get_memory_optimizer.instance = MemoryOptimizer()
    return get_memory_optimizer.instance


def get_optimized_array(shape: tuple, dtype: np.dtype = np.float32,
                       name: Optional[str] = None,
                       reuse_pattern: str = 'temp') -> np.ndarray:
    """Convenience function to get an optimized array"""
    optimizer = get_memory_optimizer()
    return optimizer.create_optimized_array(shape, dtype, name, reuse_pattern)


def release_optimized_array(array: np.ndarray) -> bool:
    """Convenience function to release an optimized array"""
    optimizer = get_memory_optimizer()
    return optimizer.release_array(array)


# Example usage and testing
if __name__ == "__main__":
    print("Testing Advanced Memory Management...")
    
    # Create memory optimizer
    mem_opt = get_memory_optimizer()
    
    # Test 1: Basic allocation and deallocation
    print("Test 1: Basic allocation and deallocation")
    arr1 = get_optimized_array((128, 128), np.float32, "test_matrix", "frequent")
    arr2 = get_optimized_array((64, 64), np.float32, "temp_matrix", "temp")
    
    print(f"  Allocated arrays: shape {arr1.shape}, shape {arr2.shape}")
    
    # Release temporary array
    release_optimized_array(arr2)
    print("  Released temporary array back to pool")
    
    # Test 2: Registered tensors
    print("\nTest 2: Registered tensors")
    frequent_tensor = mem_opt.register_frequent_tensor("model_weights", (256, 128))
    print(f"  Registered tensor: {frequent_tensor.shape}")
    
    retrieved_tensor = mem_opt.get_registered_tensor("model_weights")
    print(f"  Retrieved tensor: {retrieved_tensor is frequent_tensor}")
    
    # Test 3: Performance comparison
    print("\nTest 3: Performance comparison")
    import time
    
    # Time regular allocation
    start_time = time.time()
    for _ in range(1000):
        regular_arr = np.zeros((32, 32), dtype=np.float32)
    regular_time = time.time() - start_time
    
    # Time optimized allocation
    start_time = time.time()
    for _ in range(1000):
        opt_arr = get_optimized_array((32, 32), np.float32, "temp", "temp")
        release_optimized_array(opt_arr)
    optimized_time = time.time() - start_time
    
    print(f"  Regular allocation time: {regular_time:.4f}s")
    print(f"  Optimized allocation time: {optimized_time:.4f}s")
    
    # Check memory stats
    stats = mem_opt.get_memory_stats()
    print(f"\nMemory Statistics:")
    print(f"  Allocation hit rate: {stats['hit_rate']:.2%}")
    print(f"  Total allocations: {stats['allocation_count']}")
    print(f"  Used blocks: {stats['used_blocks']}")
    print(f"  Free blocks: {stats['free_blocks']}")
    print(f"  Pool size: {stats['pool_size_bytes'] / (1024*1024):.2f} MB")
    
    print("\nAdvanced memory management test completed!")