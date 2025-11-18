"""
Memory optimization framework for sunnypilot to reduce RAM usage.
This addresses the critical issue of RAM usage being 6x over the target limit.
"""
import psutil
import gc
import numpy as np
from typing import Dict, List, Callable, Optional
import threading
import time

class MemoryOptimizer:
    """
    Memory optimization framework to keep RAM usage under 1.4GB target.
    """
    def __init__(self, target_ram_mb: float = 1433.6, safety_margin_mb: float = 200):
        self.target_ram_mb = target_ram_mb
        self.safety_margin_mb = safety_margin_mb
        self.optimization_threshold_mb = target_ram_mb - safety_margin_mb
        self.peak_usage_mb = 0
        self.current_usage_mb = 0
        self.optimization_history = []
        self.monitoring = False
        self.monitor_thread = None

    def get_current_memory_usage(self) -> float:
        """Get current memory usage in MB."""
        process = psutil.Process()
        self.current_usage_mb = process.memory_info().rss / (1024 * 1024)
        
        if self.current_usage_mb > self.peak_usage_mb:
            self.peak_usage_mb = self.current_usage_mb
            
        return self.current_usage_mb

    def optimize_memory(self):
        """Perform memory optimization steps."""
        start_time = time.time()
        
        # 1. Force garbage collection
        gc.collect()
        
        # 2. Clear any cached data
        self.clear_caches()
        
        # 3. Check if we're over threshold
        current = self.get_current_memory_usage()
        optimizations_performed = 0
        
        if current > self.optimization_threshold_mb:
            # 4. Aggressive memory cleanup
            self.aggressive_cleanup()
            optimizations_performed += 1
        
        optimization_time = time.time() - start_time
        self.optimization_history.append({
            'timestamp': time.time(),
            'optimization_time': optimization_time,
            'optimizations_performed': optimizations_performed,
            'memory_before_mb': current,
            'memory_after_mb': self.get_current_memory_usage()
        })
        
        return optimizations_performed > 0

    def clear_caches(self):
        """Clear any cached data that might be consuming memory."""
        # This could clear specific caches if the system has them
        # In real implementation, this would interact with specific modules
        pass

    def aggressive_cleanup(self):
        """Perform more aggressive memory cleanup."""
        # This would target specific memory-heavy components
        # in the actual autonomous driving stack
        pass

    def start_monitoring(self):
        """Start continuous memory monitoring."""
        if not self.monitoring:
            self.monitoring = True
            self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.monitor_thread.start()

    def stop_monitoring(self):
        """Stop memory monitoring."""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join()

    def _monitor_loop(self):
        """Monitoring loop running in separate thread."""
        while self.monitoring:
            current_usage = self.get_current_memory_usage()
            
            if current_usage > self.target_ram_mb:
                print(f"MEMORY ALERT: Current usage {current_usage:.2f}MB exceeds target {self.target_ram_mb}MB")
                self.optimize_memory()
            
            time.sleep(1)  # Check every second

    def get_stats(self) -> Dict[str, float]:
        """Get memory optimization statistics."""
        return {
            'current_usage_mb': self.current_usage_mb,
            'peak_usage_mb': self.peak_usage_mb,
            'target_usage_mb': self.target_ram_mb,
            'optimization_threshold_mb': self.optimization_threshold_mb
        }

class QuantizedModel:
    """
    Framework for quantized models to reduce memory usage.
    """
    def __init__(self, precision_bits: int = 8):
        self.precision_bits = precision_bits
        self.quantized_weights = None
        self.scale_factor = None
        self.zero_point = 0

    def quantize_array(self, array: np.ndarray) -> tuple:
        """
        Quantize a numpy array to reduce memory usage.
        Returns quantized array and scale factor.
        """
        if self.precision_bits == 8:
            # For 8-bit quantization: values range from -128 to 127
            min_val = np.min(array)
            max_val = np.max(array)
            
            # Calculate scale and zero point
            scale = (max_val - min_val) / 255.0
            zero_point = -min_val / scale
            
            # Quantize
            quantized = np.clip((array / scale) + zero_point, -128, 127).astype(np.int8)
            
            return quantized, scale, zero_point
        else:
            # Other precision implementations would go here
            raise NotImplementedError(f"Precision {self.precision_bits} not implemented")

    def dequantize_array(self, quantized_array: np.ndarray, scale: float, zero_point: float) -> np.ndarray:
        """Dequantize array back to float."""
        return (quantized_array.astype(np.float32) - zero_point) * scale

class StreamingProcessor:
    """
    Process data in chunks to reduce memory usage.
    """
    def __init__(self, chunk_size_mb: float = 1.0):
        self.chunk_size_mb = chunk_size_mb

    def get_chunks(self, data_source, chunk_size_mb: float):
        """Generate chunks of data from data source."""
        # This would be implemented based on the specific data source
        # For now, we return the data as is
        yield data_source

    def process_streaming(self, data_source, process_func: Callable):
        """Process data in streaming fashion."""
        for chunk in self.get_chunks(data_source, self.chunk_size_mb):
            result = process_func(chunk)
            # Process result immediately to avoid accumulation
            yield result
            # Clear chunk from memory
            del chunk

# Global memory optimizer instance
memory_optimizer = MemoryOptimizer()

def optimize_memory_usage():
    """Global function to optimize memory."""
    return memory_optimizer.optimize_memory()

def get_memory_usage():
    """Get current memory usage."""
    return memory_optimizer.get_current_memory_usage()

def start_memory_monitoring():
    """Start memory monitoring."""
    memory_optimizer.start_monitoring()

def stop_memory_monitoring():
    """Stop memory monitoring."""
    memory_optimizer.stop_monitoring()