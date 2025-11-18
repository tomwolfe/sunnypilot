"""
Memory optimization module for the comma three platform.
Implements memory-efficient algorithms and data structures to meet 1.4GB target.
"""
import gc
import psutil
import numpy as np
from typing import Dict, List, Optional, Any, Callable
from collections import OrderedDict
from dataclasses import dataclass
from openpilot.selfdrive.common.metrics import Metrics, record_metric
import time

@dataclass
class MemoryProfile:
    """Memory usage profile for a specific component."""
    component_name: str
    peak_usage_mb: float
    current_usage_mb: float
    allocated_tensors: int
    timestamp: float

class MemoryLimiter:
    """Manages memory allocation to stay within hardware limits."""
    
    def __init__(self, target_mb: float = 1433.6):  # 1.4GB target
        self.target_mb = target_mb
        self.current_usage_mb = 0.0
        self.peak_usage_mb = 0.0
        self.allocation_history: List[MemoryProfile] = []
        self.tensor_cache: OrderedDict = OrderedDict()  # LRU cache for tensors
        self.max_cache_size = 100  # Limit cache size
        
    def get_current_memory_usage(self) -> float:
        """Get current system memory usage in MB."""
        memory = psutil.virtual_memory()
        self.current_usage_mb = memory.used / (1024 * 1024)
        if self.current_usage_mb > self.peak_usage_mb:
            self.peak_usage_mb = self.current_usage_mb
            
        # Record metric
        record_metric(Metrics.RAM_USAGE_MB, self.current_usage_mb, {
            "source": "memory_limiter",
            "component": "system_overview"
        })
        
        return self.current_usage_mb
    
    def allocate_tensor(self, shape: tuple, dtype: type = np.float32, name: str = "unnamed") -> np.ndarray:
        """Allocate a memory-efficient tensor."""
        # Calculate expected size
        element_size = np.dtype(dtype).itemsize
        expected_size_mb = (np.prod(shape) * element_size) / (1024 * 1024)
        
        # Check if we're near our limit
        current_mb = self.get_current_memory_usage()
        projected_mb = current_mb + expected_size_mb
        
        # If we're over budget, try to clear cache
        if projected_mb > self.target_mb:
            self._clear_cache()
        
        # Create the tensor
        tensor = np.zeros(shape, dtype=dtype)
        
        # Add to cache with LRU management
        if name != "unnamed":
            self._add_to_cache(name, tensor)
        
        # Record memory allocation
        record_metric(Metrics.RAM_USAGE_MB, self.get_current_memory_usage(), {
            "operation": "tensor_allocation",
            "tensor_name": name,
            "shape": shape,
            "projected_size_mb": expected_size_mb
        })
        
        return tensor
    
    def _add_to_cache(self, name: str, tensor: np.ndarray):
        """Add tensor to LRU cache."""
        if len(self.tensor_cache) >= self.max_cache_size:
            # Remove oldest item
            self.tensor_cache.popitem(last=False)
        
        self.tensor_cache[name] = tensor
    
    def get_cached_tensor(self, name: str) -> Optional[np.ndarray]:
        """Get tensor from cache, moving it to front (LRU)."""
        if name in self.tensor_cache:
            tensor = self.tensor_cache.pop(name)
            self.tensor_cache[name] = tensor  # Move to end (most recent)
            return tensor
        return None
    
    def _clear_cache(self):
        """Clear tensor cache to free memory."""
        cache_size_before = len(self.tensor_cache)
        self.tensor_cache.clear()
        gc.collect()  # Force garbage collection
        
        record_metric(Metrics.RAM_USAGE_MB, self.get_current_memory_usage(), {
            "operation": "cache_clear",
            "tensors_cleared": cache_size_before
        })
    
    def optimize_memory(self):
        """Perform general memory optimization."""
        # Force garbage collection
        collected = gc.collect()
        
        # Clear any unnecessary cached items
        if len(self.tensor_cache) > self.max_cache_size * 0.8:
            items_to_remove = list(self.tensor_cache.keys())[:int(len(self.tensor_cache) * 0.2)]
            for key in items_to_remove:
                del self.tensor_cache[key]
        
        current_mb = self.get_current_memory_usage()
        
        record_metric(Metrics.RAM_USAGE_MB, current_mb, {
            "operation": "memory_optimization",
            "garbage_collected": collected,
            "cache_size_after": len(self.tensor_cache)
        })

class QuantizedModelRunner:
    """Runs quantized models for memory efficiency on ARM platforms."""
    
    def __init__(self, memory_limiter: MemoryLimiter):
        self.memory_limiter = memory_limiter
        self.models: Dict[str, Any] = {}
        self.input_cache: Dict[str, np.ndarray] = {}
    
    def register_model(self, name: str, input_shape: tuple, output_shape: tuple):
        """Register a model to track its memory usage."""
        # Create quantized placeholder model
        input_tensor = self.memory_limiter.allocate_tensor(input_shape, dtype=np.int8, name=f"{name}_input")
        output_tensor = self.memory_limiter.allocate_tensor(output_shape, dtype=np.int8, name=f"{name}_output")
        
        self.models[name] = {
            "input_shape": input_shape,
            "output_shape": output_shape,
            "input_tensor": input_tensor,
            "output_tensor": output_tensor
        }
        
        record_metric(Metrics.RAM_USAGE_MB, self.memory_limiter.get_current_memory_usage(), {
            "operation": "model_registration",
            "model_name": name,
            "input_shape": input_shape,
            "output_shape": output_shape
        })
    
    def run_model(self, name: str, input_data: np.ndarray) -> np.ndarray:
        """Run a quantized model with memory efficiency."""
        if name not in self.models:
            raise ValueError(f"Model {name} not registered")
        
        model_info = self.models[name]
        
        # Apply quantization to input
        if input_data.dtype != np.int8:
            quantized_input = (input_data * 255).astype(np.int8)
        else:
            quantized_input = input_data
        
        # Copy to pre-allocated tensor to avoid new allocations
        model_info["input_tensor"][:min(quantized_input.shape[0], model_info["input_tensor"].shape[0])] = quantized_input[:model_info["input_tensor"].shape[0]]
        
        # Simulate model run (in real implementation, this would run actual model)
        start_time = time.time()
        
        # Perform simplified computation (placeholder for actual model inference)
        output = np.copy(model_info["output_tensor"])
        
        # Record computation metrics
        computation_time = time.time() - start_time
        record_metric(Metrics.PERCEPTION_LATENCY_MS, computation_time * 1000, {
            "operation": "quantized_model_inference",
            "model_name": name,
            "input_size": input_data.size,
            "computation_time": computation_time
        })
        
        return output

class MemoryEfficientPerception:
    """Memory-efficient perception system for resource-constrained platforms."""
    
    def __init__(self):
        self.memory_limiter = MemoryLimiter(target_mb=1433.6)
        self.model_runner = QuantizedModelRunner(self.memory_limiter)
        
        # Register models with memory-efficient shapes
        self.model_runner.register_model("object_detection", (1, 128, 128, 3), (1, 10, 4))  # Quantized SSD-like
        self.model_runner.register_model("traffic_light_detection", (1, 64, 64, 3), (1, 3))  # Simplified classifier
        self.model_runner.register_model("lane_detection", (1, 128, 128, 1), (1, 50, 2))  # Lane point detection
    
    def detect_objects(self, image: np.ndarray) -> Dict[str, Any]:
        """Detect objects with memory efficiency."""
        start_time = time.time()
        
        # Preprocess image to quantized format
        if len(image.shape) == 3:
            h, w, c = image.shape
            # Resize and quantize for efficient processing
            processed_img = (image / 255.0 * 255).astype(np.int8)
        else:
            processed_img = image.astype(np.int8)
        
        # Run object detection model
        detections = self.model_runner.run_model("object_detection", processed_img)
        
        # Process results
        objects = []
        for i, det in enumerate(detections[0]):
            if i < len(detections[0]) and det[2] > 0.5:  # Confidence threshold
                objects.append({
                    "type": "vehicle" if i % 3 == 0 else "pedestrian" if i % 3 == 1 else "other",
                    "bbox": det[:4].tolist(),
                    "confidence": float(det[2]),
                    "distance": self._estimate_distance(det[:4])
                })
        
        # Record metrics
        processing_time = time.time() - start_time
        record_metric(Metrics.PERCEPTION_LATENCY_MS, processing_time * 1000, {
            "operation": "object_detection",
            "num_detections": len(objects),
            "processing_time": processing_time
        })
        
        record_metric(Metrics.PERCEPTION_ACCURACY, len([o for o in objects if o["confidence"] > 0.7]) / max(len(objects), 1), {
            "operation": "object_detection_quality",
            "high_confidence_detections": len([o for o in objects if o["confidence"] > 0.7])
        })
        
        return {
            "objects": objects,
            "processing_time_ms": processing_time * 1000,
            "memory_usage_mb": self.memory_limiter.get_current_memory_usage()
        }
    
    def _estimate_distance(self, bbox) -> float:
        """Estimate distance to object based on bounding box size."""
        # Simplified distance estimation - in real system would use stereo vision or other methods
        bbox_width = bbox[2] - bbox[0]
        return max(1.0, 100.0 / (bbox_width + 0.1))

def get_memory_optimizer():
    """Get the global memory optimizer instance."""
    return MemoryLimiter()

def optimize_memory_now():
    """Perform immediate memory optimization."""
    optimizer = get_memory_optimizer()
    optimizer.optimize_memory()
    return optimizer.get_current_memory_usage()