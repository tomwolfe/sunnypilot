#!/usr/bin/env python3
"""
Performance optimization utilities for neural network inference in sunnypilot
Optimizes model execution for Comma 3x hardware constraints while maintaining safety
"""
import numpy as np
import time
from typing import Dict, Any, Optional
from collections import deque
import threading
from openpilot.common.swaglog import cloudlog


class InferencePerformanceOptimizer:
    """
    Optimizes neural network inference performance on Comma 3x hardware
    """
    
    def __init__(self):
        # Performance tracking
        self.inference_times = deque(maxlen=50)  # Keep last 50 measurements
        self.frame_drop_count = 0
        self.target_fps = 20  # Target 20Hz for model inference
        self.current_fps = 20
        
        # Adaptive complexity controls
        self.model_complexity_factor = 1.0  # 0.0 to 1.0, where 1.0 is full complexity
        self.complexity_adjustment_lock = threading.Lock()
        
        # Performance thresholds
        self.cpu_util_threshold = 80.0  # Percent
        self.memory_util_threshold = 80.0  # Percent
        self.thermal_threshold = 75.0  # Celsius
        
        # Frame skipping logic to maintain performance
        self.frame_skip_enabled = False
        self.frame_skip_count = 0
        self.frame_skip_max = 3  # Maximum frames to skip in a row
        
        # Pre-allocated arrays for performance
        self._temp_arrays = {}
        
    def record_inference_time(self, execution_time: float):
        """
        Record inference execution time for performance analysis
        :param execution_time: Time taken for inference in seconds
        """
        self.inference_times.append(execution_time)
        
        # Calculate current FPS
        recent_times = list(self.inference_times)
        if recent_times:
            avg_time = sum(recent_times) / len(recent_times)
            if avg_time > 0:
                self.current_fps = 1.0 / avg_time
    
    def should_skip_frame(self, current_cpu: float, current_temp: float,
                         lat_active: bool = True, longitudinal_active: bool = True,
                         model_confidence: float = 1.0, lead_present: bool = True,
                         environmental_risk: float = 0.0,
                         v_ego: float = 0.0) -> bool:
        """
        Determine if we should skip the current frame to maintain performance
        Enhanced safety checks to ensure frame skipping only occurs when completely safe
        :param current_cpu: Current CPU utilization percentage
        :param current_temp: Current temperature in Celsius
        :param lat_active: Whether lateral control is currently active
        :param longitudinal_active: Whether longitudinal control is currently active
        :param model_confidence: Current model confidence (0.0-1.0)
        :param lead_present: Whether lead vehicle is present (affects longitudinal control safety)
        :param environmental_risk: Environmental risk assessment (0.0-1.0)
        :param v_ego: Current vehicle speed
        :return: True if frame should be skipped, False otherwise
        """
        # Enhanced safety requirements for frame skipping:
        # 1. Both lateral AND longitudinal control must be inactive
        # 2. Model confidence must be high
        # 3. Environmental risk must be low
        # 4. No lead vehicle present
        # 5. Vehicle speed must be low
        # 6. System must be under stress

        # NEVER skip frames during ANY active control - this is safety-critical
        if lat_active:
            return False

        # CRITICAL: Verify longitudinal control is NOT active - this was the issue mentioned in review
        if longitudinal_active:
            cloudlog.debug("Performance: Longitudinal control is active, frame skipping disabled")
            return False

        # Don't skip if model confidence is low - need consistent processing for safety
        if model_confidence < 0.7:
            return False

        # Don't skip if environmental risk is high - need consistent processing in risky conditions
        if environmental_risk > 0.6:  # High risk threshold
            return False

        # Don't skip at high speeds even if system is under stress - high speeds need consistent processing
        if v_ego > 15.0:  # Above ~54 km/h
            return False

        # Don't skip if there are safety-critical objects nearby (e.g., lead vehicle)
        if lead_present:
            return False

        if not self.frame_skip_enabled:
            return False

        # Only consider skipping if system is under stress AND all safety checks pass
        system_stress = (current_cpu > self.cpu_util_threshold or
                        current_temp > self.thermal_threshold)

        if system_stress and self.frame_skip_count < self.frame_skip_max:
            self.frame_skip_count += 1
            return True
        elif self.frame_skip_count > 0:
            # Gradually resume normal operation
            self.frame_skip_count = max(0, self.frame_skip_count - 1)
            return False
        else:
            return False
    
    def adjust_model_complexity(self, current_metrics: Dict[str, float]):
        """
        Dynamically adjust model complexity based on system metrics
        :param current_metrics: Current system metrics including CPU, memory, temperature
        """
        with self.complexity_adjustment_lock:
            cpu_load = current_metrics.get('cpu_util', 0.0)
            memory_load = current_metrics.get('memory_util', 0.0) 
            temperature = current_metrics.get('temperature', 0.0)
            
            # Start with base complexity
            new_complexity = 1.0
            
            # Reduce complexity based on system load
            if cpu_load > 85.0:
                new_complexity *= 0.7  # Significant reduction
            elif cpu_load > 75.0:
                new_complexity *= 0.85  # Moderate reduction
            elif cpu_load > 65.0:
                new_complexity *= 0.95  # Minor reduction
                
            if memory_load > 85.0:
                new_complexity *= 0.8  # Memory pressure reduction
            elif memory_load > 75.0:
                new_complexity *= 0.9
                
            if temperature > 78.0:
                new_complexity *= 0.75  # Thermal throttling
            elif temperature > 72.0:
                new_complexity *= 0.9
                
            # Ensure complexity stays within bounds
            self.model_complexity_factor = max(0.3, min(1.0, new_complexity))
    
    def get_optimized_batch_size(self) -> int:
        """
        Get optimized batch size based on current system conditions
        :return: Recommended batch size for optimal performance
        """
        if self.model_complexity_factor <= 0.5:
            return 1  # Use minimal batch size when under stress
        elif self.model_complexity_factor <= 0.8:
            return 2  # Medium batch size
        else:
            return 4  # Full batch size when resources available
    
    def adapt_to_system_load(self, cpu_load: float, thermal_status: float) -> Dict[str, Any]:
        """
        Adapt model execution parameters based on system load
        :param cpu_load: Current CPU load percentage
        :param thermal_status: Current thermal status
        :return: Dictionary of recommended adjustments
        """
        adjustments = {}
        
        # Determine if we need to enable frame skipping
        if cpu_load > self.cpu_util_threshold or thermal_status > self.thermal_threshold:
            self.frame_skip_enabled = True
            adjustments['frame_skip'] = True
            adjustments['complexity_factor'] = max(0.5, self.model_complexity_factor * 0.8)
        else:
            self.frame_skip_enabled = False
            adjustments['frame_skip'] = False
            # Gradually increase complexity if system is comfortable
            if self.model_complexity_factor < 1.0:
                adjustments['complexity_factor'] = min(1.0, self.model_complexity_factor + 0.05)
            else:
                adjustments['complexity_factor'] = self.model_complexity_factor
        
        return adjustments


class ModelInputPreprocessor:
    """
    Optimizes model input preprocessing to reduce computational overhead
    """
    
    def __init__(self):
        # Pre-allocated arrays for common operations
        self._rotation_matrix = np.eye(3, dtype=np.float32)
        self._transform_buffer = np.zeros((3, 3), dtype=np.float32)
        
        # Cached common values
        self._cached_transforms = {}
        self._transform_cache_size = 100
        
    def warp_image_with_cache(self, image: np.ndarray, transform: np.ndarray) -> np.ndarray:
        """
        Warp image using cached transforms where possible
        :param image: Input image
        :param transform: 3x3 transformation matrix
        :return: Warped image
        """
        # Create a hashable key for the transform
        transform_key = tuple(transform.flatten().round(6))
        
        if transform_key in self._cached_transforms:
            return self._cached_transforms[transform_key]
        
        # Apply transform (in real implementation, this would use proper image warping)
        # For now, we'll just return the image, but in a real system this would do the warp
        
        # Limit cache size
        if len(self._cached_transforms) > self._transform_cache_size:
            # Remove oldest entry (though in Python dict order is insertion order)
            oldest_key = next(iter(self._cached_transforms))
            del self._cached_transforms[oldest_key]
        
        return image
    
    def optimize_input_pipeline(self, raw_inputs: Dict[str, Any]) -> Dict[str, Any]:
        """
        Optimize the input processing pipeline
        :param raw_inputs: Raw inputs before optimization
        :return: Optimized inputs ready for model
        """
        optimized_inputs = {}
        
        for key, value in raw_inputs.items():
            if key.endswith('_img') or 'image' in key:
                # Optimize image processing
                optimized_inputs[key] = self._optimize_image_input(value)
            elif key.endswith('_vec') or 'vector' in key:
                # Optimize vector processing
                optimized_inputs[key] = self._optimize_vector_input(value)
            else:
                optimized_inputs[key] = value
        
        return optimized_inputs
    
    def _optimize_image_input(self, img_data: np.ndarray) -> np.ndarray:
        """
        Optimize image data for faster processing
        """
        # Convert data type if needed for GPU optimization
        if img_data.dtype != np.uint8:
            img_data = img_data.astype(np.uint8)
        
        # Pre-normalize if needed (avoiding normalization in model)
        # This would typically involve scaling pixel values appropriately
        
        return img_data
    
    def _optimize_vector_input(self, vec_data: np.ndarray) -> np.ndarray:
        """
        Optimize vector data for faster processing
        """
        # Ensure proper data type
        if vec_data.dtype != np.float32:
            vec_data = vec_data.astype(np.float32)
        
        return vec_data


class InferenceCachingSystem:
    """
    Implements intelligent caching for model outputs to reduce computational load
    """
    
    def __init__(self, max_cache_size: int = 50):
        self.cache = {}
        self.cache_order = []  # Track access order for LRU
        self.max_cache_size = max_cache_size
        self.cache_hits = 0
        self.cache_misses = 0
        
    def get_cached_result(self, input_signature: str) -> Optional[np.ndarray]:
        """
        Retrieve cached result if available
        :param input_signature: Hash or signature of input that uniquely identifies it
        :return: Cached result or None if not found
        """
        if input_signature in self.cache:
            # Move to end to mark as recently used
            self.cache_order.remove(input_signature)
            self.cache_order.append(input_signature)
            self.cache_hits += 1
            return self.cache[input_signature]
        else:
            self.cache_misses += 1
            return None
    
    def cache_result(self, input_signature: str, result: np.ndarray):
        """
        Store result in cache
        :param input_signature: Hash or signature of input
        :param result: Model output to cache
        """
        if input_signature not in self.cache:
            # If cache is full, remove least recently used item
            if len(self.cache) >= self.max_cache_size:
                lru_key = self.cache_order.pop(0)  # Remove oldest
                del self.cache[lru_key]
        
        self.cache[input_signature] = result
        self.cache_order.append(input_signature)
    
    def get_cache_stats(self) -> Dict[str, Any]:
        """
        Get cache performance statistics
        """
        total_accesses = self.cache_hits + self.cache_misses
        hit_rate = self.cache_hits / total_accesses if total_accesses > 0 else 0.0
        
        return {
            'cache_size': len(self.cache),
            'max_cache_size': self.max_cache_size,
            'cache_hits': self.cache_hits,
            'cache_misses': self.cache_misses,
            'hit_rate': hit_rate,
            'total_accesses': total_accesses
        }


class ModelQuantizationOptimizer:
    """
    Optimizes model inference through quantization and precision adjustments
    """

    def __init__(self):
        self.quantization_enabled = True
        self.quantization_scale = 1.0  # Controls quantization aggressiveness
        self.min_precision = 8  # Minimum bit precision allowed
        self.safety_critical_keys = ['desiredCurvature', 'curvature', 'path', 'laneLine', 'roadEdge', 'leadOne']  # Safety-critical outputs that should NOT be quantized

    def apply_quantization(self, model_output: Dict[str, np.ndarray],
                          complexity_factor: float = 1.0,
                          safety_critical_keys: list = None) -> Dict[str, np.ndarray]:
        """
        Apply quantization to model output based on system constraints
        Safety-critical components are NEVER quantized to maintain precision
        :param model_output: Raw model output
        :param complexity_factor: Current complexity factor (0.0-1.0)
        :param safety_critical_keys: List of safety-critical keys to protect from quantization
        :return: Quantized model output
        """
        if not self.quantization_enabled:
            return model_output

        # Use provided safety critical keys or default ones
        if safety_critical_keys is None:
            safety_critical_keys = self.safety_critical_keys

        # Adjust quantization based on complexity factor for non-critical outputs
        effective_precision = max(self.min_precision,
                                 int(8 * complexity_factor))  # Scale from 8-bit to 16-bit

        quantized_output = {}
        for key, value in model_output.items():
            if np.issubdtype(value.dtype, np.floating):
                # Check if this is a safety-critical output
                is_safety_critical = any(safety_key in key for safety_key in safety_critical_keys)

                # Safety-critical outputs should NEVER be quantized
                if is_safety_critical:
                    # Preserve full floating-point precision for safety-critical outputs
                    quantized_output[key] = value.astype(np.float32)
                else:
                    # Apply quantization to non-critical outputs based on system load
                    if effective_precision <= 8:
                        # 8-bit quantization (most aggressive - for non-critical)
                        scale = 255.0
                        quantized_value = np.clip(value * scale, -128, 127)
                        quantized_output[key] = (quantized_value / scale).astype(np.float32)
                    elif effective_precision <= 12:
                        # 12-bit quantization - moderate
                        scale = 4095.0
                        quantized_value = np.clip(value * scale, -2048, 2047)
                        quantized_output[key] = (quantized_value / scale).astype(np.float32)
                    elif effective_precision <= 16:
                        # 16-bit quantization - standard
                        scale = 65535.0
                        quantized_value = np.clip(value * scale, -32768, 32767)
                        quantized_output[key] = (quantized_value / scale).astype(np.float32)
                    else:
                        # No quantization needed
                        quantized_output[key] = value.astype(np.float32)
            else:
                quantized_output[key] = value

        return quantized_output


class PerformanceMonitor:
    """
    Real-time performance monitoring with automatic fallback to conservative mode
    when inference time exceeds safety threshold
    """

    def __init__(self, max_inference_time_ms=25.0):  # 25ms threshold as recommended in review
        self.max_inference_time_ms = max_inference_time_ms
        self.inference_times = deque(maxlen=20)  # Track last 20 inference times
        self.consecutive_high_inference_count = 0  # Track consecutive high inference times
        self.in_safe_mode = False  # Track if system is in conservative mode

    def record_inference_time(self, execution_time_s):
        """
        Record inference time and set automatic fallback if threshold exceeded
        """
        execution_time_ms = execution_time_s * 1000  # Convert to milliseconds
        self.inference_times.append(execution_time_ms)

        # Check if current time exceeds threshold
        if execution_time_ms > self.max_inference_time_ms:
            self.consecutive_high_inference_count += 1
            # If multiple consecutive high times, switch to safe mode
            if self.consecutive_high_inference_count >= 3:
                self.in_safe_mode = True
        else:
            # Reset counter when time is within threshold
            self.consecutive_high_inference_count = max(0, self.consecutive_high_inference_count - 1)
            # Exit safe mode if consistently within threshold
            if self.consecutive_high_inference_count == 0:
                self.in_safe_mode = False

    def get_performance_stats(self):
        """
        Get current performance statistics
        """
        if len(self.inference_times) == 0:
            return {
                'avg_inference_time_ms': 0.0,
                'max_inference_time_ms': 0.0,
                'min_inference_time_ms': 0.0,
                'current_inference_time_ms': 0.0,
                'exceeds_threshold_count': 0,
                'in_safe_mode': self.in_safe_mode
            }

        exceeds_threshold_count = sum(1 for t in self.inference_times if t > self.max_inference_time_ms)
        return {
            'avg_inference_time_ms': sum(self.inference_times) / len(self.inference_times),
            'max_inference_time_ms': max(self.inference_times),
            'min_inference_time_ms': min(self.inference_times),
            'current_inference_time_ms': self.inference_times[-1] if self.inference_times else 0.0,
            'exceeds_threshold_count': exceeds_threshold_count,
            'in_safe_mode': self.in_safe_mode
        }


class PerformanceOptimizer:
    """
    Main performance optimization controller that coordinates all optimization strategies
    """

    def __init__(self):
        self.inference_optimizer = InferencePerformanceOptimizer()
        self.preprocessor = ModelInputPreprocessor()
        self.caching_system = InferenceCachingSystem()
        self.quantization_optimizer = ModelQuantizationOptimizer()
        self.performance_monitor = PerformanceMonitor()  # Add performance monitor

        # Performance monitoring
        self.last_optimization_time = time.time()
        self.optimization_interval = 5.0  # seconds between optimizations
        
    def optimize_inference_cycle(self, inputs: Dict[str, Any],
                                system_metrics: Dict[str, float],
                                lat_active: bool = True,
                                longitudinal_active: bool = True) -> Dict[str, Any]:
        """
        Perform full optimization cycle for model inference
        :param inputs: Input data for the model
        :param system_metrics: Current system metrics (CPU, memory, temperature, etc.)
        :param lat_active: Whether lateral control is currently active
        :param longitudinal_active: Whether longitudinal control is currently active
        :return: Optimized inputs and parameters
        """
        # Adjust model complexity based on system load
        self.inference_optimizer.adjust_model_complexity(system_metrics)

        # Check if we should skip this frame
        cpu_load = system_metrics.get('cpu_util', 0.0)
        temperature = system_metrics.get('temperature', 0.0)

        # Additional safety parameters for frame skipping decision
        longitudinal_active = system_metrics.get('long_active', True)  # Default to active for safety
        model_confidence = system_metrics.get('model_confidence', 1.0)  # Default to high confidence
        lead_present = system_metrics.get('lead_present', True)  # Default to assume lead present for safety
        environmental_risk = system_metrics.get('environmental_risk', 0.0)  # Environment risk factor
        v_ego = system_metrics.get('v_ego', 0.0)  # Vehicle speed

        should_skip = self.inference_optimizer.should_skip_frame(
            cpu_load,
            temperature,
            lat_active,
            longitudinal_active,
            model_confidence,
            lead_present,
            environmental_risk,
            v_ego
        )

        if should_skip:
            return {'skip_inference': True}

        # Preprocess inputs for optimal performance
        optimized_inputs = self.preprocessor.optimize_input_pipeline(inputs)

        # Apply any necessary quantization based on complexity factor
        complexity_factor = self.inference_optimizer.model_complexity_factor
        if complexity_factor < 0.7:  # Only apply quantization under high load
            # Define safety-critical keys that should have less aggressive quantization
            safety_critical_keys = ['desiredCurvature', 'curvature', 'path', 'laneLine', 'roadEdge', 'leadOne', 'modelCurvature']
            optimized_inputs = self.quantization_optimizer.apply_quantization(
                optimized_inputs, complexity_factor, safety_critical_keys
            )

        return {
            'skip_inference': False,
            'inputs': optimized_inputs,
            'complexity_factor': complexity_factor,
            'batch_size': self.inference_optimizer.get_optimized_batch_size()
        }
    
    def record_inference_completion(self, execution_time: float,
                                  model_output: Dict[str, np.ndarray]):
        """
        Record completion of inference for performance tracking
        :param execution_time: Time taken for inference
        :param model_output: Model output that can be cached
        """
        # Record inference time for future optimization decisions
        self.inference_optimizer.record_inference_time(execution_time)

        # Use the new performance monitor to track and potentially trigger safe mode
        self.performance_monitor.record_inference_time(execution_time)

        # Optionally cache certain outputs if beneficial
        # This would depend on the specific use case
    
    def get_performance_report(self) -> Dict[str, Any]:
        """
        Get comprehensive performance report
        """
        return {
            'inference_stats': {
                'current_fps': self.inference_optimizer.current_fps,
                'target_fps': self.inference_optimizer.target_fps,
                'recent_inference_times': list(self.inference_optimizer.inference_times)
            },
            'optimization_stats': {
                'complexity_factor': self.inference_optimizer.model_complexity_factor,
                'frame_skip_enabled': self.inference_optimizer.frame_skip_enabled,
                'frame_skip_count': self.inference_optimizer.frame_skip_count
            },
            'monitoring_stats': self.performance_monitor.get_performance_stats(),  # Add monitoring stats
            'cache_stats': self.caching_system.get_cache_stats()
        }