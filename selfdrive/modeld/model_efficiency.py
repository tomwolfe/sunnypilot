"""
Model Efficiency Enhancements for sunnypilot
Implements pruning, quantization, and other optimization techniques to reduce computational footprint
"""
import numpy as np
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from collections import OrderedDict
import pickle
import os

from openpilot.common.swaglog import cloudlog
from openpilot.common.performance_monitor import PerfTrack

# Import torch modules only when needed to prevent import errors on device
def get_torch_modules():
  """Lazy import of torch modules to prevent import errors on hardware without PyTorch"""
  try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    return torch, nn, F
  except ImportError:
    cloudlog.warning("PyTorch not available - model efficiency optimizations limited")
    return None, None, None


@dataclass
class ModelEfficiencyConfig:
  """Configuration for model efficiency optimizations"""
  # Pruning settings
  pruning_method: str = "magnitude"  # 'magnitude', 'random', 'l1', 'l2'
  pruning_ratio: float = 0.2  # Prune 20% of weights by default
  pruning_schedule: str = "iterative"  # 'iterative', 'one_shot'
  
  # Quantization settings
  quantization_enabled: bool = True
  quantization_bits: int = 8  # 8-bit quantization
  quantization_method: str = "dynamic"  # 'dynamic', 'static', 'qat' (quantization aware training)
  
  # Performance targets
  target_latency_ms: float = 50.0
  target_memory_mb: float = 100.0
  target_cpu_percent: float = 5.0
  
  # Efficiency metrics
  min_accuracy_preserved: float = 0.95  # Maintain at least 95% of original accuracy


class ModelPruner:
  """Handles model pruning to reduce computational load"""

  def __init__(self, config: ModelEfficiencyConfig):
    self.config = config
    self.torch, self.nn, _ = get_torch_modules()

  def prune_model(self, model: nn.Module) -> nn.Module:
    """Prune the model based on configuration"""
    if self.torch is None or not hasattr(self.torch.nn.utils, 'prune'):
      cloudlog.warning("Torch pruning utilities not available, skipping pruning")
      return model

    if self.config.pruning_ratio <= 0:
      return model

    # Prune model based on selected method
    if self.config.pruning_method == "magnitude":
      return self._prune_by_magnitude(model)
    elif self.config.pruning_method == "l1":
      return self._prune_by_l1_norm(model)
    elif self.config.pruning_method == "random":
      return self._prune_randomly(model)
    else:
      cloudlog.warning(f"Unknown pruning method: {self.config.pruning_method}, skipping pruning")
      return model

  def _prune_by_magnitude(self, model: nn.Module) -> nn.Module:
    """Prune weights with smallest magnitude"""
    if self.torch is None:
      return model

    import torch.nn.utils.prune as prune

    # Prune convolutional and linear layers
    for name, module in model.named_modules():
      if self.nn and isinstance(module, (self.nn.Conv2d, self.nn.Linear)):
        try:
          prune.l1_unstructured(module, name='weight', amount=self.config.pruning_ratio)
        except Exception as e:
          cloudlog.error(f"Failed to prune layer {name}: {e}")

    return model

  def _prune_by_l1_norm(self, model: nn.Module) -> nn.Module:
    """Prune based on L1 norm of weights"""
    if self.torch is None:
      return model

    import torch.nn.utils.prune as prune

    for name, module in model.named_modules():
      if self.nn and isinstance(module, (self.nn.Conv2d, self.nn.Linear)):
        try:
          prune.ln_structured(module, name='weight', amount=self.config.pruning_ratio, n=1, dim=0)
        except Exception as e:
          cloudlog.error(f"Failed to prune layer {name} with L1 norm: {e}")

    return model

  def _prune_randomly(self, model: nn.Module) -> nn.Module:
    """Randomly prune weights"""
    if self.torch is None:
      return model

    import torch.nn.utils.prune as prune

    for name, module in model.named_modules():
      if self.nn and isinstance(module, (self.nn.Conv2d, self.nn.Linear)):
        try:
          prune.random_unstructured(module, name='weight', amount=self.config.pruning_ratio)
        except Exception as e:
          cloudlog.error(f"Failed to randomly prune layer {name}: {e}")

    return model


class ModelQuantizer:
  """Handles model quantization to reduce memory and computation"""

  def __init__(self, config: ModelEfficiencyConfig):
    self.config = config
    self.torch, self.nn, _ = get_torch_modules()

  def quantize_model(self, model: nn.Module, calibration_data: Optional[List] = None) -> nn.Module:
    """Quantize the model based on configuration"""
    if not self.config.quantization_enabled or self.torch is None:
      return model

    if self.config.quantization_method == "dynamic":
      return self._dynamic_quantize(model)
    elif self.config.quantization_method == "static" and calibration_data is not None:
      return self._static_quantize(model, calibration_data)
    elif self.config.quantization_method == "qat":  # Quantization Aware Training
      return self._qat_quantize(model)
    else:
      cloudlog.warning(f"Unknown quantization method: {self.config.quantization_method}, using dynamic quantization")
      return self._dynamic_quantize(model)

  def _dynamic_quantize(self, model: nn.Module) -> nn.Module:
    """Apply dynamic quantization to the model"""
    if self.torch is None:
      return model

    # Quantize specific layers that benefit from quantization
    if self.nn:
        quantizable_layers = (self.nn.Linear, self.nn.Conv2d, self.nn.LSTM, self.nn.GRU)
    else:
        quantizable_layers = ()  # Empty if no torch.nn

    try:
      # Check if quantization is available
      if not hasattr(self.torch.quantization, 'quantize_dynamic'):
        return model

      quantized_model = self.torch.quantization.quantize_dynamic(
        model,
        {self.nn.Linear, self.nn.Conv2d, self.nn.LSTM} if self.nn else {},  # Specify layers to quantize
        dtype=self.torch.qint8
      )
      return quantized_model
    except Exception as e:
      cloudlog.error(f"Dynamic quantization failed: {e}")
      return model  # Return original model if quantization fails

  def _static_quantize(self, model: nn.Module, calibration_data: List) -> nn.Module:
    """Apply static quantization with calibration data"""
    if self.torch is None:
      return model

    # Set model to evaluation mode
    model.eval()

    # Fuse modules for better quantization (e.g., conv+relu)
    model = self._fuse_modules(model)

    try:
      # Specify quantization configuration
      if not hasattr(self.torch.quantization, 'get_default_qconfig'):
        return model
      model.qconfig = self.torch.quantization.get_default_qconfig('qnnpack')

      # Prepare model for static quantization
      self.torch.quantization.prepare(model, inplace=True)

      # Run calibration data through model
      for data in calibration_data:
        try:
          if isinstance(data, self.torch.Tensor):
            model(data)
          elif isinstance(data, (list, tuple)):
            model(*data)
        except:
          continue  # Skip problematic calibration data

      # Convert to quantized model
      quantized_model = self.torch.quantization.convert(model, inplace=False)

      return quantized_model
    except Exception as e:
      cloudlog.error(f"Static quantization failed: {e}")
      return model

  def _qat_quantize(self, model: nn.Module) -> nn.Module:
    """Apply quantization aware training quantization"""
    if self.torch is None:
      return model

    # Set model to training mode for QAT
    model.train()

    # Fuse modules
    model = self._fuse_modules(model)

    try:
      # Set quantization configuration for QAT
      if not hasattr(self.torch.quantization, 'get_default_qat_qconfig'):
        return model
      model.qconfig = self.torch.quantization.get_default_qat_qconfig('qnnpack')

      # Prepare model for QAT
      qat_model = self.torch.quantization.prepare_qat(model, inplace=False)

      return qat_model
    except Exception as e:
      cloudlog.error(f"QAT quantization failed: {e}")
      return model

  def _fuse_modules(self, model: nn.Module) -> nn.Module:
    """Fuse modules to optimize for quantization"""
    if self.torch is None:
      return model

    # Define patterns to fuse (conv+bn+relu, etc.)
    for module in model.modules():
      if isinstance(module, (self.nn.Sequential if self.nn else type(None))):
        # Try to fuse common patterns
        try:
          if hasattr(self.torch.quantization, 'fuse_modules'):
            self.torch.quantization.fuse_modules(module, [['conv', 'bn', 'relu']], inplace=True)
        except:
          pass  # Skip if fusion isn't applicable
      elif hasattr(self.torch.quantization, 'fuse_modules'):
        # Try other common patterns
        try:
          self.torch.quantization.fuse_modules(module, [['conv', 'relu']], inplace=True)
        except:
          pass

    return model


class ModelEfficiencyOptimizer:
  """Main class for model efficiency optimization"""
  
  def __init__(self, config: ModelEfficiencyConfig = None):
    self.config = config or ModelEfficiencyConfig()
    self.pruner = ModelPruner(self.config)
    self.quantizer = ModelQuantizer(self.config)
    self.efficiency_stats = {}
  
  def optimize_model(self, model: nn.Module, calibration_data: Optional[List] = None) -> Tuple[nn.Module, Dict[str, Any]]:
    """Apply all optimizations to the model"""
    original_size = self._get_model_size(model)

    with PerfTrack("model_optimization") as tracker:
      # Only perform expensive optimizations if they're enabled in config
      if self.config.pruning_ratio > 0:
        pruned_model = self.pruner.prune_model(model)
        pruned_size = self._get_model_size(pruned_model)
      else:
        pruned_model = model
        pruned_size = original_size

      # Only quantize if enabled in config
      if self.config.quantization_enabled:
        optimized_model = self.quantizer.quantize_model(pruned_model, calibration_data)
        optimized_size = self._get_model_size(optimized_model)
      else:
        optimized_model = pruned_model
        optimized_size = pruned_size

    optimization_time = tracker.get_time_ms()

    # Calculate efficiency improvements
    size_reduction = (original_size - optimized_size) / original_size * 100 if original_size > 0 else 0

    efficiency_report = {
      'original_size_mb': original_size / (1024*1024),
      'optimized_size_mb': optimized_size / (1024*1024),
      'size_reduction_percent': size_reduction,
      'optimization_time_ms': optimization_time,
      'pruning_ratio': self.config.pruning_ratio,
      'quantization_bits': self.config.quantization_bits,
      'quantization_method': self.config.quantization_method,
      'pruning_enabled': self.config.pruning_ratio > 0,
      'quantization_enabled': self.config.quantization_enabled
    }

    self.efficiency_stats = efficiency_report

    cloudlog.info(f"Model optimization completed: {size_reduction:.1f}% size reduction, "
                  f"{optimization_time:.1f}ms processing time")

    return optimized_model, efficiency_report
  
  def _get_model_size(self, model: nn.Module) -> int:
    """Get the size of the model in bytes"""
    param_size = 0
    buffer_size = 0
    
    for param in model.parameters():
      param_size += param.nelement() * param.element_size()
    
    for buffer in model.buffers():
      buffer_size += buffer.nelement() * buffer.element_size()
    
    return param_size + buffer_size
  
  def get_efficiency_report(self) -> Dict[str, Any]:
    """Get the latest efficiency optimization report"""
    return self.efficiency_stats
  
  def estimate_inference_performance(self, model: nn.Module, input_tensor) -> Dict[str, float]:
    """Estimate inference performance of the model"""
    torch_module, _, _ = get_torch_modules()
    if torch_module is None:
      # If torch is not available, return a default performance estimate
      return {
        'avg_inference_time_ms': 0.0,  # Placeholder value
        'estimated_fps': 0.0,
        'input_size': (0,)  # Placeholder value
      }

    model.eval()

    # Warm up
    for _ in range(3):
      with torch_module.no_grad():
        _ = model(input_tensor)

    # Measure performance
    import time
    start_time = time.perf_counter()

    with torch_module.no_grad():
      for _ in range(10):  # Average over 10 runs
        _ = model(input_tensor)

    end_time = time.perf_counter()

    avg_inference_time = (end_time - start_time) / 10 * 1000  # Convert to ms

    return {
      'avg_inference_time_ms': avg_inference_time,
      'estimated_fps': 1000.0 / avg_inference_time if avg_inference_time > 0 else 0,
      'input_size': tuple(input_tensor.shape)
    }


class EfficientModelWrapper:
  """Wrapper for efficiently running optimized models"""

  def __init__(self, model: nn.Module, config: ModelEfficiencyConfig = None):
    self.config = config or ModelEfficiencyConfig()
    self.model = model
    self.optimizer = ModelEfficiencyOptimizer(self.config)
    self.torch, _, _ = get_torch_modules()

    # Track performance for adaptation
    self.inference_times = []
    self.max_inference_history = 50

    # Pre-warm the model to reduce initial overhead
    self._warmup_model()

  def _warmup_model(self):
    """Warm up the model to reduce initial inference overhead"""
    if self.torch is not None:
      try:
        # Create a small dummy input for warmup
        # The specific shape may need to be adjusted based on the actual model
        dummy_input = self.torch.randn(1, 3, 224, 224) if hasattr(self.torch, 'randn') else None
        if dummy_input is not None:
          with self.torch.no_grad():
            _ = self.model(dummy_input)
      except Exception:
        # If warmup fails, continue without it
        pass

  def forward(self, *args, **kwargs):
    """Forward pass with performance tracking and optimized execution"""
    import time

    start_time = time.perf_counter()

    if self.torch is not None:
      with self.torch.no_grad():
        result = self.model(*args, **kwargs)
    else:
      # If torch is not available, run without no_grad context
      result = self.model(*args, **kwargs)

    end_time = time.perf_counter()
    inference_time = (end_time - start_time) * 1000  # Convert to ms

    # Track inference time - use more efficient approach
    self.inference_times.append(inference_time)
    if len(self.inference_times) > self.max_inference_history:
      self.inference_times.pop(0)

    return result

  def get_average_inference_time(self) -> float:
    """Get average inference time"""
    if not self.inference_times:
      return 0.0
    # Use a more efficient average calculation for frequently called methods
    return sum(self.inference_times) / len(self.inference_times)

  def should_adapt_model(self) -> bool:
    """Check if model adaptation is needed based on performance"""
    if not self.inference_times:
      return False
    avg_time = sum(self.inference_times) / len(self.inference_times)
    return avg_time > self.config.target_latency_ms * 0.8  # Adapt if within 20% of target

  def adapt_model_complexity(self, input_data) -> nn.Module:
    """Adapt model complexity based on performance requirements"""
    if not self.should_adapt_model():
      return self.model

    # If model is too slow, create a more efficient version
    current_avg_time = self.get_average_inference_time()

    # Adjust pruning ratio based on performance need
    required_speedup = current_avg_time / self.config.target_latency_ms
    new_pruning_ratio = min(self.config.pruning_ratio * required_speedup, 0.5)  # Cap at 50% pruning

    # Create new config with adjusted parameters
    adapt_config = ModelEfficiencyConfig(
      pruning_method=self.config.pruning_method,
      pruning_ratio=new_pruning_ratio,
      quantization_enabled=self.config.quantization_enabled,
      quantization_bits=self.config.quantization_bits,
      quantization_method=self.config.quantization_method,
      target_latency_ms=self.config.target_latency_ms,
      target_memory_mb=self.config.target_memory_mb,
      target_cpu_percent=self.config.target_cpu_percent,
      min_accuracy_preserved=self.config.min_accuracy_preserved
    )

    # Create a new optimized model with higher pruning
    new_optimizer = ModelEfficiencyOptimizer(adapt_config)
    adapted_model, report = new_optimizer.optimize_model(self.model)

    cloudlog.info(f"Model adapted: {report.get('size_reduction_percent', 0):.1f}% size reduction, "
                  f"target was {current_avg_time:.1f}ms, new target {self.config.target_latency_ms}ms")

    return adapted_model

  def run_efficient_inference(self, input_tensor, use_cache: bool = True):
    """Run inference with additional optimizations"""
    import time

    start_time = time.perf_counter()

    # If torch is available, use optimized execution
    if self.torch is not None:
      with self.torch.no_grad():
        # Move tensor to appropriate device if needed
        if hasattr(input_tensor, 'device'):
          if input_tensor.device != next(self.model.parameters()).device:
            input_tensor = input_tensor.to(next(self.model.parameters()).device)

        result = self.model(input_tensor)
    else:
      result = self.model(input_tensor)

    inference_time = (time.perf_counter() - start_time) * 1000

    # Track the inference time
    self.inference_times.append(inference_time)
    if len(self.inference_times) > self.max_inference_history:
      self.inference_times.pop(0)

    return result


# Global model efficiency optimizer instance
model_efficiency_optimizer = ModelEfficiencyOptimizer()


def optimize_model_for_hardware(model: nn.Module, calibration_data: Optional[List] = None) -> Tuple[nn.Module, Dict[str, Any]]:
  """Convenience function to optimize model for hardware constraints"""
  config = ModelEfficiencyConfig(
    pruning_method="magnitude",
    pruning_ratio=0.2,
    quantization_enabled=True,
    quantization_bits=8,
    quantization_method="dynamic",
    target_latency_ms=50.0,
    target_memory_mb=100.0,
    target_cpu_percent=5.0,
    min_accuracy_preserved=0.95
  )
  
  optimizer = ModelEfficiencyOptimizer(config)
  return optimizer.optimize_model(model, calibration_data)


def create_efficient_model_wrapper(model: nn.Module) -> EfficientModelWrapper:
  """Create an efficient model wrapper for automatic adaptation"""
  return EfficientModelWrapper(model)


__all__ = [
  "ModelEfficiencyConfig", "ModelPruner", "ModelQuantizer",
  "ModelEfficiencyOptimizer", "EfficientModelWrapper", "model_efficiency_optimizer",
  "optimize_model_for_hardware", "create_efficient_model_wrapper", "get_torch_modules"
]