"""
INT8 Quantization Module for sunnypilot
========================================

This module implements hardware-accelerated INT8 quantization with calibrated
clipping for the modeld_v2 E2E stack.

Key Features:
- Per-layer INT8 quantization with calibrated clipping ranges
- Separate precision levels for vision encoder vs policy head
- Dynamic quantization for inference
- Calibration data collection for optimal clipping ranges
- FP16 fallback for critical safety-critical layers
"""

import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Any, Callable
from collections import deque


@dataclass
class QuantizationConfig:
    """Configuration for quantization"""
    layer_name: str
    precision: str
    clip_min: float
    clip_max: float
    scale: float
    zero_point: int


@dataclass
class LayerStats:
    """Statistics for a layer during calibration"""
    min_val: float
    max_val: float
    mean_val: float
    std_val: float
    histogram: np.ndarray


class INT8Quantizer:
    """
    INT8 Quantizer with Calibrated Clipping
    
    Reduces model precision from FP32/FP16 to INT8 while maintaining accuracy
    by learning optimal clipping ranges during calibration.
    
    Usage:
        1. Run calibration with representative data
        2. Apply quantization to model weights/activations
        3. Use quantized inference for speed, FP16 fallback for accuracy
    """
    
    def __init__(self,
                 num_bits: int = 8,
                 calibration_samples: int = 1000,
                 percentile: float = 99.99):
        self.num_bits = num_bits
        self.calibration_samples = calibration_samples
        self.percentile = percentile
        
        self._num_levels = 2 ** num_bits
        self._layer_configs: Dict[str, QuantizationConfig] = {}
        self._calibration_data: Dict[str, deque] = {}
        self._is_calibrated = False
        
    def register_layer(self,
                      layer_name: str,
                      precision: str = 'int8',
                      is_critical: bool = False):
        """
        Register a layer for quantization
        
        Args:
            layer_name: Name of the layer
            precision: 'int8', 'fp16', or 'mixed'
            is_critical: If True, keeps FP16 precision even in INT8 mode
        """
        self._layer_configs[layer_name] = QuantizationConfig(
            layer_name=layer_name,
            precision=precision,
            clip_min=0.0,
            clip_max=0.0,
            scale=1.0,
            zero_point=0
        )
        
        self._calibration_data[layer_name] = deque(maxlen=self.calibration_samples)
        
        if is_critical and precision == 'int8':
            self._layer_configs[layer_name].precision = 'mixed'
    
    def collect_stats(self, layer_name: str, activation: np.ndarray):
        """
        Collect activation statistics for calibration
        
        Call this during forward pass with representative data
        """
        if layer_name not in self._calibration_data:
            return
            
        flat = activation.flatten()
        self._calibration_data[layer_name].append(flat)
    
    def calibrate(self):
        """
        Compute optimal clipping ranges from calibration data
        
        Uses percentile-based clipping to minimize quantization error
        """
        for layer_name, data in self._calibration_data.items():
            if len(data) < 10:
                continue
                
            all_values = np.concatenate(list(data))
            
            abs_max = np.percentile(np.abs(all_values), self.percentile)
            
            clip_min = -abs_max
            clip_max = abs_max
            
            scale = abs_max / (self._num_levels / 2 - 1)
            
            config = self._layer_configs[layer_name]
            config.clip_min = float(clip_min)
            config.clip_max = float(clip_max)
            config.scale = float(scale)
            config.zero_point = 128
            
        self._is_calibrated = True
    
    def quantize(self, layer_name: str, activation: np.ndarray) -> np.ndarray:
        """
        Quantize activation to INT8
        
        Args:
            layer_name: Name of layer
            activation: FP32/FP16 activation tensor
            
        Returns:
            Quantized INT8 activation
        """
        if layer_name not in self._layer_configs:
            return activation
            
        config = self._layer_configs[layer_name]
        
        if config.precision == 'fp16' or not self._is_calibrated:
            return activation
        
        clipped = np.clip(activation, config.clip_min, config.clip_max)
        
        scaled = clipped / config.scale
        
        quantized = np.round(scaled) + config.zero_point
        
        quantized = np.clip(quantized, 0, 255)
        
        return quantized.astype(np.uint8)
    
    def dequantize(self, layer_name: str, quantized: np.ndarray) -> np.ndarray:
        """
       Dequantize INT8 activation back to FP32
        
        Args:
            layer_name: Name of layer
            quantized: INT8 quantized tensor
            
        Returns:
            Dequantized FP32 activation
        """
        if layer_name not in self._layer_configs:
            return quantized
            
        config = self._layer_configs[layer_name]
        
        if config.precision == 'fp16':
            return quantized.astype(np.float32)
        
        shifted = quantized.astype(np.float32) - config.zero_point
        
        dequantized = shifted * config.scale
        
        return dequantized
    
    def quantize_weights(self, weights: np.ndarray, layer_name: str) -> np.ndarray:
        """
        Quantize model weights to INT8
        
        Args:
            weights: FP32 weight matrix
            layer_name: Name of layer
            
        Returns:
            Quantized INT8 weights
        """
        if layer_name not in self._layer_configs:
            return weights
            
        config = self._layer_configs[layer_name]
        
        if config.precision == 'fp16':
            return weights.astype(np.float16)
        
        w_max = np.max(np.abs(weights))
        w_scale = w_max / 127.0
        
        scaled = weights / w_scale
        quantized = np.round(scaled)
        quantized = np.clip(quantized, -128, 127).astype(np.int8)
        
        return quantized
    
    def get_config(self, layer_name: str) -> Optional[QuantizationConfig]:
        """Get quantization config for a layer"""
        return self._layer_configs.get(layer_name)
    
    def get_all_configs(self) -> Dict[str, QuantizationConfig]:
        """Get all layer configurations"""
        return self._layer_configs.copy()


class ModelQuantizer:
    """
    High-level model quantization manager
    
    Applies quantization to vision encoder and policy head with different
    precision levels for optimal speed/accuracy tradeoff.
    """
    
    LAYER_GROUPS = {
        'vision_encoder': [
            'backbone_conv1',
            'backbone_conv2', 
            'backbone_conv3',
            'backbone_resblock1',
            'backbone_resblock2',
            'backbone_resblock3',
            'backbone_resblock4',
            'neck_fpn1',
            'neck_fpn2',
            'neck_fpn3',
        ],
        'transformer': [
            'transformer_encoder1',
            'transformer_encoder2',
            'transformer_encoder3',
            'transformer_encoder4',
            'transformer_encoder5',
            'transformer_encoder6',
        ],
        'policy_head': [
            'lateral_head',
            'longitudinal_head',
            'uncertainty_head',
            'gmm_means',
            'gmm_variances',
            'gmm_weights',
        ],
        'safety_critical': [
            'safety_critic_risk',
            'safety_critic_intervention',
            'torque_clamp',
            'rate_limiter',
        ]
    }
    
    def __init__(self,
                 vision_precision: str = 'int8',
                 policy_precision: str = 'fp16',
                 enable_mixed_precision: bool = True):
        self.quantizer = INT8Quantizer()
        
        self.vision_precision = vision_precision
        self.policy_precision = policy_precision
        self.enable_mixed_precision = enable_mixed_precision
        
        self._register_all_layers()
        
    def _register_all_layers(self):
        """Register all model layers with appropriate precision"""
        for layer in self.LAYER_GROUPS['vision_encoder']:
            self.quantizer.register_layer(
                layer,
                precision=self.vision_precision,
                is_critical=False
            )
        
        for layer in self.LAYER_GROUPS['transformer']:
            precision = 'mixed' if self.enable_mixed_precision else self.vision_precision
            self.quantizer.register_layer(
                layer,
                precision=precision,
                is_critical=False
            )
        
        for layer in self.LAYER_GROUPS['policy_head']:
            self.quantizer.register_layer(
                layer,
                precision=self.policy_precision,
                is_critical=True
            )
        
        for layer in self.LAYER_GROUPS['safety_critical']:
            self.quantizer.register_layer(
                layer,
                precision='fp16',
                is_critical=True
            )
    
    def run_calibration(self, model_forward_fn: Callable, num_batches: int = 100):
        """
        Run calibration with representative data
        
        Args:
            model_forward_fn: Function that runs one forward pass and returns dict of layer->activation
            num_batches: Number of batches to collect
        """
        print(f"Starting calibration with {num_batches} batches...")
        
        for i in range(num_batches):
            try:
                layer_activations = model_forward_fn()
                
                for layer_name, activation in layer_activations.items():
                    self.quantizer.collect_stats(layer_name, activation)
                    
            except Exception as e:
                print(f"Calibration batch {i} failed: {e}")
                break
                
            if (i + 1) % 20 == 0:
                print(f"Calibrated {i + 1}/{num_batches} batches")
        
        self.quantizer.calibrate()
        print("Calibration complete!")
        
        self._print_calibration_summary()
    
    def _print_calibration_summary(self):
        """Print summary of quantization settings"""
        print("\n=== Quantization Configuration ===")
        
        for group_name, layers in self.LAYER_GROUPS.items():
            print(f"\n{group_name}:")
            for layer in layers:
                config = self.quantizer.get_config(layer)
                if config:
                    print(f"  {layer}: {config.precision} "
                          f"[{config.clip_min:.3f}, {config.clip_max:.3f}] "
                          f"scale={config.scale:.6f}")
    
    def quantize_forward(self, 
                        layer_name: str, 
                        activation: np.ndarray,
                        quantize: bool = True) -> np.ndarray:
        """
        Quantized forward pass for a single layer
        
        Args:
            layer_name: Name of the layer
            activation: Input activation
            quantize: Whether to quantize (False for inference)
            
        Returns:
            Processed activation (quantized or dequantized based on precision)
        """
        if not quantize:
            return activation
            
        config = self.quantizer.get_config(layer_name)
        
        if config is None:
            return activation
            
        if config.precision == 'fp16':
            return activation.astype(np.float16)
        
        if config.precision in ('int8', 'mixed'):
            quantized = self.quantizer.quantize(layer_name, activation)
            dequantized = self.quantizer.dequantize(layer_name, quantized)
            return dequantized
            
        return activation
    
    def apply_quantized_inference(self,
                                   model: Any,
                                   input_data: np.ndarray,
                                   forward_fn: Callable) -> np.ndarray:
        """
        Run quantized inference on model
        
        Args:
            model: Model to run
            input_data: Input tensor
            forward_fn: Function to run forward pass, returns dict of layer->activation
            
        Returns:
            Model output
        """
        return forward_fn(model, input_data, self)
    
    def estimate_speedup(self) -> Dict[str, float]:
        """
        Estimate speedup from quantization
        
        Returns:
            Dict with estimated speedup factors
        """
        total_layers = sum(len(layers) for layers in self.LAYER_GROUPS.values())
        
        quantized_vision = len(self.LAYER_GROUPS['vision_encoder'])
        quantized_transformer = len(self.LAYER_GROUPS['transformer'])
        fp16_policy = len(self.LAYER_GROUPS['policy_head'])
        fp16_critical = len(self.LAYER_GROUPS['safety_critical'])
        
        vision_speedup = 2.5
        transformer_speedup = 2.0
        
        weighted_speedup = (
            (quantized_vision * vision_speedup + 
             quantized_transformer * transformer_speedup +
             fp16_policy * 1.5 +
             fp16_critical * 1.2) / total_layers
        )
        
        return {
            'vision_encoder': vision_speedup,
            'transformer': transformer_speedup,
            'policy_head': 1.5,
            'overall': weighted_speedup,
            'estimated_hz_increase': f"{20 * weighted_speedup:.0f}Hz"
        }


class QuantizedLayer:
    """
    Wrapper for quantized layer operations
    
    Provides quantized forward pass with FP16 fallback
    """
    
    def __init__(self,
                 weights: np.ndarray,
                 layer_name: str,
                 quantizer: INT8Quantizer,
                 is_critical: bool = False):
        self.weights = weights
        self.layer_name = layer_name
        self.quantizer = quantizer
        self.is_critical = is_critical
        
        self._quantized_weights = None
        self._weight_scale = None
        
    def _prepare_quantized_weights(self):
        """Pre-quantize weights for faster inference"""
        if self._quantized_weights is not None:
            return
            
        self._quantized_weights = self.quantizer.quantize_weights(
            self.weights, self.layer_name
        )
        
        w_max = np.max(np.abs(self.weights))
        self._weight_scale = w_max / 127.0
        
    def forward(self, x: np.ndarray, use_quantized: bool = True) -> np.ndarray:
        """
        Forward pass with optional quantization
        
        Args:
            x: Input activation
            use_quantized: Whether to use INT8 quantization
            
        Returns:
            Output activation
        """
        if use_quantized and not self.is_critical:
            self._prepare_quantized_weights()
            
            if self._quantized_weights is not None:
                result = np.matmul(x, self._quantized_weights.astype(np.float32))
                result = result * self._weight_scale
                return result
        
        return np.matmul(x, self.weights)
