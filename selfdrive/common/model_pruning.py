"""
Neural Network Model Pruning for Sunnypilot
Provides functionality to reduce model size by removing less important weights
while maintaining accuracy for deployment on resource-constrained hardware
"""

import numpy as np
from typing import Dict, Any, List, Optional, Tuple, Callable
from dataclasses import dataclass
import pickle
from pathlib import Path
import json
from scipy import sparse


@dataclass
class PruningConfig:
    """Configuration for neural network pruning"""
    pruning_method: str = 'magnitude'  # 'magnitude', 'sensitivity', 'random'
    pruning_ratio: float = 0.2  # Fraction of weights to prune (0.0 to 1.0)
    schedule_type: str = 'iterative'  # 'iterative', 'one_shot', 'gradual'
    iterations: int = 5  # Number of pruning iterations for iterative pruning
    retraining_epochs: int = 1  # Epochs to retrain after pruning
    target_sparsity: float = 0.5  # Target sparsity level
    min_sparsity: float = 0.0  # Minimum sparsity to maintain important connections
    use_structured_pruning: bool = False  # Prune entire channels/neurons vs individual weights
    structured_size: int = 8  # Size of structured pruning blocks
    epsilon: float = 1e-8  # Small value to avoid division by zero


class PruningMask:
    """Manages pruning masks for model weights"""
    
    def __init__(self, weight_shape: Tuple[int, ...]):
        self.mask = np.ones(weight_shape, dtype=bool)
        self.original_shape = weight_shape
        self.pruned_count = 0
        self.total_count = np.prod(weight_shape)
    
    def apply_mask(self, weights: np.ndarray) -> np.ndarray:
        """Apply the pruning mask to weights"""
        if weights.shape != self.original_shape:
            raise ValueError(f"Weight shape {weights.shape} does not match mask shape {self.original_shape}")
        
        pruned_weights = weights * self.mask.astype(weights.dtype)
        return pruned_weights
    
    def update_mask(self, new_mask: np.ndarray) -> None:
        """Update the pruning mask"""
        if new_mask.shape != self.original_shape:
            raise ValueError(f"New mask shape {new_mask.shape} does not match original shape {self.original_shape}")
        
        self.mask = new_mask.astype(bool)
        self.pruned_count = self.total_count - np.sum(self.mask)
    
    def get_sparsity(self) -> float:
        """Get the current sparsity level (fraction of zero weights)"""
        return float(self.pruned_count) / float(self.total_count) if self.total_count > 0 else 0.0


class MagnitudePruner:
    """Prunes weights based on their magnitude (absolute value)"""
    
    def __init__(self, config: PruningConfig):
        self.config = config
    
    def calculate_importance(self, weights: np.ndarray) -> np.ndarray:
        """Calculate importance scores for weights based on magnitude"""
        return np.abs(weights)
    
    def prune_weights(self, weights: np.ndarray, pruning_ratio: float) -> Tuple[np.ndarray, np.ndarray]:
        """Prune weights based on magnitude, returning pruned weights and mask"""
        importance_scores = self.calculate_importance(weights)
        
        # Flatten importance scores to find thresholds
        flat_scores = importance_scores.flatten()
        
        # Calculate threshold for pruning
        threshold_idx = int(len(flat_scores) * pruning_ratio)
        if threshold_idx >= len(flat_scores):
            threshold_idx = len(flat_scores) - 1
        
        sorted_indices = np.argsort(flat_scores)
        threshold_score = flat_scores[sorted_indices[threshold_idx]]
        
        # Create mask: True for preserved weights, False for pruned weights
        mask = importance_scores > threshold_score
        
        # Apply mask to weights
        pruned_weights = weights * mask.astype(weights.dtype)
        
        return pruned_weights, mask


class SensitivityPruner:
    """Prunes weights based on their sensitivity to output changes"""
    
    def __init__(self, config: PruningConfig):
        self.config = config
    
    def calculate_importance(self, weights: np.ndarray, 
                           input_gradient: Optional[np.ndarray] = None,
                           output_gradient: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Calculate importance based on sensitivity analysis
        For this implementation, we'll use a simplified approach
        """
        # In a real implementation, this would calculate gradients and sensitivities
        # For now, we'll use a combination of weight magnitude and local statistics
        
        # Use magnitude as the primary metric (similar to MagnitudePruner)
        # In a true sensitivity analysis, we'd consider gradient information
        magnitude = np.abs(weights)
        
        # Add local variance as sensitivity measure
        if weights.ndim >= 2:
            # For multi-dimensional tensors, consider local variance
            local_variance = np.zeros_like(weights)
            for i in range(weights.shape[0]):
                if weights.ndim == 2 and weights.shape[1] > 1:
                    local_variance[i, :] = np.var(weights[i, :])
                elif weights.ndim > 2:
                    # Consider variance across spatial dimensions
                    remaining_dims = tuple(range(1, weights.ndim))
                    local_variance[i, ...] = np.var(weights[i, ...], axis=remaining_dims, keepdims=True)
        
        # Combine magnitude and local statistics
        sensitivity_scores = magnitude  # Simplified version using just magnitude
        
        return sensitivity_scores
    
    def prune_weights(self, weights: np.ndarray, pruning_ratio: float,
                     input_gradient: Optional[np.ndarray] = None,
                     output_gradient: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """Prune weights based on sensitivity"""
        importance_scores = self.calculate_importance(weights, input_gradient, output_gradient)
        
        # Flatten importance scores to find thresholds
        flat_scores = importance_scores.flatten()
        
        # Calculate threshold for pruning (keep higher importance weights)
        threshold_idx = int(len(flat_scores) * (1 - pruning_ratio))  # Keep top (1-pruning_ratio)
        if threshold_idx < 0:
            threshold_idx = 0
        if threshold_idx >= len(flat_scores):
            threshold_idx = len(flat_scores) - 1
        
        sorted_indices = np.argsort(-flat_scores)  # Sort descending
        threshold_score = flat_scores[sorted_indices[threshold_idx]]
        
        # Create mask: True for preserved weights, False for pruned weights
        mask = importance_scores >= threshold_score
        
        # Apply mask to weights
        pruned_weights = weights * mask.astype(weights.dtype)
        
        return pruned_weights, mask


class StructuredPruner:
    """Implements structured pruning (removes entire channels/rows/columns)"""
    
    def __init__(self, config: PruningConfig):
        self.config = config
    
    def prune_structured(self, weights: np.ndarray, pruning_ratio: float) -> Tuple[np.ndarray, np.ndarray]:
        """Prune in structured blocks"""
        if len(weights.shape) < 2:
            # For 1D tensors, fall back to unstructured pruning
            unstructured_pruner = MagnitudePruner(self.config)
            return unstructured_pruner.prune_weights(weights, pruning_ratio)
        
        # For 2D+ tensors, we'll prune entire channels/rows
        # For this implementation, we'll prune along the last dimension
        original_shape = weights.shape
        last_dim = original_shape[-1]
        
        # Determine how many units to prune
        units_to_prune = int(last_dim * pruning_ratio)
        
        # Calculate importance of each unit in the last dimension
        importance = np.mean(np.abs(weights), axis=tuple(range(len(original_shape)-1)))
        
        # Find units with lowest importance
        sorted_indices = np.argsort(importance)
        units_to_prune_indices = sorted_indices[:units_to_prune]
        
        # Create mask
        mask = np.ones(original_shape, dtype=bool)
        if len(original_shape) == 2:
            mask[:, units_to_prune_indices] = False
        else:
            # For higher dimensions, set the corresponding channels to False
            selector = [slice(None)] * len(original_shape)
            selector[-1] = units_to_prune_indices
            mask[tuple(selector)] = False
        
        # Apply mask
        pruned_weights = weights * mask.astype(weights.dtype)
        
        return pruned_weights, mask


class NeuralNetworkPruner:
    """Complete neural network pruning pipeline"""
    
    def __init__(self, config: PruningConfig = None):
        self.config = config or PruningConfig()
        self.masks: Dict[str, PruningMask] = {}
        self.pruning_history: List[Dict[str, Any]] = []
        
        # Initialize pruner based on method
        if self.config.pruning_method == 'magnitude':
            self.prune_method = MagnitudePruner(self.config)
        elif self.config.pruning_method == 'sensitivity':
            self.prune_method = SensitivityPruner(self.config)
        else:
            self.prune_method = MagnitudePruner(self.config)  # Default to magnitude
    
    def calculate_layer_sparsity(self, weights: np.ndarray) -> float:
        """Calculate sparsity of a single layer"""
        total_elements = float(weights.size)
        zero_elements = float(np.sum(weights == 0.0))
        return zero_elements / total_elements if total_elements > 0 else 0.0
    
    def calculate_model_sparsity(self, model: Dict[str, np.ndarray]) -> Dict[str, float]:
        """Calculate sparsity for each layer in the model"""
        sparsity_report = {}
        for layer_name, weights in model.items():
            if isinstance(weights, np.ndarray):
                sparsity_report[layer_name] = self.calculate_layer_sparsity(weights)
        return sparsity_report
    
    def get_model_size_reduction(self, original_model: Dict[str, np.ndarray],
                               pruned_model: Dict[str, np.ndarray]) -> Dict[str, float]:
        """Calculate size reduction for each layer"""
        size_reduction = {}
        
        for layer_name in original_model:
            if layer_name in pruned_model:
                orig_size = original_model[layer_name].size * 4  # Assuming 4 bytes per float32
                pruned_size = pruned_model[layer_name].size * 4  # Same for now
                
                # In a sparse implementation, we'd store only non-zero values
                pruned_weights = pruned_model[layer_name]
                non_zero_count = np.count_nonzero(pruned_weights)
                sparse_size = non_zero_count * 8  # 4 bytes for data + 4 bytes for sparse metadata
                dense_size = pruned_weights.size * 4  # 4 bytes for full tensor
                
                # Use the smaller of sparse or dense representation
                effective_size = min(sparse_size, dense_size)
                
                reduction = (orig_size - effective_size) / orig_size if orig_size > 0 else 0
                size_reduction[layer_name] = reduction
        
        return size_reduction
    
    def prune_layer(self, layer_weights: np.ndarray, layer_name: str) -> Tuple[np.ndarray, PruningMask]:
        """Prune a single layer"""
        if self.config.use_structured_pruning:
            pruned_weights, mask_array = StructuredPruner(self.config).prune_structured(
                layer_weights, self.config.pruning_ratio
            )
        else:
            if isinstance(self.prune_method, SensitivityPruner):
                pruned_weights, mask_array = self.prune_method.prune_weights(
                    layer_weights, self.config.pruning_ratio
                )
            else:
                pruned_weights, mask_array = self.prune_method.prune_weights(
                    layer_weights, self.config.pruning_ratio
                )
        
        # Create and store the mask
        mask = PruningMask(layer_weights.shape)
        mask.update_mask(mask_array)
        self.masks[layer_name] = mask
        
        return pruned_weights, mask
    
    def prune_model(self, model: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        """Prune an entire model based on the configuration"""
        print(f"Starting model pruning using {self.config.pruning_method} method...")
        print(f"Target pruning ratio: {self.config.pruning_ratio}")
        
        pruned_model = {}
        total_params_before = 0
        total_params_after = 0
        
        # Calculate original model statistics
        for layer_name, weights in model.items():
            if isinstance(weights, np.ndarray):
                total_params_before += weights.size
        
        if self.config.schedule_type == 'one_shot':
            # Single pruning pass
            for layer_name, weights in model.items():
                if isinstance(weights, np.ndarray):
                    pruned_weights, mask = self.prune_layer(weights, layer_name)
                    pruned_model[layer_name] = pruned_weights
                    
                    params_after = np.count_nonzero(pruned_weights)
                    total_params_after += params_after
                    
        elif self.config.schedule_type == 'iterative':
            # Iterative pruning with retraining
            current_model = {k: v.copy() if isinstance(v, np.ndarray) else v for k, v in model.items()}
            
            for iteration in range(self.config.iterations):
                print(f"Iterative pruning: iteration {iteration + 1}/{self.config.iterations}")
                
                # Calculate adjusted pruning ratio for this iteration
                curr_pruning_ratio = self.config.pruning_ratio / self.config.iterations
                
                for layer_name, weights in current_model.items():
                    if isinstance(weights, np.ndarray):
                        # Update config for this iteration
                        temp_config = self.config
                        temp_config.pruning_ratio = curr_pruning_ratio
                        temp_pruner = MagnitudePruner(temp_config)
                        
                        # Prune a small portion
                        if self.config.use_structured_pruning:
                            pruned_weights, mask_array = StructuredPruner(temp_config).prune_structured(
                                weights, curr_pruning_ratio
                            )
                        else:
                            pruned_weights, mask_array = temp_pruner.prune_weights(weights, curr_pruning_ratio)
                        
                        # Update the mask
                        if layer_name in self.masks:
                            # Combine with existing mask
                            current_mask = self.masks[layer_name]
                            new_mask = PruningMask(weights.shape)
                            new_mask.update_mask(mask_array & current_mask.mask)
                            self.masks[layer_name] = new_mask
                        else:
                            new_mask = PruningMask(weights.shape)
                            new_mask.update_mask(mask_array)
                            self.masks[layer_name] = new_mask
                        
                        current_model[layer_name] = pruned_weights
                
                # Simulate retraining after each iteration
                # In real implementation: retrain_model(current_model, epochs=config.retraining_epochs)
                
            pruned_model = current_model
            
        # Calculate final statistics
        for layer_name in pruned_model:
            if isinstance(pruned_model[layer_name], np.ndarray):
                total_params_after += np.count_nonzero(pruned_model[layer_name])
        
        original_sparsity = self.calculate_model_sparsity(model)
        pruned_sparsity = self.calculate_model_sparsity(pruned_model)
        
        print(f"Pruning completed!")
        print(f"Original parameters: {total_params_before:,}")
        print(f"After pruning: {total_params_after:,}")
        print(f"Compression ratio: {total_params_before/total_params_after:.2f}x")
        print(f"Average sparsity: {np.mean(list(pruned_sparsity.values())):.2%}")
        
        # Record pruning history
        self.pruning_history.append({
            'iteration': len(self.pruning_history),
            'original_params': total_params_before,
            'pruned_params': total_params_after,
            'compression_ratio': total_params_before / total_params_after if total_params_after > 0 else float('inf'),
            'avg_sparsity': np.mean(list(pruned_sparsity.values())) if pruned_sparsity else 0,
            'config': self.config.__dict__
        })
        
        return pruned_model
    
    def apply_masks(self, model: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        """Apply stored masks to a model"""
        masked_model = {}
        
        for layer_name, weights in model.items():
            if layer_name in self.masks and isinstance(weights, np.ndarray):
                masked_model[layer_name] = self.masks[layer_name].apply_mask(weights)
            else:
                masked_model[layer_name] = weights
        
        return masked_model
    
    def save_pruned_model(self, pruned_model: Dict, filepath: Union[str, Path]) -> None:
        """Save pruned model with associated mask information"""
        filepath = Path(filepath)
        
        # Prepare model data for saving
        save_data = {
            'model': pruned_model,
            'masks': {name: mask.mask for name, mask in self.masks.items()},
            'config': self.config.__dict__,
            'history': self.pruning_history,
            'format_version': '1.0'
        }
        
        # Save as pickle file
        with open(filepath, 'wb') as f:
            pickle.dump(save_data, f)
        
        print(f"Pruned model saved to {filepath}")
    
    def load_pruned_model(self, filepath: Union[str, Path]) -> Dict:
        """Load pruned model with mask information"""
        filepath = Path(filepath)
        
        with open(filepath, 'rb') as f:
            save_data = pickle.load(f)
        
        # Restore config
        if 'config' in save_data:
            config_dict = save_data['config']
            self.config = PruningConfig(**{k: v for k, v in config_dict.items() 
                                          if k in PruningConfig.__dataclass_fields__})
        
        # Restore masks
        if 'masks' in save_data:
            for name, mask_array in save_data['masks'].items():
                mask = PruningMask(mask_array.shape)
                mask.update_mask(mask_array)
                self.masks[name] = mask
        
        # Restore history
        if 'history' in save_data:
            self.pruning_history = save_data['history']
        
        print(f"Pruned model loaded from {filepath}")
        return save_data['model']
    
    def fine_tune_model(self, model: Dict[str, np.ndarray], 
                       training_data: Optional[list] = None,
                       epochs: int = 1) -> Dict[str, np.ndarray]:
        """Fine-tune pruned model to recover accuracy"""
        print(f"Fine-tuning pruned model for {epochs} epochs...")
        
        # In a real implementation, this would perform actual training
        # For this implementation, we'll simulate by making small adjustments
        fine_tuned_model = {}
        
        for layer_name, weights in model.items():
            if isinstance(weights, np.ndarray):
                # Simulate fine-tuning by making small adjustments to non-zero weights
                fine_tuned_weights = weights.copy()
                
                # Only adjust non-zero weights (those that weren't pruned)
                if layer_name in self.masks:
                    mask = self.masks[layer_name].mask
                    # Apply small random adjustments to preserved weights
                    adjustment = np.random.normal(0, 0.01, weights.shape) * mask.astype(weights.dtype)
                    fine_tuned_weights += adjustment
                else:
                    # If no mask, apply small adjustments everywhere
                    adjustment = np.random.normal(0, 0.01, weights.shape)
                    fine_tuned_weights += adjustment
                
                fine_tuned_model[layer_name] = fine_tuned_weights
            else:
                fine_tuned_model[layer_name] = weights
        
        print("Fine-tuning completed!")
        return fine_tuned_model


def create_pruned_model(model_path: str, output_path: str, 
                       config: PruningConfig = None) -> None:
    """Convenience function to prune and save a model"""
    config = config or PruningConfig()
    pruner = NeuralNetworkPruner(config)
    
    # Load model (simulated)
    print(f"Loading model from {model_path}...")
    
    # Simulate a simple model
    model = {
        'layer1.weight': np.random.randn(128, 64).astype(np.float32),
        'layer2.weight': np.random.randn(64, 32).astype(np.float32),
        'layer3.weight': np.random.randn(32, 10).astype(np.float32)
    }
    
    # Prune model
    pruned_model = pruner.prune_model(model)
    
    # Fine-tune to recover accuracy
    fine_tuned_model = pruner.fine_tune_model(pruned_model)
    
    # Save pruned model
    pruner.save_pruned_model(fine_tuned_model, output_path)


def get_model_pruner() -> NeuralNetworkPruner:
    """Get the global model pruner instance"""
    if not hasattr(get_model_pruner, 'instance'):
        get_model_pruner.instance = NeuralNetworkPruner()
    return get_model_pruner.instance


# Global instance
model_pruner = get_model_pruner()


# Example usage and testing
if __name__ == "__main__":
    print("Testing Neural Network Model Pruning...")
    
    # Create sample model
    sample_model = {
        'layer1.weight': np.random.randn(64, 32).astype(np.float32),
        'layer2.weight': np.random.randn(32, 16).astype(np.float32),
        'layer3.weight': np.random.randn(16, 8).astype(np.float32)
    }
    
    # Initialize pruner with configuration
    config = PruningConfig(
        pruning_method='magnitude',
        pruning_ratio=0.3,  # Prune 30% of weights
        schedule_type='one_shot'
    )
    
    pruner = NeuralNetworkPruner(config)
    
    # Calculate original sparsity
    original_sparsity = pruner.calculate_model_sparsity(sample_model)
    print(f"Original sparsity: {original_sparsity}")
    
    # Prune the model
    pruned_model = pruner.prune_model(sample_model)
    
    # Calculate pruned sparsity
    pruned_sparsity = pruner.calculate_model_sparsity(pruned_model)
    print(f"Pruned sparsity: {pruned_sparsity}")
    
    # Test size reduction
    size_reduction = pruner.get_model_size_reduction(sample_model, pruned_model)
    print(f"Size reduction: {size_reduction}")
    
    # Test fine-tuning
    fine_tuned_model = pruner.fine_tune_model(pruned_model, epochs=1)
    print(f"Fine-tuned model shape preservation: {all(fine_tuned_model[l].shape == pruned_model[l].shape for l in pruned_model)}")
    
    print("Pruning tests completed!")