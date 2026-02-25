from tinygrad.tensor import Tensor
from tinygrad.nn import Linear
from tinygrad.ops import TensorOps
import numpy as np


class NativeAttention:
  """
  Native Tinygrad implementation of Scaled Dot-Product Attention.
  This eliminates the CPU-GPU transfer latency of the Python/Numpy implementation.
  """
  def __init__(self, feature_dim: int, max_history: int = 50):
    self.feature_dim = feature_dim
    self.max_history = max_history
    
    self.W_q = Linear(feature_dim, feature_dim, bias=False)
    self.W_k = Linear(feature_dim, feature_dim, bias=False)
    self.W_v = Linear(feature_dim, feature_dim, bias=False)
    
    self.history_keys = []
    self.history_values = []
  
  def reset(self):
    """Clear the history buffer."""
    self.history_keys = []
    self.history_values = []
  
  def add_context(self, feature: np.ndarray):
    """Add a new feature to the memory buffer."""
    if len(self.history_keys) >= self.max_history:
      self.history_keys.pop(0)
      self.history_values.pop(0)
    
    self.history_keys.append(Tensor(feature, dtype="float32"))
    self.history_values.append(Tensor(feature, dtype="float32"))
  
  def forward(self, current_feature: Tensor) -> Tensor:
    """
    Compute contextual feature using Scaled Dot-Product Attention.
    
    Args:
      current_feature: Tensor of shape (feature_dim,)
      
    Returns:
      Contextual feature with residual connection, shape (feature_dim,)
    """
    if not self.history_keys:
      return current_feature
    
    query = self.W_q(current_feature)
    
    keys = self.W_k(Tensor.stack(*self.history_keys))
    values = self.W_v(Tensor.stack(*self.history_values))
    
    d_k = Tensor([self.feature_dim], dtype="float32")
    scale = (1.0 / d_k.sqrt().realize())
    
    scores = (query.reshape(1, -1) * keys).sum(axis=1) * scale
    
    weights = scores.exp()
    weights = weights / weights.sum()
    
    context = (weights.reshape(1, -1) * values).sum(axis=0)
    
    return (current_feature + context) * 0.5


class AttentionRunner:
  """
  Wrapper for running attention within the Tinygrad execution graph.
  Maintains state across inference calls for temporal context.
  """
  def __init__(self, feature_dim: int = 1024, max_history: int = 50):
    self.attention = NativeAttention(feature_dim, max_history)
    self.feature_dim = feature_dim
    self.max_history = max_history
  
  def update_and_run(self, feature: Tensor) -> Tensor:
    """
    Add feature to history and compute contextual output.
    
    This method maintains the history internally and should be called
    after each model inference to build temporal context.
    """
    result = self.attention.forward(feature)
    self.attention.add_context(feature.numpy())
    return result
  
  def run_only(self, current_feature: Tensor) -> Tensor:
    """
    Compute contextual output without updating history.
    Use this for debugging or when you want to manually control history.
    """
    return self.attention.forward(current_feature)
  
  def reset(self):
    """Reset the attention history."""
    self.attention.reset()
  
  @property
  def history_size(self) -> int:
    """Return current history size."""
    return len(self.attention.history_keys)


def create_attention_runner(feature_dim: int = 1024, max_history: int = 50) -> AttentionRunner:
  """
  Factory function to create an AttentionRunner.
  
  Args:
    feature_dim: Dimension of the latent features (default: 1024 from model)
    max_history: Maximum history buffer size (default: 50 = ~2.5s at 20Hz)
    
  Returns:
    AttentionRunner instance
  """
  return AttentionRunner(feature_dim, max_history)
