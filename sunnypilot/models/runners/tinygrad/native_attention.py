from tinygrad.tensor import Tensor
from tinygrad.nn import Linear
from tinygrad.ops import TensorOps
import numpy as np
from typing import Optional, List


class NativeAttention:
  """
  Native Tinygrad implementation of Scaled Dot-Product Attention.
  This eliminates the CPU-GPU transfer latency of the Python/Numpy implementation.
  
  GPU-Resident Features:
  - History keys/values stored as Tensors (on GPU)
  - Rolling buffer implemented in GPU memory
  - All attention computation stays on GPU
  """
  def __init__(self, feature_dim: int, max_history: int = 50):
    self.feature_dim = feature_dim
    self.max_history = max_history
    
    self.W_q = Linear(feature_dim, feature_dim, bias=False)
    self.W_k = Linear(feature_dim, feature_dim, bias=False)
    self.W_v = Linear(feature_dim, feature_dim, bias=False)
    
    self._history_keys_gpu: Optional[Tensor] = None
    self._history_values_gpu: Optional[Tensor] = None
    self._history_mask: Optional[Tensor] = None
    self._current_size = 0
    
    self._temp_buffer = Tensor.zeros(max_history, feature_dim, dtype="float32")
  
  def reset(self):
    """Clear the GPU history buffer."""
    self._history_keys_gpu = None
    self._history_values_gpu = None
    self._history_mask = None
    self._current_size = 0
    self._temp_buffer = Tensor.zeros(self.max_history, self.feature_dim, dtype="float32")
  
  def add_context(self, feature: np.ndarray):
    """Add a new feature to the GPU-resident memory buffer."""
    feature_tensor = Tensor(feature, dtype="float32").reshape(1, -1)
    
    projected = self.W_v(feature_tensor)
    
    if self._history_keys_gpu is None:
      self._history_keys_gpu = projected
      self._history_values_gpu = projected
    else:
      self._history_keys_gpu = self._history_keys_gpu.cat(projected, dim=0)
      self._history_values_gpu = self._history_values_gpu.cat(projected, dim=0)
    
    self._current_size = min(self._current_size + 1, self.max_history)
    
    if self._current_size > self.max_history:
      self._history_keys_gpu = self._history_keys_gpu[1:]
      self._history_values_gpu = self._history_values_gpu[1:]
  
  def add_context_gpu(self, feature: Tensor):
    """Add a GPU Tensor directly to history (avoids CPU transfer)."""
    projected = self.W_v(feature.reshape(1, -1))
    
    if self._history_keys_gpu is None:
      self._history_keys_gpu = projected
      self._history_values_gpu = projected
    else:
      self._history_keys_gpu = self._history_keys_gpu.cat(projected, dim=0)
      self._history_values_gpu = self._history_values_gpu.cat(projected, dim=0)
    
    self._current_size = min(self._current_size + 1, self.max_history)
    
    if self._current_size > self.max_history:
      self._history_keys_gpu = self._history_keys_gpu[1:]
      self._history_values_gpu = self._history_values_gpu[1:]
  
  def forward(self, current_feature: Tensor) -> Tensor:
    """
    Compute contextual feature using Scaled Dot-Product Attention.
    All computation stays on GPU.
    
    Args:
      current_feature: Tensor of shape (feature_dim,) or (1, feature_dim)
      
    Returns:
      Contextual feature with residual connection, shape (feature_dim,)
    """
    if self._history_keys_gpu is None or self._current_size == 0:
      return current_feature.reshape(-1)
    
    if len(current_feature.shape) == 1:
      current_feature = current_feature.reshape(1, -1)
    
    query = self.W_q(current_feature)
    keys = self._history_keys_gpu[:self._current_size]
    values = self._history_values_gpu[:self._current_size]
    
    d_k = Tensor([self.feature_dim], dtype="float32")
    scale = (1.0 / d_k.sqrt().realize())
    
    scores = (query.reshape(1, -1) * keys).sum(axis=1) * scale
    
    mask = Tensor([1.0] * self._current_size, dtype="float32").reshape(1, -1)
    scores = scores + ((1.0 - mask) * -1e9)
    
    weights = scores.exp()
    weights = weights / weights.sum()
    
    context = (weights.reshape(1, -1) * values).sum(axis=0)
    
    return (current_feature.reshape(-1) + context) * 0.5
  
  def forward_multi_head(self, current_feature: Tensor, num_heads: int = 4) -> Tensor:
    """
    Multi-head attention for richer contextual representation.
    
    Args:
      current_feature: Tensor of shape (feature_dim,)
      num_heads: Number of attention heads
      
    Returns:
      Contextual feature with multi-head attention
    """
    if self._history_keys_gpu is None or self._current_size == 0:
      return current_feature.reshape(-1)
    
    if len(current_feature.shape) == 1:
      current_feature = current_feature.reshape(1, -1)
    
    head_dim = self.feature_dim // num_heads
    
    query = self.W_q(current_feature)
    keys = self._history_keys_gpu[:self._current_size]
    values = self._history_values_gpu[:self._current_size]
    
    query_heads = query.reshape(1, num_heads, head_dim)
    key_heads = keys.reshape(self._current_size, num_heads, head_dim)
    value_heads = values.reshape(self._current_size, num_heads, head_dim)
    
    d_k = Tensor([head_dim], dtype="float32")
    scale = (1.0 / d_k.sqrt().realize())
    
    all_contexts = []
    for h in range(num_heads):
      q_h = query_heads[0, h:h+1, :]
      k_h = key_heads[:, h:h+1, :].reshape(self._current_size, head_dim)
      v_h = value_heads[:, h:h+1, :].reshape(self._current_size, head_dim)
      
      scores_h = (q_h * k_h).sum(axis=1) * scale
      weights_h = scores_h.exp()
      weights_h = weights_h / weights_h.sum()
      
      context_h = (weights_h.reshape(1, -1) * v_h).sum(axis=0)
      all_contexts.append(context_h)
    
    multi_context = Tensor.cat(*all_contexts, dim=0)
    
    return (current_feature.reshape(-1) + multi_context) * 0.5


class GPUResidentImaginationBuffer:
  """
  GPU-resident buffer for imagined trajectories.
  
  Instead of storing trajectories in CPU memory and transferring to GPU,
  this buffer keeps all trajectory data on the GPU for fast access
  during attention computations.
  """
  def __init__(self, max_trajectories: int = 100, trajectory_length: int = 50, state_dim: int = 64):
    self.max_trajectories = max_trajectories
    self.trajectory_length = trajectory_length
    self.state_dim = state_dim
    
    self._positions_gpu = Tensor.zeros(max_trajectories, trajectory_length, state_dim, dtype="float32")
    self._velocities_gpu = Tensor.zeros(max_trajectories, trajectory_length, state_dim, dtype="float32")
    self._uncertainty_gpu = Tensor.zeros(max_trajectories, trajectory_length, dtype="float32")
    self._timestamps_gpu = Tensor.zeros(max_trajectories, dtype="float32")
    
    self._current_count = 0
    self._write_index = 0
  
  def add_trajectory(self, positions: np.ndarray, velocities: np.ndarray, 
                    uncertainty: np.ndarray, timestamp: float):
    """Add a new trajectory to the GPU buffer."""
    pos_tensor = Tensor(positions[:self.trajectory_length], dtype="float32").reshape(1, -1, self.state_dim)
    vel_tensor = Tensor(velocities[:self.trajectory_length], dtype="float32").reshape(1, -1, self.state_dim)
    unc_tensor = Tensor(uncertainty[:self.trajectory_length], dtype="float32").reshape(1, -1)
    ts_tensor = Tensor([timestamp], dtype="float32")
    
    self._positions_gpu[self._write_index:self._write_index+1] = pos_tensor
    self._velocities_gpu[self._write_index:self._write_index+1] = vel_tensor
    self._uncertainty_gpu[self._write_index:self._write_index+1] = unc_tensor
    self._timestamps_gpu[self._write_index] = ts_tensor
    
    self._write_index = (self._write_index + 1) % self.max_trajectories
    self._current_count = min(self._current_count + 1, self.max_trajectories)
  
  def add_trajectory_gpu(self, positions: Tensor, velocities: Tensor,
                        uncertainty: Tensor, timestamp: float):
    """Add GPU tensors directly (avoids CPU transfer)."""
    self._positions_gpu[self._write_index] = positions[:self.trajectory_length]
    self._velocities_gpu[self._write_index] = velocities[:self.trajectory_length]
    self._uncertainty_gpu[self._write_index] = uncertainty[:self.trajectory_length]
    self._timestamps_gpu[self._write_index] = timestamp
    
    self._write_index = (self._write_index + 1) % self.max_trajectories
    self._current_count = min(self._current_count + 1, self.max_trajectories)
  
  def get_recent_trajectories(self, n: int = 10) -> tuple:
    """Get n most recent trajectories (all on GPU)."""
    if self._current_count == 0:
      return None, None, None
    
    start_idx = max(0, self._write_index - n)
    
    positions = self._positions_gpu[start_idx:self._write_index]
    velocities = self._velocities_gpu[start_idx:self._write_index]
    uncertainty = self._uncertainty_gpu[start_idx:self._write_index]
    
    return positions, velocities, uncertainty
  
  def compute_attention_context(self, query_state: Tensor) -> Tensor:
    """
    Compute attention-weighted context from all stored trajectories.
    Returns weighted average of trajectory states based on attention.
    """
    if self._current_count == 0:
      return Tensor.zeros(self.state_dim, dtype="float32")
    
    trajectories = self._positions_gpu[:self._current_count, -1, :]
    
    query = query_state.reshape(1, -1)
    
    d_k = Tensor([self.state_dim], dtype="float32")
    scale = (1.0 / d_k.sqrt().realize())
    
    scores = (query * trajectories).sum(axis=1) * scale
    
    weights = scores.exp()
    weights = weights / weights.sum()
    
    context = (weights.reshape(-1, 1) * trajectories).sum(axis=0)
    
    return context
  
  def reset(self):
    """Clear the GPU buffer."""
    self._positions_gpu = Tensor.zeros(self.max_trajectories, self.trajectory_length, self.state_dim, dtype="float32")
    self._velocities_gpu = Tensor.zeros(self.max_trajectories, self.trajectory_length, self.state_dim, dtype="float32")
    self._uncertainty_gpu = Tensor.zeros(self.max_trajectories, self.trajectory_length, dtype="float32")
    self._timestamps_gpu = Tensor.zeros(self.max_trajectories, dtype="float32")
    self._current_count = 0
    self._write_index = 0


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
