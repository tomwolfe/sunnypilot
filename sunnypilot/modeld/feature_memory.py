import numpy as np

class FeatureMemory:
  """
  Transformer-based Feature Memory for E2E Path Planning.
  Replaces rolling history with an Attention mechanism over a latent buffer.
  """
  def __init__(self, feature_len, max_history=50):
    self.feature_len = feature_len
    self.max_history = max_history
    self.history = [] # List of past latent features
    
    # Simple Linear Projections for Attention (Q, K, V)
    # In a production environment, these weights would be learned.
    # We initialize them as identity to start with "mean-pooling" behavior.
    self.W_q = np.eye(feature_len, dtype=np.float32)
    self.W_k = np.eye(feature_len, dtype=np.float32)
    self.W_v = np.eye(feature_len, dtype=np.float32)

  def add(self, feature: np.ndarray):
    """Adds a new feature to the memory."""
    if len(self.history) >= self.max_history:
      self.history.pop(0)
    self.history.append(feature.copy())

  def get_contextual_feature(self, current_feature: np.ndarray) -> np.ndarray:
    """
    Computes a contextual feature using Scaled Dot-Product Attention.
    """
    if not self.history:
      return current_feature

    # Stack history for batch computation
    history_stack = np.array(self.history) # (N, FEATURE_LEN)
    
    # 1. Project to Q, K, V
    query = current_feature @ self.W_q # (FEATURE_LEN,)
    keys = history_stack @ self.W_k    # (N, FEATURE_LEN)
    values = history_stack @ self.W_v  # (N, FEATURE_LEN)
    
    # 2. Scaled Dot-Product Attention
    d_k = self.feature_len
    scores = (query @ keys.T) / np.sqrt(d_k) # (N,)
    
    # 3. Softmax
    weights = np.exp(scores - np.max(scores))
    weights /= weights.sum()
    
    # 4. Weighted Sum of Values
    context_vector = weights @ values # (FEATURE_LEN,)
    
    # Residual Connection
    return (current_feature + context_vector) / 2.0
