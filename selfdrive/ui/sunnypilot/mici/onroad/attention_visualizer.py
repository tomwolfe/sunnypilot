"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import pyray as rl
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient


class SaliencyMap:
    """
    Saliency map data for XAI visualization
    """
    def __init__(self, width: int = 256, height: int = 128):
        self.width = width
        self.height = height
        self.data = np.zeros((height, width), dtype=np.float32)
        self.max_value = 1.0
        self.focus_points = []

    def compute_from_attention(self, attention_weights: np.ndarray, feature_h: int = 16, feature_w: int = 32):
        """Compute saliency map from attention weights"""
        if attention_weights is None or len(attention_weights.flatten()) == 0:
            return

        attn = attention_weights.flatten()

        if len(attn) > feature_h * feature_w:
            attn = attn[:feature_h * feature_w]

        attn_2d = attn.reshape(feature_h, feature_w)

        self.data = np.kron(attn_2d, np.ones((self.height // feature_h, self.width // feature_w), dtype=np.float32))

        self.max_value = np.max(self.data) if np.max(self.data) > 0 else 1.0

        self._extract_focus_points(attn_2d)

    def _extract_focus_points(self, attention_2d: np.ndarray, threshold: float = 0.7):
        """Extract focus points (high attention regions)"""
        max_attn = np.max(attention_2d)
        if max_attn > 0:
            threshold_value = max_attn * threshold
            rows, cols = np.where(attention_2d > threshold_value)

            self.focus_points = [(int(col), int(row)) for col, row in zip(cols, rows, strict=False)]

    def get_normalized(self) -> np.ndarray:
        """Get normalized saliency map [0, 1]"""
        return self.data / self.max_value if self.max_value > 0 else self.data


class AttentionVisualizer:
  """
  E2E Explainability: Renders 'Attention Heatmaps' and 'Saliency Maps' on the road.
  Uses longitudinal and lateral uncertainty (xStd, yStd) to highlight
  what the model is 'thinking' about.

  XAI Features:
  - Uncertainty Heatmap: Shows model confidence across the path
  - Saliency Maps: Highlights which image regions the model focuses on
  - Focus Point Markers: Shows specific objects/regions of interest
  """
  def __init__(self):
    self._attention_gradient = Gradient(
      start=(0.0, 1.0),
      end=(0.0, 0.0),
      colors=[],
      stops=[],
    )
    self._saliency_map = SaliencyMap()
    self._show_saliency = True
    self._show_focus_points = True
    self._saliency_opacity = 0.6

  def update_saliency(self, model_v2, camera_image=None):
    """Update saliency map from model attention"""
    if not hasattr(model_v2, 'attention_weights') or model_v2.attention_weights is None:
      if hasattr(model_v2, 'position') and hasattr(model_v2.position, 'xStd'):
        self._compute_implicit_saliency(model_v2)
      return

    try:
      attn_weights = np.array(model_v2.attention_weights)
      self._saliency_map.compute_from_attention(attn_weights)
    except Exception:
      if hasattr(model_v2, 'position') and hasattr(model_v2.position, 'xStd'):
        self._compute_implicit_saliency(model_v2)

  def _compute_implicit_saliency(self, model_v2):
    """Compute implicit saliency from model uncertainty/attention"""
    if not hasattr(model_v2.position, 'xStd') or len(model_v2.position.xStd) == 0:
      return

    num_points = min(len(model_v2.position.xStd), 33)
    attn_weights = np.zeros(num_points, dtype=np.float32)

    for i in range(num_points):
      x_std = float(model_v2.position.xStd[i]) if i < len(model_v2.position.xStd) else 0.0
      y_std = float(model_v2.position.yStd[i]) if i < len(model_v2.position.yStd) else 0.0

      combined_uncertainty = (x_std + y_std) / 20.0
      attn_weights[i] = 1.0 - np.clip(combined_uncertainty, 0.0, 1.0)

    self._saliency_map.compute_from_attention(attn_weights, feature_h=8, feature_w=16)

  def draw_saliency_overlay(self, rect, camera_image):
    """Draw saliency map as overlay on road view"""
    if not self._show_saliency:
      return

    saliency = self._saliency_map.get_normalized()
    if saliency.size == 0:
      return

    saliency_resized = self._resize_saliency(saliency, rect.width, rect.height)

    rl.begin_blend_mode(rl.BlendMode.BLEND_ADDITIVE)

    for y in range(rect.height):
      for x in range(rect.width):
        val = saliency_resized[y, x]
        if val > 0.1:
          alpha = int(val * 255 * self._saliency_opacity)

          if val > 0.7:
            color = rl.Color(255, 50, 50, alpha)
          elif val > 0.4:
            color = rl.Color(255, 200, 50, alpha)
          else:
            color = rl.Color(50, 200, 255, alpha)

          rl.draw_pixel(x + int(rect.x), y + int(rect.y), color)

    rl.end_blend_mode()

    if self._show_focus_points:
      self._draw_focus_points(rect)

  def _resize_saliency(self, saliency: np.ndarray, target_w: int, target_h: int) -> np.ndarray:
    """Resize saliency map to target dimensions"""
    src_h, src_w = saliency.shape

    if src_h == target_h and src_w == target_w:
      return saliency

    resized = np.zeros((target_h, target_w), dtype=np.float32)

    scale_x = src_w / target_w
    scale_y = src_h / target_h

    for y in range(target_h):
      for x in range(target_w):
        src_x = int(x * scale_x)
        src_y = int(y * scale_y)
        src_x = min(src_x, src_w - 1)
        src_y = min(src_y, src_h - 1)
        resized[y, x] = saliency[src_y, src_x]

    return resized

  def _draw_focus_points(self, rect):
    """Draw markers for high-attention focus points"""
    focus_points = self._saliency_map.focus_points

    if not focus_points:
      return

    scale_x = rect.width / 32.0
    scale_y = rect.height / 16.0

    for fx, fy in focus_points[:5]:
      x = int(rect.x + fx * scale_x)
      y = int(rect.y + fy * scale_y)

      rl.draw_circle_lines(x, y, 8, rl.Color(255, 255, 0, 200))
      rl.draw_circle_lines(x, y, 4, rl.Color(255, 100, 0, 255))

  def update_heatmap(self, rect, path_projected, model_v2):
    """Calculates a heatmap based on model uncertainty."""
    if not hasattr(model_v2.position, 'xStd') or len(model_v2.position.xStd) == 0:
      return None

    max_len = min(len(path_projected) // 2, len(model_v2.position.xStd))
    segment_colors = []
    gradient_stops = []

    for i in range(0, max_len, 2):
      track_y = path_projected[i][1]
      if track_y < rect.y or track_y > (rect.y + rect.height):
        continue

      lin_grad_point = 1 - (track_y - rect.y) / rect.height

      uncertainty = float(model_v2.position.xStd[i])

      heat_intensity = np.clip((uncertainty - 2.0) / 8.0, 0.0, 1.0)

      if heat_intensity > 0.1:
        color = rl.Color(255, 0, 255, int(150 * heat_intensity))
      else:
        color = rl.Color(0, 0, 0, 0)

      gradient_stops.append(lin_grad_point)
      segment_colors.append(color)

    if len(segment_colors) < 2:
      return None

    self._attention_gradient.colors = segment_colors
    self._attention_gradient.stops = gradient_stops
    return self._attention_gradient

  def draw(self, rect, path_projected, gradient):
    """Renders the attention heatmap over the path."""
    if gradient and len(gradient.colors) > 1:
      rl.begin_blend_mode(rl.BlendMode.BLEND_ADDITIVE)
      draw_polygon(rect, path_projected, gradient=gradient)
      rl.end_blend_mode()

  def set_saliency_visibility(self, visible: bool):
    """Toggle saliency map visibility"""
    self._show_saliency = visible

  def set_focus_points_visibility(self, visible: bool):
    """Toggle focus point markers visibility"""
    self._show_focus_points = visible

  def set_saliency_opacity(self, opacity: float):
    """Set saliency overlay opacity [0, 1]"""
    self._saliency_opacity = np.clip(opacity, 0.0, 1.0)
