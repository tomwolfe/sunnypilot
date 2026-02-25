"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import pyray as rl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient


class AttentionVisualizer:
  """
  E2E Explainability: Renders 'Attention Heatmaps' on the road.
  Uses longitudinal and lateral uncertainty (xStd, yStd) to highlight 
  what the model is 'thinking' about.
  """
  def __init__(self):
    self._attention_gradient = Gradient(
      start=(0.0, 1.0),
      end=(0.0, 0.0),
      colors=[],
      stops=[],
    )

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
      
      # Uncertainty Heatmap: 
      # High xStd/yStd indicates the model is struggling or focusing on an obstacle.
      # We color high-uncertainty areas with a "Heat" glow (Magenta/Red).
      uncertainty = float(model_v2.position.xStd[i])
      
      # Scale uncertainty: typically 0-15m. 
      # Anything above 5m starts to 'glow'.
      heat_intensity = np.clip((uncertainty - 2.0) / 8.0, 0.0, 1.0)
      
      if heat_intensity > 0.1:
        # Magenta for high uncertainty (Attention/Focus)
        color = rl.Color(255, 0, 255, int(150 * heat_intensity))
      else:
        # Transparent for high confidence
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
      # Draw with additive blending for a 'glow' effect
      rl.begin_blend_mode(rl.BlendMode.BLEND_ADDITIVE)
      draw_polygon(rect, path_projected, gradient=gradient)
      rl.end_blend_mode()
