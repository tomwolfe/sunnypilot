"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import pyray as rl
from dataclasses import dataclass, field
from openpilot.system.ui.lib.shader_polygon import draw_polygon


@dataclass
class MHPPath:
  """Multi-Hypothesis Path - alternate trajectories the E2E model is considering."""
  points: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float32))
  projected_points: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.float32))
  probability: float = 0.0
  is_selected: bool = False


class MultiHypothesisPathVisualizer:
  """
  Renders alternate "ghost" paths from the E2E model's Multi-Hypothesis Path (MHP) predictions.
  This provides transparency into the AI's intent at intersections and complex scenarios.
  """
  GHOST_ALPHA = 40
  GHOST_WIDTH = 0.15
  MAX_HYPOTHESES = 5
  CONFIDENCE_THRESHOLD = 0.15

  def __init__(self):
    self._hypotheses: list[MHPPath] = [MHPPath() for _ in range(self.MAX_HYPOTHESES)]
    self._enabled = True

  @property
  def enabled(self) -> bool:
    return self._enabled

  @enabled.setter
  def enabled(self, value: bool):
    self._enabled = value

  def update_paths(self, model, road_camera_offset: float = 0.0) -> None:
    """Update MHP paths from model data."""
    if not hasattr(model, 'position') or model.position is None:
      return

    if not hasattr(model.position, 'x'):
      return

    primary_x = model.position.x
    primary_y = model.position.y

    if len(primary_x) == 0:
      return

    for i in range(self.MAX_HYPOTHESES):
      offset_key = f'alternateOffset{i}'
      if not hasattr(model.position, offset_key):
        break

      offset_x = getattr(model.position, f'alternateX{i}', None)
      offset_y = getattr(model.position, f'alternateY{i}', None)

      if offset_x is None or offset_y is None or len(offset_x) == 0:
        continue

      prob_key = f'alternateProb{i}'
      probability = getattr(model.position, prob_key, 0.0) if hasattr(model.position, prob_key) else 0.0

      if probability < self.CONFIDENCE_THRESHOLD:
        continue

      points = np.zeros((len(offset_x), 3), dtype=np.float32)
      points[:, 0] = np.array(offset_x, dtype=np.float32)
      points[:, 1] = np.array(offset_y, dtype=np.float32) + road_camera_offset
      points[:, 2] = 0.0

      self._hypotheses[i] = MHPPath(
        points=points,
        probability=probability,
        is_selected=(i == 0)
      )

  def project_paths(self, transform: np.ndarray, path_offset_z: float,
                   max_idx: int, max_distance: float, rect: rl.Rectangle) -> None:
    """Project MHP paths to screen space."""
    for hp in self._hypotheses:
      if hp.points.size == 0:
        continue

      raw_x = hp.points[:, 0]
      raw_y = hp.points[:, 1]

      valid_mask = (raw_x <= max_distance)
      if not np.any(valid_mask):
        hp.projected_points = np.empty((0, 2), dtype=np.float32)
        continue

      idx = min(np.sum(valid_mask), max_idx)
      if idx == 0:
        hp.projected_points = np.empty((0, 2), dtype=np.float32)
        continue

      x = raw_x[:idx]
      y = raw_y[:idx]
      z = hp.points[:idx, 2] + path_offset_z

      screen_points = np.zeros((idx * 2, 2), dtype=np.float32)

      for j in range(idx):
        px = transform[0, 0] * x[j] + transform[0, 1] * y[j] + transform[0, 2] * z[j] + transform[0, 2]
        py = transform[1, 0] * x[j] + transform[1, 1] * y[j] + transform[1, 2] * z[j] + transform[1, 2]
        pz = transform[2, 0] * x[j] + transform[2, 1] * y[j] + transform[2, 2] * z[j] + transform[2, 2]

        if pz > 0.1:
          screen_x = (px / pz) * rect.width + rect.x
          screen_y = (py / pz) * rect.height + rect.y
          screen_points[j] = [screen_x, screen_y]
          screen_points[idx * 2 - 1 - j] = [screen_x, screen_y]
        else:
          screen_points[j] = [rect.x + rect.width / 2, rect.y + rect.height]
          screen_points[idx * 2 - 1 - j] = [rect.x + rect.width / 2, rect.y + rect.height]

      hp.projected_points = screen_points

  def draw(self, rect) -> None:
    """Render ghost paths as faint overlay lines."""
    if not self._enabled:
      return

    for i, hp in enumerate(self._hypotheses):
      if hp.projected_points.size == 0 or hp.is_selected:
        continue

      base_alpha = int(self.GHOST_ALPHA * hp.probability * 2.0)
      base_alpha = min(255, base_alpha)

      if i % 2 == 0:
        color = rl.Color(0, 200, 255, base_alpha)
      else:
        color = rl.Color(255, 180, 0, base_alpha)

      draw_polygon(rect, hp.projected_points, color)


class MHPIntersectionAnalyzer:
  """
  Analyzes MHP paths to determine if the car is at an intersection.
  Useful for providing context-aware alerts and mode switching.
  """
  def __init__(self):
    self._at_intersection = False
    self._turn_intent = None
    self._path_divergence = 0.0

  def analyze(self, mhp_visualizer: MultiHypothesisPathVisualizer, v_ego: float) -> dict:
    """
    Analyze MHP paths to detect intersection scenarios.
    Returns dict with intersection state and recommended actions.
    """
    active_hypotheses = [hp for hp in mhp_visualizer._hypotheses
                        if hp.points.size > 0 and hp.probability > 0.2]

    if len(active_hypotheses) < 2:
      return {
        'at_intersection': False,
        'turn_intent': None,
        'divergence': 0.0,
        'recommendation': 'normal'
      }

    x_coords = [hp.points[-1, 0] for hp in active_hypotheses]
    y_coords = [hp.points[-1, 1] for hp in active_hypotheses]

    x_spread = max(x_coords) - min(x_coords) if len(x_coords) > 1 else 0.0
    y_spread = max(y_coords) - min(y_coords) if len(y_coords) > 1 else 0.0

    self._path_divergence = np.sqrt(x_spread**2 + y_spread**2)

    at_intersection = self._path_divergence > 3.0 and v_ego < 15.0

    turn_intent = None
    if at_intersection and len(active_hypotheses) > 0:
      left_count = sum(1 for hp in active_hypotheses if hp.points[-1, 1] > 0.5)
      right_count = sum(1 for hp in active_hypotheses if hp.points[-1, 1] < -0.5)

      if left_count > right_count:
        turn_intent = 'left'
      elif right_count > left_count:
        turn_intent = 'right'

    recommendation = 'normal'
    if at_intersection:
      if turn_intent:
        recommendation = f'caution_{turn_intent}'
      else:
        recommendation = 'caution_unknown'

    return {
      'at_intersection': at_intersection,
      'turn_intent': turn_intent,
      'divergence': self._path_divergence,
      'recommendation': recommendation
    }
