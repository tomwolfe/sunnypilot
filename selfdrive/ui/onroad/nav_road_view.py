"""
Navigation Road View for sunnypilot - Displays navigation information on the road view
Copyright (c) 2025, sunnypilot community

Safety Note: This implementation is designed to minimize visual distraction
by positioning navigation elements in non-critical areas of the driving view.
Turn arrows appear at top center and distance/ETA info at top right,
avoiding overlap with lane lines, vehicle detection and other critical
driving information areas.
"""
import datetime
import pyray as rl
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.common.realtime import DT_MDL
from openpilot.common.transformations.model import ModelConstants


class NavRoadView(Widget):
  # UI constants to avoid hardcoded values
  NAV_ARROW_Y_OFFSET = 50
  INFO_PANEL_X_MARGIN = 20
  INFO_PANEL_Y_MARGIN = 20
  INFO_PANEL_WIDTH = 300
  INFO_PANEL_HEIGHT = 120
  INFO_PANEL_BG_ALPHA = 240
  DISTANCE_FONT_SIZE = 24
  SECONDARY_FONT_SIZE = 20
  ETA_FONT_SIZE = 20
  TEXT_LINE_SPACING = 2
  TEXT_LINE_SPACING_SECONDARY = 1.5

  def __init__(self):
    super().__init__()
    self._init_textures()

    # Navigation state
    self._nav_instruction = None
    self._distance_to_maneuver = float('inf')
    self._show_full = False
    self._eta_seconds = float('inf')
    self._distance_remaining = float('inf')

    # Animation state
    self._turn_arrow_alpha = 0.0
    self._info_panel_alpha = 0.0

  def _init_textures(self):
    """Initialize navigation-related textures"""
    # Turn arrow textures
    self._left_arrow = gui_app.texture("sunnypilot/selfdrive/assets/onroad/icon_turn_left.png", 100, 100)
    self._right_arrow = gui_app.texture("sunnypilot/selfdrive/assets/onroad/icon_turn_right.png", 100, 100)
    self._straight_arrow = gui_app.texture("sunnypilot/selfdrive/assets/onroad/icon_turn_straight.png", 100, 100)
    self._slight_left_arrow = gui_app.texture("sunnypilot/selfdrive/assets/onroad/icon_turn_slight_left.png", 100, 100)
    self._slight_right_arrow = gui_app.texture("sunnypilot/selfdrive/assets/onroad/icon_turn_slight_right.png", 100, 100)

  def update(self):
    """Update navigation data from message"""
    sm = ui_state.sm

    # Update navigation instruction if available
    if sm.updated['navInstruction']:
      nav_instr = sm['navInstruction']
      if nav_instr:
        self._nav_instruction = nav_instr
        self._distance_to_maneuver = nav_instr.maneuverDistance
        self._show_full = nav_instr.showFull
        self._eta_seconds = nav_instr.timeRemaining
        self._distance_remaining = nav_instr.distanceRemaining

        # Animate elements when approaching maneuver
        # Safety: Gradual fade-in/out prevents sudden visual distractions
        if self._distance_to_maneuver < 200:
          self._turn_arrow_alpha = min(1.0, self._turn_arrow_alpha + 0.1)
        else:
          self._turn_arrow_alpha = max(0.0, self._turn_arrow_alpha - 0.05)

        if self._show_full:
          self._info_panel_alpha = min(1.0, self._info_panel_alpha + 0.1)
        else:
          self._info_panel_alpha = max(0.0, self._info_panel_alpha - 0.05)

  def _get_turn_texture(self, maneuver_type):
    """Get appropriate texture for turn type"""
    if not maneuver_type:
      return None

    maneuver_lower = maneuver_type.lower()
    if 'left' in maneuver_lower:
      if 'slight' in maneuver_lower:
        return self._slight_left_arrow
      return self._left_arrow
    elif 'right' in maneuver_lower:
      if 'slight' in maneuver_lower:
        return self._slight_right_arrow
      return self._right_arrow
    else:
      return self._straight_arrow

  def _truncate_text(self, text, max_width, font_size):
    """Truncate text to fit within max_width with ellipsis"""
    if not text:
      return text

    # First measure the full text
    full_size = rl.measure_text_ex(gui_app.font, text, font_size, 2)
    if full_size.x <= max_width:
      return text

    # If too long, truncate and add ellipsis
    truncated_text = text
    ellipsis = "..."
    for i in range(len(text), 0, -1):
      candidate = text[:i] + ellipsis
      candidate_size = rl.measure_text_ex(gui_app.font, candidate, font_size, 2)
      if candidate_size.x <= max_width:
        truncated_text = candidate
        break

    return truncated_text

  def _render(self, rect):
    """Render navigation elements on top of road view"""
    # Update navigation data
    self.update()

    # Draw turn arrow if available
    # Safety: Only render when alpha > 0.01 to prevent rendering of nearly invisible elements
    if self._nav_instruction and self._turn_arrow_alpha > 0.01:
      self._draw_turn_arrow(rect)

    # Draw navigation info panel
    self._draw_info_panel(rect)

  def _draw_turn_arrow(self, rect):
    """Draw turn arrow with animation"""
    if not self._nav_instruction:
      return

    # Get appropriate texture
    texture = self._get_turn_texture(self._nav_instruction.maneuverType)
    if not texture:
      return

    # Calculate position (top center of screen)
    # Safety: Positioned at top center to avoid overlap with critical driving area
    arrow_x = rect.x + rect.width / 2 - texture.width / 2
    arrow_y = rect.y + self.NAV_ARROW_Y_OFFSET

    # Apply alpha for smooth fading
    color = rl.Color(255, 255, 255, int(255 * self._turn_arrow_alpha))

    # Draw arrow
    rl.draw_texture_ex(texture, rl.Vector2(arrow_x, arrow_y), 0.0, 1.0, color)

  def _draw_info_panel(self, rect):
    """Draw navigation information panel"""
    # Safety: Only render when alpha > 0.01 to prevent rendering nearly invisible elements
    if self._info_panel_alpha < 0.01:
      return

    # Panel dimensions using constants
    panel_width = self.INFO_PANEL_WIDTH
    panel_height = self.INFO_PANEL_HEIGHT
    # Safety: Positioned at top-right with margins to avoid critical driving area
    panel_x = rect.x + rect.width - panel_width - self.INFO_PANEL_X_MARGIN
    panel_y = rect.y + self.INFO_PANEL_Y_MARGIN

    # Panel background with alpha - increased for better contrast in bright conditions
    panel_color = rl.Color(0, 0, 0, int(self.INFO_PANEL_BG_ALPHA * self._info_panel_alpha))
    rl.draw_rectangle(int(panel_x), int(panel_y), int(panel_width), int(panel_height), panel_color)

    # Draw distance and ETA text
    if self._nav_instruction:
      max_text_width = panel_width - 20  # Account for left/right padding
      distance_size = rl.Vector2(0, 0)  # Initialize to prevent reference before assignment

      # Distance text with truncation to prevent overflow
      if hasattr(self._nav_instruction, 'maneuverPrimaryText') and self._nav_instruction.maneuverPrimaryText:
        primary_text = self._nav_instruction.maneuverPrimaryText
        distance_text = self._truncate_text(primary_text, max_text_width, self.DISTANCE_FONT_SIZE)
        distance_size = rl.measure_text_ex(gui_app.font, distance_text, self.DISTANCE_FONT_SIZE, self.TEXT_LINE_SPACING)
        distance_color = rl.Color(255, 255, 255, int(255 * self._info_panel_alpha))
        rl.draw_text_ex(gui_app.font, distance_text,
                       rl.Vector2(panel_x + 10, panel_y + 10),
                       self.DISTANCE_FONT_SIZE, self.TEXT_LINE_SPACING, distance_color)

      # Secondary text with truncation
      if (hasattr(self._nav_instruction, 'maneuverSecondaryText') and
          self._nav_instruction.maneuverSecondaryText):
        secondary_text = self._truncate_text(self._nav_instruction.maneuverSecondaryText, max_text_width, self.SECONDARY_FONT_SIZE)
        secondary_size = rl.measure_text_ex(gui_app.font, secondary_text, self.SECONDARY_FONT_SIZE, self.TEXT_LINE_SPACING_SECONDARY)
        secondary_color = rl.Color(200, 200, 200, int(255 * self._info_panel_alpha))
        rl.draw_text_ex(gui_app.font, secondary_text,
                       rl.Vector2(panel_x + 10, panel_y + 10 + distance_size.y + 5),
                       self.SECONDARY_FONT_SIZE, self.TEXT_LINE_SPACING_SECONDARY, secondary_color)

      # Distance to maneuver
      dist_maneuver_text = f"{self._distance_to_maneuver:.0f}m"
      dist_maneuver_size = rl.measure_text_ex(gui_app.font, dist_maneuver_text, self.SECONDARY_FONT_SIZE, self.TEXT_LINE_SPACING_SECONDARY)
      dist_maneuver_color = rl.Color(255, 255, 255, int(255 * self._info_panel_alpha))
      rl.draw_text_ex(gui_app.font, dist_maneuver_text,
                     rl.Vector2(panel_x + panel_width - dist_maneuver_size.x - 10, panel_y + 10),
                     self.SECONDARY_FONT_SIZE, self.TEXT_LINE_SPACING_SECONDARY, dist_maneuver_color)

    # ETA
    if self._eta_seconds != float('inf'):
      eta_minutes = int(self._eta_seconds / 60)
      eta_text = f"ETA: {eta_minutes} min"
      eta_size = rl.measure_text_ex(gui_app.font, eta_text, self.ETA_FONT_SIZE, self.TEXT_LINE_SPACING_SECONDARY)
      eta_color = rl.Color(100, 255, 100, int(255 * self._info_panel_alpha))
      rl.draw_text_ex(gui_app.font, eta_text,
                     rl.Vector2(panel_x + panel_width - eta_size.x - 10, panel_y + 35),
                     self.ETA_FONT_SIZE, self.TEXT_LINE_SPACING_SECONDARY, eta_color)

  def draw_route_path(self, model_points, rect, color=rl.Color(0, 255, 0, 100)):
    """
    Draw the route path on the model view.
    Note: This method is currently not used in the main render loop but available for future enhancements.
    Reason for disabling: Currently disabled pending performance optimization of route path rendering to avoid
    potential frame drops during navigation. Will be re-enabled once rendering performance is optimized.
    Safety: Only draws within screen bounds to prevent rendering outside visible area.
    """
    if not model_points or len(model_points.projected_points) < 2:
      return

    # Draw route path on top of the model
    points = model_points.projected_points
    for i in range(len(points) - 1):
      p1 = points[i]
      p2 = points[i + 1]

      # Only draw if points are within screen bounds
      if (rect.x <= p1[0] <= rect.x + rect.width and
          rect.y <= p1[1] <= rect.y + rect.height and
          rect.x <= p2[0] <= rect.x + rect.width and
          rect.y <= p2[1] <= rect.y + rect.height):
        rl.draw_line_v(
          rl.Vector2(p1[0], p1[1]),
          rl.Vector2(p2[0], p2[1]),
          color
        )