"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets.list_view import ListItem, ItemAction


class ProgressBarAction(ItemAction):
  def __init__(self, width=600):
    super().__init__(width=width)
    self.progress = 0.0
    self.main_text = ""
    self.progress_text = ""
    self.show_progress = False
    self.text_color = rl.GRAY
    self._font = gui_app.font(FontWeight.NORMAL)

  def update(self, progress, main_text, progress_text="", show_progress=False, text_color=rl.GRAY):
    self.progress = progress
    self.main_text = main_text
    self.progress_text = progress_text
    self.show_progress = show_progress
    self.text_color = text_color

  def _render(self, rect: rl.Rectangle):
    font_size = 40
    # Calculate width of main text and progress text separately
    main_text_size = measure_text_cached(self._font, self.main_text, font_size)
    progress_text_size = measure_text_cached(self._font, self.progress_text, font_size)

    padding = 30
    text_spacing = 10 # Space between main_text and progress_text

    # Calculate total width of the text content
    total_text_width = main_text_size.x
    if self.show_progress and self.progress_text:
      total_text_width += progress_text_size.x + text_spacing

    bar_width = total_text_width + 2 * padding
    bar_height = 60
    bar_rect = rl.Rectangle(rect.x + rect.width - bar_width, rect.y + (rect.height - bar_height) / 2, bar_width, bar_height)

    if self.show_progress:
      inner_rect = rl.Rectangle(bar_rect.x + 4, bar_rect.y + 4, bar_rect.width - 8, bar_rect.height - 8)
      if inner_rect.width > 0:
        fill_width = max(0, min(inner_rect.width, inner_rect.width * (self.progress / 100.0)))
        rl.draw_rectangle_rounded(rl.Rectangle(inner_rect.x, inner_rect.y, fill_width, inner_rect.height), 0.2, 10, rl.Color(30, 121, 232, 255))

    # Render main text
    main_text_x = bar_rect.x + padding
    main_text_y = bar_rect.y + (bar_height - main_text_size.y) / 2
    rl.draw_text_ex(self._font, self.main_text, rl.Vector2(main_text_x, main_text_y), font_size, 0, self.text_color)

    # Render progress text if showing progress
    if self.show_progress and self.progress_text:
      progress_text_x = main_text_x + main_text_size.x + text_spacing
      progress_text_y = bar_rect.y + (bar_height - progress_text_size.y) / 2
      rl.draw_text_ex(self._font, self.progress_text, rl.Vector2(progress_text_x, progress_text_y), font_size, 0, self.text_color)




def progress_item(title):
  action = ProgressBarAction()
  return ListItem(title=title, action_item=action)