"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl
from collections.abc import Callable

from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.list_view import ListItem, Scroller
from openpilot.system.ui.widgets.widget import Widget


class SelectionDialogSP(Widget):
  def __init__(self, title: str, items: list[str], current_value: str | None = None,
               callback: Callable[[DialogResult, str | None], None] | None = None):
    super().__init__(None) # No parent for modal overlay
    self.title_text = title
    self.callback = callback
    self.selected_item = None
    self.dialog_result = DialogResult.NONE

    list_items = []
    for item_text in items:
      is_checked = (item_text == current_value)
      list_items.append(ListItem(item_text, selectable=True, checked=is_checked,
                                 callback=lambda text=item_text: self._on_item_selected(text)))

    self._scroller = Scroller(list_items, line_separator=True, spacing=0)

  def _on_item_selected(self, item_text: str):
    self.selected_item = item_text
    self.dialog_result = DialogResult.CONFIRM
    gui_app.close_modal_overlay() # Close the dialog on selection

  def _on_cancel(self):
    self.selected_item = None
    self.dialog_result = DialogResult.CANCEL
    gui_app.close_modal_overlay()

  def _render(self, rect: rl.Rectangle):
    # Render title
    title_height = 80
    title_rect = rl.Rectangle(rect.x, rect.y, rect.width, title_height)
    rl.draw_rectangle_rec(title_rect, rl.BLACK)
    rl.draw_text_aligned(tr(self.title_text), title_rect, rl.ALIGN_CENTER, rl.ALIGN_CENTER, 36, rl.WHITE)

    # Render scroller for items
    scroller_rect = rl.Rectangle(rect.x, rect.y + title_height, rect.width, rect.height - title_height)
    self._scroller.render(scroller_rect)

  def show(self):
    def internal_callback(result: DialogResult):
      if self.callback:
        self.callback(self.dialog_result, self.selected_item)

    gui_app.set_modal_overlay(self, internal_callback)
