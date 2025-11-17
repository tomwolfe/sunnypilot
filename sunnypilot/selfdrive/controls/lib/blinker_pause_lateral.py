"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from typing import Union

from cereal import car

from openpilot.common.constants import CV
from openpilot.common.params import Params


class BlinkerPauseLateral:
  def __init__(self):
    self.params: Params = Params()

    self.enabled: bool = self.params.get_bool("BlinkerPauseLateralControl")
    self.is_metric: bool = self.params.get_bool("IsMetric")
    self.min_speed: int | float = 0
    self.blinker_timer = 0

  BL_DEBOUNCE_FRAMES = 20 # 1 second at 20Hz

  def get_params(self) -> None:
    self.enabled = self.params.get_bool("BlinkerPauseLateralControl")
    self.is_metric = self.params.get_bool("IsMetric")
    min_speed_param = self.params.get("BlinkerMinLateralControlSpeed")
    if min_speed_param is not None:
      try:
        self.min_speed = float(min_speed_param.decode('utf-8'))
      except (ValueError, AttributeError):
        self.min_speed = 20  # default value
    else:
      self.min_speed = 20  # default value

  def update(self, CS: car.CarState) -> bool:
    if not self.enabled:
      self.blinker_timer = 0
      return False

    one_blinker = CS.leftBlinker != CS.rightBlinker
    speed_factor = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS
    min_speed_ms = self.min_speed * speed_factor

    if one_blinker:
      self.blinker_timer += 1
    else:
      self.blinker_timer = 0

    return bool(self.blinker_timer >= self.BL_DEBOUNCE_FRAMES and CS.vEgo < min_speed_ms)
