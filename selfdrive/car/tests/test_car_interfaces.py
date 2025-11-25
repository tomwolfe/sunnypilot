import os
import hypothesis.strategies as st
from hypothesis import Phase, given, settings
from parameterized import parameterized
from unittest.mock import patch

from cereal import car, custom
from opendbc.car import DT_CTRL
from opendbc.car.structs import CarParams
from opendbc.car.tests.test_car_interfaces import get_fuzzy_car_interface
from opendbc.car.mock.values import CAR as MOCK
from opendbc.car.values import PLATFORMS
from openpilot.common.params import Params
from openpilot.selfdrive.car.helpers import convert_carControlSP
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.test.fuzzy_generation import FuzzyGenerator

from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces

MAX_EXAMPLES = int(os.environ.get('MAX_EXAMPLES', '60'))


def mock_params_get(key, block=False):
  """
  Mock function for Params.get that returns default values for specific parameters
  needed by LongControl and LatControlTorque, and handles missing keys properly.
  """
  # Define default values for the parameters that cause test failures
  param_defaults = {
    "LongitudinalMaxJerk": "2.2",
    "LongitudinalMaxStoppingJerk": "1.5",
    "LongitudinalMaxOutputJerk": "2.0",
    "LongitudinalStartingSpeedThreshold": "3.0",
    "LongitudinalStartingAccelMultiplier": "0.8",
    "LongitudinalStartingAccelLimit": "0.8",
    "LongitudinalAdaptiveErrorThreshold": "0.6",
    "LongitudinalAdaptiveSpeedThreshold": "5.0",
    "LateralMaxJerk": "5.0",  # Default value as defined in latcontrol_torque.py
    "LateralHighSpeedThreshold": "15.0",
    "LateralHighSpeedKiLimit": "0.15",
    "LateralCurvatureKiScaler": "0.2"
  }

  if key in param_defaults:
    return param_defaults[key].encode()  # Params.get returns bytes
  else:
    # For other keys, return empty bytes b"" which is falsy and will trigger the 'or' default
    return b""


@patch.object(Params, 'get', side_effect=mock_params_get)
class TestCarInterfaces:
  # FIXME: Due to the lists used in carParams, Phase.target is very slow and will cause
  #  many generated examples to overrun when max_examples > ~20, don't use it
  @parameterized.expand([(car,) for car in sorted(PLATFORMS)] + [MOCK.MOCK])
  @settings(max_examples=MAX_EXAMPLES, deadline=None,
            phases=(Phase.reuse, Phase.generate, Phase.shrink))
  @given(data=st.data())
  def test_car_interfaces(self, car_name, data):
    car_interface = get_fuzzy_car_interface(car_name, data.draw)
    car_params = car_interface.CP.as_reader()
    car_params_sp = car_interface.CP_SP
    sunnypilot_interfaces.setup_interfaces(car_interface)

    cc_msg = FuzzyGenerator.get_random_msg(data.draw, car.CarControl, real_floats=True)
    cc_sp_msg = FuzzyGenerator.get_random_msg(data.draw, custom.CarControlSP, real_floats=True)
    # Run car interface
    now_nanos = 0
    CC = car.CarControl.new_message(**cc_msg)
    CC = CC.as_reader()
    CC_SP = custom.CarControlSP.new_message(**cc_sp_msg)
    CC_SP = convert_carControlSP(CC_SP.as_reader())
    for _ in range(10):
      car_interface.update([])
      car_interface.apply(CC, CC_SP, now_nanos)
      now_nanos += DT_CTRL * 1e9  # 10 ms

    CC = car.CarControl.new_message(**cc_msg)
    CC.enabled = True
    CC.latActive = True
    CC.longActive = True
    CC = CC.as_reader()
    for _ in range(10):
      car_interface.update([])
      car_interface.apply(CC, CC_SP, now_nanos)
      now_nanos += DT_CTRL * 1e9  # 10ms

    # Test controller initialization
    # TODO: wait until card refactor is merged to run controller a few times,
    #  hypothesis also slows down significantly with just one more message draw
    LongControl(car_params, car_params_sp)
    if car_params.steerControlType == CarParams.SteerControlType.angle:
      LatControlAngle(car_params, car_params_sp, car_interface, DT_CTRL)
    elif car_params.lateralTuning.which() == 'pid':
      LatControlPID(car_params, car_params_sp, car_interface, DT_CTRL)
    elif car_params.lateralTuning.which() == 'torque':
      LatControlTorque(car_params, car_params_sp, car_interface, DT_CTRL)
