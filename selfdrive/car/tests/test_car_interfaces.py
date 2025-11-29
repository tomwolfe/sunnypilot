import os
import hypothesis.strategies as st
from hypothesis import Phase, given, settings
from parameterized import parameterized


from cereal import car, custom
from openpilot.selfdrive.car.helpers import convert_carControlSP
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.test.fuzzy_generation import FuzzyGenerator

from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces

MAX_EXAMPLES = int(os.environ.get('MAX_EXAMPLES', '60'))


class TestCarInterfaces:
  # FIXME: Due to the lists used in carParams, Phase.target is very slow and will cause
  #  many generated examples to overrun when max_examples > ~20, don't use it
  @parameterized.expand([(car,) for car in sorted(PLATFORMS)] + [MOCK.MOCK])
  @settings(max_examples=MAX_EXAMPLES, deadline=None,
            phases=(Phase.reuse, Phase.generate, Phase.shrink))
  @given(data=st.data())
  def test_car_interfaces(self, car_name, data, mocker):
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

    # Create a mock params object to satisfy the interface expected by the controllers
    mock_params = mocker.Mock()
    mock_params.get = mocker.Mock(return_value=None)  # Default to return None for any parameter request

    LongControl(car_params, car_params_sp, mock_params)
    if car_params.steerControlType == CarParams.SteerControlType.angle:
      LatControlAngle(car_params, car_params_sp, car_interface, DT_CTRL)
    elif car_params.lateralTuning.which() == 'pid':
      LatControlPID(car_params, car_params_sp, car_interface, DT_CTRL, mock_params)
    elif car_params.lateralTuning.which() == 'torque':
      LatControlTorque(car_params, car_params_sp, car_interface, DT_CTRL, mock_params)
