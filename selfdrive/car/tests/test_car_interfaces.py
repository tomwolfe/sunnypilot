import os
import hypothesis.strategies as st
from hypothesis import Phase, given, settings
from parameterized import parameterized
from typing import Any
from collections.abc import Callable

from cereal import car, custom
from opendbc.car import DT_CTRL, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.fingerprints import FW_VERSIONS
from opendbc.car.fw_versions import FW_QUERY_CONFIGS
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mock.values import CAR as MOCK
from opendbc.car.values import PLATFORMS
from opendbc.car.structs import CarParams

from openpilot.selfdrive.car.helpers import convert_carControlSP
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.test.fuzzy_generation import FuzzyGenerator

from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces

MAX_EXAMPLES = int(os.environ.get('MAX_EXAMPLES', '60'))

# From panda/python/__init__.py
DLC_TO_LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]
DrawType = Callable[[st.SearchStrategy], Any]

ALL_ECUS = {ecu for ecus in FW_VERSIONS.values() for ecu in ecus.keys()}
ALL_ECUS |= {ecu for config in FW_QUERY_CONFIGS.values() for ecu in config.extra_ecus}

ALL_REQUESTS = {tuple(r.request) for config in FW_QUERY_CONFIGS.values() for r in config.requests}


def get_fuzzy_car_interface(car_name: str, draw: DrawType) -> CarInterfaceBase:
  # Fuzzy CAN fingerprints and FW versions to test more states of the CarInterface
  fingerprint_strategy = st.fixed_dictionaries({0: st.dictionaries(st.integers(min_value=0, max_value=0x800),
                                                                   st.sampled_from(DLC_TO_LEN))})

  # only pick from possible ecus to reduce search space
  car_fw_strategy = st.lists(st.builds(
    lambda fw, req: structs.CarParams.CarFw(ecu=fw[0], address=fw[1], subAddress=fw[2] or 0, request=req),
    st.sampled_from(sorted(ALL_ECUS)),
    st.sampled_from(sorted(ALL_REQUESTS)),
  ))

  params_strategy = st.fixed_dictionaries({
    'fingerprints': fingerprint_strategy,
    'car_fw': car_fw_strategy,
    'alpha_long': st.booleans(),
  })

  params: dict = draw(params_strategy)
  # reduce search space by duplicating CAN fingerprints across all buses
  params['fingerprints'] |= {key + 1: params['fingerprints'][0] for key in range(6)}

  # initialize car interface
  CarInterface = interfaces[car_name]
  car_params = CarInterface.get_params(car_name, params['fingerprints'], params['car_fw'],
                                       alpha_long=params['alpha_long'], is_release=False, docs=False)
  car_params_sp = CarInterface.get_params_sp(car_params, car_name, params['fingerprints'], params['car_fw'],
                                             alpha_long=params['alpha_long'], is_release_sp=False, docs=False)
  result: CarInterfaceBase = CarInterface(car_params, car_params_sp)
  return result


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
