from parameterized import parameterized
from unittest.mock import patch

from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.hyundai.values import CAR as HYUNDAI
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.helpers import convert_to_capnp
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces


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
class TestNNTorqueModel:

  @parameterized.expand([HONDA.HONDA_CIVIC, TOYOTA.TOYOTA_RAV4, HYUNDAI.HYUNDAI_SANTA_CRUZ_1ST_GEN])
  def test_load_model(self, car_name):
    params = Params()
    params.put_bool("NeuralNetworkLateralControl", True)

    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
    CI = CarInterface(CP, CP_SP)

    sunnypilot_interfaces.setup_interfaces(CI, params)

    CP_SP = convert_to_capnp(CP_SP)

    controller = LatControlTorque(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

    assert controller.extension.has_nn_model
