import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController

class MockLeadOne:
  def __init__(self, status=0.0):
    self.status = status

class MockRadarState:
  def __init__(self, status=0.0):
    self.leadOne = MockLeadOne(status=status)

class MockCarState:
  def __init__(self, vEgo=0.0, vCruise=0.0, standstill=False):
    self.vEgo = vEgo
    self.vCruise = vCruise
    self.standstill = standstill

class MockModelData:
  def __init__(self, valid=True):
    size = 33 if valid else 10  # incomplete if invalid
    self.position = type("Pos", (), {"x": [0.0] * size})()
    self.orientation = type("Ori", (), {"x": [0.0] * size})()

class MockSelfDriveState:
  def __init__(self, experimentalMode=False):
    self.experimentalMode = experimentalMode

class MockLongitudinalPlanSP:
  def __init__(self):
    self.dec = type("Dec", (), {"mae": 0.0, "uncertaintyOffset": 0.0})()

class MockParams:
  def get_bool(self, name):
    return True

@pytest.fixture
def default_sm():
  sm = {
    'carState': MockCarState(vEgo=10.0, vCruise=20.0),
    'radarState': MockRadarState(status=1.0),
    'modelV2': MockModelData(valid=True),
    'selfdriveState': MockSelfDriveState(experimentalMode=True),
    'longitudinalPlanSP': MockLongitudinalPlanSP(),
  }
  # Add updated dict tracking for SubMaster mock
  sm_obj = type("SubMaster", (), {
    "__getitem__": lambda self, k: sm[k],
    "updated": {"longitudinalPlanSP": True},
    "all_checks": lambda self, **kwargs: True
  })()
  return sm_obj

@pytest.fixture
def mock_cp():
  class CP:
    radarUnavailable = False
  return CP()

@pytest.fixture
def mock_mpc():
  class MPC:
    crash_cnt = 0
  return MPC()

# Fake Kalman Filter that always returns a given value
class FakeKalman:
  def __init__(self, value=1.0):
    self.value = value
  def add_data(self, v): pass
  def get_value(self): return self.value
  def get_confidence(self): return 1.0
  def reset_data(self): pass

def test_initial_mode_is_acc(mock_cp, mock_mpc):
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())
  assert controller.mode() == "acc"
  assert controller.blended_weight() == 0.0

def test_standstill_triggers_blended(mock_cp, mock_mpc, default_sm):
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())
  default_sm['carState'].standstill = True
  for _ in range(10):
    controller.update(default_sm)
  assert controller.mode() == "blended"
  assert controller.blended_weight() > 0.5  # Weight should be trending toward 1.0

def test_emergency_blended_on_fcw(mock_cp, mock_mpc, default_sm):
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())
  mock_mpc.crash_cnt = 1  # simulate FCW
  for _ in range(2):
    controller.update(default_sm)
  assert controller.mode() == "blended"
  assert controller.blended_weight() > 0.0 # Starts trending immediately

def test_calibration_update(mock_cp, mock_mpc, default_sm):
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())
  default_sm['longitudinalPlanSP'].dec.mae = 0.5
  default_sm['longitudinalPlanSP'].dec.uncertaintyOffset = 1.0
  controller.update(default_sm)
  assert controller.calibration_mae() == 0.5
  assert controller.calibration_uncertainty_offset() == 1.0

def test_radarless_slowdown_triggers_blended(mock_cp, mock_mpc, default_sm):
  mock_cp.radarUnavailable = True
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())

  # Force conditions to simulate slowdown
  controller._slow_down_filter = FakeKalman(value=1.0)  # Ensure urgency triggers slowdown
  controller._v_ego_kph = 35.0
  # Mock model data with attributes that _calculate_slow_down expects
  md = MockModelData(valid=False)
  md.meta = type("Meta", (), {"hardBrakePredicted": False, "disengagePredictions": type("D", (), {"brakeDisengageProbs": [0.0]})(), "engagedProb": 1.0})()
  md.position.xStd = [0.0] * 33
  md.velocity = type("Vel", (), {"x": [0.0]})()
  md.orientationRate = type("OriR", (), {"z": [0.0]})()
  default_sm['modelV2'] = md

  for _ in range(3):
    controller.update(default_sm)

  assert controller.mode() == "blended"
  assert controller.blended_weight() > 0.0
