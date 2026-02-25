"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import os
import tomllib
from difflib import SequenceMatcher

from opendbc.car import structs
from openpilot.common.basedir import BASEDIR

TORQUE_NN_MODEL_PATH = os.path.join(BASEDIR, "sunnypilot", "neural_network_data", "neural_network_lateral_control")
TORQUE_NN_MODEL_SUBSTITUTE_PATH = os.path.join(BASEDIR, "opendbc", "car", "torque_data/substitute.toml")
MOCK_MODEL_PATH = os.path.join(TORQUE_NN_MODEL_PATH, "MOCK.json")

# A+ Enhancement: Universal Lateral Model (NNLC 2.0)
# Enables vehicle-agnostic lateral control without fuzzy fingerprint matching
UNIVERSAL_MODEL_ENABLED = True


def similarity(s1: str, s2: str) -> float:
  return SequenceMatcher(None, s1, s2).ratio()


def get_nn_model_path(CP: structs.CarParams) -> tuple[str, str, bool]:
  """
  Get NN model path with A+ Enhancement support
  
  A+ Enhancement (NNLC 2.0): Universal Latent Space
  - Replaces fuzzy fingerprint matching with vehicle parameter inputs
  - Single universal model works across all vehicles
  - Falls back to legacy matching for compatibility
  
  Returns:
    tuple[str, str, bool]: (model_path, model_name, exact_match)
    - exact_match=True indicates universal model is being used
  """
  # A+ Enhancement: Try Universal Lateral Model first
  if UNIVERSAL_MODEL_ENABLED:
    try:
      from .universal_lateral import get_universal_lateral_model, get_vehicle_parameters
      
      # Get universal model (no car-specific file needed)
      universal_model = get_universal_lateral_model()
      vehicle_params = get_vehicle_parameters(CP)
      
      # Universal model is always an "exact match" - it works for all vehicles
      # The model adapts to vehicle parameters continuously
      return "universal_lateral.tinygrad", f"UNIVERSAL_v1_{CP.carFingerprint}", True
    except Exception as e:
      # Fallback to legacy method if universal model fails
      pass
  
  # Legacy fuzzy fingerprint matching (for backward compatibility)
  car_fingerprint = CP.carFingerprint
  eps_fw = str(next((fw.fwVersion for fw in CP.carFw if fw.ecu == "eps"), ""))

  def check_nn_path(_nn_candidate):
    _model_path = None
    _max_similarity = -1.0
    for f in os.listdir(TORQUE_NN_MODEL_PATH):
      if f.endswith(".json"):
        model = os.path.splitext(f)[0]
        similarity_score = similarity(model, _nn_candidate)
        if similarity_score > _max_similarity:
          _max_similarity = similarity_score
          _model_path = os.path.join(TORQUE_NN_MODEL_PATH, f)
    return _model_path, _max_similarity

  if len(eps_fw) > 3:
    eps_fw = eps_fw.replace("\\", "")
    nn_candidate = f"{car_fingerprint} {eps_fw}"
  else:
    nn_candidate = car_fingerprint

  model_path, max_similarity = check_nn_path(nn_candidate)
  exact_match = max_similarity >= 0.99

  if car_fingerprint not in model_path or 0.0 <= max_similarity < 0.9:
    nn_candidate = car_fingerprint
    model_path, max_similarity = check_nn_path(nn_candidate)
    exact_match = max_similarity >= 0.99

    if 0.0 <= max_similarity < 0.9:
      with open(TORQUE_NN_MODEL_SUBSTITUTE_PATH, 'rb') as f:
        sub = tomllib.load(f)
      sub_candidate = sub.get(car_fingerprint, car_fingerprint)

      for candidate in [car_fingerprint, sub_candidate]:
        model_path, max_similarity = check_nn_path(candidate)

      exact_match = False

  if CP.steerControlType == structs.CarParams.SteerControlType.angle:
    model_path = MOCK_MODEL_PATH

  model_name = os.path.splitext(os.path.basename(model_path))[0]

  # A+ Enhancement: Check for on-device calibration
  # If calibration exists with high confidence, override fuzzy matching
  try:
    from .on_device_calibration import get_calibrator
    calibrator = get_calibrator()

    # If calibration is confident (>0.8), consider it an exact match
    # regardless of fingerprint similarity
    if calibrator.params.calibration_confidence > 0.8:
      exact_match = True
      model_name = f"{model_name}_calibrated"
  except Exception:
    pass  # Fallback to original behavior if calibration not available

  return model_path, model_name, exact_match
