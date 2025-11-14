"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from json import load
import numpy as np
from tinygrad.tensor import Tensor

from openpilot.selfdrive.modeld.parse_model_outputs import safe_exp

# dict used to rename activation functions whose names aren't valid python identifiers
ACTIVATION_FUNCTION_NAMES = {'σ': 'sigmoid'}


class NNTorqueModelTinygrad:
  def __init__(self, params_file, zero_bias=False):
    with open(params_file) as f:
      params = load(f)

    self.input_size = params["input_size"]
    self.output_size = params["output_size"]
    self.input_mean = Tensor(np.array(params["input_mean"], dtype=np.float32).T)
    self.input_std = Tensor(np.array(params["input_std"], dtype=np.float32).T)
    self.layers = []
    self.friction_override = False

    for layer_params in params["layers"]:
      W = Tensor(np.array(layer_params[next(key for key in layer_params.keys() if key.endswith('_W'))], dtype=np.float32).T)
      b = Tensor(np.array(layer_params[next(key for key in layer_params.keys() if key.endswith('_b'))], dtype=np.float32).T)
      if zero_bias:
        b = Tensor.zeros_like(b)
      activation = layer_params["activation"]
      for k, v in ACTIVATION_FUNCTION_NAMES.items():
        activation = activation.replace(k, v)
      self.layers.append((W, b, activation))

    self.validate_layers()
    self.check_for_friction_override()

  # Begin activation functions.
  # These are called by name using the keys in the model json file
  @staticmethod
  def sigmoid(x: Tensor) -> Tensor:
    return 1 / (1 + safe_exp(-x))

  @staticmethod
  def identity(x: Tensor) -> Tensor:
    return x
  # End activation functions

  def forward(self, x: Tensor) -> Tensor:
    for W, b, activation in self.layers:
      x = getattr(self, activation)(x.dot(W) + b)
    return x

  def evaluate(self, input_array) -> float:
    in_len = len(input_array)
    if in_len != self.input_size:
      # If the input is length 2-4, then it's a simplified evaluation.
      # In that case, need to add on zeros to fill out the input array to match the correct length.
      if 2 <= in_len:
        input_array = input_array + [0] * (self.input_size - in_len)
      else:
        raise ValueError(f"Input array length {len(input_array)} must be length 2 or greater")

    input_tensor = Tensor(np.array(input_array, dtype=np.float32))

    # Rescale the input array using the input_mean and input_std
    input_tensor = (input_tensor - self.input_mean) / self.input_std

    output_tensor = self.forward(input_tensor)

    return float(output_tensor.numpy()[0, 0])

  def validate_layers(self):
    for _, _, activation in self.layers:
      if not hasattr(self, activation):
        raise ValueError(f"Unknown activation: {activation}")

  def check_for_friction_override(self):
    # Use a dummy input for checking friction override
    dummy_input = [10.0, 0.0, 0.2] + [0.0] * (self.input_size - 3)
    y = self.evaluate(dummy_input)
    self.friction_override = (y < 0.1)

