"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from typing import Any, TYPE_CHECKING

from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.nnlc import NeuralNetworkLateralControl
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override import LatControlTorqueExtOverride

if TYPE_CHECKING:
  from cereal import car, custom
  from opendbc.car import structs
  from opendbc.car.lateral_torque import LatControlTorque
  from opendbc.car.interfaces import CarInterfaceBase
  from openpilot.selfdrive.modeld.constants import ModelConstants


class LatControlTorqueExt(NeuralNetworkLateralControl, LatControlTorqueExtOverride):
  """
  Enhanced lateral torque controller that integrates neural network feedforward control
  with traditional PID control for improved steering performance and safety.

  This class extends both NeuralNetworkLateralControl and LatControlTorqueExtOverride
  to provide advanced steering control with machine learning-based enhancements.
  """

  def __init__(self, lac_torque: 'LatControlTorque', CP: 'structs.CarParams', CP_SP: 'custom.CarParamsSP', CI: 'CarInterfaceBase'):
    """
    Initialize the enhanced lateral torque controller.

    Args:
      lac_torque: Base lateral control torque instance
      CP: Car parameters
      CP_SP: Sunnypilot-specific car parameters
      CI: Car interface instance
    """
    NeuralNetworkLateralControl.__init__(self, lac_torque, CP, CP_SP, CI)
    LatControlTorqueExtOverride.__init__(self, CP)

  def update(self,
             CS: 'car.CarState',
             VM: 'ModelConstants',
             pid: Any,
             params: Any,
             ff: float,
             pid_log: Any,
             setpoint: float,
             measurement: float,
             calibrated_pose: Any,
             roll_compensation: float,
             desired_lateral_accel: float,
             actual_lateral_accel: float,
             lateral_accel_deadzone: float,
             gravity_adjusted_lateral_accel: float,
             desired_curvature: float,
             actual_curvature: float,
             steer_limited_by_safety: bool,
             output_torque: float) -> tuple:
    """
    Update the lateral control state and compute output.

    This method stores internal state and delegates to neural network and calculation
    methods to compute the final control output.

    Args:
      CS: Car state
      VM: Vehicle model constants
      pid: PID controller instance
      params: Control parameters
      ff: Feedforward value
      pid_log: PID logging data
      setpoint: Control setpoint
      measurement: Current measurement
      calibrated_pose: Calibrated vehicle pose
      roll_compensation: Roll angle compensation
      desired_lateral_accel: Desired lateral acceleration
      actual_lateral_accel: Actual lateral acceleration
      lateral_accel_deadzone: Lateral acceleration deadzone
      gravity_adjusted_lateral_accel: Gravity-adjusted lateral acceleration
      desired_curvature: Desired curvature
      actual_curvature: Actual curvature
      steer_limited_by_safety: Whether steering is limited by safety
      output_torque: Current output torque

    Returns:
      Tuple of (pid_log, output_torque) for the control system
    """
    self._ff: float = ff
    self._pid = pid
    self._pid_log = pid_log
    self._setpoint: float = setpoint
    self._measurement: float = measurement
    self._roll_compensation: float = roll_compensation
    self._lateral_accel_deadzone: float = lateral_accel_deadzone
    self._desired_lateral_accel: float = desired_lateral_accel
    self._actual_lateral_accel: float = actual_lateral_accel
    self._desired_curvature: float = desired_curvature
    self._actual_curvature: float = actual_curvature
    self._gravity_adjusted_lateral_accel: float = gravity_adjusted_lateral_accel
    self._steer_limited_by_safety: bool = steer_limited_by_safety
    self._output_torque: float = output_torque

    self.update_calculations(CS, VM, desired_lateral_accel)
    self.update_neural_network_feedforward(CS, params, calibrated_pose)

    return self._pid_log, self._output_torque
