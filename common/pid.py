import numpy as np
from numbers import Number


class PIDController:
  def __init__(self, k_p, k_i, k_d=0.0, pos_limit=1e308, neg_limit=-1e308, rate=100):
    self._k_p = k_p
    self._k_i = k_i
    self._k_d = k_d
    if isinstance(self._k_p, Number):
      self._k_p = [[0], [self._k_p]]
    if isinstance(self._k_i, Number):
      self._k_i = [[0], [self._k_i]]
    if isinstance(self._k_d, Number):
      self._k_d = [[0], [self._k_d]]

    self.set_limits(pos_limit, neg_limit)

    self.i_dt = 1.0 / rate
    self.speed = 0.0

    self.reset()

  @property
  def k_p(self):
    return np.interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return np.interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return np.interp(self.speed, self._k_d[0], self._k_d[1])

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0

  def set_limits(self, pos_limit, neg_limit):
    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

  def update(self, error, error_rate=0.0, speed=0.0, feedforward=0.0, freeze_integrator=False, k_p_override=None, k_i_override=None, k_d_override=None):
    self.speed = speed

    # Use override values if provided, otherwise use interpolated values
    current_k_p = k_p_override if k_p_override is not None else self.k_p
    current_k_i = k_i_override if k_i_override is not None else self.k_i
    current_k_d = k_d_override if k_d_override is not None else self.k_d

    self.p = current_k_p * float(error)
    self.d = current_k_d * error_rate
    self.f = feedforward

    if not freeze_integrator:
      i = self.i + current_k_i * self.i_dt * error

      # Don't allow windup if already clipping
      test_control = self.p + i + self.d + self.f
      i_upperbound = self.i if test_control > self.pos_limit else self.pos_limit
      i_lowerbound = self.i if test_control < self.neg_limit else self.neg_limit
      self.i = np.clip(i, i_lowerbound, i_upperbound)

    control = self.p + self.i + self.d + self.f
    self.control = np.clip(control, self.neg_limit, self.pos_limit)
    return self.control
