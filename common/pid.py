import numpy as np
from numbers import Number
from openpilot.common.performance_monitor import PerfTrack

class PIDController:
  def __init__(self, k_p, k_i, k_f=0., k_d=0., pos_limit=1e308, neg_limit=-1e308, rate=100):
    self._k_p = k_p
    self._k_i = k_i
    self._k_d = k_d
    self.k_f = k_f   # feedforward gain
    if isinstance(self._k_p, Number):
      self._k_p = [[0], [self._k_p]]
    if isinstance(self._k_i, Number):
      self._k_i = [[0], [self._k_i]]
    if isinstance(self._k_d, Number):
      self._k_d = [[0], [self._k_d]]

    self.set_limits(pos_limit, neg_limit)

    self.i_rate = 1.0 / rate
    self.speed = 0.0

    # Pre-compute interpolation if using fixed arrays (for optimization)
    if not isinstance(self._k_p, Number) and len(self._k_p[0]) > 1:
      self._use_interp = True
      self._k_p_array = np.array(self._k_p)
      self._k_i_array = np.array(self._k_i)
      self._k_d_array = np.array(self._k_d)
      # Pre-sample common speed values to avoid interpolation during runtime
      self._common_speeds = np.linspace(0, max(self._k_p[0]), 100)
      self._k_p_interp = np.interp(self._common_speeds, self._k_p[0], self._k_p[1])
      self._k_i_interp = np.interp(self._common_speeds, self._k_i[0], self._k_i[1])
      self._k_d_interp = np.interp(self._common_speeds, self._k_d[0], self._k_d[1])
    else:
      self._use_interp = False

    self.reset()

  def _get_k_p(self):
    if self._use_interp:
      # Fast lookup for common speeds
      speed_idx = int(self.speed * (len(self._common_speeds) - 1) / max(self._k_p[0]) if max(self._k_p[0]) > 0 else 0)
      speed_idx = min(speed_idx, len(self._common_speeds) - 1)
      if abs(self.speed - self._common_speeds[speed_idx]) < 0.5:  # Use lookup if speed is close
        return self._k_p_interp[speed_idx]
      else:
        return np.interp(self.speed, self._k_p[0], self._k_p[1])
    else:
      return self._k_p[1][1] if hasattr(self._k_p[1], '__len__') else self._k_p

  def _get_k_i(self):
    if self._use_interp:
      speed_idx = int(self.speed * (len(self._common_speeds) - 1) / max(self._k_i[0]) if max(self._k_i[0]) > 0 else 0)
      speed_idx = min(speed_idx, len(self._common_speeds) - 1)
      if abs(self.speed - self._common_speeds[speed_idx]) < 0.5:
        return self._k_i_interp[speed_idx]
      else:
        return np.interp(self.speed, self._k_i[0], self._k_i[1])
    else:
      return self._k_i[1][1] if hasattr(self._k_i[1], '__len__') else self._k_i

  def _get_k_d(self):
    if self._use_interp:
      speed_idx = int(self.speed * (len(self._common_speeds) - 1) / max(self._k_d[0]) if max(self._k_d[0]) > 0 else 0)
      speed_idx = min(speed_idx, len(self._common_speeds) - 1)
      if abs(self.speed - self._common_speeds[speed_idx]) < 0.5:
        return self._k_d_interp[speed_idx]
      else:
        return np.interp(self.speed, self._k_d[0], self._k_d[1])
    else:
      return self._k_d[1][1] if hasattr(self._k_d[1], '__len__') else self._k_d

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0

  def set_limits(self, pos_limit, neg_limit):
    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

  def update(self, error, error_rate=0.0, speed=0.0, feedforward=0., freeze_integrator=False):
    with PerfTrack("pid_update"):
      self.speed = speed
      self.p = error * self._get_k_p()  # Removed float() wrapper for slight performance improvement
      self.f = feedforward * self.k_f
      self.d = error_rate * self._get_k_d()

      if not freeze_integrator:
        i = self.i + error * self._get_k_i() * self.i_rate

        # Don't allow windup if already clipping
        test_control = self.p + i + self.d + self.f
        i_upperbound = self.i if test_control > self.pos_limit else self.pos_limit
        i_lowerbound = self.i if test_control < self.neg_limit else self.neg_limit
        self.i = np.clip(i, i_lowerbound, i_upperbound)

      control = self.p + self.i + self.d + self.f
      self.control = np.clip(control, self.neg_limit, self.pos_limit)
      return self.control
