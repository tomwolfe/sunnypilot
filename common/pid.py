import numpy as np
from numbers import Number
from openpilot.common.performance_monitor import PerfTrack

class PIDController:
  def __init__(self, k_p, k_i, k_f=0., k_d=0., k_curvature=None, max_curvature_gain_multiplier=4.0, pos_limit=1e308, neg_limit=-1e308, rate=100):
    self._k_p = k_p
    self._k_i = k_i
    self._k_d = k_d
    self._k_curvature = k_curvature
    self.max_curvature_gain_multiplier = max_curvature_gain_multiplier  # Configurable maximum gain multiplier
    self.k_f = k_f   # feedforward gain
    if isinstance(self._k_p, Number):
      self._k_p = [[0], [self._k_p]]
    if isinstance(self._k_i, Number):
      self._k_i = [[0], [self._k_i]]
    if isinstance(self._k_d, Number):
      self._k_d = [[0], [self._k_d]]
    if isinstance(self._k_curvature, Number):
      self._k_curvature = [[0], [self._k_curvature]]

    self.set_limits(pos_limit, neg_limit)

    self.i_rate = 1.0 / rate
    self.speed = 0.0
    self._last_desired_curvature = 0.0

    # Adaptive damping mechanism for oscillation detection
    self.oscillation_history = []  # Store error values for oscillation detection
    self.oscillation_threshold = 0.5  # Threshold for considering an error significant
    self.oscillation_window = int(rate * 0.5)  # Look at last 0.5 seconds of errors (in samples)
    self.oscillation_gain_reduction = 0.9  # Reduce gain by 10% when oscillation detected
    self.oscillation_recovery_rate = 1.005  # Gradually restore gain when no oscillations
    self.min_oscillation_gain_factor = 0.5  # Minimum gain factor (50% of computed gain)
    self.max_oscillation_gain_factor = 1.0  # Maximum gain factor (100% of computed gain)
    self.oscillation_gain_factor = 1.0  # Current gain factor
    # Enhanced oscillation detection metrics for monitoring
    self.oscillation_detected = False
    self.oscillation_damping_active = False
    self.oscillation_detection_count = 0
    self.oscillation_recovery_count = 0

    # Pre-compute interpolation if using fixed arrays (for optimization)
    if len(self._k_p[0]) > 1:
      self._use_interp = True
      self._k_p_array = np.array(self._k_p)
      self._k_i_array = np.array(self._k_i)
      self._k_d_array = np.array(self._k_d)
      if self._k_curvature is not None:
        self._k_curvature_array = np.array(self._k_curvature)
      # Pre-sample common speed values to avoid interpolation during runtime
      self._common_speeds = np.linspace(0, max(self._k_p[0]), 100)
      self._k_p_interp = np.interp(self._common_speeds, self._k_p[0], self._k_p[1])
      self._k_i_interp = np.interp(self._common_speeds, self._k_i[0], self._k_i[1])
      self._k_d_interp = np.interp(self._common_speeds, self._k_d[0], self._k_d[1])
    else:
      self._use_interp = False

    self.reset()

  def _get_k_p(self):
    k_p = self._k_p[1][0] if not self._use_interp else np.interp(self.speed, self._k_p[0], self._k_p[1])

    if self._k_curvature is not None:
      curvature_gain = np.interp(abs(self._last_desired_curvature), self._k_curvature[0], self._k_curvature[1])
      # Address the potential issue of excessive gain multiplication when both
      # speed-based and curvature-based gains are high by implementing a more
      # controlled gain application that prevents explosive behavior while maintaining
      # the essential curvature-based responsiveness
      original_k_p = k_p  # Store original for safety limiting
      k_p *= curvature_gain  # Apply the basic curvature gain

      # Implement safety bounds to prevent excessive gain growth when both factors are high
      # This is particularly important for corner cases where both speed and curvature
      # require high gains simultaneously
      max_allowable_gain = original_k_p * self.max_curvature_gain_multiplier  # Allow configurable max gain
      k_p = min(k_p, max_allowable_gain)

    # Apply adaptive damping based on oscillation detection
    k_p = k_p * self.oscillation_gain_factor

    return k_p

  def _get_k_i(self):
    if self._use_interp:
      speed_idx = int(self.speed * (len(self._common_speeds) - 1) / max(self._k_i[0]) if max(self._k_i[0]) > 0 else 0)
      speed_idx = min(speed_idx, len(self._common_speeds) - 1)
      if abs(self.speed - self._common_speeds[speed_idx]) < 0.5:
        return self._k_i_interp[speed_idx]
      else:
        return np.interp(self.speed, self._k_i[0], self._k_i[1])
    else:
      # If _k_i[1] is a list/array, get the first element; otherwise return _k_i directly
      return self._k_i[1][0] if hasattr(self._k_i[1], '__len__') and len(self._k_i[1]) > 0 else self._k_i

  def _get_k_d(self):
    if self._use_interp:
      speed_idx = int(self.speed * (len(self._common_speeds) - 1) / max(self._k_d[0]) if max(self._k_d[0]) > 0 else 0)
      speed_idx = min(speed_idx, len(self._common_speeds) - 1)
      if abs(self.speed - self._common_speeds[speed_idx]) < 0.5:
        return self._k_d_interp[speed_idx]
      else:
        return np.interp(self.speed, self._k_d[0], self._k_d[1])
    else:
      # If _k_d[1] is a list/array, get the first element; otherwise return _k_d directly
      return self._k_d[1][0] if hasattr(self._k_d[1], '__len__') and len(self._k_d[1]) > 0 else self._k_d

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0

  def set_limits(self, pos_limit, neg_limit):
    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

  def update(self, error, error_rate=0.0, speed=0.0, curvature=0.0, feedforward=0., freeze_integrator=False):
    with PerfTrack("pid_update"):
      self.speed = speed
      # Handle NaN curvature values by setting to 0 to prevent NaN propagation
      self._last_desired_curvature = curvature if not np.isnan(curvature) else 0.0

      # Store current error for oscillation detection
      self.oscillation_history.append(error)
      # Keep only the most recent errors within the oscillation window
      if len(self.oscillation_history) > self.oscillation_window:
        self.oscillation_history.pop(0)

      # Detect oscillations and adjust gain accordingly
      self._update_oscillation_damping()

      self.p = error * self._get_k_p()
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

  def _update_oscillation_damping(self):
    """Update the oscillation damping based on recent error history."""
    if len(self.oscillation_history) < 10:  # Need minimum history to detect oscillations
      # Gradually restore gain when there's insufficient history
      self.oscillation_gain_factor = min(self.oscillation_gain_factor * self.oscillation_recovery_rate,
                                         self.max_oscillation_gain_factor)
      self.oscillation_detected = False
      self.oscillation_damping_active = False
      return

    # Detect oscillations using multiple methods for more robust detection
    # Method 1: Sign changes in significant errors (original method)
    significant_errors = [err for err in self.oscillation_history if abs(err) > self.oscillation_threshold]

    if len(significant_errors) < 4:  # Need sufficient significant errors to detect oscillation
      # Gradually restore gain when there's insufficient significant errors
      self.oscillation_gain_factor = min(self.oscillation_gain_factor * self.oscillation_recovery_rate,
                                         self.max_oscillation_gain_factor)
      self.oscillation_detected = False
      self.oscillation_damping_active = False
      return

    # Count sign changes to detect oscillation pattern
    sign_changes = 0
    for i in range(1, len(significant_errors)):
      if (significant_errors[i-1] > 0 and significant_errors[i] < 0) or \
         (significant_errors[i-1] < 0 and significant_errors[i] > 0):
        sign_changes += 1

    # Method 2: Variance-based oscillation detection
    # Calculate variance of recent errors to detect oscillation patterns
    recent_errors = self.oscillation_history[-20:]  # Use last 20 errors for variance calculation
    if len(recent_errors) > 1:
      error_variance = np.var(recent_errors)
      error_mean = np.mean(np.abs(recent_errors))
      # If variance is high relative to mean, it indicates oscillation
      relative_variance = error_variance / (error_mean + 1e-6)  # Add small value to prevent division by zero
    else:
      relative_variance = 0

    # Method 3: Zero-crossing detection (more sophisticated sign change detection)
    zero_crossings = 0
    for i in range(1, len(recent_errors)):
      if (recent_errors[i-1] >= 0 and recent_errors[i] < 0) or \
         (recent_errors[i-1] < 0 and recent_errors[i] >= 0):
        zero_crossings += 1

    # Combine multiple detection methods
    oscillation_ratio = sign_changes / len(significant_errors) if len(significant_errors) > 0 else 0
    zero_crossing_ratio = zero_crossings / len(recent_errors) if len(recent_errors) > 0 else 0

    # Determine if oscillation is occurring based on combined criteria
    oscillation_detected = (
      oscillation_ratio > 0.6 or  # Original sign change criteria
      relative_variance > 0.8 or  # High variance relative to mean
      zero_crossing_ratio > 0.5   # High zero crossing rate
    )

    if oscillation_detected:
      # Apply gain reduction for oscillation damping
      self.oscillation_gain_factor = max(self.oscillation_gain_factor * self.oscillation_gain_reduction,
                                         self.min_oscillation_gain_factor)
      self.oscillation_detected = True
      self.oscillation_damping_active = True
      self.oscillation_detection_count += 1
    else:
      # Gradually restore gain when no significant oscillations detected
      self.oscillation_gain_factor = min(self.oscillation_gain_factor * self.oscillation_recovery_rate,
                                         self.max_oscillation_gain_factor)
      self.oscillation_detected = False
      self.oscillation_damping_active = False
      self.oscillation_recovery_count += 1

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0
    # Reset oscillation detection state
    self.oscillation_history = []
    self.oscillation_gain_factor = 1.0
    self.oscillation_detected = False
    self.oscillation_damping_active = False
    self.oscillation_detection_count = 0
    self.oscillation_recovery_count = 0
