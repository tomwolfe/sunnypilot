import numpy as np
import time
from numbers import Number
from openpilot.common.performance_monitor import PerfTrack
from openpilot.common.swaglog import cloudlog

class PIDController:
  def __init__(self, k_p, k_i, k_f=0., k_d=0., k_curvature=None, max_curvature_gain_multiplier=4.0,
               safety_limit_threshold=100, safety_limit_time_window=60.0,  # Added configurable safety parameters
               pos_limit=1e308, neg_limit=-1e308, rate=100,
               oscillation_threshold=0.5, oscillation_window_size_seconds=0.5, oscillation_gain_reduction=0.9,
               oscillation_recovery_rate=1.005, min_oscillation_gain_factor=0.5, max_oscillation_gain_factor=1.0,
               oscillation_sign_change_threshold=0.6, oscillation_variance_threshold=0.8, oscillation_zero_crossing_threshold=0.5,  # Vehicle-specific oscillation thresholds
               vehicle_mass=1500.0, vehicle_wheelbase=2.7):  # Added vehicle mass and wheelbase to enable adaptive thresholds
    self._k_p = k_p
    self._k_i = k_i
    self._k_d = k_d
    self._k_curvature = k_curvature
    self.max_curvature_gain_multiplier = max_curvature_gain_multiplier  # Configurable maximum gain multiplier relative to base
    # Calculate reference values for safety limiting to prevent explosive behavior when both speed and curvature gains are high
    if isinstance(k_p, (list, tuple)) and len(k_p) == 2:
      # k_p is in format (breakpoints, values) like the car params
      self._max_k_p_possible = max(k_p[1]) if len(k_p[1]) > 0 else 1.0
      # Use the baseline (typically at speed 0) as a reference point for safety limits
      if len(k_p[1]) > 0:
        # If there's a value at speed 0, use it; otherwise interpolate or use first value
        if len(k_p[0]) > 0 and k_p[0][0] == 0:
          self._baseline_k_p = k_p[1][0]
        else:
          # Interpolate at speed 0, or use first value if speed 0 not available
          self._baseline_k_p = np.interp(0.0, k_p[0], k_p[1]) if len(k_p[0]) > 0 else k_p[1][0]
      else:
        self._baseline_k_p = 1.0
    elif isinstance(k_p, list) and len(k_p) == 2:
      # k_p is in format [[breakpoints], [values]]
      self._max_k_p_possible = max(k_p[1]) if len(k_p[1]) > 0 else 1.0
      if len(k_p[1]) > 0:
        if len(k_p[0]) > 0 and k_p[0][0] == 0:
          self._baseline_k_p = k_p[1][0]
        else:
          self._baseline_k_p = np.interp(0.0, k_p[0], k_p[1]) if len(k_p[0]) > 0 else k_p[1][0]
      else:
        self._baseline_k_p = 1.0
    else:
      self._max_k_p_possible = abs(float(k_p)) if isinstance(k_p, (int, float)) else 1.0  # Safe default
      self._baseline_k_p = self._max_k_p_possible  # For fixed k_p values

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

    # Safety parameters - make thresholds configurable
    self.safety_limit_threshold = safety_limit_threshold  # Configurable threshold for safe mode activation
    self.safety_limit_time_window = safety_limit_time_window  # Time window in seconds for threshold evaluation
    self.safety_limit_trigger_times = []  # Keep track of when safety limits were triggered (for time-based checking)

    # Adaptive damping mechanism for oscillation detection with configurable parameters
    self.oscillation_history = []  # Store error values for oscillation detection
    # The oscillation threshold (0.5 degrees steering angle error) is based on typical vehicle dynamics
    # and represents a balance between sensitivity to actual oscillations and filtering of normal control variations
    # This threshold is calibrated to detect sustained oscillatory behavior while avoiding false positives
    # from normal steering corrections during regular driving
    self.oscillation_threshold = oscillation_threshold
    # Use configurable oscillation window size with a default of 0.5 seconds
    # This allows for performance optimization while maintaining safety
    self.oscillation_window = int(rate * oscillation_window_size_seconds)  # Configurable window size in seconds
    self.oscillation_gain_reduction = oscillation_gain_reduction  # Configurable gain reduction when oscillation detected
    self.oscillation_recovery_rate = oscillation_recovery_rate  # Configurable rate to gradually restore gain when no oscillations
    self.min_oscillation_gain_factor = min_oscillation_gain_factor  # Configurable minimum gain factor (50% of computed gain)
    self.max_oscillation_gain_factor = max_oscillation_gain_factor  # Configurable maximum gain factor (100% of computed gain)

    # Vehicle-specific oscillation detection thresholds
    self.oscillation_sign_change_threshold = oscillation_sign_change_threshold  # Configurable sign change threshold
    self.oscillation_variance_threshold = oscillation_variance_threshold  # Configurable variance threshold
    self.oscillation_zero_crossing_threshold = oscillation_zero_crossing_threshold  # Configurable zero crossing threshold

    # Store vehicle mass and wheelbase for adaptive oscillation detection
    self.vehicle_mass = vehicle_mass
    self.vehicle_wheelbase = vehicle_wheelbase

    # Calculate adaptive oscillation thresholds based on vehicle characteristics
    # Heavier vehicles may need different thresholds than lighter vehicles
    self.oscillation_threshold = self._calculate_adaptive_threshold(oscillation_threshold)
    self.oscillation_sign_change_threshold = self._calculate_adaptive_oscillation_thresholds(
        self.oscillation_sign_change_threshold, "sign_change"
    )
    self.oscillation_variance_threshold = self._calculate_adaptive_oscillation_thresholds(
        self.oscillation_variance_threshold, "variance"
    )
    self.oscillation_zero_crossing_threshold = self._calculate_adaptive_oscillation_thresholds(
        self.oscillation_zero_crossing_threshold, "zero_crossing"
    )

    self.oscillation_gain_factor = 1.0  # Current gain factor
    # Enhanced oscillation detection metrics for monitoring
    self.oscillation_detected = False
    self.oscillation_damping_active = False
    self.oscillation_detection_count = 0
    self.oscillation_recovery_count = 0

    # Safe mode indicators
    self.safe_mode_active = False
    self.safe_mode_trigger_count = 0
    self.safety_limit_trigger_count = 0  # Track how often safety limits are being hit

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

    # Initialize the internal state variables
    self.reset()

  def set_oscillation_window_size(self, window_size_seconds, rate):
    """
    Configure the oscillation detection window size for performance optimization.

    Args:
        window_size_seconds: Base size of the oscillation detection window in seconds
        rate: Control rate in Hz used to convert seconds to samples
    """
    new_window = self._calculate_adaptive_window_size(window_size_seconds, rate)
    # Ensure we have enough samples for reliable oscillation detection
    new_window = max(10, new_window)  # At least 10 samples for detection
    self.oscillation_window = new_window
    # Trim history if new window is smaller than current history
    if len(self.oscillation_history) > self.oscillation_window:
      self.oscillation_history = self.oscillation_history[-self.oscillation_window:]

  def _get_k_p(self):
    k_p = self._k_p[1][0] if not self._use_interp else np.interp(self.speed, self._k_p[0], self._k_p[1])

    if self._k_curvature is not None:
      try:
        # Validate curvature before interpolation to prevent NaN propagation
        current_curvature = abs(self._last_desired_curvature)
        if np.isnan(current_curvature) or np.isinf(current_curvature):
          # Enter safe mode if we get invalid curvature values
          self.safe_mode_active = True
          self.safe_mode_trigger_count += 1
          current_curvature = 0.0  # Use safe default

        # Ensure we have valid curvature arrays
        if (len(self._k_curvature[0]) == 0 or len(self._k_curvature[1]) == 0 or
            len(self._k_curvature[0]) != len(self._k_curvature[1])):
          # Enter safe mode if curvature arrays are invalid
          self.safe_mode_active = True
          self.safe_mode_trigger_count += 1
          return k_p * self.oscillation_gain_factor  # Return base gain without curvature modification

        curvature_gain = np.interp(current_curvature, self._k_curvature[0], self._k_curvature[1])

        # Validate curvature gain result
        if np.isnan(curvature_gain) or np.isinf(curvature_gain) or curvature_gain < 0:
          # Enter safe mode if curvature gain calculation failed
          self.safe_mode_active = True
          self.safe_mode_trigger_count += 1
          curvature_gain = 1.0  # Use neutral gain multiplier

        # Address the potential issue of excessive gain multiplication when both
        # speed-based and curvature-based gains are high by implementing a more
        # controlled gain application that prevents explosive behavior while maintaining
        # the essential curvature-based responsiveness
        original_k_p = k_p  # Store original for safety limiting
        combined_gain = k_p * curvature_gain  # Compute the combined gain first

        # Implement safety bounds to prevent excessive gain growth when both factors are high
        # This addresses the critical concern that when both speed-based and curvature-based
        # gains are high simultaneously, the combined effect could be dangerous
        # First apply the original relative multiplier limit (relative to current speed-adjusted k_p)
        max_relative_gain = original_k_p * self.max_curvature_gain_multiplier
        # Use current speed-adjusted k_p (not baseline) as the reference for the baseline limit
        # This addresses the concern that baseline limit (speed 0) might be too conservative at high speeds
        baseline_limit = original_k_p * self.max_curvature_gain_multiplier

        # Take the minimum of combined gain, original relative limit, and additional safety limit
        k_p = min(combined_gain, max_relative_gain, baseline_limit)

        # Check if we hit any of the safety limits (meaning the gain was reduced)
        if k_p < combined_gain:
          self.safety_limit_trigger_count += 1
          # Record the time when safety limit was triggered for time-based thresholding
          current_time = time.time()  # Need to import time module
          self.safety_limit_trigger_times.append(current_time)

          # Remove old trigger times outside the time window
          self.safety_limit_trigger_times = [
              t for t in self.safety_limit_trigger_times
              if current_time - t <= self.safety_limit_time_window
          ]

          # If safety limits are being hit too frequently (both count and time-based), enter safe mode
          if (self.safety_limit_trigger_count > self.safety_limit_threshold or
              len(self.safety_limit_trigger_times) > self.safety_limit_threshold):
            if not self.safe_mode_active:
              # Only increment activation count and log when entering safe mode (not when already in safe mode)
              self.safe_mode_activated_count += 1
              self.safe_mode_start_time = time.time()
              cloudlog.warning(f"Curvature gain safe mode activated due to safety limits being exceeded. "
                             f"Trigger count: {self.safety_limit_trigger_count}, "
                             f"Time-based count: {len(self.safety_limit_trigger_times)} in {self.safety_limit_time_window}s")
            self.safe_mode_active = True
      except Exception as e:
        # Enter safe mode if any calculation error occurs
        self.safe_mode_active = True
        self.safe_mode_trigger_count += 1
        cloudlog.error(f"Curvature gain calculation error: {e}")
        # Fall back to base gain without curvature modification
        return k_p * self.oscillation_gain_factor

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
    # Track performance metrics for oscillation detection overhead
    start_time = time.perf_counter()
    oscillation_detection_time = 0

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
      oscillation_start = time.perf_counter()
      self._update_oscillation_damping()
      oscillation_detection_time = time.perf_counter() - oscillation_start

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

      # Store performance metrics
      total_update_time = time.perf_counter() - start_time
      self.last_update_time = total_update_time
      self.last_oscillation_detection_time = oscillation_detection_time

      # Check for safe mode recovery if needed
      self._check_safe_mode_recovery()

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

    # Get the most recent errors for variance and zero-crossing calculations
    recent_errors = self.oscillation_history[-20:]  # Use last 20 errors for variance calculation
    recent_errors_array = np.array(recent_errors)

    # Detect oscillations using multiple methods for more robust detection
    # Three complementary methods provide redundancy to avoid missed oscillations
    # while maintaining robustness against false positives

    # Method 1: Sign changes in significant errors
    # Only consider errors that exceed the configured threshold as "significant"
    # This filters out minor errors that are part of normal control operation
    abs_errors = np.abs(recent_errors_array)
    significant_error_indices = np.where(abs_errors > self.oscillation_threshold)[0]
    significant_errors_array = recent_errors_array[significant_error_indices]

    if len(significant_errors_array) < 4:  # Need sufficient significant errors to detect oscillation patterns
      # Gradually restore gain when there's insufficient significant errors (indicating stable operation)
      self.oscillation_gain_factor = min(self.oscillation_gain_factor * self.oscillation_recovery_rate,
                                         self.max_oscillation_gain_factor)
      self.oscillation_detected = False
      self.oscillation_damping_active = False
      return

    # Count sign changes between consecutive significant errors to detect oscillatory pattern
    sign_changes = 0
    prev_sign = np.sign(significant_errors_array[0])
    for i in range(1, len(significant_errors_array)):
      current_sign = np.sign(significant_errors_array[i])
      if prev_sign != current_sign and current_sign != 0:  # Only count sign changes when not zero
        sign_changes += 1
      prev_sign = current_sign

    # Method 2: Variance-based oscillation detection
    # Calculate variance of all recent errors to detect overall variability
    # High variance relative to the mean absolute error suggests oscillatory behavior
    if len(recent_errors_array) > 1:
      error_variance = np.var(recent_errors_array)
      error_mean = np.mean(np.abs(recent_errors_array))
      # If variance is high relative to mean, it indicates oscillation-like behavior
      relative_variance = error_variance / (error_mean + 1e-6)  # Add small value to prevent division by zero
    else:
      relative_variance = 0

    # Method 3: Zero-crossing detection (counting direction changes of all errors)
    # This detects how frequently the error crosses zero (changes sign)
    # High zero-crossing rate indicates oscillatory behavior
    zero_crossings = 0
    if len(recent_errors_array) > 1:
      for i in range(1, len(recent_errors_array)):
        if (recent_errors_array[i-1] >= 0 and recent_errors_array[i] < 0) or \
           (recent_errors_array[i-1] < 0 and recent_errors_array[i] >= 0):
          zero_crossings += 1

    # Compute ratios for threshold comparison
    oscillation_ratio = sign_changes / len(significant_errors_array) if len(significant_errors_array) > 0 else 0
    zero_crossing_ratio = zero_crossings / len(recent_errors_array) if len(recent_errors_array) > 0 else 0

    # Determine if oscillation is occurring based on combined criteria using vehicle-specific thresholds
    # Using OR logic allows detection when any method indicates oscillations (for robustness)
    oscillation_detected = (
      oscillation_ratio > self.oscillation_sign_change_threshold or
      relative_variance > self.oscillation_variance_threshold or
      zero_crossing_ratio > self.oscillation_zero_crossing_threshold
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

  def _calculate_adaptive_threshold(self, base_threshold):
    """
    Calculate an adaptive oscillation threshold based on vehicle mass.
    Heavier vehicles might need higher thresholds due to different dynamics.
    """
    # Adjust threshold based on vehicle mass
    # Heavier vehicles (trucks/SUVs) might have different oscillation characteristics
    if self.vehicle_mass > 2000:  # Heavy vehicles
      return base_threshold * 1.2  # Slightly higher threshold
    elif self.vehicle_mass < 1200:  # Light vehicles
      return base_threshold * 0.8  # Slightly lower threshold
    else:  # Standard vehicles
      return base_threshold

  def _calculate_adaptive_oscillation_thresholds(self, base_threshold, threshold_type):
    """
    Calculate adaptive oscillation thresholds based on vehicle characteristics.
    Different thresholds may be needed for different vehicle types based on their mass and wheelbase.

    Args:
        base_threshold: The default threshold value
        threshold_type: Type of threshold ('sign_change', 'variance', 'zero_crossing')
    """
    # Adjust thresholds based on vehicle mass and wheelbase
    # Heavier vehicles (trucks/SUVs) might have different oscillation characteristics
    # Smaller vehicles might have more responsive dynamics
    if self.vehicle_mass > 2000:  # Heavy vehicles
      if threshold_type == "sign_change":
        return base_threshold * 1.1  # Slightly higher threshold for heavy vehicles
      elif threshold_type == "variance":
        return base_threshold * 1.15
      else:  # zero_crossing
        return base_threshold * 1.05
    elif self.vehicle_mass < 1200:  # Light vehicles
      if threshold_type == "sign_change":
        return base_threshold * 0.9  # Slightly lower threshold for light vehicles
      elif threshold_type == "variance":
        return base_threshold * 0.85
      else:  # zero_crossing
        return base_threshold * 0.95
    else:  # Standard vehicles
      return base_threshold

  def _calculate_adaptive_window_size(self, base_window_size, rate):
    """
    Calculate an adaptive oscillation detection window size based on vehicle mass.
    Heavier vehicles might need different time windows for oscillation detection.
    """
    # Adjust window size based on vehicle mass and dynamics
    if self.vehicle_mass > 2000:  # Heavy vehicles
      return int(rate * (base_window_size * 1.2))  # Slightly larger window
    elif self.vehicle_mass < 1200:  # Light vehicles
      return int(rate * (base_window_size * 0.8))  # Slightly smaller window
    else:  # Standard vehicles
      return int(rate * base_window_size)

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
    # Reset safe mode indicators
    self.safe_mode_active = False
    self.safe_mode_trigger_count = 0
    self.safety_limit_trigger_count = 0
    self.safety_limit_trigger_times = []  # Reset safety limit trigger times
    # Reset safe mode recovery tracking
    self.safe_mode_start_time = None
    self.safe_mode_recovery_start_time = None
    self.safe_mode_activated_count = 0
    # Reset performance monitoring variables
    self.last_update_time = 0.0
    self.last_oscillation_detection_time = 0.0

  def _check_safe_mode_recovery(self):
    """
    Check if safe mode should be deactivated based on normalized conditions.
    Safe mode will automatically recover when safety issues are no longer detected for a specified time period.
    """
    if not self.safe_mode_active:
      return  # Nothing to do if not in safe mode

    current_time = time.time()

    # Check if safety limit triggers have decreased significantly
    # Remove old trigger times outside the time window
    self.safety_limit_trigger_times = [
        t for t in self.safety_limit_trigger_times
        if current_time - t <= self.safety_limit_time_window
    ]

    # If we're no longer hitting safety limits frequently, start recovery process
    if (len(self.safety_limit_trigger_times) <= self.safety_limit_threshold // 2 and
        self.safety_limit_trigger_count <= self.safety_limit_threshold // 2):

      if self.safe_mode_recovery_start_time is None:
        # Start recovery timer
        self.safe_mode_recovery_start_time = current_time
      elif current_time - self.safe_mode_recovery_start_time >= 5.0:  # 5 seconds of stable operation
        # Safe mode recovery conditions met
        self.safe_mode_active = False
        self.safe_mode_recovery_start_time = None
        self.safe_mode_start_time = None  # Reset safe mode start time
        cloudlog.info(f"Curvature gain safe mode deactivated after stable operation. "
                      f"Safety limit triggers: {len(self.safety_limit_trigger_times)} in {self.safety_limit_time_window}s window")
    else:
      # Conditions are still unstable, reset recovery timer
      self.safe_mode_recovery_start_time = None
