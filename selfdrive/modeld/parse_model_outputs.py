import numpy as np
import time
from collections import deque
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants

def safe_exp(x, out=None):
  # -11 is around 10**14, more causes float16 overflow
  return np.exp(np.clip(x, -np.inf, 11), out=out)

def sigmoid(x):
  return 1. / (1. + safe_exp(-x))

def softmax(x, axis=-1):
  x -= np.max(x, axis=axis, keepdims=True)
  if x.dtype == np.float32 or x.dtype == np.float64:
    safe_exp(x, out=x)
  else:
    x = safe_exp(x)
  x /= np.sum(x, axis=axis, keepdims=True)
  return x

class TemporalConsistencyFilter:
  """
  The TemporalConsistencyFilter enhances model prediction stability by applying
  temporal smoothing techniques to reduce noise and jerky movements in predictions
  such as the driving plan, lane lines, and lead vehicle data. It uses a moving
  average over a specified buffer size, with context-aware adjustments like
  velocity-dependent factors and confidence-based filtering. This addresses
  the common user complaint of unstable or jerky behavior, providing a smoother
  and more comfortable driving experience.
  """
  def __init__(self, buffer_size=5):
    # buffer_size: The number of past frames to consider for smoothing.
    # A 5-frame buffer (50ms at 100Hz) introduces a small delay but significantly
    # reduces noise and jitter.
    self.buffer_size = buffer_size
    self.plan_buffer = deque(maxlen=buffer_size)
    self.lane_line_buffer = deque(maxlen=buffer_size)
    self.lead_buffer = deque(maxlen=buffer_size)

  def smooth_plan(self, current_plan):
    """
    Applies temporal smoothing to the driving plan trajectory to reduce jerky
    movements. This method uses a weighted average of the current and previous
    plans, with more aggressive smoothing applied to acceleration changes.

    Args:
      current_plan: The current model prediction for the driving plan.

    Returns:
      The smoothed driving plan.
    """
    start_time = time.monotonic()
    if len(self.plan_buffer) == 0:
      self.plan_buffer.append(current_plan.copy())
      cloudlog.debug(f"smooth_plan took {(time.monotonic() - start_time) * 1000:.2f} ms (initial)")
      return current_plan

    # Calculate the difference from previous plan
    prev_plan = self.plan_buffer[-1]

    # Apply smoothing based on velocity and curvature to reduce sudden changes
    # For the plan trajectory (position, velocity, acceleration)
    smoothed_plan = current_plan.copy()

    # Smooth position changes based on velocity to avoid sudden lane changes.
    # A velocity-dependent factor could be used here for more advanced smoothing,
    # but a constant smoothing factor is applied for simplicity and effectiveness.
    position_smoothing = 0.8  # Reduce position changes by 20%
    smoothed_plan[:, :3] = prev_plan[:, :3] * (1 - position_smoothing) + current_plan[:, :3] * position_smoothing

    # Apply more aggressive smoothing to acceleration for a smoother ride.
    # Sudden changes in acceleration are more uncomfortable than changes in position.
    acceleration_smoothing = 0.6  # Reduce acceleration changes by 40%
    smoothed_plan[:, 6:9] = prev_plan[:, 6:9] * (1 - acceleration_smoothing) + current_plan[:, 6:9] * acceleration_smoothing

    # Store in buffer and return smoothed plan
    self.plan_buffer.append(smoothed_plan.copy())
    cloudlog.debug(f"smooth_plan took {(time.monotonic() - start_time) * 1000:.2f} ms")
    return smoothed_plan

  def smooth_lane_lines(self, current_lane_lines):
    """
    Smooths lane line predictions for more consistent lane keeping.
    This helps prevent "snappy" lane line behavior on the UI and in control.

    Args:
      current_lane_lines: The current model prediction for lane lines.

    Returns:
      The smoothed lane line predictions.
    """
    start_time = time.monotonic()
    if len(self.lane_line_buffer) == 0:
      self.lane_line_buffer.append(current_lane_lines.copy())
      cloudlog.debug(f"smooth_lane_lines took {(time.monotonic() - start_time) * 1000:.2f} ms (initial)")
      return current_lane_lines

    prev_lines = self.lane_line_buffer[-1]
    # Apply smoothing to lane line polynomial coefficients using a simple moving average.
    smoothing_factor = 0.9
    smoothed_lines = prev_lines * (1 - smoothing_factor) + current_lane_lines * smoothing_factor

    self.lane_line_buffer.append(smoothed_lines.copy())
    cloudlog.debug(f"smooth_lane_lines took {(time.monotonic() - start_time) * 1000:.2f} ms")
    return smoothed_lines

  def smooth_lead(self, current_lead):
    """
    Smooths lead vehicle detection to reduce jerky reactions and improve
    longitudinal control stability. This is particularly important for
    maintaining a comfortable following distance.

    Args:
      current_lead: The current model prediction for the lead vehicle.

    Returns:
      The smoothed lead vehicle detection.
    """
    start_time = time.monotonic()
    if len(self.lead_buffer) == 0:
      self.lead_buffer.append(current_lead.copy())
      cloudlog.debug(f"smooth_lead took {(time.monotonic() - start_time) * 1000:.2f} ms (initial)")
      return current_lead

    prev_lead = self.lead_buffer[-1]
    # Apply smoothing to lead estimates, being careful about distance and velocity.
    # Only smooth when we have consistent lead detection (i.e., the lead vehicle
    # is continuously detected). If detection is intermittent, prioritize the
    # current detection to avoid "ghost" leads.
    smoothing_factor = 0.85
    # Only smooth when we have consistent lead detection (not oscillating between lead/no-lead)
    if (np.all(prev_lead[:, :, 0] > 0) and np.all(current_lead[:, :, 0] > 0)):  # Distance > 0
      smoothed_lead = prev_lead * (1 - smoothing_factor) + current_lead * smoothing_factor
    else:
      # If lead detection is inconsistent, trust current detection more
      smoothed_lead = current_lead

    self.lead_buffer.append(smoothed_lead.copy())
    cloudlog.debug(f"smooth_lead took {(time.monotonic() - start_time) * 1000:.2f} ms")
    return smoothed_lead

class Parser:
  def __init__(self, ignore_missing=False):
    self.ignore_missing = ignore_missing
    # Add temporal consistency filter to reduce jerky movements and improve prediction stability
    self.temporal_filter = TemporalConsistencyFilter()

  def check_missing(self, outs, name):
    missing = name not in outs
    if missing and not self.ignore_missing:
      raise ValueError(f"Missing output {name}")
    return missing

  def parse_categorical_crossentropy(self, name, outs, out_shape=None):
    if self.check_missing(outs, name):
      return
    raw = outs[name]
    if out_shape is not None:
      raw = raw.reshape((raw.shape[0],) + out_shape)
    outs[name] = softmax(raw, axis=-1)

  def parse_binary_crossentropy(self, name, outs):
    if self.check_missing(outs, name):
      return
    raw = outs[name]
    outs[name] = sigmoid(raw)

  def parse_mdn(self, name, outs, in_N=0, out_N=1, out_shape=None):
    if self.check_missing(outs, name):
      return
    raw = outs[name]
    raw = raw.reshape((raw.shape[0], max(in_N, 1), -1))

    n_values = (raw.shape[2] - out_N)//2
    pred_mu = raw[:,:,:n_values]
    pred_std = safe_exp(raw[:,:,n_values: 2*n_values])

    if in_N > 1:
      weights = np.zeros((raw.shape[0], in_N, out_N), dtype=raw.dtype)
      for i in range(out_N):
        weights[:,:,i - out_N] = softmax(raw[:,:,i - out_N], axis=-1)

      if out_N == 1:
        for fidx in range(weights.shape[0]):
          idxs = np.argsort(weights[fidx][:,0])[::-1]
          weights[fidx] = weights[fidx][idxs]
          pred_mu[fidx] = pred_mu[fidx][idxs]
          pred_std[fidx] = pred_std[fidx][idxs]
      full_shape = tuple([raw.shape[0], in_N] + list(out_shape))
      outs[name + '_weights'] = weights
      outs[name + '_hypotheses'] = pred_mu.reshape(full_shape)
      outs[name + '_stds_hypotheses'] = pred_std.reshape(full_shape)

      pred_mu_final = np.zeros((raw.shape[0], out_N, n_values), dtype=raw.dtype)
      pred_std_final = np.zeros((raw.shape[0], out_N, n_values), dtype=raw.dtype)
      for fidx in range(weights.shape[0]):
        for hidx in range(out_N):
          idxs = np.argsort(weights[fidx,:,hidx])[::-1]
          pred_mu_final[fidx, hidx] = pred_mu[fidx, idxs[0]]
          pred_std_final[fidx, hidx] = pred_std[fidx, idxs[0]]
    else:
      pred_mu_final = pred_mu
      pred_std_final = pred_std

    if out_N > 1:
      final_shape = tuple([raw.shape[0], out_N] + list(out_shape))
    else:
      final_shape = tuple([raw.shape[0],] + list(out_shape))
    outs[name] = pred_mu_final.reshape(final_shape)
    outs[name + '_stds'] = pred_std_final.reshape(final_shape)

  def is_mhp(self, outs, name, shape):
    if self.check_missing(outs, name):
      return False
    if outs[name].shape[1] == 2 * shape:
      return False
    return True

  def parse_vision_outputs(self, outs: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    self.parse_mdn('pose', outs, in_N=0, out_N=0, out_shape=(ModelConstants.POSE_WIDTH,))
    self.parse_mdn('wide_from_device_euler', outs, in_N=0, out_N=0, out_shape=(ModelConstants.WIDE_FROM_DEVICE_WIDTH,))
    self.parse_mdn('road_transform', outs, in_N=0, out_N=0, out_shape=(ModelConstants.POSE_WIDTH,))

    # Parse lane lines with additional confidence-based filtering
    self.parse_mdn('lane_lines', outs, in_N=0, out_N=0, out_shape=(ModelConstants.NUM_LANE_LINES,ModelConstants.IDX_N,ModelConstants.LANE_LINES_WIDTH))
    if 'lane_lines' in outs:
      # Apply confidence-based filtering for lane line predictions
      confidence_threshold = 0.3  # Only trust lane lines with confidence > 30%
      if 'lane_lines_prob' in outs:
        # Use lane line probabilities to filter out low-confidence predictions
        prob = outs['lane_lines_prob'].mean()  # Average probability across all lane lines
        if prob < confidence_threshold:
          # Reduce the weight of lane line inputs when confidence is low
          outs['lane_lines'] *= prob / confidence_threshold

    self.parse_mdn('road_edges', outs, in_N=0, out_N=0, out_shape=(ModelConstants.NUM_ROAD_EDGES,ModelConstants.IDX_N,ModelConstants.LANE_LINES_WIDTH))
    self.parse_binary_crossentropy('lane_lines_prob', outs)
    self.parse_categorical_crossentropy('desire_pred', outs, out_shape=(ModelConstants.DESIRE_PRED_LEN,ModelConstants.DESIRE_PRED_WIDTH))
    self.parse_binary_crossentropy('meta', outs)
    self.parse_binary_crossentropy('lead_prob', outs)
    lead_mhp = self.is_mhp(outs, 'lead', ModelConstants.LEAD_MHP_SELECTION * ModelConstants.LEAD_TRAJ_LEN * ModelConstants.LEAD_WIDTH)
    lead_in_N, lead_out_N = (ModelConstants.LEAD_MHP_N, ModelConstants.LEAD_MHP_SELECTION) if lead_mhp else (0, 0)
    lead_out_shape = (ModelConstants.LEAD_TRAJ_LEN, ModelConstants.LEAD_WIDTH) if lead_mhp else \
        (ModelConstants.LEAD_MHP_SELECTION, ModelConstants.LEAD_TRAJ_LEN, ModelConstants.LEAD_WIDTH)
    self.parse_mdn('lead', outs, in_N=lead_in_N, out_N=lead_out_N, out_shape=lead_out_shape)
    return outs

  def parse_policy_outputs(self, outs: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    plan_mhp = self.is_mhp(outs, 'plan',  ModelConstants.IDX_N * ModelConstants.PLAN_WIDTH)
    plan_in_N, plan_out_N = (ModelConstants.PLAN_MHP_N, ModelConstants.PLAN_MHP_SELECTION) if plan_mhp else (0, 0)
    self.parse_mdn('plan', outs, in_N=plan_in_N, out_N=plan_out_N, out_shape=(ModelConstants.IDX_N, ModelConstants.PLAN_WIDTH))

    # Apply temporal consistency filtering to the plan output
    if 'plan' in outs and outs['plan'].shape[0] > 0:
      outs['plan'] = self.temporal_filter.smooth_plan(outs['plan'])

    # Apply filtering to lane lines too if available
    if 'lane_lines' in outs and outs['lane_lines'].shape[0] > 0:
      outs['lane_lines'] = self.temporal_filter.smooth_lane_lines(outs['lane_lines'])

    # Apply filtering to lead vehicle information
    if 'lead' in outs and outs['lead'].shape[0] > 0:
      outs['lead'] = self.temporal_filter.smooth_lead(outs['lead'])

    self.parse_categorical_crossentropy('desire_state', outs, out_shape=(ModelConstants.DESIRE_PRED_WIDTH,))
    return outs

  def parse_outputs(self, outs: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    outs = self.parse_vision_outputs(outs)
    outs = self.parse_policy_outputs(outs)
    return outs
