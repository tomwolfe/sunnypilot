import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants

import logging

def safe_exp(x, out=None):
  # -11 is around 10**14, more causes float16 overflow
  # Check if x is a tinygrad tensor by checking for attributes not available in numpy arrays
  # tinygrad tensors don't have the same API as numpy arrays
  if hasattr(x, 'numpy') and hasattr(x, 'exp') and hasattr(x, 'clip'):
    # This is a tinygrad tensor - doesn't support 'out' parameter
    if out is not None:
      # For tinygrad tensor, we can't use out parameter, so issue a warning
      # logging.warning("out parameter not supported for tinygrad tensors in safe_exp")
      pass

    # Convert to numpy to check condition, then back to tensor for computation
    if (x > 11).any().numpy().item():
      logging.warning(f"safe_exp clipping detected: max input value was {(x).max().numpy().item()}")

    clipped_x = x.clip(-float('inf'), 11)
    return clipped_x.exp()
  else:
    # This is a numpy array
    if np.any(x > 11):
      logging.warning(f"safe_exp clipping detected: max input value was {np.max(x)}")
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

class Parser:
  def __init__(self, ignore_missing=False):
    self.ignore_missing = ignore_missing

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
    self.parse_mdn('lane_lines', outs, in_N=0, out_N=0, out_shape=(ModelConstants.NUM_LANE_LINES,ModelConstants.IDX_N,ModelConstants.LANE_LINES_WIDTH))
    self.parse_mdn('road_edges', outs, in_N=0, out_N=0, out_shape=(ModelConstants.NUM_ROAD_EDGES,ModelConstants.IDX_N,ModelConstants.LANE_LINES_WIDTH))
    self.parse_binary_crossentropy('lane_lines_prob', outs)

    # Enhanced lane line confidence and reliability tracking
    if 'lane_lines_prob' in outs and outs['lane_lines_prob'].size > 0:
      # Calculate average confidence for each lane line and overall lane line reliability
      lane_line_probs = outs['lane_lines_prob'][0, :, 0]  # Shape is (batch, num_lanes, 1)
      avg_lane_confidence = np.mean(lane_line_probs)

      # Calculate lane line consistency - how stable the lane detections are
      # Higher confidence values indicate more reliable lane detection
      lane_reliability = min(1.0, avg_lane_confidence * 1.5)  # Scale up to account for conservative baseline values

      # Store in outs for downstream use
      outs['lane_line_reliability'] = np.array([lane_reliability])

    self.parse_categorical_crossentropy('desire_pred', outs, out_shape=(ModelConstants.DESIRE_PRED_LEN,ModelConstants.DESIRE_PRED_WIDTH))
    self.parse_binary_crossentropy('meta', outs)
    self.parse_binary_crossentropy('lead_prob', outs)
    lead_mhp = self.is_mhp(outs, 'lead', ModelConstants.LEAD_MHP_SELECTION * ModelConstants.LEAD_TRAJ_LEN * ModelConstants.LEAD_WIDTH)
    lead_in_N, lead_out_N = (ModelConstants.LEAD_MHP_N, ModelConstants.LEAD_MHP_SELECTION) if lead_mhp else (0, 0)
    lead_out_shape = (ModelConstants.LEAD_TRAJ_LEN, ModelConstants.LEAD_WIDTH) if lead_mhp else \
        (ModelConstants.LEAD_MHP_SELECTION, ModelConstants.LEAD_TRAJ_LEN, ModelConstants.LEAD_WIDTH)
    self.parse_mdn('lead', outs, in_N=lead_in_N, out_N=lead_out_N, out_shape=lead_out_shape)

    # Enhanced path reliability calculation based on lane detection quality
    if 'lane_lines' in outs and outs['lane_lines'].size > 0 and 'lane_line_reliability' in outs:
      lane_lines = outs['lane_lines'][0]  # Shape is (num_lanes, idx_n, lane_lines_width)

      # Calculate path reliability based on lane line availability and confidence
      # Consider both the confidence of lane detection and the geometric consistency of the path
      path_reliability = outs['lane_line_reliability'][0]

      # Additional check: if we have good lane line coverage for the path prediction horizon
      num_valid_lanes = 0
      if lane_lines.ndim >= 3:
        for lane_idx in range(lane_lines.shape[0]):
          # Check if lane line has sufficient valid points (not too much missing data)
          valid_points = np.sum(np.abs(lane_lines[lane_idx, :ModelConstants.IDX_N//2, :]) > 0.001)  # First half of path
          if valid_points > ModelConstants.IDX_N//4:  # At least 25% of points are valid
            num_valid_lanes += 1

      # Enhance path reliability based on number of valid lanes detected
      if num_valid_lanes >= 2:  # We have at least 2 good lane lines
        path_reliability = min(1.0, path_reliability * 1.2)  # Boost reliability
      elif num_valid_lanes == 1:  # Only one lane line
        path_reliability = max(0.3, path_reliability * 0.8)  # Reduce reliability moderately
      else:  # No good lane lines
        path_reliability = max(0.1, path_reliability * 0.5)  # Significantly reduce reliability

      outs['path_reliability'] = np.array([path_reliability])

    return outs

  def parse_policy_outputs(self, outs: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    plan_mhp = self.is_mhp(outs, 'plan',  ModelConstants.IDX_N * ModelConstants.PLAN_WIDTH)
    plan_in_N, plan_out_N = (ModelConstants.PLAN_MHP_N, ModelConstants.PLAN_MHP_SELECTION) if plan_mhp else (0, 0)
    self.parse_mdn('plan', outs, in_N=plan_in_N, out_N=plan_out_N, out_shape=(ModelConstants.IDX_N, ModelConstants.PLAN_WIDTH))
    self.parse_categorical_crossentropy('desire_state', outs, out_shape=(ModelConstants.DESIRE_PRED_WIDTH,))
    return outs

  def parse_outputs(self, outs: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    outs = self.parse_vision_outputs(outs)
    outs = self.parse_policy_outputs(outs)
    return outs
