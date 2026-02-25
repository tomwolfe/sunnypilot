#include "e2e_path_blender.h"
#include <cmath>
#include <algorithm>
#include <cstring>

namespace sunnypilot {

E2EPathBlender::E2EPathBlender()
    : recovery_power_control_(kDefaultRecoveryControl),
      speed_threshold_(kSpeedThreshold),
      long_smooth_seconds_(0.1f) {
  smooth_accel_.fill(0.0f);
  smooth_vel_.fill(0.0f);
  std::memset(prev_accel_, 0, sizeof(prev_accel_));
}

void E2EPathBlender::set_recovery_power(float control) {
  recovery_power_control_ = control;
}

void E2EPathBlender::set_speed_threshold(float threshold_ms) {
  speed_threshold_ = threshold_ms;
}

float E2EPathBlender::compute_recovery_power(float v_ego) const {
  float speed_ms = v_ego / 3.6f;
  float scale = (speed_ms > speed_threshold_) ? kHighSpeedScaling : 1.0f;
  return recovery_power_control_ * scale;
}

void E2EPathBlender::update_plan(const E2EPlan& e2e_plan, const E2EPlan& planplus_plan,
                                 float v_ego, float* output_plan) {
  float recovery = compute_recovery_power(v_ego);
  
  float blend_weight = recovery;
  
  blend_trajectory(e2e_plan, planplus_plan, blend_weight, output_plan);
  
  smooth_longitudinal(output_plan + TRAJECTORY_POINTS,
                     output_plan + 2 * TRAJECTORY_POINTS,
                     TRAJECTORY_POINTS);
}

void E2EPathBlender::blend_trajectory(const E2EPlan& e2e, const E2EPlan& planplus,
                                      float blend_weight, float* output) {
  for (size_t i = 0; i < TRAJECTORY_POINTS; ++i) {
    float w = blend_weight;
    
    output[i] = e2e.x[i] + w * (planplus.x[i] - e2e.x[i]);
    output[TRAJECTORY_POINTS + i] = e2e.y[i] + w * (planplus.y[i] - e2e.y[i]);
    output[2 * TRAJECTORY_POINTS + i] = e2e.velocity[i] + w * (planplus.velocity[i] - e2e.velocity[i]);
    output[3 * TRAJECTORY_POINTS + i] = e2e.acceleration[i] + w * (planplus.acceleration[i] - e2e.acceleration[i]);
  }
}

void E2EPathBlender::smooth_longitudinal(float* accel, float* vel, size_t count) {
  float alpha = 0.1f;
  
  for (size_t i = 0; i < count; ++i) {
    smooth_accel_[i] = alpha * accel[i] + (1.0f - alpha) * prev_accel_[i];
    prev_accel_[i] = smooth_accel_[i];
    accel[i] = smooth_accel_[i];
    
    if (i > 0) {
      vel[i] = vel[i - 1] + smooth_accel_[i] * 0.1f;
    }
  }
}

float E2EPathBlender::lerp(float a, float b, float t) const {
  return a + t * (b - a);
}

FeatureMemoryBuffer::FeatureMemoryBuffer(size_t feature_dim, size_t max_history)
    : feature_dim_(feature_dim),
      max_history_(max_history) {
  W_q_.resize(feature_dim * feature_dim, 0.0f);
  W_k_.resize(feature_dim * feature_dim, 0.0f);
  W_v_.resize(feature_dim * feature_dim, 0.0f);
  
  for (size_t i = 0; i < feature_dim; ++i) {
    W_q_[i * feature_dim + i] = 1.0f;
    W_k_[i * feature_dim + i] = 1.0f;
    W_v_[i * feature_dim + i] = 1.0f;
  }
}

FeatureMemoryBuffer::~FeatureMemoryBuffer() {}

void FeatureMemoryBuffer::add_feature(const float* feature) {
  if (history_.size() >= max_history_) {
    history_.erase(history_.begin());
  }
  
  std::vector<float> feat(feature_dim_);
  std::memcpy(feat.data(), feature, feature_dim_ * sizeof(float));
  history_.push_back(std::move(feat));
}

void FeatureMemoryBuffer::get_contextual_feature(const float* current, float* output) {
  if (history_.empty()) {
    std::memcpy(output, current, feature_dim_ * sizeof(float));
    return;
  }
  
  std::vector<float> query(feature_dim_, 0.0f);
  std::vector<float> keys(history_.size() * feature_dim_, 0.0f);
  std::vector<float> values(history_.size() * feature_dim_, 0.0f);
  
  for (size_t h = 0; h < history_.size(); ++h) {
    for (size_t i = 0; i < feature_dim_; ++i) {
      for (size_t j = 0; j < feature_dim_; ++j) {
        query[i] += current[j] * W_q_[j * feature_dim_ + i];
        keys[h * feature_dim_ + i] += history_[h][j] * W_k_[j * feature_dim_ + i];
        values[h * feature_dim_ + i] += history_[h][j] * W_v_[j * feature_dim_ + i];
      }
    }
  }
  
  scaled_dot_product_attention(query.data(), keys.data(), values.data(),
                               history_.size(), output);
  
  for (size_t i = 0; i < feature_dim_; ++i) {
    output[i] = (current[i] + output[i]) * 0.5f;
  }
}

void FeatureMemoryBuffer::reset() {
  history_.clear();
}

void FeatureMemoryBuffer::scaled_dot_product_attention(const float* query, const float* keys,
                                                       const float* values, size_t num_keys,
                                                       float* output) {
  std::vector<float> scores(num_keys, 0.0f);
  float scale = std::sqrt(static_cast<float>(feature_dim_));
  
  for (size_t k = 0; k < num_keys; ++k) {
    for (size_t i = 0; i < feature_dim_; ++i) {
      scores[k] += query[i] * keys[k * feature_dim_ + i];
    }
    scores[k] /= scale;
  }
  
  softmax(scores.data(), num_keys);
  
  std::fill(output, output + feature_dim_, 0.0f);
  for (size_t k = 0; k < num_keys; ++k) {
    for (size_t i = 0; i < feature_dim_; ++i) {
      output[i] += scores[k] * values[k * feature_dim_ + i];
    }
  }
}

void FeatureMemoryBuffer::softmax(float* arr, size_t count) {
  float max_val = arr[0];
  for (size_t i = 1; i < count; ++i) {
    if (arr[i] > max_val) max_val = arr[i];
  }
  
  float sum = 0.0f;
  for (size_t i = 0; i < count; ++i) {
    arr[i] = std::exp(arr[i] - max_val);
    sum += arr[i];
  }
  
  for (size_t i = 0; i < count; ++i) {
    arr[i] /= sum;
  }
}

E2ESafetyChecker::E2ESafetyChecker()
    : deviation_threshold_deg_(15.0f),
      cutoff_timeout_ms_(100),
      danger_frame_count_(0) {}

void E2ESafetyChecker::set_deviation_threshold(float degrees) {
  deviation_threshold_deg_ = degrees;
}

void E2ESafetyChecker::set_cutoff_timeout_ms(uint64_t ms) {
  cutoff_timeout_ms_ = ms;
}

E2ESafetyChecker::SafetyLevel E2ESafetyChecker::check_path_sanity(
    const E2EPlan& plan, float road_edge_angle, float v_ego) {
  
  if (is_path_deviation_excessive(plan, road_edge_angle)) {
    danger_frame_count_++;
    if (danger_frame_count_ > 2) {
      return SafetyLevel::CUT_TORQUE;
    }
    return SafetyLevel::DANGER;
  }
  
  if (danger_frame_count_ > 0) {
    danger_frame_count_--;
  }
  
  float max_uncertainty = 0.0f;
  for (size_t i = 0; i < TRAJECTORY_POINTS; ++i) {
    if (plan.uncertainty[i] > max_uncertainty) {
      max_uncertainty = plan.uncertainty[i];
    }
  }
  
  if (max_uncertainty > 10.0f) {
    return SafetyLevel::CAUTION;
  }
  
  return SafetyLevel::SAFE;
}

bool E2ESafetyChecker::is_path_deviation_excessive(const E2EPlan& plan,
                                                    float road_edge_angle) {
  if (TRAJECTORY_POINTS < 2) return false;
  
  float path_angle = std::atan2(plan.y[TRAJECTORY_POINTS - 1] - plan.y[0],
                                 plan.x[TRAJECTORY_POINTS - 1] - plan.x[0]);
  float angle_diff = std::abs(path_angle - road_edge_angle);
  
  float angle_diff_deg = angle_diff * 180.0f / M_PI;
  
  return angle_diff_deg > deviation_threshold_deg_;
}

VisionOnlyTrafficDetector::VisionOnlyTrafficDetector()
    : current_signal_(TrafficSignal::NONE),
      signal_distance_(0.0f),
      signal_probability_(0.0f),
      last_detection_time_(0),
      precharge_brake_prob_(0.0f) {}

void VisionOnlyTrafficDetector::update(const float* model_uncertainty,
                                       const float* disengage_probs,
                                       size_t num_points, float v_ego) {
  float stop_prob = predict_stop_probability(disengage_probs, num_points, v_ego);
  
  float max_uncertainty = 0.0f;
  for (size_t i = 0; i < num_points && i < TRAJECTORY_POINTS; ++i) {
    if (model_uncertainty[i] > max_uncertainty) {
      max_uncertainty = model_uncertainty[i];
    }
  }
  
  if (max_uncertainty > 8.0f && stop_prob > kStopSignUrgencyThreshold) {
    current_signal_ = TrafficSignal::STOP_SIGN;
    signal_probability_ = stop_prob;
    signal_distance_ = max_uncertainty * 2.0f;
  } else if (max_uncertainty > 6.0f && stop_prob > kRedLightUrgencyThreshold) {
    current_signal_ = TrafficSignal::RED_LIGHT;
    signal_probability_ = stop_prob;
    signal_distance_ = max_uncertainty * 1.5f;
  } else {
    current_signal_ = TrafficSignal::NONE;
    signal_probability_ = 0.0f;
    signal_distance_ = 0.0f;
  }
  
  if (stop_prob > 0.5f && v_ego > 2.0f) {
    precharge_brake_prob_ = stop_prob;
  } else {
    precharge_brake_prob_ *= 0.95f;
  }
}

VisionOnlyTrafficDetector::Detection VisionOnlyTrafficDetector::get_detection() const {
  Detection det;
  det.type = current_signal_;
  det.distance = signal_distance_;
  det.probability = signal_probability_;
  det.timestamp = last_detection_time_;
  return det;
}

float VisionOnlyTrafficDetector::get_precharge_brake_probability() const {
  return precharge_brake_prob_;
}

float VisionOnlyTrafficDetector::predict_stop_probability(
    const float* disengage_probs, size_t count, float v_ego) const {
  if (count == 0 || v_ego < 0.5f) return 0.0f;
  
  float lookahead_idx = kPredictiveBrakeLookaheadSec * 10.0f;
  size_t idx = static_cast<size_t>(lookahead_idx);
  if (idx >= count) idx = count - 1;
  
  float prob = disengage_probs[idx];
  
  for (size_t i = 0; i <= idx && i < count; ++i) {
    if (disengage_probs[i] > prob) {
      prob = disengage_probs[i];
    }
  }
  
  return prob;
}

}
