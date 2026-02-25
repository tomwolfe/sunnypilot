#ifndef E2E_PATH_BLENDER_H
#define E2E_PATH_BLENDER_H

#include <cstddef>
#include <cstdint>
#include <vector>
#include <array>

namespace sunnypilot {

constexpr size_t TRAJECTORY_POINTS = 33;
constexpr size_t PLAN_VELOCITY_IDX = 0;
constexpr size_t PLAN_ACCEL_IDX = 1;

struct TrajectoryPoint {
  float x;
  float y;
  float velocity;
  float acceleration;
};

struct E2EPlan {
  std::array<float, TRAJECTORY_POINTS> x;
  std::array<float, TRAJECTORY_POINTS> y;
  std::array<float, TRAJECTORY_POINTS> velocity;
  std::array<float, TRAJECTORY_POINTS> acceleration;
  float uncertainty[TRAJECTORY_POINTS];
};

class E2EPathBlender {
public:
  E2EPathBlender();
  
  void set_recovery_power(float control);
  void set_speed_threshold(float threshold_ms);
  
  void update_plan(const E2EPlan& e2e_plan, const E2EPlan& planplus_plan,
                   float v_ego, float* output_plan);
                   
  float compute_recovery_power(float v_ego) const;
  
  static constexpr float kDefaultRecoveryControl = 1.0f;
  static constexpr float kSpeedThreshold = 20.0f;
  static constexpr float kHighSpeedScaling = 0.75f;

private:
  float recovery_power_control_;
  float speed_threshold_;
  std::array<float, TRAJECTORY_POINTS> smooth_accel_;
  std::array<float, TRAJECTORY_POINTS> smooth_vel_;
  float prev_accel_[TRAJECTORY_POINTS];
  float long_smooth_seconds_;
  
  void blend_trajectory(const E2EPlan& e2e, const E2EPlan& planplus,
                        float blend_weight, float* output);
  void smooth_longitudinal(float* accel, float* vel, size_t count);
  float lerp(float a, float b, float t) const;
};

class FeatureMemoryBuffer {
public:
  FeatureMemoryBuffer(size_t feature_dim, size_t max_history);
  ~FeatureMemoryBuffer();
  
  void add_feature(const float* feature);
  void get_contextual_feature(const float* current, float* output);
  void reset();
  
  size_t size() const { return history_.size(); }
  bool empty() const { return history_.empty(); }

private:
  size_t feature_dim_;
  size_t max_history_;
  std::vector<std::vector<float>> history_;
  std::vector<float> W_q_;
  std::vector<float> W_k_;
  std::vector<float> W_v_;
  
  void scaled_dot_product_attention(const float* query, const float* keys,
                                    const float* values, size_t num_keys,
                                    float* output);
  void softmax(float* arr, size_t count);
};

class E2ESafetyChecker {
public:
  E2ESafetyChecker();
  
  enum class SafetyLevel {
    SAFE,
    CAUTION,
    DANGER,
    CUT_TORQUE
  };
  
  SafetyLevel check_path_sanity(const E2EPlan& plan, float road_edge_angle,
                                 float v_ego);
  
  void set_deviation_threshold(float degrees);
  void set_cutoff_timeout_ms(uint64_t ms);
  
private:
  float deviation_threshold_deg_;
  uint64_t cutoff_timeout_ms_;
  uint64_t danger_frame_count_;
  
  bool is_path_deviation_excessive(const E2EPlan& plan, float road_edge_angle);
};

class VisionOnlyTrafficDetector {
public:
  VisionOnlyTrafficDetector();
  
  enum class TrafficSignal {
    NONE,
    STOP_SIGN,
    RED_LIGHT,
    YELLOW_LIGHT,
    GREEN_LIGHT
  };
  
  struct Detection {
    TrafficSignal type;
    float distance;
    float probability;
    uint64_t timestamp;
  };
  
  void update(const float* model_uncertainty, const float* disengage_probs,
              size_t num_points, float v_ego);
  
  Detection get_detection() const;
  float get_precharge_brake_probability() const;
  
  static constexpr float kStopSignUrgencyThreshold = 0.7f;
  static constexpr float kRedLightUrgencyThreshold = 0.65f;
  static constexpr float kPredictiveBrakeLookaheadSec = 3.0f;

private:
  TrafficSignal current_signal_;
  float signal_distance_;
  float signal_probability_;
  uint64_t last_detection_time_;
  float precharge_brake_prob_;
  
  float predict_stop_probability(const float* disengage_probs, size_t count,
                                  float v_ego) const;
};

}
#endif
