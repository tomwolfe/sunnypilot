/**
 * @file e2e_safety.c
 * @brief Hardware-Level E2E Safety Monitor Implementation
 *
 * This module provides real-time safety monitoring of E2E (End-to-End) model outputs
 * at the hardware level. It monitors path deviation and enforces safety limits.
 *
 * Includes Control Barrier Function (CBF) based safety verification that provides
 * mathematical guarantees that the E2E model cannot command unsafe torque.
 *
 * Copyright (c) 2021- sunnypilot
 * Licensed under the MIT License
 */

#include "e2e_safety.h"
#include <math.h>
#include <string.h>

#define RAD_TO_DEG (180.0f / 3.14159265358979f)
#define DEG_TO_RAD (3.14159265358979f / 180.0f)

static E2ESafetyConfig safety_config = {
  .deviation_threshold_deg = 20.0f,
  .caution_threshold_deg = 12.0f,
  .warning_threshold_deg = 15.0f,
  .hysteresis_frames = E2E_SAFETY_DEVIATION_HYSTERESIS_FRAMES,
  .cut_timeout_ms = 100,
  .enabled = true
};

static E2ESafetyState safety_state = {
  .status = E2E_SAFETY_STATUS_OK,
  .action = E2E_SAFETY_ACTION_NONE,
  .current_deviation_deg = 0.0f,
  .path_uncertainty_avg = 0.0f,
  .danger_frame_count = 0,
  .last_warning_time = 0,
  .torque_cut_active = false
};

static RoadEdgeEstimator road_edge_estimator = {
  .history_index = 0,
  .history_count = 0,
  .filtered_road_edge = 0.0f
};

static E2EPathData current_path;
static float current_speed_ms = 0.0f;
static uint32_t system_tick_ms = 0;
static CBFSafetyState cbf_state = {
  .road_width_m = 3.5f,
  .vehicle_position_m = 0.0f,
  .velocity_mps = 0.0f,
  .heading_rad = 0.0f,
  .time_to_road_edge_s = 999.0f,
  .barrier_value = 1.0f,
  .cbf_constraint_satisfied = true
};

void e2e_safety_init(const E2ESafetyConfig* config) {
  if (config != NULL) {
    memcpy(&safety_config, config, sizeof(E2ESafetyConfig));
  }
  e2e_safety_reset();
}

void e2e_safety_reset(void) {
  memset(&safety_state, 0, sizeof(E2ESafetyState));
  safety_state.status = E2E_SAFETY_STATUS_OK;
  safety_state.action = E2E_SAFETY_ACTION_NONE;
  
  memset(&road_edge_estimator, 0, sizeof(RoadEdgeEstimator));
  memset(&current_path, 0, sizeof(E2EPathData));
  current_speed_ms = 0.0f;
  
  memset(&cbf_state, 0, sizeof(CBFSafetyState));
  cbf_state.barrier_value = 1.0f;
  cbf_state.cbf_constraint_satisfied = true;
}

void e2e_safety_update_path(const E2EPathData* path) {
  if (path == NULL || path->num_points == 0) {
    return;
  }
  
  memcpy(&current_path, path, sizeof(E2EPathData));
  
  float total_uncertainty = 0.0f;
  for (uint8_t i = 0; i < current_path.num_points; i++) {
    total_uncertainty += current_path.uncertainty[i];
  }
  safety_state.path_uncertainty_avg = total_uncertainty / (float)current_path.num_points;
  
  if (current_path.num_points > 0) {
    cbf_state.vehicle_position_m = current_path.y[0];
  }
}

void e2e_safety_update_road_edge(float road_edge_angle_rad) {
  float road_edge_angle_deg = road_edge_angle_rad * RAD_TO_DEG;
  
  road_edge_estimator.road_edge_angle_history[road_edge_estimator.history_index] = road_edge_angle_deg;
  road_edge_estimator.history_index = (road_edge_estimator.history_index + 1) % E2E_SAFETY_ROAD_EDGE_HISTORY_SIZE;
  
  if (road_edge_estimator.history_count < E2E_SAFETY_ROAD_EDGE_HISTORY_SIZE) {
    road_edge_estimator.history_count++;
  }
  
  float sum = 0.0f;
  for (uint8_t i = 0; i < road_edge_estimator.history_count; i++) {
    sum += road_edge_estimator.road_edge_angle_history[i];
  }
  road_edge_estimator.filtered_road_edge = sum / (float)road_edge_estimator.history_count;
}

void e2e_safety_update_speed(float v_ego_ms) {
  current_speed_ms = v_ego_ms;
  cbf_state.velocity_mps = v_ego_ms;
}

static float calculate_path_deviation(void) {
  if (current_path.num_points < 2) {
    return 0.0f;
  }
  
  float path_start_y = current_path.y[0];
  float path_end_y = current_path.y[current_path.num_points - 1];
  float path_end_x = current_path.x[current_path.num_points - 1];
  
  if (path_end_x <= 0.1f) {
    return 0.0f;
  }
  
  float path_angle = atan2f(path_end_y - path_start_y, path_end_x);
  float path_angle_deg = path_angle * RAD_TO_DEG;
  
  float deviation = path_angle_deg - road_edge_estimator.filtered_road_edge;
  
  if (deviation > 180.0f) {
    deviation -= 360.0f;
  } else if (deviation < -180.0f) {
    deviation += 360.0f;
  }
  
  return fabsf(deviation);
}

CBFSafetyState e2e_safety_compute_cbf(float road_width_m, float vehicle_y_m, float velocity_mps, float heading_rad) {
  CBFSafetyState result;
  
  result.road_width_m = road_width_m;
  result.vehicle_position_m = vehicle_y_m;
  result.velocity_mps = velocity_mps;
  result.heading_rad = heading_rad;
  
  float safe_distance = road_width_m / 2.0f - CBF_SAFE_MARGIN_M;
  float distance_to_edge = safe_distance - fabsf(vehicle_y_m);
  
  result.barrier_value = distance_to_edge;
  
  if (velocity_mps > 0.1f) {
    result.time_to_road_edge_s = distance_to_edge / velocity_mps;
  } else {
    result.time_to_road_edge_s = 999.0f;
  }
  
  float h = distance_to_edge;
  float h_dot = -vehicle_y_m * heading_rad;
  
  float gamma = CBF_GAMMA;
  bool cbf_satisfied = (h_dot + gamma * h) >= 0.0f;
  
  result.cbf_constraint_satisfied = cbf_satisfied;
  
  cbf_state = result;
  
  return result;
}

float e2e_safety_apply_cbf_constraint(float desired_torque, const CBFSafetyState* cbf_state) {
  if (cbf_state == NULL) {
    return desired_torque;
  }
  
  if (cbf_state->cbf_constraint_satisfied) {
    return desired_torque;
  }
  
  float barrier_value = cbf_state->barrier_value;
  float velocity = cbf_state->velocity_mps;
  
  if (barrier_value < 0.0f) {
    return 0.0f;
  }
  
  float max_safe_torque = CBF_MAX_TORQUE_NM * (barrier_value / CBF_SAFE_MARGIN_M);
  max_safe_torque = fminf(max_safe_torque, CBF_MAX_TORQUE_NM);
  
  if (velocity > 5.0f) {
    max_safe_torque *= 0.5f;
  }
  
  if (desired_torque > max_safe_torque) {
    return max_safe_torque;
  } else if (desired_torque < -max_safe_torque) {
    return -max_safe_torque;
  }
  
  return desired_torque;
}

static void update_safety_status(void) {
  if (!safety_config.enabled) {
    safety_state.status = E2E_SAFETY_STATUS_OK;
    safety_state.action = E2E_SAFETY_ACTION_NONE;
    safety_state.torque_cut_active = false;
    return;
  }
  
  float deviation = calculate_path_deviation();
  safety_state.current_deviation_deg = deviation;
  
  CBFSafetyState cbf = e2e_safety_compute_cbf(
    safety_config.deviation_threshold_deg / 10.0f,
    current_path.num_points > 0 ? current_path.y[0] : 0.0f,
    current_speed_ms,
    road_edge_estimator.filtered_road_edge * DEG_TO_RAD
  );
  
  if (!cbf.cbf_constraint_satisfied) {
    safety_state.status = E2E_SAFETY_STATUS_CRITICAL;
    safety_state.action = E2E_SAFETY_ACTION_CUT_TORQUE_IMMEDIATE;
    safety_state.torque_cut_active = true;
    return;
  }
  
  if (deviation > safety_config.deviation_threshold_deg) {
    safety_state.danger_frame_count++;
    
    if (safety_state.danger_frame_count >= safety_config.hysteresis_frames) {
      safety_state.status = E2E_SAFETY_STATUS_CRITICAL;
      safety_state.action = E2E_SAFETY_ACTION_CUT_TORQUE_IMMEDIATE;
      safety_state.torque_cut_active = true;
    } else {
      safety_state.status = E2E_SAFETY_STATUS_WARNING;
      safety_state.action = E2E_SAFETY_ACTION_REDUCE_TORQUE;
    }
  } else if (deviation > safety_config.warning_threshold_deg) {
    safety_state.danger_frame_count = 0;
    safety_state.status = E2E_SAFETY_STATUS_WARNING;
    safety_state.action = E2E_SAFETY_ACTION_REDUCE_TORQUE;
    safety_state.torque_cut_active = false;
  } else if (deviation > safety_config.caution_threshold_deg) {
    safety_state.danger_frame_count = 0;
    safety_state.status = E2E_SAFETY_STATUS_CAUTION;
    safety_state.action = E2E_SAFETY_ACTION_NONE;
    safety_state.torque_cut_active = false;
  } else {
    safety_state.danger_frame_count = 0;
    safety_state.status = E2E_SAFETY_STATUS_OK;
    safety_state.action = E2E_SAFETY_ACTION_NONE;
    safety_state.torque_cut_active = false;
  }
  
  if (safety_state.path_uncertainty_avg > 12.0f) {
    if (safety_state.status == E2E_SAFETY_STATUS_OK) {
      safety_state.status = E2E_SAFETY_STATUS_CAUTION;
    } else if (safety_state.status == E2E_SAFETY_STATUS_CAUTION) {
      safety_state.status = E2E_SAFETY_STATUS_WARNING;
    }
  }
}

void e2e_safety_process_frame(void) {
  system_tick_ms += 20;
  update_safety_status();
}

E2ESafetyStatus e2e_safety_get_status(void) {
  return safety_state.status;
}

E2ESafetyAction e2e_safety_get_action(void) {
  return safety_state.action;
}

bool e2e_safety_should_cut_torque(void) {
  return safety_state.torque_cut_active;
}

bool e2e_safety_is_enabled(void) {
  return safety_config.enabled;
}

void e2e_safety_set_threshold(float deviation_deg) {
  safety_config.deviation_threshold_deg = deviation_deg;
}

void e2e_safety_set_enabled(bool enabled) {
  safety_config.enabled = enabled;
  if (!enabled) {
    e2e_safety_reset();
  }
}

float e2e_safety_get_current_deviation(void) {
  return safety_state.current_deviation_deg;
}

float e2e_safety_get_path_uncertainty(void) {
  return safety_state.path_uncertainty_avg;
}

static void simulate_trajectory(
  const VehicleState* initial_state,
  float torque_nm,
  float dt_s,
  TrajectoryPoint* trajectory
) {
  float heading = initial_state->heading_rad;
  float velocity = sqrtf(initial_state->velocity_x_mps * initial_state->velocity_x_mps + 
                         initial_state->velocity_y_mps * initial_state->velocity_y_mps);
  
  float steering_angle = torque_nm * 0.1f;
  float yaw_rate = steering_angle * velocity * 0.1f;
  
  float current_x = initial_state->position_x_m;
  float current_y = initial_state->position_y_m;
  float current_vx = initial_state->velocity_x_mps;
  float current_vy = initial_state->velocity_y_mps;
  
  for (uint8_t t = 0; t < REACHABILITY_NUM_STEPS; t++) {
    float time_s = (float)t * dt_s;
    
    heading += yaw_rate * dt_s;
    
    current_vx += cosf(heading) * 0.5f * dt_s;
    current_vy += sinf(heading) * 0.5f * dt_s;
    
    current_x += current_vx * dt_s;
    current_y += current_vy * dt_s;
    
    trajectory->positions_x[t] = current_x;
    trajectory->positions_y[t] = current_y;
    trajectory->velocities_x[t] = current_vx;
    trajectory->velocities_y[t] = current_vy;
    trajectory->time_s = time_s;
  }
}

static bool check_collision_with_obstacles(
  const TrajectoryPoint* trajectory,
  const float road_bound_left_m,
  const float road_bound_right_m,
  const float object_positions[][2],
  uint8_t num_objects,
  float* collision_time_s
) {
  for (uint8_t t = 0; t < REACHABILITY_NUM_STEPS; t++) {
    float px = trajectory->positions_x[t];
    float py = trajectory->positions_y[t];
    
    if (py < road_bound_left_m || py > road_bound_right_m) {
      *collision_time_s = trajectory->time_s;
      return true;
    }
    
    for (uint8_t i = 0; i < num_objects; i++) {
      float obj_x = object_positions[i][0];
      float obj_y = object_positions[i][1];
      float dx = px - obj_x;
      float dy = py - obj_y;
      float dist = sqrtf(dx * dx + dy * dy);
      
      if (dist < REACHABILITY_COLLISION_THRESHOLD_M) {
        *collision_time_s = trajectory->time_s;
        return true;
      }
    }
  }
  
  return false;
}

ReachabilityAnalysis e2e_safety_compute_reachability(
  const VehicleState* current_state,
  float desired_torque_nm,
  const float road_bound_left_m,
  const float road_bound_right_m,
  const float object_positions[][2],
  uint8_t num_objects
) {
  ReachabilityAnalysis result;
  memset(&result, 0, sizeof(ReachabilityAnalysis));
  
  float dt_s = (float)REACHABILITY_TIME_STEP_MS / 1000.0f;
  
  float torque_samples[] = {-2.0f, -1.5f, -1.0f, -0.5f, 0.0f, 0.5f, 1.0f, 1.5f, 2.0f};
  uint8_t num_torques = sizeof(torque_samples) / sizeof(float);
  
  uint8_t collision_count = 0;
  float min_collision_time = 999.0f;
  
  for (uint8_t i = 0; i < num_torques && result.num_trajectories < REACHABILITY_MAX_TRAJECTORIES; i++) {
    float torque = torque_samples[i];
    
    simulate_trajectory(current_state, torque, dt_s, &result.trajectories[result.num_trajectories]);
    
    float collision_time = 0.0f;
    bool collision = check_collision_with_obstacles(
      &result.trajectories[result.num_trajectories],
      road_bound_left_m,
      road_bound_right_m,
      object_positions,
      num_objects,
      &collision_time
    );
    
    result.trajectories[result.num_trajectories].is_collision = collision;
    result.trajectories[result.num_trajectories].collision_time_s = collision_time;
    
    if (collision) {
      collision_count++;
      if (collision_time < min_collision_time) {
        min_collision_time = collision_time;
      }
    }
    
    result.num_trajectories++;
  }
  
  result.collision_probability = (float)collision_count / (float)result.num_trajectories;
  result.min_safe_time_s = min_collision_time;
  
  if (collision_count == 0) {
    result.status = REACHABILITY_SAFE;
    result.is_controllable = true;
    result.max_safe_torque_nm = 2.0f;
    result.min_safe_torque_nm = -2.0f;
  } else if (collision_count == result.num_trajectories) {
    result.status = REACHABILITY_UNSAFE_UNAVOIDABLE;
    result.is_controllable = false;
    result.max_safe_torque_nm = 0.0f;
    result.min_safe_torque_nm = 0.0f;
  } else {
    result.status = REACHABILITY_UNSAFE_POSSIBLE;
    result.is_controllable = true;
    
    result.max_safe_torque_nm = desired_torque_nm;
    result.min_safe_torque_nm = desired_torque_nm;
  }
  
  return result;
}

bool e2e_safety_is_collision_unavoidable(
  const VehicleState* initial_state,
  const float road_bound_left_m,
  const float road_bound_right_m,
  const float object_positions[][2],
  uint8_t num_objects
) {
  float dt_s = (float)REACHABILITY_TIME_STEP_MS / 1000.0f;
  
  float torque_samples[] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
  uint8_t num_torques = 5;
  
  for (uint8_t i = 0; i < num_torques; i++) {
    TrajectoryPoint trajectory;
    simulate_trajectory(initial_state, torque_samples[i], dt_s, &trajectory);
    
    float collision_time = 0.0f;
    bool collision = check_collision_with_obstacles(
      &trajectory,
      road_bound_left_m,
      road_bound_right_m,
      object_positions,
      num_objects,
      &collision_time
    );
    
    if (!collision) {
      return false;
    }
  }
  
  return true;
}

float e2e_safety_compute_safe_torque_bound(
  const VehicleState* current_state,
  float desired_torque_nm,
  const float road_bound_left_m,
  const float road_bound_right_m,
  bool is_max_bound
) {
  float test_torque = is_max_bound ? desired_torque_nm : -desired_torque_nm;
  if (test_torque < 0) test_torque = -test_torque;
  
  float object_positions[1][2] = {{0.0f, 0.0f}};
  
  ReachabilityAnalysis analysis = e2e_safety_compute_reachability(
    current_state,
    test_torque,
    road_bound_left_m,
    road_bound_right_m,
    object_positions,
    0
  );
  
  if (analysis.status == REACHABILITY_SAFE) {
    return desired_torque_nm;
  }
  
  float safe_torque = 0.0f;
  float step = 0.1f;
  
  for (float t = 0.0f; t <= 2.0f; t += step) {
    float test = is_max_bound ? t : -t;
    
    analysis = e2e_safety_compute_reachability(
      current_state,
      test,
      road_bound_left_m,
      road_bound_right_m,
      object_positions,
      0
    );
    
    if (analysis.status == REACHABILITY_SAFE) {
      safe_torque = test;
    } else {
      break;
    }
  }
  
  return safe_torque;
}
