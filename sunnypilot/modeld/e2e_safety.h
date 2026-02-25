/**
 * @file e2e_safety.h
 * @brief Hardware-Level E2E Safety Monitor for Panda Board
 *
 * This module provides real-time safety monitoring of E2E (End-to-End) model outputs
 * at the hardware level. It monitors path deviation and enforces safety limits by
 * cutting torque if the E2E path deviates excessively from expected road geometry.
 *
 * Includes Control Barrier Function (CBF) based safety guarantees for mathematical
 * verification of safety constraints.
 *
 * INCLUDES REACHABILITY ANALYSIS:
 * - Instead of just checking if the CURRENT torque is safe, the system checks
 *   if ANY possible sequence of torques over the next 200ms leads to an
 *   unavoidable collision (Forward Reachability)
 * - Also checks if there exists ANY safe trajectory from current state
 *   (Backward Reachability / Controllability)
 *
 * Copyright (c) 2021- sunnypilot
 * Licensed under the MIT License
 */
#ifndef E2E_SAFETY_H
#define E2E_SAFETY_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define E2E_SAFETY_MAX_PATH_POINTS 33
#define E2E_SAFETY_DEVIATION_HYSTERESIS_FRAMES 3
#define E2E_SAFETY_ROAD_EDGE_HISTORY_SIZE 10

#define CBF_SAFE_MARGIN_M 0.5
#define CBF_MAX_TORQUE_NM 2.0f
#define CBF_GAMMA 0.5f

#define REACHABILITY_HORIZON_MS 200
#define REACHABILITY_TIME_STEP_MS 10
#define REACHABILITY_NUM_STEPS (REACHABILITY_HORIZON_MS / REACHABILITY_TIME_STEP_MS)
#define REACHABILITY_MAX_TRAJECTORIES 20
#define REACHABILITY_COLLISION_THRESHOLD_M 0.5

typedef struct {
  float x[E2E_SAFETY_MAX_PATH_POINTS];
  float y[E2E_SAFETY_MAX_PATH_POINTS];
  float uncertainty[E2E_SAFETY_MAX_PATH_POINTS];
  uint8_t num_points;
} E2EPathData;

typedef enum {
  E2E_SAFETY_STATUS_OK = 0,
  E2E_SAFETY_STATUS_CAUTION,
  E2E_SAFETY_STATUS_WARNING,
  E2E_SAFETY_STATUS_CRITICAL
} E2ESafetyStatus;

typedef enum {
  E2E_SAFETY_ACTION_NONE = 0,
  E2E_SAFETY_ACTION_REDUCE_TORQUE,
  E2E_SAFETY_ACTION_CUT_TORQUE_IMMEDIATE,
  E2E_SAFETY_ACTION_ENGAGE_BRAKE
} E2ESafetyAction;

typedef enum {
  REACHABILITY_SAFE = 0,
  REACHABILITY_UNSAFE_UNAVOIDABLE,
  REACHABILITY_UNSAFE_POSSIBLE,
  REACHABILITY_UNKNOWN
} ReachabilityStatus;

typedef struct {
  float deviation_threshold_deg;
  float caution_threshold_deg;
  float warning_threshold_deg;
  uint16_t hysteresis_frames;
  uint32_t cut_timeout_ms;
  bool enabled;
} E2ESafetyConfig;

typedef struct {
  E2ESafetyStatus status;
  E2ESafetyAction action;
  float current_deviation_deg;
  float path_uncertainty_avg;
  uint16_t danger_frame_count;
  uint32_t last_warning_time;
  bool torque_cut_active;
} E2ESafetyState;

typedef struct {
  float road_edge_angle_history[E2E_SAFETY_ROAD_EDGE_HISTORY_SIZE];
  uint8_t history_index;
  uint8_t history_count;
  float filtered_road_edge;
} RoadEdgeEstimator;

typedef struct {
  float road_width_m;
  float vehicle_position_m;
  float velocity_mps;
  float heading_rad;
  float time_to_road_edge_s;
  float barrier_value;
  bool cbf_constraint_satisfied;
} CBFSafetyState;

typedef struct {
  float position_x_m;
  float position_y_m;
  float velocity_x_mps;
  float velocity_y_mps;
  float acceleration_x_mps2;
  float acceleration_y_mps2;
  float heading_rad;
  float yaw_rate_rps;
} VehicleState;

typedef struct {
  float positions_x[REACHABILITY_NUM_STEPS];
  float positions_y[REACHABILITY_NUM_STEPS];
  float velocities_x[REACHABILITY_NUM_STEPS];
  float velocities_y[REACHABILITY_NUM_STEPS];
  float time_s;
  bool is_collision;
  float collision_time_s;
} TrajectoryPoint;

typedef struct {
  TrajectoryPoint trajectories[REACHABILITY_MAX_TRAJECTORIES];
  uint8_t num_trajectories;
  ReachabilityStatus status;
  float collision_probability;
  float min_safe_time_s;
  bool is_controllable;
  float max_safe_torque_nm;
  float min_safe_torque_nm;
} ReachabilityAnalysis;

void e2e_safety_init(const E2ESafetyConfig* config);
void e2e_safety_reset(void);

void e2e_safety_update_path(const E2EPathData* path);
void e2e_safety_update_road_edge(float road_edge_angle_rad);
void e2e_safety_update_speed(float v_ego_ms);

E2ESafetyStatus e2e_safety_get_status(void);
E2ESafetyAction e2e_safety_get_action(void);
bool e2e_safety_should_cut_torque(void);
bool e2e_safety_is_enabled(void);

void e2e_safety_set_threshold(float deviation_deg);
void e2e_safety_set_enabled(bool enabled);

float e2e_safety_get_current_deviation(void);
float e2e_safety_get_path_uncertainty(void);

void e2e_safety_process_frame(void);

float e2e_safety_apply_cbf_constraint(float desired_torque, const CBFSafetyState* cbf_state);

CBFSafetyState e2e_safety_compute_cbf(float road_width_m, float vehicle_y_m, float velocity_mps, float heading_rad);

ReachabilityAnalysis e2e_safety_compute_reachability(
  const VehicleState* current_state,
  float desired_torque_nm,
  const float road_bound_left_m,
  const float road_bound_right_m,
  const float object_positions[][2],
  uint8_t num_objects
);

bool e2e_safety_is_collision_unavoidable(
  const VehicleState* initial_state,
  const float road_bound_left_m,
  const float road_bound_right_m,
  const float object_positions[][2],
  uint8_t num_objects
);

float e2e_safety_compute_safe_torque_bound(
  const VehicleState* current_state,
  float desired_torque_nm,
  const float road_bound_left_m,
  const float road_bound_right_m,
  bool is_max_bound
);

#ifdef __cplusplus
}
#endif

#endif
