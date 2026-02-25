/**
 * @file e2e_safety.h
 * @brief Hardware-Level E2E Safety Monitor for Panda Board
 *
 * This module provides real-time safety monitoring of E2E (End-to-End) model outputs
 * at the hardware level. It monitors path deviation and enforces safety limits by
 * cutting torque if the E2E path deviates excessively from expected road geometry.
 *
 * Copyright (c) 2021- sunnypilot
 * Licensed under the MIT License
 */
#ifndef E2E_SAFETY_H
#define E2E_SAFETY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define E2E_SAFETY_MAX_PATH_POINTS 33
#define E2E_SAFETY_DEVIATION_HYSTERESIS_FRAMES 3
#define E2E_SAFETY_ROAD_EDGE_HISTORY_SIZE 10

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

#ifdef __cplusplus
}
#endif

#endif
