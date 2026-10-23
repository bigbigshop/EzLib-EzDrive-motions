#pragma once
#include "api.h"
#include "lemlib/api.hpp"

// =======================================================
// Wall definition (for direction classification only)
// =======================================================
enum Wall {
    WALL_NORTH = 0, // +Y (ignore)
    WALL_EAST  = 1, // +X
    WALL_SOUTH = 2, // Y = 0
    WALL_WEST  = 3  // X = 0
};

// =======================================================
// Sensor mask (manual selection)
// idx: 0=left, 1=right, 2=back, 3=front
// =======================================================
enum SensorMask {
    USE_LEFT  = 1 << 0,
    USE_RIGHT = 1 << 1,
    USE_BACK  = 1 << 2,
    USE_FRONT = 1 << 3,

    USE_AUTO  = USE_LEFT | USE_RIGHT | USE_BACK | USE_FRONT
};

// =======================================================
// Sensor offset (robot local frame, inch)
// =======================================================
struct SensorOffset {
    double x; // +forward
    double y; // +left
};

extern const SensorOffset OFF_FRONT;
extern const SensorOffset OFF_BACK;
extern const SensorOffset OFF_LEFT;
extern const SensorOffset OFF_RIGHT;

// =======================================================
// API
// =======================================================
double get_filtered_dist(pros::Distance& s);
double get_smoothed_dist(pros::Distance& s);
double get_kf_dist(pros::Distance& s);
double get_median_ewma_dist(pros::Distance& s);

// Auto init:
// - start_theta_field_deg : field heading at start
// - sensor_mask           : which distance sensors are allowed
void auto_init_universal(double start_theta_field_deg, int sensor_mask = USE_AUTO);
void wallReset(char axis, int velocity, float theta, int timeout_ms = 1500);
double getRealPose(double expected, double golden, pros::Distance& sensor, int side);
// =======================================================