#pragma once

#include "customs/objDeclare.hpp"

#include <algorithm>
#include <cstdio>

// =============================================================================
// Dash temperature map — EDIT THIS FILE to change what DRV / MCH show.
//
// MotorGroup index 0/1/2 = order inside leftMotors / rightMotors in main.cpp
// (not the smart-port number).
// =============================================================================

namespace dash_config {

// DRV row: six numbers  "LF/LM/LB RF/RM/RB"
inline void format_drive_temps(char* buf, std::size_t n, double* max_out = nullptr) {
    const double a = leftMotors.get_temperature(0);  // left front
    const double b = leftMotors.get_temperature(1);  // left mid
    const double c = leftMotors.get_temperature(2);  // left back
    const double d = rightMotors.get_temperature(0); // right front
    const double e = rightMotors.get_temperature(1); // right mid
    const double f = rightMotors.get_temperature(2); // right back
    std::snprintf(buf, n, "%.0f/%.0f/%.0f  |  %.0f/%.0f/%.0f", a, b, c, d, e, f);
    if (max_out != nullptr) *max_out = std::max({a, b, c, d, e, f});
}

// MCH row: four numbers  "L_intake/R_intake  low/high roller"
inline void format_mech_temps(char* buf, std::size_t n, double* max_out = nullptr) {
    const double a = left_intake.get_temperature();
    const double b = right_intake.get_temperature();
    const double c = low_roller.get_temperature();
    const double d = high_roller.get_temperature();
    std::snprintf(buf, n, "%.0f/%.0f  |  %.0f/%.0f", a, b, c, d);
    if (max_out != nullptr) *max_out = std::max({a, b, c, d});
}

} // namespace dash_config
