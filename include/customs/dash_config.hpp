#pragma once

#include "customs/objDeclare.hpp"

#include <algorithm>
#include <cstdio>

// =============================================================================
// Dash 溫度對應表 — 若要變更 DRV／MCH 顯示之馬達，僅修改本檔。
//
// MotorGroup 之 index 0/1/2 = main.cpp 內 leftMotors／rightMotors 列表順序
// （並非 smart-port 編號）。
// =============================================================================

namespace dash_config {

// DRV 列：六個數值  「左前/左中/左後  |  右前/右中/右後」
inline void format_drive_temps(char* buf, std::size_t n, double* max_out = nullptr) {
    const double a = leftMotors.get_temperature(0);  // 左前
    const double b = leftMotors.get_temperature(1);  // 左中
    const double c = leftMotors.get_temperature(2);  // 左後
    const double d = rightMotors.get_temperature(0); // 右前
    const double e = rightMotors.get_temperature(1); // 右中
    const double f = rightMotors.get_temperature(2); // 右後
    std::snprintf(buf, n, "%.0f/%.0f/%.0f  |  %.0f/%.0f/%.0f", a, b, c, d, e, f);
    if (max_out != nullptr) *max_out = std::max({a, b, c, d, e, f});
}

// MCH 列（結構）：四個數值  「左 intake／右 intake  |  下／上 roller」
inline void format_mech_temps(char* buf, std::size_t n, double* max_out = nullptr) {
    const double a = left_intake.get_temperature();
    const double b = right_intake.get_temperature();
    const double c = low_roller.get_temperature();
    const double d = high_roller.get_temperature();
    std::snprintf(buf, n, "%.0f/%.0f  |  %.0f/%.0f", a, b, c, d);
    if (max_out != nullptr) *max_out = std::max({a, b, c, d});
}

} // namespace dash_config
