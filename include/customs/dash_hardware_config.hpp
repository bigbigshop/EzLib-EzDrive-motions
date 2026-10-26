#pragma once

// Single edit point for Brain Dash motor-temp tags and optional DIST slots.
// telem_dash iterates these tables — do not hardcode layout for 4/6/8 motors there.
//
// Drive motors: list LEFT then RIGHT (UI inserts '|' at the midpoint).
//   4-motor example: LF LB | RF RB  (drop LM/RM rows)
//   6-motor (default): LF LM LB | RF RM RB
//   8-motor example: LF LM LB LB2 | RF RM RB RB2
// Keep ports in sync with leftMotors/rightMotors in src/main.cpp for drivetrain.
//
// Mech motors: any non-drive motors to show under MECH °C (intake, rollers, …).
//
// DIST: set sensor to nullptr (or comment out the row) to hide that slot — no empty
// placeholder. Objects still live in objDeclare.hpp; this table only gates Dash UI.

#include "customs/objDeclare.hpp"

#include <cstddef>
#include <cstdint>

namespace dash_hw {

enum class TempGroup : std::uint8_t { Drive, Mech };

struct MotorTempSlot {
    const char* tag; // short label, e.g. "LF", "Ro1" — keep ≤3 chars for heap/layout
    std::int8_t port; // signed OK (reverse); temp read uses this port
    TempGroup group;
};

struct DistSlot {
    const char* tag;          // "F", "B", "L", "R", …
    pros::Distance* sensor;   // nullptr => omit from DIST card
};

// —— Edit motors here ——
inline constexpr MotorTempSlot kMotorTemps[] = {
    // Drive °C — default 6-motor mapping
    {"LF", -14, TempGroup::Drive},
    {"LM", -15, TempGroup::Drive},
    {"LB", -16, TempGroup::Drive},
    {"RF", 11, TempGroup::Drive},
    {"RM", 12, TempGroup::Drive},
    {"RB", 13, TempGroup::Drive},
    // Mech °C — intake + rollers
    {"InL", 20, TempGroup::Mech},
    {"InR", -17, TempGroup::Mech},
    {"RoL", 18, TempGroup::Mech},
    {"RoH", 19, TempGroup::Mech},
};

inline constexpr std::size_t kMotorTempCount = sizeof(kMotorTemps) / sizeof(kMotorTemps[0]);

// —— Edit distance sensors here ——
// Example: hide back sensor → {"B", nullptr}
inline DistSlot kDistSlots[] = {
    {"F", &dist_front},
    {"B", &dist_back},
    {"L", &dist_left},
    {"R", &dist_right},
};

inline constexpr std::size_t kDistSlotCount = sizeof(kDistSlots) / sizeof(kDistSlots[0]);

} // namespace dash_hw
