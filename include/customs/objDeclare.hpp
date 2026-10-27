#pragma once

#include "lemlib/api.hpp"
#include "api.h"
#include "ez_compat.hpp"
#include "pid_tuner.hpp"
#include "robodash/api.h"

//class Chassis;
extern lemlib::Chassis chassis;
extern ez_compat::EzDrive ezPID;
extern ez_compat::PidTuner pidTuner;
// Heap-allocated in initialize() AFTER boot_log SD probe (global ctor blanks 128KB).
extern rd::Selector* selector;
extern lemlib::TrackingWheel vertical;
extern lemlib::TrackingWheel horizontal;

extern const lv_img_dsc_t kukuku;
extern const lv_img_dsc_t robot01;
extern const lv_img_dsc_t robot02;
extern const lv_img_dsc_t robot03;
extern const lv_img_dsc_t robot04;
extern const lv_img_dsc_t robot05;
extern const lv_img_dsc_t robot06;
extern const lv_img_dsc_t robot07;
extern const lv_img_dsc_t robot08;
extern const lv_img_dsc_t robot09;
extern const lv_img_dsc_t robot10;
//extern const lv_img_dsc_t team_logo;

extern pros::Controller controller;
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup leftMotors;
extern pros::Imu imu;

inline pros::Distance dist_front(6);
inline pros::Distance dist_back(21);
inline pros::Distance dist_left(3);
inline pros::Distance dist_right(9);

inline pros::Motor left_intake(20);
inline pros::Motor right_intake(-17);
inline pros::Motor low_roller(18);
inline pros::Motor high_roller(19);

inline pros::MotorGroup intake_motors({20, -17});

inline pros::adi::Pneumatics arm('A', false);
inline pros::adi::Pneumatics cover('D', false);
inline pros::adi::Pneumatics hook('B', false);
inline pros::adi::Pneumatics matchload('C', false);