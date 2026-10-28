#pragma once

#include "lemlib/api.hpp"
#include "api.h"
#include "ezlib/ez_compat.hpp"
#include "ezlib/pid_tuner.hpp"
#include "liblvgl/lvgl.h"

// 底盤／EzDrive／PID：於 main.cpp 定義
extern lemlib::Chassis chassis;
extern ez_compat::EzDrive ezPID;
extern ez_compat::PidTuner pidTuner;
extern lemlib::TrackingWheel vertical;
extern lemlib::TrackingWheel horizontal;

// 動畫幀（flash）：src/assets/robotXX.c；更換圖片時修改對應 .c，並視需要增減 extern
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

extern pros::Controller controller;
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup leftMotors;
extern pros::Imu imu;

// ----- 以下為 header-only 硬體：變更 port／ADI 請修改此處 -----
// 距離感測器
inline pros::Distance dist_front(6);
inline pros::Distance dist_back(21);
inline pros::Distance dist_left(3);
inline pros::Distance dist_right(9);

// 結構馬達（intake／roller；溫度對應見 dash_config MCH）
inline pros::Motor left_intake(20);
inline pros::Motor right_intake(-17); // 負數 = 反轉
inline pros::Motor low_roller(18);
inline pros::Motor high_roller(19);

inline pros::MotorGroup intake_motors({20, -17});

inline pros::adi::Pneumatics arm('A', false);
inline pros::adi::Pneumatics cover('D', false);
inline pros::adi::Pneumatics hook('B', false);
inline pros::adi::Pneumatics matchload('C', false);
