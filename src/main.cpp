#include "main.h"
#include "ezlib/auton_select.hpp"
#include "ezlib/telem_dash.hpp"
#include "ezlib/ui_pages.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/logger/logger.hpp"
#include "ezlib/pid_tuner.hpp"
#include "pros/apix.h"
#include <cmath>
#include <cstdio>
#include <cstring>

bool autonMode = false;

bool odomIsUp = false;
bool odomUpRunning = false;

// =======================================================
// 距離感測器相對底盤中心之偏移（吋）— 供 relocate／auton 使用
// =======================================================
const SensorOffset OFF_FRONT = { -5.31,  5.39 };
const SensorOffset OFF_BACK  = { -2.95, -6.81 };
const SensorOffset OFF_LEFT  = { -5.94,  3.50 };
const SensorOffset OFF_RIGHT = {  5.90,  3.50 };

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ----- 底盤馬達（變更 port／齒輪比請修改此處）-----
// 負數 = 反轉。順序 = dash_config 溫度 index 0/1/2。
pros::MotorGroup rightMotors({11, 12, 13}, pros::MotorGearset::blue);  // 右：前／中／後
pros::MotorGroup leftMotors({-14, -15, -16}, pros::MotorGearset::blue); // 左：前／中／後

pros::Imu imu(5); // IMU 埠號

// 追蹤輪硬體（可接入 OdomSensors；目前 LemLib odom 僅使用 IMU）
pros::Rotation horizontalEnc(-21); // 橫向，port 21 反轉
pros::Rotation verticalEnc(10);    // 縱向，port 10
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -5.39); // 直徑／偏移（吋）
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -0.708);

lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              11.45,                      // 輪距（吋）
                              lemlib::Omniwheel::NEW_325, // 3.25" omni
                              360,                       // 底盤 rpm
                              8                          // horizontal drift
);

// LemLib odom：目前僅使用 IMU（追蹤輪尚未接入）
lemlib::OdomSensors sensors(nullptr, // 縱向追蹤輪
                            nullptr, // 縱向 2
                            nullptr, // 橫向追蹤輪
                            nullptr, // 橫向 2
                            &imu);

// 遙控油門曲線（deadband／最小輸出／expo）
lemlib::ExpoDriveCurve throttleCurve(3, 8, 1);
// 遙控轉向曲線
lemlib::ExpoDriveCurve steerCurve(3, 8, 1);

// Chassis 建構所需之 placeholder。已 bind ezPID 後，moveTo*／turnTo*／pursuit 皆使用 ezPID，無須調整此處。
lemlib::ControllerSettings lateral_controller(0, 0, 0, 0, 0, 0, 0, 0, 0);
lemlib::ControllerSettings angular_controller(0, 0, 0, 0, 0, 0, 0, 0, 0);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttleCurve, &steerCurve);

// 請勿對 leftMotors／rightMotors 同一 port 再建立另一個 pros::Motor（會互相干擾 reverse／encoder）。
// xPose 馬達維持 nullptr（關閉 x-drive）。
ez_compat::EzDrive ezPID(chassis, drivetrain);
ez_compat::PidTuner pidTuner(ezPID);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    // USB COBS path is unsafe on this 4.2.1 build (PC 0x0390c572).
    // Mute LemLib logger before calibrate (IMU fail → infoSink → cout → crash;
    // that Buffer task has no name → brain shows CURRENT TASK: ".").
    pros::c::serctl(SERCTL_DISABLE_COBS, nullptr);
    lemlib::infoSink()->setLowestLevel(lemlib::Level::FATAL);
    lemlib::telemetrySink()->setLowestLevel(lemlib::Level::FATAL);

    // Controller first — no USB printf.
    controller.clear();
    pros::delay(50);
    controller.print(0, 0, "%-15.15s", "YKK EzDrive");
    pros::delay(50);
    controller.print(1, 0, "%-15.15s", "init...");
    pros::delay(50);
    controller.rumble(".");

    // SD breadcrumbs only (no stdout). Retries ~2s — card often late.
    boot_log::ensure_sd("pre_dash");
    boot_log::log("initialize:enter LV_MEM_CUSTOM=1 malloc-pool");
    controller.print(1, 0, "%-15.15s", "auton list");
    pros::delay(50);

    // Auton 清單（更名／增減 routine 請修改此處；以 LEFT／RIGHT 切換）
    boot_log::log("initialize:auton_select");
    auton_select::init({
        {"Left Auton", &leftAuton},
        {"Left Final", &leftAutonFinal},
        {"Right Auton", &rightAuton},
        {"Right Auton V2", &rightAutonV2},
        {"AWP", &soloAWP},
        {"Testing", &testing},
        {"Skills", &skill},
    });

    // initialize 不啟動 LVGL／Dash（Dash 由 opcontrol 之 telemetry_fn 啟動）
    controller.print(1, 0, "%-15.15s", "calibrating...");
    pros::delay(50);
    boot_log::log("initialize:before calibrate");
    chassis.calibrate(); // IMU 校準 — 機器人須靜止
    boot_log::log("initialize:after calibrate");

    controller.print(1, 0, "%-15.15s", "calibrated OK");
    pros::delay(50);
    controller.print(2, 0, "%-15.15s", "ready");
    controller.rumble("..");
    pros::delay(200);
    ezPID.set_x_drive(false); // false = tank；true 需額外 xPose 馬達

    // ----- EzDrive PID（供腦上 PID tuner／ezPID.drive／turn 使用）-----
    // set_*_constants(kP, kI, kD, startI)
    ezPID.set_drive_constants(8.25, 0, 42, 0);   // 直行距離
    ezPID.set_heading_constants(4, 0, 10, 0);    // 直行時航向糾偏
    ezPID.set_turn_constants(4.1, 0, 30, 15);    // 原地轉向

    // Slew：數值愈小起動愈溫和；255 = 不限制
    ezPID.set_drive_slew(12);
    ezPID.set_turn_slew(255);

    // Exit：小誤差帶、小 settle（ms）、大誤差帶、大 settle（ms）、額外慢速計時（0=關閉）
    // 直行單位為吋；轉向單位為度
    ezPID.set_drive_exit_conditions(0.5, 100, 2, 500, 0);
    ezPID.set_turn_exit_conditions(1, 100, 3, 250, 0);
    ezPID.set_drive_velocity_threshold(0.10);   // 吋／10ms
    ezPID.set_turn_velocity_threshold(0.25);    // 度／10ms
    ezPID.set_heading_velocity_threshold(0.25); // 度／10ms

    // 具名 task（避免 brain 顯示 CURRENT TASK: "."）
    boot_log::log("initialize:spawn uiTask");
    pros::Task uiTask(uiTask_fn, "uiTask");

    pros::Task intakeTask(
        [] {
            while (true) {
                updateIntakeFSM();
                pros::delay(20);
            }
        },
        "intakeTask");
    boot_log::log("initialize:exit");
}

/**
 * Runs while the robot is disabled
 */
void disabled() {
    
}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
//ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/** Runs the headless auton selected in auton_select. */
bool autoStart = false;

void autonomous() { 
    chassis.setPose(0, 0, 0);
    ezPID.reset_pid();
    auton_select::run();
}

/**
 * Runs in driver control
 */
void opcontrol() {
    boot_log::log("opcontrol:enter");
    // Dash 於 telemetry_fn settle 後建立；DOWN 可重新 focus
    pros::Task my_telemetry_task(telemetry_fn);
    boot_log::log("opcontrol:telemetry spawned");

    while (true) {
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

        // 連接場控：關閉 tuner／動畫（telemetry_fn 會拆除較重 UI）
        const bool on_field = pros::competition::is_connected();
        if (on_field) {
            pidTuner.force_off();
            autoChange = false;
        }

        // 先處理 tuner（X+A），再呼叫 pneumaticsBtn——否則 X／A／Y 會被其消耗
        static bool tuner_combo_was_down = false;
        const bool tuner_combo = controller.get_digital(DIGITAL_X) && controller.get_digital(DIGITAL_A);
        bool tuner_just_opened = false;
        if (!on_field && tuner_combo && !tuner_combo_was_down && !pidTuner.controller_mode()) {
            pidTuner.toggle_controller_mode();
            tuner_just_opened = true;
        }
        tuner_combo_was_down = tuner_combo;

        if (pidTuner.controller_mode() && !tuner_just_opened) {
            pidTuner.handle_controller(controller);
        } else if (!pidTuner.controller_mode()) {
            pneumaticsBtn();
            if (!pidTuner.suppress_arcade()) {
                int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
                int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
                chassis.arcade(leftY, (leftX * 0.75)); // 0.75 = 轉向比例，可調整
            }
        }

        if (odomIsUp && !odomUpRunning) {
            odomUpRunning = true;
            matchload.set_value(true);
        }

        // 練習模式按鍵綁定（場控或 tuner 啟用時不執行此段）
        if (!pidTuner.controller_mode() && !pros::competition::is_connected()) {
            int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            if (displayMenu < 0) displayMenu = 2;
            if (displayMenu > 2) displayMenu = 0;
            if (rightX > 100) {
                ++displayMenu; // 右搖桿切換手掣顯示頁
                pros::delay(300);
            } else if (rightX < -100) {
                --displayMenu;
                pros::delay(300);
            }

            // LEFT／RIGHT：切換 auton（不會清除 pose）
            if (controller.get_digital(DIGITAL_LEFT)) {
                autoChange = false;
                auton_select::prev();
                pros::delay(300);
            }
            if (controller.get_digital(DIGITAL_RIGHT)) {
                autoChange = false;
                auton_select::next();
                pros::delay(300);
            }

            // B+DOWN：手動執行目前選中之 auton
            if (controller.get_digital(DIGITAL_B) && controller.get_digital(DIGITAL_DOWN)) {
                ui_pages::show_dash();
                autonomous();
            }
            // Y+DOWN：moveToPose 返回 (0,0,0)
            if (controller.get_digital(DIGITAL_Y) && controller.get_digital(DIGITAL_DOWN)) {
                chassis.moveToPose(0, 0, 0, 2000, {false, 3, 0.2, 100});
            }
            // UP 短按：取消運動／reset PID（按住則於下方啟動動畫）
            if (controller.get_digital_new_press(DIGITAL_UP)) {
                ezPID.cancel_movement();
                ezPID.reset_pid();
                controller.rumble(".");
                pros::delay(50);
            }
            // DOWN：清除 traveled 並返回 Dash（非 B／Y 組合鍵）
            if (controller.get_digital_new_press(DIGITAL_DOWN)) {
                if (!controller.get_digital(DIGITAL_B) && !controller.get_digital(DIGITAL_Y)) {
                    ezPID.reset_pid();
                    ui_pages::show_dash();
                    autoChange = false;
                    controller.rumble(".");
                }
            }
        }

        if (pidTuner.controller_mode()) {
            // tuner 啟用時：L1／R1 轉角、L2／R2 = 10 圈，不作為 intake
        } else if (controller.get_digital(DIGITAL_R1)) {
            setIntakeState(IntakeState::INTAKE);
        } else if (controller.get_digital(DIGITAL_R2)) {
            setIntakeState(IntakeState::REVERSE);
        } else if (controller.get_digital(DIGITAL_L1)) {
            setIntakeState(IntakeState::L2);
        } else if (controller.get_digital(DIGITAL_L2)) {
            setIntakeState(IntakeState::L1);
        } else {
            setIntakeState(IntakeState::STOP);
        }

        // 按住 UP：啟動 robot01–10 動畫（幀數／速度見 monitor.cpp）
        if (!on_field && !pidTuner.controller_mode() && controller.get_digital(DIGITAL_UP)) {
            autoChange = true;
        }
        pros::delay(20);
    }
}