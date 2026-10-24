#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pid_tuner.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>

bool autonMode = false;

bool odomIsUp = false;
bool odomUpRunning = false;

// =======================================================
// Offsets (inch) 
// =======================================================
const SensorOffset OFF_FRONT = { -5.31,  5.39 };
const SensorOffset OFF_BACK  = { -2.95, -6.81 };
const SensorOffset OFF_LEFT  = { -5.94,  3.50 };
const SensorOffset OFF_RIGHT = {  5.90,  3.50 };

rd::Selector selector({
    {"Left Auton", &leftAuton, "", 100},   
    {"Left Final", &leftAutonFinal, "", 100},
    {"Right Auton", &rightAuton, "", 100}, 
    {"Right Auton V2", &rightAutonV2, "", 100},
    {"AWP", &soloAWP, "", 100},
    {"Testing", &testing, "", 100},
    {"Skills", &skill},
});

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// motor groups
pros::MotorGroup rightMotors({11, 12, 13},	pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup leftMotors({-14, -15, -16}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(5);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 17,not reversed
pros::Rotation horizontalEnc(-21);
// vertical tracking wheel encoder. Rotation sensor, port 16, not reversed
pros::Rotation verticalEnc(10);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -5.39);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2,-0.708);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.45, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     8, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  8, // minimum output where drivetrain will move out of 127
                                  1 // expo curve gain
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(6.71, // proportional gain (kP)
                                              0.0849, // integral gain (kI)
                                              20.5, // derivative gain (kD)
                                              3, // anti windup
                                              0.5, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3.6, // proportional gain (kP)
                                              0.3, // integral gain (kI)
                                              26.95, // derivative gain (kD)
                                              5, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttleCurve, &steerCurve);

// Tank drive: do NOT also construct pros::Motor on the same ports as leftMotors/rightMotors.
// Duplicate handles (e.g. Motor(14) vs MotorGroup{-14}) fight reverse/encoder state and
// can make ezPID.drive(+12) unable to move forward. xPose motors stay nullptr (x-drive off).
ez_compat::EzDrive ezPID(chassis, drivetrain);
ez_compat::PidTuner pidTuner(ezPID);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    //pros::lcd::initialize(); // initialize brain screen
    /*
    imu.reset(); // reset imu to 0 degrees
    pros::delay(2000); // wait for imu to finish calibrating
    */
    //leftMotors.set_encoder_units_all(pros::MotorUnits::degrees);
    //rightMotors.set_encoder_units_all(pros::MotorUnits::degrees);

    chassis.calibrate(); // calibrate sensors
    pros::delay(500);
    ezPID.set_x_drive(false);

    // 設定是否使用追蹤輪
    //ezPID.trackingWheels(true);
    // 馬達 Cartridge 在 LemLib Drivetrain 配置中設定
    // [直行 PID] 控制前進後退的距離準確度
    // P: 加大會更快到達，但過大會震盪
    // D: 加大會減少震盪 (煞車)
    // Start I: 當誤差小於此數值時開始積分 (通常設 0 除非有靜態阻力)
    ezPID.set_drive_constants(8.25, 0, 42, 0); 

    // [直行修正 PID] 控制直行時 "多用力" 修正歪掉的角度
    // 這個通常 P 不需要太大，只要能稍微修正回來即可
    ezPID.set_heading_constants(4, 0, 10, 0);

    // [轉向 PID] 控制原地旋轉
    ezPID.set_turn_constants(4.1, 0, 30, 15);

    // --- 4. 設定 Slew Rate (加速度限制) ---
    // 數值愈小，起動愈溫和。設 255 代表不限制。
    ezPID.set_drive_slew(12); // 直行加速度限制 (建議 8-15)
    ezPID.set_turn_slew(255);   // 轉向加速度限制 (建議 3-8)

    // Exit: small band, small settle ms, big band, big settle ms, extra slow-time ms.
    // Last argument is NOT a derivative-only exit. 0 disables that extra timer.
    // Drive bands are inches. Turn bands are degrees.
    ezPID.set_drive_exit_conditions(0.5, 100, 2, 500, 0);
    ezPID.set_turn_exit_conditions(1, 100, 3, 250, 0);
    ezPID.set_drive_velocity_threshold(0.10);   // inches per 10ms
    ezPID.set_turn_velocity_threshold(0.25);    // degrees per 10ms
    ezPID.set_heading_velocity_threshold(0.25); // degrees per 10ms
    // Compact PID page only (full two-column UI OOMs 32KB LVGL → pc:3807204).
    pidTuner.start();

    //auto_init_universal(20,USE_BACK | USE_LEFT);
    //auto_init_universal(0); // start_theta_field = 0 (facing north)
    //pros::delay(100);

    //lv_example_tabview_2();
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    /*
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
            
            
        
            pros::delay(50); // delay to save resources
        }
    });
    
    */
   
    // UI 任務（圖片動畫 + controller 文字分頁）見 customs/monitor.cpp
    pros::Task uiTask(uiTask_fn);

    pros::Task intakeTask([]{
        while(true) {
            updateIntakeFSM();
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {
    
}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {
    //selector.focus();
}

// get a path used for pure pursuit
// this needs to be put outside a function
//ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
bool autoStart = false;

void autonomous() { 
    chassis.setPose(0, 0, 0);
    ezPID.reset_pid();
    selector.run_auton();
}

/**
 * Runs in driver control
 */
void opcontrol() {
    console.focus();
    pros::Task my_telemetry_task(telemetry_fn);


    while (true) {
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

        // Field control: tear down Anim + PID pages once (see telemetry_fn edge).
        const bool on_field = pros::competition::is_connected();
        if (on_field) {
            pidTuner.force_off();
            autoChange = false;
        }

        // Open/handle tuner BEFORE pneumaticsBtn — that helper calls
        // get_digital_new_press(X/A/Y) and would swallow exit / Print / A-drive.
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
                chassis.arcade(leftY, (leftX * 0.75));
            }
        }

        if (odomIsUp && !odomUpRunning) {
            odomUpRunning = true;
            matchload.set_value(true );
            //ekf_stop();
        }

        if (!pidTuner.controller_mode() && !pros::competition::is_connected()) {
            int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            if (displayMenu < 0) {
                displayMenu = 2;
            }
            if (displayMenu > 2) {
                displayMenu = 0;
            }
            if (rightX > 100) {
                ++displayMenu;
                pros::delay(300);
            }else if (rightX < -100) {
                --displayMenu;
                pros::delay(300);
            }

            

            if(controller.get_digital(DIGITAL_LEFT)) {
                autoChange = false;
                selector.prev_auton();
                selector.focus();
                ezPID.set_pose(0, 0, 0);
                ezPID.reset_pid();
                pros::delay(300); // delay to prevent multiple presses
            }
            if(controller.get_digital(DIGITAL_RIGHT)) {
                autoChange = false;
                selector.next_auton();
                selector.focus();
                ezPID.set_pose(0, 0, 0);
                ezPID.reset_pid();
                pros::delay(300); // delay to prevent multiple presses
            }

            if (controller.get_digital(DIGITAL_B) && controller.get_digital(DIGITAL_DOWN)) {
                console.focus();
                autonomous();
            }
            if (controller.get_digital(DIGITAL_Y) && controller.get_digital(DIGITAL_DOWN)) {                
                chassis.moveToPose(0,0,0,2000,{false,3,0.2,100});
                //chassis.waitUntilDone();
                //wallReset('X', -40, 0, 1500);
                //chassis.setPose(getRealPose(0, 17.8, dist_left, -1), getRealPose(0, 33.4, dist_back, -1), 0);
            }
            if(controller.get_digital_new_press(DIGITAL_UP)) {
                // 1. 強制停止當前任務
                ezPID.cancel_movement(); 
                // 2. 重置 PID 內部數據與 current_traveled
                ezPID.reset_pid(); 
                // 3. 震動一下提醒 Reset 成功
                controller.rumble("."); 
                pros::delay(50); 
            }
            if(controller.get_digital(DIGITAL_DOWN)) {
                //chassis.setPose(0, 0, 0);
                console.focus();
                autoChange = false;
            }            
        }        
        
        if (pidTuner.controller_mode()) {
            // Intake shoulders reserved: L1/R1 turn, L2/R2 = 10spin CCW/CW.
        } else if(controller.get_digital(DIGITAL_R1)){
            setIntakeState(IntakeState::INTAKE);

        }else if(controller.get_digital(DIGITAL_R2)){
            setIntakeState(IntakeState::REVERSE);
        }else if(controller.get_digital(DIGITAL_L1)){
            setIntakeState(IntakeState::L2);
        }else if(controller.get_digital(DIGITAL_L2)){
            setIntakeState(IntakeState::L1);
        }else{
            setIntakeState(IntakeState::STOP);
        }
        
        if (!on_field && !pidTuner.controller_mode() && controller.get_digital(DIGITAL_UP)) {
            // Starts robot01–10 anim (single view + lv_img_set_src in monitor.cpp).
            autoChange = true;
        }
        pros::delay(20); // delay to save resources
    }
}