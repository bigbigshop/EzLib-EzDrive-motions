#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pid_tuner.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>

bool autonMode = false;

bool autoChange = false;

bool odomIsUp = false;
bool odomUpRunning = false;

int displayMenu = 0;
float globalTime = 0;
float elapsedSec = 0;

// =======================================================
// Offsets (inch) 
// =======================================================
const SensorOffset OFF_FRONT = { -5.31,  5.39 };
const SensorOffset OFF_BACK  = { -2.95, -6.81 };
const SensorOffset OFF_LEFT  = { -5.94,  3.50 };
const SensorOffset OFF_RIGHT = {  5.90,  3.50 };

//rd::Image image(&kukuku, "Driver");

rd::Image img1(&robot01, "01");
rd::Image img2(&robot02, "02");
rd::Image img3(&robot03, "03");
rd::Image img4(&robot04, "04");
rd::Image img5(&robot05, "05");
rd::Image img6(&robot06, "06");
rd::Image img7(&robot07, "07");
rd::Image img8(&robot08, "08");
rd::Image img9(&robot09, "09");
rd::Image img10(&robot10, "10");


rd::Console console;
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

pros::Motor fl_m(14, pros::MotorGearset::blue);
pros::Motor bl_m(15, pros::MotorGearset::blue);
pros::Motor fr_m(11, pros::MotorGearset::blue);
pros::Motor br_m(12, pros::MotorGearset::blue);

// 更新 ezPID 初始化
ez_compat::EzDrive ezPID(chassis, drivetrain, &fl_m, &bl_m, &fr_m, &br_m);
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
   
    // 合併所有 UI 相關任務到一個 Thread，減少競爭與紅屏風險
    pros::Task uiTask([&]() {
        pros::delay(1000); // 等待系統完全啟動
        uint32_t lastAnimTime = 0;
        uint32_t lastControllerTime = 0;
        int animFrame = 1;

        while (true) {
            uint32_t now = pros::millis();
            
            // 1. 處理圖片動畫 (每 100ms 切換一幀)
            if (!pros::competition::is_connected()) {
                if (autoChange && (now - lastAnimTime > 100)) {
                    switch(animFrame) {
                        case 1: img1.focus(); break;
                        case 2: img2.focus(); break;
                        case 3: img3.focus(); break;
                        case 4: img4.focus(); break;
                        case 5: img5.focus(); break;
                        case 6: img6.focus(); break;
                        case 7: img7.focus(); break;
                        case 8: img8.focus(); break;
                        case 9: img9.focus(); break;
                        case 10: img10.focus(); break;
                        
                    }
                    animFrame = (animFrame % 10) + 1;
                    lastAnimTime = now;
                }
            }
            

            // 2. Controller text. Print a changed line only — never clear() on a timer.
            if (!pros::competition::is_connected() && (now - lastControllerTime > 60)) {
                static char pending[3][32] = {};
                static bool dirty[3] = {};
                static int next_line = 0;

                auto stage = [&](int line, const char* text) {
                    char buf[32];
                    std::snprintf(buf, sizeof(buf), "%-15.15s", text != nullptr ? text : "");
                    if (std::strncmp(pending[line], buf, sizeof(buf)) != 0) {
                        std::snprintf(pending[line], sizeof(pending[line]), "%s", buf);
                        dirty[line] = true;
                    }
                };

                char raw0[32];
                char raw1[32];
                char raw2[32];
                if (pidTuner.controller_mode()) {
                    pidTuner.format_controller(raw0, raw1, raw2);
                } else if (displayMenu == 0) {
                    elapsedSec = (now - globalTime) / 1000;
                    std::string autonName = selector.get_auton()
                        .value_or(rd::Selector::routine_t{"No auton selected", [](){}}).name;
                    std::snprintf(raw0, sizeof(raw0), "T:%4.0f", elapsedSec);
                    std::snprintf(raw1, sizeof(raw1), "B:%3.0f C:%3.0f",
                                  (double)controller.get_battery_capacity(),
                                  (double)pros::battery::get_capacity());
                    std::snprintf(raw2, sizeof(raw2), "%s", autonName.c_str());
                } else if (displayMenu == 1) {
                    std::snprintf(raw0, sizeof(raw0), "D:%7.2f", ezPID.get_traveled());
                    std::snprintf(raw1, sizeof(raw1), "T:%7.2f", chassis.getPose().theta);
                    std::snprintf(raw2, sizeof(raw2), " ");
                } else {
                    std::snprintf(raw0, sizeof(raw0), "X:%7.2f", chassis.getPose().x);
                    std::snprintf(raw1, sizeof(raw1), "Y:%7.2f", chassis.getPose().y);
                    std::snprintf(raw2, sizeof(raw2), "T:%7.2f", chassis.getPose().theta);
                }
                stage(0, raw0);
                stage(1, raw1);
                stage(2, raw2);

                for (int attempt = 0; attempt < 3; ++attempt) {
                    const int line = (next_line + attempt) % 3;
                    if (!dirty[line]) continue;
                    controller.print(line, 0, "%s", pending[line]);
                    dirty[line] = false;
                    next_line = (line + 1) % 3;
                    break;
                }
                lastControllerTime = now;
            }

            pros::delay(20);
        }
    });

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

int get_safe_conf(pros::Distance& s) {
    int32_t conf = s.get_confidence();
    // 如果斷線 (PROS_ERR) 或者數值異常，一律當 0 處理，保護 UI 唔會崩潰
    if (conf == PROS_ERR || conf < 0 || conf > 100) {
        return 0; 
    }
    return conf;
}

void telemetry_fn(){    
    float F_dist = 0.0f;
    float B_dist = 0.0f;
    float L_dist = 0.0f;
    float R_dist = 0.0f;
    
    // 🌟 新增：用來儲存安全的 confidence 數值
    int f_conf = 0, b_conf = 0, l_conf = 0, r_conf = 0;

    while(true){
        if (!pros::competition::is_connected()) {
            console.clear();
            console.printf("X: %5.2f,\t", chassis.getPose().x);
            console.printf("Y: %5.2f,\t", chassis.getPose().y);
            console.printf("Theta: %.2f\n", chassis.getPose().theta);
            console.println("");
            console.printf("LF: %.f,\t ", leftMotors.get_temperature(0));
            console.printf(" LM: %.f,\t", leftMotors.get_temperature(1));
            console.printf(" LB: %.f\n", leftMotors.get_temperature(2));

            console.printf("RF: %.f,\t ", rightMotors.get_temperature(0));
            console.printf("RM: %.f,\t", rightMotors.get_temperature(1));
            console.printf("RB: %.f\n", rightMotors.get_temperature(2));
            console.println("");

            console.printf("In_L: %.f,", left_intake.get_temperature());
            console.printf(" In_R: %.f,", right_intake.get_temperature());
            console.printf(" LRoll: %.f,", low_roller.get_temperature());
            console.printf(" HRoll: %.f\n", high_roller.get_temperature());
            console.println("");

            console.printf("Dist: %.2f\n", ezPID.get_traveled());
            console.println("");

            // 1. 獲取距離
            F_dist = get_kf_dist(dist_front);
            B_dist = get_kf_dist(dist_back);
            L_dist = get_kf_dist(dist_left);        
            R_dist = get_kf_dist(dist_right);
            
            // 2. 使用安全讀取
            f_conf = get_safe_conf(dist_front);
            b_conf = get_safe_conf(dist_back);
            l_conf = get_safe_conf(dist_left);
            r_conf = get_safe_conf(dist_right);

            // 3. 打印乾淨數值，Robodash 安全過關
            console.printf("F: %6.2f(%2d)\tB: %.2f(%2d)\n", F_dist, f_conf, B_dist, b_conf);
            console.printf("L: %6.2f(%2d)\tR: %.2f(%2d)\n", L_dist, l_conf, R_dist, r_conf);
        }
        pros::delay(50);
    }
}

/**
 * Runs in driver control
 */
void opcontrol() {
    console.focus();
    pros::Task my_telemetry_task(telemetry_fn);


    while (true) {
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        pneumaticsBtn();

        static bool tuner_combo_was_down = false;
        const bool tuner_combo = controller.get_digital(DIGITAL_X) && controller.get_digital(DIGITAL_A);
        bool tuner_just_opened = false;
        if (tuner_combo && !tuner_combo_was_down && !pidTuner.controller_mode()) {
            pidTuner.toggle_controller_mode();
            tuner_just_opened = true;
        }
        tuner_combo_was_down = tuner_combo;

        if (pidTuner.controller_mode() && !tuner_just_opened) {
            pidTuner.handle_controller(controller);
        } else if (!pidTuner.suppress_arcade()) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        chassis.arcade(leftY, (leftX*0.75));
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
                pros::delay(300); // delay to prevent multiple presses
            }
            if(controller.get_digital(DIGITAL_RIGHT)) {
                autoChange = false;
                selector.next_auton();
                selector.focus();
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
            // L1/L2 are tuner test moves while the controller tuner is open.
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
        
        if(!pidTuner.controller_mode() && controller.get_digital(DIGITAL_UP)) {
            autoChange = true;
            //pros::delay(300); // delay to prevent multiple presses
            //image.focus();
        }
        pros::delay(20); // delay to save resources
    }
}