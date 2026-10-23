#include "main.h"
#include "customs/monitor.hpp"
#include "pid_tuner.hpp"

#include <cstdio>
#include <cstring>
#include <string>

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

bool autoChange = false;
int displayMenu = 0;
float globalTime = 0;
float elapsedSec = 0;

// 合併所有 UI 相關任務到一個 Thread，減少競爭與紅屏風險
void uiTask_fn() {
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
