#include "main.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <deque>

// =======================================================
// Utility: check sensor mask
// =======================================================

const double field_size = 141.24;

inline bool sensor_allowed(int mask, int idx) {
    return (mask & (1 << idx)) != 0;
}

// =======================================================
// Median filtered distance (inch) - 原版
// =======================================================
double get_filtered_dist(pros::Distance& s) {
    std::vector<double> v;

    for (int i = 0; i < 5; i++) {
        double d = s.get_distance() / 25.4;  // 先轉做 inch
        if (d > 0.1 && d < 120.0) {  // 更合理嘅範圍
            v.push_back(d);
        }
    }

    if (v.size() < 2) return 999;  // 或者返回其他預設值

    std::sort(v.begin(), v.end());
    
    if (v.size() >= 3) {
        // 去掉最高同最低，各取平均
        double sum = 0;
        for (size_t i = 1; i < v.size() - 1; i++) {
            sum += v[i];
        }
        return sum / (v.size() - 2);
    }
    
    return v[v.size() / 2];  // 中位數
}

// =======================================================
// EWMA filtered distance
// =======================================================
double get_smoothed_dist(pros::Distance& s) {
    static double filtered_values[22] = {0.0};
    static bool initialized[22] = {false};
    const double alpha = 0.2;

    // 獲取感測器插在 Brain 上的 Port 號碼作為獨立 ID
    int port = s.get_port(); 
    if (port < 1 || port > 21) return 999; // 安全防護，避免陣列越界

    int32_t raw_dist = s.get_distance();
    
    if (raw_dist == PROS_ERR || raw_dist == 0) {
        return 999;
    }
    
    double raw = raw_dist / 25.4;

    // 排除極端無效數值
    if (raw < 0.1 || raw > 120.0) {
        return 999; 
    }

    // 針對該 Port 的感測器進行獨立更新
    if (!initialized[port]) {
        filtered_values[port] = raw;
        initialized[port] = true;
    } else {
        filtered_values[port] = alpha * raw + (1 - alpha) * filtered_values[port];
    }

    return filtered_values[port];
}

// =======================================================
// 1D Kalman Filter distance (inch) 
// =======================================================

double get_kf_dist(pros::Distance& s) {
    static double filtered_values[22] = {0.0};
    static double error_covariances[22] = {1.0};  
    static bool initialized[22] = {false};

    // 曾經成功駁過線?
    static bool ever_connected[22] = {false}; 

    const double process_noise = 0.2;   
    const double measurement_noise = 4.5; 
    const double MAX_CHANGE = 60.0;        
    int port = s.get_port();
    if (port < 1 || port > 21) return -1.0; 
    
    int32_t raw_dist = s.get_distance();
    
    if (raw_dist == PROS_ERR || raw_dist == 0) {
        // 無論邊種情況，都直接 return 999
        return 999; 
    }
    
    // 如果行到呢度，代表 Sensor 正常駁住，標記為「曾經連線」
    ever_connected[port] = true;
    
    double measurement = raw_dist / 25.4;
    
    if (measurement < 0.1 || measurement > 120.0) {
        // 如果只係暫時睇唔到嘢 (例如太遠)，直接 return 999
        return 999;
    }
    
    if (!initialized[port]) {
        filtered_values[port] = measurement;
        error_covariances[port] = 1.0;
        initialized[port] = true;
        return filtered_values[port];
    } 

    if (std::abs(measurement - filtered_values[port]) > MAX_CHANGE) {
        error_covariances[port] += process_noise; 
        return filtered_values[port];
    }
        
    double predicted_value = filtered_values[port];
    double predicted_error = error_covariances[port] + process_noise;
        
    double kalman_gain = predicted_error / (predicted_error + measurement_noise);
    filtered_values[port] = predicted_value + kalman_gain * (measurement - predicted_value);
    error_covariances[port] = (1 - kalman_gain) * predicted_error;
    
    return filtered_values[port];
}

// =======================================================
// Sliding Median + EWMA Filter (滑動中位數 + 平滑濾波)
// =======================================================
double get_median_ewma_dist(pros::Distance& s) {
    // 每個 Port 獨立的歷史紀錄緩衝區 (Buffer)，長度為 5
    static std::deque<double> history[22]; 
    static double ewma_values[22] = {0.0};
    static bool initialized[22] = {false};
    
    // 參數設定
    const int WINDOW_SIZE = 5;  // 歷史數據長度 (3 到 5 最合適)
    const double alpha = 0.3;   // EWMA 平滑係數

    int port = std::abs(s.get_port()); 
    if (port < 1 || port > 21) return 999; 

    int32_t raw_dist = s.get_distance();
    
    if (raw_dist == PROS_ERR || raw_dist == 0) {
        return initialized[port] ? ewma_values[port] : 999;
    }
    
    double raw = raw_dist / 25.4;

    // 1. 排除硬體回傳的無效範圍
    if (raw < 0.1 || raw > 120.0) {
        // 如果無效，返回上一次的安全結果 (如果有)
        return initialized[port] ? ewma_values[port] : 999; 
    }

    // 2. 將新數據加入歷史緩衝區，並保持長度不超過 WINDOW_SIZE
    history[port].push_back(raw);
    if (history[port].size() > WINDOW_SIZE) {
        history[port].pop_front(); // 踢走最舊的數據
    }

    // 如果歷史數據仲未儲夠 (例如剛開機前幾個 Loop)，就直接返回 raw
    if (history[port].size() < WINDOW_SIZE) {
        return raw; 
    }

    // 3. 獲取中位數 (Median) - 完美過濾突發的 999
    std::vector<double> sorted_window(history[port].begin(), history[port].end());
    std::sort(sorted_window.begin(), sorted_window.end());
    double median = sorted_window[WINDOW_SIZE / 2];

    // 4. 餵給 EWMA 進行最終平滑
    if (!initialized[port]) {
        ewma_values[port] = median;
        initialized[port] = true;
    } else {
        ewma_values[port] = alpha * median + (1 - alpha) * ewma_values[port];
    }

    return ewma_values[port];
}

void auto_init_universal(double start_theta_field_deg, int sensor_mask) {
    // 1. 獲取 IMU 角度並標準化 (0-360)
    double theta = fmod(start_theta_field_deg + imu.get_rotation(), 360.0);
    if (theta < 0) theta += 360;

    double rad = theta * M_PI / 180.0;
    double sin_t = sin(rad);
    double cos_t = cos(rad);

    // 硬件映射
    pros::Distance* sensors[4] = { &dist_left, &dist_right, &dist_back, &dist_front };
    SensorOffset offsets[4] = { OFF_LEFT, OFF_RIGHT, OFF_BACK, OFF_FRONT };
    
    // 安裝方向：左-90°, 右+90°, 後180°, 前0°
    double sensor_mount_angle[4] = { 270.0, 90.0, 180.0, 0.0 };
    // 牆壁法線方向：0:北(N), 1:東(E), 2:南(S), 3:西(W)
    double wall_normal_angles[4] = { 0.0, 90.0, 180.0, 270.0 };

    bool has_x = false, has_y = false;
    double best_x = 0.0, best_y = 0.0;
    double min_x_d = 1e9, min_y_d = 1e9;

    printf("--- [AutoInit] Theta: %.1f ---\n", theta);

    for (int i = 0; i < 4; i++) {
        if (!sensor_allowed(sensor_mask, i)) continue;

        double d_mm = sensors[i]->get_distance();
        if (d_mm <= 20 || d_mm > 2500) continue; 
        double d_measured = d_mm / 25.4;

        // 2. 計算該傳感器在場地上的絕對朝向
        double dir = fmod(theta + sensor_mount_angle[i] + 360.0, 360.0);
        
        // 3. 判定面對哪面牆 (0:N, 1:E, 2:S, 3:W)
        int wall = int(round(dir / 90.0)) % 4;

        // 4. 【核心修正】計算餘弦補償後的垂直距離
        // 入射角偏差 = 感測器朝向 - 牆壁法線方向
        double angle_error_rad = (dir - wall_normal_angles[wall]) * M_PI / 180.0;
        double d = d_measured * cos(angle_error_rad); 

        // 5. 計算旋轉後的全局偏移 (Global Offset)
        double ox = offsets[i].x;
        double oy = offsets[i].y;
        double global_off_x = ox * cos_t + oy * sin_t;
        double global_off_y = -ox * sin_t + oy * cos_t;

        // 6. 根據修正後的距離 d 反推車中心座標
        if (wall == 1) { // East Wall (X = 140.5)
            double x = field_size - d - global_off_x;
            if (d < min_x_d) { best_x = x; min_x_d = d; has_x = true; }
        }
        else if (wall == 3) { // West Wall (X = 0)
            double x = d - global_off_x;
            if (d < min_x_d) { best_x = x; min_x_d = d; has_x = true; }
        }
        else if (wall == 0) { // North Wall (Y = 140.5)
            double y = field_size - d - global_off_y;
            if (d < min_y_d) { best_y = y; min_y_d = d; has_y = true; }
        }
        else if (wall == 2) { // South Wall (Y = 0)
            double y = d - global_off_y;
            if (d < min_y_d) { best_y = y; min_y_d = d; has_y = true; }
        }
    }

    // 7. 更新 lemlib chassis
    lemlib::Pose cur = chassis.getPose();
    chassis.setPose(has_x ? best_x : cur.x, has_y ? best_y : cur.y, theta);

    printf("Final Pose: X=%.2f, Y=%.2f, T=%.1f\n", 
           has_x ? best_x : cur.x, has_y ? best_y : cur.y, theta);
}

double getRealPose(
    double expected,
    double golden,
    pros::Distance& sensor,
    int side
) {
    // 直接用 get_filtered_dist 或 get_smoothed_dist
    double actual = get_kf_dist(sensor);  // 改用 get_kf_dist
    
    if (actual < 0.1 || actual > 120.0) return expected;
    int32_t conf = sensor.get_confidence();
    if (actual > 60.0 && (conf == PROS_ERR || sensor.get_confidence() < 30))
        return expected;

    double diff = actual - golden;

    if (fabs(diff) < 0.25)
        return expected;

    double correction = diff * side;
    return expected - correction;
}

void wallReset(char axis, int velocity, float theta, int timeout_ms) {
    // 1. 強制鎖死模式：確保撞完後不會因為物理彈性往後跳
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    uint32_t start_time = pros::millis();
    
    // 2. 使用恆定功率壓牆 (建議速度：40-60)
    // 負值代表向後撞，正值代表向前撞
    chassis.tank(velocity, velocity); 

    int stall_timer = 0;
    const int STALL_THRESHOLD_MS = 100; // 持續 100ms 失速才判定為撞實

    while (pros::millis() - start_time < timeout_ms) { // 安全超時 1.5 秒
        // 取得左右兩側馬達組的平均實際速度與電流
        double avg_vel = (std::abs(leftMotors.get_actual_velocity()) + 
                          std::abs(rightMotors.get_actual_velocity())) / 2.0;
        
        double avg_curr = (leftMotors.get_current_draw() + 
                           rightMotors.get_current_draw()) / 2.0;

        // 失速判定邏輯：速度極低 (< 2) 且 電流維持在高位 (> 2000mA)
        if (avg_vel < 2 && avg_curr > 2000) {
            stall_timer += 10;
        } else {
            stall_timer = 0; // 只要有動彈就重置計數
        }

        if (stall_timer >= STALL_THRESHOLD_MS) {
            pros::delay(50); // 最後的微壓，確保物理對齊
            break; 
        }
        pros::delay(10);
    }

    // 3. 關鍵時刻：更新座標 (以南牆為例：Y=0, Heading=180)
    // 注意：setPose 應該在馬達還在輸出動力時執行，最準確
    if (axis == 'Y' || axis == 'y'){
        chassis.setPose(0,chassis.getPose().y , theta); 
    }
    else if (axis == 'X' || axis == 'x'){
        chassis.setPose(chassis.getPose().x, 0, theta);
    }else{
        printf("Invalid axis: %c\n", axis);
        return;
    }

    // 4. 停機並微退
    chassis.tank(0, 0);
    pros::delay(80); 

    // 彈開牆壁 0.5 吋，避免接下來轉身時角落磨牆
    //drivePID(50, 0.35,0.000325, 0.63,300);

    // 5. 還原剎車模式，恢復底盤靈活性
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}

void calibrateTrackingOffsets() {
    int iterations = 6; // 做 3 組正反轉
    double accumulated_v_offset = 0.0;
    double accumulated_h_offset = 0.0;
    int valid_tests = 0;

    printf("開始自動校準 Offset (使用 360 度旋轉)...\n");

    for (int i = 0; i < iterations; i++) {
        // 1. 記錄起始位置
        double v_start = vertical.getDistanceTraveled();
        double h_start = horizontal.getDistanceTraveled();
        double imu_start = imu.get_rotation();

        // 2. 執行轉向 (0 -> 360 -> 0)
        double target = (i % 2 == 0) ? 360.0 : 0.0;
        ezPID.turn(target, 60, 3000); // 360 度需要較長超時時間
        
        pros::delay(600); // 等待穩定

        // 3. 計算變化量
        double delta_theta_rad = std::abs(imu.get_rotation() - imu_start) * (M_PI / 180.0);
        double delta_v = std::abs(vertical.getDistanceTraveled() - v_start);
        double delta_h = std::abs(horizontal.getDistanceTraveled() - h_start);

        // 4. 累積 Offset (r = s / theta)
        if (delta_theta_rad > 1.0) { // 確保轉動超過約 60 度才計算，避免除以極小值
            accumulated_v_offset += (delta_v / delta_theta_rad);
            accumulated_h_offset += (delta_h / delta_theta_rad);
            valid_tests++;
            printf("測試 %d: V_Off: %.3f, H_Off: %.3f (Delta Angle: %.1f)\n", 
                   i+1, delta_v/delta_theta_rad, delta_h/delta_theta_rad, delta_theta_rad * 180.0 / M_PI);
        }
    }

    // 5. 計算平均值並打印
    double final_v_offset = (valid_tests > 0) ? (accumulated_v_offset / valid_tests) : 0;
    double final_h_offset = (valid_tests > 0) ? (accumulated_h_offset / valid_tests) : 0;

    printf("\n--- 校準完成 ---\n");
    printf("建議 Vertical Offset (左右偏移): %.4f\n", final_v_offset);
    printf("建議 Horizontal Offset (前後偏移): %.4f\n", final_h_offset);
    printf("----------------\n");
}
