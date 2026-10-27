#pragma once

#include "robodash/api.h"

// 顯示/遙測：動畫、controller 分頁、Dash dashboard（練習場）。
// rd::Console 已移除（慳 LVGL heap）；PID Print → USB terminal + SD CSV。

extern int displayMenu; // controller 文字頁面：0 時間/電池/auton、1 里程/朝向、2 坐標
extern bool autoChange; // UP 開動畫；LEFT/RIGHT/DOWN 關並 focus Dash

void uiTask_fn(); // robot01–10 單 view 換 src + controller 文字（initialize 開 task）
void telemetry_fn(); // practice Dash 遙測；場控接上會 release UI
void destroy_robot_anim(); // rd_view_del Anim — frees LVGL heap
