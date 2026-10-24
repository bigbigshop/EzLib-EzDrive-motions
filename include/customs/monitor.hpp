#pragma once

#include "robodash/api.h"

// 顯示/遙測模組：圖片動畫、controller 文字分頁、robodash console 遙測

extern rd::Console console;
extern int displayMenu; // controller 文字頁面：0 時間/電池/auton、1 里程/朝向、2 坐標
extern bool autoChange; // UP 開動畫；LEFT/RIGHT/DOWN 關並 focus selector/console

void uiTask_fn(); // robot01–10 單 view 換 src + controller 文字（initialize 開 task）
void telemetry_fn(); // practice console 遙測；場控接上會停刷新並關 tuner
void destroy_robot_anim(); // rd_view_del Anim — frees LVGL heap
