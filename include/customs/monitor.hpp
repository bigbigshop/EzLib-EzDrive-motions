#pragma once

#include "robodash/api.h"

// 顯示/遙測模組：圖片動畫、controller 文字分頁、robodash console 遙測

extern rd::Console console;
extern int displayMenu; // controller 文字頁面：0 時間/電池/auton、1 里程/朝向、2 坐標
extern bool autoChange; // 圖片動畫開關

void uiTask_fn(); // 圖片動畫 + controller 文字（initialize 內開 task）
void telemetry_fn(); // robodash console 遙測（opcontrol 內開 task）
