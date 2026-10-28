#pragma once

// Brain 顯示／遙測：機器人動畫、手掣三行文字、Type R Dash（練習模式）。
// 全部為 standalone LVGL。

extern int displayMenu; // 手掣頁面：0 時間／電量／auton，1 里程，2 pose
extern bool autoChange; // UP 啟動動畫；LEFT／RIGHT／DOWN 停止並返回 Dash

void uiTask_fn();         // 動畫幀 + 手掣文字
void telemetry_fn();      // Dash；連接場控時拆除較重 UI
void destroy_robot_anim(); // 刪除動畫 overlay
