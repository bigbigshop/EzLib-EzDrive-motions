# EzDrive 運動

15383K 的 PROS V5 底盤程式。位姿保留在 LemLib `chassis.getPose()`。馬達輸出與到位判斷都經過 `ezPID`（`ez_compat::EzDrive`）。

- 行駛距離的單位是**英吋**，沿該次呼叫開始時的航向計算。
- 轉向目標是**絕對航向，單位為度**。LemLib 航向以 0° 朝向 +Y，順時針為正。
- `timeout` 的單位是毫秒。時間屆滿後馬達停止，該次運動結束。

`main.h` 引入 `lemlib/api.hpp` 與 `ez_compat.hpp`。`chassis`、`ezPID` 與 `pidTuner` 在 `src/main.cpp` 建立。

此樹的版本：PROS kernel 4.1.1、LemLib 0.5.5、robodash 2.3.0、LVGL 8.3.x（`firmware/liblvgl.a` 已重建，heap **256KB**）。

| Git 參考 | 說明 |
| --- | --- |
| `main` | 目前主線（含 256KB LVGL、SD CSV、動畫、場控 teardown） |
| `LVGL-32KB-HEAP` | 加大 heap 前的穩定快照（32KB `liblvgl.a`） |
| tag `pre-lvgl-heap-bump` | 與上相同時間點，方便 `git checkout` 回退 |

---

## Changelog

### 2026-10-24 — LVGL heap 256KB + 文件

- 重建 softfp `firmware/liblvgl.a`，`LV_MEM_SIZE`：**32KB → 128KB → 256KB**。
- 同步 `include/liblvgl/lv_conf.h`。
- 32KB 原版保留在分支 `LVGL-32KB-HEAP`。

### 2026-10-24 — PID tuner / SD / 動畫 / 場控

- **控制器按鍵重配**（見下方表格）：X+A 進入、X 退出；A=pose0；B/Y=±12in；L1/R1=絕對 ±90°；L2/R2=左右各轉 10 圈；←→ 換 field **並** `set_pose(0,0,0)`。
- **SD CSV 累積記錄**：`/usd/pid_tune_log.csv`、`/usd/findingOffset.csv`（`src/customs/sd_log.cpp`）。
- ±12in 完成後記錄 **IMU / odom**，Brain 即時列顯示 `imu` / `odom` / `dIMU` / `dOdom`。
- **單 view 動畫**：`robot01`–`robot10` 用 `lv_img_set_src` 換 frame（UP 開；`autoChange`）。
- **場控接上**（`competition::is_connected()`）：關 tuner、停 Console 溫度/distance 狂刷、刪 Anim + PID page 還 LVGL heap；拔出後重建 PID page。場控時 controller 只顯示 B%/C%、auton 名、X/Y/T。

### 稍早

- `EzDrive` bind 後，`moveToPose` / `turnToHeading` 等共用 `ezPID` 的 drive/turn PID。
- 移除 purePID；auton 改走 `ezPID`。
- 程式重組：`customs/`、`ez_compat`、`pid_tuner`、`relocate`。

---

## 專案結構與宣告（declare）

### 主要路徑

| 路徑 | 內容 |
| --- | --- |
| `src/main.cpp` | `chassis` / `ezPID` / `pidTuner` 定義、`initialize` / `opcontrol` |
| `include/customs/objDeclare.hpp` | 全域 `extern` 與 inline 感測器／氣動 |
| `include/ez_compat.hpp` + `src/ez_compat.cpp` | `EzDrive` |
| `include/pid_tuner.hpp` + `src/pid_tuner.cpp` | Brain + controller PID 調校器 |
| `include/customs/sd_log.hpp` + `src/customs/sd_log.cpp` | SD CSV append |
| `src/customs/monitor.cpp` | controller 文字、動畫、telemetry Console |
| `src/assets/robot01.c` … `robot10.c` | LVGL C array 動畫帧（flash，不佔 LVGL heap） |
| `firmware/liblvgl.a` | 預編譯 LVGL（含實際 `LV_MEM_SIZE` pool） |

### `objDeclare.hpp` 慣例

在 `.cpp` **定義**一次，在 header **`extern` 宣告**給其他檔使用：

```cpp
// include/customs/objDeclare.hpp
extern lemlib::Chassis chassis;
extern ez_compat::EzDrive ezPID;
extern ez_compat::PidTuner pidTuner;
extern rd::Selector selector;
extern pros::Controller controller;
extern pros::Imu imu;
extern const lv_img_dsc_t robot01;  // … robot10
// Distance / pneumatics 可用 inline 物件（每個 TU 一份，或集中定義）
```

```cpp
// src/main.cpp（定義）
lemlib::Chassis chassis(...);
ez_compat::EzDrive ezPID(chassis, drivetrain);
ez_compat::PidTuner pidTuner(ezPID);
```

動畫帧：各 `src/assets/robotXX.c` 內有 `const lv_img_dsc_t robotXX = {...}`；`monitor.cpp` 透過 `extern` 取址。未使用的 `kukuku` 可刪 `extern`（`.c` 仍會 link 食 flash）。

新圖流程：

1. [LVGL 8 Image Converter](https://lvgl.io/tools/imageconverter) → **C array** 或 **Binary**。
2. C array → 放入 `src/assets/`，並在 `objDeclare.hpp` 加 `extern`。
3. Binary → 放 SD：`/usd/xxx.bin`，`lv_img_set_src(img, "S:/usd/xxx.bin")`（**不可**直接丟原始 PNG/JPG）。

建議 Indexed 2/4/8-bit；全屏真彩容易慢且大。

---

## 初始化

`initialize()` 校正底盤並載入增益。`EzDrive` 建構式呼叫 `chassis.bindEzDrive(this)`，因此 `moveToPoint`、`moveToPose`、`turnToHeading`、`turnToPoint`、`swingToHeading`、`swingToPoint` **共用** `ezPID` 的 drive/turn PID（不必另調一套 LemLib angular gains）。

```cpp
ezPID.set_x_drive(false);
ezPID.set_drive_constants(8.25, 0, 42, 0);    // kP, kI, kD, startI
ezPID.set_heading_constants(4, 0, 10, 0);     // heading hold during drive()
ezPID.set_turn_constants(4.1, 0, 30, 15);
ezPID.set_drive_slew(12);
ezPID.set_turn_slew(255);                     // >=250: chassis turns unslewed
ezPID.set_drive_exit_conditions(0.5, 100, 2, 500, 0);
ezPID.set_turn_exit_conditions(1, 100, 3, 250, 0);
ezPID.set_drive_velocity_threshold(0.10);
ezPID.set_turn_velocity_threshold(0.25);
ezPID.set_heading_velocity_threshold(0.25);
pidTuner.start();
```

`startI`：只有 `|error|` 小於此值且 `startI != 0` 才積分；誤差變號時積分清零。

`ezPID.turn` 與 `chassis.turnToHeading`：**PID 相同**（bind 後），motion 邏輯略有差異（direction / minSpeed / earlyExit 等）。日常 auton 用 `ezPID.turn` 即可。

`heading` PID 只服務 `ezPID.drive` 直線鎖向，**不**用於 `moveToPose` 的角向（`moveToPose` 用 turn PID）。

---

## ezPID

```cpp
void drive(double distance_inches, int max_speed, int timeout = 1500, bool async = false);
void turn(double angle, int max_speed, int timeout = 1500, bool async = false); // 絕對航向 °
```

```cpp
ezPID.drive(24, 100, 2000);
ezPID.drive(-12, 90, 1800);
ezPID.turn(90, 100, 1500);
ezPID.turn(0, 90, 1500);
```

單次增益覆寫（結束後恢復；`startI` 不變）：

```cpp
ezPID.drive(12, 90, 1800, {.p = 8.25, .i = 0, .d = 42}, false);
ezPID.turn(90, 90, 1500, {.p = 4.1, .i = 0, .d = 30}, false);
```

同一時間一個運動佔用底盤。`pidTuner` 開或 `ezPID.is_busy()` 時 arcade 關閉。

### Auton 距離注意

`src/customs/autons.cpp` 內部分舊目標仍可能是編碼器度數風格的大數字；新路線請用**英吋／度**。量度前勿把舊大數字當英吋讀。

---

## 底盤運動（LemLib）

`async` 預設 **true**。要阻塞可傳 `false` 或之後 `waitUntilDone()`。

```cpp
chassis.turnToHeading(90, 1500, {}, false);
chassis.moveToPoint(0, 24, 2500, {}, false);
chassis.moveToPose(24, 24, 90, 3000, {.lead = 0.4, .maxSpeed = 100}, false);
```

| 呼叫 | PID（bind 後） | 備註 |
| --- | --- | --- |
| `turnToHeading` / `turnToPoint` | turn | 可設 direction / minSpeed / earlyExit |
| `swingTo*` | turn + swing_power | 鎖定一側 |
| `moveToPoint` | drive + turn | ＜7.5in 時角向輸出降為 0 |
| `moveToPose` | drive + turn + lead | 無獨立 ezPID API 對等 |

常用參數：`maxSpeed`、`minSpeed`、`earlyExitRange`、`forwards`、`lead`、`direction`。

---

## `waitUntilDone`

到位：誤差在 small/big 區間維持夠久，且 `|derivative| <= velocity_threshold`（見 `initialize` 的 exit／vel 設定）。

```cpp
ezPID.drive(24, 80, 2500, true);
ezPID.waitUntil(12);
ezPID.waitUntilDone();
```

`ezPID.waitUntilDone()` 在 `motions_pending == 0` 且 `!chassis.isInMotion()` 時返回，或安全時限後 `cancel_movement()`。長底盤運動請用 `chassis.waitUntilDone()`。

---

## PID 調校器

`pidTuner.start()` 在 `initialize()` 呼叫。增益寫在 `ezPID` 上，與 auton／底盤 motion **同一套**。

Brain **PID** 頁（compact UI）：換 field、± 調值、+12/−12、h±90、Print、Ctrl、Pose0、L10/R10（十圈）。

Print 會把常數片段打到 terminal／Console，並寫一行 SD snapshot（有卡時）。貼回 `initialize()` 範例：

```cpp
ezPID.set_drive_constants(...);
ezPID.set_heading_constants(...);
ezPID.set_turn_constants(...);
ezPID.set_drive_exit_conditions(...);
ezPID.set_turn_exit_conditions(...);
ezPID.set_drive_velocity_threshold(...);
ezPID.set_turn_velocity_threshold(...);
ezPID.set_heading_velocity_threshold(...);
ezPID.set_drive_slew(...);
ezPID.set_turn_slew(...);
```

### 控制器調校（須拔場控）

**X+A** 進入（上升沿）；進入時 cancel 運動、停 arcade。**X**（未按住 A）退出。

| 按鍵 | 動作 |
| --- | --- |
| LEFT / RIGHT | 上／下一個 PID field，並 `set_pose(0,0,0)` |
| UP / DOWN | field 數值 ±step |
| A | pose (0,0,0) |
| B | `drive(+12)` async → 寫 `pid_tune_log.csv` |
| Y | `drive(-12)` async → 同上 |
| L1 | `turn` 絕對 **-90°** |
| R1 | `turn` 絕對 **+90°** |
| L2 | 向左 CCW **10 圈** → `findingOffset.csv` |
| R2 | 向右 CW **10 圈** → `findingOffset.csv` |
| X（單獨） | 退出 tuner |

十圈實作：每圈 4×(±90°) 絕對 turn（避開 wrap 無法一次轉 360°）。開始時會 `set_pose(0,0,0)`。

調校開啟時 intake 肩鍵讓給 L1/R1/L2/R2。關閉後恢復 intake。

---

## SD 卡

路徑前綴 **`/usd/`**（PROS）。需插入 microSD。

輔助 API（`sd_log`）：

```cpp
bool sd_log::card_present();
bool sd_log::append_csv(const char* path, const char* header, const char* line);
bool sd_log::append_csvf(const char* path, const char* header, const char* fmt, ...);
```

檔案不存在或為空時自動寫 header，之後 **append 累積**。

### `/usd/pid_tune_log.csv`

每次 drive／turn 完成或 Print snapshot 寫一行，欄位包括：

`timestamp_ms`, `wall_time`, `event`, `cmd_inches`, `cmd_heading`,  
drive/turn/head 的 `kp,ki,kd,error,sm,bg,vl`,  
`odom_x,y,theta`, `odom_theta_delta`, `imu_heading`, `imu_rotation`, `imu_rotation_delta`, `traveled`

V5 常無 RTC：`wall_time` 可能是 `boot_<millis>`。

### `/usd/findingOffset.csv`

L2/R2 十圈測試。欄位：

`timestamp_ms`, `wall_time`, `dir`（`CCW`/`CW`）, `rev`, `quarter`, `odom_x,y,theta`, `imu_heading`, `imu_rotation`

用途：用每圈（或每 quarter）的 `(x,y)` fit 圓，估 tracking／旋轉中心偏移。建議 CW 與 CCW 都跑。若改過 CSV header，請刪舊檔再跑，避免欄位錯位。

圖片若用 SD：converter 出 **Binary**，例如 `S:/usd/frame01.bin`（見上方宣告一節）。

---

## 動畫（Brain）

- **一個** robodash view `Anim` + **一個** `lv_img`，timer 換 `robot01`…`robot10` src。
- Controller **UP**（非場控、非 tuner）→ `autoChange = true` 開始播。
- LEFT/RIGHT 選 auton、DOWN 開 console 等會把 `autoChange = false`。
- **不要**為每帧建一個 `rd::Image` 再 `.focus()`——那會爆 LVGL heap。

---

## 場控／比賽模式

`pros::competition::is_connected()` 為真時：

1. `pidTuner.force_off()` / 禁止 X+A 開 tuner  
2. 停動畫；`destroy_robot_anim()` + `pidTuner.release_ui()`（刪 view，還 LVGL）  
3. 停 Console 溫度／distance／`clear+printf` 迴圈  
4. Controller 三行：`B:% C:%`、auton 名、`X Y T`  
5. 拔出場控後自動 `pidTuner.start()` 重建 PID 頁  

練習場（無場控）仍有完整 telemetry（含 `LVGL xx% used`）與右搖桿 display 分頁。

### 練習場 controller（tuner 關）

| 操作 | 作用 |
| --- | --- |
| 右搖桿左右 | displayMenu 0/1/2（計時電池／里程朝向／XYZ） |
| LEFT / RIGHT | 換 auton + `set_pose(0,0,0)` |
| UP（短按 new） | cancel + `reset_pid` + rumble |
| UP（按住） | 開動畫 |
| DOWN | focus Console |
| DOWN+B | 跑 `autonomous()` |
| DOWN+Y | `moveToPose(0,0,0,…)` |

---

## LVGL heap

| 項目 | 說明 |
| --- | --- |
| 設定 | `include/liblvgl/lv_conf.h` → `LV_MEM_SIZE` |
| 實池 | 必須編進 `firmware/liblvgl.a`（改 header 不夠） |
| 目前 `main` | **256KB**，`-mfloat-abi=softfp`（與本專案 `common.mk` 一致） |
| 回退 | `git checkout LVGL-32KB-HEAP` 或 tag `pre-lvgl-heap-bump` |

Console（練習場）會印 `LVGL %used / free B`，方便確認是否接近 OOM。

紅 mon 多為 LVGL object 過多／碎裂；加大 heap 降低機會，但仍應避免一次建立大量全屏 Image view。

---

## 功能一覽

| 功能 | 狀態 |
| --- | --- |
| `ezPID.drive` / `turn` / bind 底盤 PID | ✓ |
| PID Brain + controller 調校 | ✓ |
| SD `pid_tune_log.csv` / `findingOffset.csv` | ✓ |
| ±12in IMU／odom 顯示與記錄 | ✓ |
| robot01–10 單 view 動畫 | ✓ |
| 場控 teardown UI + 精簡 controller | ✓ |
| LVGL 256KB heap | ✓（`main`） |
| SD 卡直接播 PNG/JPG | ✗（須 converter） |
| 完整 MCL／多狀態 EKF | 未內建（`relocate` 有距離 1D filter） |
