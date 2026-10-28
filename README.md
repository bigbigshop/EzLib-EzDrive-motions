# EzDrive 運動

15383K 的 PROS V5 底盤程式。位姿保留在 LemLib `chassis.getPose()`。馬達輸出與到位判斷都經過 `ezPID`（`ez_compat::EzDrive`）。

- 行駛距離的單位是**英吋**，沿該次呼叫開始時的航向計算。
- 轉向目標是**絕對航向，單位為度**。LemLib 航向以 0° 朝向 +Y，順時針為正。
- `timeout` 的單位是毫秒。時間屆滿後馬達停止，該次運動結束。

`main.h` 引入 `lemlib/api.hpp` 與 `ezlib/ez_compat.hpp`。`chassis`、`ezPID` 與 `pidTuner` 在 `src/main.cpp` 建立。

此樹的版本視 branch 而定：

| Branch | Kernel | LemLib | robodash | liblvgl | ABI |
| --- | --- | --- | --- | --- | --- |
| **`main`（穩定）** | 4.1.1 | 0.5.5 | 2.3.0 | 8.3.x custom | **softfp** |
| **`ezlib_4.2.1`（實驗）** | 4.2.1 | 0.5.6（source + `angleError` fix） | 2.3.1 | 8.3.9 **hardfp + `LV_MEM_CUSTOM`** | **hard** |

兩邊都係 **`LV_MEM_CUSTOM=1` → `malloc`**。Brain 開機直入 **Type R Dash**；auton 用 **headless** `auton_select`。

舊站 [ezlib.ykyle.eu](https://ezlib.ykyle.eu/) 是較早的說明書（初始化／EzDrive／LemLib／wallReset／getRealPose 理念仍適用）。**以本 README + 倉庫原始碼為準**；站上部分範例（舊 PID 數字、度數距離、opcontrol 掣表、Selector／Console）已過期。

### 線上模擬器（調參／理解用）

| 連結 | 用途 |
| --- | --- |
| [pid.ykyle.eu](https://pid.ykyle.eu) | **PID 模擬器** — 睇 P／I／D、overshoot、調 `ezPID` drive／turn 增益前先建立直覺 |
| [vex.ykyle.eu/moveToPose](https://vex.ykyle.eu/moveToPose) | **LemLib `moveToPose` 模擬器** — 調 `lead`、drift、`minSpeed`／`earlyExit`、`forwards`、目標 θ 等，睇弧線同到位行為 |

模擬器唔代替場上／SD CSV 實測；實機增益仍用 Brain／controller **PID tuner** + `/usd/pid_tune_log.csv`。

| Git 參考 | 說明 |
| --- | --- |
| `main` | **穩定主線**（PROS 4.1.1 / softfp）：Type R Dash、`ezlib/`、`dash_config`、malloc LVGL |
| `ezlib_4.2.1` | **實驗**：PROS 4.2.1 hard float + LemLib 0.5.6 + robodash 2.3.1；須場測後先考慮合入 `main` |
| `LVGL-128KB-HEAP` | 開發時分支（內容已併入 `main`） |
| `LVGL-32KB-HEAP` / tag `pre-lvgl-heap-bump` | 舊 32KB 靜態 pool 快照 |
| `lvgl-128k-heap` | 早期 128→256KB 實驗（歷史參考） |

---

## Changelog

### 2026-10-28 — branch `ezlib_4.2.1`：PROS 4.2.1 hard float（實驗）

- Kernel **4.2.1**、`common.mk` → **`-mfloat-abi=hard`**；libc／libm／libpros 同步。
- robodash **2.3.1**、liblvgl **8.3.9**；官方 `liblvgl.a` 內建 ~10MB pool **唔可用** → 用 purduesigbots/liblvgl 以 **hardfp + `LV_MEM_CUSTOM=1`** 重編 `firmware/liblvgl.a`。
- LemLib 標成 **0.5.6**：繼續 compile `src/lemlib`（保留 `ez_compat`／`bindEzDrive`）；0.5.5→0.5.6 唯一源碼改動 `angleError`（本樹已有）。**未** apply 官方 `LemLib.a`（避免同 source 重複 symbol）。
- **未合入 `main`**：請上 Brain 驗 Dash／PID／auton 後再決定。

### 2026-10-28 — Type R Dash 成為 `main`

- **目錄**：庫內程式移去 `include/ezlib` + `src/ezlib`（`ez_compat`／`pid_tuner`／`relocate`／`boot_log`／`ui_pages`／`telem_dash`／`sd_log`／`monitor`／`auton_select`）；`customs/` 留俾 user 自寫。
- **Brain UI**：開機 `telem_dash`（Type R）— 校正前中心 gauge 0→100～2s；校正後左移電量弧 + pose／DIST／DRV／MCH／RAM。
- **Headless auton**：`auton_select` 取代 `rd::Selector`；controller ←→ 換名，Dash header 顯示。
- **Dash ↔ PID**：`ui_pages` 只 `focus`，唔刪頁（慳 malloc 碎裂）；場控仍 teardown。
- **唔再有** robodash Console／Selector LVGL 頁；溫度改讀 `include/customs/dash_config.hpp`。
- **顏色**：batt &lt;40 黃／40–80 紅／&gt;80 綠；DIST inch≈999 灰、conf&gt;50 綠、&lt;20 黃、其餘紅。
- **練習掣**：←→ **只**換 auton（唔 `set_pose`）；DOWN = `reset_pid`（清 traveled）+ 回 Dash；DOWN+B 跑 auton。
- **Chrome**：隱藏 robodash 右上 views 掣；batt 用 montserrat_48。
- **LVGL**：`LV_MEM_CUSTOM` + `malloc`；Dash 顯示真實 free RAM（`mallinfo` + sbrk）。

### 2026-10-26 — LVGL heap 128KB（softfp rebuild）

- 由 [purduesigbots/liblvgl](https://github.com/purduesigbots/liblvgl) 重建 `firmware/liblvgl.a`：`LV_MEM_SIZE=128KB` + `-mfloat-abi=softfp`（對齊本專案 `common.mk`）。
- 同步 `include/liblvgl/lv_conf.h`。驗證：`lv_mem.o` BSS ≈ 131KB、`work_mem` = `0x20000`。
- 回退：`git checkout pre-lvgl-heap-bump -- firmware/liblvgl.a include/liblvgl/lv_conf.h` 再 `pros make clean && pros make`。

### 2026-10-26 — Console 穩定 + `getRealPose` heading cos

- **Console prefetch 修補**：唔再用 robodash `Console::printf`（內部 buffer ≈ `sizeof(std::string)`，長行 stack overflow → prefetch `0x0`）；改 `snprintf` + `println`。排版改固定欄位（POSE／DRIVE／MECH／DIST）。
- **Boot／tuner**：controller 即時 boot 字；PID page 延遲啟動；CSV `drive_tick`／`turn_tick` ~15ms。
- **`get_kf_dist`**：唔再 return `-1`；壞讀數 → `999` 或上一格 filter。
- **`getRealPose` heading**：用而家 `chassis` θ + sensor mount 做 **cos 投影**（`actual_perp = actual × cos(α)`），小角度唔使次次 `wallReset`；`|α|>25°`／低 conf 回 `expected`；correction clamp ±6"。**仍然唔使 CAD offset**（`golden` 包晒）。
- （同日稍早曾短暫退回 32KB；之後按正確 softfp 流程再上 128KB。）

### 2026-10-24 — README：reset／宣告／monitor／RW 對比

- 補齊 `wallReset`、`getRealPose`、馬達／氣動 declare、controller monitor。
- 寫明 **expected − (actual − golden)** 相對校正（唔使度 sensor offset）。
- 對比 [RW-Template](https://github.com/richardbwang/RW-Template) distance reset（絕對場地座標 + CAD offset）。

### 2026-10-24 — LVGL heap 256KB + 文件

- 重建 softfp `firmware/liblvgl.a`，`LV_MEM_SIZE`：**32KB → 128KB → 256KB**。
- 同步 `include/liblvgl/lv_conf.h`。
- 32KB 原版保留在分支 `LVGL-32KB-HEAP`。

### 2026-10-24 — PID tuner / SD / 動畫 / 場控

- **控制器按鍵重配**（見下方表格）：X+A 進入、X 退出；A=pose0；B/Y=±12in；L1/R1=絕對 ±90°；L2/R2=左右各轉 10 圈；←→ 換 field **並** `set_pose(0,0,0)`。
- **SD CSV 累積記錄**：`/usd/pid_tune_log.csv`、`/usd/findingOffset.csv`（`src/ezlib/sd_log.cpp`）。
- ±12in 完成後記錄 **IMU / odom**，Brain 即時列顯示 `imu` / `odom` / `dIMU` / `dOdom`。
- **單 view 動畫**：`robot01`–`robot10` 用 `lv_img_set_src` 換 frame（UP 開；`autoChange`）。
- **場控接上**（`competition::is_connected()`）：關 tuner、停 Console 溫度/distance 狂刷、刪 Anim + PID page 還 LVGL heap；拔出後重建 PID page。場控時 controller 只顯示 B%/C%、auton 名、X/Y/T。

### 稍早

- `EzDrive` bind 後，`moveToPose` / `turnToHeading` 等共用 `ezPID` 的 drive/turn PID。
- 移除 purePID；auton 改走 `ezPID`。
- 程式重組：user 碼進 `customs/`；庫碼進 `ezlib/`（含 `EzDrive`／tuner／`relocate`／Dash）。
- 混合架構：LemLib 曲線／長距 + `ezPID` 短距精準 + 中段 `wallReset`／`getRealPose`（見舊站序言）。

---

## 專案結構與宣告（declare）

### 主要路徑

| 路徑 | 內容 |
| --- | --- |
| `src/main.cpp` | 底盤馬達／IMU／tracking／`chassis`／`ezPID`／`pidTuner` 定義 |
| **`include/customs/` + `src/customs/`** | **User 側**（自己改／寫）：`objDeclare`、`autons`、`controls`、`fsm`、`dash_config` |
| `include/customs/dash_config.hpp` | **改呢度** DRV／MCH 溫度對應邊粒馬達 |
| **`include/ezlib/` + `src/ezlib/`** | **庫內**（一般唔使改）：`ez_compat`、`pid_tuner`、`relocate`、`telem_dash`、`monitor`、`auton_select`、`sd_log`、`boot_log`、`ui_pages` |
| `include/ez_compat/` | `pid.hpp`、`motion_math.hpp`（LemLib motions 共用數學；路徑維持 `ez_compat/...`） |
| `src/assets/robot01.c` … `robot10.c` | LVGL C array 動畫幀（flash） |
| `firmware/liblvgl.a` | 預編譯 LVGL（softfp；而家 `LV_MEM_CUSTOM` 唔靠靜態 pool） |

Include 例：`#include "ezlib/ez_compat.hpp"`、`#include "ezlib/telem_dash.hpp"`、`#include "customs/dash_config.hpp"`。

### 慣例：`extern` vs `inline`

| 類型 | 放哪 | 例子 |
| --- | --- | --- |
| 單例／大物件 | `main.cpp` **定義** + `objDeclare.hpp` **`extern`** | `chassis`、`ezPID`、`leftMotors`、`imu`、`controller` |
| 小硬體、多檔共用 | `objDeclare.hpp` **`inline`**（header-only 一份） | `dist_*`、`intake_motors`、氣動 |
| 動畫幀 | 各 `robotXX.c` 定義 + header `extern` | `robot01`…`robot10` |

```cpp
// include/customs/objDeclare.hpp（摘要）
extern lemlib::Chassis chassis;
extern ez_compat::EzDrive ezPID;
extern ez_compat::PidTuner pidTuner;
// Auton 用 auton_select（headless）— 無 rd::Selector
extern pros::Controller controller;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Imu imu;
extern const lv_img_dsc_t robot01;  // … robot10

inline pros::Distance dist_front(6);
inline pros::Distance dist_back(21);
inline pros::Distance dist_left(3);
inline pros::Distance dist_right(9);

inline pros::Motor left_intake(20);
inline pros::Motor right_intake(-17);
inline pros::Motor low_roller(18);
inline pros::Motor high_roller(19);
inline pros::MotorGroup intake_motors({20, -17});

inline pros::adi::Pneumatics arm('A', false);
inline pros::adi::Pneumatics cover('D', false);
inline pros::adi::Pneumatics hook('B', false);
inline pros::adi::Pneumatics matchload('C', false);
```

```cpp
// src/main.cpp（定義底盤側）
pros::MotorGroup rightMotors({11, 12, 13}, pros::MotorGearset::blue);
pros::MotorGroup leftMotors({-14, -15, -16}, pros::MotorGearset::blue);
pros::Imu imu(5);
// … TrackingWheel / Drivetrain / Chassis …
ez_compat::EzDrive ezPID(chassis, drivetrain);
ez_compat::PidTuner pidTuner(ezPID);
```

### Dash 溫度：`dash_config.hpp`（必改檔）

Brain Dash 嘅 **DRV**／**MCH** 兩行溫度**只**由 `include/customs/dash_config.hpp` 決定——唔使改 `ezlib/telem_dash.cpp`。

| Dash 行 | 函數 | 預設顯示 |
| --- | --- | --- |
| **DRV** | `format_drive_temps` | 左 LF/LM/LB \| 右 RF/RM/RB（`leftMotors`／`rightMotors`） |
| **MCH** | `format_mech_temps` | L_intake／R_intake \| low／high roller |

**規則**

- MotorGroup 用 `get_temperature(index)`：`index` = 喺 `main.cpp` list **入面次序**（0、1、2…），**唔係** smart-port 號。  
  例：`leftMotors({-14, -15, -16})` → index `0`=port 14、`1`=15、`2`=16。
- 獨立 `pros::Motor` 用 `motor.get_temperature()`（無 index）。
- `snprintf` 字串就係 Brain 上見到嘅格式；`max_out` 畀 Dash 揀顏色（最熱嗰粒）。

**換邊／改順序**：只改對應 `get_temperature(...)` 同註解即可。

**加一粒新機構馬達（例）**

```cpp
// 1) include/customs/objDeclare.hpp
inline pros::Motor my_motor(8);

// 2) include/customs/dash_config.hpp — format_mech_temps
inline void format_mech_temps(char* buf, std::size_t n, double* max_out = nullptr) {
    const double a = left_intake.get_temperature();
    const double b = right_intake.get_temperature();
    const double c = low_roller.get_temperature();
    const double d = high_roller.get_temperature();
    const double e = my_motor.get_temperature();  // 新
    std::snprintf(buf, n, "%.0f/%.0f  |  %.0f/%.0f/%.0f", a, b, c, d, e);
    if (max_out != nullptr) *max_out = std::max({a, b, c, d, e});
}
```

底盤組加馬達：改 `main.cpp` 嘅 `leftMotors`／`rightMotors` list，再喺 `format_drive_temps` 加多一個 `get_temperature(新index)` 同更新 `snprintf`。

**馬達注意**

- 負 port = 反轉（例 `-14`）。
- **禁止**對同一 port 再 `pros::Motor(...)` 一份——會同 `MotorGroup` 搶 reverse／encoder，導致 `ezPID.drive` 異常（見 `main.cpp` 註解）。
- Intake 用獨立 `Motor`／`MotorGroup`；底盤只用 `leftMotors`／`rightMotors`。

**氣動**

- `pros::adi::Pneumatics(port, start_extended)`：第二參數 `false` = 開機收回。
- 手制：`pneumaticsBtn()`（`controls.cpp`）— **僅在 tuner 關閉時**呼叫（避免同 X/A 衝突）。

| 掣 | 氣動 |
| --- | --- |
| X | arm／hook 序列（伸臂→延遲收 hook） |
| A | `hook.toggle()` |
| Y | `matchload.toggle()` |

動畫幀：各 `src/assets/robotXX.c` 內有 `const lv_img_dsc_t robotXX`；`monitor.cpp` 透過 `extern` 取址。未使用的 `kukuku` 可刪 `extern`（`.c` 仍會 link 食 flash）。

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

調增益前可用 [pid.ykyle.eu](https://pid.ykyle.eu) 模擬 PID 行為；上機再用 `pidTuner`／CSV。

---

## 由推機到寫 Auton（建議工作流）

目標：先令 drive／turn **跟得穩**，再**人手推／開**量真實距離同航向，最後寫入 `customs/autons.cpp`。單位一律 **英吋／度**（舊碼可能仲有編碼器度數風格大數字——新路線唔好跟）。

### 1. 調 PID（一次搞掂底盤）

1. 插 SD（可選）、**拔場控**。  
2. **X+A** 開 PID tuner（詳見 **PID 調校器**）。  
3. 用 B/Y（±12″）、L1/R1（±90°）調到少震、到位準；**Print** 貼返 `initialize()`。  
4. 可抽 CSV 同 AI 畫 error 曲線（見 **SD 卡 CSV**）。  
5. 線上輔助：[pid.ykyle.eu](https://pid.ykyle.eu)。

未調好 PID 就寫長 auton，量到嘅數都會被震盪／未到位食掉。

### 2. 真實推機／開，睇 traveled 同 pose

練習場、**tuner 關**：

| 想睇咩 | 點做 |
| --- | --- |
| **直線走咗幾吋**（For PID Distance） | 右搖桿切 controller **displayMenu = 1** → 行 `D:` = `ezPID.get_traveled()`；`T:` = 航向 |
| **場地座標 X／Y／θ** | 右搖桿切 **displayMenu = 2**；或睇 Brain **Dash** pose |
| **清 traveled 重新量** | **DOWN**（`reset_pid` + 回 Dash）或 **UP** cancel／reset |
| **距離牆幾吋**（量 `golden`） | Dash **DIST** 四格 |

**推機量直線距離（常見）**

1. 車放好開局位（或某段起點），需要時 **A**（tuner 內）或自己 `setPose`／頂牆對 0。  
2. Controller 切 **menu 1**；按 **DOWN** 或 **UP** 清 traveled。  
3. **人手慢慢推**（或輕開 arcade）沿路線去到下一停點。  
4. 讀 `D:` → 呢個就係下一段 `ezPID.drive(吋, …)` 嘅距離（後退用負數）。  
5. 轉彎：睇 menu 1／2 或 Dash 嘅 **θ**，下一段用 `ezPID.turn(絕對角, …)` 或 `turnToHeading`。

**用 pose 寫曲線／長距**

1. Menu **2** 或 Dash 記起點、終點嘅 `(x, y, θ)`。  
2. 長距／弧線用 `chassis.moveToPose(x, y, θ, timeout, {forwards, … lead, maxSpeed}, false)`；參數可用 [moveToPose 模擬器](https://vex.ykyle.eu/moveToPose)。  
3. 中段要貼牆校正：`wallReset`／`getRealPose`（見該節）；`golden` 用 Dash DIST 在正確姿態讀。

### 3. 寫入 auton 程式

1. 喺 `include/customs/autons.hpp` 宣告函數；`src/customs/autons.cpp` 實作。  
2. 喺 `src/main.cpp` 的 `auton_select::init({...})` 加一項 `{"顯示名", &你的函數}`。  
3. 練習場 **←／→** 揀 auton（名會上 Dash／手掣）；**DOWN+B** 跑一次驗證。  
4. 一段一段加：量 → 寫 `drive`／`turn`／`moveToPose` → 跑 → 再量下一段。

```cpp
// include/customs/autons.hpp
void myRoute();

// src/customs/autons.cpp
void myRoute() {
    // 開局：物理對位後
    chassis.setPose(0, 0, 0);
    ezPID.drive(24, 100, 2000);   // 推機量到 ~24"
    ezPID.turn(90, 100, 1500);    // 絕對航向
    chassis.moveToPose(24, 24, 90, 3000, {.lead = 0.4, .maxSpeed = 100}, false);
    // 中段可：
    // chassis.setPose(getRealPose(0, goldenL, dist_left, -1),
    //                 getRealPose(0, goldenB, dist_back, -1), 0);
}

// src/main.cpp — initialize() 內
auton_select::init({
    // …
    {"My Route", &myRoute},
});
```

### 4. 小提示

- `ezPID.drive` 距離 = **沿該次開始時航向** 的英吋，唔係場地直線到某點嘅弦長（要去某 (x,y) 用 `moveToPoint`／`moveToPose`）。  
- 推機量距時盡量跟 auton 同一開局／同一航向；轉過彎要重新 **DOWN** 清 traveled 再量下一段直線。  
- 手掣 menu 1 嘅 `D:` 同舊站「For PID Distance」同一用途；Brain Dash 方便同時睇 pose／DIST／溫。

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

`moveToPose` 曲線／`lead`／drift／提前結束：用 [moveToPose 模擬器](https://vex.ykyle.eu/moveToPose) 互動試參（見上方 **線上模擬器**）。

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

## 座標重置：`wallReset` / `getRealPose`

Odometry 會長距離累積誤差。本庫用兩層中段校正（理念與 [舊說明書第四章](https://ezlib.ykyle.eu/) 相同，實作在 `src/ezlib/relocate.cpp`）。

### `wallReset` — 撞牆硬重置

電流＋低速判定「撞實」後，把對應軸寫死為 0，並寫入 `theta`。

```cpp
void wallReset(char axis, int velocity, float theta, int timeout_ms = 1500);
// 例：向後撞、貼著定義 Y=0 的牆（axis 'x'），航向寫 0
wallReset('x', -30, 0, 800);
```

| 參數 | 意義 |
| --- | --- |
| `axis` | `'x'`：認為貼緊 **X 軸牆** → `setPose(x, 0, theta)`；`'y'` → `setPose(0, y, theta)` |
| `velocity` | `tank(v,v)` 恆功率；負=倒車撞，正=前撞（常見 ±30～60） |
| `theta` | 撞實當下強制寫入的航向 |
| `timeout_ms` | 安全超時；內部用 `|vel|<2` 且電流 >2000mA 維持 ~100ms 當 stall |

撞完會 `tank(0,0)`、短 delay，然後還原 `BRAKE_COAST`。之後通常再接 `getRealPose` 修另一軸。

### `getRealPose` — 距離感測器**相對**校正（唔使度 offset）

```cpp
double getRealPose(double expected, double golden, pros::Distance& sensor, int side);
// 語意：return expected - (actual_perp - golden) * side
// actual_perp = get_kf_dist × cos(α)；α = beam 與牆法線夾角（來自 chassis θ）
```

| 參數 | 意義 |
| --- | --- |
| `expected` | 你**預期**此時該軸座標（例：應在 Y=0 附近就傳 `0`） |
| `golden` | 當機器人**真的停在 expected**（量時盡量正對牆）時，感測器應讀到的距離（吋）。練習場量一次寫死即可——**已包含安裝 offset** |
| `sensor` | `dist_front` / `back` / `left` / `right` |
| `side` | `+1`：感測器朝 **座標增大** 方向（北／東）；`-1`：朝南／西 |

內部：`get_kf_dist`；一律查 conf（`<30`／`PROS_ERR` → `expected`）；`|α|>25°` 放棄；`|diff|<0.25"` 忽略；correction **clamp ±6"**。

```cpp
// 例：右感測器朝東 (side=+1)，預期 X≈0，golden 量過 = 17.61"
// 第三個參數 θ：你 setPose 寫入的航向（同 getRealPose 內部用的 odom θ 係兩件事）
chassis.setPose(
    getRealPose(0, 17.61, dist_right, 1),
    getRealPose(0, 8.7,  dist_back, -1),
    0   // 若後面 auton 當 facing 0 寫；只修 XY 可改 chassis.getPose().theta
);
```

**Auton 要唔要計 offset？要唔要當 θ=0？**

| 問題 | 答案 |
| --- | --- |
| 要唔要 CAD sensor offset？ | **唔使。** `golden` 喺正確姿態量一次就包晒。 |
| `getRealPose` 計距離時 θ？ | **唔使你當 0**——函數讀 `chassis.getPose().theta` 做 cos。前提係 odom／IMU 大致啱。 |
| `setPose(..., θ)` 寫入？ | 後面路徑當咩角，就寫咩角。成段當 (0,0,0) 座標系 → 寫 `0`；只修牆距 → 傳而家 `theta`。**唔好**實際 45° 又硬寫 `0`。 |

**時間成本（heading）**

| 方法 | 大概時間 | 幾時用 |
| --- | --- | --- |
| cos 補償（內建） | 0 ms | 日常 mid-auton；偏幾度～十幾度 |
| 短 `ezPID.turn` 再 reset | ~200–400 ms | θ 已歪到 >15–20° |
| `wallReset` | ~0.8–1.5 s | 要硬鎖一軸／完全唔信距離 |

**為何唔用絕對場地座標（wall ±70.25" − 讀數 − CAD offset）？**

正式／練習場圍欄常有 **±1" 以上砌裝誤差**。若用「絕對距離 → 絕對 (x,y)」，等於把**場地公差**寫進 odometry。`getRealPose` 只問：「相對我練車時量好的 golden，差咗幾吋？」——**唔使量測 distance sensor 相對 tracking center 的 CAD offset**，也較唔會被場地歪度拖死。

量 `golden` 的做法：物理對位／治具把車放到 auton 中該 reset 的姿態 → 讀 **Dash DIST** 該格吋數 → 寫進程式。換場地若圍欄差很多，可重量 golden，而唔使改整套 offset 幾何。

`auto_init_universal`（多感測器絕對開局）仍在 `relocate.cpp`，但實戰**不建議**當開局主力——offset 難準、場地公差致命；開局優先物理頂牆 + `setPose(0,0,0)`（舊站第六章）。

### 同 RW-Template distance reset 比較

參考：[RW-Template Localization](https://richardbwang.github.io/RW-Template/guide/localization/)、[Configuration · Distance sensors](https://richardbwang.github.io/RW-Template/guide/configuration/)、repo [richardbwang/RW-Template](https://github.com/richardbwang/RW-Template)。

| | **EzLib `getRealPose`** | **RW-Template `resetPosition*()`（約 3.x／4.x）** |
| --- | --- | --- |
| 座標模型 | **相對**：`expected − (actual_perp − golden)×side` | **絕對**：場地中心原點、牆在 ±70.25"，由讀數反推場地 (x,y) |
| Sensor offset | **唔使**度 tracking center→感測器面（`golden` 包） | **必須**量／填每個 sensor 的 X/Y offset（吋，相對車心） |
| 斜角／轉角 | **cos(α)** 用而家 θ 投影；`|α|>25°` skip | 射線追蹤到實際撞牆點；斜角、近角有過濾 |
| 場地砌裝誤差 | 設計上**避開**：唔把牆當完美絕對尺 | 依賴場地幾何正確；牆歪會直接餵進 pose |
| 開局／中段 | 中段微調；開局推物理對位 | 任意時刻 `resetPositionFront/Back/Left/Right()` |
| 連續定位 | 無內建 MCL（有 1D KF 濾波） | 另有 **MCL** 粒子濾波（文件標註偏新、先練場測） |
| 調參成本 | 每段 reset 記一個 `golden` | 一次量齊 offset + 場地座標系習慣 |
| 失敗時 | 差太細／低 conf → 回傳 `expected`（唔亂改） | 過斜、近角、同 odom 差太大 → skip，回傳 `false` |

**點樣揀**

- 要**少量 CAD、對抗爛場地、auton 裡已知「我大概喺邊」** → EzLib `getRealPose` + `wallReset`。
- 要**斜牆／轉角射線、自動揀牆、可選 MCL、同絕對場地路徑緊耦合** → RW-Template 較完整；但要認真量 offset，並接受場地公差會入系統。

兩者都解決「純 odom 飄」；EzLib 刻意用 **預計−實際**，正係因為比賽場多數砌得唔準。

---

## PID 調校器

`pidTuner.start()` 在 `initialize()` 呼叫。增益寫在 `ezPID` 上，與 auton／底盤 motion **同一套**。

Brain **PID** 頁（compact UI）：換 field、± 調值、+12/−12、h±90、Print、Ctrl、Pose0、L10/R10（十圈）。亦可 **X+A** 用 controller 調（見下）。

線上先建立直覺：[pid.ykyle.eu](https://pid.ykyle.eu)；`moveToPose` 參數：[vex.ykyle.eu/moveToPose](https://vex.ykyle.eu/moveToPose)。

### 建議流程（練習場）

1. **插 microSD**（無卡都可調，但無 CSV）。拔出場控線。  
2. 開機等 Dash／校正完 → **X+A** 開 tuner（或 Brain 撳入 PID 頁）。  
3. **A** 清 pose；←→ 揀 field；↑↓ 改數值。  
4. 測直線：**B** = +12″、**Y** = −12″；測轉向：**L1／R1** = ±90°。睇震盪／overshoot／到位時間。  
5. 滿意後 Brain 撳 **Print**（或等價）→ terminal 出一段 `set_*_constants`；**貼返** `initialize()`。有卡時同時寫一行 `print` 入 CSV。  
6. **X** 退出 tuner。要估 tracking 偏移再跑 **L2／R2** 十圈（見 `findingOffset.csv`）。

Print 貼回範例：

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

## SD 卡 CSV：取出、畫圖、同 AI 合作

路徑前綴 **`/usd/`**（PROS）。需插入 microSD。檔案不存在或為空時自動寫 header，之後 **append 累積**。

輔助 API（`sd_log`）：

```cpp
bool sd_log::card_present();
bool sd_log::append_csv(const char* path, const char* header, const char* line);
bool sd_log::append_csvf(const char* path, const char* header, const char* fmt, ...);
```

### 點樣攞 CSV 出嚟

1. 測完後 **關 Brain 電源**（或確保唔再寫檔），抽出 microSD。  
2. 用電腦讀卡器打開卡根目錄（或 PROS 掛載顯示嘅磁碟）。  
3. 複製：  
   - `pid_tune_log.csv` — drive／turn 誤差曲線  
   - `findingOffset.csv` — 十圈 (x,y)  
4. 可改名備份（例 `pid_tune_2026-10-28_drive12.csv`），再插返卡繼續測。  
5. 若改過程式 CSV **header**，請刪 Brain 卡上舊檔再跑，避免欄位錯位。

V5 常無 RTC：`wall_time` 可能是 `boot_<millis>`；畫圖用 **`timestamp_ms`**。

### `/usd/pid_tune_log.csv`

Tuner drive／turn 時約每 **15ms** 寫一行 `drive_tick`／`turn_tick`（整段誤差曲線，可睇震盪），結束再寫一行 `drive`／`turn`。Print snapshot 仍寫一行 `print`。

欄位包括：

`timestamp_ms`, `wall_time`, `event`, `cmd_inches`, `cmd_heading`,  
drive/turn/head 的 `kp,ki,kd,error,sm,bg,vl`,  
`odom_x,y,theta`, `odom_theta_delta`, `imu_heading`, `imu_rotation`, `imu_rotation_delta`, `traveled`

### `/usd/findingOffset.csv`

L2/R2 十圈測試。欄位：

`timestamp_ms`, `wall_time`, `dir`（`CCW`/`CW`）, `rev`, `quarter`, `odom_x,y,theta`, `imu_heading`, `imu_rotation`

用途：用每圈（或每 quarter）的 `(x,y)` fit 圓，估 tracking／旋轉中心偏移。建議 CW 與 CCW 都跑。

### 自己畫圖（Excel／Google Sheets／Python）

- 篩 `event == drive_tick`（或 `turn_tick`）。  
- X 軸：`timestamp_ms`（可先減第一點令 t=0）。  
- Y 軸：對應誤差欄（drive／turn error；header 名以檔內為準）。  
- 可另畫 `traveled`、`odom_theta`、`imu_heading` 對時間。  
- `findingOffset`：散點 `odom_x` vs `odom_y`，分 `CW`／`CCW` 色；可 fit 圓心估偏移。

### 同 AI 合作畫圖／分析（建議做法）

1. 把 CSV（或截一段有問題嘅 run）一齊交俾 AI（Cursor／ChatGPT 等）。  
2. 講清楚目標，例如：  
   - 「呢個 `pid_tune_log.csv`：畫 `drive_tick` 嘅 error vs `timestamp_ms`，標註 overshoot／settle」  
   - 「比較兩次 +12″ run，邊組 kp/kd 少震」  
   - 「`findingOffset.csv` fit CW／CCW 圓心，估 tracking offset」  
3. 請 AI 出 **Python（pandas + matplotlib／plotly）** 或 Jupyter；跑完睇圖，再決定加 P／加 D／減 I、改 exit／slew。  
4. 把結論寫返 `initialize()` 增益；必要時再上機跑一輪 CSV 驗證。

提示詞可附：本 README 欄位說明 +「EzLib tuner，單位英吋／度，約 15ms 一 tick」。AI 唔代替場感——圖係輔助，最終以車行為準。

圖片若用 SD 播動畫：converter 出 **Binary**，例如 `S:/usd/frame01.bin`（見上方宣告一節）。

---

## Type R Dash（`telem_dash`）

開機流程：

1. `telem_dash::start()` — 只顯示**置中** ignition arc（CAL／LOAD），品牌隱藏。  
2. IMU 校正完 → blocking 0→100 掃完 → `finish_boot()`：arc 移左做 **BATT %**，揭開 pose／DIST／DRV／MCH／RAM／auton 名。  
3. `ui_pages::show_dash()`／`show_pid()` 互斥 focus；**唔會**銷毀對方頁。

溫度來源集中喺 `include/customs/dash_config.hpp`（改 DRV／MCH 唔使改 `telem_dash.cpp`）。用法見上方 **Dash 溫度：`dash_config.hpp`**。

**唔再用** `rd::Console::printf`（內部小 buffer 易 stack smash）。舊 Console 遙測已由 Dash 取代。

---

## 動畫（Brain）

- **一個** robodash view `Anim` + **一個** `lv_img`，timer 換 `robot01`…`robot10` src。
- Controller 相關邏輯會把 `autoChange = false`（例如換 auton、DOWN 回 Dash）。
- **不要**為每幀建一個 `rd::Image` 再 `.focus()`——會爆 heap／碎片。

---

## Controller monitor（`monitor.cpp`）

`initialize()` 開 `uiTask_fn`（controller 三行 + 動畫 + 隱藏 views chrome）同 telemetry 路徑刷新 **Dash**（唔再刷 Console）。

### 設計要點

- Controller **只在字變了才 `print` 一行**（`dirty[]` + round-robin），唔定時 `clear()`。
- 場控接上：teardown Anim／Dash／PID，慳 LVGL；拔出後重建。
- `displayMenu` 用右搖桿左右切頁（練習場、非 tuner）。
- 右上 robodash views 掣會被 **隱藏**（Dash／PID 用 `ui_pages` 切）。

### Controller 三行內容

| 模式 | L0 | L1 | L2 |
| --- | --- | --- | --- |
| **場控** `is_connected()` | `B:% C:%`（手制／Brain 電量） | auton 名 | `X Y T`（整數吋／度） |
| **Tuner 開** | `pidTuner.format_controller`（field／值／狀態） | | |
| **練習 displayMenu=0** | `T:秒` | `B:% C:%` | auton 名 |
| **displayMenu=1** | `D:` `ezPID.get_traveled()`（For PID Distance） | `T:` theta | （空） |
| **displayMenu=2** | `X:` | `Y:` | `T:` |

舊站「用手推車睇 For PID Distance」＝練習場切到 **menu 1**。Brain 上同時間睇 **Dash** DIST／pose／溫。

### 場控／比賽模式

`pros::competition::is_connected()` 為真時：

1. `pidTuner.force_off()` / 禁止 X+A 開 tuner  
2. 停動畫；釋放 Anim／Dash／PID UI  
3. Controller 精簡三行（上表）  
4. 拔出場控後重建 PID／Dash  

### 練習場快捷鍵（tuner 關）

| 操作 | 作用 |
| --- | --- |
| 右搖桿左右 | `displayMenu` 0／1／2 |
| LEFT / RIGHT | 換 auton（**唔** `set_pose`） |
| UP（短按 new） | cancel + `reset_pid` + rumble |
| DOWN | `reset_pid`（清 traveled）+ 回 Dash |
| DOWN+B | focus Dash + 跑 `autonomous()` |
| DOWN+Y | `moveToPose(0,0,0,…)` |
| X+A | 開 PID tuner（見上表） |

氣動掣見上方 **氣動** 表；tuner 開時 X／A／B／Y／肩鍵改歸 PID 調校。Tuner 內 LEFT／RIGHT 仍會 `set_pose(0,0,0)`（換 field）。

---

## LVGL 記憶體

| 項目 | 說明 |
| --- | --- |
| 設定 | `include/liblvgl/lv_conf.h` → **`LV_MEM_CUSTOM 1`**（`malloc`／`free`／`realloc`） |
| 舊靜態 pool | `LV_MEM_CUSTOM 0` + `LV_MEM_SIZE` 編進 `firmware/liblvgl.a` 先有效 |
| ABI | `liblvgl.a` 須同 `common.mk`：`main`=softfp；`ezlib_4.2.1`=hard |
| Dash RAM 字 | `mallinfo().fordblks` +（`HeapLimit − sbrk`）≈ 系統剩餘 |

紅 mon／abort 多為 LVGL object 過多、錯 thread 刪 widget、或碎裂。Dash／PID 用 focus 互斥；場控先 teardown。唔好用 `rd::Console::printf`。

---

## 功能一覽

| 功能 | 狀態 |
| --- | --- |
| `ezPID.drive` / `turn` / bind 底盤 PID | ✓ |
| LemLib `moveToPoint`／`moveToPose`／`turnTo*` | ✓ |
| `wallReset` 撞牆硬重置 | ✓ |
| `getRealPose` 相對距離校正（無 CAD offset） | ✓ |
| 馬達／氣動／distance `objDeclare` | ✓ |
| Type R Dash + `dash_config` 溫度 map | ✓ |
| Headless `auton_select`（無 Selector 頁） | ✓ |
| Controller monitor 分頁 + 場控精簡 | ✓ |
| PID Brain + controller 調校 | ✓ |
| SD `pid_tune_log.csv` / `findingOffset.csv` | ✓ |
| robot01–10 單 view 動畫 | ✓ |
| 場控 teardown UI | ✓ |
| LVGL `LV_MEM_CUSTOM`／malloc（softfp `.a`） | ✓（`main`） |
| robodash Selector／Console 頁 | ✗（已移除，慳 heap） |
| SD 卡直接播 PNG/JPG | ✗（須 converter） |
| RW 式絕對場地 distance reset / MCL | 未內建（可對照 [RW-Template](https://github.com/richardbwang/RW-Template)） |
