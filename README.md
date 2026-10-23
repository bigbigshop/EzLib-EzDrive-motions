# 15383K 底盤用法（EzDrive + LemLib）

呢份係團隊第一份要打開嘅文件。底盤幾何同 odometry 仍然係 LemLib `chassis.getPose()`。出力同 settle 用 `ez_compat::EzDrive`（`ezPID`）入面嗰組 PID，單位係 **drive = inches**、**turn = degrees**。LemLib heading：0° 望住 +Y，順時針為正。

程式係 PROS V5 kernel 4.1.1、LemLib 0.5.5（source 已經喺樹入面）、robodash 2.3.0、LVGL 8.3.8。`main.h` 已經 include `lemlib/api.hpp` 同 `ez_compat.hpp`。全域物件喺 `src/main.cpp`，`include/objDeclare.hpp` 有 `extern`。

```cpp
#include "main.h"

// 已經存在，唔好再 new 一次
// lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttleCurve, &steerCurve);
// ez_compat::EzDrive ezPID(chassis, drivetrain, &fl_m, &bl_m, &fr_m, &br_m);
```

`EzDrive` 建構子會呼叫 `chassis.bindEzDrive(this)`。之後 `moveToPoint`、`moveToPose`、`turnToHeading`、`turnToPoint`、`swingToHeading`、`swingToPoint` 嘅橫向／角度出力都入 Ez PID。`main.cpp` 入面嘅 `lateral_controller` / `angular_controller` 只係 `ezDrive == nullptr` 先會用到嘅後備；而家建構子會 bind，比賽路徑唔會行 LemLib `lateralPID` / `angularPID`。

`initialize()` 已經做咗 `chassis.calibrate()`，然後寫入而家用緊嘅增益（drive inches、turn degrees）：

```cpp
ezPID.set_x_drive(false);
ezPID.set_drive_constants(8.25, 0, 42, 0);    // kP, kI, kD, startI
ezPID.set_heading_constants(4, 0, 10, 0);     // 只係 drive() 行直線時修正航向
ezPID.set_turn_constants(4.1, 0, 30, 15);
ezPID.set_drive_slew(12);                     // 每 10 ms 可以加幾多 motor power
ezPID.set_turn_slew(255);                     // >= 250：chassis 動作當無 slew
ezPID.set_drive_exit_conditions(0.5, 100, 2, 500, 0); // in, ms, in, ms, extra slow ms
ezPID.set_turn_exit_conditions(1, 100, 3, 250, 0);    // deg, ms, deg, ms, extra slow ms
ezPID.set_drive_velocity_threshold(0.10);     // inches / 10 ms
ezPID.set_turn_velocity_threshold(0.25);      // degrees / 10 ms
ezPID.set_heading_velocity_threshold(0.25);
pidTuner.start();
```

`startI`：誤差絕對值細過呢個數、而且 `startI != 0`，先至積 integral。誤差變號會清 integral。

## 單位：autons.cpp 入面啲舊數字唔係 inches

`ezPID.drive` 嘅第一個參數而家係 **LemLib pose 沿出發航向投影出嚟嘅 inches**。`src/autons.cpp` 入面好似 `ezPID.drive(-1900, ...)`、`3100`、`drivePID(-200, ...)` 係舊馬達 encoder degree，**冇改**。唔好當佢係 1900 吋，亦唔好順手改成英寸，除非你喺場上重新量過條路線。

`drivePID` / `turnPID`（`src/purePID.cpp`）仍然用馬達 encoder degrees 同 IMU degrees，有自己嘅 kP/kI/kD，**唔係** Ez PID。佢哋而家會 `acquireMotion()`，同 `ezPID` / `chassis` 動作共用同一條 motion slot，所以唔會同時寫馬達。唔好同 Ez 動作平行呼叫。

## 點樣叫每個動作

同一時間只有一個動作擁有馬達。新動作會排隊等 mutex。`opcontrol` 喺 tuner 開住、或者 `ezPID.is_busy()` 時唔會行 arcade。

`timeout` 一律係 milliseconds。超時會停馬達並結束動作，就算未 settled 都一樣。

### `ezPID.drive` / `ezPID.turn` — 預設阻塞

```cpp
void drive(double distance_inches, int max_speed, int timeout = 1500, bool async = false);
void turn(double angle, int max_speed, int timeout = 1500, bool async = false);
```

- `distance_inches`：正數跟住出發嗰刻嘅 heading 向前，負數向後。
- `angle`：絕對 LemLib heading（度），唔係相對角。
- `max_speed`：motor power，一般 60–127。
- `async` 預設 **false**。呢個 call 會等到 settled 或者 timeout 先 return。

```cpp
ezPID.drive(24, 100, 2000);     // 向前約 24 in，阻塞
ezPID.drive(-12, 90, 1800);     // 向後 12 in
ezPID.turn(90, 100, 1500);      // 轉到 heading 90°
ezPID.turn(0, 90, 1500);
```

一次過換增益（只換 kP/kI/kD，`startI` 保持不變）。動作結束會還原，async 都係喺 worker 入面還原：

```cpp
ezPID.drive(12, 90, 1800, {.p = 8.25, .i = 0, .d = 42}, false);
ezPID.turn(90, 90, 1500, {.p = 4.1, .i = 0, .d = 30}, false);
```

`drive()` 用 drive PID 行距離，再用 heading PID 拉返出發航向。Settle 只睇 drive PID。`turn()` 用 turn PID。

`xPose` 只有 `ezPID.set_x_drive(true)` 先會郁。`initialize()` 設咗 `false`，而家部車唔係 X-drive，呼叫會即刻 return。

### Chassis 動作 — 預設 async

以下全部 `async` 預設 **true**（LemLib 原樣）。要呢句行完先至落下一句，傳 `false`，或者之後自己等。

```cpp
chassis.turnToHeading(90, 1500, {}, false);
chassis.turnToHeading(180, 1500, {.maxSpeed = 80}, false);
chassis.turnToHeading(0, 1500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);

chassis.swingToHeading(45, lemlib::DriveSide::LEFT, 1500, {}, false);
chassis.swingToHeading(45, lemlib::DriveSide::RIGHT, 1500, {}, false);

chassis.swingToPoint(24, 24, lemlib::DriveSide::LEFT, 2000, {}, false);

chassis.moveToPoint(0, 24, 2500, {}, false);
chassis.moveToPoint(24, 24, 2500, {.forwards = false, .maxSpeed = 80}, false);

chassis.moveToPose(0, 24, 0, 3000, {}, false);
chassis.moveToPose(24, 24, 90, 3000, {.lead = 0.4, .maxSpeed = 100}, false);
```

| 呼叫 | 邊組 PID | 幾時停 |
| --- | --- | --- |
| `turnToHeading` / `turnToPoint` | turn PID，`commandAngular(degrees)` | angular settled，或者 timeout，或者 `minSpeed != 0` 時過咗 `earlyExitRange` / 轉過靶 |
| `swingToHeading` / `swingToPoint` | 同上，再經 `swing_power`。`LEFT` 鎖左輪（brake），右輪郁；`RIGHT` 相反 | 同 turn |
| `moveToPoint` | drive PID 行距離誤差（inches），turn PID 行角度誤差（degrees）。離目標 < 7.5 in 之後角度出力變 0 | **近**（< 7.5 in）**而且** lateral settled。唔使 angular settled。`minSpeed != 0` 可以提早因為穿過目標而停 |
| `moveToPose` | 同上，carrot / `lead` 仍然係 boomerang。`lead` 預設 0.6，愈大愈彎 | **近**而且 lateral settled **而且** angular settled |

`params` 入面同 settle 有關、而家仲用緊嘅欄位：

- `maxSpeed`：0–127，預設 127。
- `minSpeed`：預設 0。非 0 先會啟用 `earlyExitRange` 嗰種較早、較順嘅出口。
- `earlyExitRange`：turn / swing 係度；`moveToPoint` / `moveToPose` 係吋。
- `forwards`：預設 true。
- `lead`：只係 `moveToPose`。
- `direction`：`AngularDirection::AUTO` / `CW_CLOCKWISE` / `CCW_COUNTERCLOCKWISE`。開始 settle（誤差變號）之後會改返最短角。

Async 例子（預設已經係 async，所以可以唔寫最後個 `true`）：

```cpp
chassis.moveToPoint(0, 24, 2500);
chassis.waitUntil(12);      // distTraveled，吋，唔係 settle
chassis.waitUntilDone();    // distTraveled == -1
```

`chassis.follow` 係 pure pursuit 路徑速度，**唔經** Ez PID。佢仍然佔住 motion slot，所以 `drive` / `turn` 會等佢。

## `waitUntilDone` 同 settled

### Settled 而家要誤差帶 + 慢

`PID::is_settled(10)` 唔再因為「derivative 好細」就當停好。每個 10 ms loop 只可以呼叫一次（佢會推 timer）。Tuner 只讀 `.error`，唔會呼叫 `is_settled`。

要同時滿足：

1. 誤差喺一條帶入面，而且維持夠耐：
   - 細帶：`|error| <= small_error` 連續 `small_time` ms（而家 drive 0.5 in / 100 ms，turn 1° / 100 ms）
   - 或者大帶：`|error| <= big_error` 連續 `big_time` ms（drive 2 in / 500 ms，turn 3° / 250 ms）
2. 同時要慢：`|derivative| <= velocity_threshold`（drive 0.10 in／每個 10 ms sample，turn 0.25°／sample）。一快，條帶嘅計時會清零。
3. 第五個 exit 參數 `velocity_time`。`0` 代表關閉。而家兩個 `set_*_exit_conditions` 都傳 `0`。如果改成正數，仲要「已經慢咗咁多 ms」先可以 settle。

帶嘅門檻 `<= 0` 等於關閉嗰條帶。比較係 `<=`，包含邊界。

車停咗但誤差仲大（例如輪被擋住）**唔會** settle，會去到你傳入嘅 `timeout`。Timeout 喺動作 loop 入面，唔喺 `is_settled` 入面。

Chassis 橫向 settle 用 drive PID 嘅 exit（`lateralMotionSettled` → `lateral_is_settled`）。角度 settle 用 turn PID 嘅 exit（`angularMotionSettled`）。Heading PID 只負責 `drive()` 行直線時嘅航向修正，唔參與呢個出口。

### 兩個 wait

`ezPID.drive` / `turn` 喺 `async == false` 時，function return 就係動作完結，唔使再 wait。

```cpp
ezPID.drive(24, 80, 2500, true);  // 背景行
ezPID.waitUntil(12);              // 由呼叫 waitUntil 嗰刻嘅 pose 起，平面位移 >= 12 in
ezPID.waitUntilDone();            // 等 Ez task 同 chassis motion 都放咗馬達
```

`waitUntil(inches)` / `waitUntilAngle(degrees)` 只係進度觸發，唔係 settle。下一句如果要車停定，仍然要 `waitUntilDone()`。

`ezPID.waitUntilDone()` 的條件係 `motions_pending == 0` 而且 `chassis.isInMotion() == false`。安全上限：

- 如果之前呼叫過 `ezPID.drive` / `turn` / `xPose`：上限 = **上一次嗰個 Ez timeout + 400 ms**（最少 400 ms）。
- 如果未呼叫過 Ez 動作：上限 20000 ms。

去到上限會 `printf("EzDrive::waitUntilDone safety timeout ...")`，然後 `cancel_movement()`（設 break flag、`chassis.cancelAllMotions()`）。

因為上限跟「上一次 Ez timeout」，唔係跟而家 chassis 動作嘅 timeout。短 Ez 動作之後再等一個長嘅 async `moveToPoint`，用 `chassis.waitUntilDone()`：

```cpp
chassis.moveToPoint(0, 48, 4000); // async
chassis.waitUntilDone();          // 等到 distTraveled == -1，冇上面嗰個 Ez cap
```

Ez 動作同改過嘅 chassis 動作都會喺開始時把 `distTraveled` 設做 0、結束設做 -1，所以兩個 wait 都睇到佢哋。`chassis.waitUntilDone()` 本身冇安全 timeout。

`ezPID.cancel_movement()` 或者 `chassis.cancelAllMotions()` 可以切斷。`ezPID.is_busy()` 係 `motions_pending > 0 || chassis.isInMotion()`。

## 腦畫面 PID tuner（冇 SD 卡）

`pidTuner.start()` 已經喺 `initialize()`。唔會讀寫 SD。

1. Brain 上打開 robodash 畫面，個 view 名係 **PID**。
2. 左欄 **DRIVE in**，右欄 **TURN deg**。每行有 `kP` `kI` `kD` `small` `big` `vel`，撳 `-` / `+`。
   - 步進：kP 0.05，kI 0.01，kD 0.5，drive 誤差帶 0.1 in，turn 誤差帶 0.5°，drive vel 0.01，turn vel 0.05。
   - 增益同門檻會夾到 `>= 0`。
3. 頂部 live 字：`err drive +0.00 in   turn +0.0 deg`。呢個係 PID 入面最後一次誤差，唔係再 call 一次 settle。
4. 底部：**+12in** **-12in** **+90** **-90** 會 `drive(..., true)` / `turn(..., true)`（如果 `is_busy()` 就忽略）。**Print** 會 `printf` 一段可以直接貼嘅程式。**Ctrl** 開關手掣 tuner。
5. 將 terminal 嗰段貼返 `initialize()`，覆蓋 `set_drive_constants` 到 `set_turn_slew` 嗰幾行。Print 出嚟嘅格式：

```cpp
ezPID.set_drive_constants(8.2500, 0.0000, 42.0000, 0.0000);
ezPID.set_heading_constants(4.0000, 0.0000, 10.0000, 0.0000);
ezPID.set_turn_constants(4.1000, 0.0000, 30.0000, 15.0000);
ezPID.set_drive_exit_conditions(0.500, 100, 2.000, 500, 0); // inches, ms
ezPID.set_turn_exit_conditions(1.000, 100, 3.000, 250, 0); // degrees, ms
ezPID.set_drive_velocity_threshold(0.1000); // inches per 10ms
ezPID.set_turn_velocity_threshold(0.2500); // degrees per 10ms
ezPID.set_heading_velocity_threshold(0.2500); // degrees per 10ms
ezPID.set_drive_slew(12.00);
ezPID.set_turn_slew(255.00);
```

Heading 嘅 kP/kI/kD 唔喺 Brain 兩欄入面。用手掣 tuner 先調到：左右揀欄位（包括 `H kP` / `H kI` / `H kD`），上下加減。

### 手掣 tuner

比賽開關拔走。同時撳 **X+A** 進入（只係 rising edge；入咗嗰一圈唔會同時當 X 退出）。進入時會 cancel 正在行嘅動作，arcade 停。

| 鍵 | 作用 |
| --- | --- |
| LEFT / RIGHT | 上一個 / 下一個欄位 |
| UP / DOWN | 減 / 加一步 |
| A | `drive(+12 in)` |
| B | `turn(+90°)` 絕對 heading |
| Y | Print 到 terminal |
| L1 | `drive(-12 in)` |
| L2 | `turn(-90°)` |
| X（冇撳住 A） | 退出並 cancel |

Tuner 開住嗰陣：唔行 arcade、唔轉 auton。L1/L2 變成測試動作，intake 唔會再跟手掣改，保持入 tuner 之前嗰個狀態。畫面三行會變成欄位名同數值、`e` 誤差、`A+12 B+90 Ypr`。

比賽開關拔走、又未開 tuner 時，練習鍵仍然係：UP 一下會 `cancel_movement()`、`reset_pid()`，然後震一下；DOWN+B 跑 `autonomous()`；左右箭咀轉 auton。DOWN+Y 會 `moveToPose(0, 0, 0, ...)`，而且冇傳 `false`，所以係 async。

## 手掣畫面：唔再成版 clear

`uiTask` **唔會**再每半秒 `controller.clear()`。只有某一行字變咗先 `controller.print`，大約 60 ms 先印一行，所以三行唔會同一圈刷光。每行用 `%-15.15s` 填滿 15 格，短字會蓋住舊字，畫面唔會閃空白。

接住比賽開關時，呢個 controller 更新會停。

未開 tuner 時，右搖桿左右切三頁（練習時先有效）：

- 0：`T:` 計時、`B:` 手掣電、`C:` 腦電、auton 名
- 1：`D:` `ezPID.get_traveled()`（inches）、`T:` heading
- 2：`X` `Y` `T` pose

## `wallReset` 刻意保持原樣

`wallReset` 喺 `src/fsm.cpp`，宣告喺 `include/fsm.hpp`。軸向同呼叫方式係刻意留返上傳嗰版，**呢份文件唔係喺度修佢**。`src/main.cpp` 入面仍然係註釋：

```cpp
//wallReset('X', -40, 0, 1500);
```

場地 checklist 都冇包括 `wallReset`。

## 點樣跑 3× 自動測試同場地 checklist

雲端同 laptop 冇機械人。Host test 用 g++、C++17，編譯 `tests/host/test_motions.cpp`，唔使 PROS。每個 case 經 `repeat3` 跑三次：settle 唔可以只靠 derivative 就退出、誤差帶入面要慢先 settle、drive / turn / turnToHeading / swing / moveToPoint / moveToPose 嘅出口、馬達符號、冇 NaN。模擬車有輪速 lag，先至會用到團隊而家嘅 kD。

```bash
make -C tests/host test
```

或者：

```bash
g++ -std=c++17 -O2 -Wall -Wextra -I include tests/host/test_motions.cpp -o /tmp/test_motions && /tmp/test_motions
```

成功會印 `all motion tests passed (3 runs each)`，exit code 0。呢個樹已經用上面嘅 g++ 指令跑過，輸出就係呢句。二進位 `tests/host/test_motions` 唔好 commit。

上到真車之後跟 `docs/FIELD_CHECKLIST.md`。每個函數三格：drive、turn、turnToHeading、swingToHeading、swingToPoint、moveToPoint、moveToPose、settled / `waitUntilDone`。擋住輪子嗰次必須去到 timeout，唔可以當 settled。Checklist 最後有一次 tuner Print，對返你貼入 `initialize()` 嘅字。

## 機器人編譯

有 PROS CLI 同 `arm-none-eabi-g++` 的話，喺專案根目錄：

```bash
pros make
```

呢個樹已經帶 `firmware/`、`include/pros`、`include/liblvgl`、LemLib source。Host test 過咗唔等於 ARM 連結過咗。冇 `arm-none-eabi-g++` / `pros` 嘅機器就只跑上面嗰個 g++。

## 邊啲檔案改咗、PID 由邊度入

| 檔案 | 而家嘅 PID 入口 |
| --- | --- |
| `include/ez_compat/pid.hpp` | `PID::compute`、`PID::compute_error`、`PID::is_settled` |
| `src/ez_compat.cpp` `run_drive` | `drivePID.compute`（inches）+ `headingPID.compute`（degrees, wrap） |
| `src/ez_compat.cpp` `run_turn` | `turnPID.compute`（絕對 heading, wrap） |
| `src/ez_compat.cpp` `run_xpose` | drive + turn `compute`（只係 X-drive） |
| `src/lemlib/chassis/chassis.cpp` | `commandLateral` → `power_from_lateral_error` → `drivePID.compute_error`；`commandAngular` → `power_from_angular_error` → `turnPID.compute_error`；settle 行 `lateral_is_settled` / `angular_is_settled`。`ezDrive == nullptr` 先退回 `lateralPID.update` / `angularPID.update` |
| `src/lemlib/chassis/motions/moveToPoint.cpp` | `commandLateral` + `commandAngular`；出口 `lateralMotionSettled` |
| `src/lemlib/chassis/motions/moveToPose.cpp` | 同上；出口要 lateral 同 angular 都 settled，而且 close |
| `src/lemlib/chassis/motions/turnToHeading.cpp` | `commandAngular` + `angularMotionSettled` |
| `src/lemlib/chassis/motions/turnToPoint.cpp` | 同上 |
| `src/lemlib/chassis/motions/swingToHeading.cpp` | `commandAngular`，再 `swing_power` |
| `src/lemlib/chassis/motions/swingToPoint.cpp` | 同上 |
| `src/lemlib/chassis/motions/pursuit.cpp` | **唔係** Ez PID。路徑速度。佔 motion slot |
| `src/purePID.cpp` | 舊 encoder / IMU PID。佔同一個 motion slot。唔好同 Ez 動作一齊行 |
| `src/pid_tuner.cpp`、`include/pid_tuner.hpp` | 只改 Ez PID 數字同呼叫 `drive` / `turn`。冇 SD |
| `src/main.cpp` `uiTask` | 改行先 print。冇 `controller.clear()` |

Slew：chassis 動作經 `lateralSlewStep()` / `angularSlewStep()`。Ez slew `>= 250` 會回傳 0，LemLib `slew(..., 0)` 當無限。`ezPID.drive` / `turn` 自己用每 loop ±step 夾功率；`set_turn_slew(255)` 大過 127，所以轉向實際上冇限制。`set_drive_slew(12)` 先係直行加速度限制。
