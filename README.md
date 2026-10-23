# EzDrive 運動

15383K 的 PROS V5 底盤程式。位姿保留在 LemLib `chassis.getPose()`。馬達輸出與到位判斷都經過 `ezPID`（`ez_compat::EzDrive`）。

- 行駛距離的單位是**英吋**，沿該次呼叫開始時的航向計算。
- 轉向目標是**絕對航向，單位為度**。LemLib 航向以 0° 朝向 +Y，順時針為正。
- `timeout` 的單位是毫秒。時間屆滿後馬達停止，該次運動結束。

`main.h` 引入 `lemlib/api.hpp` 與 `ez_compat.hpp`。`chassis`、`ezPID` 與 `pidTuner` 在 `src/main.cpp` 建立。

此樹的版本：PROS kernel 4.1.1、LemLib 0.5.5、robodash 2.3.0、LVGL 8.3.8。

## 初始化

`initialize()` 校正底盤，並載入下列增益。`EzDrive` 建構式呼叫 `chassis.bindEzDrive(this)`，因此 `moveToPoint`、`moveToPose`、`turnToHeading`、`turnToPoint`、`swingToHeading` 與 `swingToPoint` 使用同一組 PID。

```cpp
ezPID.set_x_drive(false);
ezPID.set_drive_constants(8.25, 0, 42, 0);    // kP, kI, kD, startI
ezPID.set_heading_constants(4, 0, 10, 0);     // heading hold during drive()
ezPID.set_turn_constants(4.1, 0, 30, 15);
ezPID.set_drive_slew(12);                     // max motor-power change per 10 ms
ezPID.set_turn_slew(255);                     // 250 or above: chassis turns are unslewed
ezPID.set_drive_exit_conditions(0.5, 100, 2, 500, 0); // in, ms, in, ms, extra slow ms
ezPID.set_turn_exit_conditions(1, 100, 3, 250, 0);    // deg, ms, deg, ms, extra slow ms
ezPID.set_drive_velocity_threshold(0.10);     // inches per 10 ms
ezPID.set_turn_velocity_threshold(0.25);      // degrees per 10 ms
ezPID.set_heading_velocity_threshold(0.25);
pidTuner.start();
```

`startI`：只有 `|error|` 小於此值，而且 `startI` 不為 0 時，積分才會累積。誤差變號時，積分清零。

`set_drive_slew` 與 `set_turn_slew` 是 `ezPID.drive` 與 `ezPID.turn` 每 10 毫秒允許的功率變化量。底盤運動在對應步進大於或等於 250 時不套用 slew，因此 `set_turn_slew(255)` 使底盤轉向不受 slew 限制。

`xPose` 要在 `ezPID.set_x_drive(true)` 之後才會移動。此坦克式底盤在 `initialize()` 設為 `false`，因此 `xPose` 會立即返回。

## ezPID

`ezPID.drive` 與 `ezPID.turn` 都會阻塞，直到該次運動到位或逾時。最後一個參數傳入 `true` 時，運動在背景執行。

```cpp
void drive(double distance_inches, int max_speed, int timeout = 1500, bool async = false);
void turn(double angle, int max_speed, int timeout = 1500, bool async = false);
```

- 正距離沿呼叫開始時的航向前進。負距離倒車。
- `angle` 是絕對航向，單位為度。
- `max_speed` 是馬達功率，常用範圍是 60–127。

```cpp
ezPID.drive(24, 100, 2000);   // about 24 in forward
ezPID.drive(-12, 90, 1800);   // 12 in backward
ezPID.turn(90, 100, 1500);    // heading 90°
ezPID.turn(0, 90, 1500);
```

單次增益覆寫只改該次運動的 kP、kI 與 kD，運動結束後恢復原來的值（`startI` 維持不變）。非同步運動結束時同樣會恢復：

```cpp
ezPID.drive(12, 90, 1800, {.p = 8.25, .i = 0, .d = 42}, false);
ezPID.turn(90, 90, 1500, {.p = 4.1, .i = 0, .d = 30}, false);
```

`drive()` 用直行 PID 控制距離，並用航向 PID 保持起始航向。到位判斷跟隨直行 PID。`turn()` 使用轉向 PID。

同一時間只有一個運動佔用馬達。下一個運動會等候，直至目前的運動結束。在 `opcontrol` 中，調校器開啟或 `ezPID.is_busy()` 為真時，arcade 保持關閉。

### `src/autons.cpp` 內既有的距離

新路線請以英吋與度撰寫。`src/autons.cpp` 內若干 `ezPID.drive(...)` 的目標（例如 `-1900`、`3100`、`1700`）仍是舊的馬達編碼器度數。在場地上量度這些路徑之前，不要將這些數值讀作英吋。

`rightAutonV2` 原有兩次 `drivePID(-200, 0.35, 0.000325, 0.63, 800)`。現在是 `ezPID.drive(-3.4, 100, 800)`：

`inches = degrees / 360 * pi * 3.25 * (360 wheel rpm / 600 blue cartridge) = -3.4`

舊的 kP、kI、kD 按編碼器度數縮放，因此這兩次呼叫使用 `initialize()` 的增益。程式中已無 `turnPID(...)` 呼叫。

## 底盤運動

`async` 預設為 **true**。下一行必須等機器人停下才執行時，傳入 `false`，或在呼叫之後等候。

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

| 呼叫 | PID | 停止條件 |
| --- | --- | --- |
| `turnToHeading` / `turnToPoint` | 轉向 PID，單位為度 | 角向到位、逾時，或（`minSpeed != 0`）已過 `earlyExitRange` 或已過目標 |
| `swingToHeading` / `swingToPoint` | 同一輸出，再經 `swing_power`。`LEFT` 將左側功率設為 0，`RIGHT` 將右側功率設為 0 | 與轉向相同 |
| `moveToPoint` | 直行 PID（英吋）與轉向 PID（度）。距離目標少於 7.5 英吋時，角向輸出降為 0 | 少於 7.5 英吋且橫向到位。`minSpeed != 0` 時，機器人經過目標點亦可退出 |
| `moveToPose` | 同上，並使用 boomerang `lead`（預設 0.6；數值愈大，彎愈寬） | 已進入 7.5 英吋內，而且橫向與角向都到位。`minSpeed != 0` 時，接近目標後經過目標亦可退出 |

會改變該次運動的參數：

- `maxSpeed`：0–127，預設 127。
- `minSpeed`：預設 0。非 0 時，較早的 `earlyExitRange` 退出才會生效。
- `earlyExitRange`：轉向與 swing 的單位是度；`moveToPoint` 與 `moveToPose` 的單位是英吋。
- `forwards`：預設 true。
- `lead`：只用於 `moveToPose`。
- `direction`：`AngularDirection::AUTO`、`CW_CLOCKWISE` 或 `CCW_COUNTERCLOCKWISE`。誤差變號之後，轉向改走短路徑。

```cpp
chassis.moveToPoint(0, 24, 2500); // async by default
chassis.waitUntil(12);            // inches of distTraveled, not a settle check
chassis.waitUntilDone();          // returns when distTraveled == -1
```

`chassis.follow` 是 pure pursuit。此呼叫下達路徑速度，並佔用同一個運動位置，因此 `drive` 與 `turn` 會等到該路徑結束。

## `waitUntilDone`

到位是指誤差在某個區間內維持足夠時間，而且機器人夠慢。`PID::is_settled(10)` 在每 10 毫秒的迴圈執行一次。調校器讀取 `.error`，並不呼叫 `is_settled`。

1. 誤差落在區間內，並維持足夠時間：
   - small：`|error| <= small_error` 達 `small_time` 毫秒（直行 0.5 英吋 / 100 毫秒，轉向 1° / 100 毫秒）
   - 或 big：`|error| <= big_error` 達 `big_time` 毫秒（直行 2 英吋 / 500 毫秒，轉向 3° / 250 毫秒）
2. 同時必須夠慢：`|derivative| <= velocity_threshold`（直行每個 10 毫秒取樣 0.10 英吋，轉向每個取樣 0.25°）。較快的取樣會把區間計時清零。
3. 第五個退出參數是 `velocity_time`。`0` 表示關閉此額外計時。正值另要求機器人已經維持慢速達該毫秒數。

區間門檻為 `0` 或以下時，該區間關閉。比較使用 `<=`，因此落在邊界上亦算到位。

機器人已經停下，但誤差仍在區間之外（例如輪子受阻）時，運動會繼續，直到該次 `timeout`。逾時在運動迴圈內檢查。

橫向到位使用直行 PID 的退出條件。角向到位使用轉向 PID 的退出條件。航向 PID 只在 `drive()` 內修正航向。

阻塞的 `ezPID.drive` 或 `ezPID.turn`（`async == false`）在該次運動完成時返回。

```cpp
ezPID.drive(24, 80, 2500, true);
ezPID.waitUntil(12);     // planar travel since this call, in inches
ezPID.waitUntilDone();   // Ez task and chassis motion have both released the drive
```

`waitUntil(inches)` 與 `waitUntilAngle(degrees)` 用來標示進度。下一行需要機器人停下時，呼叫 `waitUntilDone()`。`ezPID.waitUntil` 由呼叫當下的位姿起算平面距離；`ezPID.waitUntilAngle` 由呼叫當下的航向起算轉過的角度。這兩個等候都沒有自己的逾時。`cancel_movement()` 可使等候結束。

`ezPID.waitUntilDone()` 在 `motions_pending == 0` 且 `chassis.isInMotion() == false` 時返回，或在安全時限屆滿時返回：

- 在 `ezPID.drive`、`turn` 或 `xPose` 之後：時限是最近一次 Ez 的 `timeout` 再加 400 毫秒（至少 400 毫秒）。
- 在任何 Ez 運動之前：時限是 20000 毫秒。

時限屆滿時，終端機會印出 `EzDrive::waitUntilDone safety timeout ...`，並呼叫 `cancel_movement()`（`break` 旗標，加上 `chassis.cancelAllMotions()`）。

此時限跟隨最近一次 Ez 的 `timeout`。短的 Ez 運動之後，若要等一段較長的非同步底盤運動，請用 `chassis.waitUntilDone()`：

```cpp
chassis.moveToPoint(0, 48, 4000);
chassis.waitUntilDone();
```

Ez 運動與上述底盤運動在開始時把 `distTraveled` 設為 0，結束時設為 -1，因此 `ezPID.waitUntilDone()` 與 `chassis.waitUntilDone()` 都能觀察到這些運動。`chassis.waitUntilDone()` 本身沒有安全時限。`chassis.waitUntil(inches)` 看的是 `distTraveled` 的英吋數，不是到位檢查。

`ezPID.cancel_movement()` 或 `chassis.cancelAllMotions()` 可以截斷進行中的運動。`motions_pending > 0` 或 `chassis.isInMotion()` 時，`ezPID.is_busy()` 為真。

## PID 調校器

`pidTuner.start()` 由 `initialize()` 呼叫。調校器不使用 SD 卡。增益寫在 `ezPID` 上，與自動賽共用同一組 PID。

1. 在主機上開啟名為 **PID** 的 robodash 畫面。
2. 左欄是 **DRIVE in**，右欄是 **TURN deg**。每一列是 `kP`、`kI`、`kD`、`small`、`big`、`vel`。輕觸 `-` 或 `+`。
   - 步進：kP 0.05、kI 0.01、kD 0.5、直行區間 0.1 英吋、轉向區間 0.5°、直行速度 0.01、轉向速度 0.05。
   - 增益與門檻維持在 0 或以上。
3. 即時列是 `err  drive +0.00 in   turn +0.0 deg`（最近一次 PID 誤差）。控制器調校開啟時，列尾另有 `CTRL`。
4. **+12in**、**-12in**、**+90** 與 **-90** 呼叫 `drive(..., true)` 或 `turn(..., true)`。`is_busy()` 為真時，這些按鍵會被忽略。**Print** 把常數片段寫到終端機，畫面註記為 `printed to terminal — paste into initialize()`。**Ctrl** 切換控制器調校。
5. **Pose0** 呼叫 `ezPID.set_pose(0, 0, 0)`，也就是已綁定底盤上的 `chassis.setPose(0, 0, 0)`。此動作只改里程計，不啟動行駛，也不取消進行中的運動。畫面註記為 `pose set to 0,0,0`。控制器調校開啟時，**R1** 的作用相同。
6. 把終端機的片段貼回 `initialize()`，取代由 `set_drive_constants` 到 `set_turn_slew` 的呼叫。列印格式：

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

主機 **+12in** 與控制器 **A** 都是非同步的 `ezPID.drive`，使用與自動賽相同的 PID，並不覆寫增益：

```cpp
ezPID.drive(12, 90, 1800, true);   // +12in and A
ezPID.drive(-12, 90, 1800, true);  // -12in and L1
ezPID.turn(90, 90, 1500, true);    // +90 and B
ezPID.turn(-90, 90, 1500, true);   // -90 and L2
```

航向的 kP、kI、kD 在控制器調校中調整：左右鍵切換欄位（包括 `H kP`、`H kI`、`H kD`），上下鍵改變數值。步進與直行、轉向的 kP、kI、kD 相同。主機畫面的兩欄不含這三項。

### 控制器調校

拔下比賽開關。同時按下 **X+A** 進入（上升沿；該次按下不會一併退出）。進入時會取消進行中的運動，並停止 arcade。

| 按鍵 | 動作 |
| --- | --- |
| LEFT / RIGHT | 上一個欄位 / 下一個欄位 |
| UP / DOWN | 增加一步 / 減少一步 |
| A | `ezPID.drive(+12 in)`，非同步，與自動賽同一 PID |
| B | `turn` 至航向 +90° |
| Y | 把常數列印到終端機 |
| L1 | `ezPID.drive(-12 in)` |
| L2 | `turn` 至航向 -90° |
| R1 | `setPose(0, 0, 0)`，不行駛 |
| X，且已放開 A | 退出並取消運動 |

調校器開啟時，arcade 與自動賽選擇保持關閉。L1 與 L2 執行上表的行駛與轉向，R1 把位姿設為 0, 0, 0，intake 維持開啟調校器時的狀態。控制器三行分別顯示欄位與數值、誤差，以及 `A+12 B90 R1=0`。

比賽開關拔下，而且調校器關閉時：UP 取消運動、呼叫 `reset_pid()` 並使控制器短促震動；DOWN+B 執行 `autonomous()`；左右方向鍵切換自動賽。DOWN+Y 非同步執行 `moveToPose(0, 0, 0, ...)`。調校器關閉時，R1 恢復為 intake，不會把位姿歸零。

控制器只在該行文字改變時更新，大約每 60 毫秒一行，並以 `%-15.15s` 補足寬度，使較短的字串覆蓋原先的文字。插上比賽開關之後，這些控制器畫面更新停止。

調校器關閉時，右搖桿左右（只供練習）循環三頁：

- 0：比賽計時、控制器電量、主機電量、自動賽名稱
- 1：`ezPID.get_traveled()`（英吋）與航向
- 2：位姿 `X`、`Y`、`T`

`reset_pid()` 之後，`get_traveled()` 由當時的位姿起算。
