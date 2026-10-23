# EzDrive motions

PROS V5 chassis code for 15383K. Pose stays on LemLib `chassis.getPose()`. Output and settle go through `ezPID` (`ez_compat::EzDrive`).

- Drive distance is **inches**, measured along the heading at the start of the move.
- Turn targets are **absolute headings in degrees**. LemLib heading: 0° faces +Y, clockwise is positive.
- `timeout` is milliseconds. When it expires the motors stop and the motion ends.

`main.h` includes `lemlib/api.hpp` and `ez_compat.hpp`. `chassis`, `ezPID`, and `pidTuner` are created in `src/main.cpp`.

Versions in this tree: PROS kernel 4.1.1, LemLib 0.5.5, robodash 2.3.0, LVGL 8.3.8.

## Setup

`initialize()` calibrates the chassis and loads the gains below. The `EzDrive` constructor calls `chassis.bindEzDrive(this)`, so `moveToPoint`, `moveToPose`, `turnToHeading`, `turnToPoint`, `swingToHeading`, and `swingToPoint` use this same PID.

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

`startI`: the integral builds only while `|error|` is under that value and `startI` is not 0. A sign change clears the integral.

`xPose` moves only after `ezPID.set_x_drive(true)`. `initialize()` sets `false` on this tank drive, so `xPose` returns immediately.

## `ezPID.drive` and `ezPID.turn`

Both block until the motion settles or times out. Pass `true` as the last argument to run in the background.

```cpp
void drive(double distance_inches, int max_speed, int timeout = 1500, bool async = false);
void turn(double angle, int max_speed, int timeout = 1500, bool async = false);
```

- A positive distance follows the heading from the start of the call. A negative distance drives backward.
- `angle` is an absolute heading in degrees.
- `max_speed` is motor power, usually 60–127.

```cpp
ezPID.drive(24, 100, 2000);   // about 24 in forward
ezPID.drive(-12, 90, 1800);   // 12 in backward
ezPID.turn(90, 100, 1500);    // heading 90°
ezPID.turn(0, 90, 1500);
```

A one-shot gain override changes kP, kI, and kD for that motion and restores them when it ends (`startI` stays put). The same restore happens for async motions:

```cpp
ezPID.drive(12, 90, 1800, {.p = 8.25, .i = 0, .d = 42}, false);
ezPID.turn(90, 90, 1500, {.p = 4.1, .i = 0, .d = 30}, false);
```

`drive()` uses the drive PID for distance and the heading PID to hold the starting heading. Settle follows the drive PID. `turn()` uses the turn PID.

One motion owns the motors at a time. The next motion waits for that slot. In `opcontrol`, arcade stays off while the tuner is open or `ezPID.is_busy()` is true.

### Distances already in `src/autons.cpp`

Write new routes in inches and degrees. Several `ezPID.drive(...)` targets already in `src/autons.cpp` (for example `-1900`, `3100`, `1700`) are still the old motor-encoder degree numbers. Measure those paths on the field before reading them as inches.

`rightAutonV2` had two `drivePID(-200, 0.35, 0.000325, 0.63, 800)` backups. Those are now `ezPID.drive(-3.4, 100, 800)`:

`inches = degrees / 360 * pi * 3.25 * (360 wheel rpm / 600 blue cartridge) = -3.4`

The old kP/kI/kD were scaled for encoder degrees, so these calls use the gains from `initialize()`. There is no live `turnPID(...)` call left to migrate.

## Chassis motions

`async` defaults to **true**. Pass `false`, or wait afterwards, when the next line should run after the robot has stopped.

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

| Call | PID | Stops when |
| --- | --- | --- |
| `turnToHeading` / `turnToPoint` | turn PID, degrees | angular settled, timeout, or (`minSpeed != 0`) past `earlyExitRange` or past the target |
| `swingToHeading` / `swingToPoint` | same output, then `swing_power`. `LEFT` brakes the left side; `RIGHT` brakes the right side | same as turn |
| `moveToPoint` | drive PID (inches) and turn PID (degrees). Angular output drops to 0 inside 7.5 in | closer than 7.5 in and lateral settled. `minSpeed != 0` can exit when the robot passes the point |
| `moveToPose` | same, with boomerang `lead` (default 0.6; larger bends wider) | close, lateral settled, and angular settled |

Params that change the move:

- `maxSpeed`: 0–127, default 127.
- `minSpeed`: default 0. A non-zero value enables the earlier `earlyExitRange` exit.
- `earlyExitRange`: degrees for turn and swing; inches for `moveToPoint` and `moveToPose`.
- `forwards`: default true.
- `lead`: `moveToPose` only.
- `direction`: `AngularDirection::AUTO`, `CW_CLOCKWISE`, or `CCW_COUNTERCLOCKWISE`. After the error changes sign, the turn takes the short way.

```cpp
chassis.moveToPoint(0, 24, 2500); // async by default
chassis.waitUntil(12);            // inches of distTraveled, not a settle check
chassis.waitUntilDone();          // returns when distTraveled == -1
```

`chassis.follow` is pure pursuit. It commands path speed and holds the same motion slot, so `drive` and `turn` wait until it finishes.

## `waitUntilDone`

Settled means the error has stayed inside a band **and** the robot is slow. `PID::is_settled(10)` runs once per 10 ms loop. The tuner reads `.error` and does not call `is_settled`.

1. Error band, held long enough:
   - small: `|error| <= small_error` for `small_time` ms (drive 0.5 in / 100 ms, turn 1° / 100 ms)
   - or big: `|error| <= big_error` for `big_time` ms (drive 2 in / 500 ms, turn 3° / 250 ms)
2. Slow at the same time: `|derivative| <= velocity_threshold` (drive 0.10 in per 10 ms sample, turn 0.25° per sample). A faster sample clears the band timer.
3. The fifth exit argument is `velocity_time`. `0` leaves that extra timer off. A positive value also requires the robot to have been slow for that many milliseconds.

A band threshold of `0` or below turns that band off. The comparison is `<=`, so the boundary counts.

A robot that is stopped but still outside the band (wheels blocked) keeps running until the motion `timeout`. Timeout is checked in the motion loop.

Lateral settle uses the drive PID exits. Angular settle uses the turn PID exits. The heading PID only corrects heading inside `drive()`.

A blocking `ezPID.drive` or `ezPID.turn` (`async == false`) returns when that motion is done.

```cpp
ezPID.drive(24, 80, 2500, true);
ezPID.waitUntil(12);     // planar travel since this call, in inches
ezPID.waitUntilDone();   // Ez task and chassis motion have both released the drive
```

`waitUntil(inches)` and `waitUntilAngle(degrees)` mark progress. Call `waitUntilDone()` when the next line needs the robot stopped.

`ezPID.waitUntilDone()` returns when `motions_pending == 0` and `chassis.isInMotion() == false`, or when its safety cap expires:

- After `ezPID.drive`, `turn`, or `xPose`: cap is the last Ez timeout plus 400 ms (at least 400 ms).
- Before any Ez motion: cap is 20000 ms.

At the cap it prints `EzDrive::waitUntilDone safety timeout ...` and calls `cancel_movement()` (`break` flag plus `chassis.cancelAllMotions()`).

The cap follows the last Ez timeout. After a short Ez move, wait out a long async chassis motion with `chassis.waitUntilDone()`:

```cpp
chassis.moveToPoint(0, 48, 4000);
chassis.waitUntilDone();
```

Ez motions and the chassis motions above set `distTraveled` to 0 at the start and to -1 at the end, so both waits see them. `chassis.waitUntilDone()` has no safety cap of its own.

`ezPID.cancel_movement()` or `chassis.cancelAllMotions()` cuts a motion short. `ezPID.is_busy()` is true when `motions_pending > 0` or `chassis.isInMotion()`.

## PID tuner

`pidTuner.start()` is called from `initialize()`. The tuner does not use the SD card.

1. On the brain, open the robodash view named **PID**.
2. Left column is **DRIVE in**, right column is **TURN deg**. Each row is `kP`, `kI`, `kD`, `small`, `big`, `vel`. Tap `-` or `+`.
   - Steps: kP 0.05, kI 0.01, kD 0.5, drive bands 0.1 in, turn bands 0.5°, drive velocity 0.01, turn velocity 0.05.
   - Gains and thresholds stay at 0 or above.
3. The live line is `err drive +0.00 in   turn +0.0 deg` (the last PID error).
4. **+12in**, **-12in**, **+90**, and **-90** call `drive(..., true)` or `turn(..., true)`. A press is ignored while `is_busy()` is true. **Print** writes a snippet to the terminal. **Ctrl** toggles the controller tuner.
5. **Pose0** (controller **R1** while the tuner is open) sets pose to 0, 0, 0 and does not start a drive; the note reads `pose set to 0,0,0`. **+12in** and **A** run `ezPID.drive` async on the same PID as auton.
6. Paste that snippet over `set_drive_constants` through `set_turn_slew` in `initialize()`. Print format:

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

Heading kP, kI, and kD are on the controller tuner: left and right move between fields (including `H kP`, `H kI`, and `H kD`), up and down change the value.

### Controller tuner

Unplug the competition switch. Press **X+A** together to enter (rising edge; that press does not also exit). Entering cancels the motion in progress and stops arcade.

| Button | Action |
| --- | --- |
| LEFT / RIGHT | previous / next field |
| UP / DOWN | one step down / up |
| A | `drive(+12 in)` |
| B | `turn` to heading +90° |
| Y | print constants to the terminal |
| L1 | `drive(-12 in)` |
| L2 | `turn` to heading -90° |
| R1 | set pose to 0, 0, 0 (no drive) |
| X, released from A | exit and cancel |

With the tuner open, arcade and auton changes stay off. L1 and L2 are the test motions, R1 sets the pose to 0, 0, 0, and the intake holds the state it had when the tuner opened. The three controller lines show the field and value, the error, and `A+12 B90 R1=0`.

With the competition switch unplugged and the tuner closed: UP cancels the motion, calls `reset_pid()`, and rumbles; DOWN+B runs `autonomous()`; the left and right arrows change the auton. DOWN+Y runs `moveToPose(0, 0, 0, ...)` async.

The controller prints a line only when that line's text changes, about one line every 60 ms, padded with `%-15.15s` so a shorter string covers the old one. Plugging in the competition switch stops those controller updates.

With the tuner closed, the right stick left/right (practice only) cycles three pages:

- 0: match timer, controller battery, brain battery, auton name
- 1: `ezPID.get_traveled()` in inches, and heading
- 2: pose `X`, `Y`, `T`

## On the field

Practice-field checks for each motion are in [docs/FIELD_CHECKLIST.md](docs/FIELD_CHECKLIST.md).

`wallReset` in `src/fsm.cpp` is unchanged. The call in `src/main.cpp` stays commented:

```cpp
//wallReset('X', -40, 0, 1500);
```

## Build

From the project root, with the PROS CLI and `arm-none-eabi-g++`:

```bash
pros make
```

`firmware/`, `include/pros`, `include/liblvgl`, and the LemLib sources are already in the tree.
