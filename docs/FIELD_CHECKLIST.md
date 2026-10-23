# Field checklist — 3 runs each

Practice field only. Brain on, competition switch disconnected. Odometry calibrated (`chassis.calibrate()` already runs in `initialize()`).

Units: **drive is inches**, **turn is degrees** (LemLib heading, 0 faces +Y). Do not type the old motor-degree distances from `src/autons.cpp` into these tests.

Mark each run pass only if the robot stops inside the band, the side that should move actually moves, and nothing oscillates into a timeout. If a run times out while the robot looks stopped, open the PID tuner and raise that loop's velocity threshold slightly, then print the constants.

| Function | What to run | Run 1 | Run 2 | Run 3 |
| --- | --- | --- | --- | --- |
| `ezPID.drive` | `ezPID.drive(12, 90, 1800);` then `ezPID.drive(-12, 90, 1800);` from a marked spot. Expect about 12 in forward, then back to the start. | [ ] | [ ] | [ ] |
| `ezPID.turn` | `ezPID.turn(90, 90, 1500);` then `ezPID.turn(0, 90, 1500);`. Heading should match the target within about 1–3 deg. | [ ] | [ ] | [ ] |
| `turnToHeading` | `chassis.turnToHeading(90, 1500, {}, false);` then `180`, then `0`. Same settle band as turn. | [ ] | [ ] | [ ] |
| `swingToHeading` | `chassis.swingToHeading(45, DriveSide::LEFT, 1500, {}, false);` and the same with `DriveSide::RIGHT`. Locked side stays planted. | [ ] | [ ] | [ ] |
| `swingToPoint` | Place a tape mark 24 in forward and 24 in to the right. `swingToPoint` to that point, left locked, then right locked. | [ ] | [ ] | [ ] |
| `moveToPoint` | From `(0,0)` heading 0: `chassis.moveToPoint(0, 24, 2500, {}, false);`. Then a point 24 in to the side. Stops within about 2 in. | [ ] | [ ] | [ ] |
| `moveToPose` | `chassis.moveToPose(0, 24, 0, 3000, {}, false);` then `chassis.moveToPose(24, 24, 90, 3000, {.lead = 0.4}, false);`. Pose within about 2 in and about 3 deg. | [ ] | [ ] | [ ] |
| settled / `waitUntilDone` | `ezPID.drive(24, 80, 2500, true); ezPID.waitUntilDone();` three times. The next line in auton must not start early. Block the wheels once: it must **not** count as settled; it should hit the timeout instead. | [ ] | [ ] | [ ] |

Tuner check (once, not instead of the table): open the Brain **PID** view, bump drive kP, run **+12in**, press **Print**, and confirm the terminal block matches what you paste back into `initialize()`.

`wallReset` is not in this list. Its behavior was left as-is on purpose.
