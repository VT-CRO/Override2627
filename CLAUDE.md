# Override2627

VEX V5 robot code. PROS kernel 4.2.2, LemLib 0.5.6, C++20. Target `15_bot_v_1`.

## Build

`pros` is not on PATH. In PowerShell:

```powershell
$i="$env:APPDATA\Code\User\globalStorage\sigbots.pros\install"
$env:PROS_TOOLCHAIN="$i\pros-toolchain-windows\usr"
$env:PATH="$i\pros-cli-windows;$i\pros-toolchain-windows\usr\bin;$env:PATH"
pros make
```

Run `pros make clean` first if it says "Nothing to be done". Sources under `src/` are found recursively; a new `.cpp` needs no registration. Headers are included with quotes relative to `include/` (`#include "subsystems/elevator.h"`).

## Layout

```
src/main.cpp              PROS hooks only, forwards to Robot
src/robot/                Robot: owns controller + subsystems, homing, driver loop, X+Y auton test
src/subsystems/           Drivetrain, Elevator, EndEffector (each class owns its motors and state)
src/auton/                Auton: routine selector and the routines
src/tune/                 Auto-tune / sysID code, defined as methods of the subsystem it tunes
src/util/                 Hardware-free helpers. TurnController: config-free profiled turn controller, all inputs passed in.
                          turn_schedule: builds its inputs from config by elevator height
test/                     PC checks, not in the robot build (build line at the top of each file)
include/<same folders>    Headers
include/config/           All ports and tuning numbers, one header per area (namespace config::<area>)
include/lemlib|fmt|liblvgl|pros, firmware/   Vendored. Do not edit.
```

## Rules

- Tuning values live only in `include/config/`. Keep the comment next to each number, it records how it was measured.
- Subsystems take `pros::v5::Controller&` (and other subsystems) in their constructor. No globals.
- `Robot` is the only place that wires subsystems together.
- Motors only run once the robot is enabled, so homing happens in `autonomous` / the X button, not `initialize`.
- `TurnController` stays config-free: everything it needs is passed in. Config and elevator-height scheduling go through `turn_schedule`, which both `Drivetrain` and the PC sim use, so they build the same inputs.
- The elevator is a DR4B: it flexes sideways, more the higher it is. All the way down it touches the drive wheels, so turn tests and the sysID run from the intake height up, never below.
- After changing turn code, run the PC sim (`test/turn_controller_sim.cpp`) as well as `pros make`. The sim has no high-elevator stutter or heading jumps, so for those use the robot's RIGHT-tap log.

## Turns

- In-place turns (`Drivetrain::turnToHeading` / `turnToPoint`) use `TurnController`: a time-planned S-curve (constant-jerk pieces), feedforward from the sysID model, PD feedback, and an integral at the end against friction. They join LemLib's motion queue, so `isInMotion` / `cancelMotion` work on them.
- `moveToPoint` still steers with LemLib's angular PID, using gains worked out from the intake-height model.
- The turn rate comes from the IMU gyro (`ScaledImu::turnRate`, `imuRateSign` +1). If the gyro and heading ever disagree, the turn loop prints "flip ... imuRateSign" and uses the heading's own rate instead.
- Driver control: with no turn stick, `drive()` holds the heading so the robot drives straight.

## Controls (driver)

- X tap: re-home everything. X + Y: run the selected auton (B or a stick stops it).
- L1/L2 elevator up/down, Y/A/Up/Down/B elevator levels. R1/R2 and right stick: claw and pivot bench test.
- RIGHT tap: 90 deg turn test. It prints a summary, then one CSV line per 10 ms loop (`ms,height,heading,gyro_dps,heading_dps,plan_accel,mv`), which is the main tool for tuning turns. RIGHT alone held 1.5 s: drivetrain rotation sysID at each elevator height (on the floor, elevator homed). LEFT + RIGHT held 1.5 s: elevator auto-tune (on blocks).

## Known leftovers

- `matchLeft` / `matchRight` autons are empty stubs.
- With the elevator up, turns can show a ~9 Hz swing in the output during the slow-down. `turnFeedbackWnHigh` was lowered to 8 for it; this hasn't been checked on the robot yet.
- The pose heading sometimes jumps 2-3 deg in one loop and stays there. The cause is unknown; turns only see it through the P term.
- sysID kA at 0.60 height is 18% off the straight line between intake and top. More `turnModelHeights` (array size 3 to 5) would follow it better.
- Slew-rate code (drivetrain vs elevator) and the `sense/tick/emit` helpers in the two tune files are duplicated on purpose; they use different tuning.
