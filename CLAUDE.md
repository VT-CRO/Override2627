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
include/<same folders>    Headers
include/config/           All ports and tuning numbers, one header per area (namespace config::<area>)
include/lemlib|fmt|liblvgl|pros, firmware/   Vendored. Do not edit.
```

## Rules

- Tuning values live only in `include/config/`. Keep the comment next to each number, it records how it was measured.
- Subsystems take `pros::v5::Controller&` (and other subsystems) in their constructor. No globals.
- `Robot` is the only place that wires subsystems together.
- Motors only run once the robot is enabled, so homing happens in `autonomous` / the X button, not `initialize`.

## Controls (driver)

- X tap: re-home everything. X + Y: run the selected auton (B or a stick stops it).
- L1/L2 elevator up/down, Y/A/Up/Down/B elevator levels. R1/R2 and right stick: claw and pivot bench test.
- RIGHT tap: 90 deg turn test. LEFT + RIGHT held 1.5 s: elevator auto-tune (on blocks).

## Known leftovers

- `Drivetrain::rotationSysId` is defined but nothing calls it.
- `matchLeft` / `matchRight` autons are empty stubs.
- Slew-rate code (drivetrain vs elevator) and the `sense/tick/emit` helpers in the two tune files are duplicated on purpose; they use different tuning.
