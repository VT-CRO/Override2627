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
src/auton/                Auton: routine selector, shared helpers (auton.cpp) and the skills run (skills.cpp)
src/tune/                 Auto-tune / sysID code, defined as methods of the subsystem it tunes
include/<same folders>    Headers
include/config/           All ports and tuning numbers, one header per area (namespace config::<area>):
                          ports, drivetrain, elevator, endeffector, auton
skills.cpp.bak            Old backup of the skills routine, not built
include/lemlib|fmt|liblvgl|pros, firmware/   Vendored. Do not edit.
```

## Rules

- Tuning values live only in `include/config/`. Keep the comment next to each number, it records how it was measured.
- Subsystems take `pros::v5::Controller&` (and other subsystems) in their constructor. No globals.
- `Robot` is the only place that wires subsystems together.
- Motors only run once the robot is enabled, so homing happens in `autonomous` / the X button, not `initialize`.

- Auton coordinates are local: the start pose is (0, 0) heading 0, +y forward, +x right. `score()` shifts the origin to the goal (`scoreShiftX/Y`).
- Homing: autonomous and the X + Y test home with the preload held (`homeAll(true)`); an X tap homes with an empty claw. The Skills routine homes the elevator while it drives (`homesElevatorWhileMoving`).

## Autons

Selected on the controller screen in `competition_initialize` (LEFT/RIGHT cycle, default is the first): Skills, Turn 90, Drive 48, OG Preload, Match Left, Match Right, None. Turn 90 and Drive 48 are odometry/tuning tests that show the pose on the brain screen.

## Controls (driver)

- X tap: re-home everything (empty claw, blocks driving). X + Y: run the selected auton (B or a stick stops it).
- L1/L2 elevator manual up/down. Y / A jump to elevator levels 1 / 3 (steps of `stepsPerSection`). B stows the elevator to zero.
- UP held raises the pivot, DOWN held lowers it, neither holds. R1 held: open claw, pivot horizontal, elevator to intake height; release closes the claw. R2: close claw, pivot to stow.
- RIGHT tap: 90 deg turn test. LEFT + RIGHT held 1.5 s: elevator auto-tune (on blocks).
- Drive is arcade (left stick Y, right stick X) with slew limits and speed scaled down as the elevator rises (anti tip).

## Known leftovers

- `Drivetrain::rotationSysId` is declared and defined in `src/tune/drivetrain_tune.cpp` but nothing calls it.
- `matchLeft` / `matchRight` autons are empty stubs.
- Many values in `config/auton.h` and the drivetrain PID are still marked TODO tune.
- Slew-rate code (drivetrain vs elevator) and the `sense/tick/emit` helpers in the two tune files are duplicated on purpose; they use different tuning.
