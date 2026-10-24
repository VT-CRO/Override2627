# Coding Guidelines

These rules apply to every change to first-party code, whether a person or an AI agent writes it. AI agents also follow [CLAUDE.md](../CLAUDE.md).

| Keyword | Meaning |
| --- | --- |
| **MUST** / **MUST NOT** | Required. A reviewer blocks the PR until it is fixed. |
| **SHOULD** / **SHOULD NOT** | The default. Deviate only with a reason written in a comment or the PR description. |
| **MAY** | Allowed; your call. |

Rule IDs (`STR-3`, `CPP-4`, ...) are stable. Use them in review comments and commit messages.

## 0. Scope

**First-party code** is everything in `src/` plus the top-level headers in `include/` (`commands.h`, `constants.h`, `drivetrain.h`, `elevator.h`, `endeffector.h`, and any new top-level header). These rules apply to it.

**Vendored code** MUST NOT be edited or reformatted: `include/pros/`, `include/lemlib/`, `include/liblvgl/`, `include/fmt/`, `include/api.h`, `firmware/`, `common.mk`. PROS and LemLib template upgrades overwrite these.

**PROS template files** (`include/main.h`, `Makefile`, `project.pros`): edit only the sections the template marks as user-editable, and only when the task needs it.

Existing code that breaks a rule is not a precedent. New and modified lines MUST follow these rules. Untouched lines are fixed in dedicated cleanup PRs, not as drive-by edits inside unrelated changes.

---

## 1. Project structure (STR)

```
include/<subsystem>.h         public API of one subsystem: declarations + doc comments only
src/<subsystem>.cpp           implementation and all private state of that subsystem
src/<subsystem>_<part>.cpp    extra translation unit for a large subsystem (e.g. elevator_tune.cpp)
src/<subsystem>_internal.h    declarations shared only between one subsystem's own .cpp files
include/constants.h           ports, gear ratios, and every tuned number
include/util.h, src/util.cpp  hardware-free helpers shared by subsystems (slew limiter, loop timing, math)
src/commands.cpp              actions that coordinate more than one subsystem
src/main.cpp                  competition entry points and the driver-control loop, nothing else
```

- **STR-1** One subsystem per header/source pair, named after the physical mechanism. File, namespace, and header agree: `include/elevator.h` ↔ `src/elevator.cpp` ↔ `namespace elevator`.
- **STR-2** Headers contain the public API only: function declarations and the types callers need. Headers MUST NOT define variables or device objects. The exception is `constexpr` values in `constants.h`.
- **STR-3** Anything not declared in the subsystem's header MUST have internal linkage, which means it goes in an anonymous namespace (CPP-8). A `.cpp` file MUST NOT `extern`-declare another file's variables. If two files of one subsystem need the same object, declare it once in `src/<subsystem>_internal.h`.
- **STR-4** Every `.cpp` MUST include its own header first, so the compiler checks each definition against its declaration.
- **STR-5** Every subsystem exposes the same lifecycle API, with exactly these names:

  | Function | Called from | Contract |
  | --- | --- | --- |
  | `init()` | `initialize()` | Configure devices. No motion, because motors do not run until the robot is enabled. |
  | `handleInput()` | driver loop | Read the controller and turn presses into requests. No motor output. |
  | `update()` | driver loop or a blocking routine, once per loop | Run the state machine and write motor outputs. No controller reads. Never blocks. |
  | `request<Action>(...)` | other subsystems, `commands` | Ask for a state change that `update()` carries out. Safe to call every loop. |
  | `is<State>()`, `get<Quantity>()` | anyone | Read-only queries, marked `[[nodiscard]]`. |
  | `<action>Blocking()` | `autonomous()`, explicit test entry points | May block. MUST follow STR-9. |

- **STR-6** Only `handleInput()` functions and `commands.cpp` read the controller. Every button binding MUST appear in the control map comment at the top of `src/main.cpp`, and a binding change updates the map in the same commit. Two subsystems MUST NOT react to the same button in the same mode.
- **STR-7** `constants.h` MUST contain only `constexpr` values and the includes those values need. Motors, sensors, and the controller MUST NOT live there. The existing `constants::master` is a known violation that will move to its own header in a dedicated PR.
- **STR-8** `update()`, `handleInput()`, and everything they call MUST NOT block: no `pros::delay()` and no loops waiting on time or sensors. The driver loop runs every 20 ms. One blocked call freezes every subsystem and leaves every motor at its last command.
- **STR-9** A blocking routine (`*Blocking()`, tuning routines) MUST:
  - put every other subsystem in a safe state first (drivetrain stopped);
  - keep calling the `update()` of the subsystem it drives;
  - have a hard timeout;
  - have a driver abort button when it can run during driver control.
- **STR-10** Dependencies between subsystems flow one way. Each file comment lists what the file talks to (CMT-1). The current graph is `drivetrain → elevator`, `endeffector → elevator`, `commands → elevator, endeffector`. The elevator depends on no other subsystem. A change MUST NOT add a cycle.
- **STR-11** No duplicated logic. When a computation appears a second time (slew limiting, loop `dt`, encoder averaging, linear blend, highest motor current), it moves into `util` and both call sites use the shared version.
- **STR-12** Function bodies SHOULD be at most 40 lines and MUST be at most 60. Cognitive complexity MUST be at most 25, and nesting at most 4 levels; clang-tidy enforces both. A `switch` case longer than about 10 lines becomes a named function, such as `updateManualHoming()`.
- **STR-13** Order inside a `.cpp`:
  1. own header, then other includes;
  2. the subsystem namespace;
  3. an anonymous namespace holding file-local constants, types, state, and helpers, in that order;
  4. public functions, in the same order as the header.

  Define helpers before their first use. No forward declarations of file-local functions mid-file.
- **STR-14** No dead code: no functions declared without a definition, no unused functions or variables, no commented-out code, no `#if 0`. Delete it, because git keeps the history.
- **STR-15** Include what you use and nothing more. A header MUST NOT pull in heavy headers it does not need, such as `lemlib/api.hpp`, because every file that includes it pays the cost. Every header starts with `#pragma once` and compiles on its own.

---

## 2. Modern C++ (CPP)

The build passes `--std=gnu++26` (set in `common.mk`). Write **standard C++23**. Code MUST NOT rely on GNU extensions or C++26-only features, so it builds on every teammate's PROS toolchain and in clangd.

- **CPP-1** Use `constexpr` for compile-time values and `const` for runtime values that never change. MUST NOT use `#define` for constants or function-like macros.
- **CPP-2** Every local variable that is not modified after initialization MUST be `const`. clang-tidy enforces this with `misc-const-correctness`.
  ```cpp
  double error {target - currentRotations};        // bad
  const double error {target - currentRotations};  // good
  ```
- **CPP-3** Initialize every variable where it is declared, using braces `{}`. Declare one variable per statement: `double lowG {0}, lowF {0};` is not allowed.
- **CPP-4** No implicit narrowing. In standard C++, narrowing inside braces is ill-formed. GCC only warns, but clang (and therefore clangd) rejects it. Convert explicitly:
  ```cpp
  // bad: int32_t -> double inside braces is ill-formed
  double leftY {master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)};
  // good
  const double leftY {static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y))};
  ```
  Watch APIs that take `float` (LemLib) or `int32_t` (`move()`, `move_voltage()`): cast at the call and make sure the value fits.
- **CPP-5** Use `static_cast` for conversions. MUST NOT use C-style or functional casts. A `reinterpret_cast` or `const_cast` needs a comment explaining why it is safe.
- **CPP-6** Integer types: use fixed-width types spelled with `std::` (`std::uint32_t`, `std::int32_t`).
  - Timestamps from `pros::millis()` MUST be `std::uint32_t` and MUST NOT be stored in `int`.
  - Compute elapsed time as `now - start` in unsigned arithmetic, which stays correct when the counter wraps.
  - Do not mix signed and unsigned values in a comparison.
- **CPP-7** Every set of states or modes is an `enum class`.
  - A `switch` over an `enum class` MUST list every enumerator and MUST NOT have `default:`, so `-Wswitch` flags any new state that is not handled.
  - Case labels have no parentheses: `case Mode::Idle:`.
- **CPP-8** Give file-local entities internal linkage with an anonymous namespace, not with `static`:
  ```cpp
  namespace elevator {
      namespace {
          enum class Mode { Homing, Stowed /* ... */ };
          Mode currentMode {Mode::Homing};
          double positionForLevel(int level) { /* ... */ }
      }  // namespace

      void update() { /* ... */ }  // declared in elevator.h
  }  // namespace elevator
  ```
- **CPP-9** Group related mutable state into a struct and reset it in one assignment, instead of keeping loose variables that several places reset by hand:
  ```cpp
  struct SettleState {
      bool isSettled {false};
      double integral {0.0};
      std::uint32_t settleSinceMs {0};
  };
  SettleState settle {};
  // ...
  settle = {};  // one reset; a field cannot be forgotten
  ```
- **CPP-10** Reusable mechanisms (slew limiter, PID, timers, debouncers) SHOULD be small classes in `util` that keep their state as private members. Then every instance carries its own state, and the logic is written once:
  ```cpp
  class SlewLimiter {
  public:
      SlewLimiter(double riseMvPerSec, double fallMvPerSec);
      [[nodiscard]] double step(double targetMv, double dtSec);
      void reset();
  private:
      double riseMvPerSec;
      double fallMvPerSec;
      double outputMv {0.0};
  };
  ```
- **CPP-11** Use the standard library instead of hand-written loops: `std::clamp`, `std::lerp`, `std::midpoint`, `std::ranges::max`, `std::ranges::sort`, `std::accumulate`.
  ```cpp
  // bad
  int32_t highest {0};
  for (int32_t c : currents) { highest = std::max(highest, c); }
  // good (the motor group always has two motors, so the range is never empty)
  const std::int32_t highestMa {std::ranges::max(elevatorMotors.get_current_draw_all())};
  ```
- **CPP-12** Use `std::array` instead of C arrays, and `std::span` or `std::string_view` for non-owning views. A fixed text buffer is `std::array<char, N>`, filled with `std::snprintf(buf.data(), buf.size(), ...)`.
- **CPP-13** Manage every resource with RAII. MUST NOT use `new`, `delete`, `malloc`, or `free`. A file opened with `std::fopen` is owned by a `std::unique_ptr<std::FILE, decltype(&std::fclose)>`. Raw pointers are non-owning, and only appear where an API requires them (LemLib takes `pros::MotorGroup*`).
- **CPP-14** Mark functions `[[nodiscard]]` when ignoring their result is a bug: queries (`is*`, `get*`) and anything that returns success or failure.
- **CPP-15** A function that may fail to produce a value SHOULD return `std::optional<T>` instead of a `bool` plus out-parameters:
  ```cpp
  bool breakaway(double at, double& gravityMv, double& frictionMv);    // avoid
  std::optional<Breakaway> breakaway(double atRotations);              // prefer
  ```
- **CPP-16** Lambdas list their captures by name (`[height]`). MUST NOT use default captures (`[=]`, `[&]`).
- **CPP-17** Use `auto` only when the type is obvious from the right-hand side (`static_cast`, constructors) or cannot be spelled (lambdas). Do not use it for arithmetic results.
- **CPP-18** Floating point:
  - A `double` literal has a decimal point (`2500.0`, not `2500`).
  - A `float` parameter gets an `f` literal or an explicit cast.
  - Do not compare computed values with `==`. Compare against a tolerance, or make the producing function return the exact value by construction and say so in a comment. For example, a slew limiter can return `target` exactly once it is within one step.
- **CPP-19** Code MUST NOT throw exceptions or use them for control flow. A PROS task cannot usefully recover from one.
- **CPP-20** MUST NOT use `using namespace` in headers. `.cpp` files SHOULD NOT use it either; prefer `using` declarations for specific names.
- **CPP-21** Pass cheap types by value: arithmetic types, enums, and structs of 16 bytes or less. Pass everything else by `const&`.
- **CPP-22** Pure math on constants SHOULD be `constexpr` (for example `positionForLevel`). Check relationships between constants with `static_assert`:
  ```cpp
  static_assert(elevatorResettleTolerance > elevatorSettleTolerance, "resettle window must be wider than settle window");
  ```

---

## 3. Comments and documentation (CMT)

Comments are written for the next student who has to tune or debug this robot at a competition. Code says *what*. Comments say *why*: intent, physics, units, hardware quirks, and pitfalls.

- **CMT-1** Every first-party file starts with a file comment saying what the file is, the states or modes it contains, the controls it reads, and the subsystems it talks to:
  ```cpp
  /**
   * @file elevator.cpp
   * @brief Double-reverse-four-bar elevator: homing, preset levels, manual drive, and closed-loop hold.
   *
   * Controls: B tap = stow, B hold = re-home, Y/X/A = preset heights, Up/Down = one level, L1/L2 = manual.
   * Talks to: nothing. Other subsystems call the request*() functions.
   */
  ```
- **CMT-2** Every function declared in a header has a doc comment directly above it. It gives a one-sentence summary, then what a caller must know: when to call it, whether and how long it blocks, the units of every parameter and return value, and what happens on bad input or an unhomed mechanism.
  ```cpp
  /**
   * Raises the elevator by `rotations` from where it is now, capped at the top of travel.
   *
   * Non-blocking: update() performs the move. Does nothing until the elevator has homed.
   *
   * @param rotations distance to raise, in motor rotations (height per rotation varies on a DR4B)
   */
  void requestLift(double rotations);
  ```
  A private helper gets a one-line comment when its name does not say everything (units, side effects, preconditions).
- **CMT-3** Each enumerator of a state enum is documented inside the enum, on its own line or directly above it. Mode documentation MUST NOT sit in a separate block away from the enum.
- **CMT-4** Every constant in `constants.h` documents four things:
  - its unit (which also appears in the name, NAM-5);
  - its sign convention or valid range;
  - how it was measured or tuned;
  - what raising or lowering it does.

  The elevator PID and homing constants are the model to copy.
- **CMT-5** Explain why, not what. Comments MUST NOT restate the code (`// set mode to idle`) or teach C++ rules (`// braces are needed around a case that declares variables`). Do explain physics, hardware behavior, timing, safety reasons, and rejected alternatives, as in "current is used instead of velocity because the chain keeps creeping as it tightens".
- **CMT-6** Comments MUST stay true. When behavior, units, or a constant's meaning changes, every comment that describes it is updated in the same commit. A wrong comment is a bug, and reviewers treat it as one.
- **CMT-7** No history in comments ("~17% gentler than the first tune", "was 400 ms"). Put history in the commit message; a comment describes the code as it is now.
- **CMT-8** TODO format is `// TODO(owner): <what> — done when <condition>`.
  - Example: `// TODO(jacob): tune elevatorKd on the competition robot — done when a one-level step settles without overshoot.`
  - A bare `TODO tune` is not allowed.
  - A TODO that is done is deleted in the same commit.
  - Debug-only code (a `printf` marked "remove once tuned") MUST NOT be merged to `main`.
- **CMT-9** Style:
  - Line comments are `// ` with one space, in sentence case, written as full sentences.
  - Multi-line explanations use consecutive `//` lines.
  - Header doc comments (CMT-2) and file comments (CMT-1) use `/** ... */`.
  - Trailing comments are for short notes on a single declaration, placed two spaces after the code.
- **CMT-10** Positional library arguments that do not explain themselves, like those of LemLib's `Drivetrain` and `ControllerSettings` constructors, get a trailing comment for each argument naming the parameter and its unit. "Read the documentation" is not a comment.
- **CMT-11** A comment does not make a magic number acceptable. Name it instead (CON-3).

---

## 4. Naming (NAM)

| Entity | Style | Example |
| --- | --- | --- |
| Files | `snake_case` | `elevator_tune.cpp` |
| Namespaces | lowercase, one word | `elevator`, `endeffector` |
| Types, enums, enumerators | `PascalCase` | `enum class Mode { ManualHold }` |
| Functions, variables, parameters, members, constants | `camelCase` | `requestLift`, `elevatorKp` |
| Macros (avoid them) | `UPPER_SNAKE_CASE` | |

No prefixes (`k`, `m_`, `g_`) and no Hungarian notation. clang-tidy's `readability-identifier-naming` enforces the table.

- **NAM-1** Names say what the thing is in robot terms: `throttleVoltageMv`, `liftTargetRotations`, not `v`, `t`, `g`, or `ok`. Single letters are only for loop indices and for short formulas whose meaning is documented on the spot.
- **NAM-2** Booleans read as yes/no questions: `isHomed`, `isSettled`, `hasPiece`, `wasAborted`.
- **NAM-3** Function verbs follow STR-5: `request*` (asynchronous), `*Blocking` (blocks), `is*` and `get*` (queries), plus exactly `init`, `handleInput`, and `update`.
- **NAM-4** Do not repeat the namespace in the name: `elevator::isStowed()`, not `elevator::isElevatorStowed()`. Entries in `constants::` keep their subsystem prefix (`constants::elevatorKp`) because that namespace is shared.
- **NAM-5** Every physical quantity ends in its unit, and the unit MUST be correct:

  | Suffix | Unit |
  | --- | --- |
  | `Mv` | millivolts (`move_voltage()`, ±12000) |
  | `Power` | PROS `move()` units (±127) |
  | `Ms`, `Sec` | milliseconds, seconds |
  | `Rotations` | motor rotations |
  | `Rpm` | rotations per minute |
  | `Ma` | milliamps |
  | `MvPerSec` | millivolts per second (slew rates) |
  | `In`, `Deg` | inches, degrees |

  A ratio from 0 to 1 says so in its name (`heightFraction`, `speedScale`). A gain states its unit in its comment (`// mV per rotation of error`).
- **NAM-6** Name a function after its behavior, not the library behind it: `arcadeDrive()`, not `lemlibArcade()`.
- **NAM-7** Spell names the same way everywhere: "end effector" in prose and commit scopes, `endeffector` in code.

---

## 5. Constants, units, and tunables (CON)

- **CON-1** Every port, gear ratio, and tuned number lives in `include/constants.h`, grouped by subsystem under a `// ---- <subsystem> ----` banner. A value used only inside one file's implementation MAY live in that file's anonymous namespace instead (for example auto-tune test parameters). It is still `constexpr`, named, and documented.
- **CON-2** `constants.h` holds `constexpr` values only (STR-7).
- **CON-3** No magic numbers in `src/`. Every literal MUST be a named constant, except `0`, `1`, `-1`, a `2` used to halve or double, and `1000.0` used to convert between ms and s. This includes the hardware's own limits:
  ```cpp
  constexpr double maxVoltageMv {12000.0};    // V5 motor voltage range is ±12000 mV
  constexpr double maxStickValue {127.0};     // controller analog range is ±127
  constexpr std::uint32_t loopPeriodMs {20};  // driver-control loop period
  ```
- **CON-4** Compute derived values instead of copying them. For example, the top elevator level is a `constexpr` derived from `elevatorMajorIncrements.size()` and `elevatorStepsPerSection`, defined in one place. If two constants must keep a relationship, `static_assert` it (CPP-22).
- **CON-5** A literal's type matches the declared type: `constexpr double endEffectorHoldMv {2500.0};`. Each subsystem banner documents the sign convention (positive = up, forward, intake).
- **CON-6** Changing a tuned value is a behavior change. Its commit (type `tune`) gives the old value, the new value, and the reason (a measurement or driver feedback).
- **CON-7** A port constant is a valid V5 smart port: 1 to 21, negative for reversed. A device that is not installed does not get a fake port such as `0`. Leave it out instead, for example by passing `nullptr` to LemLib's `OdomSensors`.

---

## 6. Safety and real-time behavior (SAF)

The robot is heavy, fast, and run near people. When code fails, it must fail to a stopped, safe state.

- **SAF-1** Every motor command is bounded before it is sent: voltages clamped to ±`maxVoltageMv`, `move()` power to ±127.
- **SAF-2** Every wait or loop that depends on hardware has a timeout. Every motion that can run during driver control can be aborted by the driver.
- **SAF-3** Check for PROS errors before acting on a reading. Motor and sensor getters return `PROS_ERR` (`INT32_MAX`) or `PROS_ERR_F` (infinity) when a device is unplugged or fails. Stall detection, position control, and travel limits MUST treat these values as "no reading" and put the mechanism in a safe state (stop or hold). They MUST NOT read them as a huge current or a far-away position.
  ```cpp
  const std::int32_t currentMa {endEffector.get_current_draw()};
  if (currentMa == PROS_ERR) {  // unplugged: no reading, so this is not a stall
      // ...
  }
  ```
- **SAF-4** Nothing called from the 20 ms driver loop blocks (STR-8). V5 motors keep running at their last command until told otherwise, so a routine that must block first stops the drivetrain (STR-9).
- **SAF-5** Every state machine has a safe default: an interrupted or unknown state brakes the mechanism. Switching modes resets the previous mode's state in one place (CPP-9).
- **SAF-6** Every time-based term (integrals, slew rates, timers) uses the measured loop time `dtSec`. Do not mix the measured `dt` with a hard-coded `0.02`.
- **SAF-7** No unconditional `printf` in code that runs every loop. Diagnostic output is gated behind a `constexpr bool` in `constants.h` (for example `constants::debugElevatorHoming`) that is `false` on `main`.
- **SAF-8** New per-loop code SHOULD NOT allocate on the heap when a fixed-size alternative exists. The `std::vector` returned by PROS `*_all()` getters is acceptable.
- **SAF-9** Global device objects are the normal PROS pattern and are allowed. They live in the `.cpp` of the subsystem that owns them (STR-3), and a static initializer in another file MUST NOT touch them.

---

## 7. Formatting (FMT)

- **FMT-1** The repository's `.clang-format` is the source of truth. Run `clang-format -i <file>` on every first-party file you change; in VS Code, use Format Document with the C/C++ extension or clangd, which both read the repository file. Never format vendored code.
- **FMT-2** The resulting style:
  - 4-space indent, no tabs, 120-column limit;
  - attached braces (`if (x) {`), and braces always, even around one-line bodies;
  - a space after `if`, `for`, `while`, and `switch`;
  - indented namespace contents, closed with `}  // namespace name`;
  - a newline at the end of every file;
  - includes grouped as own header, standard library, PROS and LemLib, then project headers.
- **FMT-3** Formatting-only changes go in their own commit (`style: ...`). Never mix them with logic changes, so reviewers see the real diff.

---

## 8. Verification, git, and review (VER)

- **VER-1** The code MUST build with `pros make` with no errors and no new warnings.
- **VER-2** Run `tools/check.sh --changed` before pushing. It checks formatting, strict compiler warnings, and clang-tidy. Lines you add or change MUST be clean. Pre-existing findings on lines you did not touch do not block a PR.
- **VER-3** Pure logic (math, level mapping, slew, PID) lives in hardware-free functions that take values instead of reading devices, so it can be unit tested on a laptop.
- **VER-4** Every PR states how it was tested and what was checked: "built only", "on blocks", or "driven". If tuned values changed, the PR says on which robot. "Works" without saying how is not accepted.
- **VER-5** Commits follow Conventional Commits: `type(scope): summary`, imperative and lowercase, at most 72 characters, one logical change per commit.
  - Types: `feat`, `fix`, `refactor`, `tune` (constant values only), `style`, `docs`, `chore`.
  - Scopes: `drivetrain`, `elevator`, `endeffector`, `commands`, `constants`, `build`, `docs`.
- **VER-6** Make one branch per change, cut from `main`. PRs into `main` need approval from a CODEOWNER. Never commit build output; `.gitignore` already covers `bin/` and `compile_commands.json`.

### Review checklist

- [ ] Builds; `tools/check.sh --changed` is clean on changed lines (VER-1, VER-2)
- [ ] New or changed public functions have doc comments; affected comments are updated (CMT-2, CMT-6)
- [ ] No new magic numbers; new tunables are in `constants.h` and fully documented (CON-3, CMT-4)
- [ ] Nothing blocks in `update()` or `handleInput()`; every wait has a timeout (STR-8, SAF-2)
- [ ] New sensor reads handle `PROS_ERR` (SAF-3)
- [ ] No duplicated logic and no dead code (STR-11, STR-14)
- [ ] Everything not in a header has internal linkage (STR-3)
- [ ] Commit messages and PR testing notes follow VER-4 and VER-5
