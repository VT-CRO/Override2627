# CLAUDE.md

Instructions for AI agents working in this repository. Read all of it before editing anything.

The full rulebook is [docs/CODING_GUIDELINES.md](docs/CODING_GUIDELINES.md). Rule IDs below (`STR-3`, `CPP-4`, ...) refer to it. If a request conflicts with a rule here, say which rule it breaks and ask before going ahead.

## Project

VEX V5 competition robot code for VT-CRO's 2026–27 robot. It is built on the PROS 4.2.2 kernel and LemLib 0.5.6 and runs on the V5 brain (ARM Cortex-A9, PROS RTOS). The build passes `--std=gnu++26`; write standard C++23 (see the intro to section 2 of the guidelines).

The code drives real hardware. There are no unit tests, and you cannot run it. You can only build it and check it statically.

### Layout

| Path | What it is |
| --- | --- |
| `src/main.cpp` | Competition entry points. `opcontrol()` is the 20 ms driver loop calling each subsystem. |
| `src/drivetrain.cpp`, `include/drivetrain.h` | Six-motor tank drive with arcade control and anti-tip slew that scales with elevator height. |
| `src/elevator.cpp`, `include/elevator.h` | DR4B elevator state machine: homing, preset levels, manual drive, PID hold with gravity feedforward. |
| `src/elevator_tune.cpp` | Blocking auto-tune routine (hold LEFT + RIGHT, robot on blocks). Prints constants to paste into `constants.h`. |
| `src/endeffector.cpp`, `include/endeffector.h` | One-motor claw and wheels: intake, stall-detected seat, hold, release. |
| `src/commands.cpp`, `include/commands.h` | Actions that coordinate more than one subsystem (score). |
| `include/constants.h` | Every port, gear ratio, and tuned value, documented. |
| `docs/CODING_GUIDELINES.md` | Coding rules. |
| `tools/check.sh` | Format, strict-warning, and clang-tidy check for first-party code. |

Everything else in `include/` and all of `firmware/` is vendored (hard rule 1).

### How the code is shaped

Each subsystem is a namespace whose state lives in its `.cpp`. It exposes `init()`, `handleInput()` (reads the controller), and `update()` (runs the state machine and writes the motors), plus `request*()` functions that other subsystems call. Follow this shape for new code (STR-5).

The existing code predates the guidelines and has known violations: `static` instead of anonymous namespaces, magic numbers, slew logic copied three times, very long `update()` functions. **Do not copy a pattern from existing code that contradicts the guidelines. Follow the guidelines.**

## Hard rules

Never break these unless the user explicitly approves it in the current conversation.

1. **Do not edit or reformat vendored code**: `include/pros/`, `include/lemlib/`, `include/liblvgl/`, `include/fmt/`, `include/api.h`, `firmware/`, `common.mk`. Edit `include/main.h`, `Makefile`, and `project.pros` only where the PROS template marks a section as user-editable.
2. **Do not change motor ports, gear ratios, or tuned values** in `constants.h` unless the task is to change that value. When new code needs a new tunable:
   - add it with a reasoned starting value;
   - mark it `// TODO(owner): tune ... — done when ...`;
   - tell the user it must be tuned on the robot.
3. **Do not remove or weaken a safety mechanism**: voltage clamps, travel limits (`manualPastLimit`), timeouts, abort buttons, integral anti-windup, slew limits, settle hysteresis.
4. **Do not add blocking code** (`pros::delay()`, loops waiting on time or sensors) to anything reachable from `update()` or `handleInput()` (STR-8). Blocking routines follow STR-9.
5. **Do not claim robot behavior is verified.** Say "builds and passes checks; not tested on the robot". Never write that something works on the robot.
6. **Stay in scope.** Change only what the task needs: no drive-by refactors, renames, or reformatting of lines you did not otherwise change. List what you noticed as suggestions instead. Formatting-only and refactor-only changes get their own commits (FMT-3).
7. **No new libraries, templates, or `pros c` commands** without asking.
8. **Do not commit, push, or open a PR unless asked.** Never force-push, and never push to `main` directly. `main` requires an approving CODEOWNERS review.

## Code rules (summary; the guidelines are binding)

**Structure**
- Public API goes in `include/<subsystem>.h`. Everything else goes in an anonymous namespace in the `.cpp` (STR-3, CPP-8). Never reach into another file with `extern`.
- Each `.cpp` includes its own header first (STR-4).
- Only `handleInput()` and `commands.cpp` read the controller. Bindings are listed in the control map at the top of `src/main.cpp` (STR-6).
- Logic that appears twice moves into `util` (STR-11).
- Functions are at most 60 lines with cognitive complexity at most 25 (STR-12).
- No dead code or commented-out code (STR-14).

**Modern C++**
- Make every local that is never modified `const` (CPP-2).
- Brace-initialize with no narrowing, and use `static_cast` for conversions (CPP-3, CPP-4, CPP-5).
- Store timestamps as `std::uint32_t` (CPP-6).
- Use `enum class` with an exhaustive `switch` and no `default:` (CPP-7).
- Group related state in a struct that resets in one assignment (CPP-9).
- Prefer standard algorithms (`std::ranges::max`, `std::lerp`, `std::clamp`) over hand-written loops (CPP-11).
- Use `std::array` and RAII; never `new` or `delete` (CPP-12, CPP-13).
- Mark queries `[[nodiscard]]` (CPP-14).

**Comments**
- Every file starts with a file comment (CMT-1).
- Every function in a header has a doc comment covering when to call it, whether it blocks, and units (CMT-2).
- Explain why, not what (CMT-5).
- When behavior changes, update every comment that describes it (CMT-6). A stale comment is a bug.
- TODOs use the form `// TODO(owner): what — done when` (CMT-8).
- Write comments as `// ` followed by a sentence-case sentence (CMT-9).

**Naming**
- Functions, variables, and constants are `camelCase`. Types and enumerators are `PascalCase`. Namespaces are lowercase.
- Every physical quantity carries a correct unit suffix such as `Mv`, `Ms`, `Rotations`, or `Ma` (NAM-5).
- Booleans read as questions, like `isHomed` (NAM-2).

**Constants**
- No magic numbers in `src/` (CON-3).
- `constants.h` holds `constexpr` values only (STR-7).
- Every tunable documents its unit, range, how it was measured, and the effect of changing it (CMT-4).

**Safety**
- Treat `PROS_ERR` and `PROS_ERR_F` as "no reading" and fail safe (SAF-3).
- Clamp every motor command (SAF-1).
- Give every hardware wait a timeout and a driver abort (SAF-2).
- No `printf` that runs every loop (SAF-7).

## Workflow

1. **Read before writing.** Read the files you will change and the headers they include. Find every caller of a function you change by searching for its name across `src/` and the top-level headers in `include/`.
2. **Make the change** following the rules above.
3. **Format only the first-party files you changed:** `clang-format -i <files>`.
4. **Verify.** Build:
   ```bash
   pros make
   ```
   Then run the guideline checks on changed files:
   ```bash
   tools/check.sh --changed
   ```
   Fix every finding on lines you added or changed. Leave pre-existing findings on untouched lines alone (hard rule 6), but mention them.
5. **Report:**
   - what changed and why;
   - what you verified (build, check script);
   - what you could not verify (behavior on the robot);
   - any constants that need tuning;
   - any guideline violations you saw nearby but did not fix.

## Commits and PRs (when asked)

- Use `type(scope): summary`: imperative, lowercase, at most 72 characters (VER-5).
  - Types: `feat`, `fix`, `refactor`, `tune`, `style`, `docs`, `chore`.
  - Scopes: `drivetrain`, `elevator`, `endeffector`, `commands`, `constants`, `build`, `docs`.
- Make one logical change per commit. A `tune` commit gives the old value, the new value, and why (CON-6).
- A PR description states how the change was tested: "built only", "on blocks", or "driven" (VER-4).
