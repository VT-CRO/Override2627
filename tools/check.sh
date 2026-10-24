#!/usr/bin/env bash
# Checks first-party code against docs/CODING_GUIDELINES.md:
#   1. formatting       clang-format with the repository's .clang-format (FMT)
#   2. strict warnings  arm-none-eabi-g++ -fsyntax-only with extra warnings (CPP-4, CPP-6, ...)
#   3. static analysis  clang-tidy with the repository's .clang-tidy, if clang-tidy is installed
# Vendored code (include/pros, include/lemlib, ...) is compiled but never reported.
#
# usage: tools/check.sh [--changed | file ...]
#   (no arguments)  check every first-party file
#   --changed       check first-party files that differ from origin/main, committed or not
#   file ...        check only these files
set -uo pipefail

cd "$(git rev-parse --show-toplevel)" || exit 2

first_party() {
    git ls-files --cached --others --exclude-standard -- \
        ':(glob)src/*.cpp' ':(glob)src/*.h' ':(glob)include/*.h' ':(glob)include/*.hpp' |
        grep -vxE 'include/(api|main)\.h' | sort -u
}

changed_files() {
    local base
    base="$(git merge-base HEAD origin/main 2>/dev/null || echo HEAD)"
    { git diff --name-only "$base" --; git ls-files --others --exclude-standard; } | sort -u
}

if [[ "${1:-}" == "--changed" ]]; then
    mapfile -t files < <(comm -12 <(first_party) <(changed_files))
elif (( $# > 0 )); then
    files=("$@")
else
    mapfile -t files < <(first_party)
fi

existing=()
for f in "${files[@]}"; do
    [[ -f "$f" ]] && existing+=("$f")
done
files=("${existing[@]}")
if (( ${#files[@]} == 0 )); then
    echo "No first-party files to check."
    exit 0
fi

# Diagnostics are reported only for the selected files.
report_pattern="^($(printf '%s\n' "${files[@]}" | sed 's/[.]/\\./g' | paste -sd'|')):[0-9]+:[0-9]+: (warning|error):"
# A selected header can produce findings in any source that includes it, so compile every source then.
if printf '%s\n' "${files[@]}" | grep -qE '\.(h|hpp)$'; then
    mapfile -t sources < <(first_party | grep -E '\.cpp$')
else
    sources=("${files[@]}")
fi
failed=0

echo "== Formatting (clang-format)"
if command -v clang-format >/dev/null; then
    for f in "${files[@]}"; do
        lines="$(clang-format "$f" | diff "$f" - | grep -c '^<')"
        if (( lines > 0 )); then
            echo "  $f: $lines lines differ (fix: clang-format -i $f)"
            failed=1
        fi
    done
else
    echo "  skipped: clang-format not installed"
fi

echo "== Strict warnings (arm-none-eabi-g++)"
std="$(sed -nE 's/^CXX_STANDARD\?=(.*)$/\1/p' common.mk)"
gcc_flags=(
    -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=hard -mthumb -Os "--std=${std:-gnu++26}"
    -D_POSIX_THREADS -D_UNIX98_THREAD_MUTEX_ATTRIBUTES -D_POSIX_TIMERS -D_POSIX_MONOTONIC_CLOCK
    -Iinclude -iquote include -fsyntax-only -fdiagnostics-color=never
    -Wall -Wextra -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast -Wuseless-cast
    -Wdouble-promotion -Wimplicit-fallthrough -Wnull-dereference -Wno-psabi
)
if command -v arm-none-eabi-g++ >/dev/null; then
    gcc_out="$(for s in "${sources[@]}"; do arm-none-eabi-g++ "${gcc_flags[@]}" "$s" 2>&1; done |
        grep -E "$report_pattern" | sort -u)"
    if [[ -n "$gcc_out" ]]; then
        echo "$gcc_out" | sed 's/^/  /'
        failed=1
    fi
else
    echo "  skipped: arm-none-eabi-g++ not on PATH"
fi

echo "== Static analysis (clang-tidy)"
if command -v clang-tidy >/dev/null && command -v arm-none-eabi-g++ >/dev/null; then
    # clang needs the ARM toolchain's own standard library headers.
    mapfile -t sysdirs < <(arm-none-eabi-g++ -mcpu=cortex-a9 -mfloat-abi=hard -mthumb -E -x c++ - -v </dev/null 2>&1 |
        sed -n '/#include <...> search starts here:/,/End of search list./p' | sed '1d;$d;s/^ *//')
    tidy_flags=(--target=arm-none-eabi -mcpu=cortex-a9 -mfloat-abi=hard -mthumb "-std=${std:-gnu++26}" -nostdinc -nostdinc++)
    for d in "${sysdirs[@]}"; do
        tidy_flags+=(-isystem "$d")
    done
    # FMT_CONSTEVAL= works around a clang-only consteval error inside the vendored fmt headers.
    tidy_flags+=(-Iinclude -D_POSIX_THREADS -D_UNIX98_THREAD_MUTEX_ATTRIBUTES -D_POSIX_TIMERS
        -D_POSIX_MONOTONIC_CLOCK -DFMT_CONSTEVAL=)
    tmp="$(mktemp -d)"
    for s in "${sources[@]}"; do
        clang-tidy --quiet "$s" -- "${tidy_flags[@]}" >"$tmp/$(basename "$s").log" 2>&1 &
    done
    wait
    tidy_out="$(cat "$tmp"/*.log | sed "s#^$PWD/##" | grep -E "$report_pattern" | sort -u)"
    rm -rf "$tmp"
    if [[ -n "$tidy_out" ]]; then
        echo "$tidy_out" | sed 's/^/  /'
        echo "  $(echo "$tidy_out" | wc -l) findings"
        failed=1
    fi
else
    echo "  skipped: clang-tidy or arm-none-eabi-g++ not installed"
fi

if (( failed )); then
    echo "FAIL: fix findings on lines you added or changed (docs/CODING_GUIDELINES.md, VER-2)."
    exit 1
fi
echo "OK"
