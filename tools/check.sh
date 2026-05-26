#!/usr/bin/env bash
# Local mirror of the CI `lint` gate (.github/workflows/build.yml), plus one
# local-only extra (the mingw clang-tidy pass below).
#
# Run from anywhere: tools/check.sh
# Gates, in order: clang-format, cppcheck, clang-tidy (portable Windows-free TUs),
# clang-tidy modernization of the OpenCV-free Windows TUs via mingw (skipped if
# the toolchain is absent), host-arch build + ctest. Each runs even if an earlier
# one fails; the script exits non-zero if ANY gate failed.
#
# The MSVC /W4 /WX warnings-as-errors build of the Windows-GDI translation units
# (preview/passthrough/capture/dllmain, which need <windows.h>) is the CI `build`
# job's responsibility — it can't run on a Linux dev box.
#
# Requires: clang-format, cppcheck, clang-tidy, cmake, a C++ compiler, OpenCV
# (core/imgproc/imgcodecs) dev headers. Optional: an i686-w64-mingw32 toolchain
# (for the Windows-TU modernization gate).
set -uo pipefail
cd "$(dirname "$0")/.."

rc=0
gate() { printf '\n== %s ==\n' "$1"; }
result() { if [ "$1" -eq 0 ]; then echo "  OK"; else echo "  FAIL"; rc=1; fi; }

gate "clang-format"
clang-format --dry-run --Werror src/*.cpp src/*.h
result $?

gate "cppcheck"
# --suppress=unusedFunction: this is a 36-export DLL + DllMain; every export and
# the OS-called entry point look "unused" to whole-program analysis.
cppcheck --enable=all --std=c++17 --language=c++ -I src \
    --suppress=missingIncludeSystem --suppress=checkersReport \
    --suppress=normalCheckLevelMaxBranches --suppress=unmatchedSuppression \
    --suppress=unusedFunction --error-exitcode=2 src
result $?

gate "configure (host tests + compile DB for clang-tidy)"
cmake -S . -B build-lint -DBUILD_TESTS=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON >/dev/null
result $?

gate "clang-tidy (portable, Windows-free TUs: src/detect_*.cpp + settings.cpp)"
# Parallel across cores via xargs -P. --header-filter restricts warning
# emission to our own src/ so the OpenCV system header tree is parsed but
# not analysed. xargs returns non-zero if any child fails, propagated to
# `result` -- one TU's errors still fail the gate. Several-x faster than
# the per-file serial run.
N=$(nproc 2>/dev/null || echo 4)
printf '%s\n' src/detect_common.cpp src/detect_circle.cpp \
    src/detect_contour.cpp src/detect_template.cpp src/detect_symmetry.cpp \
    src/detect_component.cpp src/settings.cpp src/thread_pool.cpp |
    xargs -P "$N" -n 1 clang-tidy -p build-lint --quiet \
        --header-filter='.*/src/.*'
result $?

# Local-only: the OpenCV-free Windows TUs (capture/controller/settings_ui) need
# <windows.h>, so the stock Linux clang-tidy can't see them. If an i686 mingw
# toolchain is present, tidy them against the mingw target (no compile DB / no
# OpenCV needed) with the FULL .clang-tidy config. Skipped (not failed) when the
# toolchain is absent. passthrough/preview/dllmain need OpenCV -> MSVC build only.
gate "clang-tidy (OpenCV-free Windows TUs via mingw)"
if command -v i686-w64-mingw32-g++ >/dev/null 2>&1; then
    clang-tidy src/settings_ui.cpp src/capture.cpp src/controller.cpp -- \
        --target=i686-w64-windows-gnu -std=c++23 -I src -DNOMINMAX -D_WIN32_WINNT=0x0601
    result $?
else
    echo "  SKIP (no i686-w64-mingw32 toolchain)"
fi

gate "host build + ctest"
cmake --build build-lint >/dev/null && ctest --test-dir build-lint --output-on-failure
result $?

echo
if [ "$rc" -eq 0 ]; then echo "ALL GATES GREEN"; else echo "GATES FAILED (rc=$rc)"; fi
exit "$rc"
