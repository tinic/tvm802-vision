#!/usr/bin/env bash
# Local mirror of the CI `lint` gate (.github/workflows/build.yml).
#
# Run from anywhere: tools/check.sh
# Gates, in order: clang-format, cppcheck, clang-tidy (portable detector core),
# host-arch build + ctest. Each runs even if an earlier one fails; the script
# exits non-zero if ANY gate failed.
#
# The MSVC /W4 /WX warnings-as-errors build of the Windows-GDI translation units
# (preview/passthrough/capture/dllmain, which need <windows.h>) is the CI `build`
# job's responsibility — it can't run on a Linux dev box.
#
# Requires: clang-format, cppcheck, clang-tidy, cmake, a C++ compiler, OpenCV
# (core/imgproc/imgcodecs) dev headers.
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

gate "clang-tidy (portable detector core: src/vision.cpp)"
clang-tidy -p build-lint src/vision.cpp
result $?

gate "host build + ctest"
cmake --build build-lint >/dev/null && ctest --test-dir build-lint --output-on-failure
result $?

echo
if [ "$rc" -eq 0 ]; then echo "ALL GATES GREEN"; else echo "GATES FAILED (rc=$rc)"; fi
exit "$rc"
