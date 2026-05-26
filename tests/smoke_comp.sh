#!/usr/bin/env bash
# Smoke test for the up-vision CheckComp detector.
#
# Builds the offline tune_comp rig and runs it against every subdirectory under
# the corpus root. Each subdirectory is treated as one part class; the smoke
# passes if at least 2/3 of the .png frames in each class yield a
# plausible-aspect detection (10 <= min(w,h), max(w,h) <= 200, aspect <= 6:1).
#
# The default corpus is the small representative set in tests/corpus/ shipped
# with the public repo. Larger private corpora can be pointed at via
# TVM802_CORPUS, e.g. TVM802_CORPUS=$HOME/qihetvm802b tests/smoke_comp.sh -- in
# that case subdirs that LOOK like component classes are tested and
# fiducial-only subdirs are skipped.
#
# Run BEFORE every detector change and BEFORE every deploy. Detector edits
# that drop a class below the 2/3 floor fail the smoke and don't ship.
#
# Usage:  tests/smoke_comp.sh
set -uo pipefail

REPO=$(cd "$(dirname "$0")/.." && pwd)
CORPUS_ROOT="${TVM802_CORPUS:-$REPO/tests/corpus}"
RIG=/tmp/tune_comp

echo "== building tune_comp =="
cd "$REPO"
CV_FLAGS=$(pkg-config --cflags --libs opencv4)
# shellcheck disable=SC2086 # word-split CV_FLAGS for the compiler driver
g++ -std=c++23 -O2 -pthread -I src tests/tune_comp.cpp \
    src/detect_common.cpp src/detect_circle.cpp src/detect_contour.cpp \
    src/detect_template.cpp src/detect_symmetry.cpp src/detect_component.cpp \
    src/settings.cpp src/thread_pool.cpp \
    $CV_FLAGS -o "$RIG"
[ -x "$RIG" ] || { echo "BUILD FAILED"; exit 1; }

[ -d "$CORPUS_ROOT" ] || { echo "MISSING corpus root: $CORPUS_ROOT"; exit 1; }

fail=0
for subdir in "$CORPUS_ROOT"/*/; do
    [ -d "$subdir" ] || continue
    class=$(basename "$subdir")
    case "$class" in
        # Fiducial corpus is preserved on disk but uses a different detector
        # (CheckMark / CheckMark2) -- not exercised by tune_comp. Skipped here
        # with a note; future fiducial smoke goes in its own rig.
        fid*|fiducial*) echo "  SKIP $class (component detector only)"; continue ;;
    esac
    # Find frames; both comp_NNNN.png (live capture format) and free-form names
    # (corpus format) supported.
    ls "$subdir"*.png 2>/dev/null > /tmp/smoke_list.txt
    n=$(wc -l < /tmp/smoke_list.txt)
    [ "$n" -gt 0 ] || { echo "  EMPTY $class"; continue; }
    xargs "$RIG" < /tmp/smoke_list.txt 2>/dev/null > /tmp/smoke_out.csv
    found=$(awk -F, 'NR>1 && $2==1 {
        w=$5+0; h=$6+0; lo=(w<h?w:h); hi=(w<h?h:w);
        if (lo>=10 && hi<=200 && hi/lo<=6.0) c++
    } END{print c+0}' /tmp/smoke_out.csv)
    floor=$(( (n * 2 + 2) / 3 ))  # >= 2/3 of frames, rounded up
    if [ "$found" -lt "$floor" ]; then
        echo "  FAIL $class: $found / $n  (floor $floor = 2/3)"
        # show a few detected sizes so a regression is debuggable
        awk -F, 'NR>1 && $2==1 {printf "%.0f %.0f\n",$5,$6}' /tmp/smoke_out.csv \
            | sort | uniq -c | sort -rn | head -3 | sed 's/^/    sizes: /'
        fail=1
    else
        echo "  OK   $class: $found / $n"
    fi
done

if [ "$fail" -ne 0 ]; then
    echo
    echo "SMOKE FAIL"
    exit 1
fi
echo
echo "SMOKE OK"
