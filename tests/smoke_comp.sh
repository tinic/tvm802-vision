#!/usr/bin/env bash
# Smoke test for both detectors -- the up-vision CheckComp component pipeline
# and the down-vision circular-symmetry fiducial pipeline.
#
# Builds two offline rigs (tune_comp + tune_csym) and runs them against every
# subdirectory under the corpus root. A subdirectory whose name starts with
# `fid` or `fiducial` is treated as fiducial input and run through tune_csym;
# everything else is treated as a part class and run through tune_comp.
#
# Pass criteria (>= 2/3 of frames per subdir, rounded up):
#   component: a plausible-aspect detection -- 10 <= min(w,h), max(w,h) <= 200,
#              aspect <= 6:1.
#   fiducial:  a lock with detected radius >= 5 px and quality > 1.0 -- catches
#              both "missed entirely" and "locked on noise" regressions.
#
# The default corpus is the small representative set in tests/corpus/ shipped
# with the public repo. Larger private corpora can be pointed at via
# TVM802_CORPUS, e.g. TVM802_CORPUS=$HOME/qihetvm802b tests/smoke_comp.sh.
#
# Run BEFORE every detector change and BEFORE every deploy. Detector edits
# that drop a class below the 2/3 floor fail the smoke and don't ship.
#
# Usage:  tests/smoke_comp.sh
set -uo pipefail

REPO=$(cd "$(dirname "$0")/.." && pwd)
CORPUS_ROOT="${TVM802_CORPUS:-$REPO/tests/corpus}"
COMP_RIG=/tmp/tune_comp
CSYM_RIG=/tmp/tune_csym

cd "$REPO"
CV_FLAGS=$(pkg-config --cflags --libs opencv4)

BUILD_LOG=/tmp/smoke_build.log
echo "== building tune_comp =="
# shellcheck disable=SC2086 # word-split CV_FLAGS for the compiler driver
g++ -std=c++23 -O2 -pthread -I src tests/tune_comp.cpp \
    src/detect_common.cpp src/detect_circle.cpp src/detect_contour.cpp \
    src/detect_template.cpp src/detect_symmetry.cpp src/detect_component.cpp \
    src/settings.cpp src/thread_pool.cpp \
    $CV_FLAGS -o "$COMP_RIG" 2>"$BUILD_LOG"
[ -x "$COMP_RIG" ] || { cat "$BUILD_LOG"; echo "BUILD FAILED (tune_comp)"; exit 1; }

echo "== building tune_csym =="
# shellcheck disable=SC2086
g++ -std=c++23 -O2 -pthread -I src tests/tune_csym.cpp \
    src/detect_common.cpp src/detect_circle.cpp src/detect_contour.cpp \
    src/detect_template.cpp src/detect_symmetry.cpp \
    src/settings.cpp src/thread_pool.cpp \
    $CV_FLAGS -o "$CSYM_RIG" 2>>"$BUILD_LOG"
[ -x "$CSYM_RIG" ] || { cat "$BUILD_LOG"; echo "BUILD FAILED (tune_csym)"; exit 1; }

[ -d "$CORPUS_ROOT" ] || { echo "MISSING corpus root: $CORPUS_ROOT"; exit 1; }

# Fiducial smoke ref-point: 370/270 / r=45 -- the host's typical mark-search
# location for the public corpus frames. Tight enough to reject noisy locks
# but wide enough to tolerate motion drift across the 3-frame set.
FID_REFX=370
FID_REFY=270
FID_R=45

# Per-frame regression baseline (tests/corpus/baselines.csv). For comp classes,
# every (cx, cy, w, h, angle) must match the baseline within tolerance, and
# (found, method) must match exactly. Catches detector drift that the
# plausible-aspect floor check would miss. Regenerate after an intentional
# detector change with tools/regen_baselines.sh.
BASELINE="$REPO/tests/corpus/baselines.csv"

fail=0
for subdir in "$CORPUS_ROOT"/*/; do
    [ -d "$subdir" ] || continue
    class=$(basename "$subdir")
    ls "$subdir"*.png 2>/dev/null > /tmp/smoke_list.txt
    n=$(wc -l < /tmp/smoke_list.txt)
    [ "$n" -gt 0 ] || { echo "  EMPTY $class"; continue; }
    floor=$(( (n * 2 + 2) / 3 ))  # >= 2/3 of frames, rounded up

    case "$class" in
        fid*|fiducial*)
            # tune_csym CLI: --refx --refy --r before the frame paths
            { printf -- '--refx\n%s\n--refy\n%s\n--r\n%s\n' "$FID_REFX" "$FID_REFY" "$FID_R"; \
              cat /tmp/smoke_list.txt; } | xargs "$CSYM_RIG" 2>/dev/null > /tmp/smoke_out.csv
            # csv: file,found,cx,cy,radius,score  -- pass if radius >= 5 && score > 1
            found=$(awk -F, 'NR>1 && $2==1 && $5+0>=5 && $6+0>1.0 {c++} END{print c+0}' \
                    /tmp/smoke_out.csv)
            label="fid"
            ;;
        *)
            xargs "$COMP_RIG" < /tmp/smoke_list.txt 2>/dev/null > /tmp/smoke_out.csv
            # csv: file,found,...,w,h  -- pass if plausible aspect
            found=$(awk -F, 'NR>1 && $2==1 {
                w=$5+0; h=$6+0; lo=(w<h?w:h); hi=(w<h?h:w);
                if (lo>=10 && hi<=200 && hi/lo<=6.0) c++
            } END{print c+0}' /tmp/smoke_out.csv)
            label="comp"
            ;;
    esac

    if [ "$found" -lt "$floor" ]; then
        echo "  FAIL $class ($label): $found / $n  (floor $floor = 2/3)"
        # show a few detected rows so a regression is debuggable
        awk -F, 'NR>1 {print "    " $0}' /tmp/smoke_out.csv | head -5
        fail=1
        continue
    fi
    echo "  OK   $class ($label): $found / $n"

    # ---- Per-frame regression baseline comparison (comp classes only) -------
    # found + method must match exactly; cx/cy/w/h/angle within +/- 0.01;
    # quality within +/- 0.005. Any drift fails the gate and prints the offending
    # row so the operator can decide if it's an intended change (regen the
    # baseline) or a regression (debug the detector). Skipped silently if no
    # baseline file is present yet, or for the fid class (different rig).
    [ -f "$BASELINE" ] || continue
    case "$label" in fid) continue ;; esac
    if ! awk -F, -v cls="$class" -v base="$BASELINE" -v tol=0.01 -v qtol=0.005 '
        function diff(actual, expected, lbl, file,    d) {
            d = actual - expected
            if (d < 0) d = -d
            if (d > (lbl == "quality" ? qtol : tol)) {
                printf "  %s  %-7s  %s != %s  (drift %.4f)\n", file, lbl, actual, expected, d
                bad = 1
            }
        }
        BEGIN {
            while ((getline line < base) > 0) {
                if (line ~ /^#/ || line == "") continue
                split(line, b, ",")
                if (b[1] == cls) {
                    k = b[2]
                    bfound[k] = b[3]; bcx[k] = b[4] + 0; bcy[k] = b[5] + 0
                    bw[k] = b[6] + 0; bh[k] = b[7] + 0; ba[k] = b[8] + 0
                    bq[k] = b[9] + 0; bm[k] = b[10]
                    have[k] = 1
                }
            }
            close(base)
        }
        NR > 1 {
            np = split($1, p, "/"); f = p[np]
            if (!(f in have)) next  # no baseline for this file -> not gated
            if ($2 != bfound[f]) { printf "  %s  found    %s != %s\n", f, $2, bfound[f]; bad = 1; next }
            if ($9 != bm[f])     { printf "  %s  method   %s != %s\n", f, $9, bm[f]; bad = 1 }
            diff($3, bcx[f], "cx",      f)
            diff($4, bcy[f], "cy",      f)
            diff($5, bw[f],  "w",       f)
            diff($6, bh[f],  "h",       f)
            diff($7, ba[f],  "angle",   f)
            diff($8, bq[f],  "quality", f)
        }
        END { exit bad + 0 }
    ' /tmp/smoke_out.csv; then
        echo "  DRIFT $class -- regression baseline mismatch:"
        # awk wrote the drift lines to stdout above; if the operator wants the
        # full csv they can re-run mvision-tune. Suggest the regen path.
        echo "    (re-pin via tools/regen_baselines.sh if the change is intentional)"
        fail=1
    fi
done

if [ "$fail" -ne 0 ]; then
    echo
    echo "SMOKE FAIL"
    exit 1
fi
echo
echo "SMOKE OK"
