# tests/corpus — public smoke-test images

Representative up-vision frames for the offline `tests/smoke_comp.sh`
regression. One subdirectory per part class; ≤3 frames per class so the corpus
stays small (~2 MB total).

All images are 640×480 8-bit BGR PNGs captured by `MVision.dll` on a Gen-2
QiHe TVM802B, written to `C:\mvision_capture\comp_NNNN.png` while the `comp`
and `frames` trigger files were present (see `src/capture.cpp`).

| Class | What's in it | Why it's a smoke test |
|---|---|---|
| `chip_0603/` | 0603/1608 chip caps + resistors (bright metallic terminals over a dim ceramic body). | Catches regressions in the symmetry-primary path on the most common SMT body. |
| `led_ws2816/` | WS2816C-2121 addressable LEDs (square ~2.1×2.1 mm, single bright body with halo). | Locks-down LED center stability vs the halo gradient — past regressions wandered cx by 5+ px when the symmetry edge picker was loosened. |
| `sot23/` | SOT-23-3 (2+1 pin layout) and SOT-23-6 (3+3 pin layout) — the body is invisible between bright pin tips. | Exercises the union-of-contours fallback that spans the full body box from individual pin contours. |
| `fiducial/` | Down-vision `frame_NNNN.png` from a real placement, used by `CheckMark` / `CheckMark2` / `CheckTemplate`. One frame is deliberately interlace-combed (head was still moving when the down-cam captured) so a future fiducial smoke covers the field-aware deinterlace branch as well as the settled woven-frame branch. Currently SKIPPED by `smoke_comp.sh` (the component smoke uses `detect_component` only). Kept here for the future rig. |

The smoke pass criterion is **≥2/3 of frames per class yield a
plausible-aspect detection** (`min(w,h) >= 10`, `max(w,h) <= 200`, aspect ≤
6:1). A detector change that drops any class below that floor fails the smoke
and shouldn't ship.

Frames are real, anonymous, no IP risk — captured on our own machine, no part
markings visible, no schematic correlation.
