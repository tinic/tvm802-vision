# tests/corpus — public smoke-test images

A small set of representative up-vision frames used by the offline `tests/smoke_comp.sh` regression test. The corpus is organized with one subdirectory per part class, and each class holds at most three frames so that the corpus stays small (around 2 MB total).

All images are 640×480 8-bit BGR PNGs captured by `MVision.dll` on a Gen-2 QiHe TVM802B. They were written to `C:\mvision_capture\comp_NNNN.png` while the `comp` and `frames` trigger files were present (see `src/capture.cpp`).

| Class | What's in it | Why it's a smoke test |
|---|---|---|
| `chip_0603/` | 0603 / 1608 chip caps and resistors. Bright metallic terminals over a dim ceramic body. | Catches regressions in the symmetry-primary path on the most common SMT body. |
| `led_ws2816/` | WS2816C-2121 addressable LEDs. Roughly square at 2.1 × 2.1 mm, with a single bright body surrounded by a halo. | Pins down LED-center stability against the halo gradient. Past regressions wandered cx by 5 pixels or more when the symmetry edge picker was loosened. |
| `sot23/` | SOT-23-3 (2 + 1 pin layout) and SOT-23-6 (3 + 3 pin layout). The body is essentially invisible between the bright pin tips. | Exercises the union-of-contours fallback, which spans the full body box from the individual pin contours. |
| `fiducial/` | Down-vision `frame_NNNN.png` frames from a real placement, exercised by `detect_circular_symmetry` (the `CheckMark2` Round path). One frame is deliberately interlace-combed, because the head was still moving when the down camera captured it; this lets the smoke test cover the field-aware deinterlace branch as well as the settled woven-frame branch. |

The pass criterion is **at least 2 of 3 frames per class**, rounded up:

- For `comp` subdirectories (chips, LEDs, SOT-23, and so on), a frame passes when it produces a plausible detection: `min(w, h) >= 10`, `max(w, h) <= 200`, and aspect ratio no greater than 6:1.
- For `fid` subdirectories, a frame passes when it produces a lock with `radius >= 5 px` and `quality > 1.0`. This catches both "missed entirely" and "locked onto noise" regressions.

A detector change that drops any class below this floor fails the smoke test and should not ship.

The frames are real but anonymous, so there is no IP risk. They were captured on our own machine, no part markings are visible, and there is no schematic correlation.
