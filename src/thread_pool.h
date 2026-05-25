#pragma once
// Minimal persistent fork-join pool for the detector's embarrassingly-parallel
// inner work (the circular-symmetry coarse grid: thousands of independent score
// evaluations per frame). Windows-free (std threads only), so it links into the
// portable detector core and the off-target tests.
//
// WHY a custom pool (not cv::parallel_for_ / OpenMP): the OpenCV build routes
// parallel_for_ through MSVC's ConcRT pool whose scheduler overhead dwarfs this
// fine-grained work, and MSVC /openmp pulls in a vcomp runtime DLL (breaks the
// self-contained DLL). A small std::thread pool, created ONCE and reused, has
// ~microsecond per-fork overhead -- negligible against ~100 ms of detection.

#include <functional>

namespace vis {

// Run body(i) for every i in [0, n), distributed across a small persistent pool
// (capped at 4 participating cores incl. the caller; created lazily on first use).
// The calling thread participates, then blocks until all i are done. Falls back to
// a plain serial loop on a single-core host or for n <= 1.
//
// body MUST be safe to call concurrently for distinct i (it is invoked from
// multiple threads at once); give each i its own output slot and reduce afterwards.
void parallel_for(int n, const std::function<void(int)>& body);

}  // namespace vis
