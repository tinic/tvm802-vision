#include "thread_pool.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

namespace vis {
namespace {

constexpr int kMaxParticipants = 4;  // cap total cores used (incl. the calling thread)

// Persistent fork-join pool. Workers sleep on a condition variable between jobs;
// run() hands out tasks via an atomic counter (work-stealing, so uneven per-task
// cost self-balances) and the caller participates. Bit-identical to a serial loop:
// distinct i run on different threads but each writes its own slot, so the caller's
// post-reduction order -- not the execution order -- decides the result.
class Pool {
public:
    explicit Pool(int participants)
        : nWorkers_(participants - 1) {
        workers_.reserve(static_cast<size_t>(nWorkers_));
        for (int i = 0; i < nWorkers_; ++i) {
            workers_.emplace_back([this] { worker_loop(); });
        }
    }
    // Never destroyed: the single instance is leaked (see pool()) so worker threads
    // are reclaimed by the OS at process exit; joining in a destructor could run under
    // the DLL loader lock at unload and deadlock. ~Pool is deleted to enforce that.
    Pool(const Pool&) = delete;
    Pool& operator=(const Pool&) = delete;
    Pool(Pool&&) = delete;
    Pool& operator=(Pool&&) = delete;
    ~Pool() = delete;

    void run(int n, const std::function<void(int)>& body) {
        {
            const std::lock_guard<std::mutex> lk(mtx_);
            body_ = &body;
            n_ = n;
            next_.store(0, std::memory_order_relaxed);
            remaining_.store(nWorkers_ + 1, std::memory_order_relaxed);  // workers + caller
            ++gen_;
        }
        wake_.notify_all();
        drain();  // the caller is a participant
        std::unique_lock<std::mutex> lk(doneMtx_);
        done_.wait(lk, [this] { return remaining_.load(std::memory_order_acquire) == 0; });
    }

private:
    // Pull tasks until the job is exhausted, then mark this participant finished.
    void drain() {
        const std::function<void(int)>& body = *body_;
        const int n = n_;
        for (int t = next_.fetch_add(1, std::memory_order_relaxed); t < n;
             t = next_.fetch_add(1, std::memory_order_relaxed)) {
            body(t);
        }
        if (remaining_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
            const std::lock_guard<std::mutex> lk(doneMtx_);
            done_.notify_one();
        }
    }

    void worker_loop() {
        unsigned seen = 0;
        for (;;) {
            std::unique_lock<std::mutex> lk(mtx_);
            wake_.wait(lk, [this, &seen] { return gen_ != seen; });
            seen = gen_;
            lk.unlock();  // body_/n_/next_ were published under mtx_ by run()
            drain();
        }
    }

    std::mutex mtx_;
    std::condition_variable wake_;
    std::mutex doneMtx_;
    std::condition_variable done_;
    std::vector<std::thread> workers_;
    const std::function<void(int)>* body_ = nullptr;
    int n_ = 0;
    int nWorkers_ = 0;
    unsigned gen_ = 0;
    std::atomic<int> next_{0};
    std::atomic<int> remaining_{0};
};

// Lazily-built, intentionally-leaked singleton (no join at DLL unload). nullptr when
// the host has a single core -> parallel_for runs serially.
Pool* make_pool() {
    const unsigned hc = std::thread::hardware_concurrency();
    const int participants = std::min((hc == 0) ? 1 : static_cast<int>(hc), kMaxParticipants);
    if (participants <= 1) {
        return nullptr;
    }
    return new Pool(participants);  // NOLINT(cppcoreguidelines-owning-memory) -- leaked by design
}

Pool* pool() {
    static Pool* p = make_pool();  // NOLINT(clang-analyzer-cplusplus.NewDeleteLeaks) -- see make_pool
    return p;
}

}  // namespace

void parallel_for(int n, const std::function<void(int)>& body) {
    if (n <= 0) {
        return;
    }
    Pool* p = pool();
    if (p == nullptr || n == 1) {  // single core, or nothing to split
        for (int i = 0; i < n; ++i) {
            body(i);
        }
        return;
    }
    p->run(n, body);
}

}  // namespace vis
