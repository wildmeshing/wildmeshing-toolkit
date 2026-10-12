#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace wmtk {

/**
 * @brief While alive, periodically hands memory the program has freed back to the system.
 *
 * glibc returns freed memory to the system only from the top of a heap: memory freed below a
 * live allocation stays resident, reusable by later allocations but by nothing else. A run frees
 * a great deal of it in the middle of the heap -- the arrangement once the insertion is over,
 * then most of the mesh's per-vertex and per-cell allocations when the first collapse pass
 * shrinks the mesh five- to twenty-fold. On the tetwild gate (serial runs, kirby) that was a
 * quarter to a third of the peak resident memory (1017012: 572 MB peak with at most 403 MB ever
 * allocated), and for most of a run more memory sat free than in use. malloc_trim(0) hands the free
 * pages of every arena back. The alternatives measured worse: a fixed mmap threshold recovered half
 * to three quarters as much, fewer arenas raised the peak, and so did mimalloc and tbbmalloc.
 *
 * It has to run while the work does, not between phases: the free memory builds up inside the
 * insertion and inside the passes, and trimming at every pass boundary instead both missed most
 * of it and cost up to a third of the run time in serial trims. From a background thread, every
 * half second, it measured no cost in run time.
 *
 * Process-wide, so it belongs to an executable, not to the library: construct one in main().
 * A no-op outside glibc.
 */
class HeapTrimmer
{
public:
    explicit HeapTrimmer(std::chrono::milliseconds period = std::chrono::milliseconds(500));
    ~HeapTrimmer();

    HeapTrimmer(const HeapTrimmer&) = delete;
    HeapTrimmer& operator=(const HeapTrimmer&) = delete;

private:
    std::chrono::milliseconds m_period;
    std::mutex m_mutex;
    std::condition_variable m_wake;
    bool m_stop = false;
    std::thread m_thread;
};

} // namespace wmtk
