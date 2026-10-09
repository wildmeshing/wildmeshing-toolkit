#pragma once

#include <atomic>
#include <cfenv>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <type_traits>
#include <utility>

#if defined(__unix__) || defined(__APPLE__)
#include <pthread.h>
#define WMTK_WORKER_POOL_RESET_ON_FORK 1
#endif

namespace wmtk::threading {

namespace detail {

/// What the pool queues: one task, type-erased. Unlike std::function it does not require the task
/// to be copyable -- std::thread, which ran task_group's tasks before the pool, did not either.
struct job
{
    virtual ~job() = default;
    /// Runs the task and destroys its state. Never throws.
    virtual void run() noexcept = 0;
    /// Reports the task finished. Called after run(), once the worker that ran it is idle again.
    /// Never throws.
    virtual void finish() noexcept = 0;
};

/**
 * @brief The process-wide set of threads that task_group runs its tasks on.
 *
 * Every pass of the optimizers runs as a task_group of NUM_THREADS tasks, and task_group used to
 * spawn a fresh std::thread per task and join it at the barrier. Thread creation itself is cheap;
 * what it threw away was every thread_local the workers had built. Each pass started on threads
 * that had never run an operation, so each of them rebuilt its per-thread polysolve solver --
 * Solver::create validates its JSON parameters against the full spec, and at 16 threads on
 * Thingi10K 103197 that was 12% of all thread time -- and re-zeroed its ring-lock scratch, sized
 * to the vertex capacity. A pass of a few milliseconds paid that start-up again every time.
 *
 * The workers here are started once and kept, so a thread's caches survive from one pass to the
 * next exactly as they already did in serial mode, where one thread runs everything.
 *
 * SEMANTICS ARE THOSE OF A THREAD PER TASK. A task submitted while no worker is free for it gets a
 * new worker; it never waits in a queue behind tasks that are still running. So a task may block
 * on another task of the same or of another group, a task may run a task_group of its own, and
 * several threads may submit at once, exactly as before -- a fixed-size pool would deadlock on
 * the first two. Every queued job is owed an idle worker, or a worker being started for it, and
 * no other job can take that worker.
 *
 * A worker counts as idle again before its task is reported finished, so a caller that starts its
 * next group as soon as wait() returns finds every worker of the last one free. With one thread
 * submitting -- every caller in this repository -- the pool therefore never grows past the
 * largest number of tasks that were outstanding at once, which is NUM_THREADS.
 *
 * The workers are never joined: they block on the condition variable for the life of the
 * process, and the pool is deliberately leaked so that nothing in it is destroyed while a
 * detached worker could still touch it.
 *
 * A forked child (POSIX) inherits the pool's memory but none of its threads, and possibly its
 * mutex in a locked state. The child therefore abandons it and starts a fresh pool on its first
 * task -- as it would have started fresh threads before the pool existed.
 */
class worker_pool
{
public:
    static worker_pool& instance()
    {
#ifdef WMTK_WORKER_POOL_RESET_ON_FORK
        static const bool registered = (pthread_atfork(nullptr, nullptr, &abandon_in_child), true);
        (void)registered;
#endif
        auto& slot = current();
        worker_pool* p = slot.load(std::memory_order_acquire);
        if (p != nullptr) return *p;
        // Created lazily, so that a forked child can start over. Threads racing to create it
        // agree on one; the losers' pools have no workers yet and are simply deleted.
        auto* fresh = new worker_pool();
        if (slot.compare_exchange_strong(p, fresh, std::memory_order_acq_rel)) return *fresh;
        delete fresh;
        return *p;
    }

    /// Run @p job on a worker. Throws (and queues nothing) if a needed worker cannot be started.
    void submit(std::unique_ptr<job> j)
    {
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            // Every queued job must have an idle worker of its own: a worker that is busy may be
            // running a task that waits for this one. A worker being started is already owed to
            // the job that started it, so it does not count as free here: another thread taking
            // it in the meantime would leave that job with no worker. Waking an idle worker that
            // then finds the queue already drained by a worker finishing early is harmless; it
            // waits again.
            if (m_jobs.size() + m_spawning < m_idle) {
                m_jobs.push_back(std::move(j));
                lock.unlock();
                m_cv.notify_one();
                return;
            }
            ++m_spawning;
        }
        // No idle worker is free for this job. Start one BEFORE queueing the job, so that a
        // failure to start it (std::system_error) leaves nothing behind for wait() to hang on.
        // If an idle worker turned up in the meantime, one of the two just waits for the next job.
        try {
            std::thread([this] { work(); }).detach();
        } catch (...) {
            std::lock_guard<std::mutex> lock(m_mutex);
            --m_spawning;
            throw;
        }
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            ++m_workers;
            --m_spawning;
            m_jobs.push_back(std::move(j));
        }
        m_cv.notify_one();
    }

    /// Queue @p j on a worker that is idle right now, as submit() would, and return true; or, if
    /// every idle worker is already owed to a queued job, return false with @p j left untouched.
    /// It never starts a worker.
    bool try_submit(std::unique_ptr<job>& j)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        if (m_jobs.size() + m_spawning >= m_idle) return false;
        m_jobs.push_back(std::move(j));
        lock.unlock();
        m_cv.notify_one();
        return true;
    }

    /// How many workers the pool has started. For tests.
    size_t worker_count()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_workers;
    }

private:
    worker_pool() = default;

    static std::atomic<worker_pool*>& current()
    {
        static std::atomic<worker_pool*> pool{nullptr};
        return pool;
    }

#ifdef WMTK_WORKER_POOL_RESET_ON_FORK
    /// pthread_atfork child handler: leak the parent's pool -- its workers do not exist here and
    /// its mutex may be held by one of them -- and let instance() create a fresh one. Only resets
    /// a pointer, which is safe in a child of a multi-threaded process.
    static void abandon_in_child() { current().store(nullptr, std::memory_order_relaxed); }
#endif

    [[noreturn]] void work()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        ++m_idle;
        for (;;) {
            m_cv.wait(lock, [this] { return !m_jobs.empty(); });
            --m_idle;
            std::unique_ptr<job> j = std::move(m_jobs.front());
            m_jobs.pop_front();
            lock.unlock();
            // A task expects round-to-nearest, the mode the code that created its thread ran in.
            // A reused worker starts in whatever the previous task left behind, and the exact
            // predicates switch the rounding mode around their interval filters, so an exception
            // in the middle of one would leak an upward mode into every later task on this thread.
            std::fesetround(FE_TONEAREST);
            j->run();
            // Idle again BEFORE the task is reported finished: once it is, wait() may return and
            // its caller start the next group, which must find this worker free rather than start
            // another one. Until it waits again this worker does nothing that can block.
            lock.lock();
            ++m_idle;
            lock.unlock();
            j->finish();
            j.reset();
            lock.lock();
        }
    }

    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::deque<std::unique_ptr<job>> m_jobs;
    size_t m_idle = 0; ///< workers that are free for a job: waiting for one, or about to
    size_t m_spawning = 0; ///< workers being started, each owed to the job that started it
    size_t m_workers = 0; ///< workers started
};

} // namespace detail

// ---------------------------------------------------------------------------
// task_group: replaces tbb::task_group. run() hands the task to a worker of the persistent
// pool above (one worker per outstanding task, so the semantics are a thread per task);
// wait() blocks until every task run since the last wait has finished, then rethrows the first
// exception any of them threw.
// ---------------------------------------------------------------------------
class task_group
{
    std::mutex m_mutex;
    std::condition_variable m_done;
    size_t m_pending = 0;
    std::exception_ptr m_eptr;

public:
    task_group() = default;
    task_group(const task_group&) = delete;
    task_group& operator=(const task_group&) = delete;

    template <typename F>
    void run(F&& f)
    {
        // The task is copied or moved into its job here, on the calling thread: a task whose copy
        // or move throws makes run() throw -- as std::thread's constructor did -- before anything
        // is counted or queued.
        auto j = std::make_unique<task_job<std::decay_t<F>>>(*this, std::forward<F>(f));
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            ++m_pending;
        }
        try {
            detail::worker_pool::instance().submit(std::move(j));
        } catch (...) {
            // Nothing was queued (see worker_pool::submit), so nothing will decrement this.
            std::lock_guard<std::mutex> lock(m_mutex);
            --m_pending;
            throw;
        }
    }

    /**
     * @brief As run(), but only on a worker of the pool that is idle right now. Returns false,
     * having run nothing, if there is none; it never starts a thread.
     *
     * For optional help: work the caller would do alone anyway, split up only if threads are
     * sitting idle, where starting a thread for it would cost more than the split saves. Call
     * wait() either way. See SampleEnvelope::is_outside() for the use it was written for.
     */
    template <typename F>
    bool try_run(F&& f)
    {
        std::unique_ptr<detail::job> j =
            std::make_unique<task_job<std::decay_t<F>>>(*this, std::forward<F>(f));
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            ++m_pending;
        }
        if (detail::worker_pool::instance().try_submit(j)) return true;
        // Not queued, so nothing will report it finished.
        std::lock_guard<std::mutex> lock(m_mutex);
        --m_pending;
        return false;
    }

    void wait()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_done.wait(lock, [this] { return m_pending == 0; });
        if (m_eptr) {
            std::exception_ptr e = m_eptr;
            m_eptr = nullptr;
            lock.unlock();
            std::rethrow_exception(e);
        }
    }

    ~task_group()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_done.wait(lock, [this] { return m_pending == 0; });
    }

private:
    /// A task of this group, queued on the pool. Move-only tasks are fine.
    template <typename Fn>
    class task_job final : public detail::job
    {
        task_group& m_group;
        std::optional<Fn> m_fn;

    public:
        template <typename G>
        task_job(task_group& group, G&& fn)
            : m_group(group)
            , m_fn(std::in_place, std::forward<G>(fn))
        {}

        void run() noexcept override
        {
            try {
                (*m_fn)();
            } catch (...) {
                m_group.record(std::current_exception());
            }
            // Destroy the task's own state before it is reported finished: once it is, wait()
            // may return and the caller may tear down whatever the task captured by reference.
            m_fn.reset();
        }

        // After this the job touches nothing of the group's.
        void finish() noexcept override { m_group.finish_one(); }
    };

    void record(std::exception_ptr e)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_eptr) {
            m_eptr = std::move(e);
        }
    }

    void finish_one()
    {
        // Notify while holding the lock, so wait() cannot return -- and the group cannot be
        // destroyed -- until this thread has let go of the group's mutex.
        std::lock_guard<std::mutex> lock(m_mutex);
        if (--m_pending == 0) {
            m_done.notify_all();
        }
    }
};

} // namespace wmtk::threading
