#pragma once

#include <cfenv>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

namespace wmtk::threading {

namespace detail {

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
 * SEMANTICS ARE THOSE OF A THREAD PER TASK. A task submitted while no worker is idle gets a new
 * worker; it never waits in a queue behind tasks that are still running. So a task may block on
 * another task of the same or of another group, and a task may run a task_group of its own,
 * exactly as before -- a fixed-size pool would deadlock on either. The pool only ever grows to
 * the largest number of tasks that were outstanding at once, which for every caller in this
 * repository is NUM_THREADS.
 *
 * The workers are never joined: they block on the condition variable for the life of the
 * process, and the pool is deliberately leaked so that nothing in it is destroyed while a
 * detached worker could still touch it.
 */
class worker_pool
{
public:
    static worker_pool& instance()
    {
        static worker_pool* pool = new worker_pool(); // leaked on purpose, see above
        return *pool;
    }

    /// Run @p job on a worker. Throws (and queues nothing) if a needed worker cannot be started.
    void submit(std::function<void()> job)
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            // Every queued job must have an idle worker of its own: a worker that is busy may be
            // running a task that waits for this one. Waking an idle worker that then finds the
            // queue already drained by a worker finishing early is harmless; it waits again.
            if (m_jobs.size() < m_idle) {
                m_jobs.push_back(std::move(job));
                m_cv.notify_one();
                return;
            }
        }
        // No idle worker is free for this job. Start one BEFORE queueing the job, so that a
        // failure to start it (std::system_error) leaves nothing behind for wait() to hang on.
        // If an idle worker turned up in the meantime, the new one just waits for the next job.
        std::thread([this] { work(); }).detach();
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_jobs.push_back(std::move(job));
        }
        m_cv.notify_one();
    }

private:
    worker_pool() = default;

    void work()
    {
        for (;;) {
            std::function<void()> job;
            {
                std::unique_lock<std::mutex> lock(m_mutex);
                ++m_idle;
                m_cv.wait(lock, [this] { return !m_jobs.empty(); });
                --m_idle;
                job = std::move(m_jobs.front());
                m_jobs.pop_front();
            }
            // A fresh std::thread starts in round-to-nearest. A reused one starts in whatever
            // the previous task left behind, and the exact predicates switch the rounding mode
            // around their interval filters, so an exception in the middle of one would leak an
            // upward mode into every later task on this thread. Restore the fresh-thread state.
            std::fesetround(FE_TONEAREST);
            job();
        }
    }

    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::deque<std::function<void()>> m_jobs;
    size_t m_idle = 0; ///< workers blocked in work(), waiting for a job
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
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            ++m_pending;
        }
        try {
            submit(std::forward<F>(f));
        } catch (...) {
            // Nothing was queued (see worker_pool::submit), so nothing will decrement this.
            std::lock_guard<std::mutex> lock(m_mutex);
            --m_pending;
            throw;
        }
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
    template <typename F>
    void submit(F&& f)
    {
        detail::worker_pool::instance().submit([this, f = std::forward<F>(f)]() mutable {
            {
                // Destroy the task's own state before reporting it finished: once it is
                // reported, wait() may return and the caller may tear down whatever the task
                // captured by reference.
                auto task = std::move(f);
                try {
                    task();
                } catch (...) {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    if (!m_eptr) {
                        m_eptr = std::current_exception();
                    }
                }
            }
            // Notify while holding the lock, so wait() cannot return -- and the group cannot be
            // destroyed -- until this thread has let go of the group's mutex.
            std::lock_guard<std::mutex> lock(m_mutex);
            if (--m_pending == 0) {
                m_done.notify_all();
            }
        });
    }
};

} // namespace wmtk::threading
