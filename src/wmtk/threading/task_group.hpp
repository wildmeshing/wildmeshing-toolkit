#pragma once

#include <exception>
#include <mutex>
#include <thread>
#include <vector>

namespace wmtk::threading {

// ---------------------------------------------------------------------------
// task_group: replaces tbb::task_group. Each run() spawns a thread; wait() joins.
// ---------------------------------------------------------------------------
class task_group
{
    std::vector<std::thread> m_threads;
    std::exception_ptr m_eptr;
    std::mutex m_eptr_mutex;

public:
    task_group() = default;
    task_group(const task_group&) = delete;
    task_group& operator=(const task_group&) = delete;

    template <typename F>
    void run(F&& f)
    {
        m_threads.emplace_back([this, f = std::forward<F>(f)]() mutable {
            try {
                f();
            } catch (...) {
                std::lock_guard<std::mutex> lock(m_eptr_mutex);
                if (!m_eptr) {
                    m_eptr = std::current_exception();
                }
            }
        });
    }

    void wait()
    {
        for (auto& t : m_threads) {
            if (t.joinable()) {
                t.join();
            }
        }
        m_threads.clear();
        if (m_eptr) {
            std::exception_ptr e = m_eptr;
            m_eptr = nullptr;
            std::rethrow_exception(e);
        }
    }

    ~task_group()
    {
        for (auto& t : m_threads) {
            if (t.joinable()) {
                t.join();
            }
        }
    }
};

} // namespace wmtk::threading