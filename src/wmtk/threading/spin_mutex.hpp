#pragma once

#include <atomic>
#include <thread>

namespace wmtk::threading {
// ---------------------------------------------------------------------------
// spin_mutex: replaces tbb::spin_mutex.
// Movable/copyable (resets to unlocked) so it can live inside elements of a
// std::vector that is grown single-threaded (e.g. the per-vertex mutex array).
// ---------------------------------------------------------------------------
class spin_mutex
{
    std::atomic<bool> m_locked{false};

public:
    spin_mutex() = default;
    spin_mutex(const spin_mutex&) noexcept {}
    spin_mutex(spin_mutex&&) noexcept {}
    spin_mutex& operator=(const spin_mutex&) noexcept { return *this; }
    spin_mutex& operator=(spin_mutex&&) noexcept { return *this; }

    void lock()
    {
        bool expected = false;
        while (!m_locked.compare_exchange_weak(
            expected,
            true,
            std::memory_order_acquire,
            std::memory_order_relaxed)) {
            expected = false;
            std::this_thread::yield();
        }
    }

    bool try_lock()
    {
        bool expected = false;
        return m_locked.compare_exchange_strong(expected, true, std::memory_order_acquire);
    }

    void unlock() { m_locked.store(false, std::memory_order_release); }
};

}