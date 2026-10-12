#include "HeapTrimmer.hpp"

// Any C library header defines __GLIBC__ under glibc; the compiler does not predefine it.
#include <cstdlib>

#if defined(__GLIBC__)
#include <malloc.h>
#endif

namespace wmtk {

HeapTrimmer::HeapTrimmer(std::chrono::milliseconds period)
    : m_period(period)
{
#if defined(__GLIBC__)
    m_thread = std::thread([this] {
        std::unique_lock<std::mutex> lock(m_mutex);
        while (!m_wake.wait_for(lock, m_period, [this] { return m_stop; })) {
            lock.unlock();
            malloc_trim(0);
            lock.lock();
        }
    });
#endif
}

HeapTrimmer::~HeapTrimmer()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stop = true;
    }
    m_wake.notify_one();
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

} // namespace wmtk
