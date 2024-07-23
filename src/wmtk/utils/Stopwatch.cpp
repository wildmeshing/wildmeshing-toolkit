#include "Stopwatch.hpp"


#include <spdlog/common.h>

namespace wmtk::utils {


StopWatch::StopWatch(const std::string& name)
    : StopWatch(name, spdlog::level::info)
{}

StopWatch::StopWatch(const std::string& name, const spdlog::level::level_enum log_level)
    : m_name(name)
    , m_log_level(log_level)
{
    start();
}

StopWatch::~StopWatch()
{
    if (m_is_running) {
        stop();
        log_msg();
    }
}

void StopWatch::start()
{
    m_is_running = true;
    m_start = std::chrono::high_resolution_clock::now();
}

void StopWatch::stop()
{
    if (!m_is_running) {
        return;
    }
    m_is_running = false;
    m_stop = std::chrono::high_resolution_clock::now();
}

inline void StopWatch::log_msg()
{
    int64_t h = getElapsedTime<std::chrono::hours>();
    int64_t m = getElapsedTime<std::chrono::minutes>() % 60;
    int64_t s = getElapsedTime<std::chrono::seconds>() % 60;
    int64_t ms = getElapsedTime<std::chrono::milliseconds>() % 1000;

    logger().log(
        m_log_level,
        "[runtime h:m:s.ms] {}: {:02d}:{:02d}:{:02d}.{:03d}",
        m_name,
        h,
        m,
        s,
        ms);
}

} // namespace wmtk::utils