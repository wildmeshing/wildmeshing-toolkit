#pragma once

#include <spdlog/common.h>
#include <chrono>
#include <type_traits>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::utils {

class StopWatch
{
public:
    StopWatch(const std::string& name);
    StopWatch(const std::string& name, const spdlog::level::level_enum log_level);

    virtual ~StopWatch();

    void start();
    void stop();

    template <typename T = std::chrono::seconds>
    int64_t getElapsedTime();

    void log_msg();


private:
    std::string m_name;
    spdlog::level::level_enum m_log_level = spdlog::level::info;
    std::chrono::high_resolution_clock::time_point m_start;
    std::chrono::high_resolution_clock::time_point m_stop;

    bool m_is_running = false;
};

template <typename T>
inline int64_t StopWatch::getElapsedTime()
{
    auto duration = std::chrono::duration_cast<T>(m_stop - m_start);
    return duration.count();
}

} // namespace wmtk::utils