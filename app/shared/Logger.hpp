#pragma once

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/fmt/bundled/ranges.h>
#include <wmtk/utils/EnableWarnings.hpp>

#include <ostream>
// clang-format on

namespace apps {

struct Logger
{
    static std::shared_ptr<spdlog::async_logger> logger_;

    // By default, write to stdout, but don't write to any file
    static void init(bool use_cout = true, const std::string& filename = "", bool truncate = true);
    static void init(std::ostream& os);
    static void init(std::vector<spdlog::sink_ptr>& sinks);
};

// Retrieve current logger, or create one if not available
inline spdlog::async_logger& logger()
{
    if (!Logger::logger_) {
        Logger::init();
    }
    return *Logger::logger_;
}

} // namespace apps
