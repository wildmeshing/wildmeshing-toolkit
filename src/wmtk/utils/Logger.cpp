#include <wmtk/utils/Logger.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <sstream>

namespace wmtk {

namespace {

// Custom logger instance defined by the user, if any
std::shared_ptr<spdlog::logger>& get_shared_logger()
{
    static std::shared_ptr<spdlog::logger> logger;
    return logger;
}

} // namespace

// Retrieve current logger
spdlog::logger& logger()
{
    if (get_shared_logger()) {
        return *get_shared_logger();
    } else {
        // When using factory methods provided by spdlog (_st and _mt functions),
        // names must be unique, since the logger is registered globally.
        // Otherwise, you will need to create the logger manually. See
        // https://github.com/gabime/spdlog/wiki/2.-Creating-loggers
        static auto default_logger = spdlog::stdout_color_mt("wmtk");
        return *default_logger;
    }
}

// Use a custom logger
void set_logger(std::shared_ptr<spdlog::logger> x)
{
    get_shared_logger() = std::move(x);
}

} // namespace wmtk
