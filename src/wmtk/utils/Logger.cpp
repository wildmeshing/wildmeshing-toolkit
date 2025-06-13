#include <wmtk/utils/Logger.hpp>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <sstream>

namespace wmtk {
bool has_user_overloaded_logger_level()
{
    const char* val = std::getenv("WMTK_LOGGER_LEVEL");
    if (val == nullptr) {
        return false;
    }
    std::string env_val = val;
    return !env_val.empty();
}

namespace {
inline void load_env_levels(spdlog::logger& logger)
{
    const char* val = std::getenv("WMTK_LOGGER_LEVEL");
    if (val == nullptr) {
        return;
    }
    std::string env_val = val;
    if (!env_val.empty()) {
        auto level = spdlog::level::from_str(env_val);
        if (level == spdlog::level::off) {
            if (env_val != "off") {
                // we cannot call our logger here because it could be off!
                spdlog::warn(
                    "Unknown logger level due to env value WMTK_LOGGER_LEVEL={}!",
                    env_val);
            }
        }
        logger.set_level(level);
        logger.flush_on(level);
    }
}

// Custom logger instance defined by the user, if any
std::shared_ptr<spdlog::logger>& get_shared_logger()
{
    static std::shared_ptr<spdlog::logger> logger;
    return logger;
}

// Custom logger instance defined by the user, if any
std::shared_ptr<spdlog::logger>& get_shared_opt_logger()
{
    static std::shared_ptr<spdlog::logger> logger;
    return logger;
}

} // namespace

// Retrieve current logger
spdlog::logger& logger()
{
    if (auto l = get_shared_logger(); bool(l)) {
        return *l;
    } else {
        // When using factory methods provided by spdlog (_st and _mt functions),
        // names must be unique, since the logger is registered globally.
        // Otherwise, you will need to create the logger manually. See
        // https://github.com/gabime/spdlog/wiki/2.-Creating-loggers
        static auto default_logger = spdlog::stdout_color_mt("wmtk");
        load_env_levels(*default_logger);
        set_logger(default_logger);
        return *default_logger;
    }
}

// Retrieve current logger
spdlog::logger& opt_logger()
{
    if (get_shared_opt_logger()) {
        return *get_shared_opt_logger();
    } else {
        // When using factory methods provided by spdlog (_st and _mt functions),
        // names must be unique, since the logger is registered globally.
        // Otherwise, you will need to create the logger manually. See
        // https://github.com/gabime/spdlog/wiki/2.-Creating-loggers
        static auto default_logger = spdlog::stdout_color_mt("wmtk-opt");
        return *default_logger;
    }
}

// Use a custom logger
void set_logger(std::shared_ptr<spdlog::logger> x)
{
    get_shared_logger() = std::move(x);
}

// Use a custom logger
void set_opt_logger(std::shared_ptr<spdlog::logger> x)
{
    get_shared_opt_logger() = std::move(x);
}

void log_and_throw_error(const std::string& msg)
{
    logger().error(msg);
    throw std::runtime_error(msg);
}

} // namespace wmtk