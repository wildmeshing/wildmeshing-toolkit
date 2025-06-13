#pragma once

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <spdlog/fmt/bundled/ranges.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

namespace wmtk {

bool has_user_overloaded_logger_level();
///
/// Retrieves the current logger.
///
/// @return     A const reference to WildmeshToolkit's logger object.
///
spdlog::logger& logger();

///
/// Retrieves the logger for the optimization.
///
/// @return     A const reference to WildmeshToolkit's  optimization logger object.
///
spdlog::logger& opt_logger();

///
/// Setup a logger object to be used by WildmeshToolkit. Calling this function with other WildmeshToolkit function
/// is not thread-safe.
///
/// @param[in]  logger  New logger object to be used by WildmeshToolkit. Ownership is shared with WildmeshToolkit.
///
void set_logger(std::shared_ptr<spdlog::logger> logger);

///
/// Setup a logger object to be used by WildmeshToolkit optimization. Calling this function with other WildmeshToolkit function
/// is not thread-safe.
///
/// @param[in]  logger  New logger object to be used by WildmeshToolkit logger. Ownership is shared with WildmeshToolkit.
///
void set_opt_logger(std::shared_ptr<spdlog::logger> logger);

[[noreturn]] void log_and_throw_error(const std::string& msg);

template <typename... Args>
[[noreturn]] void log_and_throw_error(const std::string& msg, const Args&... args)
{
    log_and_throw_error(fmt::format(fmt::runtime(msg), args...));
}
} // namespace wmtk