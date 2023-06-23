#pragma once

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <spdlog/fmt/bundled/ranges.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

namespace wmtk {

///
/// Retrieves the current logger.
///
/// @return     A const reference to WildmeshToolkit's logger object.
///
spdlog::logger& logger();

///
/// Setup a logger object to be used by WildmeshToolkit. Calling this function with other WildmeshToolkit function
/// is not thread-safe.
///
/// @param[in]  logger  New logger object to be used by WildmeshToolkit. Ownership is shared with WildmeshToolkit.
///
void set_logger(std::shared_ptr<spdlog::logger> logger);

} // namespace wmtk
