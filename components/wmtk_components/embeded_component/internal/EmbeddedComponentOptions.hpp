#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct EmbeddedComponentOptions
{
    std::string type;
    std::string input;
    std::string output;
    double length_abs = -1;
    double length_rel = -1;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    EmbeddedComponentOptions,
    type,
    input,
    output,
    length_abs,
    length_rel);

} // namespace internal
} // namespace components
} // namespace wmtk