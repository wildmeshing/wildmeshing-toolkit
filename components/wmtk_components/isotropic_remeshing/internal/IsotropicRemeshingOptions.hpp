#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct IsotropicRemeshingOptions
{
    std::string type;
    std::string input;
    std::string output;
    double length_abs = -1;
    double length_rel = -1;
    long iterations = -1;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    IsotropicRemeshingOptions,
    type,
    input,
    output,
    length_abs,
    length_rel,
    iterations);

} // namespace internal
} // namespace components
} // namespace wmtk