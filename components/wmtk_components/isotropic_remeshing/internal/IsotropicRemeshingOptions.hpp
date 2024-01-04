#pragma once

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
    int64_t iterations = -1;
    bool lock_boundary = true;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    IsotropicRemeshingOptions,
    type,
    input,
    output,
    length_abs,
    length_rel,
    iterations,
    lock_boundary);

} // namespace internal
} // namespace components
} // namespace wmtk