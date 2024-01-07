#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components::internal {

struct IsotropicRemeshingAttributes
{
    std::string position;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(IsotropicRemeshingAttributes, position);

struct IsotropicRemeshingOptions
{
    std::string input;
    std::string output;
    IsotropicRemeshingAttributes attributes;
    std::vector<std::string> pass_through;
    int64_t iterations;
    double length_abs;
    double length_rel;
    bool lock_boundary;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    IsotropicRemeshingOptions,
    input,
    output,
    attributes,
    pass_through,
    length_abs,
    length_rel,
    iterations,
    lock_boundary);

} // namespace wmtk::components::internal