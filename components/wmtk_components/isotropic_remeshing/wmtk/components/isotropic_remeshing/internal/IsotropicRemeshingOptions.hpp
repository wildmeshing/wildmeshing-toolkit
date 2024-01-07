#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components::internal {

struct IsotropicRemeshingAttributes
{
    std::string position;
    nlohmann::json inversion_position;
    nlohmann::json other_positions;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    IsotropicRemeshingAttributes,
    position,
    inversion_position,
    other_positions);

struct IsotropicRemeshingOptions
{
    std::string input;
    std::string output;
    IsotropicRemeshingAttributes attributes;
    nlohmann::json pass_through;
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