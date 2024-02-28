#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components::internal {

struct IsotropicRemeshingAttributes
{
    nlohmann::json position;
    nlohmann::json inversion_position;
    nlohmann::json other_positions;
    bool update_other_positions;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    IsotropicRemeshingAttributes,
    position,
    inversion_position,
    other_positions,
    update_other_positions);

struct IsotropicRemeshingOptions
{
    std::string input;
    std::string output;
    double envelope_size_rel;
    IsotropicRemeshingAttributes attributes;
    nlohmann::json pass_through;
    int64_t iterations;
    double length_abs;
    double length_rel;
    bool lock_boundary;
    bool use_for_periodic;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    IsotropicRemeshingOptions,
    input,
    output,
    envelope_size_rel,
    attributes,
    pass_through,
    length_abs,
    length_rel,
    iterations,
    lock_boundary,
    use_for_periodic);

} // namespace wmtk::components::internal
