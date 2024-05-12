#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components::internal {

struct SECAttributes
{
    nlohmann::json position;
    nlohmann::json other_positions;
    nlohmann::json inversion_position;
    bool update_other_positions;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    SECAttributes,
    position,
    other_positions,
    inversion_position,
    update_other_positions);

struct SECOptions
{
    std::string input;
    std::string output;
    SECAttributes attributes;
    nlohmann::json pass_through;
    int64_t iterations;
    double length_abs;
    double length_rel;
    bool lock_boundary;
    bool use_for_periodic;
    bool fix_uv_seam;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    SECOptions,
    input,
    output,
    attributes,
    pass_through,
    length_abs,
    length_rel,
    iterations,
    lock_boundary,
    use_for_periodic);

} // namespace wmtk::components::internal
