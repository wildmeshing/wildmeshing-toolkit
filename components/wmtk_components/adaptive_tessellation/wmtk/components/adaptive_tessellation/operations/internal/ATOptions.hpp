#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {

struct ATOptions
{
public:
    bool planar;
    int64_t passes;
    std::string input;
    std::string uv_output;
    std::string xyz_output;
    double target_edge_length;
    double barrier_weight;
    double barrier_triangle_area;
    double quadrature_weight;
    double amips_weight;
    bool intermediate_output;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    ATOptions,
    planar,
    passes,
    input,
    uv_output,
    xyz_output,
    target_edge_length,
    barrier_weight,
    barrier_triangle_area,
    quadrature_weight,
    amips_weight,
    intermediate_output);

} // namespace wmtk::components