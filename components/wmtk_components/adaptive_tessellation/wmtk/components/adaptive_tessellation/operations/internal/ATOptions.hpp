#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {

struct ATOptions
{
public:
    int64_t passes;
    std::string parent, child;
    std::string uv_output;
    std::string xyz_output;
    double target_distance;
    double barrier_weight;
    double barrier_triangle_area;
    double quadrature_weight;
    double amips_weight;
    bool area_weighted_amips;
    bool intermediate_output;
    std::string position_path;
    std::string normal_path;
    std::string height_path;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    ATOptions,
    passes,
    parent,
    child,
    uv_output,
    xyz_output,
    target_distance,
    barrier_weight,
    barrier_triangle_area,
    quadrature_weight,
    amips_weight,
    area_weighted_amips,
    intermediate_output,
    position_path,
    normal_path,
    height_path);

} // namespace wmtk::components