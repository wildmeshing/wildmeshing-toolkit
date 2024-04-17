#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {

struct ATOptions
{
public:
    std::string parent, child;
    std::string uv_output;
    std::string xyz_output;
    double target_distance;
    bool max_distance;
    bool analytical_function = false;
    bool terrain_texture = false;
    float min_height = 0.f;
    float max_height = 1.f;
    std::string position_path;
    std::string normal_path;
    std::string height_path;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    ATOptions,

    parent,
    child,
    uv_output,
    xyz_output,
    target_distance,
    max_distance,
    analytical_function,
    terrain_texture,
    min_height,
    max_height,
    position_path,
    normal_path,
    height_path);

} // namespace wmtk::components