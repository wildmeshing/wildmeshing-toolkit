#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {


struct ToPtsOptions
{
    std::string input;
    std::string position;
    std::string name;
    bool add_box;
    double box_scale;
    bool add_grid;
    double grid_spacing;
    double min_dist;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    ToPtsOptions,
    input,
    position,
    name,
    add_box,
    box_scale,
    add_grid,
    grid_spacing,
    min_dist);

} // namespace wmtk::components
