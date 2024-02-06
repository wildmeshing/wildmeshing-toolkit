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
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ToPtsOptions, input, position, name, add_box, box_scale);

} // namespace wmtk::components
