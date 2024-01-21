#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {


struct ToPtsOptions
{
    std::string input;
    std::string position;
    std::string name;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ToPtsOptions, input, position, name);

} // namespace wmtk::components
