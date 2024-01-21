#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {
struct TriInsOptions
{
    std::string input;
    std::string position;
    std::string name;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TriInsOptions, input, position, name);

} // namespace wmtk::components
