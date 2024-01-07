#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {

struct MultimeshOptions
{
public:
    std::string parent;
    std::string child;
    std::string name;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultimeshOptions, parent, child, name);

} // namespace wmtk::components
