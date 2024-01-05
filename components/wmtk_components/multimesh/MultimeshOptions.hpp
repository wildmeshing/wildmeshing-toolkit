#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {

struct MultimeshOptions
{
public:
    std::string type;
    std::string parent;
    std::string child;
    std::string output;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultimeshOptions, type, parent, child, output);

} // namespace wmtk::components
