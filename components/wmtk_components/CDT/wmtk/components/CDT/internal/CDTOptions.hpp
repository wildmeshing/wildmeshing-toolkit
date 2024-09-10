#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {
struct CDTOptions
{
    std::string input;
    std::string output;
    bool inner_only;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CDTOptions, input, output, inner_only);

} // namespace wmtk::components