#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {
struct CDTOptions
{
    std::string input;
    std::string output;
    bool inner_only;
    bool rational_output;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CDTOptions, input, output, inner_only, rational_output);

} // namespace wmtk::components