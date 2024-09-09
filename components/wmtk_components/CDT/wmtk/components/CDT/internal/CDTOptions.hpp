#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {
struct CDTOptions
{
    std::string input;
    std::string output;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CDTOptions, input, output);

} // namespace wmtk::components