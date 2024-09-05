#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {
struct FusionOptions
{
    bool fusion_X;
    bool fusion_Y;
    bool fusion_Z;

    std::string input;
    std::string name;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(FusionOptions, input, fusion_X, fusion_Y, fusion_Z, name);


} // namespace wmtk::components