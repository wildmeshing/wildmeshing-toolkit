#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {
struct FusionOptions
{
    enum class Axis : int64_t { X = 0, Y = 1, Z = 2, All = 3 };
    Axis fusion_axis;

    std::string input;
    std::string name;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(FusionOptions, input, name, fusion_axis);


} // namespace wmtk::components