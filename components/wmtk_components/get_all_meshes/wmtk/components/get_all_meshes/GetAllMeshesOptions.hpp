#pragma once

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct GetAllMeshesOptions
{
    std::string input;
    std::string output;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(GetAllMeshesOptions, input, output);

} // namespace internal
} // namespace components
} // namespace wmtk