#pragma once

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct GetAllMeshesOptions
{
    std::string input;
    std::string name;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(GetAllMeshesOptions, input, name);

} // namespace internal
} // namespace components
} // namespace wmtk