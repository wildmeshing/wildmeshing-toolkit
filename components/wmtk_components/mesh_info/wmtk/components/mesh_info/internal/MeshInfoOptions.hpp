#pragma once

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct MeshInfoOptions
{
    std::string input;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MeshInfoOptions, input);

} // namespace internal
} // namespace components
} // namespace wmtk