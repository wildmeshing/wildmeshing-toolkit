#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct MeshInfoOptions
{
    std::string type;
    std::string input;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MeshInfoOptions, type, input);

} // namespace internal
} // namespace components
} // namespace wmtk