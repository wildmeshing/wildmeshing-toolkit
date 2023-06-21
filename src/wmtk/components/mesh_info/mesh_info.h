#pragma once

#include <map>
#include <nlohmann/json.hpp>
#include "internal/MeshInfoOptions.h"

namespace wmtk {
namespace components {
inline void mesh_info(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    MeshInfoOptions options = j.get<MeshInfoOptions>();

    std::filesystem::path& file = files[options.input];
}
} // namespace components
} // namespace wmtk