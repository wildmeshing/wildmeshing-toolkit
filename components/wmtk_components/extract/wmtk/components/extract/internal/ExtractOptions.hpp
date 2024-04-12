#pragma once

#include <wmtk/components/base/json_utils.hpp>

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct ExtractOptions
{
    std::string name;
    std::filesystem::path file;
    bool ignore_z;
    std::vector<std::string> tetrahedron_attributes;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ExtractOptions, name, file, ignore_z, tetrahedron_attributes);

} // namespace internal
} // namespace components
} // namespace wmtk