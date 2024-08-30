#pragma once

#include <wmtk/components/utils/json_utils.hpp>

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct InputOptions
{
    std::filesystem::path file;
    bool ignore_z = false;
    std::vector<std::string> tetrahedron_attributes;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(InputOptions, file, ignore_z, tetrahedron_attributes);

} // namespace internal
} // namespace components
} // namespace wmtk