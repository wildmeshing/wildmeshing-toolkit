#pragma once

#include <wmtk/components/base/json_utils.hpp>

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {

struct NonManifoldInputOptions
{
    std::string name;
    std::filesystem::path file;
    bool ignore_z;
    std::vector<std::string> tetrahedron_attributes;
    std::string non_manifold_vertex_label;
    std::string non_manifold_edge_label;
    int64_t non_manifold_tag_value;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    NonManifoldInputOptions,
    name,
    file,
    ignore_z,
    tetrahedron_attributes,
    non_manifold_vertex_label,
    non_manifold_edge_label,
    non_manifold_tag_value);

} // namespace components
} // namespace wmtk