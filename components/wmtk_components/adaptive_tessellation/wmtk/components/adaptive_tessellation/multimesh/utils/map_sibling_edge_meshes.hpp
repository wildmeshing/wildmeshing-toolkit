#pragma once
#include <set>
#include <wmtk/Mesh.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/Attribute.hpp>

namespace wmtk::components::multimesh::utils {
//// this is a costly implementation. It shoulnd't be used in production
std::map<Mesh*, Mesh*> map_sibling_edge_meshes(const Mesh& position_mesh);

std::map<Mesh*, Mesh*> map_sibling_edge_meshes(
    const std::vector<std::shared_ptr<Mesh>> edge_meshes);
} // namespace wmtk::components::multimesh::utils
