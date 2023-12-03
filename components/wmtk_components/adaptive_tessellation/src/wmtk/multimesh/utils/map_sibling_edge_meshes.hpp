#pragma once
#include <set>
#include <wmtk/Mesh.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/Attribute.hpp>

namespace wmtk::components::adaptive_tessellation::multimesh::utils {
//// this is a costly implementation. It shoulnd't be used in production
std::map<EdgeMesh*, EdgeMesh*> map_sibling_edge_meshes(const TriMesh& position_mesh);

std::map<EdgeMesh*, EdgeMesh*> map_sibling_edge_meshes(
    const std::vector<std::shared_ptr<EdgeMesh>> edge_meshes);
} // namespace wmtk::components::adaptive_tessellation::multimesh::utils
