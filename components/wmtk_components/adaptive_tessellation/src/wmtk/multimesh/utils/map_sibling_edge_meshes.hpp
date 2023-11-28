#pragma once
#include <set>
#include <wmtk/Mesh.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/Attribute.hpp>

namespace wmtk::components::adaptive_tessellation::multimesh::utils {
std::map<Mesh*, Mesh*> map_sibling_edge_meshes(const Mesh& position_mesh);
}