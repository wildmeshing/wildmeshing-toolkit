#pragma once

#include "internal/generate_submesh.hpp"
#include "internal/new_topology_separate.hpp"
namespace wmtk {

namespace components {

/*
This function provides a unified interface for extracting a subset of a mesh, 2d or 3d, with or
without preserving geometry.
*/
std::unique_ptr<wmtk::Mesh>
extract_subset(wmtk::Mesh& m, const std::vector<int>& tag_vec, bool pos);
// wmtk::Mesh& extract_subset(wmtk::Mesh& m, const std::vector<int>& tag_vec, bool pos);
} // namespace components
} // namespace wmtk