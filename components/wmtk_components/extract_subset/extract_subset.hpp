#pragma once

#include <wmtk/TriMesh.hpp>
#include "internal/extract_subset_2d.hpp"
#include "internal/extract_subset_3d.hpp"
#include "internal/topology_separate_2d.hpp"
#include "internal/topology_separate_3d.hpp"

namespace wmtk {

namespace components {

/*
This function provides a unified interface for extracting a subset of a mesh, 2d or 3d, with or
without preserving geometry.
*/
std::unique_ptr<wmtk::Mesh>
extract_subset(wmtk::Mesh& m, const std::vector<int>& tag_vec, bool pos);
} // namespace components
} // namespace wmtk