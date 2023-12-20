#pragma once

#include <wmtk/TriMesh.hpp>
#include "internal/extract_subset_2d.hpp"
#include "internal/extract_subset_3d.hpp"
#include "internal/topology_separate_2d.hpp"
#include "internal/topology_separate_3d.hpp"

namespace wmtk {

namespace components {

// wmtk::TriMesh extract_subset(wmtk::TriMesh m, long dimension, std::vector<int>& tag_vec, bool
// pos);
std::unique_ptr<wmtk::Mesh>
extract_subset(wmtk::Mesh& m, long dimension, std::vector<int>& tag_vec, bool pos);
} // namespace components
} // namespace wmtk