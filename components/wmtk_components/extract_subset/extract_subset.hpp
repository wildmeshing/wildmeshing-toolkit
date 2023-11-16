#pragma once

#include <wmtk/TriMesh.hpp>
#include "internal/extract_subset_2d.hpp"
#include "internal/topology_separate_2d.hpp"

namespace wmtk {

namespace components {

wmtk::TriMesh extract_subset(wmtk::TriMesh m, long dimension, std::vector<int>& tag_vec, bool pos);
} // namespace components
} // namespace wmtk