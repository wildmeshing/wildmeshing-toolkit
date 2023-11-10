#pragma once

#include <wmtk/TriMesh.hpp>
#include "internal/extract_subset_2d.hpp"

namespace wmtk {

namespace components {
wmtk::TriMesh
extract_subset(wmtk::TriMesh m, wmtk::MeshAttributeHandle<long> tag_handle, long dimension);
} // namespace components
} // namespace wmtk