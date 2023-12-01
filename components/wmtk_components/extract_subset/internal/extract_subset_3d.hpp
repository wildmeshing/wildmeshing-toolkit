#pragma once

#include <wmtk/TetMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "utils.hpp"

namespace wmtk::components::internal {

wmtk::TetMesh
extract_subset_3d(wmtk::TetMesh m, wmtk::MeshAttributeHandle<long> taghandle, bool pos);
} // namespace wmtk::components::internal