#pragma once

#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk::components::internal {

wmtk::TriMesh extract_subset_2d(wmtk::TriMesh m, wmtk::MeshAttributeHandle<long> taghandle);
} // namespace wmtk::components::internal