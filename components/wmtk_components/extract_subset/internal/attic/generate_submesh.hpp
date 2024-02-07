#pragma once

#include <wmtk/TriMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk::components::internal {

wmtk::Mesh& generate_submesh(wmtk::Mesh& m, wmtk::MeshAttributeHandle<long> taghandle, bool pos);
} // namespace wmtk::components::internal