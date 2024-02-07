#pragma once

#include <wmtk/TetMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk::components::internal {

wmtk::Mesh& generate_submesh(wmtk::Mesh& m, wmtk::MeshAttributeHandle<long> taghandle, bool pos)
{
    return m;
}
} // namespace wmtk::components::internal