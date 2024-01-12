
#include "VolumeRemesherTriangleInsertion.hpp"
#include <wmtk/TetMesh.hpp>

// clang-format off
#include <VolumeRemesher/embed.h>
// clang-format on


namespace wmtk::utils {

// void generate_background_mesh(
//     const RowVectors3d& V,
//     RowVectors3d& background_V,
//     RowVecors4l& background_TV)
// {}

std::shared_ptr<wmtk::TetMesh> generate_raw_tetmesh_from_input_surface(
    const RowVectors3d& V,
    const RowVectors3l& F)
{
    return nullptr;
}

} // namespace wmtk::utils