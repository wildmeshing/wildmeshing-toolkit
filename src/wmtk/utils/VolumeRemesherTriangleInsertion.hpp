#pragma once

#include <memory>
#include <wmtk/Types.hpp>

namespace wmtk {
class TetMesh;
}

namespace wmtk::utils {

std::shared_ptr<wmtk::TetMesh> generate_raw_tetmesh_from_input_surface(
    const RowVectors3d& V,
    const RowVectors3l& F);


} // namespace wmtk::utils
