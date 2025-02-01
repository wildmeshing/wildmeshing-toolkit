#pragma once

#include <memory>
#include <wmtk/Types.hpp>

namespace wmtk {
class TetMesh;
class Mesh;
} // namespace wmtk

namespace wmtk::utils {

/**
 * @brief input a triangle surface mesh, embed it into a background tet mesh,
 * track the input surfaces on the tetmesh
 *
 * @param V input vertex coordinates
 * @param F input FV matrix
 * @param bgV input background vertex coordinates
 * @param bgT input background TV (tets) matrix
 * @return [tetmesh ptr, tet local faces on input surface]
 */

std::tuple<std::shared_ptr<wmtk::TetMesh>, std::vector<std::array<bool, 4>>>
generate_raw_tetmesh_from_input_surface(
    const RowVectors3d& V,
    const RowVectors3l& F,
    const RowVectors3d& bgV,
    const RowVectors4l& bgT);


} // namespace wmtk::utils
