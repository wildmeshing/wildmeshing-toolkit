#pragma once

#include <memory>
#include <wmtk/Types.hpp>

namespace wmtk {
class TetMesh;
class Mesh;
} // namespace wmtk

namespace wmtk::utils {


std::tuple<std::shared_ptr<wmtk::TetMesh>, std::shared_ptr<wmtk::Mesh>>
generate_raw_tetmesh_with_surface_from_input(
    const RowVectors3d& V,
    const RowVectors3l& F,
    const double eps_target);

/**
 * @brief input a triangle surface mesh, embed it into a regular grid and generate a tetmesh,
 * track the input surfaces on the tetmesh
 *
 * @param V input vertex coordinates
 * @param F input FV matrix
 * @param target_edge_length target edge length
 * @return [tetmesh ptr, tet local faces on input surface]
 */

std::tuple<std::shared_ptr<wmtk::TetMesh>, std::vector<std::array<bool, 4>>>
generate_raw_tetmesh_from_input_surface(
    const RowVectors3d& V,
    const RowVectors3l& F,
    const double eps_target);


} // namespace wmtk::utils
