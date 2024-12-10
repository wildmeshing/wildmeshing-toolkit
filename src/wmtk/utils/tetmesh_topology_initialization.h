
#pragma once

#include <wmtk/Types.hpp>

namespace wmtk {


/**
 * @brief Given the mesh connectivity in matrix format, finds unique edges and faces and their
 * relations
 */

std::tuple<RowVectors6l, RowVectors4l, RowVectors4l, VectorXl, VectorXl, VectorXl>
tetmesh_topology_initialization(Eigen::Ref<const RowVectors4l> TV);
// returns TE, TF, TT, VT, ET, FT
} // namespace wmtk
