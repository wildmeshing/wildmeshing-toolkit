
#pragma once
#include <spdlog/logger.h>
#include <filesystem>
namespace wmtk {


/**
 * @brief Given the mesh connectivity in matrix format, finds unique edges and faces and their relations
 */

void tetmesh_topology_initialization(
    Eigen::Ref<const RowVectors3l>& T,
    Eigen::Ref<RowVectors3l>& TE,
    Eigen::Ref<RowVectors3l>& TF,
    Eigen::Ref<RowVectors3l>& TT,
    Eigen::Ref<VectorXl>& FT,
    Eigen::Ref<VectorXl>& ET,
    Eigen::Ref<VectorXl>& VT);
}