
#pragma once
#include <spdlog/logger.h>
#include <filesystem>
namespace wmtk {


/**
 * @brief Given the mesh connectivity in matrix format, finds unique edges and faces and their relations
 */

std::tuple<RowVectors3l,RowVectors3l,RowVectors3l,VectorXl,VectorXl,VectorXl> tetmesh_topology_initialization(
    Eigen::Ref<const RowVectors3l>& T);
    
}