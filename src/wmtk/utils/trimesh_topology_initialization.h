
#pragma once

#include <wmtk/Types.hpp>


namespace wmtk {

void trimesh_topology_initialization(
    Eigen::Ref<const RowVectors3l> F,
    Eigen::Ref<RowVectors3l> FE,
    Eigen::Ref<RowVectors3l> FF,
    Eigen::Ref<VectorXl> VF,
    Eigen::Ref<VectorXl> EF);

}