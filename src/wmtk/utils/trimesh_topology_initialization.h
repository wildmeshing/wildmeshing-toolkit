
#pragma once

#include <wmtk/Types.hpp>


namespace wmtk {

std::tuple<RowVectors3l,RowVectors3l,VectorXl,VectorXl> trimesh_topology_initialization(
    Eigen::Ref<const RowVectors3l> F); // returns {FE, FF, VF, EF}

}