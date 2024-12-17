
#pragma once

#include <wmtk/Types.hpp>


namespace wmtk {

std::tuple<RowVectors2l, VectorXl> edgemesh_topology_initialization(
    Eigen::Ref<const RowVectors2l> EV); // returns {EE, VE}

}
