#pragma once

#include <wmtk/Types.hpp>


namespace wmtk {

    // returns a topology for a free mesh and a vector for redirecting indices
std::tuple<MatrixXl,VectorXl> make_free_sv_mesh(
    Eigen::Ref<const MatrixXl> S);

// reindices the vertices
std::tuple<MatrixXl,Eigen::MatrixXd> make_free_sv_mesh_with_positions(
    Eigen::Ref<const MatrixXl> S,
    Eigen::Ref<const Eigen::MatrixXd> V
    );
}
