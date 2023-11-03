#pragma once

#include "probabilistic-quadrics.hh"

#include <Eigen/Core>

namespace wmtk {

/// Quadric error function in 3D.
template <typename Scalar>
using Quadric = pq::quadric<pq::math<
    Scalar,
    Eigen::Matrix<Scalar, 3, 1>,
    Eigen::Matrix<Scalar, 3, 1>,
    Eigen::Matrix<Scalar, 3, 3>>>;

} // namespace wmtk
