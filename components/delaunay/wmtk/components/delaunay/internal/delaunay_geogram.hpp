#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::components::internal {

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay_geogram(
    Eigen::Ref<const Eigen::MatrixXd> points);
} // namespace wmtk::components::internal