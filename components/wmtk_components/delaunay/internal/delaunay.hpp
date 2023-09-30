#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::components::internal {
std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay(Eigen::Ref<const Eigen::MatrixXd> points);
} // namespace wmtk::components::internal
