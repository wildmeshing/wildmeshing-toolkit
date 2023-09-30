#include "delaunay_2d.hpp"
#include <wmtk/Types.hpp>
#include "delaunay.hpp"


namespace wmtk::components::internal {

void delaunay_2d(
    Eigen::Ref<const RowVectors2d> points,
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXi& triangles)
{
    std::tie(vertices, triangles) = delaunay(points);
}

} // namespace wmtk::components::internal
