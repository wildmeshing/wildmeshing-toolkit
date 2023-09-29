#include "delaunay_3d.hpp"

#include "delaunay_geogram.hpp"

namespace wmtk::components::internal {

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay_3d(
    const std::vector<Eigen::Vector3d>& input_points)
{
    return delaunay_geogram(input_points);
}

} // namespace wmtk::components::internal