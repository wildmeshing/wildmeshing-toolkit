#include "delaunay_2d.hpp"

#include "delaunay_geogram.hpp"

namespace wmtk::components::internal {

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay_2d(
    const std::vector<Eigen::Vector2d>& input_points)
{
    return delaunay_geogram(input_points);
}

} // namespace wmtk::components::internal