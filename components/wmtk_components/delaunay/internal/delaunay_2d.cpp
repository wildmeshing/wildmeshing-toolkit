#include "delaunay_2d.hpp"
#include <wmtk/Types.hpp>
#include "delaunay_geogram.hpp"


namespace wmtk::components::internal {

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay_2d(Eigen::Ref<const RowVectors2d> points)
{
    return delaunay_geogram(points);
}

} // namespace wmtk::components::internal
