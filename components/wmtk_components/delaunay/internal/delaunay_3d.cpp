#include "delaunay_3d.hpp"
#include "delaunay_geogram.hpp"


namespace wmtk::components::internal {

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay_3d(Eigen::Ref<const RowVectors3d> points)
{
    return delaunay_geogram(points);
}

} // namespace wmtk::components::internal
