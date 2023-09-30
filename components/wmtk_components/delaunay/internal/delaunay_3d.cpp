#include "delaunay_3d.hpp"
#include "delaunay.hpp"


namespace wmtk::components::internal {

void delaunay_3d(
    Eigen::Ref<const RowVectors3d> points,
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXi& tetrahedra)
{
    std::tie(vertices, tetrahedra) = delaunay(points);
}

} // namespace wmtk::components::internal
