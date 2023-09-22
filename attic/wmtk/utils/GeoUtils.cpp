#include <igl/predicates/predicates.h>
#include <wmtk/utils/GeoUtils.h>

namespace wmtk {

template <>
int orient3d_t(
    const Eigen::Matrix<double, 3, 1>& p1,
    const Eigen::Matrix<double, 3, 1>& p2,
    const Eigen::Matrix<double, 3, 1>& p3,
    const Eigen::Matrix<double, 3, 1>& p4)
{
    igl::predicates::exactinit();

    auto res = igl::predicates::orient3d(p1, p2, p3, p4);
    return res == igl::predicates::Orientation::COPLANAR
               ? 0
               : (res == igl::predicates::Orientation::NEGATIVE ? -1 : 1);
}

template <>
int orient2d_t(
    const Eigen::Matrix<double, 2, 1>& p1,
    const Eigen::Matrix<double, 2, 1>& p2,
    const Eigen::Matrix<double, 2, 1>& p3)
{
    igl::predicates::exactinit();

    auto res = igl::predicates::orient2d(p1, p2, p3);
    return res == igl::predicates::Orientation::COLLINEAR
               ? 0
               : (res == igl::predicates::Orientation::NEGATIVE ? -1 : 1);
}
} // namespace wmtk
