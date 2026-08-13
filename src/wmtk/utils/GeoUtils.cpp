#include <wmtk/utils/GeoUtils.h>
#include <wmtk/utils/predicates.hpp>

namespace wmtk {

template <>
int orient3d_t(
    const Eigen::Matrix<double, 3, 1>& p1,
    const Eigen::Matrix<double, 3, 1>& p2,
    const Eigen::Matrix<double, 3, 1>& p3,
    const Eigen::Matrix<double, 3, 1>& p4)
{
    wmtk::utils::predicates::exactinit();

    auto res = wmtk::utils::predicates::orient3d(p1, p2, p3, p4);
    return res == wmtk::utils::predicates::Orientation::COPLANAR
               ? 0
               : (res == wmtk::utils::predicates::Orientation::NEGATIVE ? -1 : 1);
}

template <>
int orient2d_t(
    const Eigen::Matrix<double, 2, 1>& p1,
    const Eigen::Matrix<double, 2, 1>& p2,
    const Eigen::Matrix<double, 2, 1>& p3)
{
    wmtk::utils::predicates::exactinit();

    auto res = wmtk::utils::predicates::orient2d(p1, p2, p3);
    return res == wmtk::utils::predicates::Orientation::COLLINEAR
               ? 0
               : (res == wmtk::utils::predicates::Orientation::NEGATIVE ? -1 : 1);
}
} // namespace wmtk
