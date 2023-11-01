#include "amips.hpp"
#include <Eigen/Dense>
#include <array>
#include <wmtk/utils/triangle_areas.hpp>
namespace wmtk::function::utils {
namespace detail {
namespace {
auto make_amips_target_triangle()
{
    const static std::array<std::array<double, 2>, 3> m_target_triangle{{
        // comments to keep formatting
        std::array<double, 2>{{0., 0.}}, //
        std::array<double, 2>{{1., 0.}}, //
        std::array<double, 2>{{0.5, sqrt(3) / 2.}},
        //
    }};
    auto map = Eigen::Matrix<double, 2, 3>::ConstMapType(m_target_triangle[0].data());


#if !defined(NDEBUG)
    auto x = map.col(0);
    auto y = map.col(1);
    auto z = map.col(2);
    assert(wmtk::utils::triangle_signed_2d_area(x, y, z) > 0);
#endif
    return map;
}

} // namespace
const Eigen::Matrix<double, 2, 3> amips_target_triangle = make_amips_target_triangle();

namespace {
Eigen::Matrix2d make_amips_reference_to_barycentric()
{
    const auto& A = amips_target_triangle;
    Eigen::Matrix2d Ds = (A.rightCols<2>().colwise() - A.col(0));

    return Ds.inverse();
}

} // namespace
const Eigen::Matrix2d amips_reference_to_barycentric = make_amips_reference_to_barycentric();
} // namespace detail
} // namespace wmtk::function::utils
