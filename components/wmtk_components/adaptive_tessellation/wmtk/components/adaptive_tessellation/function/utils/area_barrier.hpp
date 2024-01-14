
#pragma once
#include <Eigen/Dense>
#include <wmtk/Types.hpp>
#include <wmtk/utils/triangle_areas.hpp>

namespace wmtk::function::utils {
template <typename V0Type, typename V1Type, typename V2Type>
auto area_barrier(
    const Eigen::MatrixBase<V0Type>& v0,
    const Eigen::MatrixBase<V1Type>& v1,
    const Eigen::MatrixBase<V2Type>& v2,
    double barrier_triangle_area)
{
    using Scalar = typename V0Type::Scalar;
    constexpr static int Rows = V0Type::RowsAtCompileTime;
    constexpr static int Cols0 = V0Type::ColsAtCompileTime;
    constexpr static int Cols1 = V1Type::ColsAtCompileTime;
    constexpr static int Cols2 = V1Type::ColsAtCompileTime;


    // check that these are vectors
    static_assert(Cols0 == 1);
    static_assert(Cols1 == 1);
    static_assert(Cols2 == 1);

    // just check that the inputs had the right dimensions
    constexpr static int Rows1 = V1Type::RowsAtCompileTime;
    constexpr static int Rows2 = V1Type::RowsAtCompileTime;
    static_assert(Rows == Rows1);
    static_assert(Rows == Rows2);
    static_assert(Rows == 2);
    auto A = wmtk::utils::triangle_signed_2d_area(v0, v1, v2);
    if (A >= static_cast<Scalar>(barrier_triangle_area)) {
        return static_cast<Scalar>(0.);
    }
    return -pow(A - static_cast<Scalar>(barrier_triangle_area), 2) *
           log(A / static_cast<Scalar>(barrier_triangle_area));
}
} // namespace wmtk::function::utils