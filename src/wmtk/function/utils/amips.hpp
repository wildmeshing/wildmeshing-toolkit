#pragma once
#include <Eigen/Dense>
#include <wmtk/Types.hpp>

namespace wmtk::function::utils {

namespace detail {
// returns v0,v1,v2 of the target triangle as row vectors
extern const Eigen::Matrix<double, 2, 3> amips_target_triangle;
// maps from the embedding of the reference triangle to the barycentric coordinates
extern const Eigen::Matrix2d amips_reference_to_barycentric;

} // namespace detail


// Given an basis vectors following a "v1-v0,v2-v0" convention for a triangle (v0,v1,v2)
// return the AMIPS energy for a triangle.
// Input are assumed to be column vectors, opposite of the standard IGL formation
template <typename Derived>
auto amips(const Eigen::MatrixBase<Derived>& B)
{
    using Scalar = typename Derived::Scalar;
    constexpr static int Rows = Derived::RowsAtCompileTime;
    constexpr static int Cols = Derived::ColsAtCompileTime;


    // check that these are vectors
    static_assert(Cols == 2);
    static_assert(Rows == 2);


    // MTAO: Why can't this work with more than 2 rows?
    // define of transform matrix F = Dm@Ds.inv
    Eigen::Matrix<Scalar, Rows, 2> J;
    J = B * detail::amips_reference_to_barycentric.template cast<Scalar>();

    auto Jdet = J.determinant();
    if (abs(Jdet) < std::numeric_limits<double>::denorm_min()) {
        return static_cast<Scalar>(std::numeric_limits<double>::infinity());
    }
    assert(Jdet >= 0);

    return (J * J.transpose()).trace() / Jdet;
}


//
template <typename V0Type, typename V1Type, typename V2Type>
auto amips(
    const Eigen::MatrixBase<V0Type>& v0,
    const Eigen::MatrixBase<V1Type>& v1,
    const Eigen::MatrixBase<V2Type>& v2)
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

    Eigen::Matrix<Scalar, 2, 2> Dm;


    static_assert(Rows == 2 || Rows == 3);

    if constexpr (Rows == 2) {
        Dm.col(0) = (v1.template cast<Scalar>() - v0);
        Dm.col(1) = (v2.template cast<Scalar>() - v0);
    } else if constexpr (Rows == 3) {
        // in 3d we compute a basis
        // local vectors
        Eigen::Matrix<Scalar, Rows, 2> V;
        V.col(0) = v1.template cast<Scalar>() - v0;
        V.col(1) = v2.template cast<Scalar>() - v0;

        // compute a basis plane
        Eigen::Matrix<Scalar, 3, 2> B = V;

        auto e0 = B.col(0);
        auto e1 = B.col(1);

        // TODO: shouldnt we make sure the normms are over some eps instead of 0?
        auto e0norm = e0.norm();
        assert(e0norm > 0); // check norm is not 0
        e0 = e0 / e0norm;

        Vector3<Scalar> n = e0.cross(e1);
        e1 = n.cross(e0);
        auto e1norm = e1.norm();
        assert(e1norm > 0); // check norm is not 0
        e1 = e1 / e1norm;


        Dm = (B.transpose() * V).eval();
    }


    return amips(Dm);
}

double Tet_AMIPS_energy(const std::array<double, 12>& T);
void Tet_AMIPS_hessian(const std::array<double, 12>& T, Eigen::Matrix3d& result_0);
void Tet_AMIPS_jacobian(const std::array<double, 12>& T, Eigen::Vector3d& result_0);

} // namespace wmtk::function::utils
