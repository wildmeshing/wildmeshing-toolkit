#pragma once
#include <Eigen/Dense>
#include <wmtk/Types.hpp>

namespace wmtk::function::utils {

// Given an basis vectors following a "v1-v0,v2-v0" convention for a triangle (v0,v1,v2)
// return the symdir energy.
// Input are assumed to be column vectors, opposite of the standard IGL formation
template <typename Derived1, typename Derived2>
auto symdir(const Eigen::MatrixBase<Derived1>& Dm, const Eigen::MatrixBase<Derived2>& Ds)
{
    using Scalar = typename Derived1::Scalar;

    constexpr static int Rows = Derived1::RowsAtCompileTime;
    constexpr static int Cols = Derived1::ColsAtCompileTime;


    // check that these are vectors
    static_assert(Cols == 2);
    static_assert(Rows == 2);

    // define of transform matrix F = Dm@Ds.inv
    Eigen::Matrix<Scalar, Rows, 2> J;
    J = Dm * Ds;

    auto Jdet = J.determinant();
    if (abs(Jdet) < std::numeric_limits<double>::denorm_min()) {
        return static_cast<Scalar>(std::numeric_limits<double>::infinity());
    }

    return (J(0, 0) * J(0, 0) + J(0, 1) * J(0, 1) + J(1, 0) * J(1, 0) + J(1, 1) * J(1, 1)) *
           (1 + 1 / (Jdet * Jdet));
    // return (J * J.transpose()).trace() / Jdet;
}


//
template <
    typename RefV0Type,
    typename RefV1Type,
    typename RefV2Type,
    typename V0Type,
    typename V1Type,
    typename V2Type>
auto symdir(
    const Eigen::MatrixBase<RefV0Type>& ref_v0,
    const Eigen::MatrixBase<RefV1Type>& ref_v1,
    const Eigen::MatrixBase<RefV2Type>& ref_v2,
    const Eigen::MatrixBase<V0Type>& v0,
    const Eigen::MatrixBase<V1Type>& v1,
    const Eigen::MatrixBase<V2Type>& v2)
{
    using RefScalar = typename RefV0Type::Scalar;
    using Scalar = typename V0Type::Scalar;
    constexpr static int RefRows = RefV0Type::RowsAtCompileTime;
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
    constexpr static int Rows2 = V2Type::RowsAtCompileTime;
    static_assert(Rows == Rows1);
    static_assert(Rows == Rows2);
    static_assert(Rows == 2);

    Eigen::Matrix<Scalar, 2, 2> Dm;
    Dm.col(0) = (v1.template cast<Scalar>() - v0);
    Dm.col(1) = (v2.template cast<Scalar>() - v0);

    Eigen::Matrix<RefScalar, 2, 2> Ds;
    {
        Eigen::Matrix<RefScalar, RefRows, 2> V;
        V.col(0) = ref_v1.template cast<RefScalar>() - ref_v0;
        V.col(1) = ref_v2.template cast<RefScalar>() - ref_v0;

        // compute a basis plane
        Eigen::Matrix<RefScalar, 3, 2> B = V;

        auto e0 = B.col(0);
        auto e1 = B.col(1);

        // TODO: shouldnt we make sure the normms are over some eps instead of 0?
        auto e0norm = e0.norm();
        // assert(e0norm > 0); // check norm is not 0
        e0 = e0 / e0norm;

        Vector3<RefScalar> n = e0.cross(e1);
        e1 = n.cross(e0);
        auto e1norm = e1.norm();
        // assert(e1norm > 0); // check norm is not 0
        e1 = e1 / e1norm;

        Ds = (B.transpose() * V).eval();
        Ds = Ds.inverse().eval();
    }
    return symdir(Dm, Ds);
}
} // namespace wmtk::function::utils
