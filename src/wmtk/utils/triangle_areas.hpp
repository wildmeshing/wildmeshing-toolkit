#pragma once

#include <Eigen/Core>


namespace wmtk {
class Tuple;
class TriMesh;
namespace attribute {
template <typename T>
class MeshAttributeHandle;
}
} // namespace wmtk
namespace wmtk::utils {
class Area
{
public:
    // template get 3d tri area
    template <typename ADerived, typename BDerived, typename CDerived>
    static auto triangle_3d(
        const Eigen::MatrixBase<ADerived>& a,
        const Eigen::MatrixBase<BDerived>& b,
        const Eigen::MatrixBase<CDerived>& c) -> typename ADerived::Scalar
    {
        auto ba = b - a;
        auto ca = c - a;
        return typename ADerived::Scalar(.5) * ba.cross(ca).norm();
    }

    // template get 3d tri area
    template <typename ADerived, typename BDerived, typename CDerived>
    static auto triangle_signed_2d(
        const Eigen::MatrixBase<ADerived>& a,
        const Eigen::MatrixBase<BDerived>& b,
        const Eigen::MatrixBase<CDerived>& c) -> typename ADerived::Scalar
    {
        auto ba = (b - a).eval();
        auto ca = (c - a).eval();
        return typename ADerived::Scalar(.5) * ba.homogeneous().cross(ca.homogeneous()).z();
    }

    template <typename ADerived, typename BDerived, typename CDerived>
    static auto triangle_unsigned_2d(
        const Eigen::MatrixBase<ADerived>& a,
        const Eigen::MatrixBase<BDerived>& b,
        const Eigen::MatrixBase<CDerived>& c) -> typename ADerived::Scalar
    {
        return std::abs(triangle_signed_2d(a, b, c));
    }

    template <typename ADerived, typename BDerived, typename CDerived>
    static bool triangle_2d_orientation(
        const Eigen::MatrixBase<ADerived>& a,
        const Eigen::MatrixBase<BDerived>& b,
        const Eigen::MatrixBase<CDerived>& c)
    {
        auto res = orient2d_aux(a.data(), b.data(), c.data());
        if (res > 0)
            return true;
        else
            return false;
    }

private:
    class Internal
    {
    private:
        Internal();

    public:
        Internal& instance();
        double orient2d_aux(double pa[2], double pb[2], double pc[2]);
        double orient3d_aux(double pa[3], double pb[3], double pc[3], double pd[3]);
        double incircle_aux(double pa[2], double pb[2], double pc[2], double pd[2]);
        double insphere_aux(double pa[3], double pb[3], double pc[3], double pd[3], double pe[3]);
    };
};
} // namespace wmtk::utils
