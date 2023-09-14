#pragma once
#include <Eigen/Core>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Tuple.hpp>
namespace wmtk {
double triangle_2d_area(
    const TriMesh& m,
    const MeshAttributeHandle<double>& vertex_uv_handle,
    const Tuple& tuple);

// template get 3d tri area
template <typename T>
T triangle_3d_area(
    const Eigen::Matrix<T, 3, 1>& a,
    const Eigen::Matrix<T, 3, 1>& b,
    const Eigen::Matrix<T, 3, 1>& c)
{
    const T n0 = (a[1] - b[1]) * (a[2] - c[2]) - (a[1] - c[1]) * (a[2] - b[2]);
    const T n1 = -(a[0] - b[0]) * (a[2] - c[2]) + (a[0] - c[0]) * (a[2] - b[2]);
    const T n2 = (a[0] - b[0]) * (a[1] - c[1]) - (a[0] - c[0]) * (a[1] - b[1]);

    return sqrt(n0 * n0 + n1 * n1 + n2 * n2) * static_cast<T>(0.5);
}

// template get 3d tri area
template <typename T>
T triangle_2d_area(
    const Eigen::Matrix<T, 2, 1>& A,
    const Eigen::Matrix<T, 2, 1>& B,
    const Eigen::Matrix<T, 2, 1>& C)
{
    auto B_A = B - A;
    auto C_A = C - A;
    T area = static_cast<T>(0.5) * abs(B_A.x() * C_A.y() - B_A.y() * C_A.x());
    return area;
}


} // namespace wmtk