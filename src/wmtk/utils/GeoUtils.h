#pragma once


#include <igl/predicates/predicates.h>
#include <Eigen/Core>
#include <Eigen/Dense>


#include <array>

namespace wmtk {
template <typename T>
bool segment_triangle_intersection(
    const std::array<Eigen::Matrix<T, 3, 1>, 2>& seg,
    const std::array<Eigen::Matrix<T, 3, 1>, 3>& tri,
    Eigen::Matrix<T, 3, 1>& p)
{
    Eigen::Matrix<T, 3, 1> e0, e1, t1, t2, t3;

    for (int d = 0; d < 3; ++d) {
        e0[d] = seg[0][d];
        e1[d] = seg[1][d];

        t1[d] = tri[0][d];
        t2[d] = tri[1][d];
        t3[d] = tri[2][d];
    }

    const T d = e0[0] * t1[1] * t2[2] - e0[0] * t1[1] * t3[2] - e0[0] * t1[2] * t2[1] +
                e0[0] * t1[2] * t3[1] + e0[0] * t2[1] * t3[2] - e0[0] * t2[2] * t3[1] -
                e0[1] * t1[0] * t2[2] + e0[1] * t1[0] * t3[2] + e0[1] * t1[2] * t2[0] -
                e0[1] * t1[2] * t3[0] - e0[1] * t2[0] * t3[2] + e0[1] * t2[2] * t3[0] +
                e0[2] * t1[0] * t2[1] - e0[2] * t1[0] * t3[1] - e0[2] * t1[1] * t2[0] +
                e0[2] * t1[1] * t3[0] + e0[2] * t2[0] * t3[1] - e0[2] * t2[1] * t3[0] -
                e1[0] * t1[1] * t2[2] + e1[0] * t1[1] * t3[2] + e1[0] * t1[2] * t2[1] -
                e1[0] * t1[2] * t3[1] - e1[0] * t2[1] * t3[2] + e1[0] * t2[2] * t3[1] +
                e1[1] * t1[0] * t2[2] - e1[1] * t1[0] * t3[2] - e1[1] * t1[2] * t2[0] +
                e1[1] * t1[2] * t3[0] + e1[1] * t2[0] * t3[2] - e1[1] * t2[2] * t3[0] -
                e1[2] * t1[0] * t2[1] + e1[2] * t1[0] * t3[1] + e1[2] * t1[1] * t2[0] -
                e1[2] * t1[1] * t3[0] - e1[2] * t2[0] * t3[1] + e1[2] * t2[1] * t3[0];

    // Coplanar
    if (d == 0) return false;

    const T t = (e0[0] * t1[1] * t2[2] - e0[0] * t1[1] * t3[2] - e0[0] * t1[2] * t2[1] +
                 e0[0] * t1[2] * t3[1] + e0[0] * t2[1] * t3[2] - e0[0] * t2[2] * t3[1] -
                 e0[1] * t1[0] * t2[2] + e0[1] * t1[0] * t3[2] + e0[1] * t1[2] * t2[0] -
                 e0[1] * t1[2] * t3[0] - e0[1] * t2[0] * t3[2] + e0[1] * t2[2] * t3[0] +
                 e0[2] * t1[0] * t2[1] - e0[2] * t1[0] * t3[1] - e0[2] * t1[1] * t2[0] +
                 e0[2] * t1[1] * t3[0] + e0[2] * t2[0] * t3[1] - e0[2] * t2[1] * t3[0] -
                 t1[0] * t2[1] * t3[2] + t1[0] * t2[2] * t3[1] + t1[1] * t2[0] * t3[2] -
                 t1[1] * t2[2] * t3[0] - t1[2] * t2[0] * t3[1] + t1[2] * t2[1] * t3[0]) /
                d;
    const T u = (e0[0] * e1[1] * t3[2] + e0[0] * e1[2] * t1[1] - e0[0] * e1[1] * t1[2] -
                 e0[0] * e1[2] * t3[1] - e0[0] * t1[1] * t3[2] + e0[0] * t1[2] * t3[1] +
                 e0[1] * e1[0] * t1[2] - e0[1] * e1[0] * t3[2] - e0[1] * e1[2] * t1[0] +
                 e0[1] * e1[2] * t3[0] + e0[1] * t1[0] * t3[2] - e0[1] * t1[2] * t3[0] -
                 e0[2] * e1[0] * t1[1] + e0[2] * e1[0] * t3[1] + e0[2] * e1[1] * t1[0] -
                 e0[2] * e1[1] * t3[0] - e0[2] * t1[0] * t3[1] + e0[2] * t1[1] * t3[0] +
                 e1[0] * t1[1] * t3[2] - e1[0] * t1[2] * t3[1] - e1[1] * t1[0] * t3[2] +
                 e1[1] * t1[2] * t3[0] + e1[2] * t1[0] * t3[1] - e1[2] * t1[1] * t3[0]) /
                d;
    const T v = (e0[0] * e1[1] * t1[2] - e0[0] * e1[1] * t2[2] - e0[0] * e1[2] * t1[1] +
                 e0[0] * e1[2] * t2[1] + e0[0] * t1[1] * t2[2] - e0[0] * t1[2] * t2[1] -
                 e0[1] * e1[0] * t1[2] + e0[1] * e1[0] * t2[2] + e0[1] * e1[2] * t1[0] -
                 e0[1] * e1[2] * t2[0] - e0[1] * t1[0] * t2[2] + e0[1] * t1[2] * t2[0] +
                 e0[2] * e1[0] * t1[1] - e0[2] * e1[0] * t2[1] - e0[2] * e1[1] * t1[0] +
                 e0[2] * e1[1] * t2[0] + e0[2] * t1[0] * t2[1] - e0[2] * t1[1] * t2[0] -
                 e1[0] * t1[1] * t2[2] + e1[0] * t1[2] * t2[1] + e1[1] * t1[0] * t2[2] -
                 e1[1] * t1[2] * t2[0] - e1[2] * t1[0] * t2[1] + e1[2] * t1[1] * t2[0]) /
                d;

    if (t <= 0 || t >= 1) return false;

    if (u < 0 || u > 1 || v < 0 || v > 1 || u + v > 1) return false;

    p = (1 - t) * e0 + t * e1;

    return true;
}

template <typename T>
void squeeze_points_to_2d(
    const std::vector<Eigen::Matrix<T, 3, 1>>& points3,
    const std::vector<Eigen::Matrix<T, 2, 1>>&
        points2) // note: use the first 3 points to construct the plane
{
    const auto& p1 = points3[0];
    const auto& p2 = points3[1];
    const auto& p3 = points3[2];

    Eigen::Matrix<T, 3, 1> n = (p2 - p1).cross(p3 - p1);

    int J = 0;
    T max = n[J]; // delete max
    for (; J < 3; J++) {
        if (n[J] > max) max = n[J];
    }

    points2.resize(points3.size());
    for (int i = 0; i < points3.size(); i++) {
        if (J == 0) {
            points2[i] = Eigen::Matrix<T, 2, 1>(points3[i][1], points3[i][2]);
        } else if (J == 1) {
            points2[i] = Eigen::Matrix<T, 2, 1>(points3[i][0], points3[i][2]);
        } else {
            points2[i] = Eigen::Matrix<T, 2, 1>(points3[i][0], points3[i][1]);
        }
    }
}

template <typename T>
bool is_point_inside_triangle(
    const Eigen::Matrix<T, 2, 1>& p,
    const std::array<Eigen::Matrix<T, 2, 1>, 3>& tri)
{
    Eigen::Matrix<T, 2, 2> a0, a1, a2;
    a0.col(0) = tri[0] - p;
    a0.col(1) = tri[1] - p;

    a1.col(0) = tri[1] - p;
    a1.col(1) = tri[2] - p;

    a2.col(0) = tri[2] - p;
    a2.col(1) = tri[0] - p;

    return a0.determinant() >= 0 && a1.determinant() >= 0 && a2.determinant() >= 0;
}


template <>
bool is_point_inside_triangle(
    const Eigen::Matrix<double, 2, 1>& p,
    const std::array<Eigen::Matrix<double, 2, 1>, 3>& tri)
{
    igl::predicates::exactinit();
    auto res = igl::predicates::orient2d(p, tri[0], tri[1]);
    if (res == igl::predicates::Orientation::NEGATIVE) return false;

    res = igl::predicates::orient2d(p, tri[1], tri[2]);
    if (res == igl::predicates::Orientation::NEGATIVE) return false;

    res = igl::predicates::orient2d(p, tri[2], tri[0]);
    if (res == igl::predicates::Orientation::NEGATIVE) return false;

    return true;
}

} // namespace wmtk
