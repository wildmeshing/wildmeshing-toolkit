#pragma once


#include <igl/predicates/predicates.h>
#include <Eigen/Core>
#include <Eigen/Dense>


#include <array>

namespace wmtk {


template <typename T>
int orient3d_t(
    const Eigen::Matrix<T, 3, 1>& p1,
    const Eigen::Matrix<T, 3, 1>& p2,
    const Eigen::Matrix<T, 3, 1>& p3,
    const Eigen::Matrix<T, 3, 1>& p4)
{
    const auto v = ((p2 - p1).cross(p3 - p1)).dot(p4 - p1);
    return v == 0 ? 0 : (v < 0 ? -1 : 1);
}

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


template <typename T>
T cross_2d(const Eigen::Matrix<T, 2, 1>& p1, const Eigen::Matrix<T, 2, 1>& p2)
{
    Eigen::Matrix<T, 2, 2> mat;
    mat.col(0) = (p1);
    mat.col(1) = (p2);
    return mat.determinant();
}


template <typename T>
int orient2d_t(
    const Eigen::Matrix<T, 2, 1>& p1,
    const Eigen::Matrix<T, 2, 1>& p2,
    const Eigen::Matrix<T, 2, 1>& p3)
{
    const auto det = cross_2d<T>(p2 - p1, p3 - p1);
    return det == 0 ? 0 : (det < 0 ? -1 : 1);
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

template <typename T>
bool open_segment_triangle_intersection_3d(
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
bool open_segment_plane_intersection_3d(
    const std::array<Eigen::Matrix<T, 3, 1>, 2>& seg,
    const std::array<Eigen::Matrix<T, 3, 1>, 3>& tri,
    Eigen::Matrix<T, 3, 1>& p,
    bool& is_inside)
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

    is_inside = true;
    if (u < 0 || u > 1 || v < 0 || v > 1 || u + v > 1) is_inside = false;

    p = (1 - t) * e0 + t * e1;

    return true;
}

template <typename T>
bool segment_triangle_coplanar_3d(
    const std::array<Eigen::Matrix<T, 3, 1>, 2>& seg,
    const std::array<Eigen::Matrix<T, 3, 1>, 3>& tri)
{
    const auto o1 = orient3d_t(seg[0], tri[0], tri[1], tri[2]);
    const auto o2 = orient3d_t(seg[1], tri[0], tri[1], tri[2]);

    return o1 == 0 && o2 == 0;
}

// note: t1 is for seg1, intersection point: (1 - t1) * seg1[0] + t1 * seg1[1]
template <typename T>
bool open_segment_open_segment_intersection_2d(
    const std::array<Eigen::Matrix<T, 2, 1>, 2>& seg_1,
    const std::array<Eigen::Matrix<T, 2, 1>, 2>& seg_2,
    T& t1)
{
    const auto o1 = orient2d_t(seg_1[0], seg_2[0], seg_2[1]);
    const auto o2 = orient2d_t(seg_1[1], seg_2[0], seg_2[1]);
    t1 = -1;

    if (o1 == 0 && o2 == 0) // Collinear
    {
        // If they overlap we return false, if they dont we return false
        return false;
    }

    if (o1 * o2 >= 0) // two vertices of seg1 are on the same side, or on
        return false;

    const auto o3 = orient2d_t(seg_2[0], seg_1[0], seg_1[1]);
    const auto o4 = orient2d_t(seg_2[1], seg_1[0], seg_1[1]);

    if (o3 * o4 >= 0) // two vertices of seg1 are on the same side, or on
        return false;

    const Eigen::Matrix<T, 2, 1> e1 = seg_1[1] - seg_1[0];
    const Eigen::Matrix<T, 2, 1> e2 = seg_2[1] - seg_2[0];
    t1 = cross_2d<T>(seg_2[0] - seg_1[0], e2) / cross_2d(e1, e2);

    // TODO add t
    return true;
}

// note: use the first 3 points to construct the plane
template <typename T>
int project_triangle_to_2d(
    const std::array<Eigen::Matrix<T, 3, 1>, 3>& points3,
    std::array<Eigen::Matrix<T, 2, 1>, 3>& points2)
{
    const auto& p1 = points3[0];
    const auto& p2 = points3[1];
    const auto& p3 = points3[2];

    Eigen::Matrix<T, 3, 1> n = (p2 - p1).cross(p3 - p1);

    int J = 0;
    T max = n[J].abs(); // delete max
    for (int j = 0; j < 3; j++) {
        if (n[j].abs() > max) {
            max = n[j].abs();
            J = j;
        }
    }

    for (int i = 0; i < points3.size(); i++) {
        if (J == 0) {
            points2[i] = Eigen::Matrix<T, 2, 1>(points3[i][1], points3[i][2]);
        } else if (J == 1) {
            points2[i] = Eigen::Matrix<T, 2, 1>(points3[i][0], points3[i][2]);
        } else {
            points2[i] = Eigen::Matrix<T, 2, 1>(points3[i][0], points3[i][1]);
        }
    }

    return J;
}

template <typename T>
Eigen::Matrix<T, 2, 1> project_point_to_2d(const Eigen::Matrix<T, 3, 1>& p, int t)
{
    if (t == 0)
        return Eigen::Matrix<T, 2, 1>(p[1], p[2]);
    else if (t == 1)
        return Eigen::Matrix<T, 2, 1>(p[0], p[2]);
    else
        return Eigen::Matrix<T, 2, 1>(p[0], p[1]);
}

template <typename T>
bool is_point_inside_triangle( // closed triangle
    const Eigen::Matrix<T, 2, 1>& p,
    const std::array<Eigen::Matrix<T, 2, 1>, 3>& tri)
{
    //    auto res = orient2d_t(p, tri[0], tri[1]);
    //    if (res < 0) return false;
    //
    //    res = orient2d_t(p, tri[1], tri[2]);
    //    if (res < 0) return false;
    //
    //    res = orient2d_t(p, tri[2], tri[0]);
    //    if (res < 0) return false;

    auto res1 = orient2d_t(p, tri[0], tri[1]);
    auto res2 = orient2d_t(p, tri[1], tri[2]);
    auto res3 = orient2d_t(p, tri[2], tri[0]);

    if (res1 == 0 && res2 == 0) return true;
    if (res1 == 0 && res3 == 0) return true;
    if (res2 == 0 && res3 == 0) return true;

    if ((res1 == res2 && res2 == res3) || (res1 == 0 && res2 == res3) ||
        (res2 == 0 && res3 == res1) || (res3 == 0 && res2 == res1))
        return true;
    return false;
}

} // namespace wmtk
