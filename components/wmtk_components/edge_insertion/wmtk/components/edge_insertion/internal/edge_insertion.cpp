#include "edge_insertion.hpp"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::internal {

wmtk::Rational det(const wmtk::Vector2r& a, const wmtk::Vector2r& b)
{
    return a[0] * b[1] - a[1] * b[0];
}

int is_point_inside_triangle(
    const wmtk::Vector2r& P,
    const wmtk::Vector2r& A,
    const wmtk::Vector2r& B,
    const wmtk::Vector2r& C)
{
    Vector2r AP = P - A;
    Vector2r BP = P - B;
    Vector2r CP = P - C;
    Vector2r AB = B - A;
    Vector2r BC = C - B;
    Vector2r CA = A - C;

    auto S_pab = abs(det(AP, AB)) / 2;
    auto S_pbc = abs(det(BP, BC)) / 2;
    auto S_pca = abs(det(CP, CA)) / 2;
    auto S_abc = abs(det(AB, -CA)) / 2;

    if (S_pab + S_pbc + S_pca != S_abc) {
        // outside
        return -1;
    }

    if (S_pab * S_pbc * S_pca > 0) {
        // inside
        return 0;
    }

    if (S_pab == 0) {
        if (S_pbc * S_pca > 0) {
            // on AB
            return 1;
        } else {
            // is endpoint
            return 4;
        }
    }

    if (S_pbc == 0) {
        if (S_pca * S_pab > 0) {
            // on BC
            return 2;
        } else {
            // is endpoint
            return 4;
        }
    }

    if (S_pca == 0) {
        if (S_pab * S_pbc > 0) {
            // on CA
            return 3;
        } else {
            // is endpoint
            return 4;
        }
    }

    return -1;
}

bool segment_segment_inter(
    const Vector2r& s0,
    const Vector2r& e0,
    const Vector2r& s1,
    const Vector2r& e1,
    Vector2r& res)
{
    Rational dd = e0[0] * e1[1] - e0[0] * s1[1] - e0[1] * e1[0] + e0[1] * s1[0] + e1[0] * s0[1] -
                  e1[1] * s0[0] + s0[0] * s1[1] - s0[1] * s1[0];

    if (dd.get_sign() == 0) {
        return false;
    }

    const Rational t0 = (e1[0] * s0[1] - e1[0] * s1[1] - e1[1] * s0[0] + e1[1] * s1[0] +
                         s0[0] * s1[1] - s0[1] * s1[0]) /
                        dd;
    const Rational t1 = (e0[0] * s0[1] - e0[0] * s1[1] - e0[1] * s0[0] + e0[1] * s1[0] +
                         s0[0] * s1[1] - s0[1] * s1[0]) /
                        dd;

    if (t0 < 0 || t0 > 1 || t1 < 0 || t1 > 1) {
        return false;
    }

    res = (1 - t0) * s0 + t0 * e0;
#ifndef NDEBUG
    const Vector2r p1 = (1 - t1) * s1 + t1 * e1;

    assert(res[0] == p1[0] && res[1] == p1[1] && res[2] == p1[2]);
#endif
    return true;
}

bool is_point_in_bbox(const Vector2r& point, const bbox& box)
{
    if (point[0] >= box.x_min && point[0] <= box.xmax && point[1] >= box.y_min &&
        point[1] <= box.y_max) {
        return true;
    }
    return false;
}

bool is_bbox_intersect(const bbox& b0, const bbox& b1)
{
    // can intersect at boundary
    if (b1.x_min >= b0.x_min && b1.x_min <= b0.x_max && b1.y_min >= b0.y_min &&
        b1.y_max <= b0.y_max) {
        return true;
    }
    return false;
}

void edge_insertion(TriMesh& triangles, EdgeMesh& segments)
{
    // convert trimesh and edgemesh into eigen matrices
}

} // namespace wmtk::components::internal