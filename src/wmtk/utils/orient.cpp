#include "orient.hpp"

#include <igl/predicates/predicates.h>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::utils {

bool orient3d(const Vector3r& p0, const Vector3r& p1, const Vector3r& p2, const Vector3r& p3)
{
    const Vector3r a(p1 - p0);
    const Vector3r b(p2 - p0);
    // Vector3r n = a.cross(b);
    // Vector3r d = p3 - p0;
    // auto res = n.dot(d);

    // cross product
    const Rational nx = a[1] * b[2] - a[2] * b[1];
    const Rational ny = a[2] * b[0] - a[0] * b[2];
    const Rational nz = a[0] * b[1] - a[1] * b[0];

    const Vector3r d = p3 - p0;

    // dot product: n · d
    Rational res = nx * d[0] + ny * d[1] + nz * d[2];


    if (res > 0) {
        // predicates returns pos value: non-inverted
        return true;
    } else {
        return false;
    }
}

bool orient3d(const Vector3d& p0, const Vector3d& p1, const Vector3d& p2, const Vector3d& p3)
{
    igl::predicates::exactinit();
    auto res = igl::predicates::orient3d(p0, p1, p2, p3);
    int result;
    if (res == igl::predicates::Orientation::POSITIVE) {
        result = 1;
    } else if (res == igl::predicates::Orientation::NEGATIVE) {
        result = -1;
    } else {
        result = 0;
    }

    if (result < 0) {
        // neg result == pos tet (tet origin from geogram delaunay)
        return true;
    }
    return false;
}

bool orient2d(const Vector2r& p0, const Vector2r& p1, const Vector2r& p2)
{
    Eigen::Matrix2<Rational> M;
    M.row(0) = p1 - p0;
    M.row(1) = p2 - p0;
    const Rational det = M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0);
    // logger().info("{}", det.to_double());
    return det.get_sign() == 1;
}

bool orient2d(const Vector2d& p0, const Vector2d& p1, const Vector2d& p2)
{
    igl::predicates::exactinit();
    auto res = igl::predicates::orient2d(p0, p1, p2);
    if (res == igl::predicates::Orientation::POSITIVE) {
        return true;
    }
    return false;
}

} // namespace wmtk::utils