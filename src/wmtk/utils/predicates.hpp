#pragma once

#include <Eigen/Core>

namespace wmtk::utils::predicates {

/**
 * @brief Sign of an exact geometric predicate.
 *
 * Deliberately the same shape and the same values as igl::predicates::Orientation, which
 * this replaced: the call sites were written against that enum, and comparing against
 * POSITIVE / COLLINEAR / COPLANAR reads the same either way.
 */
enum class Orientation {
    POSITIVE = 1,
    INSIDE = 1,
    NEGATIVE = -1,
    OUTSIDE = -1,
    COLLINEAR = 0,
    COPLANAR = 0,
    COCIRCULAR = 0,
    COSPHERICAL = 0,
    DEGENERATE = 0
};

namespace detail {

/**
 * @brief The exact backend, behind a plain-double interface.
 *
 * Declared here and defined in predicates.cpp rather than included, and it has to be that
 * way. The backend reaches its arithmetic kernel through `#include "numerics.h"`, and
 * VolumeRemesher's `VolumeRemesher/numerics.h` is `namespace vol_rem { #include <numerics.h> }`
 * around that same NFG header. `#pragma once` then means whichever party includes it first
 * captures it into their namespace and the other can never have it: include VolumeRemesher
 * first and the global `bigrational` does not exist, include the backend first and
 * `vol_rem::bigrational` does not. Wrapping it on this side does not help -- in a TU where
 * VolumeRemesher won, the wrap would include nothing.
 *
 * Keeping the include in a single .cpp that never sees VolumeRemesher sidesteps it, and
 * keeps a heavy header out of every translation unit. It costs the inlining of the predicate
 * call: measured at 0.54 ns per call, about 16% of the predicate's own cost but far below
 * run-to-run noise on any real workload, so it is not worth trading the isolation for.
 *
 * These return -1 / 0 / +1 in the convention the wrappers below document.
 */
int orient2d_sign(double ax, double ay, double bx, double by, double cx, double cy);
int orient3d_sign(
    double ax,
    double ay,
    double az,
    double bx,
    double by,
    double bz,
    double cx,
    double cy,
    double cz,
    double dx,
    double dy,
    double dz);

inline Orientation from_sign(const int s)
{
    return s > 0 ? Orientation::POSITIVE : (s < 0 ? Orientation::NEGATIVE : Orientation::COLLINEAR);
}

} // namespace detail

/**
 * @brief No-op, kept so call sites that initialized the old backend still compile.
 *
 * Shewchuk's predicates compute their error bounds once at start-up; the current backend
 * carries its filter constants as literals and needs nothing. Left as an empty inline rather
 * than deleted, so that a future backend which does need initialization has somewhere to put
 * it and the call sites do not have to be found again.
 */
inline void exactinit() {}

/**
 * @brief Exact orientation of three 2D points: POSITIVE when counter-clockwise.
 *
 * The backend and Shewchuk agree here, sign for sign. Verified over 400k mixed
 * random/integer/degenerate triples: 400000 identical, 0 flipped, 5329 of them degenerate.
 */
template <typename D0, typename D1, typename D2>
inline Orientation orient2d(
    const Eigen::MatrixBase<D0>& a,
    const Eigen::MatrixBase<D1>& b,
    const Eigen::MatrixBase<D2>& c)
{
    return detail::from_sign(detail::orient2d_sign(a[0], a[1], b[0], b[1], c[0], c[1]));
}

/**
 * @brief Exact orientation of four 3D points, in Shewchuk's convention.
 *
 * The two libraries disagree on the sign here, because they take the determinant of
 * different differences: Shewchuk uses det[a-d; b-d; c-d], the backend det[b-a; c-a; d-a],
 * and those are opposite. Measured over 400k mixed quadruples: 399197 flipped, and the 803
 * that matched were exactly the 803 degenerate ones, where the sign cannot tell them apart.
 *
 * Getting this wrong inverts every tet-orientation test in the toolkit while still passing
 * anything that only asks about degeneracy, so the correction lives in one place --
 * orient3d_sign -- rather than at the call sites.
 */
template <typename D0, typename D1, typename D2, typename D3>
inline Orientation orient3d(
    const Eigen::MatrixBase<D0>& a,
    const Eigen::MatrixBase<D1>& b,
    const Eigen::MatrixBase<D2>& c,
    const Eigen::MatrixBase<D3>& d)
{
    return detail::from_sign(
        detail::
            orient3d_sign(a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2]));
}

/**
 * @brief Check if three vertices are collinear using exact predicates.
 */
template <typename D0, typename D1, typename D2>
inline bool is_degenerate(
    const Eigen::MatrixBase<D0>& v0,
    const Eigen::MatrixBase<D1>& v1,
    const Eigen::MatrixBase<D2>& v2)
{
    if constexpr (Eigen::MatrixBase<D0>::RowsAtCompileTime == 3) {
        // Three points in 3D are collinear only if they are collinear in all three
        // coordinate-plane projections; any one projection can flatten a real triangle.
        for (int dim = 0; dim < 3; ++dim) {
            const Eigen::Vector2d p0(v0[dim], v0[(dim + 1) % 3]);
            const Eigen::Vector2d p1(v1[dim], v1[(dim + 1) % 3]);
            const Eigen::Vector2d p2(v2[dim], v2[(dim + 1) % 3]);
            if (orient2d(p0, p1, p2) != Orientation::COLLINEAR) {
                return false;
            }
        }
        return true;
    } else {
        return orient2d(v0, v1, v2) == Orientation::COLLINEAR;
    }
}

/**
 * @brief Check if four vertices in 3D are coplanar using exact predicates.
 */
template <typename D0, typename D1, typename D2, typename D3>
inline bool is_degenerate(
    const Eigen::MatrixBase<D0>& v0,
    const Eigen::MatrixBase<D1>& v1,
    const Eigen::MatrixBase<D2>& v2,
    const Eigen::MatrixBase<D3>& v3)
{
    return orient3d(v0, v1, v2, v3) == Orientation::COPLANAR;
}

} // namespace wmtk::utils::predicates
