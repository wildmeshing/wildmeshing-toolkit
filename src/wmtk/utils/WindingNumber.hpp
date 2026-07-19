#pragma once

// A winding-number evaluation copied from libigl (igl::solid_angle /
// igl::winding_number, Alec Jacobson et al., github.com/libigl/libigl, MPL-2.0),
// renamed into wmtk, with two deliberate departures from igl:
//
//   * the solid-angle formula's atan2 is routed through wmtk::atan2, which is the
//     fast architecture-specific std::atan2 by default and the hand-rolled
//     deterministic atan2 when WMTK_FP_STRICT is set. atan2 is the only
//     transcendental here; everything else uses IEEE-754 correctly-rounded
//     operations (+ - * / sqrt), so gating atan2 is sufficient to make the result
//     bit-identical across platforms in a strict build; and
//   * the hierarchical igl::WindingNumberAABB accelerator is not used -- the sum
//     is evaluated directly, O(#queries * #faces). The AABB tree is built with a
//     floating-point spatial split that is not bit-identical across STL/compiler/
//     CPU, so it would reintroduce non-determinism even with a deterministic
//     atan2. The direct sum carries no such state.
//
// The query loop is parallelised with wmtk's own threading framework so it honours
// the requested num_threads.

#include <wmtk/utils/Transcendentals.hpp>

#include <wmtk/threading/parallel_for.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace wmtk::utils {

/**
 * @brief Solid angle subtended by triangle (A,B,C) at point P, divided by 2*pi.
 *
 * Van Oosterom-Strackee formula, as in igl::solid_angle. The single atan2 is
 * wmtk::atan2 (deterministic when WMTK_FP_STRICT is set), which is what makes the
 * result bit-identical across platforms in a strict build.
 */
inline double solid_angle_2pi(
    const Eigen::RowVector3d& A,
    const Eigen::RowVector3d& B,
    const Eigen::RowVector3d& C,
    const Eigen::RowVector3d& P)
{
    // vectors from P to the three corners
    const double a0 = A(0) - P(0), a1 = A(1) - P(1), a2 = A(2) - P(2);
    const double b0 = B(0) - P(0), b1 = B(1) - P(1), b2 = B(2) - P(2);
    const double c0 = C(0) - P(0), c1 = C(1) - P(1), c2 = C(2) - P(2);

    // sqrt is IEEE-754 correctly rounded, hence already deterministic
    const double la = std::sqrt(a0 * a0 + a1 * a1 + a2 * a2);
    const double lb = std::sqrt(b0 * b0 + b1 * b1 + b2 * b2);
    const double lc = std::sqrt(c0 * c0 + c1 * c1 + c2 * c2);

    // determinant of [a; b; c]
    const double detf =
        a0 * b1 * c2 + b0 * c1 * a2 + c0 * a1 * b2 - c0 * b1 * a2 - b0 * a1 * c2 - a0 * c1 * b2;

    // pairwise dot products
    const double dp0 = b0 * c0 + b1 * c1 + b2 * c2; // b . c
    const double dp1 = c0 * a0 + c1 * a1 + c2 * a2; // c . a
    const double dp2 = a0 * b0 + a1 * b1 + a2 * b2; // a . b

    const double denom = la * lb * lc + dp0 * la + dp1 * lb + dp2 * lc;

    // 2*pi as its nearest double (identical everywhere)
    static const double TWO_PI = 6.28318530717958623200;
    return wmtk::atan2(detf, denom) / TWO_PI;
}

/**
 * @brief Winding number of every query point O.row(i) with respect to the triangle
 * mesh (V, F), by direct summation of per-face solid angles. Equivalent to
 * igl::winding_number(V, F, O, W) up to floating point, with the query loop
 * parallelised over num_threads.
 */
inline void winding_number(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXd& O,
    Eigen::VectorXd& W,
    int num_threads)
{
    W.setZero(O.rows());
    if (O.rows() == 0 || F.rows() == 0 || V.rows() == 0) return;

    const int nf = static_cast<int>(F.rows());

    wmtk::threading::task_arena arena(std::max(num_threads, 1));
    arena.execute([&]() {
        wmtk::threading::parallel_for(
            wmtk::threading::blocked_range<int>(0, static_cast<int>(O.rows())),
            [&](wmtk::threading::blocked_range<int> r) {
                for (int o = r.begin(); o < r.end(); ++o) {
                    const Eigen::RowVector3d p = O.row(o);
                    double w = 0.0;
                    for (int f = 0; f < nf; ++f) {
                        w += solid_angle_2pi(V.row(F(f, 0)), V.row(F(f, 1)), V.row(F(f, 2)), p);
                    }
                    W(o) = w;
                }
            });
    });
}

} // namespace wmtk::utils
