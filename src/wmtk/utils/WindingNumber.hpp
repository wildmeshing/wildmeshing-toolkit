#pragma once

// A local copy of libigl's winding-number evaluation, but with the per-query
// parallelism driven by wmtk's own threading framework (parallel_for)
// instead of igl::parallel_for. This means the winding number honours the
// requested `num_threads` (like every other parallel section in wmtk) rather than
// always grabbing all hardware cores.
//
// The actual algorithm is unchanged: it reuses igl::WindingNumberAABB (a
// header-only hierarchical accelerator) so it stays as fast as igl::winding_number
// while producing identical results.

#include <Eigen/Core>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/winding_number.h>      // must precede WindingNumberAABB.h (its guard needs it)
#include <igl/WindingNumberAABB.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <wmtk/threading/parallel_for.hpp>

#include <algorithm>
#include <cmath>

namespace wmtk::utils {

namespace detail {

/**
 * @brief Signed angle subtended at P by the oriented 2D segment (A, B), in turns.
 *
 * Vendored from libigl's igl::signed_angle (MPL-2.0, Copyright (C) 2015 Alec Jacobson),
 * with the same arithmetic and the same order of operations, so the result is bit-identical.
 *
 * It is copied rather than called because the only libigl entry point that evaluates a whole
 * point set, igl::winding_number(V, E, O, W), parallelises internally -- see the note on
 * winding_number_2d. Having the per-point evaluation here is what lets that function look
 * like the 3D one above: one loop, one level of parallelism, driven by wmtk.
 */
inline double signed_angle_2d(double ax, double ay, double bx, double by, double px, double py)
{
    // Gather vectors to source and destination, and their lengths.
    double o2A[2] = {px - ax, py - ay};
    double o2B[2] = {px - bx, py - by};
    double o2Al = 0;
    double o2Bl = 0;
    for (int i = 0; i < 2; i++) {
        o2Al += o2A[i] * o2A[i];
        o2Bl += o2B[i] * o2B[i];
    }
    o2Al = std::sqrt(o2Al);
    o2Bl = std::sqrt(o2Bl);
    // Normalize, guarding the degenerate case where P coincides with an endpoint.
    for (int i = 0; i < 2; i++) {
        if (o2Al != 0) {
            o2A[i] /= o2Al;
        }
        if (o2Bl != 0) {
            o2B[i] /= o2Bl;
        }
    }
    constexpr double two_pi = 2.0 * 3.14159265358979323846;
    return -std::atan2(o2B[0] * o2A[1] - o2B[1] * o2A[0], o2B[0] * o2A[0] + o2B[1] * o2A[1]) /
           two_pi;
}

} // namespace detail

/**
 * @brief Winding number of a single query point with respect to the segment soup (V, E).
 *
 * The 2D counterpart of WindingNumberAABB::winding_number(p): a direct O(#E) sweep, since
 * libigl has no hierarchical accelerator in 2D. Segments are summed in index order, matching
 * igl::winding_number(V, E, p) exactly.
 */
inline double
winding_number_2d_point(const Eigen::MatrixXd& V, const Eigen::MatrixXi& E, double px, double py)
{
    double w = 0;
    for (Eigen::Index f = 0; f < E.rows(); ++f) {
        const Eigen::Index a = E(f, 0);
        const Eigen::Index b = E(f, 1);
        w += detail::signed_angle_2d(V(a, 0), V(a, 1), V(b, 0), V(b, 1), px, py);
    }
    return w;
}

/**
 * @brief Winding number of every query point O.row(i) with respect to the triangle
 * mesh (V, F). Equivalent to igl::winding_number(V, F, O, W), but the query loop is
 * parallelised with threading::parallel_for.
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

    // Build the accelerator once (same as igl::winding_number for triangle meshes).
    igl::WindingNumberAABB<Eigen::Matrix<double, 1, 3>, Eigen::MatrixXd, Eigen::MatrixXi> hier(
        V,
        F);
    hier.grow();

    // hier.winding_number(p) is const and used by igl the same way from parallel_for,
    // so concurrent queries against the shared hierarchy are safe.
    threading::parallel_for(
        threading::range(0, static_cast<size_t>(O.rows())),
        [&](const threading::range& r) {
            for (int o = r.begin(); o < r.end(); ++o) {
                W(o) = hier.winding_number(O.row(o));
            }
        },
        std::max(num_threads, 1));
}

/**
 * @brief Winding number of every query point O.row(i) with respect to the segment soup
 * (V, E), with V and O two-column. Equivalent to igl::winding_number(V, E, O, W), but the
 * query points are processed in parallel chunks.
 *
 * The 2D winding number has no hierarchical accelerator in libigl -- it is a direct
 * O(#E) sweep per query -- so unlike the 3D version above there is nothing to build once.
 *
 * Each point is evaluated with winding_number_2d_point, NOT by handing a chunk to
 * igl::winding_number(V, E, O, W). That call parallelises internally: for F.cols() == 2 it
 * runs igl::parallel_for(O.rows(), ..., 10000), which spawns igl::default_num_threads()
 * threads -- hardware_concurrency() unless IGL_NUM_THREADS says otherwise -- whenever the
 * chunk exceeds 10000 rows. Nesting that inside this parallel_for MULTIPLIED the two thread
 * counts instead of capping them: with num_threads 8 on a 128-core machine, a single model
 * held 1003 live threads and drove the load average to 1000, which is the opposite of what
 * this file exists to do. The 3D overload above never had the problem because it calls the
 * per-point hier.winding_number(), not the whole-matrix entry point.
 */
inline void winding_number_2d(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& E,
    const Eigen::MatrixXd& O,
    Eigen::VectorXd& W,
    int num_threads)
{
    W.setZero(O.rows());
    if (O.rows() == 0 || E.rows() == 0 || V.rows() == 0) return;

    threading::parallel_for(
        threading::range(0, static_cast<size_t>(O.rows())),
        [&](const threading::range& r) {
            for (size_t o = r.begin(); o < r.end(); ++o) {
                const Eigen::Index i = static_cast<Eigen::Index>(o);
                W(i) = winding_number_2d_point(V, E, O(i, 0), O(i, 1));
            }
        },
        std::max(num_threads, 1));
}

} // namespace wmtk::utils
