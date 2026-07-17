#pragma once

// A local copy of libigl's winding-number evaluation, but with the per-query
// parallelism driven by wmtk's own threading framework (task_arena + parallel_for)
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

namespace wmtk::utils {

/**
 * @brief Winding number of every query point O.row(i) with respect to the triangle
 * mesh (V, F). Equivalent to igl::winding_number(V, F, O, W), but the query loop is
 * parallelised with wmtk::threading::parallel_for inside a
 * wmtk::threading::task_arena(num_threads).
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
    wmtk::threading::task_arena arena(std::max(num_threads, 1));
    arena.execute([&]() {
        wmtk::threading::parallel_for(
            wmtk::threading::blocked_range<int>(0, static_cast<int>(O.rows())),
            [&](wmtk::threading::blocked_range<int> r) {
                for (int o = r.begin(); o < r.end(); ++o) {
                    W(o) = hier.winding_number(O.row(o));
                }
            });
    });
}

} // namespace wmtk::utils
