#pragma once

// A local copy of libigl's winding-number evaluation, but with the per-query
// parallelism driven by wmtk's own threading framework instead of igl::parallel_for.
// This means the winding number honours the requested `num_threads` (like every other
// parallel section in wmtk) rather than always grabbing all hardware cores.
//
// The 3D algorithm is igl's hierarchy (WindingNumberHierarchy, a deterministic copy of
// igl::WindingNumberAABB), so it is as fast as igl::winding_number; unlike igl's, its
// result does not depend on rand() or on the number of threads, to the last bit.

#include <Eigen/Core>

#include <wmtk/utils/WindingNumberHierarchy.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/remove_unreferenced.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <wmtk/threading/dynamic_parallel_for.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

namespace wmtk::utils {

/// Query points per chunk handed to a thread by the winding-number evaluations below.
inline constexpr size_t kWindingNumberChunk = 256;

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
 * mesh (V, F). igl::winding_number(V, F, O, W), but deterministic (see
 * WindingNumberHierarchy) and with the query loop parallelised by wmtk.
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

    // Build the accelerator once (as igl::winding_number does for triangle meshes).
    const WindingNumberHierarchy hier(V, F);

    // hier.winding_number(p) is const and touches no shared state, so concurrent queries
    // against the one hierarchy are safe.
    //
    // Handed out in chunks on demand rather than one fixed slice per thread: a query costs
    // far more near the surface, where the hierarchy is descended to its leaves, than away
    // from it, and the queries of a mesh are not in random order -- the cells of an
    // arrangement cluster by location -- so fixed slices left most threads waiting on the one
    // that drew the surface. Each query is evaluated on its own, so W does not depend on the
    // schedule or the number of threads.
    threading::dynamic_parallel_for(
        static_cast<size_t>(O.rows()),
        std::max(num_threads, 1),
        kWindingNumberChunk,
        [&](const size_t begin, const size_t end) {
            for (size_t o = begin; o < end; ++o) {
                W(o) = hier.winding_number(O.row(o));
            }
        });
}

/**
 * @brief The inside test of every winding-number tag and filter: strictly above 1/2.
 *
 * A point on the surface -- the barycenter of a flat tet lying in it, say -- has winding number
 * 1/2 and is not inside. tetwild, triwild and simwild all decide inside this way.
 */
inline bool winding_number_inside(double w)
{
    return w > 0.5;
}

/// Whether any entry of W is inside (winding_number_inside).
inline bool any_winding_number_inside(const Eigen::VectorXd& W)
{
    return (W.array() > 0.5).any();
}

/**
 * @brief Orients the winding number W of a surface (or curve) that may be wound inside out:
 * when none of the query points is inside, the surface is taken as inside out and W negated.
 * Returns whether it was.
 *
 * Negating stands in for evaluating the reversed surface again, which is what this used to do:
 * reversing every face (segment) negates its solid (signed) angle, so the winding number changes
 * sign, up to rounding.
 */
inline bool orient_winding_number(Eigen::VectorXd& W)
{
    if (W.size() == 0 || any_winding_number_inside(W)) return false;
    W = -W;
    return true;
}

/**
 * @brief Winding number of every query point O.row(i) with respect to each group of faces of
 * (V, F) on its own: W(i, g) is the winding number with respect to the faces f with
 * group[f] == g, for g in [0, n_groups).
 *
 * The winding number is additive over the faces, so W.rowwise().sum() is the winding number
 * with respect to the whole mesh. Summing is also the cheaper way to get it when the groups are
 * closed surfaces, as separate input files usually are: a closed surface's hierarchy answers 0
 * for a point outside its bounding box without looking at a face, while one hierarchy over all
 * of them mixes the surfaces in its nodes, and nodes that cut through surfaces have a boundary
 * that every query has to sum over. On the 11 surfaces of an EMI-Meshing dataset the whole mesh
 * took 3x as long as the 11 groups together.
 */
inline void winding_number_by_group(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const std::vector<int>& group,
    int n_groups,
    const Eigen::MatrixXd& O,
    Eigen::MatrixXd& W,
    int num_threads)
{
    assert(group.size() == size_t(F.rows()));
    W.setZero(O.rows(), n_groups);
    std::vector<std::vector<int>> faces(n_groups);
    for (Eigen::Index f = 0; f < F.rows(); ++f) {
        assert(group[f] >= 0 && group[f] < n_groups);
        faces[group[f]].push_back(int(f));
    }
    for (int g = 0; g < n_groups; ++g) {
        if (faces[g].empty()) continue;
        Eigen::MatrixXi Fg(faces[g].size(), 3);
        for (size_t i = 0; i < faces[g].size(); ++i) Fg.row(i) = F.row(faces[g][i]);
        // Only the group's own vertices, so that its hierarchy's root box is the group's.
        Eigen::MatrixXd Vg;
        Eigen::MatrixXi Fg_compact;
        Eigen::VectorXi I;
        igl::remove_unreferenced(V, Fg, Vg, Fg_compact, I);
        Eigen::VectorXd w;
        winding_number(Vg, Fg_compact, O, w, num_threads);
        W.col(g) = w;
    }
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

    // In chunks on demand, as the 3D winding_number: see there.
    threading::dynamic_parallel_for(
        static_cast<size_t>(O.rows()),
        std::max(num_threads, 1),
        kWindingNumberChunk,
        [&](const size_t begin, const size_t end) {
            for (size_t o = begin; o < end; ++o) {
                const Eigen::Index i = static_cast<Eigen::Index>(o);
                W(i) = winding_number_2d_point(V, E, O(i, 0), O(i, 1));
            }
        });
}

} // namespace wmtk::utils
