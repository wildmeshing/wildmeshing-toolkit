
#include "Quadrics.hpp"
#include "TopoOffsetTetMesh.h"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/predicates/predicates.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <optional>
#include <set>
#include <vector>

namespace wmtk::components::topological_offset {

namespace {
/// Diagnostic-only running maximum over a smoothing pass, which is run in parallel.
void atomic_max(std::atomic<int>& target, int value)
{
    int cur = target.load();
    while (value > cur && !target.compare_exchange_weak(cur, value)) {
    }
}
} // namespace

bool TopoOffsetTetMesh::smooth_before(const Tuple& t)
{
    ++m_smooth_trace.attempted;
    // Read before the base call: the base folds "on the bounding box" and "could not be
    // rounded to doubles" into one false, and those mean completely different things.
    const bool on_bbox = !m_vertex_attribute[t.vid(*this)].on_bbox_faces.empty();
    // rounds the vertex and refuses the bounding box
    if (!TetOptimizerMesh::smooth_before(t)) {
        if (on_bbox) {
            ++m_smooth_trace.before_bbox;
        } else {
            ++m_smooth_trace.before_unrounded;
        }
        return false;
    }
    // the input complex must stay exactly where it is: it is the geometry the offset is
    // measured against, not something to be improved
    if (m_vertex_extra[t.vid(*this)].m_is_on_input) {
        ++m_smooth_trace.before_on_input;
        return false;
    }
    return true;
}

bool TopoOffsetTetMesh::smooth_after(const Tuple& t)
{
    // An offset-surface vertex is placed by the quadrics of the input complex's implicit
    // offset field, which is what keeps the offset faithful. Every other vertex is an ordinary
    // interior one and gets the shared two-stage AMIPS smoother.
    if (!get_offset_surface_faces_for_vertex(t).empty()) {
        ++m_smooth_trace.offset_attempted;
        const bool ok = smooth_after_offset_surface(t);
        if (ok) ++m_smooth_trace.offset_accepted;
        return ok;
    }
    ++m_smooth_trace.interior_attempted;
    return TetOptimizerMesh::smooth_after(t);
}

void TopoOffsetTetMesh::log_smooth_trace() const
{
    const auto& s = m_smooth_trace;
    logger().info(
        "\tsmooth trace: attempted {} | before: bbox {}, unrounded {}, on-input {} | "
        "offset path: attempted {} -> accepted {}, no-neighbours {}, on-complex {}, "
        "inverted {}, envelope {} | interior path: attempted {} ({} of them on another region "
        "boundary) ({})",
        s.attempted.load(),
        s.before_bbox.load(),
        s.before_unrounded.load(),
        s.before_on_input.load(),
        s.offset_attempted.load(),
        s.offset_accepted.load(),
        s.offset_no_neighbours.load(),
        s.offset_on_complex.load(),
        s.offset_inverted.load(),
        s.offset_envelope.load(),
        s.interior_attempted.load(),
        s.region_attempted.load(),
        m_smooth_rejects.to_string());
    const int acc = std::max(1, s.offset_accepted.load());
    logger().info(
        "\tsmooth moves: accepted {} of which clamped {} (envelope {}, inverted {}, slid {}) | "
        "err over moved verts: avg {:.6} -> {:.6}, max {:.6} -> {:.6}",
        s.offset_accepted.load(),
        s.offset_clamped.load(),
        s.offset_clamp_env.load(),
        s.offset_clamp_inv.load(),
        s.offset_slid.load(),
        double(s.offset_err_before_nano.load()) / acc * 1e-9,
        double(s.offset_err_after_nano.load()) / acc * 1e-9,
        s.offset_err_max_before_nano.load() * 1e-9,
        s.offset_err_max_after_nano.load() * 1e-9);
}

bool TopoOffsetTetMesh::is_offset_face(const Tuple& f) const
{
    return is_offset_face(f.fid(*this));
}

bool TopoOffsetTetMesh::is_offset_face(const size_t fid) const
{
    return face_is_offset(fid);
}

std::vector<TopoOffsetTetMesh::Tuple> TopoOffsetTetMesh::get_offset_surface_faces_for_vertex(
    const Tuple& t) const
{
    std::vector<Tuple> result;
    std::set<size_t> seen_fids;

    const size_t vid = t.vid(*this);
    for (const size_t tid : get_one_ring_tids_for_vertex(t)) {
        const auto tet_vids = oriented_tet_vids(tid);

        // the 3 faces of the tet incident to vid are those obtained by omitting one of the
        // *other* 3 vertices; omitting vid itself gives the one face that does not contain it
        for (int skip = 0; skip < 4; ++skip) {
            if (tet_vids[skip] == vid) {
                continue;
            }

            std::array<size_t, 3> face_vids;
            int k = 0;
            for (int j = 0; j < 4; ++j) {
                if (j != skip) {
                    face_vids[k++] = tet_vids[j];
                }
            }

            const auto [face_tuple, unused_tid] = tuple_from_face(face_vids);
            const size_t fid = face_tuple.fid(*this);
            if (!seen_fids.insert(fid).second) {
                continue;
            }

            if (is_offset_face(face_tuple)) {
                result.push_back(face_tuple);
            }
        }
    }
    return result;
}

std::array<OffsetSurfaceSample, 4> TopoOffsetTetMesh::offset_surface_samples(const Tuple& f) const
{
    const std::array<Tuple, 3> fv = get_face_vertices(f);
    const Vector3d p0 = m_vertex_attribute[fv[0].vid(*this)].m_posf;
    const Vector3d p1 = m_vertex_attribute[fv[1].vid(*this)].m_posf;
    const Vector3d p2 = m_vertex_attribute[fv[2].vid(*this)].m_posf;
    const Vector3d p_mid = (p0 + p1 + p2) / 3.;

    constexpr double u = 0.1;
    std::array<OffsetSurfaceSample, 4> samples;
    samples[0].point = p_mid;
    samples[0].weight = 1.;
    samples[1].point = (1 - u) * p0 + u * p_mid;
    samples[1].weight = u;
    samples[2].point = (1 - u) * p1 + u * p_mid;
    samples[2].weight = u;
    samples[3].point = (1 - u) * p2 + u * p_mid;
    samples[3].weight = u;

    for (OffsetSurfaceSample& s : samples) {
        s.nearest = m_input_complex_bvh.nearest_point(s.point);
        const Vector3d diff = s.point - s.nearest;
        const double dist = diff.norm();

        s.normal = (dist < 1e-12) ? Vector3d::Zero() : Vector3d(diff / dist);
    }
    return samples;
}

bool TopoOffsetTetMesh::smooth_after_offset_surface(const Tuple& t)
{
    const size_t vid = t.vid(*this);
    const Vector3d p0 = m_vertex_attribute[vid].m_posf;

    const std::vector<Tuple> offset_faces = get_offset_surface_faces_for_vertex(t);
    assert(!offset_faces.empty());

    const std::vector<Tuple> locs = get_one_ring_tets_for_vertex(t);
    auto any_inverted = [&]() {
        for (const Tuple& tet : locs) {
            if (is_inverted(tet)) return true;
        }
        return false;
    };
    // Move the vertex to `target`; if that inverts an incident tet, binary search along the
    // segment from the known-valid p0 to `target` for the furthest point that does not.
    // Returns the fraction of the move that was accepted, or -1 if even the zero step is
    // refused, i.e. p0 itself already inverts a tet. The fraction is diagnostic only -- the
    // position it leaves the vertex at is exactly what it always was.
    auto move_to = [&](const Vector3d& target) -> double {
        set_vertex_position(vid, target);
        if (!any_inverted()) return 1.;

        double lo = 0.; // m_posf = p0 + lo * (target - p0), always valid
        double hi = 1.; // always invalid
        constexpr int max_iters = 10;
        for (int iter = 0; iter < max_iters; ++iter) {
            const double mid = 0.5 * (lo + hi);
            set_vertex_position(vid, p0 + mid * (target - p0));
            if (any_inverted()) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        set_vertex_position(vid, p0 + lo * (target - p0));
        return any_inverted() ? -1. : lo;
    };

    // How far this vertex is off the target distance -- the quantity the offset is optimizing,
    // and the one the acceptance count says nothing about: a move that is "accepted" may have
    // been clamped to a fraction of what was asked for and delivered none of the correction.
    const auto dist_err = [&](const Vector3d& p) {
        return std::abs(
            (p - m_input_complex_bvh.nearest_point(p)).norm() - m_offset_params.target_distance);
    };
    const double e_before = dist_err(p0);

    const auto record = [&](double frac) {
        const double e_after = dist_err(m_vertex_attribute[vid].m_posf);
        // Clamped so a large absolute error on a large model cannot overflow the int the max is
        // accumulated in; 1e-9 units keep the sum atomic. (2D has the same cast without the
        // clamp -- see the 2D notes in .claude/CLAUDE.md.)
        const auto nano = [](double x) { return static_cast<int>(std::min(x * 1e9, 2.0e9)); };
        m_smooth_trace.offset_err_before_nano += nano(e_before);
        m_smooth_trace.offset_err_after_nano += nano(e_after);
        atomic_max(m_smooth_trace.offset_err_max_before_nano, nano(e_before));
        atomic_max(m_smooth_trace.offset_err_max_after_nano, nano(e_after));
        if (frac < 0.99) {
            ++m_smooth_trace.offset_clamped;
            // No probe to attribute the clamp, unlike 2D: inversion is the ONLY constraint on
            // this path in 3D, since the region-boundary envelope is 2D-only.
            ++m_smooth_trace.offset_clamp_inv;
        }
    };

    // Laplacian smoothing, restricted to neighbors that are also on the offset surface --
    // pulling in input-complex or interior neighbors would drag the surface off its shape.
    Vector3d p_laplace = Vector3d::Zero();
    {
        int n_neighs = 0;
        for (const size_t nb : get_one_ring_vids_for_vertex(vid)) {
            if (!m_vertex_extra[nb].m_is_on_offset) continue;
            p_laplace += m_vertex_attribute[nb].m_posf;
            ++n_neighs;
        }
        if (n_neighs == 0) {
            ++m_smooth_trace.offset_no_neighbours;
            return false;
        }
        p_laplace /= n_neighs;
    }

    // Quadric built from 4 offset samples of the input complex per incident offset-surface face
    // (centroid + one near each corner, weighted 1/0.1/0.1/0.1), following Quadrics.cpp's
    // get_triangle_samples_and_area(). Taking several samples rather than just the centroid is
    // what lets the quadric stay feature-aware: near a sharp fold of the input complex, samples
    // on either side pull the per-vertex quadric's minimum toward the fold instead of averaging
    // it away, the same way classic QEM simplification preserves features by summing quadrics
    // from multiple differently-oriented triangles.
    //
    // ONE SHARED TARGET DISTANCE is used for every sample of every incident face, following the
    // reference implementation's vertex-level Quadrics constructor
    // (internal/utils/Quadrics.cpp, the PrimitiveType::Vertex branch), which pools the samples
    // of all incident faces into a single quadric and places every sample's plane at
    // `nearest + dist_avg * normal` for one aggregate distance:
    //
    //     dist_avg = sum(offset_distance * area) / sum(area)  // adapted delta-hat, area-weighted
    //     dist_min = min over samples of the CURRENT distance to the input complex
    //     dist_avg = 0.5 * (dist_avg + dist_min)
    //
    // The first line collapses to target_distance here: distance adaptation (paper Sec. 5.3.1)
    // is deliberately not implemented, so delta-hat is a single global constant and the
    // area-weighted mean of a constant is that constant. The third line -- the damping -- is
    // deliberately NOT applied; see the measurements at dist_target below.
    //
    // Samples are gathered into one list before any quadric is built because the reference's
    // shared distance is an aggregate over all of them, so no plane can be placed until every
    // sample is known. That stays true of any spatially adapted delta-hat, which is why the
    // shape is kept even though the value used below is currently a constant.
    struct WeightedSample
    {
        OffsetSurfaceSample s;
        double area; // of the face the sample came from
    };
    std::vector<WeightedSample> samples;
    samples.reserve(4 * offset_faces.size());
    for (const Tuple& f : offset_faces) {
        const std::array<Tuple, 3> face_verts = get_face_vertices(f);
        const Vector3d p_a = m_vertex_attribute[face_verts[0].vid(*this)].m_posf;
        const Vector3d p_b = m_vertex_attribute[face_verts[1].vid(*this)].m_posf;
        const Vector3d p_c = m_vertex_attribute[face_verts[2].vid(*this)].m_posf;
        const double area = 0.5 * (p_b - p_a).cross(p_c - p_a).norm();

        for (const OffsetSurfaceSample& s : offset_surface_samples(f)) {
            if (s.normal.squaredNorm() < 1e-20) continue; // degenerate: no valid normal
            samples.push_back({s, area});
        }
    }
    if (samples.empty()) {
        ++m_smooth_trace.offset_on_complex;
        return false;
    }

    // THE REFERENCE'S DAMPING IS DELIBERATELY NOT APPLIED. Its third line,
    //
    //     dist_avg = 0.5 * (dist_avg + dist_min)
    //
    // aims the quadric halfway between the target distance and where the surface currently is.
    // Measured on prism (delta = 0.8368, convergence_target 0.0418), final iteration of a 5
    // iteration run, against the undamped `delta` used here:
    //
    //     undamped delta                    max 0.1647  avg 0.0595   worst vertex 20% too far OUT
    //     0.5 * (delta + dist_min)          max 0.3540  avg 0.0459   worst vertex 42% too far IN
    //     0.5 * (delta + this vertex's d)   max 0.3351  avg 0.1218   worst vertex too far out
    //
    // The damping buys average distance error and costs more than twice as much maximum, and
    // max_dist_err is the criterion the run is failing. Keying it to the vertex's own current
    // distance rather than the minimum over samples -- the obvious suspect, since dist_min lets
    // one badly-placed neighbour drag a whole patch's target down -- is worse still on the
    // average, so that was not the mechanism either.
    //
    // The reference can afford the damping because it runs distance adaptation (paper Sec.
    // 5.3.1) first, so its offset starts near-correct and dist_min is already close to
    // delta-hat: there the term is a small correction, not a systematic inward pull. This
    // implementation deliberately skips adaptation, so the offset starts wherever conservative
    // growth left it and the damping becomes a brake on the one quantity that has to converge.
    //
    // NOTE the pooling above is behaviour-neutral on its own: summing one quadric over every
    // sample weighted by w*area is algebraically the same as summing a per-face quadric scaled
    // by area, which is what this built before. It is kept because it is the reference's shape
    // and it is where a spatially adapted delta-hat would attach.
    const double dist_target = m_offset_params.target_distance;

    Quadrics q(0., 0., 0., 0.);
    for (const WeightedSample& ws : samples) {
        const Vector3d target = ws.s.nearest + dist_target * ws.s.normal;
        q += Quadrics(target, ws.s.normal) * (ws.s.weight * ws.area);
    }

    const Vector3d p_optimal = q.solve(p_laplace, m_offset_params.quadrics_svd_threshold);

    const double w = m_offset_params.smooth_quadrics_weight;
    const double u = m_offset_params.smooth_laplacian_weight;
    const Vector3d p_blend = (1 - w - u) * p0 + w * p_optimal + u * p_laplace;

    const Vector3d p_final = p_blend;

    const double frac = move_to(p_final);

    // A SEARCH THAT FINDS NO LEGAL STEP IS A REJECTION, not a zero-length success. This follows
    // the reference implementation (internal/OffsetOptimization.cpp, the smoothing lambda),
    // which backs the move off geometrically (u = 1/2, 1/4, ... 1/1024), accepts the first
    // fraction that clears the inversion invariant, and if all ten fail restores p0 and returns
    // FALSE -- "Vertex position is not optimal but it cannot be moved either".
    //
    // Note what this does and does not change. A PARTIAL fraction is still accepted, exactly as
    // the reference accepts its backed-off u; only a search that reaches nothing at all is
    // refused. What it ends is reporting a vertex the search could not move as an accepted
    // smooth: this returned true unconditionally, so offset_accepted counted vertices that had
    // received none of their correction, and record() folded their unchanged error into the
    // "err over moved verts" averages of vertices that had in fact not moved.
    //
    // move_to() has already restored p0 in both failing cases -- it ends by setting
    // p0 + lo*(target - p0) with lo == 0 -- so there is nothing to undo here.
    //
    // 2D DIFFERS: project_offset_vertex() accepts its clamped result and returns success, which
    // is why .claude/CLAUDE.md tells the reader to watch the `smooth moves:` line rather than
    // the rejection counts to spot a fully-blocked offset. In 3D the rejection counts now carry
    // it directly. This is a deliberate divergence, adopted on the reference's authority rather
    // than forced by the dimension; the two logs still diff field-for-field.
    if (frac <= 0.) {
        // Inversion is the only constraint on this path in 3D, so it is what blocked every
        // fraction -- whether or not p0 itself is inverted as well (frac < 0).
        ++m_smooth_trace.offset_inverted;
        return false;
    }
    record(frac);

#if 0
    // ---------------------------------------------------------------------------------------
    // DISABLED. Candidate-selection placement: implemented, measured, and switched off until the
    // rest of the 3D component is complete, so that the structural work is done against the same
    // placement 2D and the reference implementation use. Re-enable by deleting the #if 0 and the
    // five lines above. Everything below was verified to build and run; the measurements are in
    // the smoother section of .claude/CLAUDE.md. Summary of why it exists and what it cost:
    //
    //   avg_dist_err stopped rising and fell 14-100x on all three test models, and prism
    //   converged in 2 iterations -- but max_norm_dev roughly doubled (box 36.7 -> 75.0, prism
    //   36.4 -> 65.1) because p_proj is not feature-aware, and compute_distance_deviation()
    //   samples at VERTICES, which p_proj satisfies exactly by construction, so the metric stops
    //   being independent of the placement.
    // ---------------------------------------------------------------------------------------
    //
    // The quadric is a LOCAL fit: it assumes the offset surface is resolved relative to
    // target_distance, so that the four samples of one incident face project onto the same
    // piece of the input complex and their tangent planes agree. Where the surface is coarse
    // relative to delta that assumption fails, the planes are mutually inconsistent, and the
    // least-squares point is not near the offset at all. Measured on specific_models/prism
    // (offset edges up to 7*delta), mean distance error over one smoothing pass:
    //
    //     leave the vertex alone   0.0108      quadric x''            0.0263  (2.4x worse)
    //     Laplacian centroid       0.0631      damped blend (shipped) 0.0150  (1.4x worse)
    //
    // -- every placement was worse than not moving, and the quadric beat p0 on only 168 of 2499
    // vertices. So this does NOT follow the paper (Sec. 5.5) in taking x'' unconditionally, and
    // it does not follow the reference implementation in taking the damped blend
    // unconditionally either: both are kept as CANDIDATES and the one that actually lowers
    // |dist - delta| wins, with p0 in the set so a pass can never make a vertex worse. That
    // never-degrade rule is 2D's, from minimize_distance_along_tangent().
    //
    // p_proj is 2D's placement (project_offset_vertex): step from the nearest point on the
    // input complex out along the direction the vertex already lies, by exactly delta. It
    // satisfies dist == delta by construction with no resolution assumption, but it is not
    // feature-aware -- near a sharp fold it picks whichever side the BVH returns. The quadric
    // is feature-aware and needs resolution. They fail in opposite regimes, which is the whole
    // reason for choosing between them per vertex rather than picking one globally.
    //
    // Ordered blend, quadric, projection, and compared with <=, so a tie is won by the earlier
    // entry: the two that carry tangential smoothing beat the pure projection when distance
    // cannot tell them apart, and any of them beats standing still. That is what keeps triangle
    // shape being improved on the patches where distance is already satisfied.
    std::array<Vector3d, 3> candidates{{p_blend, p_optimal, p_blend}};
    int n_candidates = 2;
    {
        const Vector3d nearest0 = m_input_complex_bvh.nearest_point(p0);
        const Vector3d diff0 = p0 - nearest0;
        const double d0 = diff0.norm();
        if (d0 > 1e-12) { // on the complex: no direction to offset along, skip this candidate
            candidates[2] = nearest0 + m_offset_params.target_distance * (diff0 / d0);
            n_candidates = 3;
        }
    }

    Vector3d best_p = p0;
    double best_e = e_before;
    double best_frac = 0.; // no candidate accepted => the vertex does not move at all
    int best_i = -1;
    bool pre_inverted = false;
    for (int i = 0; i < n_candidates; ++i) {
        // Evaluate the position the vertex would actually END UP at, which is the candidate
        // clamped back for inversion -- not the candidate as requested. A clamped move delivers
        // a fraction of the correction, and that fraction is what has to be compared.
        const double frac = move_to(candidates[i]);
        if (frac < 0.) {
            pre_inverted = true;
            continue; // p0 itself inverts; nothing on this segment is legal
        }
        const Vector3d p_try = m_vertex_attribute[vid].m_posf;
        const double e_try = dist_err(p_try);
        if (e_try <= best_e) {
            best_e = e_try;
            best_p = p_try;
            best_frac = frac;
            best_i = i;
        }
    }
    // Every candidate above was verified non-inverting at the position recorded for it, and
    // nothing but this vertex has moved since, so restoring the winner cannot invert.
    set_vertex_position(vid, best_p);

    if (pre_inverted) {
        // p0 itself inverts a tet, so no fraction of any move is legal and the vertex is stuck
        // where it is. Not a rejection here -- invariants(), below, is what rolls it back.
        ++m_smooth_trace.offset_inverted;
    }
    if (best_i > 0) {
        // A candidate other than the primary damped blend won. The 3D counterpart of 2D's
        // offset_slid, which counts the same thing there: the fallback placement recovered
        // ground the primary one could not.
        ++m_smooth_trace.offset_slid;
    }
    record(best_frac);
#endif // DISABLED candidate-selection placement

    // Any remaining inversion (from the binary search's finite precision) is caught by
    // invariants(), called right after this by TetMesh::smooth_vertex.
    return true;
}

} // namespace wmtk::components::topological_offset
