
#include "TopoOffsetTetMesh.h"

#include <wmtk/optimization/solver.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>

#include <cmath>
#include <set>
#include <vector>

namespace wmtk::components::topological_offset {

bool TopoOffsetTetMesh::smooth_before(const Tuple& t)
{
    ++m_smooth_trace.attempted;
    const size_t vid = t.vid(*this);
    ++m_move_stats[size_t(vertex_class(vid))].attempted;

    // THE BASE'S smooth_before MINUS ITS BOUNDING-BOX REFUSAL, which is why this does not call
    // it. The base rounds the vertex and then refuses outright any vertex with a non-empty
    // on_bbox_faces, i.e. it FREEZES the domain wall. That is a stronger constraint than any
    // envelope and it is the second half of what made the wall a degeneracy trap: a tet resting
    // on it had its longest edge unsplittable (EdgeSplittingTet.cpp, now lifted) AND its three
    // wall vertices unmovable, so nothing could relieve the sliver from either side.
    //
    // Here the wall is a region boundary held in m_envelope like every other one, so its
    // vertices are smoothed and the containment check decides whether the move survives --
    // exactly the contract the input complex already gets. What keeps the wall a wall is that
    // check, not immobility.
    //
    // Rounding still has to happen, and its failure still refuses the move; the two outcomes the
    // base folded into one false are separated here because they mean different things.
    const bool rounded_now = round(t);
    if (!m_vertex_attribute[vid].m_is_rounded && !rounded_now) {
        ++m_move_stats[size_t(vertex_class(vid))].refused_before;
        ++m_smooth_trace.before_unrounded;
        return false;
    }
    // AN INPUT-COMPLEX VERTEX IS SMOOTHED, exactly as TetWild smooths a surface vertex.
    //
    // This used to refuse it: "the input complex must stay exactly where it is, it is the
    // geometry the offset is measured against". The second half is true and the first half does
    // not follow from it. What the offset is measured against is m_input_complex_bvh and
    // m_offset_potential, both built ONCE from the input as loaded and never rebuilt -- so the
    // distance field does not care where the mesh elements representing the complex end up. All
    // freezing them bought was a fixed set of vertices; what it cost is that every element
    // touching the input complex was unimprovable, and the faces pinned between two frozen
    // vertices could never reach stop_energy.
    //
    // TetWild's mechanism is the right one and it is already here: the vertex is smoothed by the
    // shared solver and held inside its tags' boundary envelopes -- the intersection of them at
    // a junction -- which smoothing_containment_envelope() answers for it, with the projection
    // step smoothing_mode = "projected" performs. That is a tolerance the input surface may
    // drift within, which is exactly what TetWild's own input surface gets -- not a licence to
    // move anywhere.
    //
    // Measured before this change: 14758 of 229276 smoothing attempts refused here.

    // PHASE B PLACES THE OFFSET SURFACE AND MOVES NOTHING ELSE.
    //
    // Phase B's objective is the offset potential, which only offset-only vertices carry. Every
    // other vertex used to be smoothed here by AMIPS plus an envelope pull -- that is quality
    // work, and quality work belongs to Phase A, which runs every round with the full TetWild
    // pass set. Doing it again in B bought nothing the next A would not redo, and it did it
    // against a mesh B was mid-way through re-placing.
    //
    // Dropping AMIPS but keeping those vertices smoothed is NOT the alternative: with only an
    // envelope pull and no shape term their solve is ill-posed the same way the rank-1
    // distance-only offset solve was, so they are refused outright instead.
    if (m_phase == OptPhase::B) {
        const auto& ve = m_vertex_extra[vid];
        if (!ve.m_is_on_offset || ve.m_is_on_input || !m_offset_potential) {
            ++m_move_stats[size_t(vertex_class(vid))].refused_before;
            ++m_smooth_trace.before_phase_b_not_offset;
            return false;
        }
    }
    return true;
}

/**
 * @brief Every vertex goes through the shared smoother. Nothing is dispatched anywhere else.
 *
 * This used to fork: an offset-surface vertex was placed by smooth_after_offset_surface(), a
 * blend of a quadric fit to sampled offset planes and the offset-surface Laplacian, guarded by
 * its own hand-rolled inversion bisection; everything else got the shared two-stage AMIPS
 * smoother. The fork existed only because the Euclidean distance to the input complex has no
 * usable gradient, so the offset could not be an ENERGY -- and the price was that the one
 * surface the whole component exists to place was the one surface that bypassed the shared
 * smoother's line search, its exact inversion test and its accept checks.
 *
 * The smooth offset potential removes the reason: the offset is now the term
 * smooth_offset_vertex_backtracking() drives to zero directly, by a 1-D root find rather than
 * a blended minimisation. See OffsetPotential.
 */
bool TopoOffsetTetMesh::smooth_after(const Tuple& t)
{
    const size_t vid = t.vid(*this);
    if (m_vertex_extra[vid].m_is_on_offset) {
        ++m_smooth_trace.offset_attempted;
    } else {
        ++m_smooth_trace.interior_attempted;
    }
    // HOW FAR IT ACTUALLY GOT. Captured around the base call because that is what runs the
    // solver, the projection and every accept gate; on any refusal the base leaves the vertex
    // where it started, so a false answer contributes nothing. See MoveStats.
    const Vector3d before = m_vertex_attribute[vid].m_posf;
    // Both halves of the discriminator have to be read BEFORE the move: how many tracked faces
    // the containment check will have to work with, and where the vertex started relative to the
    // walls it claims. See WallMoveStats.
    const bool on_wall = !m_vertex_attribute[vid].on_bbox_faces.empty();
    const bool zero_tracked = on_wall && get_surface_faces_for_vertex(vid).faces().empty();
    const double dev_before = on_wall ? wall_offplane_deviation(vid) : 0.;

    // TWO PHASES, TWO SMOOTHERS, NO AMIPS IN B.
    //
    // Phase B places offset-only vertices with the offset potential alone: a 1-D Newton root
    // find on Phi(x) = target along the gradient, backtracked into the element by bisection if
    // the root would invert the ring. smooth_before() has already refused every other vertex in
    // this phase, so the branch below is exhaustive.
    //
    // WHY NOT THE SHARED SOLVE. Blending w_amips * AMIPS into this vertex's objective leaves it
    // resting a w_amips-proportional distance off the level set (the header on
    // smooth_offset_vertex_backtracking has the measurement) -- the at-vertex wall. And AMIPS
    // alone cannot be dropped from the shared solve either: the offset energy's Hessian is
    // 2*w*g*g^T, rank 1, with a 2-D nullspace in the level set's tangent plane, so a 3-D
    // minimize of it is ill-posed. The 1-D root find sidesteps both -- it solves the only
    // direction the potential determines, and asks nothing of the two it does not.
    const bool ok = (m_phase == OptPhase::B) ? smooth_offset_vertex_backtracking(t)
                                             : TetOptimizerMesh::smooth_after(t);
    if (!ok) {
        return false;
    }

    // Phi's lower strata -- wires and isolated points -- used to need a dedicated point check
    // here, because the base's containment walks tracked FACES and a wire vertex has none
    // carrying its geometry. The per-tag envelopes closed that hole structurally: a wire or
    // isolated point only arises where two or more selected tags meet, so its vertex carries
    // several boundary-mask bits and smoothing_containment_envelope() hands the base an
    // IntersectionEnvelope whose is_outside(face) test -- and, before that, the pull toward the
    // most-violated member tube -- holds it at the junction.
    m_move_stats[size_t(vertex_class(vid))].add((m_vertex_attribute[vid].m_posf - before).norm());
    if (on_wall) {
        m_wall_moves.note(zero_tracked, dev_before, wall_offplane_deviation(vid));
    }
    return true;
}

bool TopoOffsetTetMesh::smooth_offset_vertex_backtracking(const Tuple& t)
{
    const size_t vid = t.vid(*this);
    const auto locs = get_one_ring_tets_for_vertex(t);
    if (locs.empty()) {
        return false;
    }

    // Same entry guard the shared smoother applies: a one-ring already inverted in floats has
    // no valid segment to search, and the exact predicate below would reject every candidate.
    for (const Tuple& loc : locs) {
        if (is_inverted_f(loc)) {
            ++m_smooth_rejects.already_inverted;
            return false;
        }
    }

    const Vector3d x_orig = m_vertex_attribute[vid].m_posf;

    // ALONG THE NORMAL, ONTO THE LEVEL SET -- a 1-D root find on Phi(x) = c, NOT a 3-D
    // minimisation of the offset energy.
    //
    // The distinction is the whole point. OffsetEnergy's Gauss-Newton Hessian is
    // 2 * w * g * g^T with g = grad Phi: RANK ONE. It constrains the normal direction and
    // leaves a two-dimensional nullspace in the level set's tangent plane, so minimising it
    // alone does not determine where the vertex goes -- it fixes one of three degrees of
    // freedom and lets the other two drift. Measured on prism at 5% with AMIPS removed and the
    // full 3-D solve kept: round 1 came out at 19.31x tolerance against 8.6-9.0x with AMIPS
    // present, 1772 faces carrying an over-tolerance vertex against 0, and envelope refusals up
    // an order of magnitude -- vertices sliding along the surface, exactly the nullspace.
    //
    // What the offset term actually says is "move along the normal until Phi = c", and that is
    // well posed on its own. Each iteration takes the Newton step for the scalar equation,
    // -(Phi - c) / |grad Phi|^2 * grad Phi, which IS the normal direction; the tangential
    // components never enter, so there is no nullspace to regularise and no AMIPS needed. Phi
    // is nonlinear, so a few iterations are taken and the last one is kept.
    constexpr int kRootFindIters = 8;
    Vector3d x = x_orig;
    for (int it = 0; it < kRootFindIters; ++it) {
        const double r = m_offset_potential->value(x) - m_offset_potential->target_level();
        const Vector3d g = m_offset_potential->gradient(x);
        const double g2 = g.squaredNorm();
        if (!(g2 > 0.) || !std::isfinite(r)) {
            break; // no usable normal here; keep the best point so far
        }
        const Vector3d step = -(r / g2) * g;
        if (!step.allFinite()) {
            break;
        }
        x += step;
        // Converged when the step is negligible against the offset distance itself.
        if (step.norm() <= 1e-12 * std::max(m_offset_params.target_distance, 1e-16)) {
            break;
        }
    }
    const Vector3d x_new = x;
    if (!x_new.allFinite()) {
        set_vertex_position(vid, x_orig);
        ++m_smooth_rejects.inverted;
        return false;
    }

    // Place a candidate and report whether any incident tet inverted. Exact, on the rational
    // position, exactly as the shared smoother's accept test is.
    const auto inverts = [&](const Vector3d& p) {
        set_vertex_position(vid, p);
        for (const Tuple& loc : locs) {
            if (is_inverted(loc)) return true;
        }
        return false;
    };

    if (inverts(x_new)) {
        // BISECT THE SEGMENT, keeping the invariant lo = valid, hi = invalid. s = 1 is known
        // invalid (just tested) and s = 0 is known valid (the entry guard), so this converges
        // UP to the constraint from below rather than capping at the midpoint -- the vertex
        // gets as close to the level set as the one-ring allows.
        //
        // The step count is OURS and not project_line_search_nested_steps, which defaults to 0
        // -- a max(1, that) gave a single probe at s = 0.5 and discarded the move whenever that
        // one point inverted, which is not a line search at all (measured: 893 inversion
        // refusals per pass). 30 halvings take s below 1e-9 of the segment, so a refusal here
        // means no admissible motion exists rather than that the search gave up.
        constexpr int kBisectionSteps = 30;
        double lo = 0., hi = 1.;
        Vector3d best = x_orig;
        bool found = false;
        for (int j = 0; j < kBisectionSteps; ++j) {
            const double mid = 0.5 * (lo + hi);
            const Vector3d cand = x_orig + mid * (x_new - x_orig);
            if (inverts(cand)) {
                hi = mid;
            } else {
                lo = mid;
                best = cand;
                found = true;
            }
        }
        if (!found) {
            // Not even the smallest step is admissible: leave the vertex where it started.
            set_vertex_position(vid, x_orig);
            ++m_smooth_rejects.inverted;
            return false;
        }
        // A SAFETY MARGIN, but ONLY on this path. `best` sits at s = lo, which after 30
        // halvings is within ~1e-9 of the first inverting point -- valid by the orientation
        // test and numerically degenerate in every other sense, which is what fed near-zero
        // volume tets to Phase A and (while the criterion still assembled AMIPS) produced the
        // inf gradients that hung Phase B.
        //
        // The margin is deliberately NOT applied when the root is reachable: that case never
        // enters this branch at all, so a vertex whose true minimum lies inside its one-ring
        // still lands exactly on the level set. Backing off is a concession to the constraint,
        // and it is only owed where the constraint actually bound.
        //
        // The valid set along the segment need not be a single interval -- tet orientation is
        // a cubic in s -- so the retreated point is re-tested rather than assumed, and falls
        // back to `best` if it somehow inverts.
        constexpr double kBacktrackMargin = 0.8;
        const Vector3d retreated = x_orig + kBacktrackMargin * lo * (x_new - x_orig);
        // Either way set the position explicitly rather than relying on where inverts() left it.
        set_vertex_position(vid, inverts(retreated) ? best : retreated);
    }

    // The one-ring's stored qualities are now stale; the split/collapse/swap passes read them.
    for (const Tuple& loc : locs) {
        set_cell_quality(loc.tid(*this), get_quality(loc));
    }
    ++m_smooth_rejects.accepted;
    return true;
}

void TopoOffsetTetMesh::log_smooth_trace() const
{
    const auto& s = m_smooth_trace;
    logger().info(
        "\tsmooth trace: attempted {} | before: bbox {}, unrounded {}, on-input {}, "
        "phase-B non-offset {} | offset-surface vertices {}, interior {} ({})",
        s.attempted.load(),
        s.before_bbox.load(),
        s.before_unrounded.load(),
        s.before_on_input.load(),
        s.before_phase_b_not_offset.load(),
        s.offset_attempted.load(),
        s.interior_attempted.load(),
        m_smooth_rejects.to_string());
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

} // namespace wmtk::components::topological_offset
