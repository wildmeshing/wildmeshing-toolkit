
#include "TopoOffsetTetMesh.h"

#include <wmtk/optimization/SmoothVertex.hpp>
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

    // The base's smooth_before minus its bounding-box refusal, which is why this does not call it:
    // the base freezes every vertex on the domain wall. Here the wall is a region boundary held in
    // m_envelope like any other, so its vertices are smoothed and the containment check decides
    // whether the move survives. What keeps the wall a wall is that check, not immobility.
    // Rounding still has to happen, and its failure still refuses the move.
    const bool rounded_now = round(t);
    if (!m_vertex_attribute[vid].m_is_rounded && !rounded_now) {
        ++m_move_stats[size_t(vertex_class(vid))].refused_before;
        ++m_smooth_trace.before_unrounded;
        return false;
    }
    // An input-complex vertex is smoothed, as TetWild smooths a surface vertex. The offset is
    // measured against m_input_complex_bvh and m_offset_potential, both built once from the input
    // as loaded and never rebuilt, so the field does not care where the elements representing the
    // complex end up; the vertex is held inside its tags' boundary envelopes instead, which is a
    // tolerance it may drift within rather than a licence to move anywhere.

    // Phase B runs in two ordered sub-sweeps (see PhaseBSub): place every offset-surface vertex,
    // then relax the background under the surface those placements just defined. Relaxing against
    // the previous pass's surface wastes the work, and interleaving makes each vertex's one-ring a
    // moving target within its own sweep, so each sub-sweep refuses the other's class here.
    //
    // Phase B releases the offset envelope and takes no containment responsibility, so a vertex an
    // envelope should hold has nobody to hold it: an offset one throws rather than being silently
    // skipped off the level set, and a background one is skipped (see the stencil at the end of
    // smooth_interior_vertex_phase_b()). A vertex on the offset surface is admitted whatever its
    // other flags say -- m_is_on_region accretes through splits and collapses, and the genuinely
    // contradictory case is geometric and already refused by check_no_vertex_on_both_surfaces().
    if (m_phase == OptPhase::B) {
        const bool is_offset = m_vertex_extra[vid].m_is_on_offset && m_offset_potential;
        const bool enveloped = vertex_is_on_region(vid);

        if (m_phase_b_sub == PhaseBSub::Offset) {
            if (!is_offset) {
                ++m_move_stats[size_t(vertex_class(vid))].refused_before;
                ++m_smooth_trace.before_phase_b_not_offset;
                return false;
            }
            if (enveloped) {
                log_and_throw_error(
                    "Phase B: offset-surface vertex {} is also held by an envelope (on a tag "
                    "region boundary or the domain wall). Placing it needs the envelope's pull "
                    "and containment, which this scheme does not implement yet; it is a hard "
                    "error rather than a skip so the surface cannot silently be left off the "
                    "level set. Test on an offset that lies inside no envelope.",
                    vid);
            }
            return true;
        }

        // PhaseBSub::Background
        if (is_offset || vertex_class(vid) != VClass::Interior) {
            ++m_move_stats[size_t(vertex_class(vid))].refused_before;
            ++m_smooth_trace.before_phase_b_not_offset;
            return false;
        }
        if (enveloped) {
            // See the stencil in smooth_interior_vertex_phase_b().
            ++m_move_stats[size_t(vertex_class(vid))].refused_before;
            ++m_smooth_trace.before_phase_b_enveloped_background;
            return false;
        }
    }
    return true;
}

/**
 * @brief Every vertex goes through the shared smoother. Nothing is dispatched anywhere else.
 *
 * The offset is a term smooth_offset_vertex_backtracking() drives to zero directly, so the one
 * surface the component exists to place goes through the same line search, exact inversion test
 * and accept checks as every other vertex. See OffsetPotential.
 */
bool TopoOffsetTetMesh::smooth_after(const Tuple& t)
{
    const size_t vid = t.vid(*this);
    if (m_vertex_extra[vid].m_is_on_offset) {
        ++m_smooth_trace.offset_attempted;
    } else {
        ++m_smooth_trace.interior_attempted;
    }
    // Captured around the base call, which is what runs the solver, the projection and every
    // accept gate; on a refusal the base leaves the vertex where it started. See MoveStats.
    const Vector3d before = m_vertex_attribute[vid].m_posf;
    // Both halves of the discriminator must be read before the move: how many tracked faces the
    // containment check will have to work with, and where the vertex started relative to the walls
    // it claims. See WallMoveStats.
    const bool on_wall = !m_vertex_attribute[vid].on_bbox_faces.empty();
    const bool zero_tracked = on_wall && get_surface_faces_for_vertex(vid).faces().empty();
    const double dev_before = on_wall ? wall_offplane_deviation(vid) : 0.;

    // Two phases; in B, two solves. Phase B places offset-only vertices with the offset potential
    // alone, backtracked into the element if the step would invert the ring, and minimises its
    // interior vertices' one-ring AMIPS (smooth_interior_vertex_phase_b). smooth_before() has
    // already refused everything else in this phase, so the branch below is exhaustive. Why the
    // offset placement carries no AMIPS term: see smooth_offset_vertex_backtracking().
    const bool ok = (m_phase == OptPhase::B)
                        ? (m_vertex_extra[vid].m_is_on_offset ? smooth_offset_vertex_backtracking(t)
                                                              : smooth_interior_vertex_phase_b(t))
                        : TetOptimizerMesh::smooth_after(t);
    if (!ok) {
        return false;
    }

    // Phi's lower strata -- wires and isolated points -- need no dedicated point check, even
    // though the base's containment walks tracked faces and a wire vertex carries none: such a
    // vertex only arises where two or more selected tags meet, so it carries several boundary-mask
    // bits and smoothing_containment_envelope() hands the base an IntersectionEnvelope that holds
    // it at the junction.
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
            ++m_phase_b_constrained; // cannot even attempt its minimum
            return false;
        }
    }

    const Vector3d x_orig = m_vertex_attribute[vid].m_posf;

    // Minimise E = (Phi - c)^2 along the field normal by a damped Gauss-Newton step -- not a plain
    // root find on Phi = c, and not a free 3-D minimisation. E's Gauss-Newton Hessian 2*w*g*g^T
    // (g = grad Phi) is rank one, leaving a nullspace in the level set's tangent plane, so a 3-D
    // solve lets vertices slide along the surface. Stepping along g keeps the tangential
    // components out entirely, which is why no AMIPS term is needed to regularise them.
    //
    // The damping is what makes this safe where no root exists. Undamped, -(r/|g|^2) g divides by
    // a |g| that vanishes at exactly the stationary points of E with r != 0 -- every pinch, where
    // two objects closer than 2 x target_distance have no level set between them and the surface
    // belongs on the ridge where grad Phi cancels by symmetry. Levenberg-Marquardt's
    // -(r / (|g|^2 + mu)) g sends the step to zero there instead of to infinity; mu carries units
    // of |grad Phi|^2 and so is held relative to the potential's own level-set slope, shrinking on
    // a step that lowers E and growing on one that does not.
    //
    // The visit stops at vertex_grad_tol_rel of |grad E| at its own start: a per-vertex tolerance,
    // separate from the run's convergence bar. The iteration cap guards a cycling Newton.
    constexpr int kRootFindIters = 50;
    constexpr int kLmTries = 8; ///< damping increases per iteration before giving up
    constexpr double kLmInit = 1e-8; ///< mu/s^2 at entry: effectively undamped Gauss-Newton
    constexpr double kLmMin = 1e-12;
    constexpr double kLmMax = 1e8;
    const double vertex_tol_rel = m_offset_params.vertex_grad_tol_rel;
    const double s_ref = m_offset_potential->level_set_slope();
    const double mu_scale = (s_ref > 0. && std::isfinite(s_ref)) ? s_ref * s_ref : 1.;
    double mu_rel = kLmInit;
    double e_grad_entry = -1.; // |grad E| at the visit's start; captured on the first iteration
    Vector3d x = x_orig;
    for (int it = 0; it < kRootFindIters; ++it) {
        const double r = m_offset_potential->value(x) - m_offset_potential->target_level();
        const Vector3d g = m_offset_potential->gradient(x);
        const double g2 = g.squaredNorm();
        if (!std::isfinite(r)) break;
        if (!(g2 > 0.)) {
            // grad E = 2 r g = 0 with g = 0: a stationary point of E, which under the damped step
            // is a legitimate place to stop -- it is the pinch minimum.
            break;
        }
        const double e_grad = 2. * std::abs(r) * std::sqrt(g2);
        if (e_grad_entry < 0.) {
            e_grad_entry = e_grad;
            if (!(e_grad_entry > 0.)) break; // already at the minimum
        }
        if (e_grad <= vertex_tol_rel * e_grad_entry) {
            break;
        }
        // Damped step, with mu raised until E actually decreases. |r| is monotone in E, so the
        // acceptance test is on |r| directly.
        bool advanced = false;
        Vector3d step = Vector3d::Zero();
        for (int t = 0; t < kLmTries; ++t) {
            step = -(r / (g2 + mu_rel * mu_scale)) * g;
            if (!step.allFinite()) break;
            const Vector3d x_try = x + step;
            const double r_try =
                m_offset_potential->value(x_try) - m_offset_potential->target_level();
            if (std::isfinite(r_try) && std::abs(r_try) < std::abs(r)) {
                x = x_try;
                mu_rel = std::max(mu_rel * 0.1, kLmMin);
                advanced = true;
                break;
            }
            mu_rel = std::min(mu_rel * 10., kLmMax);
        }
        if (!advanced) {
            break; // no admissible damping lowers E from here
        }
        // A step this small cannot move the gradient test above; stop burning evaluations.
        if (step.norm() <= 1e-12 * std::max(m_offset_params.target_distance, 1e-16)) {
            break;
        }
    }
    const Vector3d x_new = x;
    if (!x_new.allFinite()) {
        set_vertex_position(vid, x_orig);
        ++m_smooth_rejects.inverted;
        ++m_phase_b_constrained; // no admissible motion toward its minimum
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
        // The minimum this visit solved for lies outside what the one-ring admits -- the
        // count the pass loop's backtrack-free exit watches.
        ++m_phase_b_constrained;
        // Bisect the segment keeping the invariant lo = valid, hi = invalid. s = 1 is known
        // invalid (just tested) and s = 0 known valid (the entry guard), so this converges up to
        // the constraint from below and the vertex gets as close to the level set as the one-ring
        // allows. The step count is deliberately ours rather than
        // project_line_search_nested_steps, which defaults to 0; 30 halvings take s below 1e-9 of
        // the segment, so a refusal here means no admissible motion exists.
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
        // A safety margin on what the ring allows, and only on this path. `lo` after 30 halvings
        // sits within ~1e-9 of the first inverting configuration: valid by the orientation
        // predicate, numerically degenerate in every other sense. A vertex whose minimum is
        // reachable inside its ring never enters this branch and still lands exactly on the level
        // set, so backing off is owed only where the constraint actually bound. The valid set
        // along the segment need not be a single interval -- tet orientation is a cubic in s --
        // so the retreated point is re-tested rather than assumed.
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

bool TopoOffsetTetMesh::smooth_interior_vertex_phase_b(const Tuple& t)
{
    // Pure one-ring AMIPS, solved to this vertex's own minimum; see the declaration. Unit weight,
    // since Newton cancels any positive scale, and no envelope terms: an interior vertex carries
    // no surface, so the shared smoother's pull and containment branches are bypassed.
    optimization::SmoothVertexOptions opts;
    opts.w_amips = 1.0;
    opts.w_envelope = 0.0;
    opts.s_amips = 1.0;
    opts.s_envelope = 0.0;
    opts.two_stage = false;
    // The veto the 3D pipeline runs everywhere: a Newton descent from the current position should
    // not worsen the worst incident element, so this only refuses line-search accidents.
    // Do not drop it; measured worse -- see git history of this file.
    opts.quality_veto = m_params.smooth_quality_veto;

    auto& solver = m_phase_b_solver.local();
    if (!solver) {
        // The base solver's twin, with the stopping rule this phase is about: polysolve's
        // rel_grad_norm_tol is gradNorm / initial_grad_norm, so the solve stops at
        // vertex_grad_tol_rel of the visit's initial gradient, and the iteration budget is deep
        // enough that the tolerance is what fires.
        polysolve::json params = optimization::basic_nonlinear_solver_params;
        params["max_iterations"] = 50;
        params["rel_grad_norm_tol"] = m_offset_params.vertex_grad_tol_rel;
        solver = polysolve::nonlinear::Solver::create(
            params,
            optimization::basic_linear_solver_params,
            1,
            opt_logger());
    }
    return optimization::smooth_vertex_3d(*this, t, opts, solver, &m_smooth_rejects);

    // Not implemented: the envelope-held background vertex, which smooth_before() refuses. Such a
    // vertex sits on a tag-region boundary or the domain wall, so AMIPS alone is the wrong
    // objective; it needs Phase A's treatment (w_amips and w_envelope, the configured smoothing
    // mode and its projection line-search steps), after which smooth_vertex_3d picks up
    // smoothing_energy_envelope() for the pull and smoothing_containment_envelope() for the accept
    // test, both of which already answer correctly here. It must run last in the pass, after the
    // plain interior sweep, so it relaxes against a background that has already settled.
}

void TopoOffsetTetMesh::log_smooth_trace() const
{
    const auto& s = m_smooth_trace;
    logger().info(
        "\tsmooth trace: attempted {} | before: bbox {}, unrounded {}, on-input {}, "
        "phase-B wrong-sub-sweep {}, phase-B enveloped background {} | offset-surface "
        "vertices {}, interior {} ({})",
        s.attempted.load(),
        s.before_bbox.load(),
        s.before_unrounded.load(),
        s.before_on_region.load(),
        s.before_phase_b_not_offset.load(),
        s.before_phase_b_enveloped_background.load(),
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
