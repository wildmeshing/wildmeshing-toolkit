
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

    // PHASE B RUNS IN TWO ORDERED SUB-SWEEPS; SEE PhaseBSub.
    //
    // A pass places every offset-surface vertex first, then relaxes the background under the
    // surface those placements just defined. The order is the point: relaxing the background
    // against last pass's surface wastes the work, and interleaving the two makes each vertex's
    // one-ring a moving target for its neighbours in the same sweep. This admission is what
    // splits them -- each sub-sweep refuses the other's class outright.
    //
    // ENVELOPES ARE THE OTHER AXIS, and the two classes answer differently. Phase B runs with
    // the offset envelope RELEASED and takes no containment responsibility, so a vertex an
    // envelope is supposed to hold has no one to hold it here:
    //
    //   - an OFFSET vertex that is also envelope-held is a case this scheme does not yet
    //     handle, and it THROWS rather than being silently skipped -- a skipped one would sit
    //     off the level set for the whole phase while the convergence counters reported the
    //     surface placed. The models under test have no such vertex.
    //   - a BACKGROUND vertex that is envelope-held is skipped for now. Its solve wants the
    //     envelope's own pull and containment, which is stencilled at the end of
    //     smooth_interior_vertex_phase_b() rather than written, so until then leaving it where
    //     Phase A put it is the honest behaviour.
    //
    // ON THE OFFSET SURFACE admits regardless of the other flags. The admission used to refuse
    // `ve.m_is_on_region` too, which was the last vertex freeze left in the phase: a vertex
    // carrying both flags -- which collapse_after_vertex() creates every time it merges an
    // offset vertex into an input one -- sat on the offset surface and was never placed on the
    // level set. The flag does not mean "at distance 0 from the complex, where Phi diverges";
    // it accretes through splits and collapses. The genuinely contradictory case is geometric
    // and check_no_vertex_on_both_surfaces() already throws on it, so anything reaching here
    // has a finite Phi and a usable normal.
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

    // TWO PHASES; IN B, TWO SOLVES, AND NO AMIPS IN THE OFFSET PLACEMENT.
    //
    // Phase B places offset-only vertices with the offset potential alone: a 1-D Newton root
    // find on Phi(x) = target along the gradient, backtracked into the element by bisection if
    // the root would invert the ring. Its interior vertices minimize their one-ring AMIPS to
    // their own minimum (smooth_interior_vertex_phase_b). smooth_before() has already refused
    // everything else in this phase, so the branch below is exhaustive.
    //
    // WHY THE OFFSET SOLVE TAKES NO AMIPS. Blending w_amips * AMIPS into an offset vertex's
    // objective leaves it resting a w_amips-proportional distance off the level set (the header
    // on smooth_offset_vertex_backtracking has the measurement) -- the at-vertex wall. And
    // AMIPS alone cannot be dropped from the shared solve either: the offset energy's Hessian
    // is 2*w*g*g^T, rank 1, with a 2-D nullspace in the level set's tangent plane, so a 3-D
    // minimize of it is ill-posed. The 1-D root find sidesteps both -- it solves the only
    // direction the potential determines, and asks nothing of the two it does not. The interior
    // solve has no such conflict: it carries no offset term at all.
    const bool ok = (m_phase == OptPhase::B)
                        ? (m_vertex_extra[vid].m_is_on_offset ? smooth_offset_vertex_backtracking(t)
                                                              : smooth_interior_vertex_phase_b(t))
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
            ++m_phase_b_constrained; // cannot even attempt its minimum
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
    // components never enter, so there is no nullspace to regularise and no AMIPS needed.
    //
    // TO THIS VERTEX'S OWN MINIMUM, not a fixed handful of steps. The visit ends when the
    // gradient of the quadratic error E = (Phi - c)^2, |grad E| = 2 |Phi - c| |grad Phi|,
    // falls below vertex_grad_tol_rel of ITS OWN value at the visit's start -- the
    // per-vertex tolerance, deliberately separate from the run's convergence bar, which is
    // checked once per A/B round. The iteration cap is a guard against a cycling Newton, not
    // the intended stop; the step floor below it is the numerical one.
    // MINIMISE E = (Phi - c)^2. NOT a root find on Phi = c -- that distinction is the whole
    // point of the damping below.
    //
    // Gauss-Newton on E gives the undamped step -(r/|g|^2) g, which is what this used to take.
    // It is exact where a root exists and CATASTROPHIC where one does not: at a local minimum
    // of E with r != 0, stationarity 2 r g = 0 forces g = ∇Phi = 0, so the step divides by a
    // quantity going to zero exactly as it approaches the answer. That is not a corner case --
    // it is every pinch. Two objects closer than 2 x target_distance have no level set between
    // them, and the surface belongs on the ridge where the contributions cancel and ∇Phi
    // vanishes by symmetry. Measured on two_spheres: 6 vertices ejected to 2.86x delta with the
    // gap only 0.062 delta wider than 2 delta, and 44 to 8.84x delta from the far field, where
    // |g| is small for the other reason (the barrier's decaying tail).
    //
    // Levenberg-Marquardt: solve (2 g g^T + lambda I) d = -2 r g, which along g is
    //     d = -( r / (|g|^2 + mu) ) g,     mu = lambda / 2
    // so |g| -> 0 sends the step to -(r/mu) g -> 0 instead of to infinity. mu carries units of
    // |∇Phi|^2, so it is held relative to the potential's own level-set slope. Shrink it on a
    // step that lowers E, grow it on one that does not: exact Gauss-Newton where a root exists
    // (unchanged convergence on convex inputs), short damped steps where one does not.
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
            // grad E = 2 r g = 0 with g = 0: a STATIONARY POINT of E, which under the damped
            // step is a legitimate place to stop -- it is the pinch minimum. Under the old
            // undamped step this branch was unreachable except by escaping the support.
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
        // A SAFETY MARGIN ON WHAT THE RING ALLOWS. `lo` after 30 halvings sits within ~1e-9 of
        // the first inverting configuration -- valid by the orientation predicate and
        // numerically degenerate in every other sense, which is what fed near-zero-volume tets
        // to Phase A and (while the criterion still assembled AMIPS) produced the inf gradients
        // that hung Phase B.
        //
        // IT IS A DAMAGE-VS-PROGRESS DIAL WITH A LOW CEILING, and 0.8 is measured to be a
        // reasonable place on it rather than a tuned optimum. AMIPS is
        // tr(J^T J)^{3/2} / (3 det J), so a backtracked placement blows up from both ends at
        // once: the long displacement inflates the numerator while the near-inversion drives
        // the denominator toward zero. Swept on prism d0.02 g0.2, raw construction:
        //
        //   0.9  3 rounds, 22s, 41594 T, err 7.9%  of delta, worst handoff 6.0e8 / avg 1.2e5
        //   0.8  5 rounds, 49s, 45574 T, err 1.5%  of delta, worst handoff 7.5e8 / avg 4.2e4
        //   0.5  4 rounds, 43s, 44928 T, err 9.3%  of delta, worst handoff 5.3e7 / avg 3.2e4
        //
        // The knob is NOT monotone in any of those columns, so read it as scatter and not as a
        // trend: 0.8 costs two more rounds than 0.9 yet lands 5x more accurate, while its peak
        // handoff damage is the worst of the three. All three stop at 0.994x the gradient bar,
        // so they are equally converged BY THE RUN'S OWN CRITERION and the 6x spread in
        // max_dist_err (0.026 to 0.156) is the criterion failing to pin the distance, not the
        // margin steering it. DO NOT expect more from this knob: it only
        // governs BACKTRACKED placements, which are ~3% of the work (18-21 constrained out of
        // ~615 placed per pass). The other ~97% reach their minimum inside the ring, bypass
        // this branch entirely, and move as far as they like -- round 1's first pass moves a
        // vertex 2.96 units against a target edge length of 4.18 at EVERY margin setting,
        // identically. The mesh damage is done by the placements this line never sees. Nor can
        // it shrink the stuck set: "constrained" means the minimum lies outside the one-ring,
        // a fact about the mesh rather than about how far one steps after discovering it (the
        // plateau sat at 18 vertices at 0.9 and 21 at 0.5).
        //
        // ONLY ON THIS PATH. A vertex whose minimum is reachable inside its ring never enters
        // this branch, so it still lands exactly on the level set; the margin is a concession
        // to the constraint and is owed only where the constraint actually bound.
        //
        // The valid set along the segment need not be a single interval -- tet orientation is
        // a cubic in s -- so the retreated point is re-tested rather than assumed.
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
    // Pure one-ring AMIPS, solved to this vertex's own minimum; see the declaration. AMIPS
    // alone at unit weight -- Newton cancels any positive scale, so the value is irrelevant --
    // and no envelope terms: an interior vertex carries no surface, so the shared smoother's
    // pull and containment branches are bypassed and this is exactly `solve()` plus the exact
    // inversion accept and the quality veto.
    optimization::SmoothVertexOptions opts;
    opts.w_amips = 1.0;
    opts.w_envelope = 0.0;
    opts.s_amips = 1.0;
    opts.s_envelope = 0.0;
    opts.two_stage = false;
    // The veto the 3D pipeline runs everywhere: a Newton descent from the current position
    // should not worsen the worst incident element, so this only refuses line-search accidents.
    // It was once suspected of deadlocking cooperative repair on the mesh the offset placement
    // mangles -- measured on prism d0.02 g0.2 and DISPROVED: dropping it took interior accepts
    // from 2351 to 3710 per pass with zero quality refusals, and improved the handoff only ~2x
    // against a four-orders-of-magnitude problem. Keep it on.
    opts.quality_veto = m_params.smooth_quality_veto;

    auto& solver = m_phase_b_solver.local();
    if (!solver) {
        // The base solver's twin, with the stopping rule this phase is about: polysolve's
        // rel_grad_norm_tol is exactly gradNorm / initial_grad_norm, so the solve stops at
        // vertex_grad_tol_rel of the visit's initial gradient, and the iteration budget is
        // deep enough that the tolerance is what actually fires (the base runs a fixed 10
        // with no tolerance at all).
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

    // ---------------------------------------------------------------------------------------
    // STENCIL: the envelope-held background vertex, which smooth_before() currently refuses.
    //
    // Such a vertex sits on a tag-region boundary or the domain wall, so AMIPS alone is the
    // wrong objective for it -- nothing in the solve above would keep the surface it carries
    // where that surface belongs, and Phase B has released the offset envelope and takes no
    // containment responsibility. What it needs is the treatment Phase A already gives it:
    //
    //     opts.w_amips        = m_params.w_amips;          // shape, lightly weighted
    //     opts.w_envelope     = m_params.w_envelope;       // the tube it must stay in
    //     opts.smoothing_mode = <as m_params.smoothing_mode>;
    //     opts.project_line_search_steps        = m_params.project_line_search_steps;
    //     opts.project_line_search_nested_steps = m_params.project_line_search_nested_steps;
    //
    // and then smooth_vertex_3d picks up smoothing_energy_envelope(vid) for the pull and
    // smoothing_containment_envelope(vid) for the accept test, both of which already answer
    // correctly for these vertices (the per-tag dispatch handles junctions by intersection).
    //
    // ORDER: these should run LAST in the pass, after the plain interior sweep, so they relax
    // against a background that has already settled rather than dragging it.
    //
    // Not written yet because the models under test have no such vertex, and a half-constrained
    // version would be worse than leaving them where Phase A put them.
    // ---------------------------------------------------------------------------------------
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
