#include "TopoOffsetTetMesh.h"

#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * Front placement: an offset-surface vertex is placed by the shared 3-D smoother with the offset's
 * energy (see smooth_front_vertex()). Everything else is smoothed by Optimize3d.cpp. The 2D twin
 * is FrontSmooth2d.cpp.
 */

bool TopoOffsetTetMesh::smooth_front_vertex(const Tuple& t)
{
    // See the header: the shared smoother with the offset's options. The whole objective arrives
    // through smoothing_extra_energy() -- the ring barrier plus the offset error over the
    // vertex's front faces, NO AMIPS (front_objective()) -- and w_amips 0 keeps the smoother from
    // adding an AMIPS term of its own. The
    // smoother holds a front vertex to no envelope (see smoothing_containment_envelope()), so
    // neither the projected path nor the containment check applies -- solve, exact inversion
    // test, then the veto on the same objective below.
    const size_t vid = t.vid(*this);
    optimization::SmoothVertexOptions opts;
    opts.w_amips = 0.;
    opts.w_envelope = m_params.w_envelope;
    opts.s_amips = m_s_amips;
    opts.s_envelope = m_s_envelope;
    opts.two_stage = false;
    // The engine's veto compares AMIPS^3 alone (smoothing_cell_energy()), and a front vertex
    // must be free to worsen its ring's shape on the way to the level set: off, as it always was
    // here. solve_3d() vetoes on the energy instead.
    opts.quality_veto = false;
    opts.max_step = damping_step(vid); // EXPERIMENTAL_damping; infinity when off
    const Vector3d x0 = std::as_const(m_vertex_attribute)[vid].m_posf;
    auto& solver =
        m_solver.local(); // the thread's shared solver, criteria set by smoothing_solver()
    smoothing_solver();
    // THE FRONT SMOOTHER'S VETO, on every 3-D solve: the objective itself -- the ring barrier
    // (frozen at this solve's start, as the solver's own instance is) plus the offset error over
    // the vertex's front faces -- may not rise. A tie passes, as in the engine's AMIPS veto. Read
    // before the solve and after it, on the mesh. Refused,
    // TetMesh::smooth_vertex() rolls back the position and the ring's stored qualities.
    // smooth_vertex_3d() has counted the move accepted by then; it is recounted as a quality
    // refusal, so the counters still partition the attempts.
    // The change this solve makes to the vertex's own error, 0 when the move is refused and
    // rolled back (see VertexExtra::m_front_change).
    const double r_before = front_vertex_relative_residual(vid);
    const auto record_change = [&](const bool moved) {
        VertexExtra& ve = m_vertex_extra[vid];
        ve.m_front_change_group = m_smooth_group;
        const double r_after = moved ? front_vertex_relative_residual(vid) : r_before;
        ve.m_front_change =
            moved ? std::abs(r_after - r_before) / m_offset_params.front_conv_frac() : 0.;
    };
    const auto solve_3d = [&]() {
        // The veto's instance of the objective: frozen at the same start as the one the solver
        // minimises (both are built from the mesh as it is now), so the two are the same function.
        const auto objective = front_objective(vid);
        const Eigen::VectorXd xv0 = x0;
        const double before = objective->value(xv0);
        // growth_mode "individual_op_reject": the folded offset faces through vid before the
        // move, which may not raise them (every face it changes goes through vid).
        int folds_before = -1;
        if (fold_reject_active()) {
            std::set<std::array<size_t, 3>> f;
            folded_front_faces_at(vid, f);
            folds_before = int(f.size());
        }
        // DEBUG_crossings (log-only): the ring measures this move can change -- of vid and of
        // every vertex that shares a front face with it -- before the solve.
        std::vector<size_t> nb;
        std::vector<double> nb_before;
        if (m_offset_params.debug_crossings) {
            nb.push_back(vid);
            for (const Tuple& ft : offset_surface_faces_live_at(vid)) {
                for (const size_t u : get_face_vids(ft)) nb.push_back(u);
            }
            wmtk::vector_unique(nb);
            for (const size_t u : nb) nb_before.push_back(ring_measure_at(u));
        }
        // The solve's own counter, so DEBUG_output can pin its outcome to vid
        // (m_front_solve_log), folded into the pass's m_newton_front, which therefore counts
        // exactly what passing it in counted. At most one solve (two_stage is off), none when
        // smooth_vertex_3d() refuses an already-inverted ring before solving; nothing after the
        // solve touches the solver, so it still holds that solve's state here.
        optimization::NewtonCounters one;
        const bool ok =
            optimization::smooth_vertex_3d(*this, t, opts, solver, &m_smooth_rejects, &one);
        if (one.solves() > 0) {
            m_newton_front.record(*solver, one.threw.load() > 0);
            if (m_offset_params.debug_output) {
                std::lock_guard<std::mutex> lock(m_front_solve_log_mutex);
                m_front_solve_log.push_back(
                    {vid, int(solver->current_criteria().iterations), int(solver->status()) + 1});
            }
        }
        if (!ok) {
            record_change(false);
            return false;
        }
        {
            // Diagnostic: the solve's final gradient norm and its ratio to the first one.
            const auto& cr = solver->current_criteria();
            const auto bin = [](double v, int lo) {
                if (!(v > 0.)) return 0;
                return std::clamp(int(std::floor(std::log10(v))) - lo + 1, 0, kGradBins - 1);
            };
            ++m_front_grad_abs[size_t(bin(cr.gradNorm, -14))];
            ++m_front_grad_rel[size_t(bin(cr.relGradNorm, -14))];
        }
        ++m_front_veto_asked;
        const Eigen::VectorXd xv1 = std::as_const(m_vertex_attribute)[vid].m_posf;
        if (!(objective->value(xv1) <= before)) { // a NaN refuses
            ++m_front_veto_fired;
            --m_smooth_rejects.accepted;
            ++m_smooth_rejects.quality;
            record_change(false);
            return false;
        }
        if (folds_before >= 0) {
            std::set<std::array<size_t, 3>> f;
            folded_front_faces_at(vid, f);
            if (int(f.size()) > folds_before) {
                ++m_fold_reject_smooth;
                --m_smooth_rejects.accepted;
                ++m_smooth_rejects.quality;
                record_change(false);
                return false;
            }
        }
        record_change(true);
        count_damped_move(vid, x0, opts.max_step);
        if (m_offset_params.debug_crossings) {
            for (size_t k = 0; k < nb.size(); ++k) {
                const double after = ring_measure_at(nb[k]);
                if (!(nb_before[k] <= 1.) || !(after > 1.)) continue;
                if (nb[k] == vid) {
                    ++m_cross_own;
                } else {
                    ++m_cross_neighbour;
                }
            }
        }
        return true;
    };
    return solve_3d();
}

bool TopoOffsetTetMesh::smooth_repulsion_vertex(const Tuple& t)
{
    // The front's path with the repulsion term in place of the band term: the whole objective
    // arrives through smoothing_extra_energy() -- the AMIPS term over the ring and
    // repulsion_energy(vid) -- and w_amips 0 keeps the smoother from adding an AMIPS term of its
    // own; no engine veto.
    // A repulsion vertex is never envelope-held (repulsion_smoothing() leaves those to TetWild's
    // rule), so the solve has no envelope term and no containment check.
    const size_t vid = t.vid(*this);
    optimization::SmoothVertexOptions opts;
    opts.w_amips = 0.;
    opts.w_envelope = m_params.w_envelope;
    opts.s_amips = m_s_amips;
    opts.s_envelope = m_s_envelope;
    opts.two_stage = false;
    opts.quality_veto = false;
    opts.max_step = damping_step(vid); // EXPERIMENTAL_damping; infinity when off
    const Vector3d x0 = std::as_const(m_vertex_attribute)[vid].m_posf;
    auto& solver = m_solver.local();
    smoothing_solver();
    // The front veto's rule (see tet_energy()): the sum of the per-tet energy over the ring --
    // which includes the repulsion terms of the cells that carry them, O(v) among them -- may
    // not rise (a tie passes). Read before the solve and after it; refused,
    // TetMesh::smooth_vertex() rolls the move back.
    const std::vector<size_t>& ring = get_one_ring_tids_for_vertex(vid);
    const double before = tet_energy_sum(ring);
    optimization::NewtonCounters one;
    const bool ok = optimization::smooth_vertex_3d(*this, t, opts, solver, &m_smooth_rejects, &one);
    if (one.solves() > 0) m_newton_repulsion.record(*solver, one.threw.load() > 0);
    if (!ok) return false;
    ++m_repulsion_veto_asked;
    if (!(tet_energy_sum(ring) <= before)) { // a NaN refuses
        ++m_repulsion_veto_fired;
        --m_smooth_rejects.accepted;
        ++m_smooth_rejects.quality;
        return false;
    }
    count_damped_move(vid, x0, opts.max_step);
    return true;
}

double TopoOffsetTetMesh::front_move_alignment(const size_t vid) const
{
    // |cos| between the direction the placement is allowed to move the vertex in and the field
    // normal, which is the direction that actually reduces its distance to the level set. Same
    // meaning as the 2D twin: 1 the test measures the step toward the level set, 0 it measures a
    // step that cannot reduce the distance at all. Negative marks a missing direction or gradient.
    const Vector3d n = front_vertex_move_direction(vid);
    if (!(n.squaredNorm() > 0.)) return -2.;
    const Vector3d g = front_vertex_field_gradient(vid);
    const double gn = g.norm();
    if (!(gn > 0.) || !std::isfinite(gn)) return -2.;
    return std::abs(n.normalized().dot(g / gn));
}

Vector3d TopoOffsetTetMesh::front_vertex_move_direction(const size_t vid) const
{
    // A front vertex held by an input envelope (it sits on a tag-region boundary or the domain
    // wall) may only move WITHIN that boundary. In 2D that is the curve's tangent; here the
    // boundary is a surface, so the direction is the field normal projected into the surface's
    // tangent plane -- or, where the incident region faces fold (a crease, a junction curve),
    // onto the crease's tangent. Every other front vertex moves along the field normal. Zero when
    // neither is defined.
    const Vector3d n_field = front_vertex_normal(vid);
    if (smoothing_containment_envelope(vid)) {
        std::vector<Vector3d> normals;
        const simplex::SimplexCollection surf = get_surface_faces_for_vertex(vid);
        for (const simplex::Face& f : surf.faces()) {
            const auto& fv = f.vertices();
            const auto found = try_tuple_from_face({{fv[0], fv[1], fv[2]}});
            if (!found) continue;
            const size_t fid = std::get<1>(*found);
            if (!m_face_attribute[fid].m_is_surface_fs || face_is_offset(fid)) continue;
            const Vector3d a = m_vertex_attribute[fv[0]].m_posf;
            const Vector3d b = m_vertex_attribute[fv[1]].m_posf;
            const Vector3d c = m_vertex_attribute[fv[2]].m_posf;
            Vector3d N = (b - a).cross(c - a);
            if (!(N.norm() > 0.)) continue;
            N /= N.norm();
            if (!normals.empty() && N.dot(normals.front()) < 0.) N = -N; // sign is arbitrary
            normals.push_back(N);
        }
        if (!normals.empty()) {
            // The sheet's normal, and whether the faces fold: a second normal well off the first
            // makes this a crease, whose tangent is the only admissible direction.
            Vector3d N1 = Vector3d::Zero();
            for (const Vector3d& N : normals) N1 += N;
            Vector3d crease = Vector3d::Zero();
            if (N1.norm() > 0.) {
                N1 /= N1.norm();
                for (const Vector3d& N : normals) {
                    if (std::abs(N.dot(N1)) < 0.9) {
                        crease = N1.cross(N);
                        break;
                    }
                }
            }
            Vector3d d;
            if (crease.norm() > 0.) {
                d = crease / crease.norm();
                if (d.dot(n_field) < 0.) d = -d;
            } else if (N1.norm() > 0.) {
                d = n_field - n_field.dot(N1) * N1;
            } else {
                d = Vector3d::Zero();
            }
            if (d.norm() > 0.) return d / d.norm();
            return Vector3d::Zero();
        }
    }
    // The field normal. Its known failure: on the input's medial axis grad Phi is undefined, and
    // a front vertex at a concave input corner sits exactly there. Same choice as 2D.
    return n_field;
}

Vector3d TopoOffsetTetMesh::front_vertex_normal(const size_t vid) const
{
    const Vector3d g = front_vertex_field_gradient(vid);
    const double gn = g.norm();
    return (std::isfinite(gn) && gn > 0.) ? Vector3d(g / gn) : Vector3d::Zero();
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTetMesh::front_objective(
    const size_t vid) const
{
    // smoothing_extra_energy() at a front vertex the loop places: the IPC barrier on the boundary
    // of the one-ring plus the offset error over the vertex's front faces -- NO AMIPS. The
    // barrier is 0 where the solve starts and climbs to +inf at the ring's boundary, so the
    // vertex chases the level set freely inside its ring and can never leave or invert it.
    auto sum = std::make_shared<optimization::EnergySum>();
    if (const auto barrier = ring_barrier_energy(vid)) sum->add_energy(barrier);
    if (const auto error = front_error_energy(vid)) sum->add_energy(error);
    return sum;
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTetMesh::front_error_energy(
    const size_t vid) const
{
    // The vertex's part of the offset error (face_offset_error() per face, offset_error() over
    // the front): its live offset faces' stencil terms with vid moving, on the union field. The
    // stencil weights are gathered once, the moving corner's weight first; every face shares them
    // (for_each_face_sample() samples every face the same way).
    if (!m_offset_potential) return nullptr;
    std::vector<FaceStencilError3D::Face> faces;
    for (const Tuple& ft : offset_surface_faces_live_at(vid)) {
        const std::array<size_t, 3> f = get_face_vids(ft);
        FaceStencilError3D::Face fe;
        int k = 0;
        for (const size_t u : f) {
            if (u == vid) continue;
            (k == 0 ? fe.q1 : fe.q2) = m_vertex_attribute[u].m_posf;
            ++k;
        }
        if (k != 2) continue;
        faces.push_back(fe);
    }
    if (faces.empty()) return nullptr;
    std::vector<std::array<double, 3>> weights;
    weights.reserve(size_t(stencil_points_per_face()));
    for_each_face_sample(
        Vector3d::Zero(),
        Vector3d::UnitX(),
        Vector3d::UnitY(),
        [&](const Vector3d&, const double wa, const double wb, const double wc) {
            weights.push_back({{wa, wb, wc}});
        });
    return std::make_shared<FaceStencilError3D>(
        m_offset_potential,
        std::move(faces),
        std::move(weights));
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTetMesh::ring_barrier_energy(
    const size_t vid) const
{
    // See the declaration: per ring tet the plane of the face opposite vid, oriented toward vid's
    // current position; the activation distance is 0.2x the distance now.
    const Vector3d& x0 = m_vertex_attribute[vid].m_posf;
    std::vector<RingBarrier3D::Wall> walls;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        std::array<Vector3d, 3> q;
        int k = 0;
        for (const size_t u : oriented_tet_vids(tid)) {
            if (u == vid) continue;
            if (k < 3) q[size_t(k)] = m_vertex_attribute[u].m_posf;
            ++k;
        }
        if (k != 3) continue;
        Vector3d n = (q[1] - q[0]).cross(q[2] - q[0]);
        const double nn = n.norm();
        if (!(nn > 0.) || !std::isfinite(nn)) continue;
        n /= nn;
        double s0 = n.dot(x0 - q[0]);
        if (s0 < 0.) {
            n = -n;
            s0 = -s0;
        }
        if (!(s0 > 0.)) continue; // a degenerate start: no wall to hold
        walls.push_back({n, n.dot(q[0]), 0.2 * s0});
    }
    if (walls.empty()) return nullptr;
    return std::make_shared<RingBarrier3D>(std::move(walls));
}

} // namespace wmtk::components::topological_offset
