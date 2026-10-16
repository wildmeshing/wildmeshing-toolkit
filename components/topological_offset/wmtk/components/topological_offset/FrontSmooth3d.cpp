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
    // through smoothing_extra_energy() -- the vertex's part of the energy, the AMIPS term over the
    // ring and the band term of its band cells -- and w_amips 0 keeps the smoother from adding an
    // AMIPS term of its own; a front vertex an input envelope pins carries no band term. The
    // smoother holds a front vertex to no envelope (see smoothing_containment_envelope()), so
    // neither the projected path nor the containment check applies -- solve, exact inversion
    // test, then the veto on tet_energy() below.
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
    // THE FRONT SMOOTHER'S VETO (see tet_energy()), on every 3-D solve: the sum of tet_energy()
    // over the vertex's one-ring may not rise. A tie passes, as in the engine's AMIPS veto. Read
    // before the solve and after it, on the mesh: a smoothing move changes no label and moves one
    // vertex, so the one-ring holds every cell whose energy it can change. Refused,
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
        const std::vector<size_t>& ring = get_one_ring_tids_for_vertex(vid);
        const double before = tet_energy_sum(ring);
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
        // EXPERIMENTAL_band_volume_rule "front_flux": the band term's exceptions (a curved
        // bisector crossing a piece, a partition that does not tile) are errors, not failed line
        // searches; the shared smoother takes any exception out of a solve for the latter, so
        // the term keeps its message (BandVolumeEnergy3D::front_flux_error()) and it is rethrown
        // here.
        std::string& ff_error = BandVolumeEnergy3D::front_flux_error();
        ff_error.clear();
        const bool ok =
            optimization::smooth_vertex_3d(*this, t, opts, solver, &m_smooth_rejects, &one);
        if (!ff_error.empty()) {
            const std::string msg = std::move(ff_error);
            ff_error.clear();
            log_and_throw_error("front smoother, vertex {}: {}", vid, msg);
        }
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
        if (!(tet_energy_sum(ring) <= before)) { // a NaN refuses
            ++m_front_veto_fired;
            --m_smooth_rejects.accepted;
            ++m_smooth_rejects.quality;
            record_change(false);
            return false;
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
    // smoothing_extra_energy() at a front vertex that carries the offset term, with the offset
    // term included unconditionally: the AMIPS term over the one-ring and the band term
    // (front_energy()).
    auto sum = std::make_shared<optimization::EnergySum>();
    sum->add_energy(amips3_energy(vid));
    sum->add_energy(front_energy(vid));
    return sum;
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTetMesh::front_energy(
    const size_t vid) const
{
    // The vertex's part of the energy's band term; see band_volume_energy_at().
    auto sum = std::make_shared<optimization::EnergySum>();
    if (const auto bv = band_volume_energy_at(vid)) sum->add_energy(bv);
    return sum;
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTetMesh::band_volume_energy_at(
    const size_t vid) const
{
    // band_cell_term() of each band cell of the ring, x moving (BandVolumeEnergy3D).
    if (m_n_regions > 1 && !m_offset_params.allow_mult_comp) {
        log_and_throw_error(
            "the energy's band term reads one field, and this input has {} regions",
            m_n_regions);
    }
    std::vector<BandVolumeEnergy3D::Cell> cells;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        if (!cell_is_offset_band(tid)) continue;
        BandVolumeEnergy3D::Cell c;
        int k = 0;
        for (const size_t u : oriented_tet_vids(tid)) {
            if (u == vid) continue;
            (k == 0 ? c.q1 : k == 1 ? c.q2 : c.q3) = m_vertex_attribute[u].m_posf;
            ++k;
        }
        if (k != 3) continue;
        cells.push_back(c);
    }
    if (cells.empty()) return nullptr;
    auto e = std::make_shared<BandVolumeEnergy3D>(
        m_band_tris,
        std::move(cells),
        m_vertex_attribute[vid].m_posf,
        m_offset_params.target_distance);
    // EXPERIMENTAL_band_volume_rule "exact": each cell's closed-form integral, as band_cell_term().
    if (m_offset_params.band_volume_rule == "exact") {
        e->set_exact_integral(m_band_exact, m_offset_params.target_distance);
    }
    // EXPERIMENTAL_band_volume_rule "front_flux": the front form of the same term
    // (BandVolumeEnergy3D::set_front_flux()), over the band's boundary faces through vid. A face
    // through vid lies in one or two cells of vid's one-ring, so pairing the ring's faces through
    // vid finds them without reading beyond the ring: a boundary face has exactly one band cell
    // among its cells (the other not a band cell, or none on the domain boundary). Each is
    // ordered (vid, a, b) with its normal out of its band cell, away from the cell's fourth corner.
    if (m_offset_params.band_volume_rule == "front_flux") {
        struct Rec
        {
            size_t lo, hi;
            int band = 0;
            BandVolumeEnergy3D::Face f;
        };
        std::vector<Rec> recs;
        const Vector3d& x = m_vertex_attribute[vid].m_posf;
        for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
            const bool band = cell_is_offset_band(tid);
            std::array<size_t, 3> o;
            int k = 0;
            for (const size_t u : oriented_tet_vids(tid)) {
                if (u != vid && k < 3) o[size_t(k++)] = u;
            }
            if (k != 3) continue;
            for (int i = 0; i < 3; ++i) {
                const size_t u = o[size_t(i)], w = o[size_t((i + 1) % 3)],
                             z = o[size_t((i + 2) % 3)];
                const size_t lo = std::min(u, w), hi = std::max(u, w);
                Rec* r = nullptr;
                for (Rec& q : recs) {
                    if (q.lo == lo && q.hi == hi) r = &q;
                }
                if (!r) {
                    recs.push_back({lo, hi, 0, {}});
                    r = &recs.back();
                }
                if (!band) continue;
                ++r->band;
                Vector3d a = m_vertex_attribute[u].m_posf, b = m_vertex_attribute[w].m_posf;
                if ((a - x).cross(b - x).dot(m_vertex_attribute[z].m_posf - x) > 0.)
                    std::swap(a, b);
                r->f = {a, b};
            }
        }
        std::vector<BandVolumeEnergy3D::Face> faces;
        for (const Rec& r : recs) {
            if (r.band == 1) faces.push_back(r.f);
        }
        e->set_front_flux(m_band_exact, m_offset_params.target_distance, std::move(faces));
    }
    return e;
}

} // namespace wmtk::components::topological_offset
