#include "TopoOffsetTetMesh.h"

#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/EnvelopeEnergy.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * THE smoother (smooth_vertex()), for every vertex in every phase: one Newton solve on E_V, the
 * vertex's one-ring sum of E_T (vertex_energy()). The 2D twin is FrontSmooth2d.cpp.
 */

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTetMesh::vertex_energy(
    const size_t vid) const
{
    // E_V(x) = sum over vid's one-ring t of E_T(t)(x): VolAMIPSEnergy3D over every cell -- against
    // its rest when it is plastic, the regular tet otherwise, exactly as cell_vol_amips3() reads
    // it -- and BandVolumeEnergy3D over the band cells, D(t)'s candidates taken from the corners
    // as they are now (the nearest triangle of each, and the cell's stored one).
    const Eigen::Matrix3d R_regular = VolAMIPSEnergy3D::regular_rest();
    std::vector<VolAMIPSEnergy3D::Cell> amips;
    std::vector<BandVolumeEnergy3D::Cell> band;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        const std::array<size_t, 4> orig = oriented_tet_vids(tid);
        // The moving vertex first, winding preserved, as the shared smoother orders it.
        const std::array<size_t, 4> vs = wmtk::orient_preserve_tet_reorder(orig, vid);
        std::array<Vector3d, 3> q;
        for (int k = 1; k < 4; ++k) q[size_t(k - 1)] = m_vertex_attribute[vs[size_t(k)]].m_posf;
        Eigen::Matrix3d R = R_regular;
        const TetAttributes& ta = m_tet_attribute[tid];
        if (cell_is_plastic(tid) && ta.rest_valid) {
            std::array<int, 4> from{};
            for (int k = 0; k < 4; ++k) {
                for (int j = 0; j < 4; ++j) {
                    if (orig[size_t(j)] == vs[size_t(k)]) from[size_t(k)] = j;
                }
            }
            Eigen::Matrix3d Rp;
            for (int k = 1; k < 4; ++k) {
                Rp.col(k - 1) = ta.rest_pos[size_t(from[size_t(k)])] - ta.rest_pos[size_t(from[0])];
            }
            if (Rp.determinant() > 0.) R = Rp; // a rest that holds no shape: the regular tet
        }
        VolAMIPSEnergy3D::Cell c;
        if (VolAMIPSEnergy3D::cell(q, R, c)) amips.push_back(c);
        if (cell_is_offset_band(tid) && m_band_tris) {
            BandVolumeEnergy3D::Cell b;
            b.q = q;
            for (const size_t u : orig) {
                m_band_tris->nearest_all(m_vertex_attribute[u].m_posf, b.candidates);
            }
            if (ta.band_tri >= 0) b.candidates.push_back(ta.band_tri);
            std::sort(b.candidates.begin(), b.candidates.end());
            b.candidates.erase(
                std::unique(b.candidates.begin(), b.candidates.end()),
                b.candidates.end());
            band.push_back(std::move(b));
        }
    }
    if (amips.empty()) return nullptr;
    auto sum = std::make_shared<optimization::EnergySum>();
    sum->add_energy(std::make_shared<VolAMIPSEnergy3D>(std::move(amips), amips_weight()));
    if (!band.empty()) {
        sum->add_energy(
            std::make_shared<BandVolumeEnergy3D>(
                m_band_tris,
                std::move(band),
                m_offset_params.target_distance,
                band_weight()));
    }
    return sum;
}

bool TopoOffsetTetMesh::held_faces_contained(const size_t vid) const
{
    // Only the faces the envelope holds (face_is_held()): a held vertex on another tracked face --
    // an unheld region boundary, the offset surface -- is not refused for that face lying outside
    // an envelope that was never built around it.
    if (!m_envelope) return true;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        for (int j = 0; j < 4; ++j) {
            const Tuple f = tuple_from_face(tid, j);
            if (!m_face_attribute[f.fid(*this)].m_is_surface_fs) continue;
            const auto vs = get_face_vids(f);
            if (vs[0] != vid && vs[1] != vid && vs[2] != vid) continue;
            if (!face_is_held(f)) continue;
            const std::array<Vector3d, 3> tri = {
                {m_vertex_attribute[vs[0]].m_posf,
                 m_vertex_attribute[vs[1]].m_posf,
                 m_vertex_attribute[vs[2]].m_posf}};
            if (m_envelope->is_outside(tri)) return false;
        }
    }
    return true;
}

bool TopoOffsetTetMesh::smooth_vertex(const Tuple& t)
{
    // See the declaration. A refusal returns false and TetMesh::smooth_vertex() rolls the
    // position and the ring's stored qualities back; every path below that moved the vertex also
    // puts it back itself, so the counters and diagnostics read the state the engine restores.
    const size_t vid = t.vid(*this);
    const std::vector<Tuple> locs = get_one_ring_tets_for_vertex(t);
    for (const Tuple& loc : locs) {
        if (is_inverted_f(loc)) {
            // A neighbour that is not rounded can leave a tet inverted in floats though it is fine
            // in exact arithmetic: there is nothing to optimise from.
            ++m_smooth_rejects.already_inverted;
            return false;
        }
    }
    const auto energy = vertex_energy(vid);
    if (!energy) return false;
    const bool front = m_vertex_extra[vid].m_is_on_offset;
    const bool held = vertex_is_held(vid);
    const Vector3d x0 = m_vertex_attribute[vid].m_posf;
    const std::vector<size_t> ring = get_one_ring_tids_for_vertex(vid);

    // EXPERIMENTAL_unreachable_exit: the change this solve makes to a front vertex's own error, 0
    // when the move is refused (see VertexExtra::m_front_change).
    const double r_before = front ? front_vertex_relative_residual(vid) : 0.;
    const auto record_change = [&](const bool moved) {
        if (!front) return;
        VertexExtra& ve = m_vertex_extra[vid];
        ve.m_front_change_group = m_smooth_group;
        const double r_after = moved ? front_vertex_relative_residual(vid) : r_before;
        ve.m_front_change =
            moved ? std::abs(r_after - r_before) / m_offset_params.front_conv_frac() : 0.;
    };
    // DEBUG_crossings (log-only): the ring measures this move can change -- of vid and of every
    // vertex that shares a front face with it -- before the solve.
    std::vector<size_t> nb;
    std::vector<double> nb_before;
    if (front && m_offset_params.debug_crossings) {
        nb.push_back(vid);
        for (const Tuple& ft : offset_surface_faces_live_at(vid)) {
            for (const size_t u : get_face_vids(ft)) nb.push_back(u);
        }
        wmtk::vector_unique(nb);
        for (const size_t u : nb) nb_before.push_back(ring_measure_at(u));
    }

    // THE SMOOTHING VETO, for every vertex: the ring's sum of E_T at the new position, finite and
    // not above the sum before (energy_not_raised()), read on the mesh with the minimisers the
    // move would store (ring_after()). E_V equals that sum only up to roundoff, which near a
    // nearly singular plastic rest is not small, so the solve alone does not bound E.
    const double ring_before = energy_sum(ring);
    // D(t)'s candidate from the moving corner at its start, which E_V holds and E_T would drop
    // once the corner's nearest triangle changes; see store_band_minimisers().
    const int64_t nearest_x0 = m_band_tris ? m_band_tris->nearest(x0) : -1;
    const auto ring_after = [&]() {
        // The minimisers store_band_minimisers() would keep, tried and put back.
        std::vector<int64_t> kept;
        for (const size_t tid : ring) kept.push_back(m_tet_attribute[tid].band_tri);
        store_band_minimisers(ring, nearest_x0);
        const double e = energy_sum(ring);
        for (size_t i = 0; i < ring.size(); ++i) {
            const size_t tid = ring[i];
            m_tet_attribute[tid].band_tri = kept[i];
        }
        return e;
    };
    const bool exact = held && m_params.smoothing_mode == "exact";

    // THE SOLVE: E_V from the vertex's position, plus the envelope's exact distance term for a
    // held vertex in smoothing_mode "exact".
    std::shared_ptr<polysolve::nonlinear::Problem> objective = energy;
    if (exact && m_envelope) {
        auto sum = std::make_shared<optimization::EnergySum>();
        sum->add_energy(energy);
        sum->add_energy(
            std::make_shared<optimization::ExactDistanceEnergy3D>(
                m_envelope,
                m_s_envelope * m_params.w_envelope));
        objective = sum;
    }
    polysolve::nonlinear::Solver& solver = smoothing_solver();
    Eigen::VectorXd xv = x0;
    bool threw = false;
    try {
        solver.minimize(*objective, xv);
    } catch (const std::exception&) {
        // polysolve reports a failed line search by throwing; the position it reached is still
        // the best it found, and the checks below decide whether to keep it.
        threw = true;
    }
    (front ? m_newton_front : m_newton).record(solver, threw);
    if (front && m_offset_params.debug_output) {
        std::lock_guard<std::mutex> lock(m_front_solve_log_mutex);
        m_front_solve_log.push_back(
            {vid, int(solver.current_criteria().iterations), int(solver.status()) + 1});
    }
    if (front) {
        // Diagnostic: the solve's final gradient norm and its ratio to the first one.
        const auto& cr = solver.current_criteria();
        const auto bin = [](double v, int lo) {
            if (!(v > 0.)) return 0;
            return std::clamp(int(std::floor(std::log10(v))) - lo + 1, 0, kGradBins - 1);
        };
        ++m_front_grad_abs[size_t(bin(cr.gradNorm, -14))];
        ++m_front_grad_rel[size_t(bin(cr.relGradNorm, -14))];
    }
    const Vector3d x = xv.head<3>();

    const auto ring_valid = [&]() {
        for (const Tuple& loc : locs) {
            if (is_inverted(loc)) return false; // exact
        }
        return true;
    };
    const auto refuse = [&](std::atomic<size_t>& counter) {
        set_vertex_position(vid, x0);
        ++counter;
        record_change(false);
        return false;
    };

    if (!held) {
        // The solve's point, kept when the ring stays valid and the veto passes. A failed solve
        // (a thrown line search, a non-finite iterate) keeps the start.
        if (!x.allFinite()) return refuse(m_smooth_rejects.quality);
        set_vertex_position(vid, x);
        if (!ring_valid()) return refuse(m_smooth_rejects.inverted);
        if (!energy_not_raised(ring_after(), ring_before)) return refuse(m_smooth_rejects.quality);
    } else if (!exact && m_envelope) {
        // Projected: solve in free space, project back onto the envelope, and walk back toward
        // the start, t = 1, 1/2, 1/4, ..., accepting the first projected candidate whose ring is
        // valid, whose held faces stay inside the envelope and whose ring's sum of E_T passes the
        // veto. Then, if none did, the nested pass: for each candidate, bisect between the
        // interpolated point and its projection for the longest acceptable step toward the input.
        const auto acceptable = [&](const Vector3d& q) {
            set_vertex_position(vid, q);
            return ring_valid() && held_faces_contained(vid) &&
                   energy_not_raised(ring_after(), ring_before);
        };
        bool accepted = false;
        std::vector<Vector3d> interp, proj;
        if (x.allFinite()) {
            for (int k = 0; k < m_params.project_line_search_steps && !accepted; ++k) {
                const Vector3d p = x0 + std::pow(0.5, k) * (x - x0);
                Vector3d q;
                m_envelope->nearest_point(p, q);
                interp.push_back(p);
                proj.push_back(q);
                accepted = acceptable(q);
            }
            for (size_t k = 0;
                 k < proj.size() && !accepted && m_params.project_line_search_nested_steps > 0;
                 ++k) {
                double lo = 0., hi = 1.;
                Vector3d best;
                bool found = false;
                for (int j = 0; j < m_params.project_line_search_nested_steps; ++j) {
                    const double mid = 0.5 * (lo + hi);
                    const Vector3d cand = interp[k] + mid * (proj[k] - interp[k]);
                    if (acceptable(cand)) {
                        lo = mid, best = cand, found = true;
                    } else {
                        hi = mid;
                    }
                }
                if (found) accepted = acceptable(best);
            }
        }
        if (!accepted) return refuse(m_smooth_rejects.quality);
    } else {
        // Exact: the solve already held the vertex near the envelope; the same three tests.
        if (!x.allFinite()) return refuse(m_smooth_rejects.quality);
        set_vertex_position(vid, x);
        if (!ring_valid()) return refuse(m_smooth_rejects.inverted);
        if (!held_faces_contained(vid)) return refuse(m_smooth_rejects.envelope);
        if (!energy_not_raised(ring_after(), ring_before)) return refuse(m_smooth_rejects.quality);
    }

    store_band_minimisers(ring, nearest_x0);
    for (const Tuple& loc : locs) set_cell_quality(loc.tid(*this), get_quality(loc));
    ++m_smooth_rejects.accepted;
    record_change(true);
    if (front && m_offset_params.debug_crossings) {
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

} // namespace wmtk::components::topological_offset
