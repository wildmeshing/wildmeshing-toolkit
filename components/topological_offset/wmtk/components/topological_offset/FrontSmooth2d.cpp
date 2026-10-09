#include "TopoOffsetTriMesh.h"

#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/EnvelopeEnergy.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * THE smoother (smooth_vertex()), for every vertex in every phase: one Newton solve on E_V, the
 * vertex's one-ring sum of E_T (vertex_energy()). The 3D twin is FrontSmooth3d.cpp.
 */

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTriMesh::vertex_energy(
    const size_t vid) const
{
    // E_V(x) = sum over vid's one-ring t of E_T(t)(x): VolAMIPSEnergy2D over every face -- against
    // its rest when it is plastic, the equilateral triangle otherwise, exactly as
    // face_vol_amips2() reads it -- and BandVolumeEnergy2D over the band faces, D(t)'s candidates
    // taken from the corners as they are now (the nearest segment of each, and the face's stored
    // one). As in 3D.
    const Eigen::Matrix2d R_regular = VolAMIPSEnergy2D::regular_rest();
    std::vector<VolAMIPSEnergy2D::Cell> amips;
    std::vector<BandVolumeEnergy2D::Cell> band;
    for (const size_t fid : get_one_ring_fids_for_vertex(vid)) {
        const auto orig = oriented_tri_vids(fid);
        // The moving vertex first: a cyclic rotation, which keeps the winding.
        int i0 = 0;
        for (int k = 0; k < 3; ++k) {
            if (orig[size_t(k)] == vid) i0 = k;
        }
        std::array<int, 3> from{};
        for (int k = 0; k < 3; ++k) from[size_t(k)] = (i0 + k) % 3;
        std::array<Vector2d, 2> q;
        for (int k = 1; k < 3; ++k) {
            q[size_t(k - 1)] = m_vertex_attribute[orig[size_t(from[size_t(k)])]].m_posf;
        }
        Eigen::Matrix2d R = R_regular;
        const FaceExtra2d& fx = m_face_extra[fid];
        if (face_is_plastic(fid) && fx.rest_valid) {
            Eigen::Matrix2d Rp;
            for (int k = 1; k < 3; ++k) {
                Rp.col(k - 1) = fx.rest_pos[size_t(from[size_t(k)])] - fx.rest_pos[size_t(from[0])];
            }
            if (Rp.determinant() > 0.) R = Rp; // a rest that holds no shape: the equilateral one
        }
        VolAMIPSEnergy2D::Cell c;
        if (VolAMIPSEnergy2D::cell(q, R, c)) amips.push_back(c);
        if (face_is_offset_band(fid) && m_band_segs) {
            BandVolumeEnergy2D::Cell b;
            b.q = q;
            for (const size_t u : orig) {
                b.candidates.push_back(m_band_segs->nearest(m_vertex_attribute[u].m_posf));
            }
            if (fx.band_seg >= 0) b.candidates.push_back(fx.band_seg);
            std::sort(b.candidates.begin(), b.candidates.end());
            b.candidates.erase(
                std::unique(b.candidates.begin(), b.candidates.end()),
                b.candidates.end());
            band.push_back(std::move(b));
        }
    }
    if (amips.empty()) return nullptr;
    auto sum = std::make_shared<optimization::EnergySum>();
    sum->add_energy(std::make_shared<VolAMIPSEnergy2D>(std::move(amips), amips_weight()));
    if (!band.empty()) {
        sum->add_energy(
            std::make_shared<BandVolumeEnergy2D>(
                m_band_segs,
                std::move(band),
                m_offset_params.target_distance,
                band_weight()));
    }
    return sum;
}

bool TopoOffsetTriMesh::held_edges_contained(const size_t vid) const
{
    // The engine's own containment test: every tracked edge at vid inside the vertex's
    // containment envelope (smoothing_containment_envelope(), the region tubes alone).
    const std::shared_ptr<SampleEnvelope> hold = smoothing_containment_envelope(vid);
    if (!hold || !m_vertex_attribute[vid].m_is_on_surface) return true;
    const Vector2d p = m_vertex_attribute[vid].m_posf;
    const simplex::SimplexCollection es = get_surface_edges_for_vertex(vid);
    for (const simplex::Edge& e : es.edges()) {
        const auto& evs = e.vertices();
        const size_t u = evs[0] != vid ? evs[0] : evs[1];
        const std::array<Eigen::Vector2d, 2> edge = {{p, m_vertex_attribute[u].m_posf}};
        if (hold->is_outside(edge)) return false;
    }
    return true;
}

bool TopoOffsetTriMesh::smooth_vertex(const Tuple& t)
{
    // See the declaration. A refusal returns false and TriMesh::smooth_vertex() rolls the
    // position and the ring's stored qualities back; every path below that moved the vertex also
    // puts it back itself, so the counters and diagnostics read the state the engine restores.
    // As in 3D.
    const size_t vid = t.vid(*this);
    const std::vector<size_t> ring = get_one_ring_fids_for_vertex(vid);
    for (const size_t fid : ring) {
        if (is_inverted_f(fid)) {
            // A neighbour that is not rounded can leave a face inverted in floats though it is
            // fine in exact arithmetic: there is nothing to optimise from.
            ++m_smooth_rejects.already_inverted;
            return false;
        }
    }
    const auto energy = vertex_energy(vid);
    if (!energy) return false;
    const bool front = m_vertex_extra[vid].m_is_on_offset;
    // Held: an input envelope pulls it (smoothing_energy_envelope()).
    const std::shared_ptr<SampleEnvelope> pull = smoothing_energy_envelope(vid);
    const bool held = pull != nullptr;
    const Vector2d x0 = m_vertex_attribute[vid].m_posf;

    // DEBUG_crossings (log-only): the ring measures this move can change -- of vid and of every
    // vertex that shares a front chord with it -- before the solve.
    std::vector<size_t> nb;
    std::vector<double> nb_before;
    if (front && m_offset_params.debug_crossings) {
        nb.push_back(vid);
        for (const Tuple& et : offset_surface_edges_live_at(vid)) {
            for (const size_t u : get_edge_vids(et)) nb.push_back(u);
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
    // once the corner's nearest segment changes; see store_band_minimisers().
    const int64_t nearest_x0 = m_band_segs ? m_band_segs->nearest(x0) : -1;
    const auto ring_after = [&]() {
        // The minimisers store_band_minimisers() would keep, tried and put back.
        std::vector<int64_t> kept;
        for (const size_t fid : ring) kept.push_back(m_face_extra[fid].band_seg);
        store_band_minimisers(ring, nearest_x0);
        const double e = energy_sum(ring);
        for (size_t i = 0; i < ring.size(); ++i) {
            const size_t fid = ring[i];
            m_face_extra[fid].band_seg = kept[i];
        }
        return e;
    };
    const bool exact = held && m_params.smoothing_mode == "exact";

    // THE SOLVE: E_V from the vertex's position, plus the envelope's exact distance term for a
    // held vertex in smoothing_mode "exact".
    std::shared_ptr<polysolve::nonlinear::Problem> objective = energy;
    if (exact) {
        auto sum = std::make_shared<optimization::EnergySum>();
        sum->add_energy(energy);
        sum->add_energy(
            std::make_shared<optimization::ExactDistanceEnergy2D>(
                pull,
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
    const Vector2d x = xv.head<2>();

    const auto ring_valid = [&]() {
        for (const size_t fid : ring) {
            if (is_inverted(fid)) return false; // exact
        }
        return true;
    };
    const auto refuse = [&](std::atomic<size_t>& counter) {
        set_vertex_position(vid, x0);
        ++counter;
        return false;
    };

    if (!held) {
        // The solve's point, kept when the ring stays valid and the veto passes. A failed solve
        // (a thrown line search, a non-finite iterate) keeps the start.
        if (!x.allFinite()) return refuse(m_smooth_rejects.quality);
        set_vertex_position(vid, x);
        if (!ring_valid()) return refuse(m_smooth_rejects.inverted);
        if (!energy_not_raised(ring_after(), ring_before)) return refuse(m_smooth_rejects.quality);
    } else if (!exact) {
        // Projected: solve in free space, project back onto the envelope, and walk back toward
        // the start, t = 1, 1/2, 1/4, ..., accepting the first projected candidate whose ring is
        // valid, whose tracked edges stay inside the containment envelope and whose ring's sum of
        // E_T passes the veto. Then, if none did, the nested pass: for each candidate, bisect
        // between the interpolated point and its projection for the longest acceptable step
        // toward the input. As in 3D.
        const auto acceptable = [&](const Vector2d& q) {
            set_vertex_position(vid, q);
            return ring_valid() && held_edges_contained(vid) &&
                   energy_not_raised(ring_after(), ring_before);
        };
        bool accepted = false;
        std::vector<Vector2d> interp, proj;
        if (x.allFinite()) {
            for (int k = 0; k < m_params.project_line_search_steps && !accepted; ++k) {
                const Vector2d p = x0 + std::pow(0.5, k) * (x - x0);
                Vector2d q;
                pull->nearest_point(p, q);
                interp.push_back(p);
                proj.push_back(q);
                accepted = acceptable(q);
            }
            for (size_t k = 0;
                 k < proj.size() && !accepted && m_params.project_line_search_nested_steps > 0;
                 ++k) {
                double lo = 0., hi = 1.;
                Vector2d best;
                bool found = false;
                for (int j = 0; j < m_params.project_line_search_nested_steps; ++j) {
                    const double mid = 0.5 * (lo + hi);
                    const Vector2d cand = interp[k] + mid * (proj[k] - interp[k]);
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
        if (!held_edges_contained(vid)) return refuse(m_smooth_rejects.envelope);
        if (!energy_not_raised(ring_after(), ring_before)) return refuse(m_smooth_rejects.quality);
    }

    store_band_minimisers(ring, nearest_x0);
    for (const size_t fid : ring) m_face_attribute[fid].m_quality = get_quality(fid);
    ++m_smooth_rejects.accepted;
    m_released_tube_dirty.store(true, std::memory_order_release);
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

double TopoOffsetTriMesh::front_move_alignment(const size_t vid) const
{
    // |cos| between the direction the placement is allowed to move the vertex in and the field
    // normal, which is the direction that actually reduces its distance to the level set.
    //
    // 1: the vertex moves along the field normal, so the 1-D Newton step the convergence test
    //    measures is the step toward the level set and a small step really does mean placed.
    // 0: the two are perpendicular. front_vertex_move_direction() returns the BOUNDARY TANGENT
    //    for a vertex an input envelope holds, and the test then measures a step that cannot
    //    reduce the distance at all, so the vertex reads as placed wherever it happens to sit.
    // Negative marks a direction or a gradient that does not exist.
    const Vector2d n = front_vertex_move_direction(vid);
    if (!(n.squaredNorm() > 0.)) return -2.;
    const Vector2d g = potential_for(vid).gradient(m_vertex_attribute[vid].m_posf);
    const double gn = g.norm();
    if (!(gn > 0.) || !std::isfinite(gn)) return -2.;
    return std::abs(n.normalized().dot(g / gn));
}

Vector2d TopoOffsetTriMesh::front_vertex_move_direction(const size_t vid) const
{
    // A front vertex held by an input envelope (it sits on a tag-region boundary or the domain
    // wall) may only move ALONG that boundary: its direction is the boundary's tangent, the mean
    // of its region-class surface edges' unit directions. Every other front vertex moves along
    // the field normal. Zero when neither is defined.
    if (smoothing_containment_envelope(vid)) {
        const Vector2d x = m_vertex_attribute[vid].m_posf;
        Vector2d t = Vector2d::Zero();
        int n = 0;
        for (const Tuple& e : get_one_ring_edges_for_vertex(vid)) {
            const size_t eid = e.eid(*this);
            if (!m_edge_attribute[eid].m_is_surface_fs || edge_is_offset(eid)) continue;
            const size_t va = e.vid(*this), vb = e.switch_vertex(*this).vid(*this);
            const size_t q = (va == vid) ? vb : va;
            Vector2d d = m_vertex_attribute[q].m_posf - x;
            if (!(d.norm() > 0.)) continue;
            d /= d.norm();
            if (n > 0 && d.dot(t) < 0.) d = -d; // the two edges point away from x: make them agree
            t += d;
            ++n;
        }
        if (n > 0 && t.norm() > 0.) return t / t.norm();
    }
    // The field normal. Its known failure: on the input's medial axis grad Phi is undefined, and a
    // front vertex at a concave input corner sits exactly there, equidistant from two walls, so
    // the line solve cannot reach the level set's corner point and oscillates. Do not replace it
    // with the polyline's own (Voronoi-weighted) normal, and do not add a detect-and-2-D-solve
    // escape at such vertices; both measured worse -- see git history of this file.
    return front_vertex_normal(vid);
}

Vector2d TopoOffsetTriMesh::front_vertex_normal(const size_t vid) const
{
    const Vector2d g = potential_for(vid).gradient(m_vertex_attribute[vid].m_posf);
    const double gn = g.norm();
    return (std::isfinite(gn) && gn > 0.) ? Vector2d(g / gn) : Vector2d::Zero();
}

} // namespace wmtk::components::topological_offset
