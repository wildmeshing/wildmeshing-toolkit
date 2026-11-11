#include "TopoOffsetTriMesh.h"

#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * Front placement: an offset-front vertex is placed by the shared 2-D smoother with the offset's
 * energy (see smooth_front_vertex()). Everything else is smoothed by Optimize2d.cpp. The 3D twin
 * is FrontSmooth3d.cpp.
 */

bool TopoOffsetTriMesh::smooth_front_vertex(const Tuple& t)
{
    // See the header: the shared smoother with the offset's options. The whole objective arrives
    // through smoothing_extra_energy() -- the vertex's part of the energy, the AMIPS term over the
    // ring and the band term of its band faces -- and w_amips 0 keeps the smoother from adding an
    // AMIPS term of its own; a front vertex held on the input carries no band term. A front vertex
    // on no region boundary is held to no envelope (see smoothing_energy_envelope()), so neither
    // the projected path nor the containment check applies -- solve, exact inversion test, then
    // the veto on tri_energy() below. As in 3D. Where 2D differs: a front vertex held on a region
    // boundary takes the engine's projected step, which compares the ring's max AMIPS (the 2D
    // engine has no ring hook, see tri_energy()), and then this veto; 3D's projected step compares
    // the ring's sum itself.
    const size_t vid = t.vid(*this);
    optimization::SmoothVertexOptions opts;
    opts.w_amips = 0.;
    opts.w_envelope = m_params.w_envelope;
    opts.s_amips = m_s_amips;
    opts.s_envelope = m_s_envelope;
    opts.two_stage = false;
    // The engine's veto compares AMIPS alone, and a front vertex must be free to worsen its
    // ring's shape on the way to the level set: off. The veto below is on the energy instead.
    opts.quality_veto = false;
    polysolve::nonlinear::Solver& solver = smoothing_solver();
    // THE FRONT SMOOTHER'S VETO (see tri_energy()), on every solve: the sum of tri_energy() over
    // the vertex's one-ring may not rise. A tie passes, as in the engine's AMIPS veto. Read
    // before the solve and after it, on the mesh: a smoothing move changes no label and moves one
    // vertex, so the one-ring holds every face whose energy it can change. Refused,
    // TriMesh::smooth_vertex() rolls back the position and the ring's stored qualities. The
    // smoother has counted the move accepted by then; it is recounted as a quality refusal, so
    // the counters still partition the attempts. "exact_swept": the AMIPS part alone before, and
    // the move accepted by swept_accept(), the band term's exact change with halvings. As in 3D.
    // Where 2D differs: a front vertex held on a region boundary takes the engine's projected
    // step (the engine's condition: a pull envelope, the options' default projected mode), whose
    // position is tested alone, without halving (EXACT SWEPT CHANGE, where 2D differs).
    const std::vector<size_t> ring = get_one_ring_fids_for_vertex(vid);
    const Vector2d x0 = m_vertex_attribute[vid].m_posf;
    const bool swept = swept_rule();
    const bool projected =
        opts.smoothing_mode == optimization::SmoothVertexOptions::SmoothingMode::Projected &&
        smoothing_energy_envelope(vid) != nullptr;
    const double before = tri_energy_sum(ring, !swept);
    // DEBUG_crossings (log-only): the ring measures this move can change -- of vid and of every
    // vertex that shares a front chord with it -- before the solve.
    std::vector<size_t> nb;
    std::vector<double> nb_before;
    if (m_offset_params.debug_crossings) {
        nb.push_back(vid);
        for (const Tuple& et : offset_surface_edges_live_at(vid)) {
            for (const size_t u : get_edge_vids(et)) nb.push_back(u);
        }
        wmtk::vector_unique(nb);
        for (const size_t u : nb) nb_before.push_back(ring_measure_at(u));
    }
    // The solve is folded into the pass's m_newton_front; under DEBUG_output its outcome is also
    // pinned to vid (m_front_solve_log). At most one solve (two_stage is off), none when the ring
    // is already inverted; nothing after the solve touches the solver, so it still holds that
    // solve's state here.
    bool solved = false;
    const bool ok = smooth_vertex_2d_counted(t, opts, m_newton_front, &solved);
    if (solved && m_offset_params.debug_output) {
        std::lock_guard<std::mutex> lock(m_front_solve_log_mutex);
        m_front_solve_log.push_back(
            {vid, int(solver.current_criteria().iterations), int(solver.status()) + 1});
    }
    if (!ok) {
        return false;
    }
    {
        // Diagnostic: the solve's final gradient norm and its ratio to the first one.
        const auto& cr = solver.current_criteria();
        const auto bin = [](double v, int lo) {
            if (!(v > 0.)) return 0;
            return std::clamp(int(std::floor(std::log10(v))) - lo + 1, 0, kGradBins - 1);
        };
        ++m_front_grad_abs[size_t(bin(cr.gradNorm, -14))];
        ++m_front_grad_rel[size_t(bin(cr.relGradNorm, -14))];
    }
    ++m_front_veto_asked;
    if (swept ? !swept_accept(
                    vid,
                    ring,
                    x0,
                    before,
                    projected ? m_swept_projected : m_swept_front,
                    projected ? 0 : kSweptHalvings)
              : !(tri_energy_sum(ring) <= before)) { // a NaN refuses
        ++m_front_veto_fired;
        --m_smooth_rejects.accepted;
        ++m_smooth_rejects.quality;
        return false;
    }
    m_released_tube_dirty.store(true, std::memory_order_release);
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
}

bool TopoOffsetTriMesh::smooth_repulsion_vertex(const Tuple& t)
{
    // The front's path with the repulsion term in place of the band term: the whole objective
    // arrives through smoothing_extra_energy() -- the AMIPS term over the ring and
    // repulsion_energy(vid) -- and w_amips 0 keeps the smoother from adding an AMIPS term of its
    // own; no engine veto. A repulsion vertex is never envelope-held (repulsion_smoothing() leaves
    // those to TriWild's rule), so the solve has no envelope term and no containment check. As in
    // 3D.
    const size_t vid = t.vid(*this);
    optimization::SmoothVertexOptions opts;
    opts.w_amips = 0.;
    opts.w_envelope = m_params.w_envelope;
    opts.s_amips = m_s_amips;
    opts.s_envelope = m_s_envelope;
    opts.two_stage = false;
    opts.quality_veto = false;
    smoothing_solver();
    // The front veto's rule (see tri_energy()): the sum of the per-tri energy over the ring --
    // which includes the repulsion terms of the faces that carry them, O(v) among them -- may
    // not rise (a tie passes). Read before the solve and after it; refused,
    // TriMesh::smooth_vertex() rolls the move back. As in 3D.
    const std::vector<size_t> ring = get_one_ring_fids_for_vertex(vid);
    const double before = tri_energy_sum(ring);
    const bool ok = smooth_vertex_2d_counted(t, opts, m_newton_repulsion);
    if (!ok) return false;
    ++m_repulsion_veto_asked;
    if (!(tri_energy_sum(ring) <= before)) { // a NaN refuses
        ++m_repulsion_veto_fired;
        --m_smooth_rejects.accepted;
        ++m_smooth_rejects.quality;
        return false;
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

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTriMesh::front_objective(
    const size_t vid) const
{
    // smoothing_extra_energy() at a front vertex that carries the offset term, with the offset
    // term included unconditionally: the AMIPS term over the one-ring, the plastic faces'
    // rest-shape AMIPS, and the band term (front_energy()). As in 3D, whose plastic cells are in
    // its AMIPS term.
    auto sum = std::make_shared<optimization::EnergySum>();
    sum->add_energy(amips3_energy(vid));
    if (const auto rest = rest_energy_for_vertex(vid)) sum->add_energy(rest);
    sum->add_energy(front_energy(vid));
    return sum;
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTriMesh::front_energy(
    const size_t vid) const
{
    // The vertex's part of the energy's band term; see band_volume_energy_at(). 3D's
    // EXPERIMENTAL_band_volume_rule "surface" puts the face term of the offset faces through vid
    // here (SurfaceOffsetError3D); 2D refuses that rule (band_rule_exact()).
    auto sum = std::make_shared<optimization::EnergySum>();
    if (const auto bv = band_volume_energy_at(vid)) sum->add_energy(bv);
    return sum;
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTriMesh::band_volume_energy_at(
    const size_t vid) const
{
    // band_cell_term() of each band face of the ring, x moving (BandVolumeEnergy2D): the face's
    // other two corners in its winding after vid, and under "exact_swept" its piece.
    const bool swept = swept_rule();
    std::vector<BandVolumeEnergy2D::Cell> cells;
    std::vector<int> cell_piece;
    for (const size_t fid : get_one_ring_fids_for_vertex(vid)) {
        if (!face_is_offset_band(fid)) continue;
        const std::array<size_t, 3> vs = oriented_tri_vids(fid);
        int k = 0;
        while (k < 3 && vs[size_t(k)] != vid) ++k;
        if (k == 3) continue;
        BandVolumeEnergy2D::Cell c;
        c.q1 = m_vertex_attribute[vs[size_t((k + 1) % 3)]].m_posf;
        c.q2 = m_vertex_attribute[vs[size_t((k + 2) % 3)]].m_posf;
        cells.push_back(c);
        if (swept) cell_piece.push_back(place_piece(vs.data(), 3));
    }
    if (cells.empty()) return nullptr;
    const bool exact = band_rule_exact(); // refuses a rule with no 2D port, as band_cell_term()
    // EXPERIMENTAL_band_volume_rule "exact_swept": the search model is the function the move is
    // judged on (BandVolumeEnergy2D::set_swept()): one per piece r, over the boundary edges of
    // B_r through vid in the lists and order swept_accept() reads (band_swept_edges()), with the
    // ring's band faces of piece r and r's distance and integral, summed where there are several
    // (EXACT SWEPT CHANGE, several entries). None: no band term, the move's band change being 0.
    // As in 3D.
    if (swept) {
        std::vector<SweptEdgesOfPiece> pieces;
        band_swept_edges(vid, m_vertex_attribute[vid].m_posf, pieces);
        auto& models = m_swept_model.local();
        models.clear();
        if (pieces.empty()) return nullptr;
        auto sum = pieces.size() > 1 ? std::make_shared<optimization::EnergySum>() : nullptr;
        std::shared_ptr<BandVolumeEnergy2D> e;
        for (SweptEdgesOfPiece& q : pieces) {
            std::vector<BandVolumeEnergy2D::Cell> cq;
            for (size_t i = 0; i < cells.size(); ++i) {
                if (cell_piece[i] == q.piece) cq.push_back(cells[i]);
            }
            const double delta = region_delta(q.piece);
            const auto& band_exact = band_exact_for(q.piece);
            e = std::make_shared<BandVolumeEnergy2D>(
                band_exact,
                std::move(cq),
                m_vertex_attribute[vid].m_posf,
                delta);
            e->set_swept(band_exact, delta, std::move(q.edges), &m_swept_derivs);
            models.push_back(e);
            if (sum) sum->add_energy(e);
        }
        if (sum) return sum;
        return e;
    }
    // The ring's piece's distance and integral (place_piece(); the run's and the whole complex's
    // with one entry): the faces of a band vertex's ring are its piece's band. As in 3D.
    const int piece = place_piece(&vid, 1);
    const double delta = region_delta(piece);
    const auto& band_exact = band_exact_for(piece);
    auto e = std::make_shared<BandVolumeEnergy2D>(
        band_exact,
        std::move(cells),
        m_vertex_attribute[vid].m_posf,
        delta);
    // EXPERIMENTAL_band_volume_rule "exact": each face's closed-form integral, as
    // band_cell_term(); "front_flux" takes it too, as band_cell_term() does. 3D gives
    // "front_flux" its front form (BandVolumeEnergy3D::set_front_flux()), not ported.
    if (exact) e->set_exact_integral(band_exact, delta);
    return e;
}

} // namespace wmtk::components::topological_offset
