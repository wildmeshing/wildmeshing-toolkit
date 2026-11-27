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
 * The front's objective and placement measures, and the repulsion smoother. Every vertex the loop
 * smooths, the front's included, is placed by smooth_vertex_under_energy() in Optimize2d.cpp. The
 * 3D twin is FrontSmooth3d.cpp.
 */

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
    // term included unconditionally: the AMIPS term over the one-ring and the band term
    // (front_energy()). As in 3D.
    auto sum = std::make_shared<optimization::EnergySum>();
    sum->add_energy(amips3_energy(vid));
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
