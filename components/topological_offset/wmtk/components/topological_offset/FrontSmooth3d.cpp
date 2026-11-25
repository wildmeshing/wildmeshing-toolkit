#include "TopoOffsetTetMesh.h"
#include <wmtk/utils/Profiler.hpp>

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
 * The front's objective and placement measures, and the repulsion smoother. Every vertex the loop
 * smooths, the front's included, is placed by smooth_vertex_under_energy() in Optimize3d.cpp. The
 * 2D twin is FrontSmooth2d.cpp.
 */

bool TopoOffsetTetMesh::smooth_repulsion_vertex(const Tuple& t)
{
    WMTK_PROF_CALL("smooth/repulsion_vertex");
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
    WMTK_PROF_CALL("smooth/build_front_energy");
    // The vertex's part of the energy's band term; see band_volume_energy_at().
    auto sum = std::make_shared<optimization::EnergySum>();
    // EXPERIMENTAL_band_volume_rule "surface": the face term of the live offset faces through
    // vid, vid moving (SurfaceOffsetError3D), in place of the band term.
    if (surface_energy()) {
        if (!m_offset_potential) return sum;
        std::vector<std::array<size_t, 3>> faces;
        live_offset_faces_at(vid, faces);
        std::vector<SurfaceOffsetError3D::Face> fs;
        fs.reserve(faces.size());
        const auto& VA = std::as_const(m_vertex_attribute);
        for (const auto& f : faces) {
            std::array<size_t, 2> o{{0, 0}};
            int n = 0;
            for (const size_t u : f) {
                if (u != vid && n < 2) o[size_t(n++)] = u;
            }
            if (n != 2) continue;
            fs.push_back({VA[o[0]].m_posf, VA[o[1]].m_posf});
        }
        if (!fs.empty()) {
            // Several entries of offsets: the vertex's piece's field and bar (one entry: the whole
            // complex's and the run's).
            const int piece = m_offset_params.offsets.size() > 1 ? place_piece(&vid, 1) : -1;
            sum->add_energy(
                std::make_shared<SurfaceOffsetError3D>(
                    piece >= 0 ? potential_ptr_for(vid) : m_offset_potential,
                    std::move(fs),
                    face_stencil(),
                    face_stencil_weights(),
                    surface_term_weight(piece)));
        }
        return sum;
    }
    if (const auto bv = band_volume_energy_at(vid)) sum->add_energy(bv);
    return sum;
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTetMesh::band_volume_energy_at(
    const size_t vid) const
{
    WMTK_PROF_CALL("smooth/build_band_volume_energy");
    // band_cell_term() of each band cell of the ring, x moving (BandVolumeEnergy3D), and under
    // "exact_swept" its piece. EXPERIMENTAL_band_volume_rule "surface": no cell carries a band
    // term.
    if (surface_energy()) return nullptr;
    const bool swept = swept_rule();
    std::vector<BandVolumeEnergy3D::Cell> cells;
    std::vector<int> cell_piece;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        if (!cell_is_offset_band(tid)) continue;
        BandVolumeEnergy3D::Cell c;
        int k = 0;
        const std::array<size_t, 4> vs = oriented_tet_vids(tid);
        for (const size_t u : vs) {
            if (u == vid) continue;
            (k == 0 ? c.q1 : k == 1 ? c.q2 : c.q3) = m_vertex_attribute[u].m_posf;
            ++k;
        }
        if (k != 3) continue;
        cells.push_back(c);
        if (swept) cell_piece.push_back(place_piece(vs.data(), 4));
    }
    if (cells.empty()) return nullptr;
    // EXPERIMENTAL_band_volume_rule "exact_swept": the search model is the function the move is
    // judged on (BandVolumeEnergy3D::set_swept()): one per piece r, over the boundary faces of
    // B_r through vid in the lists and order swept_accept() reads (band_swept_faces()), with the
    // ring's band cells of piece r and r's distance, engine and integral, summed where there are
    // several (EXACT SWEPT CHANGE, several entries). None: no band term, the move's band change
    // being 0.
    if (swept) {
        std::vector<SweptFacesOfPiece> pieces;
        band_swept_faces(vid, m_vertex_attribute[vid].m_posf, pieces);
        auto& models = m_swept_model.local();
        models.clear();
        if (pieces.empty()) return nullptr;
        auto sum = pieces.size() > 1 ? std::make_shared<optimization::EnergySum>() : nullptr;
        std::shared_ptr<BandVolumeEnergy3D> e;
        for (SweptFacesOfPiece& q : pieces) {
            std::vector<BandVolumeEnergy3D::Cell> cq;
            for (size_t i = 0; i < cells.size(); ++i) {
                if (cell_piece[i] == q.piece) cq.push_back(cells[i]);
            }
            const double delta = region_delta(q.piece);
            e = std::make_shared<BandVolumeEnergy3D>(
                envelope_for(q.piece),
                m_input_complex_pad,
                std::move(cq),
                m_vertex_attribute[vid].m_posf,
                delta);
            e->set_swept(band_exact_for(q.piece), delta, std::move(q.faces), &m_swept_derivs);
            models.push_back(e);
            if (sum) sum->add_energy(e);
        }
        if (sum) return sum;
        return e;
    }
    // The ring's piece's distance, engine and integral (place_piece(); the run's and the whole
    // complex's with one entry): the cells of a band vertex's ring are its piece's band.
    const int piece = place_piece(&vid, 1);
    const double delta = region_delta(piece);
    const auto& band_exact = band_exact_for(piece);
    auto e = std::make_shared<BandVolumeEnergy3D>(
        envelope_for(piece),
        m_input_complex_pad,
        std::move(cells),
        m_vertex_attribute[vid].m_posf,
        delta);

    // EXPERIMENTAL_band_volume_rule "exact": each cell's closed-form integral, as band_cell_term().
    if (m_offset_params.band_volume_rule == "exact") {
        e->set_exact_integral(band_exact, delta);
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
        e->set_front_flux(band_exact, delta, std::move(faces));
    }
    return e;
}

} // namespace wmtk::components::topological_offset
