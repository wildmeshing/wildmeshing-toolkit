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

namespace {
/// One live front chord at x, as the offset term wants it: the chord's other end, and the
/// barycentric weights of the chord's stencil_order stencil. Only the weights are frozen -- the
/// sample points themselves slide with x and the field is read live -- see StencilEnergy2D. The
/// 3D twin is stencil_face_at().
template <typename Mesh>
bool stencil_edge_at(
    const Mesh& m,
    const typename Mesh::Tuple& e,
    const size_t vid,
    StencilEnergy2D::Edge& out)
{
    const auto vs = m.get_edge_vids(e);
    size_t q = 0;
    int k = 0;
    for (const size_t v : vs) {
        if (v == vid) continue;
        q = v;
        ++k;
    }
    if (k != 1) return false;
    const Vector2d x = m.m_vertex_attribute[vid].m_posf;
    out.q1 = m.m_vertex_attribute[q].m_posf;
    out.samples.clear();
    m.for_each_edge_sample(x, out.q1, [&](const Vector2d&, const double wa, const double wb) {
        StencilEnergy2D::Sample sm;
        sm.a = wa;
        sm.b = wb;
        out.samples.push_back(sm);
    });
    return !out.samples.empty();
}
} // namespace

bool TopoOffsetTriMesh::smooth_front_vertex(const Tuple& t)
{
    // See the header: the shared smoother with the offset's options. The whole objective arrives
    // through smoothing_extra_energy() -- w AMIPS over the ring and the offset terms at
    // offset_term_weight(), tri_energy()'s two parts -- and w_amips 0 keeps the smoother from
    // adding an AMIPS term of its own; a front vertex an input envelope pins carries no offset
    // term. The smoother holds a front vertex to no envelope (see
    // smoothing_containment_envelope()), so neither the projected path nor the containment check
    // applies -- solve, exact inversion test, then the veto on tri_energy() below. As in 3D.
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
    // THE FRONT SMOOTHER'S VETO (see tri_energy()): the max of tri_energy() over the vertex's
    // one-ring may not rise, under offset_front_smooth_veto. A tie passes. Read before the solve
    // and after it, on the mesh: a smoothing move changes no label and moves one vertex, so the
    // one-ring holds every face whose energy it can change. Refused, TriMesh::smooth_vertex()
    // rolls back the position and the ring's stored qualities. The smoother has counted the move
    // accepted by then; it is recounted as a quality refusal, so the counters still partition the
    // attempts.
    const std::vector<size_t> ring = get_one_ring_fids_for_vertex(vid);
    const bool veto = m_offset_params.offset_front_smooth_veto;
    const double before = veto ? max_tri_energy(ring) : 0.;
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
    if (veto) ++m_front_veto_asked;
    if (veto && !(max_tri_energy(ring) <= before)) { // a NaN refuses
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
    // The front's path with the repulsion term in place of the chord terms: the whole objective
    // arrives through smoothing_extra_energy() -- w AMIPS over the ring and repulsion_energy(vid)
    // -- and w_amips 0 keeps the smoother from adding an AMIPS term of its own; no engine veto. A
    // repulsion vertex is never envelope-held (repulsion_smoothing() leaves those to TriWild's
    // rule), so the solve has no envelope term and no containment check. As in 3D.
    const size_t vid = t.vid(*this);
    optimization::SmoothVertexOptions opts;
    opts.w_amips = 0.;
    opts.w_envelope = m_params.w_envelope;
    opts.s_amips = m_s_amips;
    opts.s_envelope = m_s_envelope;
    opts.two_stage = false;
    opts.quality_veto = false;
    smoothing_solver();
    // The front veto's rule (see tri_energy()): the max of the per-tri energy over the ring --
    // which includes the repulsion terms of the faces that carry them, O(v) among them -- may
    // not rise (a tie passes). Read before the solve and after it; refused,
    // TriMesh::smooth_vertex() rolls the move back.
    const std::vector<size_t> ring = get_one_ring_fids_for_vertex(vid);
    const bool veto = m_offset_params.offset_front_smooth_veto;
    const double before = veto ? max_tri_energy(ring) : 0.;
    const bool ok = smooth_vertex_2d_counted(t, opts, m_newton_repulsion);
    if (!ok) return false;
    if (veto) ++m_repulsion_veto_asked;
    if (veto && !(max_tri_energy(ring) <= before)) { // a NaN refuses
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
    const size_t vid,
    const Vector2d& x) const
{
    // The one-ring's AMIPS with the vertex first in every cell (what AMIPS2D_jacobian
    // differentiates against), at weight 1, plus the offset terms on the vertex's own region's
    // field. Diagnostic: front_vertex_normal_gradient() differentiates it. As in 3D.
    std::vector<std::array<double, 6>> cells;
    std::vector<RestAMIPSEnergy2D::Cell> plastic_cells; // deform_others: increments only
    for (const size_t fid : get_one_ring_fids_for_vertex(vid)) {
        const std::array<size_t, 3> vs = oriented_tri_vids(fid);
        int k = 0;
        while (k < 3 && vs[k] != vid) ++k;
        if (k == 3) continue;
        const Vector2d& a = m_vertex_attribute[vs[(k + 1) % 3]].m_posf;
        const Vector2d& b = m_vertex_attribute[vs[(k + 2) % 3]].m_posf;
        // A plastic ring face brakes the front only by its increment since the group started
        // (rest-shape AMIPS on the group-start rest); judged equilateral it becomes a permanent
        // brake that parks the front at an elastic equilibrium. Band faces stay equilateral,
        // except a band face that is a released object's material.
        const FaceExtra2d& fx = m_face_extra[fid];
        if ((face_is_plastic(fid) || face_is_released_band(fid)) && fx.rest_valid) {
            Eigen::Matrix2d R;
            R.col(0) = fx.rest_pos[(k + 1) % 3] - fx.rest_pos[k];
            R.col(1) = fx.rest_pos[(k + 2) % 3] - fx.rest_pos[k];
            if (R.determinant() > 0.) {
                RestAMIPSEnergy2D::Cell c;
                c.q1 = a;
                c.q2 = b;
                c.rest_inv = R.inverse();
                plastic_cells.push_back(c);
                continue;
            }
        }
        cells.push_back({{x.x(), x.y(), a.x(), a.y(), b.x(), b.y()}});
    }
    // AMIPS at weight 1 beside the offset term in units of the tolerance, the rest-shape term of
    // a plastic face 1:1 with the AMIPS it replaces. As in 3D.
    const double amips_w = 1.;
    auto sum = std::make_shared<optimization::EnergySum>();
    if (!cells.empty())
        sum->add_energy(std::make_shared<optimization::AMIPSEnergy2D>(cells, amips_w));
    if (!plastic_cells.empty())
        sum->add_energy(std::make_shared<RestAMIPSEnergy2D>(std::move(plastic_cells), amips_w));
    sum->add_energy(front_energy(vid, potential_ptr_for(vid)));
    return sum;
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTriMesh::front_energy(
    const size_t vid,
    const std::shared_ptr<const OffsetPotential2D>& pot) const
{
    // 1 / front_conv_frac()^2: the squared relative error (Phi - c)/c becomes the squared error
    // in units of the tolerance, so the term below is sum_e O(e) -- for the euclidean field the
    // very terms the per-cell energy tri_energy() carries on these chords' band faces, and the
    // ring measure's n_v r_v^2. As in 3D.
    const double w_off = offset_term_weight();
    auto sum = std::make_shared<optimization::EnergySum>();
    // THE offset term, and the only one: the mean squared relative error over each incident
    // chord's stencil, summed over the chords. The stencil contains the chord's ends, so the
    // moving vertex's own residual is in it once per incident chord. See StencilEnergy2D. Every
    // chord weighs 1.
    std::vector<StencilEnergy2D::Edge> stencil_edges;
    for (const Tuple& e : offset_surface_edges_live_at(vid)) {
        StencilEnergy2D::Edge se;
        if (!stencil_edge_at(*this, e, vid, se)) continue;
        stencil_edges.push_back(std::move(se));
    }
    if (!stencil_edges.empty()) {
        sum->add_energy(std::make_shared<StencilEnergy2D>(pot, std::move(stencil_edges), w_off));
    }
    return sum;
}

} // namespace wmtk::components::topological_offset
