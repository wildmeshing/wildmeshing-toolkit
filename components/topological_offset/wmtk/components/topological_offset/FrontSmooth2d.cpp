#include "TopoOffsetTriMesh.h"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * Phase B's front placement: an offset-front vertex is placed by the shared 2-D smoother with the
 * offset's energy (see smooth_front_vertex_phase_b()). Everything else is smoothed by
 * Optimize2d.cpp. The 3D twin is FrontSmooth3d.cpp.
 */

bool TopoOffsetTriMesh::smooth_front_vertex_phase_b(const Tuple& t)
{
    // See the header: the shared smoother with the offset's options. The offset terms arrive
    // through smoothing_extra_energy(), AMIPS is weighted as in TriOptimizerMesh::smooth_after(),
    // and a front vertex has no envelope in Phase B, so neither the projected path nor the
    // containment check applies -- solve, exact inversion test, done.
    optimization::SmoothVertexOptions opts;
    opts.w_amips = m_params.w_amips;
    opts.w_envelope = m_params.w_envelope;
    opts.s_amips = m_s_amips;
    opts.s_envelope = m_s_envelope;
    opts.two_stage = false;
    opts.quality_veto = false;
    auto& solver = m_solver.local();
    if (!solver) {
        solver = polysolve::nonlinear::Solver::create(
            optimization::basic_nonlinear_solver_params,
            optimization::basic_linear_solver_params,
            1,
            opt_logger());
    }
    const bool ok = optimization::smooth_vertex_2d(*this, t, opts, solver, &m_smooth_rejects);
    if (ok) m_released_tube_dirty.store(true, std::memory_order_release);
    return ok;
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

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTriMesh::phase_b_front_objective(
    const size_t vid,
    const Vector2d& x) const
{
    // The one-ring's AMIPS with the vertex first in every cell (what AMIPS2D_jacobian
    // differentiates against), weighted as the shared smoother weights it, plus the offset terms
    // on the vertex's own region's field.
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
        // except a band cell that is a released object's material (face_is_released_band): the
        // front pushing through the overlap must do work against that material too. Only the
        // front placement reads this; the band's interior smoothing stays equilateral.
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
    const double amips_w = m_params.w_amips > 0 ? m_s_amips * m_params.w_amips : 1.0;
    auto sum = std::make_shared<optimization::EnergySum>();
    if (m_params.w_amips > 0 && !cells.empty())
        sum->add_energy(std::make_shared<optimization::AMIPSEnergy2D>(cells, amips_w));
    if (!plastic_cells.empty())
        sum->add_energy(std::make_shared<RestAMIPSEnergy2D>(std::move(plastic_cells), amips_w));
    sum->add_energy(phase_b_front_energy(vid, potential_ptr_for(vid)));
    return sum;
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTriMesh::phase_b_front_energy(
    const size_t vid,
    const std::shared_ptr<const OffsetPotential2D>& pot) const
{
    const double w_off = 1. - m_params.w_amips;
    auto sum = std::make_shared<optimization::EnergySum>();
    // The same under both front_measure values. The 3D twin weights its stencil energy by face
    // area under "vertex_ring", so that the smoother minimises what the ring measure tests; this
    // offset term is OffsetEnergy2D, the vertex's own residual, with no per-chord term of the
    // chord measure to weight.
    // Gauss-Newton Hessian (the default); the exact Hessian adds r grad^2 Phi and buys nothing.
    sum->add_energy(std::make_shared<OffsetEnergy2D>(pot, w_off, true, true));
    return sum;
}

} // namespace wmtk::components::topological_offset
