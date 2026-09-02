#include "TopoOffsetTriMesh.h"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * Phase B's front placement: an offset-front vertex is moved by a one-dimensional Newton solve
 * along the offset field's gradient, widened to the shared 2-D smoother where the alignment term
 * traps that line. Everything else is smoothed by Optimize2d.cpp. The 3D twin is Smooth.cpp.
 */

namespace {
/// The front vertex's objective restricted to the line x(s) = x0 + s n: the same energy, one
/// unknown. Every hook the solver's line search asks -- step validity (the AMIPS inversion
/// guard), the step cap, begin/end, solution_changed -- is forwarded on the mapped points.
class LineProblem2D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;
    LineProblem2D(
        std::shared_ptr<polysolve::nonlinear::Problem> p,
        const Vector2d& x0,
        const Vector2d& n)
        : m_p(std::move(p))
        , m_x0(x0)
        , m_n(n)
    {}
    Eigen::VectorXd at(const TVector& s) const { return Eigen::VectorXd(m_x0 + s(0) * m_n); }
    double value(const TVector& s) override { return m_p->value(at(s)); }
    void gradient(const TVector& s, TVector& g) override
    {
        Eigen::VectorXd g2(2);
        m_p->gradient(at(s), g2);
        g.resize(1);
        g(0) = m_n.dot(Vector2d(g2));
    }
    void hessian(const TVector&, THessian&) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& s, Eigen::MatrixXd& h) override
    {
        Eigen::MatrixXd h2(2, 2);
        m_p->hessian(at(s), h2);
        h.resize(1, 1);
        h(0, 0) = m_n.dot(h2 * m_n);
    }
    bool is_step_valid(const TVector& s0, const TVector& s1) override
    {
        return m_p->is_step_valid(at(s0), at(s1));
    }
    double max_step_size(const TVector& s0, const TVector& s1) override
    {
        return m_p->max_step_size(at(s0), at(s1));
    }
    void line_search_begin(const TVector& s0, const TVector& s1) override
    {
        m_p->line_search_begin(at(s0), at(s1));
    }
    void line_search_end() override { m_p->line_search_end(); }
    void solution_changed(const TVector& s) override { m_p->solution_changed(at(s)); }

private:
    std::shared_ptr<polysolve::nonlinear::Problem> m_p;
    Vector2d m_x0, m_n;
};
} // namespace

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
    if (!m_offset_params.front_normal_projection) {
        const bool ok = optimization::smooth_vertex_2d(*this, t, opts, solver, &m_smooth_rejects);
        if (ok) m_released_tube_dirty.store(true, std::memory_order_release);
        return ok;
    }

    // Normal-only placement is a one-dimensional solve: the same objective the 2-D path minimises
    // (phase_b_front_objective), restricted to the line along n = grad Phi / |grad Phi|, with the
    // same solver, the same line search and the same accept test as the shared smoother's
    // no-envelope path (exact inversion of the ring; no envelope, no veto). Nothing is added to
    // the energy -- do not re-add a stiff tangential penalty standing in for the restriction; it
    // wrecks the Hessian's conditioning and was never what "along the normal" meant.
    const size_t vid = t.vid(*this);
    const Vector2d n = front_vertex_move_direction(vid);
    if (!(n.squaredNorm() > 0.)) { // no direction here: the 2-D solve is the fallback
        const bool ok = optimization::smooth_vertex_2d(*this, t, opts, solver, &m_smooth_rejects);
        if (ok) m_released_tube_dirty.store(true, std::memory_order_release);
        return ok;
    }
    // Widen the motion where the alignment term traps the 1-D solve: at a jog of the outline the
    // alignment residual's fixing motion is tangential, which the radial line projects away, so
    // where it opposes the placement pull it cancels it and the jog freezes. The trap is the
    // fight, not the perpendicular edge -- on a travelling staircase the same radial stretching
    // preference points with placement and is a travel engine -- hence both conditions in the
    // predicate. Do not gate on edge orientation alone; measured worse -- see git history of this
    // file. Seam edges run parallel to the field, so a pressed seam never takes this path.
    if (m_offset_params.front_alignment_energy && front_vertex_alignment_traps_1d_solve(vid)) {
        const bool ok = optimization::smooth_vertex_2d(*this, t, opts, solver, &m_smooth_rejects);
        if (ok) m_released_tube_dirty.store(true, std::memory_order_release);
        return ok;
    }
    const std::shared_ptr<SampleEnvelope> hold = smoothing_containment_envelope(vid);
    const std::vector<size_t>& locs = get_one_ring_fids_for_vertex(t);
    for (const size_t fid : locs) {
        if (is_inverted_f(fid)) {
            ++m_smooth_rejects.already_inverted;
            return false;
        }
    }
    const Vector2d x0 = smoothing_position(vid);
    auto line = std::make_shared<LineProblem2D>(phase_b_front_objective(vid, x0), x0, n);
    Eigen::VectorXd s = Eigen::VectorXd::Zero(1);
    try {
        solver->minimize(*line, s);
    } catch (const std::exception&) {
    }
    set_smoothing_position(vid, Vector2d(x0 + s(0) * n));
    if (hold) { // the boundary's tube, on the region edges the move reshaped -- as the shared
                // smoother
        const Vector2d p = smoothing_position(vid);
        for (const Tuple& e : get_one_ring_edges_for_vertex(vid)) {
            const size_t eid = e.eid(*this);
            if (!m_edge_attribute[eid].m_is_surface_fs || edge_is_offset(eid)) continue;
            const size_t va = e.vid(*this), vb = e.switch_vertex(*this).vid(*this);
            const Vector2d q = m_vertex_attribute[(va == vid) ? vb : va].m_posf;
            if (hold->is_outside(std::array<Vector2d, 2>{{p, q}})) {
                set_smoothing_position(vid, x0);
                ++m_smooth_rejects.envelope;
                return false;
            }
        }
    }
    for (const size_t fid : locs) {
        if (is_inverted(fid)) {
            set_smoothing_position(vid, x0);
            ++m_smooth_rejects.inverted;
            return false;
        }
    }
    for (const size_t fid : locs) m_face_attribute[fid].m_quality = get_quality(fid);
    ++m_smooth_rejects.accepted;
    m_released_tube_dirty.store(true, std::memory_order_release); // the boundary may have moved
    return true;
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

bool TopoOffsetTriMesh::front_vertex_alignment_traps_1d_solve(const size_t vid) const
{
    // Three conditions, all required -- see the use in smooth_front_vertex_phase_b().
    //
    // (1) An incident live front edge at or past perpendicular to the field. Only there does the
    //     alignment residual sit on its plateau, where its fixing motion is tangential. The edge
    //     normal is built exactly as phase_b_front_energy() builds it, so this predicate and the
    //     energy cannot disagree.
    // (2) The alignment term's 1-D gradient OPPOSES the placement term's along the vertex's move
    //     direction. On a travelling staircase both point outward and the radial stretching is a
    //     travel engine, not a trap; the trap is specifically the fight.
    const std::shared_ptr<const OffsetPotential2D> pot = potential_ptr_for(vid);
    if (!pot) return false;
    const Vector2d x = m_vertex_attribute[vid].m_posf;
    // (3) And stationary off the level set: residual over the tube while the remaining 1-D step
    //     is under the vertex test's bar (checked last, it is the expensive one). Mid-travel the
    //     alignment gradient opposes placement too, weakly, and the 1-D solve drives through it,
    //     so (1)+(2) alone would cut in at travelling staircase vertices as well. A trapped
    //     vertex is one the 1-D solve has finished with, at a position off the level set.
    const double rho = pot->residual_length(x);
    const double tube =
        std::max(m_offset_params.offset_envelope_rel * m_offset_params.target_distance, 1e-12);
    if (!std::isfinite(rho) || rho <= tube) return false;
    const double s = m_offset_params.offset_field == "euclidean" ? 1. : -1.;
    bool past_perpendicular = false;
    std::vector<AlignEnergy2D::Edge> edges;
    for (const Tuple& e : get_one_ring_edges_for_vertex(vid)) {
        if (!edge_is_offset_surface_live(e)) continue;
        const size_t va = e.vid(*this), vb = e.switch_vertex(*this).vid(*this);
        const size_t q = (va == vid) ? vb : va;
        const std::optional<Tuple> opp = e.switch_face(*this);
        if (!opp) continue;
        const size_t fa = e.fid(*this), fb = opp->fid(*this);
        const size_t band_f = face_is_offset_band(fa) ? fa : fb;
        const auto vs = oriented_tri_vids(band_f);
        const Vector2d c = (m_vertex_attribute[vs[0]].m_posf + m_vertex_attribute[vs[1]].m_posf +
                            m_vertex_attribute[vs[2]].m_posf) /
                           3.;
        const Vector2d d = m_vertex_attribute[q].m_posf - x;
        if (!(d.norm() > 0.)) continue;
        const Vector2d n_plus(-d.y(), d.x()); // R90 (q - x)
        const Vector2d mid = 0.5 * (x + m_vertex_attribute[q].m_posf);
        const double sigma = n_plus.dot(mid - c) >= 0. ? 1. : -1.;
        const Vector2d g = pot->gradient(mid);
        const double gn = g.norm();
        if (!std::isfinite(gn) || gn <= 0.) continue;
        // The true weights, so the probe gradient is the term's own: an edge the agreement weight
        // silences (a Euclidean crease) exerts no force and must neither count as
        // past-perpendicular nor pollute the conflict sign.
        const Vector2d ga_e = pot->gradient(x);
        const Vector2d gb_e = pot->gradient(m_vertex_attribute[q].m_posf);
        const double na = ga_e.norm(), nb = gb_e.norm();
        const bool ok = std::isfinite(na) && na > 0. && std::isfinite(nb) && nb > 0.;
        const double agree = ok ? std::max(0., ga_e.dot(gb_e) / (na * nb)) : 0.;
        if (agree > 0. && (sigma / d.norm()) * n_plus.dot(s * g / gn) <= 0.)
            past_perpendicular = true;
        edges.push_back({m_vertex_attribute[q].m_posf, sigma, agree});
    }
    if (!past_perpendicular || edges.empty()) return false;
    const Vector2d n_dir = front_vertex_move_direction(vid);
    if (!(n_dir.squaredNorm() > 0.)) return false;
    const double w_off = 1. - m_params.w_amips;
    AlignEnergy2D align(pot, std::move(edges), s, w_off);
    OffsetEnergy2D place(pot, w_off, true, true);
    Eigen::VectorXd xv(2), ga(2), gp(2);
    xv << x.x(), x.y();
    align.gradient(xv, ga);
    place.gradient(xv, gp);
    if (!((ga.dot(n_dir)) * (gp.dot(n_dir)) < 0.)) return false;
    return front_vertex_conv_ratio(vid) <= 1.;
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
    // Gauss-Newton Hessian (the default); the exact Hessian adds r grad^2 Phi and buys nothing.
    sum->add_energy(std::make_shared<OffsetEnergy2D>(pot, w_off, true, true));
    // One alignment residual per incident live front edge. sigma orients the edge's normal
    // away from its band face, read off the face centroid exactly as distance_criterion() does.
    std::vector<AlignEnergy2D::Edge> edges;
    const Vector2d x = m_vertex_attribute[vid].m_posf;
    for (const Tuple& e : get_one_ring_edges_for_vertex(vid)) {
        if (!edge_is_offset_surface_live(e)) continue;
        const size_t va = e.vid(*this), vb = e.switch_vertex(*this).vid(*this);
        const size_t q = (va == vid) ? vb : va;
        const std::optional<Tuple> opp = e.switch_face(*this);
        if (!opp) continue;
        const size_t fa = e.fid(*this), fb = opp->fid(*this);
        const size_t band_f = face_is_offset_band(fa) ? fa : fb;
        const auto vs = oriented_tri_vids(band_f);
        const Vector2d c = (m_vertex_attribute[vs[0]].m_posf + m_vertex_attribute[vs[1]].m_posf +
                            m_vertex_attribute[vs[2]].m_posf) /
                           3.;
        const Vector2d d = m_vertex_attribute[q].m_posf - x;
        if (!(d.norm() > 0.)) continue;
        const Vector2d n_plus(-d.y(), d.x()); // R90 (q - x)
        const Vector2d mid = 0.5 * (x + m_vertex_attribute[q].m_posf);
        const double sigma = n_plus.dot(mid - c) >= 0. ? 1. : -1.;
        // The agreement weight: see AlignEnergy2D::Edge::agree. The two endpoint gradients; the
        // field's outward sign squares away in the dot product. Either one degenerate means no
        // consistent target for this edge: weight 0, not a guess.
        const Vector2d ga = pot->gradient(x);
        const Vector2d gb = pot->gradient(m_vertex_attribute[q].m_posf);
        const double na = ga.norm(), nb = gb.norm();
        const bool ok = std::isfinite(na) && na > 0. && std::isfinite(nb) && nb > 0.;
        const double agree = ok ? std::max(0., ga.dot(gb) / (na * nb)) : 0.;
        edges.push_back({m_vertex_attribute[q].m_posf, sigma, agree});
    }
    // Kept: without it the seam is rougher, since under front_normal_projection nothing else acts
    // on the edge normals. The few percent of passes it costs is not worth that.
    if (!edges.empty() && m_offset_params.front_alignment_energy) {
        const double sign = m_offset_params.offset_field == "euclidean" ? 1. : -1.;
        sum->add_energy(std::make_shared<AlignEnergy2D>(pot, std::move(edges), sign, w_off));
    }
    return sum;
}

} // namespace wmtk::components::topological_offset
