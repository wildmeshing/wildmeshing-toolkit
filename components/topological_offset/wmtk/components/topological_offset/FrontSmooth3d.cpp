#include "TopoOffsetTetMesh.h"

#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * Phase B's front placement: an offset-surface vertex is moved by a one-dimensional Newton solve
 * along the offset field's gradient, widened to the shared 3-D smoother where the alignment term
 * traps that line. Everything else is smoothed by Optimize3d.cpp. The 2D twin is FrontSmooth2d.cpp.
 */

namespace {
/// The front vertex's objective restricted to the line x(s) = x0 + s n: the same energy, one
/// unknown. Every hook the solver's line search asks -- step validity (the AMIPS inversion
/// guard), the step cap, begin/end, solution_changed -- is forwarded on the mapped points.
class LineProblem3D : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;
    LineProblem3D(
        std::shared_ptr<polysolve::nonlinear::Problem> p,
        const Vector3d& x0,
        const Vector3d& n)
        : m_p(std::move(p))
        , m_x0(x0)
        , m_n(n)
    {}
    Eigen::VectorXd at(const TVector& s) const { return Eigen::VectorXd(m_x0 + s(0) * m_n); }
    double value(const TVector& s) override { return m_p->value(at(s)); }
    void gradient(const TVector& s, TVector& g) override
    {
        Eigen::VectorXd g3(3);
        m_p->gradient(at(s), g3);
        g.resize(1);
        g(0) = m_n.dot(Vector3d(g3));
    }
    void hessian(const TVector&, THessian&) override
    {
        log_and_throw_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& s, Eigen::MatrixXd& h) override
    {
        Eigen::MatrixXd h3(3, 3);
        m_p->hessian(at(s), h3);
        h.resize(1, 1);
        h(0, 0) = m_n.dot(h3 * m_n);
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
    Vector3d m_x0, m_n;
};
} // namespace

bool TopoOffsetTetMesh::smooth_front_vertex_phase_b(const Tuple& t)
{
    // See the header: the shared smoother with the offset's options. The offset terms arrive
    // through smoothing_extra_energy(), AMIPS is weighted as in TetOptimizerMesh::smooth_after(),
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
        const bool ok = optimization::smooth_vertex_3d(*this, t, opts, solver, &m_smooth_rejects);
        if (ok) m_released_tube_dirty.store(true, std::memory_order_release);
        return ok;
    }

    // Normal-only placement is a one-dimensional solve: the same objective the 3-D path minimises
    // (phase_b_front_objective), restricted to the line along n = grad Phi / |grad Phi|, with the
    // same solver, the same line search and the same accept test as the shared smoother's
    // no-envelope path (exact inversion of the ring; no envelope, no veto). Nothing is added to
    // the energy, as in 2D.
    const size_t vid = t.vid(*this);
    const Vector3d n = front_vertex_move_direction(vid);
    if (!(n.squaredNorm() > 0.)) { // no direction here: the 3-D solve is the fallback
        const bool ok = optimization::smooth_vertex_3d(*this, t, opts, solver, &m_smooth_rejects);
        if (ok) m_released_tube_dirty.store(true, std::memory_order_release);
        return ok;
    }
    // Widen the motion where the alignment term traps the 1-D solve: at a jog of the surface the
    // alignment residual's fixing motion is tangential, which the radial line projects away, so
    // where it opposes the placement pull it cancels it and the jog freezes. The trap is the
    // fight, not the perpendicular face -- both conditions in the predicate, as in 2D.
    if (m_offset_params.front_alignment_energy && front_vertex_alignment_traps_1d_solve(vid)) {
        const bool ok = optimization::smooth_vertex_3d(*this, t, opts, solver, &m_smooth_rejects);
        if (ok) m_released_tube_dirty.store(true, std::memory_order_release);
        return ok;
    }
    const std::shared_ptr<SampleEnvelope> hold = smoothing_containment_envelope(vid);
    const std::vector<Tuple> locs = get_one_ring_tets_for_vertex(t);
    for (const Tuple& loc : locs) {
        if (is_inverted_f(loc)) {
            ++m_smooth_rejects.already_inverted;
            return false;
        }
    }
    const Vector3d x0 = m_vertex_attribute[vid].m_posf;
    auto line = std::make_shared<LineProblem3D>(phase_b_front_objective(vid, x0), x0, n);
    Eigen::VectorXd s = Eigen::VectorXd::Zero(1);
    try {
        solver->minimize(*line, s);
    } catch (const std::exception&) {
    }
    set_vertex_position(vid, Vector3d(x0 + s(0) * n));
    if (hold) { // the boundary's tube, on the region faces the move reshaped -- as the shared
                // smoother
        const simplex::SimplexCollection surf = get_surface_faces_for_vertex(vid);
        for (const simplex::Face& f : surf.faces()) {
            const auto& fv = f.vertices();
            const auto found = try_tuple_from_face({{fv[0], fv[1], fv[2]}});
            if (!found) continue;
            const size_t fid = std::get<1>(*found);
            if (!m_face_attribute[fid].m_is_surface_fs || face_is_offset(fid)) continue;
            const std::array<Vector3d, 3> tri = {
                {m_vertex_attribute[fv[0]].m_posf,
                 m_vertex_attribute[fv[1]].m_posf,
                 m_vertex_attribute[fv[2]].m_posf}};
            if (hold->is_outside(tri)) {
                set_vertex_position(vid, x0);
                ++m_smooth_rejects.envelope;
                return false;
            }
        }
    }
    for (const Tuple& loc : locs) {
        if (is_inverted(loc)) {
            set_vertex_position(vid, x0);
            ++m_smooth_rejects.inverted;
            return false;
        }
    }
    for (const Tuple& loc : locs) set_cell_quality(loc.tid(*this), get_quality(loc));
    ++m_smooth_rejects.accepted;
    m_released_tube_dirty.store(true, std::memory_order_release); // the boundary may have moved
    return true;
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
    const Vector3d g = potential_for(vid).gradient(m_vertex_attribute[vid].m_posf);
    const double gn = g.norm();
    return (std::isfinite(gn) && gn > 0.) ? Vector3d(g / gn) : Vector3d::Zero();
}

namespace {
/// The cells of vid's one-ring as AMIPSEnergy3D wants them: the moving vertex first, winding
/// preserved. Returns the reordered vids per tet as well, so a rest shape can follow the same
/// permutation.
struct RingCell
{
    size_t tid;
    std::array<size_t, 4> vs; ///< vid first
    std::array<int, 4> from; ///< vs[k] == oriented_tet_vids(tid)[from[k]]
};
} // namespace

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTetMesh::phase_b_front_objective(
    const size_t vid,
    const Vector3d& x) const
{
    // The one-ring's AMIPS with the vertex first in every cell (what AMIPS_jacobian
    // differentiates against), weighted as the shared smoother weights it, plus the offset terms
    // on the vertex's own region's field.
    std::vector<std::array<double, 12>> cells;
    std::vector<RestAMIPSEnergy3D::Cell> plastic_cells; // deform_others: increments only
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        const std::array<size_t, 4> orig = oriented_tet_vids(tid);
        const std::array<size_t, 4> vs = wmtk::orient_preserve_tet_reorder(orig, vid);
        std::array<int, 4> from{};
        for (int k = 0; k < 4; ++k) {
            for (int j = 0; j < 4; ++j) {
                if (orig[j] == vs[k]) from[k] = j;
            }
        }
        // A plastic ring cell brakes the front only by its increment since the group started
        // (rest-shape AMIPS on the group-start rest); judged regular it becomes a permanent
        // brake that parks the front at an elastic equilibrium. Band cells stay regular, except
        // a band cell that is a released object's material. As in 2D.
        const TetAttributes& ta = m_tet_attribute[tid];
        if ((cell_is_plastic(tid) || cell_is_released_band(tid)) && ta.rest_valid) {
            Eigen::Matrix3d R;
            for (int k = 1; k < 4; ++k) {
                R.col(k - 1) = ta.rest_pos[size_t(from[k])] - ta.rest_pos[size_t(from[0])];
            }
            if (R.determinant() > 0.) {
                RestAMIPSEnergy3D::Cell c;
                c.q1 = m_vertex_attribute[vs[1]].m_posf;
                c.q2 = m_vertex_attribute[vs[2]].m_posf;
                c.q3 = m_vertex_attribute[vs[3]].m_posf;
                c.rest_inv = R.inverse();
                plastic_cells.push_back(c);
                continue;
            }
        }
        std::array<double, 12> T;
        for (int k = 0; k < 4; ++k) {
            const Vector3d p = (k == 0) ? x : m_vertex_attribute[vs[k]].m_posf;
            for (int j = 0; j < 3; ++j) T[k * 3 + j] = p[j];
        }
        cells.push_back(T);
    }
    const double amips_w = m_params.w_amips > 0 ? m_s_amips * m_params.w_amips : 1.0;
    auto sum = std::make_shared<optimization::EnergySum>();
    if (m_params.w_amips > 0 && !cells.empty())
        sum->add_energy(std::make_shared<optimization::AMIPSEnergy3D>(cells, amips_w));
    if (!plastic_cells.empty())
        sum->add_energy(std::make_shared<RestAMIPSEnergy3D>(std::move(plastic_cells), amips_w));
    sum->add_energy(phase_b_front_energy(vid, potential_ptr_for(vid)));
    return sum;
}

namespace {
/// One live offset face at x, as the alignment term wants it: the two other corners, the
/// orientation sign that points the normal away from the band, and the gradient agreement.
template <typename Mesh>
bool align_face_at(
    const Mesh& m,
    const typename Mesh::Tuple& f,
    const size_t vid,
    const OffsetPotential3D& pot,
    AlignEnergy3D::Face& out,
    bool& past_perpendicular_out,
    const double s)
{
    const auto vs = m.get_face_vids(f);
    std::array<size_t, 2> q{{0, 0}};
    int k = 0;
    for (const size_t v : vs) {
        if (v == vid) continue;
        if (k < 2) q[size_t(k)] = v;
        ++k;
    }
    if (k != 2) return false;
    const Vector3d x = m.m_vertex_attribute[vid].m_posf;
    const Vector3d p1 = m.m_vertex_attribute[q[0]].m_posf;
    const Vector3d p2 = m.m_vertex_attribute[q[1]].m_posf;
    const Vector3d N = (p1 - x).cross(p2 - x);
    if (!(N.norm() > 0.)) return false;
    // sigma orients the normal away from the band tet, read off its centroid exactly as the 2D
    // term reads the band face's.
    const size_t ta = f.tid(m);
    const std::optional<typename Mesh::Tuple> opp = f.switch_tetrahedron(m);
    const size_t band_t = m.cell_is_offset_band(ta) ? ta : (opp ? opp->tid(m) : ta);
    Vector3d ct = Vector3d::Zero();
    for (const size_t v : m.oriented_tet_vids(band_t)) ct += m.m_vertex_attribute[v].m_posf / 4.;
    const Vector3d cf = (x + p1 + p2) / 3.;
    const double sigma = N.dot(cf - ct) >= 0. ? 1. : -1.;
    // The agreement weight: min over the corner pairs of the endpoint gradients' agreement. Any
    // degenerate gradient means no consistent target for this face: weight 0, not a guess.
    const Vector3d g0 = pot.gradient(x), g1 = pot.gradient(p1), g2 = pot.gradient(p2);
    const double n0 = g0.norm(), n1 = g1.norm(), n2 = g2.norm();
    const bool ok = std::isfinite(n0) && n0 > 0. && std::isfinite(n1) && n1 > 0. &&
                    std::isfinite(n2) && n2 > 0.;
    double agree = 0.;
    if (ok) {
        const Vector3d u0 = g0 / n0, u1 = g1 / n1, u2 = g2 / n2;
        agree = std::max(0., std::min({u0.dot(u1), u0.dot(u2), u1.dot(u2)}));
    }
    out.q1 = p1;
    out.q2 = p2;
    out.sigma = sigma;
    out.agree = agree;
    // Past perpendicular: the face's outward normal at or past 90 degrees from the field's
    // outward direction at its centroid.
    past_perpendicular_out = false;
    const Vector3d gc = pot.gradient(cf);
    const double gcn = gc.norm();
    if (agree > 0. && std::isfinite(gcn) && gcn > 0.) {
        past_perpendicular_out = (sigma * N / N.norm()).dot(s * gc / gcn) <= 0.;
    }
    return true;
}
} // namespace

bool TopoOffsetTetMesh::front_vertex_alignment_traps_1d_solve(const size_t vid) const
{
    // Three conditions, all required -- see the use in smooth_front_vertex_phase_b() and the 2D
    // twin: (1) an incident live front face at or past perpendicular to the field, (2) the
    // alignment term's 1-D gradient opposing the placement term's along the move direction, and
    // (3) stationary off the level set.
    const std::shared_ptr<const OffsetPotential3D> pot = potential_ptr_for(vid);
    if (!pot) return false;
    const Vector3d x = m_vertex_attribute[vid].m_posf;
    const double rho = pot->residual_length(x);
    const double tube =
        std::max(m_offset_params.offset_envelope_rel * m_offset_params.target_distance, 1e-12);
    if (!std::isfinite(rho) || rho <= tube) return false;
    const double s = m_offset_params.offset_field == "euclidean" ? 1. : -1.;
    bool past_perpendicular = false;
    std::vector<AlignEnergy3D::Face> faces;
    for (const Tuple& f : offset_surface_faces_live_at(vid)) {
        AlignEnergy3D::Face af;
        bool pp = false;
        if (!align_face_at(*this, f, vid, *pot, af, pp, s)) continue;
        past_perpendicular = past_perpendicular || pp;
        faces.push_back(af);
    }
    if (!past_perpendicular || faces.empty()) return false;
    const Vector3d n_dir = front_vertex_move_direction(vid);
    if (!(n_dir.squaredNorm() > 0.)) return false;
    const double w_off = 1. - m_params.w_amips;
    AlignEnergy3D align(pot, std::move(faces), s, w_off);
    OffsetEnergy3D place(pot, w_off, true, true);
    Eigen::VectorXd xv(3), ga(3), gp(3);
    xv << x.x(), x.y(), x.z();
    align.gradient(xv, ga);
    place.gradient(xv, gp);
    if (!((ga.dot(n_dir)) * (gp.dot(n_dir)) < 0.)) return false;
    return front_vertex_conv_ratio(vid) <= 1.;
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTetMesh::phase_b_front_energy(
    const size_t vid,
    const std::shared_ptr<const OffsetPotential3D>& pot) const
{
    const double w_off = 1. - m_params.w_amips;
    auto sum = std::make_shared<optimization::EnergySum>();
    // Gauss-Newton Hessian (the default); the exact Hessian adds r grad^2 Phi and buys nothing.
    sum->add_energy(std::make_shared<OffsetEnergy3D>(pot, w_off, true, true));
    // One alignment residual per incident live front face.
    const double sign = m_offset_params.offset_field == "euclidean" ? 1. : -1.;
    std::vector<AlignEnergy3D::Face> faces;
    for (const Tuple& f : offset_surface_faces_live_at(vid)) {
        AlignEnergy3D::Face af;
        bool pp = false;
        if (!align_face_at(*this, f, vid, *pot, af, pp, sign)) continue;
        faces.push_back(af);
    }
    // Kept: without it the seam is rougher, since under front_normal_projection nothing else acts
    // on the face normals.
    if (!faces.empty() && m_offset_params.front_alignment_energy) {
        sum->add_energy(std::make_shared<AlignEnergy3D>(pot, std::move(faces), sign, w_off));
    }
    return sum;
}

} // namespace wmtk::components::topological_offset
