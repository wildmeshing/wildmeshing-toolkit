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
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * Front placement: an offset-surface vertex is placed by the shared 3-D smoother with the offset's
 * energy (see smooth_front_vertex()). Everything else is smoothed by Optimize3d.cpp. The 2D twin
 * is FrontSmooth2d.cpp.
 */

namespace {
/// One live offset face at x, as the offset term wants it: the two other corners, and the
/// barycentric weights of the face's stencil_order stencil. Only the weights are frozen -- the
/// sample points themselves slide with x and the field is read live -- see StencilEnergy3D.
template <typename Mesh>
bool stencil_face_at(
    const Mesh& m,
    const typename Mesh::Tuple& f,
    const size_t vid,
    const OffsetPotential3D& pot,
    StencilEnergy3D::Face& out)
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
    out.q1 = m.m_vertex_attribute[q[0]].m_posf;
    out.q2 = m.m_vertex_attribute[q[1]].m_posf;

    // Only the barycentric weights are frozen here; the residual is evaluated live at every
    // solve iterate, since the sample points slide with x. Nothing about the field is cached --
    // the sag term this replaced had to freeze 1/|grad Phi| per sample to keep its measure a
    // length, and the relative error needs no such factor.
    out.samples.clear();
    m.for_each_face_sample(
        x,
        out.q1,
        out.q2,
        [&](const Vector3d&, const double wa, const double wb, const double wc, const double w) {
            StencilEnergy3D::Sample sm;
            sm.a = wa;
            sm.b = wb;
            sm.c = wc;
            sm.w = w;
            out.samples.push_back(sm);
        });
    return !out.samples.empty();
}

} // namespace

bool TopoOffsetTetMesh::smooth_front_vertex(const Tuple& t)
{
    // See the header: the shared smoother with the offset's options. The whole objective arrives
    // through smoothing_extra_energy() -- the offset terms at offset_term_weight() and the shape
    // term shape_energy() -- and w_amips 0 keeps the smoother from adding an AMIPS term of its
    // own; a front vertex an input envelope pins carries no offset term. The smoother holds a front
    // vertex to no envelope (see smoothing_containment_envelope()), so neither the projected path
    // nor the containment check applies -- solve, exact inversion test, then the veto on
    // tet_energy() below.
    const size_t vid = t.vid(*this);
    optimization::SmoothVertexOptions opts;
    opts.w_amips = 0.;
    opts.w_envelope = m_params.w_envelope;
    opts.s_amips = m_s_amips;
    opts.s_envelope = m_s_envelope;
    opts.two_stage = false;
    // The engine's veto compares AMIPS^3 alone, and a front vertex must be free to worsen its
    // ring's shape on the way to the level set: off, as it always was here. solve_3d() vetoes on
    // the energy instead.
    opts.quality_veto = false;
    auto& solver =
        m_solver.local(); // the thread's shared solver, criteria set by smoothing_solver()
    smoothing_solver();
    // THE FRONT SMOOTHER'S VETO (see tet_energy()), on every 3-D solve: the max of tet_energy()
    // over the vertex's one-ring may not rise. A tie passes, as in the engine's AMIPS veto, whose
    // place it takes under the front's own key, offset_front_smooth_veto (the engine's field, key
    // offset_smooth_veto, stays the interior vertices'). Read before the solve and after it, on the
    // mesh: a smoothing move changes no label and moves one vertex, so the one-ring holds every
    // cell whose energy it can change -- the shape of each, and every front face with the vertex as
    // a corner, which only a cell of the ring can carry. Refused, TetMesh::smooth_vertex() rolls
    // back the position and the ring's stored qualities. smooth_vertex_3d() has counted the move
    // accepted by then; it is recounted as a quality refusal, so the counters still partition the
    // attempts.
    // EXPERIMENTAL_unreachable_exit: the change this solve makes to the vertex's own error, 0 when
    // the move is refused and rolled back (see VertexExtra::m_front_change).
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
        // EXPERIMENTAL_integral_energy: no veto -- the solve descends the vertex's part of E'.
        // EXPERIMENTAL_band_volume_energy: the same.
        // EXPERIMENTAL_ops_global_energy: the veto is E itself, as in smooth_nonfront_vertex().
        const bool e_veto = m_offset_params.ops_global_energy;
        const bool veto = !e_veto && m_offset_params.offset_front_smooth_veto &&
                          !m_offset_params.integral_energy && !m_offset_params.band_volume_energy;
        const double before = e_veto ? global_sum(ring) : veto ? max_tet_energy(ring) : 0.;
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
        const bool ok =
            optimization::smooth_vertex_3d(*this, t, opts, solver, &m_smooth_rejects, &one);
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
        if (veto) ++m_front_veto_asked;
        if ((veto && !(max_tet_energy(ring) <= before)) ||
            (e_veto && !(global_sum(ring) <= before))) { // a NaN refuses
            ++m_front_veto_fired;
            --m_smooth_rejects.accepted;
            ++m_smooth_rejects.quality;
            record_change(false);
            return false;
        }
        record_change(true);
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
    // The front's path with the repulsion term in place of the face terms: the whole objective
    // arrives through smoothing_extra_energy() -- w AMIPS^3 over the ring and repulsion_energy(vid)
    // -- and w_amips 0 keeps the smoother from adding an AMIPS term of its own; no engine veto.
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
    auto& solver = m_solver.local();
    smoothing_solver();
    // The front veto's rule (see tet_energy()): the max of the per-tet energy over the ring --
    // which includes the repulsion terms of the cells that carry them, O(v) among them -- may
    // not rise (a tie passes). Read before the solve and after it; refused,
    // TetMesh::smooth_vertex() rolls the move back.
    const std::vector<size_t>& ring = get_one_ring_tids_for_vertex(vid);
    const bool veto = m_offset_params.offset_front_smooth_veto;
    const double before = veto ? max_tet_energy(ring) : 0.;
    optimization::NewtonCounters one;
    const bool ok = optimization::smooth_vertex_3d(*this, t, opts, solver, &m_smooth_rejects, &one);
    if (one.solves() > 0) m_newton_repulsion.record(*solver, one.threw.load() > 0);
    if (!ok) return false;
    if (veto) ++m_repulsion_veto_asked;
    if (veto && !(max_tet_energy(ring) <= before)) { // a NaN refuses
        ++m_repulsion_veto_fired;
        --m_smooth_rejects.accepted;
        ++m_smooth_rejects.quality;
        return false;
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

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTetMesh::front_objective(
    const size_t vid) const
{
    // smoothing_extra_energy() at a front vertex that carries the offset term, with the offset
    // term included unconditionally: the shape term (shape_energy(), w sum V_p pAMIPS^3 under
    // use_rest_pose, w sum AMIPS^3 without it) and the offset terms on the vertex's own region's
    // field.
    // An empty sum is a ZeroEnergy3D: EnergySum's gradient and Hessian read its first term.
    auto sum = std::make_shared<optimization::EnergySum>();
    bool any = false;
    if (const auto shape = shape_energy(vid)) {
        sum->add_energy(shape);
        any = true;
    }
    if (const auto front = front_energy(vid, potential_ptr_for(vid))) {
        sum->add_energy(front);
        any = true;
    }
    if (!any) sum->add_energy(std::make_shared<ZeroEnergy3D>());
    return sum;
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTetMesh::front_energy(
    const size_t vid,
    const std::shared_ptr<const OffsetPotential3D>& pot) const
{
    // 1 / front_conv_frac()^2: the squared relative error (Phi - c)/c becomes the squared error
    // in units of the tolerance, so the term below is sum_f O(f) -- for the euclidean field the
    // very terms the per-tet energy tet_energy() carries on these faces' band cells, and the
    // ring measure's n_v r_v^2. It replaced 1 - w_amips, a weight with no unit, on 2026-09-28.
    const double w_off = offset_term_weight();
    auto sum = std::make_shared<optimization::EnergySum>();
    if (m_offset_params.band_volume_energy) {
        // EXPERIMENTAL_band_volume_energy: the vertex's part of int_B e dV in place of the face
        // term.
        if (const auto bv = band_volume_energy_at(vid)) sum->add_energy(bv);
        return sum;
    }
    // THE offset term, and the only one: the mean squared relative error over each incident
    // face's stencil, summed over the ring. It subsumes the placement term that used to sit here
    // -- the stencil contains the face's corners, so the moving vertex's own residual is in it
    // V/N_s times for a vertex of valence V -- and the sag term that used to sit below, whose
    // interior samples are the stencil's non-corner points. See StencilEnergy3D. Every face
    // weighs 1: the area weights front_measure "vertex_ring" used to put here went with the
    // per-tet energy, which has no area in it, and the ring measure dropped them with it -- except
    // under EXPERIMENTAL_area_weighted_ring, where the term is n R(v)^2 with R(v)^2 the
    // area-weighted ring measure and the areas variables (StencilEnergy3D, AREA WEIGHTING).
    {
        std::vector<StencilEnergy3D::Face> stencil_faces;
        // EXPERIMENTAL_visible_distance: each face's corners in the stencil's order (vid first)
        // and its band cell, for the sample reader below.
        std::vector<std::array<size_t, 3>> face_vids;
        std::vector<int64_t> face_band;
        for (const Tuple& f : offset_surface_faces_live_at(vid)) {
            StencilEnergy3D::Face sf;
            if (!stencil_face_at(*this, f, vid, *pot, sf)) continue;
            stencil_faces.push_back(std::move(sf));
            if (m_visible_active) {
                std::array<size_t, 3> fv{{vid, vid, vid}};
                int k = 1;
                for (const size_t v : get_face_vids(f)) {
                    if (v != vid && k < 3) fv[size_t(k++)] = v;
                }
                face_vids.push_back(fv);
                face_band.push_back(front_face_band_cell(fv[0], fv[1], fv[2]));
            }
        }
        if (!stencil_faces.empty()) {
            auto energy = std::make_shared<StencilEnergy3D>(pot, std::move(stencil_faces), w_off);
            energy->set_area_weighted(
                m_offset_params.area_weighted_ring && !m_offset_params.integral_energy);
            energy->set_area_integral(m_offset_params.integral_energy);
            if (m_visible_active) {
                // The field at a sample is d_vis with the moving vertex at the iterate x. The walk
                // reads the cells around vid at x too, so a trial x whose cells cannot be walked
                // (an inverted ring the line search would refuse anyway) scores +inf rather than
                // stopping the run; at the vertex's own position a Stop does stop it.
                const Vector3d x0 = m_vertex_attribute[vid].m_posf;
                energy->set_sample_reader(
                    [this, vid, x0, face_vids, face_band](
                        const size_t fi,
                        const StencilEnergy3D::Sample& sm,
                        const Eigen::Vector3d& x,
                        const Eigen::Vector3d& p,
                        double& v,
                        Eigen::Vector3d& g,
                        Eigen::Matrix3d& H) -> int {
                        MeshCells cells(*this);
                        cells.moved_vid = int64_t(vid);
                        cells.moved_pos = x;
                        const VisibleStart st = visible_start(
                            face_vids[fi],
                            {{sm.a, sm.b, sm.c}},
                            face_band[fi],
                            cells);
                        const bool at_own_position = x == x0;
                        const VisibleField::Feature f =
                            visible_feature(p, st, cells, /*allow_stop=*/!at_own_position);
                        if (f.status == VisibleField::Feature::Status::Stop) return -1;
                        if (f.status != VisibleField::Feature::Status::Found) return 0;
                        v = f.d / m_visible_field->delta();
                        g = m_visible_field->gradient(p, f);
                        H = m_visible_field->hessian(p, f);
                        return 1;
                    });
            }
            sum->add_energy(energy);
        }
    }
    return sum;
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTetMesh::band_volume_energy_at(
    const size_t vid) const
{
    // band_cell_term() of each band cell of the ring, x moving (BandVolumeEnergy3D).
    if (m_n_regions > 1) {
        log_and_throw_error(
            "EXPERIMENTAL_band_volume_energy reads one field, and this input has {} regions",
            m_n_regions);
    }
    std::vector<BandVolumeEnergy3D::Cell> cells;
    std::vector<std::vector<int64_t>> cand; // corner_bound: per cell, its candidate triangles
    const bool corner_bound = m_offset_params.band_volume_rule == "corner_bound";
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        if (!cell_is_offset_band(tid)) continue;
        BandVolumeEnergy3D::Cell c;
        int k = 0;
        std::vector<int64_t> cc;
        for (const size_t u : oriented_tet_vids(tid)) {
            if (corner_bound) cc.push_back(m_band_tris->nearest(m_vertex_attribute[u].m_posf));
            if (u == vid) continue;
            (k == 0 ? c.q1 : k == 1 ? c.q2 : c.q3) = m_vertex_attribute[u].m_posf;
            ++k;
        }
        if (k != 3) continue;
        cells.push_back(c);
        if (corner_bound) {
            if (m_tet_attribute[tid].band_tri >= 0) cc.push_back(m_tet_attribute[tid].band_tri);
            std::sort(cc.begin(), cc.end());
            cc.erase(std::unique(cc.begin(), cc.end()), cc.end());
            cand.push_back(std::move(cc));
        }
    }
    if (cells.empty()) return nullptr;
    auto e = std::make_shared<BandVolumeEnergy3D>(
        m_offset_potential,
        std::move(cells),
        m_vertex_attribute[vid].m_posf,
        band_volume_factor());
    e->set_centroid_only(m_offset_params.band_volume_rule == "centroid");
    if (corner_bound)
        e->set_corner_bound(m_band_tris, std::move(cand), m_offset_params.target_distance);
    return e;
}

} // namespace wmtk::components::topological_offset
