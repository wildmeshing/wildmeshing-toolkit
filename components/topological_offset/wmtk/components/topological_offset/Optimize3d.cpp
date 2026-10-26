#include "TopoOffsetTetMesh.h"

#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/RunPass.hpp>
#include <wmtk/utils/SizingField.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace wmtk {
/// Defined in src/wmtk/TetOptimizerMeshSwaps.cpp and declared in no engine header.
void face_attribute_tracker(
    const TetMesh& m,
    const std::vector<size_t>& incident_tets,
    const TetOptimizerMesh::FaceAttCol& m_face_attribute,
    std::map<std::array<size_t, 3>, TetOptimizerMesh::FaceAttributes>& changed_faces);
} // namespace wmtk

namespace wmtk::components::topological_offset {

/**
 * The 3D optimization phase, the twin of Optimize2d.cpp. The operations themselves -- split,
 * collapse, the swaps, and the driver that sequences them -- are wmtk::TetOptimizerMesh's. What is
 * here is only what the offset knows: where its two tracked surfaces are, how a vertex on the
 * offset surface is allowed to move, and the loop that places the front.
 */

namespace {

/// The three corners of face `fid` in `tid` as (vid, other, other) for a given vid.
inline std::array<size_t, 3> face_corners_from(
    const std::array<size_t, 4>& tet_vids,
    const int skip)
{
    std::array<size_t, 3> f{};
    int k = 0;
    for (int j = 0; j < 4; ++j) {
        if (j != skip) f[size_t(k++)] = tet_vids[size_t(j)];
    }
    return f;
}
} // namespace

bool TopoOffsetTetMesh::is_edge_on_region(const Tuple& loc)
{
    size_t v1_id = loc.vid(*this);
    auto loc1 = loc.switch_vertex(*this);
    size_t v2_id = loc1.vid(*this);
    if (!m_vertex_extra[v1_id].m_is_on_region || !m_vertex_extra[v2_id].m_is_on_region)
        return false;

    auto tets = get_incident_tets_for_edge(loc);
    std::vector<size_t> n_vids;
    for (auto& t : tets) {
        auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            if (vs[j].vid(*this) != v1_id && vs[j].vid(*this) != v2_id)
                n_vids.push_back(vs[j].vid(*this));
        }
    }
    wmtk::vector_unique(n_vids);

    for (size_t vid : n_vids) {
        auto [_, fid] = tuple_from_face({{v1_id, v2_id, vid}});
        if (face_is_region(fid)) return true;
    }

    return false;
}

bool TopoOffsetTetMesh::is_edge_on_offset(const Tuple& loc)
{
    size_t v1_id = loc.vid(*this);
    auto loc1 = loc.switch_vertex(*this);
    size_t v2_id = loc1.vid(*this);
    if (!m_vertex_extra[v1_id].m_is_on_offset || !m_vertex_extra[v2_id].m_is_on_offset)
        return false;

    auto tets = get_incident_tets_for_edge(loc);
    std::vector<size_t> n_vids;
    for (auto& t : tets) {
        auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            if (vs[j].vid(*this) != v1_id && vs[j].vid(*this) != v2_id)
                n_vids.push_back(vs[j].vid(*this));
        }
    }
    wmtk::vector_unique(n_vids);

    for (size_t vid : n_vids) {
        auto [_, fid] = tuple_from_face({{v1_id, v2_id, vid}});
        if (face_is_offset(fid)) return true;
    }

    return false;
}

bool TopoOffsetTetMesh::vertex_is_on_surface(const size_t vid) const
{
    // The domain wall must count here, or it drifts: the smoother's containment check walks
    // exactly the collection this decides.
    return vertex_is_on_region(vid) || m_vertex_extra.at(vid).m_is_on_offset;
}

bool TopoOffsetTetMesh::face_is_on_surface(const size_t fid) const
{
    return m_face_attribute.at(fid).m_is_surface_fs;
}

void TopoOffsetTetMesh::label_offset_boundary()
{
    // Runs once at the top of the optimization, never again -- as in 2D. It only upgrades the
    // offset surface to its own class: region and input faces keep class 0 and are
    // envelope-checked by the shared operations exactly as in tetwild and simwild.

    // Cell quality, which the shared operations read and keep up to date from here on.
    for (const Tuple& t : get_tets()) {
        m_tet_attribute[t.tid(*this)].m_quality = get_quality(t);
    }

    size_t n_off = 0, n_wall_band = 0;
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        if (m_face_extra[fid].label != 2) {
            continue; // face not on the offset, skip
        }
        const std::optional<Tuple> opp = f.switch_tetrahedron(*this);
        if (!opp) {
            // The band ran into the domain wall. With no second tet this cannot be classified
            // as offset surface -- and must not be, or the wall face would lose the per-tag
            // containment that keeps the box a box.
            ++n_wall_band;
            continue;
        }
        if (m_tet_attribute[f.tid(*this)].label == m_tet_attribute[opp->tid(*this)].label) {
            continue; // face not between different labels, skip
        }

        m_face_attribute[fid].m_is_surface_fs = true;
        m_face_attribute[fid].m_surface_class = OFFSET_SURFACE_CLASS;
        ++n_off;

        for (const size_t vid : get_face_vids(f)) {
            m_vertex_extra[vid].m_is_on_offset = true;
            // The base's union flag, and the one the shared operations actually read.
            m_vertex_attribute[vid].m_is_on_surface = true;
        }
    }

    size_t n_reg = 0, n_box = 0;
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        n_reg += face_is_region(fid);
        n_box += (m_face_attribute[fid].m_is_bbox_fs >= 0);
    }
    logger().info(
        "\ttracked faces: {} offset surface, {} region boundary (input complex included), {} "
        "bbox | {} band faces lying ON the wall",
        n_off,
        n_reg,
        n_box,
        n_wall_band);
}

double TopoOffsetTetMesh::face_offset_term(
    const OffsetPotential3D& pot,
    const Vector3d& pa,
    const Vector3d& pb,
    const Vector3d& pc) const
{
    // THE face measure, and since 2026-09-24 the only one: the MEAN over the face's stencil of
    // the squared relative error, in units of the tolerance. At every stencil point q of
    // for_each_face_sample():
    //
    //     r(q) = pot.relative_residual(q) / front_conv_frac()
    //          = (signed distance from q to the level set, along the field) / front_conv
    //
    // and the face's term is mean of r^2, so 1 exactly at the bar; its root is the "Nx the bar"
    // figure the logs print. One arithmetic for every reader: the exit and refinement
    // (energy_criterion()) and the debug frames. Not part of the energy (tet_energy()).
    //
    // WHY THE RELATIVE ERROR AND NOT A SAG. The stencil contains the face's CORNERS (order 0 is
    // the corners alone), where an interpolation error is identically zero but a distance to the
    // level set is not. Measuring the distance makes the corners informative, and that is what
    // lets this one number replace both the old per-vertex placement test and the old per-face
    // sag: a face is resolved when its root is within the bar over the whole stencil.
    //
    // WHY THE FIELD IS ASKED, not (Phi(q) - c)/c formed here. That relative FIELD error is the
    // relative distance error only for a field linear in the distance, as the euclidean one is.
    // For the smooth field it is the distance error times delta |dPhi/dd| / c = 3.44 at the
    // default offset_dhat_factor 2, so every face and vertex read 3.44x its real error: on the
    // cube at target_distance_rel 1e-2 / front_conv_rel 1e-4 that bought an extra halving -- 9
    // turns, 80054 front faces and 477 s against the euclidean field's 7 turns, 25006 faces and
    // 75 s on the same offset surface. relative_residual() is the distance to the level set
    // along the field over target_distance for both fields, and for the euclidean one it is
    // still (value - c)/c, so that path is unchanged bit for bit.
    const double m = face_offset_mean_sq(pot, pa, pb, pc);
    if (m < 0.) return -1.;
    if (!(m_offset_params.front_conv_frac() > 0.)) return std::numeric_limits<double>::infinity();
    return offset_term_weight() * m;
}

double TopoOffsetTetMesh::face_offset_mean_sq(
    const OffsetPotential3D& pot,
    const Vector3d& pa,
    const Vector3d& pb,
    const Vector3d& pc) const
{
    // The mean over the stencil of relative_residual(q)^2 (for the euclidean field
    // ((d - delta) / delta)^2): face_offset_term()'s mean before its 1 / front_conv_frac()^2
    // scale. -1 unmeasurable.
    const double level = pot.target_level();
    if (!(level > 0.)) return -1.;
    double sum = 0.;
    size_t n = 0;
    bool unmeasurable = false;
    for_each_face_sample(pa, pb, pc, [&](const Vector3d& q, double, double, double) {
        if (unmeasurable) return;
        const double r = pot.relative_residual(q);
        if (!std::isfinite(r)) {
            unmeasurable = true;
            return;
        }
        sum += r * r;
        ++n;
    });
    // n == 0 only when stencil_order < 0, which the spec's min refuses; an unmeasurable sample
    // reads the whole face unmeasurable.
    if (unmeasurable || n == 0) return -1.;
    return sum / double(n);
}

double TopoOffsetTetMesh::face_offset_term(const size_t a, const size_t b, const size_t c) const
{
    return face_offset_term(
        potential_for_edge(a, b),
        m_vertex_attribute[a].m_posf,
        m_vertex_attribute[b].m_posf,
        m_vertex_attribute[c].m_posf);
}

void TopoOffsetTetMesh::face_stencil_points(
    const size_t a,
    const size_t b,
    const size_t c,
    std::vector<StencilPoint>& out) const
{
    // See StencilPoint: the lattice over the face's corners in ascending order, each point named
    // by its nonzero weights' vertices (still ascending), so a point on a corner or an edge gets
    // the same name from every face around it.
    const int k = m_offset_params.stencil_order;
    if (k < 0) return;
    std::array<size_t, 3> s{{a, b, c}};
    std::sort(s.begin(), s.end());
    const auto push = [&](const std::array<int, 3>& wv, const int den) {
        StencilPoint p;
        p.den = den;
        for (int m = 0; m < 3; ++m) {
            if (wv[size_t(m)] == 0) continue;
            p.ids[size_t(p.n)] = s[size_t(m)];
            p.w[size_t(p.n)] = wv[size_t(m)];
            ++p.n;
        }
        out.push_back(p);
    };
    if (k == 0) {
        push({{1, 1, 1}}, 3);
        return;
    }
    for (int i = k; i >= 0; --i) {
        for (int j = k - i; j >= 0; --j) push({{i, j, k - i - j}}, k);
    }
}

Vector3d TopoOffsetTetMesh::stencil_point_position(const StencilPoint& s) const
{
    // From the name alone, in its vertex order: the same bits from every face. A vertex point
    // is its vertex exactly (w / den = 1).
    Vector3d q = Vector3d::Zero();
    for (int m = 0; m < s.n; ++m) {
        q += (double(s.w[size_t(m)]) / double(s.den)) * m_vertex_attribute[s.ids[size_t(m)]].m_posf;
    }
    return q;
}

double TopoOffsetTetMesh::stencil_point_error(const StencilPoint& s) const
{
    // See the declaration: the union field, never the per-region ones.
    if (!m_offset_potential) return -1.;
    const double r = m_offset_potential->relative_residual(stencil_point_position(s));
    if (!std::isfinite(r)) return -1.;
    return r * r;
}

TopoOffsetTetMesh::StencilMax TopoOffsetTetMesh::stencil_max(std::vector<StencilPoint> pts) const
{
    // See the declaration: each point once.
    std::sort(pts.begin(), pts.end());
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
    StencilMax out;
    out.n = pts.size();
    for (const StencilPoint& p : pts) {
        const double e = stencil_point_error(p);
        if (e < 0.) {
            ++out.n_unmeasurable;
            continue;
        }
        out.max = std::max(out.max, e);
    }
    return out;
}

std::vector<TopoOffsetTetMesh::StencilPoint> TopoOffsetTetMesh::stencil_points_changed_at(
    const size_t v1,
    const size_t v2) const
{
    // See the declaration: a point of a face at v1 (or v2) is on the region's boundary exactly
    // when its name involves neither.
    std::vector<StencilPoint> pts, out;
    for (const size_t v : {v1, v2}) {
        if (v == static_cast<size_t>(-1)) continue;
        for (const Tuple& ft : offset_surface_faces_live_at(v)) {
            const std::array<size_t, 3> f = get_face_vids(ft);
            face_stencil_points(f[0], f[1], f[2], pts);
        }
    }
    for (const StencilPoint& p : pts) {
        if (p.involves(v1) || (v2 != static_cast<size_t>(-1) && p.involves(v2))) {
            out.push_back(p);
        }
    }
    return out;
}

std::vector<TopoOffsetTetMesh::StencilPoint> TopoOffsetTetMesh::stencil_points_of_diagonal(
    const size_t p,
    const size_t q,
    const size_t r1,
    const size_t r2) const
{
    // See the declaration: the shared edge's points and the two faces' interior points.
    std::vector<StencilPoint> pts, out;
    face_stencil_points(p, q, r1, pts);
    face_stencil_points(p, q, r2, pts);
    const size_t lo = std::min(p, q), hi = std::max(p, q);
    for (const StencilPoint& s : pts) {
        if (s.n == 3 || (s.n == 2 && s.ids[0] == lo && s.ids[1] == hi)) out.push_back(s);
    }
    return out;
}

double TopoOffsetTetMesh::offset_error(size_t* n_unmeasurable) const
{
    // See the declaration: the unique points of every offset face, each evaluated on the mesh's
    // threads, summed serially in point order.
    std::vector<StencilPoint> pts;
    for (const auto& f : offset_surface_faces()) face_stencil_points(f[0], f[1], f[2], pts);
    std::sort(pts.begin(), pts.end());
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
    std::vector<double> e(pts.size(), 0.);
    const TopoOffsetTetMesh& self = *this;
    diagnostic_parallel_for(pts.size(), kDiagnosticChunk, [&](const size_t b, const size_t en) {
        for (size_t k = b; k < en; ++k) e[k] = self.stencil_point_error(pts[k]);
    });
    double s = 0.;
    size_t bad = 0;
    for (const double x : e) {
        if (x < 0.) {
            ++bad;
        } else {
            s += x;
        }
    }
    if (n_unmeasurable) *n_unmeasurable = bad;
    return s;
}

double TopoOffsetTetMesh::tet_energy(const size_t tid) const
{
    // See the declaration. get_quality() is the base's: AMIPS^3 or MAX_ENERGY.
    const double a3 = get_quality(tuple_from_tet(tid));
    const std::array<size_t, 4> vids = oriented_tet_vids(tid);
    Eigen::Matrix3d R;
    if (a3 < MAX_ENERGY && plastic_rest(tid, R)) {
        // use_rest_pose: a plastic cell's AMIPS term is vol A^3 against its stamped rest, the
        // number the smoother's VolAMIPSEnergy3D minimises. Never a band cell (cell_is_plastic()).
        std::array<Vector3d, 4> p;
        for (int k = 0; k < 4; ++k) p[size_t(k)] = m_vertex_attribute[vids[size_t(k)]].m_posf;
        const double v = VolAMIPSEnergy3D::value_of(p, R);
        if (!std::isfinite(v)) return MAX_ENERGY;
        double e = amips_term_weight() * v;
        if (m_repulsion_potential) e += repulsion_cell_term(vids);
        return e;
    }
    return cell_energy(vids, a3);
}

bool TopoOffsetTetMesh::plastic_rest(const size_t tid, Eigen::Matrix3d& R) const
{
    // See the declaration. The rest's corners are in the oriented order, as the cell's are.
    if (!cell_is_plastic(tid)) return false;
    const TetAttributes& ta = m_tet_attribute[tid];
    if (!ta.rest_valid) return false;
    for (int k = 1; k < 4; ++k) R.col(k - 1) = ta.rest_pos[size_t(k)] - ta.rest_pos[0];
    return R.determinant() > 0.;
}

void TopoOffsetTetMesh::stamp_rest_cell(const size_t tid)
{
    // See the declaration.
    if (!cell_is_plastic(tid)) return;
    TetAttributes& x = m_tet_attribute[tid];
    const auto vs = oriented_tet_vids(tid);
    for (int i = 0; i < 4; ++i) x.rest_pos[size_t(i)] = m_vertex_attribute[vs[size_t(i)]].m_posf;
    x.rest_valid = true;
}

void TopoOffsetTetMesh::stamp_plastic_rests()
{
    // See the declaration.
    if (!m_plastic_active) return;
    for (const Tuple& t : get_tets()) stamp_rest_cell(t.tid(*this));
}

double TopoOffsetTetMesh::plastic_quality(const size_t tid) const
{
    // See the declaration: AMIPS^3 against the stamped rest for a plastic cell with a valid rest
    // (VolAMIPSEnergy3D::value_of() is vol * A^3; dividing by the cell's volume recovers A^3),
    // the engine's AMIPS^3 for every other cell.
    Eigen::Matrix3d R;
    if (!plastic_rest(tid, R)) return get_quality(tuple_from_tet(tid));
    const std::array<size_t, 4> vs = oriented_tet_vids(tid);
    std::array<Vector3d, 4> p;
    for (int k = 0; k < 4; ++k) p[size_t(k)] = m_vertex_attribute[vs[size_t(k)]].m_posf;
    const double vol6 = (p[1] - p[0]).dot((p[2] - p[0]).cross(p[3] - p[0]));
    if (!(vol6 > 0.)) return MAX_ENERGY;
    const double q = VolAMIPSEnergy3D::value_of(p, R) * 6. / vol6;
    return std::isfinite(q) ? q : MAX_ENERGY;
}

bool TopoOffsetTetMesh::all_plastic(const std::vector<size_t>& tids) const
{
    // See the declaration.
    if (tids.empty()) return false;
    for (const size_t tid : tids) {
        if (!cell_is_plastic(tid)) return false;
    }
    return true;
}

bool TopoOffsetTetMesh::plastic_tie(
    const std::vector<size_t>& before,
    const std::vector<int>& after_labels) const
{
    // See the declaration.
    if (!m_plastic_active || after_labels.empty()) return false;
    for (const int l : after_labels) {
        if (l == 1 || l == 2) return false; // an elastic cell: the band or the input complex
    }
    return all_plastic(before);
}

double TopoOffsetTetMesh::global_energy() const
{
    // E = (w / s^3) sum vol AMIPS^3: every live tet's tet_energy() once.
    double s = 0.;
    for (const Tuple& t : get_tets()) s += tet_energy(t.tid(*this));
    return s;
}

double TopoOffsetTetMesh::global_energy_parallel() const
{
    // See the declaration: global_energy()'s sum, each cell evaluated on the mesh's threads,
    // summed serially in cell order.
    const std::vector<Tuple> tets = get_tets();
    std::vector<double> e_k(tets.size(), 0.);
    diagnostic_parallel_for(tets.size(), kDiagnosticChunk, [&](const size_t b, const size_t e) {
        for (size_t k = b; k < e; ++k) e_k[k] = tet_energy(tets[k].tid(*this));
    });
    double s = 0.;
    for (size_t k = 0; k < tets.size(); ++k) s += e_k[k];
    return s;
}

void TopoOffsetTetMesh::log_energy_step(const char* step) const
{
    logger().info(
        "\t[energy step] turn {} {}: E {:.12g}",
        m_energy_step_turn,
        step,
        global_energy_parallel());
}

double TopoOffsetTetMesh::tet_energy_sum(const std::vector<size_t>& tids) const
{
    double s = 0.;
    for (const size_t tid : tids) s += tet_energy(tid);
    return s;
}


bool TopoOffsetTetMesh::swap_capture_surface_sides(
    const std::vector<size_t>& tids,
    const size_t a,
    const size_t b,
    const size_t c,
    const size_t d)
{
    // See the declaration. The ring of a surface flip is two-sided BY CONSTRUCTION -- that is
    // what makes its faces offset surface -- so demanding one tag and one label for the whole
    // ring, as the interior rule does, can never be satisfied here. Map each ring vertex that is
    // not one of a, b, c, d to the side it belongs to instead.
    auto& sides = m_swap_sides.local();
    sides.by_vertex.clear();
    // Carried to swap_after_cells(), which refreshes exactly these four; see SwapSurfaceSides.
    sides.abcd = {{a, b, c, d}};
    for (const size_t tid : tids) {
        const std::pair<CellTag, int> side{m_tet_attribute[tid].tag, m_tet_attribute[tid].label};
        for (const size_t v : oriented_tet_vids(tid)) {
            if (v == a || v == b || v == c || v == d) continue;
            const auto [it, inserted] = sides.by_vertex.try_emplace(v, side);
            if (!inserted && it->second != side) return false;
        }
    }
    return true;
}

bool TopoOffsetTetMesh::swap_capture_tag(const std::vector<size_t>& tids)
{
    std::map<CellTag, size_t> tag_count;
    std::set<int> labels;
    for (const size_t t : tids) {
        tag_count[m_tet_attribute[t].tag]++;
        labels.insert(m_tet_attribute[t].label);
    }
    // Region membership is read from the construction label, and a swap reuses recycled tet slots
    // carrying whatever label was there before, so the label must be carried across explicitly. A
    // ring spanning two labels has a region boundary running through it, and is refused.
    if (labels.size() > 1) {
        return swap_reject(SwapReject::app_capture_label);
    }
    m_swap_label.local() = *labels.begin();
    // A face between differently tagged tets is the offset surface, so a ring spanning two tags
    // has that surface running through it: refused, because one tag for the whole ring moves it.
    // Do not re-add "majority tag wins"; measured worse -- see git history of this file.
    if (tag_count.size() > 1) {
        return swap_reject(SwapReject::app_capture_tag);
    }

    size_t max_count = 0;
    CellTag max_tag;
    for (const auto& [tag, count] : tag_count) {
        if (count > max_count) {
            max_count = count;
            max_tag = tag;
        }
    }
    m_swap_tag.local() = max_tag;
    return true;
}

bool TopoOffsetTetMesh::swap_before_interior(const std::vector<size_t>& tids)
{
    // m_face_extra and the edge label, as the base's face_attribute_tracker(); swap_after_cells()
    // assigns them.
    face_extra_capture(tids, m_face_extra_swap.local());
    edge_label_capture(tids, m_edge_label_swap.local());
    // Clear whatever the last surface flip on this thread left behind: the flags outlive one
    // operation, and an interior swap is no flip of the offset surface.
    {
        SwapSurfaceSides& sides = m_swap_sides.local();
        sides.worthwhile = false;
        sides.saw_case = false;
    }
    m_swap_folds_before.local() = -1; // an interior swap changes no offset face
    m_swap_front.local() = SwapFront{}; // likewise: no front rule
    m_swap_plastic_before.local() = -1.;
    if (!swap_capture_tag(tids)) return false;
    // An interior swap changes no offset-surface face: the base's max rule decides
    // (swap_quality_allowed(), the base case scores) -- on PLASTIC AMIPS under use_rest_pose
    // (the before half here, the rule in swap_after_cells(), the case scores on it too).
    if (m_plastic_active && !tetwild_rules()) {
        double mx = 0.;
        for (const size_t tid : tids) mx = std::max(mx, plastic_quality(tid));
        m_swap_plastic_before.local() = mx;
    }
    // DEBUG_pinned_counts: counted, never refused (see m_pinned_counts).
    if (m_offset_params.debug_pinned_counts) pinned_count_swap(tids);
    return true;
}

bool TopoOffsetTetMesh::swap_before_surface(
    const std::vector<size_t>& tids,
    const size_t a,
    const size_t b,
    const size_t c,
    const size_t d)
{
    // m_face_extra and the edge label, as the base's face_attribute_tracker(); swap_after_cells()
    // assigns them.
    face_extra_capture(tids, m_face_extra_swap.local());
    edge_label_capture(tids, m_edge_label_swap.local());
    // Re-decided below, for this flip alone. Cleared first, so a refusal on any path below
    // leaves nothing behind for the funnel. swap_before_interior() clears them too.
    {
        SwapSurfaceSides& sides = m_swap_sides.local();
        sides.worthwhile = false;
        sides.saw_case = false;
    }
    m_swap_folds_before.local() = -1; // set below for a flip of the offset surface
    m_swap_front.local() = SwapFront{}; // likewise: the front max rule's before half
    m_swap_plastic_before.local() = -1.; // set at the end, never for an offset flip

    // NOT swap_capture_tag(): that is the interior rule, and on this path it can never pass --
    // the ring of a surface flip always spans band and background, which is what makes its faces
    // offset surface in the first place. Calling it here made every offset-surface flip
    // impossible; the instrumentation counted 5642 of 5642 application refusals on its label
    // test alone. See swap_capture_surface_sides().
    if (!swap_capture_surface_sides(tids, a, b, c, d)) {
        return swap_reject(SwapReject::app_capture_label);
    }

    // The flip replaces surface faces (a,b,c) and (a,b,d) with (a,c,d),(b,c,d). Both must belong
    // to the same tracked surface: a mixed flip would hand a triangle of the input complex to the
    // offset surface, or the reverse, moving the line where one meets the other.
    const auto [ftup_abc, fid_abc] = tuple_from_face(std::array<size_t, 3>{{a, b, c}});
    const auto [ftup_abd, fid_abd] = tuple_from_face(std::array<size_t, 3>{{a, b, d}});
    if (fid_abc == static_cast<size_t>(-1) || fid_abd == static_cast<size_t>(-1)) {
        return swap_reject(SwapReject::app_fid_missing);
    }
    if (m_face_attribute[fid_abc].m_surface_class != m_face_attribute[fid_abd].m_surface_class) {
        return swap_reject(SwapReject::app_class_mismatch);
    }
    // A flip across a junction would detach the new diagonal from one of the boundaries the old
    // faces lay on: a region-class flip is refused unless both faces separate the same two tag
    // sets (the wall counting as its own side), read live off the incident cells. The engine's
    // reason code for it is app_mask_mismatch. An offset-surface flip is
    // swap_capture_surface_sides()'s to judge.
    if (m_face_attribute[fid_abc].m_surface_class != OFFSET_SURFACE_CLASS) {
        const auto sides = [&](const Tuple& ft) {
            const std::optional<Tuple> o = ft.switch_tetrahedron(*this);
            std::pair<CellTag, CellTag> p{
                m_tet_attribute[ft.tid(*this)].tag,
                o ? m_tet_attribute[o->tid(*this)].tag : CellTag{}};
            if (o && p.second < p.first) std::swap(p.first, p.second);
            return std::make_pair(bool(o), p);
        };
        if (sides(ftup_abc) != sides(ftup_abd)) {
            return swap_reject(SwapReject::app_mask_mismatch);
        }
    }
    // A flip OF THE OFFSET SURFACE -- both faces it replaces on the front, band on one side and
    // background on the other, asked live of the labels rather than read from the cached
    // m_surface_class, since these operations run between one labelling pass and the next -- is
    // followed through the rest of the swap by [flip funnel], and judged by the front max rule
    // (swap_after_cells()).
    if (face_is_offset_surface_live(ftup_abc) && face_is_offset_surface_live(ftup_abd)) {
        // growth_mode "individual_op_reject": the folded faces among the two the flip replaces
        // (their band cells are in the ring), for swap_after_cells().
        if (fold_reject_active()) {
            m_swap_folds_before.local() =
                int(face_folded_in(a, b, c, tids)) + int(face_folded_in(a, b, d, tids));
        }
        // THE FRONT MAX RULE's before half (front_op_allowed()): the max stencil-point offset
        // error over the points the flip changes -- the diagonal (a, b)'s and the two replaced
        // faces' interior points (stencil_points_of_diagonal(); an unmeasurable point reads 0, on
        // both sides alike) -- and the max AMIPS^3 over the cells it replaces;
        // swap_after_cells() compares the points and the cells it makes against them. Not in a
        // TetWild pass (its own rules).
        if (!tetwild_rules()) {
            SwapFront& sf = m_swap_front.local();
            const StencilMax om = stencil_max(stencil_points_of_diagonal(a, b, c, d));
            sf.o_before = om.max;
            sf.o_points = om.n;
            sf.a_before = 0.;
            for (const size_t tid : tids) {
                sf.a_before = std::max(sf.a_before, plastic_quality(tid));
            }
        }
        SwapSurfaceSides& sides = m_swap_sides.local();
        sides.worthwhile = true;
        sides.kind = static_cast<int>(tids.size());
        // The funnel follows real attempts: a screened pass's dry run (see TetMesh::dry_run())
        // is followed by the real attempt, which counts the flip itself.
        if (!dry_run()) {
            ++funnel_offered;
            if (sides.kind >= 3 && sides.kind <= 5) ++funnel_kind[size_t(sides.kind - 3)];
        }
    }

    // Non-offset surface flips are not refused categorically: the shared swap checks both new
    // triangles with surface_triangle_is_outside(), which holds a held face to m_envelope, and
    // that envelope is the geometric constraint. The class-match and same-boundary refusals above
    // are the topology half; the base's max rule (swap_quality_allowed()) is the quality half --
    // on PLASTIC AMIPS under use_rest_pose (never for a flip of the offset surface, which the
    // front max rule judges).
    if (m_plastic_active && !tetwild_rules() && m_swap_front.local().o_before < 0.) {
        double mx = 0.;
        for (const size_t tid : tids) mx = std::max(mx, plastic_quality(tid));
        m_swap_plastic_before.local() = mx;
    }
    return true;
}

void TopoOffsetTetMesh::warn_if_offset_reaches_domain_boundary() const
{
    // A band face with no opposite tet lies ON the domain boundary: the band ran out of room
    // before reaching target_distance. Counted in vertices as well as faces because the vertices
    // are what is pinned.
    size_t n_faces = 0, n_verts = 0;
    std::vector<bool> counted(vert_capacity(), false);
    for (const Tuple& f : get_faces()) {
        if (f.switch_tetrahedron(*this)) continue; // interior face; the band has room here
        if (!cell_is_offset_band(f.tid(*this))) continue;
        ++n_faces;
        for (const size_t v : get_face_vids(f)) {
            if (!counted[v]) {
                counted[v] = true;
                ++n_verts;
            }
        }
    }
    if (n_faces == 0) return;

    logger().warn(
        "Offset band reaches the domain boundary: {} band faces ({} vertices) lie ON the "
        "bounding box. target_distance ({}) exceeds the clearance between the input complex and "
        "the box, so the offset is CLIPPED there and cannot reach the target distance -- those "
        "vertices are on the frozen bounding box and no operation may move them. They ARE "
        "included in max_dist_err / avg_dist_err (see compute_distance_deviation), so expect the "
        "reported error to be dominated by them and to stay flat across iterations. Reduce "
        "target_distance, or pad the background mesh.",
        n_faces,
        n_verts,
        m_offset_params.target_distance);
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTetMesh::amips3_energy(
    const size_t vid) const
{
    // See the declaration. The cells exactly as the shared smoother assembles them for its own
    // AMIPS term (smooth_vertex_3d()): the moving vertex first, orientation preserved.
    std::vector<std::array<double, 12>> cells;
    // use_rest_pose: a plastic cell with a valid rest goes to VolAMIPSEnergy3D against that rest,
    // exactly as tet_energy() reads it; every other cell to CubedAMIPSEnergy3D.
    std::vector<VolAMIPSEnergy3D::Cell> plastic;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        const std::array<size_t, 4> orig = oriented_tet_vids(tid);
        const auto vs = wmtk::orient_preserve_tet_reorder(orig, vid);
        Eigen::Matrix3d R_orig;
        if (plastic_rest(tid, R_orig)) {
            // The rest's corners re-ordered as vs is: rest_pos is in orig's order.
            const TetAttributes& ta = m_tet_attribute[tid];
            std::array<int, 4> from{};
            for (int k = 0; k < 4; ++k) {
                for (int j = 0; j < 4; ++j) {
                    if (orig[size_t(j)] == vs[size_t(k)]) from[size_t(k)] = j;
                }
            }
            Eigen::Matrix3d R;
            for (int k = 1; k < 4; ++k) {
                R.col(k - 1) = ta.rest_pos[size_t(from[size_t(k)])] - ta.rest_pos[size_t(from[0])];
            }
            std::array<Vector3d, 3> q;
            for (int k = 1; k < 4; ++k) q[size_t(k - 1)] = m_vertex_attribute[vs[k]].m_posf;
            VolAMIPSEnergy3D::Cell c;
            if (VolAMIPSEnergy3D::cell(q, R, c)) {
                plastic.push_back(c);
                continue;
            }
        }
        std::array<double, 12> c;
        for (int k = 0; k < 4; ++k) {
            for (int j = 0; j < 3; ++j) c[size_t(k * 3 + j)] = m_vertex_attribute[vs[k]].m_posf[j];
        }
        cells.push_back(c);
    }
    // (w / s^3) sum vol AMIPS^3, the energy's AMIPS term -- a volume, as the band term is.
    auto elastic = std::make_shared<CubedAMIPSEnergy3D>(std::move(cells), amips_term_weight());
    if (plastic.empty()) return elastic;
    auto sum = std::make_shared<optimization::EnergySum>();
    sum->add_energy(elastic);
    sum->add_energy(std::make_shared<VolAMIPSEnergy3D>(std::move(plastic), amips_term_weight()));
    return sum;
}

bool TopoOffsetTetMesh::ring_has_plastic_rest(const size_t vid) const
{
    if (!m_plastic_active || tetwild_rules()) return false;
    Eigen::Matrix3d R;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        if (plastic_rest(tid, R)) return true;
    }
    return false;
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTetMesh::plastic_amips_energy(
    const size_t vid) const
{
    // See the declaration: TetWild's raw AMIPS sum, each plastic cell against its stamped rest
    // and every other cell against the regular tet (PlasticAMIPSEnergy3D reads the engine's value
    // on those). The cells are assembled exactly as amips3_energy() assembles its plastic ones:
    // the moving vertex first, orientation preserved, the rest's corners re-ordered with it.
    if (!ring_has_plastic_rest(vid)) return nullptr;
    const Eigen::Matrix3d regular = VolAMIPSEnergy3D::regular_rest();
    std::vector<PlasticAMIPSEnergy3D::Cell> cells;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        const std::array<size_t, 4> orig = oriented_tet_vids(tid);
        const auto vs = wmtk::orient_preserve_tet_reorder(orig, vid);
        std::array<Vector3d, 3> q;
        for (int k = 1; k < 4; ++k) q[size_t(k - 1)] = m_vertex_attribute[vs[k]].m_posf;
        Eigen::Matrix3d R_orig;
        PlasticAMIPSEnergy3D::Cell c;
        if (plastic_rest(tid, R_orig)) {
            // The rest's corners re-ordered as vs is: rest_pos is in orig's order.
            const TetAttributes& ta = m_tet_attribute[tid];
            std::array<int, 4> from{};
            for (int k = 0; k < 4; ++k) {
                for (int j = 0; j < 4; ++j) {
                    if (orig[size_t(j)] == vs[size_t(k)]) from[size_t(k)] = j;
                }
            }
            Eigen::Matrix3d R;
            for (int k = 1; k < 4; ++k) {
                R.col(k - 1) = ta.rest_pos[size_t(from[size_t(k)])] - ta.rest_pos[size_t(from[0])];
            }
            if (VolAMIPSEnergy3D::cell(q, R, c)) {
                cells.push_back(c);
                continue;
            }
        }
        if (VolAMIPSEnergy3D::cell(q, regular, c)) cells.push_back(c);
    }
    if (cells.empty()) return nullptr;
    return std::make_shared<PlasticAMIPSEnergy3D>(std::move(cells), m_params.w_amips);
}

bool TopoOffsetTetMesh::smooth_nonfront_vertex(const Tuple& t)
{
    // See the declaration. TetOptimizerMesh::smooth_after()'s options as TetWild runs them --
    // the engine's own AMIPS term (w_amips), no extra energy (smoothing_extra_energy() returns
    // null here), the quality veto on the ring's max -- plus this component's damping
    // (opts.max_step) and the fold check below. use_rest_pose: a ring with a plastic cell takes
    // the same raw AMIPS sum with that cell against its rest from smoothing_extra_energy()
    // instead (plastic_amips_energy(); w_amips 0, so the engine adds no second term), and the
    // quality veto reads plastic AMIPS through smoothing_cell_energy().
    const size_t vid = t.vid(*this);
    optimization::SmoothVertexOptions opts;
    opts.w_amips = ring_has_plastic_rest(vid) ? 0. : m_params.w_amips;
    opts.w_envelope = m_params.w_envelope;
    opts.s_amips = m_s_amips;
    opts.s_envelope = m_s_envelope;
    opts.two_stage = false;
    opts.smoothing_mode = m_params.smoothing_mode == "exact"
                              ? optimization::SmoothVertexOptions::SmoothingMode::Exact
                              : optimization::SmoothVertexOptions::SmoothingMode::Projected;
    opts.project_line_search_steps = m_params.project_line_search_steps;
    opts.project_line_search_nested_steps = m_params.project_line_search_nested_steps;
    opts.quality_veto = m_params.smooth_quality_veto;
    opts.max_step = damping_step(vid); // EXPERIMENTAL_damping; infinity when off
    const Vector3d x0 = std::as_const(m_vertex_attribute)[vid].m_posf;
    // growth_mode "individual_op_reject": a front vertex held on a surface (the domain wall, say)
    // moves its offset faces too; the move may not raise the folded ones among them.
    int folds_before = -1;
    if (fold_reject_active() && m_vertex_extra[vid].m_is_on_offset) {
        std::set<std::array<size_t, 3>> f;
        folded_front_faces_at(vid, f);
        folds_before = int(f.size());
    }
    if (!optimization::smooth_vertex_3d(
            *this,
            t,
            opts,
            m_solver.local(),
            &m_smooth_rejects,
            &m_newton)) {
        return false;
    }
    if (folds_before >= 0) {
        std::set<std::array<size_t, 3>> f;
        folded_front_faces_at(vid, f);
        if (int(f.size()) > folds_before) {
            ++m_fold_reject_smooth;
            --m_smooth_rejects.accepted;
            ++m_smooth_rejects.quality;
            return false;
        }
    }
    count_damped_move(vid, x0, opts.max_step);
    return true;
}

double TopoOffsetTetMesh::damping_step(const size_t vid) const
{
    // See the declaration. Every edge at vid lies in a cell of its one-ring, so the ring's corners
    // other than vid are exactly its neighbours.
    if (!damping_active()) return std::numeric_limits<double>::infinity();
    const Vector3d& x = m_vertex_attribute[vid].m_posf;
    double shortest = std::numeric_limits<double>::infinity();
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        for (const size_t u : oriented_tet_vids(tid)) {
            if (u != vid) shortest = std::min(shortest, (m_vertex_attribute[u].m_posf - x).norm());
        }
    }
    return m_offset_params.damping_factor * shortest;
}

void TopoOffsetTetMesh::count_damped_move(
    const size_t vid,
    const Vector3d& x0,
    const double max_step)
{
    if (!damping_active()) return;
    ++m_damping_moves;
    const double moved = (std::as_const(m_vertex_attribute)[vid].m_posf - x0).norm();
    if (moved >= max_step * (1. - 1e-9)) ++m_damping_bounded;
}

void TopoOffsetTetMesh::count_damped_front_move(
    const size_t vid,
    const Vector3d& x0,
    const double max_step)
{
    // See the declaration. Log-only: reads the mesh and the input, changes nothing.
    if (!damping_active()) return;
    ++m_damping_front_moves;
    const Vector3d m = std::as_const(m_vertex_attribute)[vid].m_posf - x0;
    if (!(m.norm() >= max_step * (1. - 1e-9))) return;
    ++m_damping_front_bounded;
    const Envelope& env = *m_input_complex_envelope;
    const int64_t prim = nearest_primitive(env, m_input_complex_pad, x0);
    Vector3d g = Vector3d::Zero();
    const double d0 = prim < 0 ? 0. : env.primitive_distance(prim, x0, &g);
    if (prim < 0 || !(g.norm() > 0.)) {
        ++m_damping_front_nograd;
        return;
    }
    g.normalize();
    const double along = m.dot(g);
    if (std::abs(along) >= (m - along * g).norm()) ++m_damping_front_along;
    const double delta = m_offset_params.target_distance;
    if (along * (delta - d0) > 0.) ++m_damping_front_toward;
    if (std::abs(d0 - delta) > max_step) ++m_damping_front_far;
}

double TopoOffsetTetMesh::swap_edge_44_energy(
    const std::vector<std::array<size_t, 4>>& tets,
    const int op_case)
{
    // See the declaration: the base's score (the max of AMIPS^3 over the cells), except that a
    // flip of the offset surface scores the CURRENT cells (op_case 0) double::max(): the search
    // never keeps the current cells on quality grounds, it picks the best-shaped candidate, and
    // the offset error over the flip's two faces decides in swap_after_cells(). Under
    // use_rest_pose (m_swap_plastic_before taken) the score is PLASTIC AMIPS: op_case 0 is the
    // before half, a candidate the max of candidate_plastic_quality() over its cells with the
    // label each will take (an inverted cell double::max(), as the base scores it).
    // DEBUG_pinned_counts: each candidate case, before any delegation (see m_pinned_counts).
    // Case 0 is the current cells.
    if (m_offset_params.debug_pinned_counts && op_case >= 1) pinned_count_case(tets);
    SwapSurfaceSides& sides = m_swap_sides.local();
    const bool front_flip = !tetwild_rules() && sides.worthwhile;
    const double plastic_before =
        !tetwild_rules() && !front_flip ? m_swap_plastic_before.local() : -1.;
    double e = -1.;
    if (front_flip && op_case == 0) {
        e = std::numeric_limits<double>::max();
    } else if (plastic_before >= 0. && op_case == 0) {
        e = plastic_before;
    } else if (plastic_before >= 0.) {
        e = 0.;
        for (const std::array<size_t, 4>& vids : tets) {
            if (is_inverted(vids)) {
                e = std::numeric_limits<double>::max();
                break;
            }
            int label = m_swap_label.local();
            for (const size_t v : vids) {
                const auto it = sides.by_vertex.find(v);
                if (it != sides.by_vertex.end()) {
                    label = it->second.second;
                    break;
                }
            }
            e = std::max(e, candidate_plastic_quality(vids, label));
        }
    } else {
        e = TetOptimizerMesh::swap_edge_44_energy(tets, op_case);
    }
    if (!front_flip || dry_run()) return e; // the funnel follows real attempts
    if (op_case == 0) {
        sides.case0_energy = e;
        return e;
    }
    // a case that survived swap_edge_44_accept_case(), i.e. one that really does make the (c,d)
    // diagonal. The flip is counted once, for the funnel; its cases one by one.
    if (!sides.saw_case) {
        sides.saw_case = true;
        ++funnel_cases;
    }
    if (e == std::numeric_limits<double>::max())
        ++funnel_case_inverted;
    else if (!(e < sides.case0_energy))
        ++funnel_case_not_better;
    else
        ++funnel_case_better;
    return e;
}

double TopoOffsetTetMesh::swap_edge_56_energy(
    const std::vector<std::array<size_t, 4>>& tets,
    const int op_case)
{
    // The same score and the same counting: the engine keeps one hook per swap kind, and the
    // base's two are identical too.
    return swap_edge_44_energy(tets, op_case);
}

std::string TopoOffsetTetMesh::flip_funnel_report() const
{
    // See the declaration for how to read it.
    return fmt::format(
        "offset-surface flips {} (3-2 {}, 4-4 {}, 5-6 {}) -> a case was scored for {} -> "
        "reached swap_after_cells {} -> sides captured, energy rule asked {} -> passed it, "
        "committed {} | cases (energy): inverted {}, not better than the current cells {}, "
        "better {}",
        funnel_offered.load(),
        funnel_kind[0].load(),
        funnel_kind[1].load(),
        funnel_kind[2].load(),
        funnel_cases.load(),
        funnel_after_cells.load(),
        funnel_energy.load(),
        funnel_committed.load(),
        funnel_case_inverted.load(),
        funnel_case_not_better.load(),
        funnel_case_better.load());
}

void TopoOffsetTetMesh::flip_funnel_reset()
{
    funnel_offered = 0;
    for (auto& k : funnel_kind) k = 0;
    funnel_cases = 0;
    funnel_case_inverted = 0;
    funnel_case_not_better = 0;
    funnel_case_better = 0;
    funnel_after_cells = 0;
    funnel_energy = 0;
    funnel_committed = 0;
}

bool TopoOffsetTetMesh::swap_after_cells(const std::vector<size_t>& tids, bool is_surface_flip)
{
    // m_face_extra, as the base's tracker_assign_after() does m_face_attribute after this hook:
    // a new cell's face that was a face of the old cells keeps its value, every other is reset.
    // A refusal below rolls it back with the rest. The edge label likewise; both are closed once
    // the new cells' labels are written below (close_labels()).
    face_extra_assign(tids, m_face_extra_swap.local());
    edge_label_assign(tids, m_edge_label_swap.local());
    // THE FRONT MAX RULE for a flip OF THE OFFSET SURFACE (since 2026-10-26, front_op_allowed()):
    // the max stencil-point offset error over the points it makes against those it replaces,
    // and the max AMIPS^3 over the cells it makes (labelled and stamped by the time this runs)
    // against the cells it replaces, each against its bound (the before halves taken by
    // swap_before_surface(), m_swap_front). STRICTLY lower, because a swap pass has to terminate:
    // a tie could flip back and forth forever. Every other swap was judged by the base's max rule
    // before this hook (swap_quality_allowed(), the base case scores and face gate) and passes
    // here.
    //
    // A refusal is rolled back by the engine. It is counted as app_sag_raised, the engine's name
    // for this after-hook refusal (SwapReject lives in the engine, which this does not change).
    const auto energy_lowered = [&]() {
        if (tetwild_rules()) return true;
        const SwapFront& sf = m_swap_front.local();
        if (sf.o_before >= 0.) {
            // The new diagonal (c, d)'s and the two new faces' interior points.
            const auto& q = m_swap_sides.local().abcd;
            const StencilMax om = stencil_max(stencil_points_of_diagonal(q[2], q[3], q[0], q[1]));
            double a_after = 0.;
            for (const size_t t : tids) a_after = std::max(a_after, plastic_quality(t));
            if (front_op_allowed(
                    sf.o_before,
                    om.max,
                    sf.a_before,
                    a_after,
                    front_offset_bar(),
                    front_amips_bar(),
                    /*strict=*/true,
                    /*offset_changes=*/sf.o_points > 0 || om.n > 0)) {
                return true;
            }
            ++m_volume_reject_swap;
            return swap_reject(SwapReject::app_sag_raised);
        }
        // THE PLASTIC SWAP RULE (use_rest_pose, 2026-10-22): the base's strict max rule on
        // plastic AMIPS over the cells the swap made (labelled and stamped above, so a plastic
        // one reads 27) against the max over the cells it replaced (swap_before_interior() /
        // swap_before_surface()). Strict, as the base is: an all-plastic swap at rest reads
        // 27 < 27 and is refused, so a swap pass terminates.
        const double plastic_before = m_swap_plastic_before.local();
        if (plastic_before >= 0.) {
            double after = 0.;
            for (const size_t t : tids) after = std::max(after, plastic_quality(t));
            if (after < plastic_before) return true;
            ++m_plastic_reject_swap;
            if (current_op_kind() == OpKind::swap_face) {
                return swap_reject_kind_only(SwapReject::app_sag_raised);
            }
            return swap_reject(SwapReject::app_sag_raised);
        }
        return true; // no face and no plastic rule: the base's max rule decided already
    };

    if (!is_surface_flip) {
        // Interior: one tag and one label for the whole ring, captured by swap_capture_tag().
        const CellTag& tag = m_swap_tag.local();
        const int label = m_swap_label.local();
        for (const size_t t : tids) {
            m_tet_attribute[t].tag = tag;
            m_tet_attribute[t].label = label;
            // use_rest_pose: a cell the swap creates is stamped at creation.
            stamp_rest_cell(t);
        }
        close_labels(tids);
        if (m_repulsion_potential && !repulsion_embedding_kept(tids)) {
            ++m_repulsion_embed_refused;
            return false;
        }
        if (!energy_lowered()) return false;
        ++iter_cnt_swap;
        return true;
    }

    SwapSurfaceSides& sides = m_swap_sides.local();
    if (sides.worthwhile) ++funnel_after_cells;

    // Surface flip: the ring is two-sided and the flip keeps both sides, so each new cell takes
    // the side of a ring vertex it contains (swap_capture_surface_sides()). A cell containing no
    // ring vertex cannot be placed on a side and the flip is refused rather than guessed -- the
    // base turns that into a rollback. Two ring vertices disagreeing inside one new cell means
    // the flip would straddle the interface, which is the case this must not let through.
    for (const size_t t : tids) {
        const std::pair<CellTag, int>* side = nullptr;
        for (const size_t v : oriented_tet_vids(t)) {
            const auto it = sides.by_vertex.find(v);
            if (it == sides.by_vertex.end()) continue;
            if (side != nullptr && *side != it->second) {
                return swap_reject(SwapReject::app_after_side_conflict);
            }
            side = &it->second;
        }
        if (side == nullptr) return swap_reject(SwapReject::app_after_no_side);
        m_tet_attribute[t].tag = side->first;
        m_tet_attribute[t].label = side->second;
        stamp_rest_cell(t); // created by the swap: stamped at creation, as above
    }
    close_labels(tids);
    // growth_mode "individual_op_reject": the flip may not raise the folded faces among the two it
    // changes -- (a,b,c), (a,b,d) out, (a,c,d), (b,c,d) in, whose band cells are new cells now.
    // Counted as app_sag_raised, the engine's name for a flip that makes the surface worse.
    if (const int before = m_swap_folds_before.local(); before >= 0) {
        const auto& q = sides.abcd;
        const int after = int(face_folded_in(q[0], q[2], q[3], tids)) +
                          int(face_folded_in(q[1], q[2], q[3], tids));
        if (after > before) {
            ++m_fold_reject_swap;
            return swap_reject(SwapReject::app_sag_raised);
        }
    }
    if (sides.worthwhile) ++funnel_energy;
    if (m_repulsion_potential && !repulsion_embedding_kept(tids)) {
        ++m_repulsion_embed_refused;
        return false;
    }
    if (!energy_lowered()) return false;
    // Only now, with every new cell's label written: the refresh reads labels, so it has to run
    // after the loop that sets them. The flip's net surface change is -(a,b,c) -(a,b,d) +(a,c,d)
    // +(b,c,d), so a and b can lose their last offset face and c and d can gain their first; no
    // other vertex's answer moves. This runs before the base's Hausdorff check on the two new
    // surface faces, so a flip refused there is rolled back -- m_vertex_extra is registered in
    // m_vertex_attr_group, so these writes roll back with it.
    for (const size_t v : sides.abcd) refresh_offset_membership(v);
    if (sides.worthwhile) ++funnel_committed;
    ++iter_cnt_swap;
    return true;
}

bool TopoOffsetTetMesh::collapse_edge_after(const Tuple& t)
{
    if (!TetOptimizerMesh::collapse_edge_after(t)) {
        if (m_trace_collapse.open) trace_collapse_end("refused after");
        return false;
    }
    const size_t v2_id = collapse_cache.local().v2_id;
    // repulsion_rounds: the complex must stay simplicially embedded (repulsion_embedding_kept());
    // every cell the collapse made is in the survivor's ring.
    if (m_repulsion_potential && !repulsion_embedding_kept(get_one_ring_tids_for_vertex(v2_id))) {
        ++m_repulsion_embed_refused;
        if (m_trace_collapse.open) trace_collapse_end("refused after (repulsion embedding)");
        return false;
    }
    if (m_trace_collapse.open) trace_collapse_end("committed");
    // The energy rule has run by now, inside the base's call above (collapse_after_connectivity(),
    // which says why there); a collapse that reaches this line has passed it.
    if (!m_offset_params.sizing_collapse_min) { // see collapse_edge_before()
        m_vertex_attribute[v2_id].m_sizing_scalar = m_collapse_survivor_sizing.local();
    }
    return true;
}

bool TopoOffsetTetMesh::collapse_edge_before(const Tuple& t)
{
    // The collapse length gate lives in the shared pass, which filters the candidate list against
    // collapsing_l2 scaled by the endpoints' sizing scalars.
    const bool traced = trace_collapse_begin(t); // DEBUG_trace_collapse, log-only
    if (!TetOptimizerMesh::collapse_edge_before(t)) {
        if (traced) trace_collapse_end("refused before");
        return false;
    }
    // The survivor's own sizing scalar, for sizing_collapse_min = false: the base collapse
    // overwrites it with the min of the two, and collapse_edge_after() puts it back.
    // collapse_cache is the base's, filled by the call above; v2 survives.
    m_collapse_survivor_sizing.local() =
        m_vertex_attribute[collapse_cache.local().v2_id].m_sizing_scalar;
    // Applied unconditionally, where the base asks only when both endpoints already sit on a
    // tracked simplex: the offset region is a thin shell, so a collapse with one endpoint in the
    // interior can still pinch its two sides together while every tracked surface survives.
    if (!substructure_link_condition(t)) {
        const bool r = collapse_reject(CollapseReject::app_substructure_link);
        if (traced) trace_collapse_end("refused before");
        return r;
    }
    // growth_mode "individual_op_reject": the folded faces through the two ends, for
    // collapse_after_connectivity(); -1 (no rule) when neither end is on the front.
    {
        const auto& cc = collapse_cache.local();
        const bool front =
            m_vertex_extra[cc.v1_id].m_is_on_offset || m_vertex_extra[cc.v2_id].m_is_on_offset;
        int& folds = m_collapse_folds_before.local();
        folds = -1;
        if (fold_reject_active() && front) {
            std::set<std::array<size_t, 3>> f;
            folded_front_faces_at(cc.v1_id, f);
            folded_front_faces_at(cc.v2_id, f);
            folds = int(f.size());
        }
        // THE FRONT MAX RULE's before half (see m_collapse_front): the max stencil-point offset
        // error over the points the collapse changes (stencil_points_changed_at() its two ends),
        // the max AMIPS^3 over v1's one-ring and its tids. Setting o_before also makes
        // collapse_quality_allowed() admit the collapse: the front max rule judges it
        // (collapse_after_connectivity()).
        CollapseFront& cf = m_collapse_front.local();
        cf.o_before = -1.;
        if (front && !tetwild_rules() && !m_coarsen_mode && m_offset_potential) {
            const StencilMax om = stencil_max(stencil_points_changed_at(cc.v1_id, cc.v2_id));
            cf.o_before = om.max;
            cf.o_points = om.n;
            cf.tids = get_one_ring_tids_for_vertex(cc.v1_id);
            cf.a_before = 0.;
            for (const size_t tid : cf.tids)
                cf.a_before = std::max(cf.a_before, plastic_quality(tid));
            std::sort(cf.tids.begin(), cf.tids.end());
        }
        const double err = cf.o_before;
        // THE PLASTIC COLLAPSE RULE's before half (use_rest_pose, see m_collapse_plastic): the
        // base's max rule on PLASTIC AMIPS -- the max of plastic_quality() over v1's one-ring,
        // judged against the reshaped cells in collapse_after_connectivity(). Never beside the
        // front rule, whose face error stands alone.
        CollapsePlastic& cp = m_collapse_plastic.local();
        cp.before = -1.;
        if (m_plastic_active && !tetwild_rules() && !m_coarsen_mode && err < 0.) {
            cp.tids = get_one_ring_tids_for_vertex(cc.v1_id);
            double mx = 0.;
            for (const size_t tid : cp.tids) mx = std::max(mx, plastic_quality(tid));
            std::sort(cp.tids.begin(), cp.tids.end());
            cp.before = mx;
            cp.v1_rounded = m_vertex_attribute[cc.v1_id].m_is_rounded;
        }
    }
    return true;
}

bool TopoOffsetTetMesh::collapse_before_vertex(
    const size_t v1_id,
    const size_t v2_id,
    const double edge_length)
{
    // m_face_extra, as the base's collapse_edge_before() caches m_face_attribute: around the edge
    // each cell (v1, v2, l1, l2) has faces (v1, l1, l2) and (v2, l1, l2), which become the one
    // face (v2, l1, l2), merged; collapse_after_vertex() writes it.
    {
        auto& faces = m_face_extra_collapse.local();
        faces.clear();
        for (const Tuple& t : get_incident_tets_for_edge(v1_id, v2_id)) {
            std::array<size_t, 2> l{{0, 0}};
            int n = 0;
            for (const size_t u : oriented_tet_vids(t.tid(*this))) {
                if (u != v1_id && u != v2_id) l[size_t(n++)] = u;
            }
            FaceExtra fe = m_face_extra.at(std::get<1>(tuple_from_face({{v1_id, l[0], l[1]}})));
            fe.merge(m_face_extra.at(std::get<1>(tuple_from_face({{v2_id, l[0], l[1]}}))));
            faces.push_back({fe, l});
        }
    }
    // The edge label: every edge of the two rings, v1 renamed v2, so (v1, l) and (v2, l) merge
    // (closed_label()); collapse_after_vertex() writes them onto v2's ring.
    {
        std::vector<size_t> tids = get_one_ring_tids_for_vertex(v1_id);
        for (const size_t t : get_one_ring_tids_for_vertex(v2_id)) tids.push_back(t);
        std::sort(tids.begin(), tids.end());
        tids.erase(std::unique(tids.begin(), tids.end()), tids.end());
        edge_label_capture(tids, m_edge_label_collapse.local(), v1_id, v2_id);
    }
    // Diagnostic: the flattest cell this collapse is about to reshape, read back by
    // record_flatness() in collapse_after_vertex().
    {
        double f = 1.;
        for (const size_t tid : get_one_ring_tids_for_vertex(v1_id)) {
            f = std::min(f, tet_flatness(tid));
        }
        m_collapse_parent_flatness.local() = f;
    }

    // The link of the edge about to be collapsed: the only vertices besides v2 whose offset
    // membership this collapse can change. Captured here because the edge no longer exists in
    // collapse_after_vertex(), which is where the refresh runs. See m_collapse_edge_link.
    {
        std::vector<size_t>& link = m_collapse_edge_link.local();
        link.clear();
        for (const size_t tid : get_incident_tids_for_edge(v1_id, v2_id)) {
            for (const size_t w : oriented_tet_vids(tid)) {
                if (w != v1_id && w != v2_id) link.push_back(w);
            }
        }
        wmtk::vector_unique(link);
    }

    const auto& VE = m_vertex_extra;

    // v1 is the vertex the collapse removes; it merges into v2, which keeps its position. A
    // vertex on any tracked boundary may be removed provided it merges onto a vertex of the same
    // class, the result stays inside its tags' envelopes, and the substructure link condition
    // survives. That is TetWild's rule for its input surface, applied uniformly.

    // Never both surfaces on one vertex: such a vertex sits at distance 0 from the input complex
    // and is asked to sit at target_distance from it at once. Refused here, and asserted
    // independently by check_no_vertex_on_both_surfaces().
    {
        const bool input = VE[v1_id].m_is_on_input || VE[v2_id].m_is_on_input;
        const bool offset = VE[v1_id].m_is_on_offset || VE[v2_id].m_is_on_offset;
        if (input && offset) {
            return collapse_reject(CollapseReject::app_both_surfaces);
        }
    }

    // The front is always length-limited, whatever the pass says: it deliberately has no
    // envelope while it moves, so its sizing field is the only thing bounding its resolution.
    if (!m_collapse_limit_length && VE[v1_id].m_is_on_offset) {
        return collapse_reject(CollapseReject::app_front_unlimited);
    }

    // The base only knows that both endpoints are on SOME tracked surface. A vertex may not leave
    // the particular surface it belongs to, and each class is checked separately.
    if (VE[v1_id].m_is_on_input && !VE[v2_id].m_is_on_input) {
        return collapse_reject(CollapseReject::app_leaves_input);
    }
    if (VE[v1_id].m_is_on_offset && !VE[v2_id].m_is_on_offset) {
        return collapse_reject(CollapseReject::app_leaves_offset);
    }
    if (VE[v1_id].m_is_on_region && !VE[v2_id].m_is_on_region) {
        return collapse_reject(CollapseReject::app_leaves_region);
    }

    // open boundary: an order-2 vertex may not merge into a lower-order one.
    if (edge_length > 0 && m_vertex_attribute[v1_id].m_order == 2 &&
        m_vertex_attribute[v2_id].m_order < 2) {
        return collapse_reject(CollapseReject::app_order2);
    }

    // DEBUG_pinned_counts: counted, never refused (see m_pinned_counts). After every refusal of
    // this hook, so a collapse refused above is not counted; in a TetWild pass too.
    if (m_offset_params.debug_pinned_counts) pinned_count_collapse(v1_id, v2_id);

    return true;
}

bool TopoOffsetTetMesh::collapse_after_connectivity(
    const size_t,
    const size_t v2_id,
    const std::vector<std::array<size_t, 2>>&)
{
    // growth_mode "individual_op_reject": the faces through the survivor are every offset face
    // the collapse changed (it moves no vertex); refused, the base counts after_connectivity and
    // rolls back, as for the energy rule below.
    if (const int before = m_collapse_folds_before.local(); before >= 0) {
        std::set<std::array<size_t, 3>> f;
        folded_front_faces_at(v2_id, f);
        if (int(f.size()) > before) {
            ++m_fold_reject_collapse;
            return false;
        }
    }
    // THE FRONT MAX RULE for a collapse that touches the offset surface (since 2026-10-26,
    // front_op_allowed()): the max stencil-point offset error over the points it changes before
    // (collapse_edge_before(), m_collapse_front) against the survivor's now, and
    // the max AMIPS^3 over v1's one-ring before against the cells the collapse reshaped -- the
    // connectivity is final, the collapse moves no vertex. after <= before is "lower" (a tie
    // passes: the collapse still removes a vertex, so a pass terminates); a NaN refuses. Every
    // other collapse was judged by the base's max rule (collapse_quality_allowed()). Refused
    // HERE, so the base counts after_connectivity and rolls back. Not in coarsening, which keeps
    // its absolute bar below.
    //
    // use_rest_pose: every changed cell is stamped at the change, before anything reads it, as
    // split and swap stamp the cells they make (re-stamping an unchanged cell rewrites the same
    // rest).
    if (!m_coarsen_mode && !tetwild_rules()) {
        for (const size_t tid : get_one_ring_tids_for_vertex(v2_id)) stamp_rest_cell(tid);
        const CollapseFront& cf = m_collapse_front.local();
        if (cf.o_before >= 0.) {
            const StencilMax om = stencil_max(stencil_points_changed_at(v2_id));
            const double o_after = om.max;
            double a_after = 0.;
            for (const size_t tid : get_one_ring_tids_for_vertex(v2_id)) {
                if (!std::binary_search(cf.tids.begin(), cf.tids.end(), tid)) continue;
                a_after = std::max(a_after, plastic_quality(tid));
            }
            if (m_trace_collapse.open) { // DEBUG_trace_collapse: the rule's four maxima
                m_trace_collapse.energy = true;
                m_trace_collapse.e_before = cf.o_before;
                m_trace_collapse.e_after = o_after;
                m_trace_collapse.a_before = cf.a_before;
                m_trace_collapse.a_after = a_after;
            }
            if (!front_op_allowed(
                    cf.o_before,
                    o_after,
                    cf.a_before,
                    a_after,
                    front_offset_bar(),
                    front_amips_bar(),
                    /*strict=*/false,
                    /*offset_changes=*/cf.o_points > 0 || om.n > 0)) {
                ++m_volume_reject_collapse;
                return false;
            }
        }
        // THE PLASTIC COLLAPSE RULE (use_rest_pose, 2026-10-22): the base's max rule on plastic
        // AMIPS. The reshaped cells are v1's old ring in their own slots (the dead ones are gone
        // from the survivor's ring); each was stamped above, so a plastic one reads 27, and only
        // a band or complex cell can really block. An unrounded v1 passes, as the base's rule
        // does; after <= before passes (the base's tie).
        const CollapsePlastic& cp = m_collapse_plastic.local();
        if (cp.before >= 0. && cp.v1_rounded) {
            double after = 0.;
            for (const size_t tid : get_one_ring_tids_for_vertex(v2_id)) {
                if (!std::binary_search(cp.tids.begin(), cp.tids.end(), tid)) continue;
                after = std::max(after, plastic_quality(tid));
            }
            if (!(after <= cp.before)) {
                ++m_plastic_reject_collapse;
                return false;
            }
        }
    }
    // Coarsening keeps an absolute bar besides, because it runs after the loop and trades
    // elements for nothing but the promise that the result is still good. As in 2D.
    if (m_coarsen_mode && m_offset_potential) {
        double after = 0.;
        for (const Tuple& f : offset_surface_faces_live_at(v2_id)) {
            after = std::max(after, face_criterion_rel(f));
        }
        if (after > 1.0) {
            ++iter_cnt_collapse_offset_reject;
            return false;
        }
    }
    return true;
}

void TopoOffsetTetMesh::collapse_after_vertex(const size_t v1_id, const size_t v2_id)
{
    // Diagnostic. Runs after the collapse is committed, so what it sees is real; the survivor's
    // ring is every cell the collapse reshaped.
    for (const size_t tid : get_one_ring_tids_for_vertex(v2_id)) {
        if (tet_amips(tid) >= kNeedleQuality) report_needle("COLLAPSE", tid, -1.);
        record_flatness("COLLAPSE", m_collapse_parent_flatness.local(), tid);
    }

    if (m_vertex_extra.at(v1_id).m_is_on_offset) ++iter_cnt_collapse_offset_removed;

    // The base ORs its own m_is_on_surface, which is the union of the two; these say which.
    // m_is_on_input is written by close_labels() below, from the label.
    // The offset half is NOT an OR: it is re-derived from the labels, for v2 and for the link of
    // the edge that just died. An OR can only ever add the flag, so a collapse that takes a
    // vertex off the offset surface used to leave it flagged for the rest of the run -- counted
    // by energy_criterion(), smoothed as a front vertex, and unable to satisfy a criterion that
    // measures its distance to a level set it is no longer on. That is what stalled the cube at
    // target_distance_rel 1e-3 with the front already placed. See refresh_offset_membership().
    refresh_offset_membership(v2_id);
    for (const size_t w : m_collapse_edge_link.local()) {
        if (w == v1_id || w == v2_id) continue;
        refresh_offset_membership(w);
    }
    m_vertex_extra[v2_id].m_is_on_region =
        m_vertex_extra.at(v1_id).m_is_on_region || m_vertex_extra.at(v2_id).m_is_on_region;

    // m_face_extra: the merged faces around the edge (collapse_before_vertex()), as the base writes
    // m_face_attribute just after this returns. A face it cannot find makes the base refuse.
    for (const auto& [fe, l] : m_face_extra_collapse.local()) {
        if (const auto found = try_tuple_from_face({{v2_id, l[0], l[1]}}); found.has_value()) {
            m_face_extra[std::get<1>(found.value())] = fe;
        }
    }
    // The edge label (collapse_before_vertex()), then the closure over v2's ring -- which is
    // where v2 gets v1's label when v1 carried more.
    const std::vector<size_t> ring2 = get_one_ring_tids_for_vertex(v2_id);
    edge_label_assign(ring2, m_edge_label_collapse.local());
    close_labels(ring2);

    // The base calls this only once a collapse has actually gone through.
    ++iter_cnt_collapse;
}

void TopoOffsetTetMesh::split_after_vertex(const size_t v_id, const bool is_edge_open_boundary)
{
    const auto& cache = m_opt_split_cache.local();
    // The base has already set m_is_on_surface, which is the union; this says which. The offset
    // half is never rewritten here -- split_after_cells() took it from the same cached edge
    // property this line uses, and that is the authority.
    m_vertex_extra[v_id].m_is_on_region = cache.is_edge_on_region;
    if (is_edge_open_boundary) {
        m_vertex_attribute[v_id].m_order = 2;
    }

    // Diagnostic, see the header. Every cell incident to the midpoint was created by this split,
    // so a MAX_ENERGY cell here is one this split manufactured; a split is never refused on
    // quality, so nothing upstream would have stopped it.
    const double parent_q = cache.parent_q_max;
    for (const size_t tid : get_one_ring_tids_for_vertex(v_id)) {
        const double q = tet_amips(tid);
        if (cell_quality(tid) >= MAX_ENERGY) ++m_deg_split_created;
        if (q >= kNeedleQuality) report_needle("SPLIT", tid, parent_q);
        record_flatness("SPLIT", cache.parent_flatness, tid);
    }

    // use_rest_pose: every cell at the midpoint was created by this split and the snapshot copy
    // gave each the parent's rest -- re-stamp, or a child measures itself against a tet twice its
    // size. Before split_edge_after()'s EXPERIMENTAL_split_energy_gate reads the children.
    for (const size_t tid : get_one_ring_tids_for_vertex(v_id)) {
        stamp_rest_cell(tid);
    }
}

bool TopoOffsetTetMesh::split_adjust_position(const size_t v_id, const std::vector<Tuple>&)
{
    // The new vertex's tracked-surface membership must be written before the shared split's own
    // containment check, which reads m_is_on_region for all three vertices of each new triangle;
    // an unrecognised triangle yields a null envelope and the check is silently skipped rather
    // than failed. split_adjust_position() is the last hook the base offers before that check,
    // which is the only reason this bookkeeping lives in a positioning hook. Safe against a
    // refused split: m_vertex_extra is in the base's vertex attribute group, so a rollback undoes
    // it, and the write is idempotent with split_after_vertex()'s own below.
    const auto& cache = m_opt_split_cache.local();
    m_vertex_extra[v_id].m_is_on_region = cache.is_edge_on_region;
    return true; // the position itself is the base's business, and it is happy with it
}

bool TopoOffsetTetMesh::smooth_before(const Tuple& t)
{
    const size_t vid = t.vid(*this);
    // The loop's front / background blocks (m_smooth_set): a vertex the block does not smooth is
    // left where it is, before any diagnostic or rounding.
    if (!smooth_set_admits(vid)) {
        ++m_smooth_set_skipped;
        return false;
    }
    ++m_smooth_trace.attempted;

    // Diagnostic, recorded for every visit; only visits whose ring already holds a needle are
    // counted, and smooth_after() reads this back.
    auto& pre = m_needle_pre.local();
    pre = {ring_max_quality(vid), m_vertex_attribute[vid].m_posf};
    if (pre.first >= kNeedleQuality) ++m_needle_smooth_offered;

    // The base's smooth_before minus its bounding-box refusal, which is why this does not call
    // it: the base freezes every vertex on the domain wall. Here the wall is a region boundary
    // held in ambient's tag envelope like any other, so its vertices are smoothed and the
    // containment check decides whether the move survives. As in 2D.
    //
    // Rounding still has to happen, and its failure still refuses the move.
    const bool rounded_now = round(t);
    if (!m_vertex_attribute[vid].m_is_rounded && !rounded_now) {
        ++m_smooth_trace.before_unrounded;
        return false;
    }

    return true;
}

polysolve::nonlinear::Solver& TopoOffsetTetMesh::smoothing_solver()
{
    // See the declaration. Created here with the engine's own parameters when the thread has
    // none yet -- exactly what the engine's smoother would create -- and the one criterion this
    // component adds is set on every visit, so it holds whichever path created the solver.
    auto& solver = m_solver.local();
    if (!solver) {
        solver = polysolve::nonlinear::Solver::create(
            optimization::basic_nonlinear_solver_params,
            optimization::basic_linear_solver_params,
            1,
            opt_logger());
    }
    // TetWild's rules (m_op_rules): the engine's own criterion, basic_nonlinear_solver_params'
    // rel_grad_norm_tol 0.
    solver->stop_criteria().relGradNorm = tetwild_rules() ? 0. : kSmoothRelGradNormTol;
    return *solver;
}

bool TopoOffsetTetMesh::smooth_after(const Tuple& t)
{
    smoothing_solver(); // the thread's solver carries this component's stopping rule, every path
    const size_t vid = t.vid(*this);
    const auto& ve = m_vertex_extra[vid];

    // Diagnostic, the other half of smooth_before()'s record.
    {
        const auto& pre = m_needle_pre.local();
        if (pre.first >= kNeedleQuality) {
            ++m_needle_smooth_reached;
            const double after = ring_max_quality(vid);
            const double moved = (m_vertex_attribute[vid].m_posf - pre.second).norm();
            if (after < kNeedleQuality) ++m_needle_smooth_fixed;
            if (moved < 1e-12) ++m_needle_smooth_stationary;
            if (m_needle_smooth_reports.fetch_add(1) < 8) {
                logger().info(
                    "[needle-smooth #{}] vid {} ring max {:.6g} -> {:.6g} ({:.3g}x) | moved "
                    "{:.6g} | input {} offset {} region {} held {} | pos ({:.17g}, {:.17g}, "
                    "{:.17g})",
                    m_needle_smooth_reports.load(),
                    vid,
                    pre.first,
                    after,
                    after / std::max(pre.first, 1e-300),
                    moved,
                    ve.m_is_on_input,
                    ve.m_is_on_offset,
                    ve.m_is_on_region,
                    vertex_is_held(vid),
                    m_vertex_attribute[vid].m_posf[0],
                    m_vertex_attribute[vid].m_posf[1],
                    m_vertex_attribute[vid].m_posf[2]);
            }
        }
    }
    if (ve.m_is_on_region) {
        ++m_smooth_trace.region_attempted;
    }
    if (ve.m_is_on_offset) {
        ++m_smooth_trace.offset_attempted;
    } else {
        ++m_smooth_trace.interior_attempted;
    }

    // The repulsion passes before the march (repulsion_smoothing()); false at every other time.
    if (is_repulsion_vertex(vid)) return smooth_repulsion_vertex(t);

    // TetWild's rules (m_op_rules): TetWild's own smoother, every vertex -- the raw sum of AMIPS
    // over the ring (smoothing_extra_energy() gives none), its quality veto on the ring's max, a
    // held vertex's projected step on the max (smoothing_ring_energy()); the front held in its
    // envelope by smoothing_energy_envelope() like any held vertex.
    if (tetwild_rules()) return TetOptimizerMesh::smooth_after(t);

    // A front vertex the loop places goes through the shared smoother -- same solver, line
    // search and accept tests as every other vertex -- with the offset's options: its objective
    // is the ring barrier plus the offset error over its front faces, NO AMIPS
    // (smoothing_extra_energy() -> front_objective()), and the veto is on that objective (see
    // smooth_front_vertex()). Every other vertex -- held front vertices among them -- is
    // TetWild's smoother (smooth_nonfront_vertex()). A TetWild pass returned above.
    if (vertex_front_placed(vid)) {
        const bool ok = smooth_front_vertex(t);
        if (ok) ++m_smooth_trace.offset_accepted;
        return ok;
    }
    return smooth_nonfront_vertex(t);
}

double TopoOffsetTetMesh::front_vertex_normal_gradient(const size_t vid) const
{
    // ||grad F|| at the vertex's current position, F the objective smooth_front_vertex()
    // minimises, taken along the move direction where there is one.
    const Vector3d x = m_vertex_attribute[vid].m_posf;
    Eigen::VectorXd xv = x, g(3);
    front_objective(vid)->gradient(xv, g);
    if (!g.allFinite()) return std::numeric_limits<double>::infinity();
    const Vector3d n = front_vertex_move_direction(vid);
    if (n.squaredNorm() > 0.) return std::abs(n.dot(Vector3d(g)));
    return g.norm();
}

void TopoOffsetTetMesh::audit_surface_containment(const std::string& when) const
{
    // Every tracked face against the envelope surface_envelope_for_face() holds it to -- the
    // dispatch the operations and the sanity check use, so this cannot disagree with them --
    // and, for the faces inside, how much room is left as a fraction of eps: `is_outside` is a
    // yes/no, and a face resting on the skin of its envelope is the one the next operation pushes
    // out. Both envelopes are real Envelopes, so squared_distance() is safe on them.
    struct Face
    {
        std::array<size_t, 3> v{{0, 0, 0}};
        bool offset = false;
        double d = 0.; ///< outside: the worst sample distance; inside: that over eps
    };
    std::vector<Face> bad, snug;
    size_t n_tracked = 0, n_held = 0;
    double worst_frac = 0.;
    std::array<size_t, 5> band{{0, 0, 0, 0, 0}}; // <0.5, <0.9, <0.99, <1, >=1 of eps
    // The farthest a lattice of (k+1)(k+2)/2 points on the triangle is from the envelope.
    const auto farthest = [](const Envelope& env, const std::array<Vector3d, 3>& p, int k) {
        double d = 0.;
        for (int i = 0; i <= k; ++i) {
            for (int j = 0; j <= k - i; ++j) {
                const Vector3d q =
                    p[0] + double(i) / k * (p[1] - p[0]) + double(j) / k * (p[2] - p[0]);
                d = std::max(d, std::sqrt(std::max(env.squared_distance(q), 0.)));
            }
        }
        return d;
    };
    for (const Tuple& f : get_faces()) {
        if (!m_face_attribute[f.fid(*this)].m_is_surface_fs) continue;
        ++n_tracked;
        const std::array<size_t, 3> vids = get_face_vids(f);
        const std::shared_ptr<Envelope> env = surface_envelope_for_face(vids);
        if (!env) continue;
        ++n_held;
        const std::array<Vector3d, 3> p{
            {m_vertex_attribute[vids[0]].m_posf,
             m_vertex_attribute[vids[1]].m_posf,
             m_vertex_attribute[vids[2]].m_posf}};
        const bool offset = env == m_offset_envelope;
        if (env->is_outside(p)) {
            bad.push_back({vids, offset, farthest(*env, p, 6)});
            continue;
        }
        if (!(env->eps2 > 0.)) continue;
        const double frac = farthest(*env, p, 2) / std::sqrt(env->eps2);
        worst_frac = std::max(worst_frac, frac);
        ++band[frac < 0.5 ? 0 : frac < 0.9 ? 1 : frac < 0.99 ? 2 : frac < 1. ? 3 : 4];
        if (frac >= 0.9) snug.push_back({vids, offset, frac});
    }
    const auto by_d = [](const Face& x, const Face& y) { return x.d > y.d; };
    const auto show = [&](std::vector<Face>& faces, const char* what, const char* unit) {
        std::sort(faces.begin(), faces.end(), by_d);
        for (size_t i = 0; i < std::min<size_t>(faces.size(), 4); ++i) {
            const Face& r = faces[i];
            const Vector3d& pa = m_vertex_attribute[r.v[0]].m_posf;
            logger().info(
                "\t  [{} {}] face [{}, {}, {}] at ({:.6g}, {:.6g}, {:.6g}): {:.6g}{}",
                r.offset ? "offset" : "held",
                what,
                r.v[0],
                r.v[1],
                r.v[2],
                pa.x(),
                pa.y(),
                pa.z(),
                r.d,
                unit);
        }
    };
    const size_t n_inside = n_held - bad.size();
    if (n_inside > 0) {
        const auto pct = [&](size_t n) { return 100. * double(n) / double(n_inside); };
        logger().info(
            "\t[containment {} margin] {} inside faces against their envelope eps: {} under 0.5 "
            "({:.1f}%), {} in 0.5-0.9 ({:.1f}%), {} in 0.9-0.99 ({:.1f}%), {} in 0.99-1.0 "
            "({:.1f}%), {} at or over 1.0 ({:.1f}%) | worst {:.4f} of eps",
            when,
            n_inside,
            band[0],
            pct(band[0]),
            band[1],
            pct(band[1]),
            band[2],
            pct(band[2]),
            band[3],
            pct(band[3]),
            band[4],
            pct(band[4]),
            worst_frac);
        show(snug, "snug", " of eps");
    }
    logger().log(
        bad.empty() ? spdlog::level::info : spdlog::level::warn,
        "\t[containment {}] {} of {} held faces OUTSIDE their envelope ({} tracked faces, the "
        "rest held by nothing)",
        when,
        bad.size(),
        n_held,
        n_tracked);
    show(bad, "OUT BY", "");
}

std::vector<double> TopoOffsetTetMesh::front_ring_measures() const
{
    // energy_criterion()'s ring measure, face for face: one face_offset_term() per offset face
    // with three front corners, added to each corner; an unmeasurable face leaves its corners
    // without a ring measure.
    const auto front = [&](const size_t vid) {
        return m_vertex_extra[vid].m_is_on_offset && m_vertex_attribute[vid].m_is_rounded;
    };
    std::vector<double> sum(vert_capacity(), 0.);
    std::vector<size_t> n(vert_capacity(), 0);
    std::vector<char> bad(vert_capacity(), 0);
    for (const auto& f : offset_surface_faces()) {
        if (!front(f[0]) || !front(f[1]) || !front(f[2])) continue;
        const double term = face_offset_term(f[0], f[1], f[2]);
        for (const size_t u : f) {
            if (term < 0.) {
                bad[u] = 1;
            } else {
                sum[u] += term;
                ++n[u];
            }
        }
    }
    std::vector<double> r(vert_capacity(), std::numeric_limits<double>::quiet_NaN());
    for (size_t v = 0; v < vert_capacity(); ++v) {
        if (!bad[v] && n[v] > 0) r[v] = std::sqrt(sum[v] / double(n[v]));
    }
    return r;
}

double TopoOffsetTetMesh::ring_measure_at(const size_t vid) const
{
    const auto front = [&](const size_t u) {
        return m_vertex_extra[u].m_is_on_offset && m_vertex_attribute[u].m_is_rounded;
    };
    double sum = 0.;
    size_t n = 0;
    for (const Tuple& ft : offset_surface_faces_live_at(vid)) {
        const auto f = get_face_vids(ft);
        if (!front(f[0]) || !front(f[1]) || !front(f[2])) continue;
        const double term = face_offset_term(f[0], f[1], f[2]);
        if (term < 0.) return std::numeric_limits<double>::quiet_NaN();
        sum += term;
        ++n;
    }
    return n > 0 ? std::sqrt(sum / double(n)) : std::numeric_limits<double>::quiet_NaN();
}

void TopoOffsetTetMesh::crossing_snapshot(
    const std::string& pass,
    const bool compare,
    const bool match_positions)
{
    std::vector<double> r = front_ring_measures();
    std::vector<Vector3d> pos(vert_capacity());
    for (size_t v = 0; v < vert_capacity(); ++v) pos[v] = m_vertex_attribute[v].m_posf;
    if (compare && m_cross_valid) {
        size_t up = 0, down = 0, new_over = 0, gone_over = 0, over = 0, n = 0;
        const size_t n_old = m_cross_ring.size();
        for (size_t v = 0; v < std::max(n_old, r.size()); ++v) {
            const double now_r = v < r.size() ? r[v] : std::numeric_limits<double>::quiet_NaN();
            const double old_r =
                v < n_old ? m_cross_ring[v] : std::numeric_limits<double>::quiet_NaN();
            const bool now = std::isfinite(now_r);
            bool had = std::isfinite(old_r);
            const bool same =
                !(had && now && match_positions && v < m_cross_pos.size() && v < pos.size() &&
                  pos[v] != m_cross_pos[v]);
            if (now) {
                ++n;
                if (now_r > 1.) ++over;
            }
            if (had && now && same) {
                if (old_r <= 1. && now_r > 1.) ++up;
                if (old_r > 1. && now_r <= 1.) ++down;
                continue;
            }
            if (now && now_r > 1.) ++new_over;
            if (had && old_r > 1.) ++gone_over;
        }
        std::string extra;
        if (pass == "smooth") {
            extra = fmt::format(
                " | smoothing moves that crossed: own {}, neighbour {}",
                m_cross_own.exchange(0),
                m_cross_neighbour.exchange(0));
        }
        logger().info(
            "\t[crossings] turn {} {}: up {} (<= 1 -> > 1), down {}, new over {}, gone over {} | "
            "over now {} of {}{}",
            m_round,
            pass,
            up,
            down,
            new_over,
            gone_over,
            over,
            n,
            extra);
    }
    m_cross_ring = std::move(r);
    m_cross_pos = std::move(pos);
    m_cross_valid = true;
}

void TopoOffsetTetMesh::update_attributes()
{
    TetOptimizerMesh::update_attributes();
    if (!m_offset_params.debug_crossings || m_round <= 0) return;
    const std::string& p = m_debug_pass_name;
    if (p == "split" || p == "collapse" || p == "swap") {
        crossing_snapshot(p, true, true);
    } else if (!m_cross_valid) {
        crossing_snapshot("start", false, false);
    }
}

void TopoOffsetTetMesh::log_smoothing_pass_accounting()
{
    // Per pass, after the base's own "newton, smooth_after" line (the background).
    logger().info("\tnewton, front: {}", m_newton_front.to_string());
    logger().info(
        "\tfront veto: fired {} of {} front moves that reached it (ring sum of tet_energy rose)",
        m_front_veto_fired.exchange(0),
        m_front_veto_asked.exchange(0));
    if (damping_active()) {
        const size_t bounded = m_damping_bounded.exchange(0);
        const size_t moves = m_damping_moves.exchange(0);
        logger().info(
            "\t[damping] {} of {} accepted smoothing moves ended on the bound ({} x the shortest "
            "edge at the vertex)",
            bounded,
            moves,
            m_offset_params.damping_factor);
        logger().info(
            "\t[damping front] {} of {} accepted front moves ended on the bound; of those: {} "
            "mostly along grad d, {} with the along-gradient part toward the target, {} with the "
            "target farther than the bound, {} with no gradient",
            m_damping_front_bounded.exchange(0),
            m_damping_front_moves.exchange(0),
            m_damping_front_along.exchange(0),
            m_damping_front_toward.exchange(0),
            m_damping_front_far.exchange(0),
            m_damping_front_nograd.exchange(0));
    }
    {
        // Diagnostic: where the front solves stop (see m_front_grad_abs). Bins are log10 of the
        // value; the first bin is "<= 0 or below the range".
        const auto line =
            [&](const char* what, std::array<std::atomic<size_t>, kGradBins>& bins, int lo) {
                std::string out;
                size_t n = 0;
                for (int i = 0; i < kGradBins; ++i) {
                    const size_t k = bins[size_t(i)].exchange(0);
                    n += k;
                    if (k == 0) continue;
                    if (i == 0)
                        out += fmt::format(" <1e{}:{}", lo, k);
                    else
                        out += fmt::format(" 1e{}:{}", lo + i - 1, k);
                }
                if (n > 0) logger().info("\tfront solve final {} (log10 bins:count):{}", what, out);
            };
        line("|grad|", m_front_grad_abs, -14);
        line("|grad|/|grad_0|", m_front_grad_rel, -14);
    }
    m_newton_front.reset();
    if (m_offset_params.debug_crossings && m_round > 0) crossing_snapshot("smooth", true, false);
}

void TopoOffsetTetMesh::log_smooth_trace() const
{
    const auto& s = m_smooth_trace;
    logger().info(
        "\tsmooth trace: attempted {} | before: bbox {}, unrounded {} | reached the smoother: {} "
        "on the offset surface, {} elsewhere ({} of them on another region boundary) | ({})",
        s.attempted.load(),
        s.before_bbox.load(),
        s.before_unrounded.load(),
        s.offset_attempted.load(),
        s.interior_attempted.load(),
        s.region_attempted.load(),
        m_smooth_rejects.to_string());
    logger().info(
        "\toffset term: {} attempted -> {} accepted",
        s.offset_attempted.load(),
        s.offset_accepted.load());
}

void TopoOffsetTetMesh::log_refine_block_census(const std::string& when, const double filter_energy)
    const
{
    enum Verdict { kShort = 0, kValence, kContain, kFree, kNVerdict };
    static const char* kName[kNVerdict] = {"short", "valence", "contain", "free"};

    const double l = std::max(m_params.l, 1e-16);
    const size_t val_thresh = m_params.split_high_valence_threshold > 0
                                  ? size_t(m_params.split_high_valence_threshold)
                                  : std::numeric_limits<size_t>::max();
    static constexpr int E[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};

    std::array<size_t, kNVerdict> edge_hist{};
    std::array<size_t, kNVerdict> cell_first{};
    size_t n_bad = 0, n_inverted = 0, n_any_free = 0;
    size_t n_contain_offset = 0, n_contain_region = 0;

    struct Ex
    {
        double q = -1.;
        size_t tid = 0;
        Vector3d c = Vector3d::Zero();
        double dist = -1., phi_over_c = -1., sizing = 0.;
        std::array<double, 6> len{}, gate{};
        std::array<int, 6> verd{{-1, -1, -1, -1, -1, -1}};
        bool inverted = false;
        int label = -1;
    };
    std::array<Ex, kNVerdict> ex;

    const double c_level = m_offset_potential ? m_offset_potential->target_level() : 0.;

    for (size_t tid = 0; tid < tet_capacity(); ++tid) {
        if (!tuple_from_tet(tid).is_valid(*this)) continue;
        const double q = tet_amips(tid);
        if (!(q >= filter_energy)) continue;
        ++n_bad;

        const auto vs = oriented_tet_vids(tid);
        const bool inv = is_inverted(tuple_from_tet(tid));
        if (inv) ++n_inverted;

        Ex cand;
        cand.q = q;
        cand.tid = tid;
        cand.inverted = inv;
        cand.label = m_tet_attribute[tid].label;
        cand.sizing = std::numeric_limits<double>::max();
        for (const size_t v : vs) {
            cand.c += m_vertex_attribute[v].m_posf / 4.;
            cand.sizing = std::min(cand.sizing, m_vertex_attribute[v].m_sizing_scalar);
        }
        // The line's "dist to complex": the distance to the input complex itself, 0 inside its
        // solid (input_complex_distance()).
        if (m_input_complex_envelope) cand.dist = input_complex_distance(cand.c);
        if (m_offset_potential && c_level > 0.) {
            cand.phi_over_c = m_offset_potential->value(cand.c) / c_level;
        }

        int best = kShort;
        for (int k = 0; k < 6; ++k) {
            const size_t a = vs[size_t(E[k][0])], b = vs[size_t(E[k][1])];
            const Vector3d& pa = m_vertex_attribute[a].m_posf;
            const Vector3d& pb = m_vertex_attribute[b].m_posf;
            const double len2 = (pb - pa).squaredNorm();
            const double sr = 0.5 * (m_vertex_attribute[a].m_sizing_scalar +
                                     m_vertex_attribute[b].m_sizing_scalar);
            const double gate2 = m_params.splitting_l2 * sr * sr;
            cand.len[size_t(k)] = std::sqrt(len2);
            cand.gate[size_t(k)] = std::sqrt(std::max(gate2, 0.));

            Verdict v;
            if (len2 < gate2) {
                v = kShort;
            } else {
                // The link of the edge: the two other vertices of this tet stand in for it.
                bool valence_blocked = false;
                for (const size_t w : vs) {
                    if (w != a && w != b && vertex_valence(w) > val_thresh) valence_blocked = true;
                }
                if (valence_blocked) {
                    v = kValence;
                } else {
                    v = kFree;
                    // The child triangles' envelope is the parent's, so one dispatch serves both
                    // halves.
                    for (const size_t w : vs) {
                        if (w == a || w == b) continue;
                        const auto found = try_tuple_from_face({{a, b, w}});
                        if (!found) continue;
                        const size_t fid = std::get<1>(*found);
                        if (!m_face_attribute[fid].m_is_surface_fs) continue;
                        const std::shared_ptr<Envelope> env =
                            surface_envelope_for_face({{a, b, w}});
                        if (!env) continue;
                        const Vector3d mid = 0.5 * (pa + pb);
                        const Vector3d& pw = m_vertex_attribute[w].m_posf;
                        if (env->is_outside(std::array<Vector3d, 3>{{pa, mid, pw}}) ||
                            env->is_outside(std::array<Vector3d, 3>{{mid, pb, pw}})) {
                            v = kContain;
                            if (face_is_offset(fid))
                                ++n_contain_offset;
                            else
                                ++n_contain_region;
                            break;
                        }
                    }
                }
            }
            cand.verd[size_t(k)] = int(v);
            if (v == kFree)
                best = kFree;
            else if (best != kFree && v > best)
                best = v;
            ++edge_hist[size_t(v)];
        }
        ++cell_first[size_t(best)];
        if (best == kFree) ++n_any_free;
        if (cand.q > ex[size_t(best)].q) ex[size_t(best)] = cand;
    }

    if (n_bad == 0) {
        logger().info("\t[refine-block {}] no element at or above {:.4g}", when, filter_energy);
        return;
    }

    std::string cells, edges;
    for (int v = 0; v < kNVerdict; ++v) {
        if (cell_first[size_t(v)])
            cells +=
                fmt::format("{}{} {}", cells.empty() ? "" : ", ", cell_first[size_t(v)], kName[v]);
        if (edge_hist[size_t(v)])
            edges +=
                fmt::format("{}{} {}", edges.empty() ? "" : ", ", edge_hist[size_t(v)], kName[v]);
    }
    logger().info(
        "\t[refine-block {}] {} elements >= {:.4g} ({} exactly inverted) | best edge per element: "
        "{} | all {} edges: {} | containment refusals by class: {} offset, {} region | "
        "target l {:.6g}, split needs length >= {:.4g} x mean sizing",
        when,
        n_bad,
        filter_energy,
        n_inverted,
        cells,
        6 * n_bad,
        edges,
        n_contain_offset,
        n_contain_region,
        l,
        std::sqrt(std::max(m_params.splitting_l2, 0.)));

    logger().info(
        "\t[refine-block {}] {} of {} bad elements have at least one splittable edge -- for those "
        "the gates are NOT the obstacle",
        when,
        n_any_free,
        n_bad);

    for (int v = 0; v < kNVerdict; ++v) {
        const Ex& e = ex[size_t(v)];
        if (e.q < 0.) continue;
        std::string per_edge;
        for (int k = 0; k < 6; ++k) {
            per_edge += fmt::format(
                "{}{:.4g}/{:.4g} [{}]",
                k ? ", " : "",
                e.len[size_t(k)],
                e.gate[size_t(k)],
                kName[e.verd[size_t(k)]]);
        }
        logger().info(
            "\t  worst [{}]: t{} q {:.4g}{} label {} at ({:.6g}, {:.6g}, {:.6g}) | dist to complex "
            "{:.6g} = {:.4g}x delta | Phi/c {:.6g} | min sizing {:.6g} = {:.4g}x l | edges "
            "len/gate {}",
            kName[v],
            e.tid,
            e.q,
            e.inverted ? " INVERTED" : "",
            e.label,
            e.c.x(),
            e.c.y(),
            e.c.z(),
            e.dist,
            e.dist / std::max(m_offset_params.target_distance, 1e-16),
            e.phi_over_c,
            e.sizing,
            e.sizing / l,
            per_edge);
    }
}

void TopoOffsetTetMesh::log_stuck_refine_census(const double max_metric, const double filter_energy)
{
    ++m_stuck_calls;

    const double l = std::max(m_params.l, 1e-16);
    const double cell = l / 10.;

    size_t n_cells = 0, n_over_filter = 0, n_max = 0;
    size_t n_exact_inverted = 0, n_float_only = 0, n_unrounded = 0;
    std::array<size_t, 3> by_class{{0, 0, 0}}; // ambient / input complex / band
    size_t n_below_gate = 0, n_at_floor = 0;
    std::vector<double> volumes, shortest, aspects;
    std::vector<size_t> max_tids;
    std::set<std::tuple<long, long, long>> cells;
    static constexpr int E[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};

    for (size_t tid = 0; tid < tet_capacity(); ++tid) {
        const Tuple tt = tuple_from_tet(tid);
        if (!tt.is_valid(*this)) continue;
        ++n_cells;
        const double q = tet_amips(tid);
        if (q >= filter_energy) ++n_over_filter;
        if (cell_quality(tid) < MAX_ENERGY) continue;
        ++n_max;
        max_tids.push_back(tid);

        const auto vs = oriented_tet_vids(tid);
        const bool exact_bad = is_inverted(tt);
        const bool float_bad = is_inverted_f(tt);
        if (exact_bad)
            ++n_exact_inverted;
        else if (float_bad)
            ++n_float_only;
        bool any_unrounded = false;
        for (const size_t v : vs) any_unrounded |= !m_vertex_attribute[v].m_is_rounded;
        if (any_unrounded) ++n_unrounded;

        const int lab = m_tet_attribute[tid].label;
        by_class[lab >= 0 && lab <= 2 ? size_t(lab) : size_t(0)]++;

        std::array<Vector3d, 4> p;
        for (int i = 0; i < 4; ++i) p[size_t(i)] = m_vertex_attribute[vs[size_t(i)]].m_posf;
        volumes.push_back(std::abs((p[1] - p[0]).cross(p[2] - p[0]).dot(p[3] - p[0]) / 6.));
        double lo = std::numeric_limits<double>::max(), hi = 0.;
        for (const auto& e : E) {
            const double len = (p[size_t(e[0])] - p[size_t(e[1])]).norm();
            lo = std::min(lo, len);
            hi = std::max(hi, len);
        }
        shortest.push_back(lo);
        aspects.push_back(lo > 0. ? hi / lo : std::numeric_limits<double>::infinity());

        double sbar_hi = 0.;
        for (const size_t v : vs) sbar_hi += m_vertex_attribute[v].m_sizing_scalar;
        sbar_hi /= 4.;
        if (hi <= l * sbar_hi * 4. / 3.) ++n_below_gate;
        bool at_floor = true;
        for (const size_t v : vs)
            at_floor &= m_vertex_attribute[v].m_sizing_scalar <=
                        m_params.stuck_refine_min_scalar * (1. + 1e-9);
        if (at_floor) ++n_at_floor;

        const Vector3d ctr = (p[0] + p[1] + p[2] + p[3]) / 4.;
        cells.insert(
            {long(std::floor(ctr[0] / cell)),
             long(std::floor(ctr[1] / cell)),
             long(std::floor(ctr[2] / cell))});
    }

    if (n_max == 0) {
        logger().info(
            "[stuck-census #{}] {} tets, {} at or over filter {:.4}, NONE at MAX_ENERGY -- the "
            "stall is merely-bad elements, not degenerate ones (max metric {:.4})",
            m_stuck_calls,
            n_cells,
            n_over_filter,
            filter_energy,
            max_metric);
        m_stuck_prev_cells.clear();
        return;
    }

    auto pct = [&](size_t k) { return 100. * double(k) / double(n_max); };
    auto med = [](std::vector<double>& v) {
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    };

    // Connected clusters among the MAX_ENERGY tets, by shared face.
    std::unordered_map<size_t, size_t> idx_of;
    for (size_t i = 0; i < max_tids.size(); ++i) idx_of[max_tids[i]] = i;
    std::vector<size_t> parent(max_tids.size());
    std::iota(parent.begin(), parent.end(), size_t(0));
    std::function<size_t(size_t)> find = [&](size_t x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    };
    std::map<std::array<size_t, 3>, size_t> face_owner;
    for (size_t i = 0; i < max_tids.size(); ++i) {
        const auto vs = oriented_tet_vids(max_tids[i]);
        for (int skip = 0; skip < 4; ++skip) {
            std::array<size_t, 3> f = face_corners_from(vs, skip);
            std::sort(f.begin(), f.end());
            auto it = face_owner.find(f);
            if (it == face_owner.end()) {
                face_owner[f] = i;
            } else {
                const size_t ra = find(it->second), rb = find(i);
                if (ra != rb) parent[ra] = rb;
            }
        }
    }
    std::unordered_map<size_t, size_t> comp_size;
    for (size_t i = 0; i < max_tids.size(); ++i) comp_size[find(i)]++;
    size_t largest = 0;
    for (const auto& [root, sz] : comp_size) largest = std::max(largest, sz);

    size_t overlap = 0;
    for (const auto& c : cells)
        if (m_stuck_prev_cells.count(c)) ++overlap;
    const double overlap_pct =
        m_stuck_prev_cells.empty() ? 0. : 100. * double(overlap) / double(cells.size());

    logger().info(
        "[stuck-census #{}] {} tets | {} at/over filter {:.4} | {} at MAX_ENERGY ({:.2f}%)",
        m_stuck_calls,
        n_cells,
        n_over_filter,
        filter_energy,
        n_max,
        100. * double(n_max) / double(std::max<size_t>(n_cells, 1)));
    logger().info(
        "[stuck-census #{}]   cause: exactly inverted {} ({:.1f}%), float-degenerate only {} "
        "({:.1f}%), neither {} | with an unrounded vertex {} ({:.1f}%)",
        m_stuck_calls,
        n_exact_inverted,
        pct(n_exact_inverted),
        n_float_only,
        pct(n_float_only),
        n_max - n_exact_inverted - n_float_only,
        n_unrounded,
        pct(n_unrounded));
    logger().info(
        "[stuck-census #{}]   class: ambient {}, input complex {}, band {} | clusters {}, "
        "largest {} tets | grid cells {} ({:.1f}% shared with the previous census)",
        m_stuck_calls,
        by_class[0],
        by_class[1],
        by_class[2],
        comp_size.size(),
        largest,
        cells.size(),
        overlap_pct);
    logger().info(
        "[stuck-census #{}]   geometry: volume med {:.6g} (min {:.6g}), shortest edge med {:.6g}, "
        "aspect med {:.6g} | target l {:.6g}",
        m_stuck_calls,
        med(volumes),
        volumes.front(),
        med(shortest),
        med(aspects),
        l);
    logger().info(
        "[stuck-census #{}]   refinement applicable? {} of {} are ALREADY below the split gate "
        "({:.1f}%), {} are at the sizing floor {:.6g} ({:.1f}%)",
        m_stuck_calls,
        n_below_gate,
        n_max,
        pct(n_below_gate),
        n_at_floor,
        m_params.stuck_refine_min_scalar,
        pct(n_at_floor));

    const size_t split_created = m_deg_split_created.load();
    logger().info(
        "[stuck-census #{}]   created since the last census: by SPLIT {} needle tets (a split "
        "is never refused on quality)",
        m_stuck_calls,
        split_created - m_deg_prev_split_created);
    m_deg_prev_split_created = split_created;

    m_stuck_prev_cells = std::move(cells);
}

void TopoOffsetTetMesh::report_needle(const char* op, const size_t tid, const double parent_q) const
{
    if (m_needle_reports.fetch_add(1) >= kNeedleReports) return;

    const auto vs = oriented_tet_vids(tid);
    std::array<Vector3d, 4> p;
    for (int i = 0; i < 4; ++i) p[size_t(i)] = m_vertex_attribute[vs[size_t(i)]].m_posf;
    const double vol = (p[1] - p[0]).cross(p[2] - p[0]).dot(p[3] - p[0]) / 6.;
    static constexpr int E[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
    std::string edges;
    for (const auto& e : E) {
        edges += fmt::format(
            "{}{:.6g}",
            edges.empty() ? "" : " ",
            (p[size_t(e[0])] - p[size_t(e[1])]).norm());
    }

    std::string per_vertex;
    for (int k = 0; k < 4; ++k) {
        const size_t v = vs[size_t(k)];
        const auto& x = m_vertex_extra[v];
        per_vertex += fmt::format(
            "\n\t    v{} id {} ({:.17g}, {:.17g}, {:.17g}) input {} offset {} region {} held {} "
            "epoch {} rounded {} sizing {:.6g}",
            k,
            v,
            p[size_t(k)][0],
            p[size_t(k)][1],
            p[size_t(k)][2],
            x.m_is_on_input,
            x.m_is_on_offset,
            x.m_is_on_region,
            vertex_is_held(v),
            x.m_born_epoch,
            m_vertex_attribute[v].m_is_rounded,
            m_vertex_attribute[v].m_sizing_scalar);
    }
    logger().info(
        "[needle #{}] created at {} | tid {} label {} | volume {:.6g} | edges {} "
        "| parent AMIPS {} | is_inverted {} is_inverted_f {} | epoch {}{}",
        m_needle_reports.load(),
        op,
        tid,
        m_tet_attribute[tid].label,
        vol,
        edges,
        parent_q < 0. ? std::string("n/a") : fmt::format("{:.6g}", parent_q),
        is_inverted(tuple_from_tet(tid)),
        is_inverted_f(tuple_from_tet(tid)),
        m_op_epoch,
        per_vertex);
}

void TopoOffsetTetMesh::needle_scan(const char* when) const
{
    size_t n = 0;
    double worst_vol = std::numeric_limits<double>::max();
    size_t worst_tid = 0;
    std::array<size_t, 3> by_class{{0, 0, 0}};
    for (size_t tid = 0; tid < tet_capacity(); ++tid) {
        if (!tuple_from_tet(tid).is_valid(*this)) continue;
        if (tet_amips(tid) < kNeedleQuality) continue;
        ++n;
        const int lab = m_tet_attribute[tid].label;
        by_class[lab >= 0 && lab <= 2 ? size_t(lab) : size_t(0)]++;
        const auto vs = oriented_tet_vids(tid);
        const Vector3d& a = m_vertex_attribute[vs[0]].m_posf;
        const Vector3d& b = m_vertex_attribute[vs[1]].m_posf;
        const Vector3d& c = m_vertex_attribute[vs[2]].m_posf;
        const Vector3d& d = m_vertex_attribute[vs[3]].m_posf;
        const double vol = std::abs((b - a).cross(c - a).dot(d - a)) / 6.;
        if (vol < worst_vol) {
            worst_vol = vol;
            worst_tid = tid;
        }
    }
    if (n == 0) {
        logger().info("[needle-scan] {}: NONE", when);
        return;
    }
    logger().warn(
        "[needle-scan] {}: {} tets over AMIPS {:g} (ambient {}, input complex {}, band {}), "
        "smallest volume {:.6g} at tid {}",
        when,
        n,
        kNeedleQuality,
        by_class[0],
        by_class[1],
        by_class[2],
        worst_vol,
        worst_tid);
    needle_forensics();
    logger().info(
        "[needle-smooth] cumulative: {} visits with a needle in the ring | {} produced a "
        "candidate | {} actually repaired it | {} did not move the vertex at all",
        m_needle_smooth_offered.load(),
        m_needle_smooth_reached.load(),
        m_needle_smooth_fixed.load(),
        m_needle_smooth_stationary.load());
    report_needle("scan", worst_tid, -1.);
}

double TopoOffsetTetMesh::ring_max_quality(const size_t vid) const
{
    double m = -1.;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        m = std::max(m, tet_amips(tid));
    }
    return m;
}

double TopoOffsetTetMesh::tet_flatness(const size_t tid) const
{
    const auto vs = oriented_tet_vids(tid);
    const Vector3d& a = m_vertex_attribute[vs[0]].m_posf;
    const Vector3d& b = m_vertex_attribute[vs[1]].m_posf;
    const Vector3d& c = m_vertex_attribute[vs[2]].m_posf;
    const Vector3d& d = m_vertex_attribute[vs[3]].m_posf;
    const double six_vol = std::abs((b - a).cross(c - a).dot(d - a));
    const double lmax = std::max(
        {(b - a).norm(),
         (c - a).norm(),
         (d - a).norm(),
         (c - b).norm(),
         (d - b).norm(),
         (d - c).norm()});
    return lmax > 0. ? six_vol / (lmax * lmax * lmax) : 0.;
}

void TopoOffsetTetMesh::record_flatness(
    const char* op,
    const double parent_flat,
    const size_t child_tid) const
{
    const double child = tet_flatness(child_tid);
    if (child >= kFlatThreshold) return;
    const bool from_healthy = parent_flat >= kFlatThreshold;
    if (from_healthy) {
        if (op[0] == 'S')
            ++m_flat_created_split;
        else
            ++m_flat_created_collapse;
    } else {
        ++m_flat_worsened_split;
    }
    if (from_healthy && m_flat_genesis_reports.fetch_add(1) < 10) {
        const auto vs = oriented_tet_vids(child_tid);
        std::string vtx;
        for (int k = 0; k < 4; ++k) {
            const auto& x = m_vertex_extra[vs[size_t(k)]];
            const Vector3d& p = m_vertex_attribute[vs[size_t(k)]].m_posf;
            vtx += fmt::format(
                "\n\t    v{} id {} ({:.17g}, {:.17g}, {:.17g}) epoch {} input {} region {} held "
                "{}",
                k,
                vs[size_t(k)],
                p[0],
                p[1],
                p[2],
                x.m_born_epoch,
                x.m_is_on_input,
                x.m_is_on_region,
                vertex_is_held(vs[size_t(k)]));
        }
        logger().info(
            "[genesis #{}] {} turned a HEALTHY tet into a flat one: flatness {:.6g} -> {:.6g} "
            "(threshold {:g}) | tid {} label {} | AMIPS {:.6g}{}",
            m_flat_genesis_reports.load(),
            op,
            parent_flat,
            child,
            kFlatThreshold,
            child_tid,
            m_tet_attribute[child_tid].label,
            tet_amips(child_tid),
            vtx);
    }
}

void TopoOffsetTetMesh::needle_forensics() const
{
    const double l = std::max(m_params.l, 1e-16);
    const double coll_c = std::sqrt(std::max(m_params.collapsing_l2, 0.)); // = 4/5 l
    const double split_c = std::sqrt(std::max(m_params.splitting_l2, 0.)); // = 4/3 l
    static constexpr int E[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};

    std::vector<std::pair<double, size_t>> flat;
    for (size_t tid = 0; tid < tet_capacity(); ++tid) {
        if (!tuple_from_tet(tid).is_valid(*this)) continue;
        const double f = tet_flatness(tid);
        if (f < kFlatThreshold) flat.emplace_back(f, tid);
    }
    std::sort(flat.begin(), flat.end());
    if (flat.empty()) {
        logger().info("[forensics] 0 tets flatter than {:g}", kFlatThreshold);
    }
    if (!flat.empty())
        logger().warn(
            "[forensics] {} tets flatter than {:g} | gates: collapse 4/5*l = {:.6g}, split 4/3*l "
            "= {:.6g}, both scaled by the edge's mean sizing scalar",
            flat.size(),
            kFlatThreshold,
            coll_c,
            split_c);

    const size_t show = std::min<size_t>(flat.size(), 4);
    for (size_t i = 0; i < show; ++i) {
        const size_t tid = flat[i].second;
        const auto vs = oriented_tet_vids(tid);
        logger().warn(
            "[forensics] tet {} flatness {:.4g} AMIPS {:.6g} label {}",
            tid,
            flat[i].first,
            tet_amips(tid),
            m_tet_attribute[tid].label);
        for (const auto& e : E) {
            const size_t u = vs[size_t(e[0])], w = vs[size_t(e[1])];
            const double len = (m_vertex_attribute[u].m_posf - m_vertex_attribute[w].m_posf).norm();
            const double sbar =
                (m_vertex_attribute[u].m_sizing_scalar + m_vertex_attribute[w].m_sizing_scalar) /
                2.;
            const Tuple et = tuple_from_edge({{u, w}});
            const bool surf = const_cast<TopoOffsetTetMesh*>(this)->is_edge_on_surface(et);
            logger().warn(
                "[forensics]   edge {}-{} len {:.6g} | collapse gate {:.6g} -> {} | split gate "
                "{:.6g} -> {} | force-split queued {} | on_surface {}",
                u,
                w,
                len,
                coll_c * sbar,
                len <= coll_c * sbar ? "offered" : "NEVER OFFERED (too long)",
                split_c * sbar,
                len > split_c * sbar ? "SPLIT CANDIDATE" : "too short",
                is_force_split_edge(u, w),
                surf);
        }
    }

    // ---- coincident vertices ----
    const double eps = 1e-9 * l;
    std::unordered_map<long long, std::vector<size_t>> cells;
    const auto key = [&](const Vector3d& p) {
        return (long long)(std::llround(p[0] / (eps * 10.))) * 1000003LL * 1000003LL +
               (long long)(std::llround(p[1] / (eps * 10.))) * 1000003LL +
               (long long)(std::llround(p[2] / (eps * 10.)));
    };
    std::vector<size_t> live;
    for (const Tuple& v : get_vertices()) live.push_back(v.vid(*this));
    for (const size_t v : live) cells[key(m_vertex_attribute[v].m_posf)].push_back(v);
    size_t n_pairs = 0, n_pairs_no_edge = 0;
    std::string first;
    for (const auto& [k, group] : cells) {
        for (size_t i = 0; i < group.size(); ++i) {
            for (size_t j = i + 1; j < group.size(); ++j) {
                const double d =
                    (m_vertex_attribute[group[i]].m_posf - m_vertex_attribute[group[j]].m_posf)
                        .norm();
                if (d > eps) continue;
                ++n_pairs;
                const auto nbs = get_one_ring_vids_for_vertex(group[i]);
                const bool shares = std::find(nbs.begin(), nbs.end(), group[j]) != nbs.end();
                if (!shares) ++n_pairs_no_edge;
                if (first.empty()) {
                    first = fmt::format(
                        "first: {} and {} are {:.3g} apart, share an edge: {}",
                        group[i],
                        group[j],
                        d,
                        shares);
                }
            }
        }
    }
    if (n_pairs == 0) {
        logger().info("[forensics] coincident vertices (closer than {:.3g}): none", eps);
    }
    if (n_pairs > 0)
        logger().warn(
            "[forensics] coincident vertices (closer than {:.3g}): {} pairs, {} of them NOT joined "
            "by an edge (no collapse can reach those). {}",
            eps,
            n_pairs,
            n_pairs_no_edge,
            first.empty() ? "none" : first);
    logger().info(
        "[forensics] genesis tally: flat tets made from a HEALTHY parent -- split {}, collapse "
        "{} | flat-from-flat (multiplication) {}",
        m_flat_created_split.load(),
        m_flat_created_collapse.load(),
        m_flat_worsened_split.load());
}

std::vector<bool> TopoOffsetTetMesh::band_vertex_mask() const
{
    std::vector<bool> on_band(vert_capacity(), false);
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        for (const size_t vid : get_face_vids(f)) on_band[vid] = true;
    }
    return on_band;
}

double TopoOffsetTetMesh::band_vertex_distance_error(const size_t vid) const
{
    const Vector3d p = m_vertex_attribute[vid].m_posf;
    return std::abs(input_complex_distance(p) - m_offset_params.target_distance);
}

double TopoOffsetTetMesh::band_vertex_residual(const size_t vid) const
{
    return front_vertex_residual_length(vid);
}

TopoOffsetTetMesh::FaceSamples TopoOffsetTetMesh::offset_face_samples(const Tuple& f) const
{
    FaceSamples s;
    if (m_offset_params.stencil_order < 0) return s;
    for (const size_t v : get_face_vids(f)) {
        if (!band_vertex_is_reachable(v)) return s;
    }
    const OffsetPotential3D& pot = potential_for_face(f);
    for_each_offset_face_sample(f, [&](const Vector3d& q, double, double, double) {
        const double r = pot.residual_length(q);
        s.max = std::max(s.max, r);
        s.sum += r;
        ++s.n;
    });
    return s;
}

TopoOffsetTetMesh::DistanceSplit TopoOffsetTetMesh::residual_split() const
{
    // The band's Phi residual. Every offset-surface vertex and every face sample counts toward
    // the driving max, pinned ones included; the reachable/pinned split is attribution. Same as
    // 2D.
    const std::vector<bool> on_band = band_vertex_mask();

    DistanceSplit s;
    double sum_reachable = 0.;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid]) continue;
        const Vector3d p = m_vertex_attribute[vid].m_posf;
        const double err = front_vertex_residual_length(vid);
        s.max_reachable = std::max(s.max_reachable, err);
        s.max_at_vertex = std::max(s.max_at_vertex, err);
        sum_reachable += err;
        ++s.n_reachable;
        if (band_vertex_is_reachable(vid)) {
            if (!potential_for(vid).within_support(p)) {
                ++s.n_outside_support;
                const double d = input_complex_distance(p);
                if (d > s.worst_outside_dist) {
                    s.worst_outside_dist = d;
                    s.worst_outside_vid = vid;
                }
            }
        } else {
            s.max_pinned = std::max(s.max_pinned, err);
            ++s.n_pinned;
        }
    }
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        const FaceSamples fs = offset_face_samples(f);
        if (fs.n == 0) continue;
        s.max_reachable = std::max(s.max_reachable, fs.max);
        s.max_in_face = std::max(s.max_in_face, fs.max);
        sum_reachable += fs.sum;
        s.n_reachable += fs.n;
    }

    s.avg_reachable = (s.n_reachable > 0) ? sum_reachable / s.n_reachable : 0.;
    return s;
}

TopoOffsetTetMesh::GradientSplit TopoOffsetTetMesh::gradient_split(
    const bool include_face_samples) const
{
    // ||grad (Phi(x) - c)^2|| at every band vertex, on the field the vertex is placed on, plus
    // the face-interior half on the same lattice the residual is sampled on. Same as 2D.
    const std::vector<bool> on_band = band_vertex_mask();
    std::vector<std::unique_ptr<OffsetEnergy3D>> energies;
    for (const auto& rp : m_region_potentials)
        energies.push_back(std::make_unique<OffsetEnergy3D>(rp, 1.0, true, true));
    OffsetEnergy3D union_energy(m_offset_potential, 1.0, true, true);
    const auto energy_for = [&](const int region) -> OffsetEnergy3D& {
        return (region >= 0 && size_t(region) < energies.size()) ? *energies[size_t(region)]
                                                                 : union_energy;
    };

    GradientSplit s;
    double sum_reachable = 0.;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid]) continue;
        if (!m_vertex_extra[vid].m_is_on_offset) continue;

        if (!m_vertex_attribute[vid].m_is_rounded) {
            ++s.n_skipped_unrounded;
            continue;
        }
        const std::vector<Tuple> locs = get_one_ring_tets_for_vertex(v);
        if (locs.empty()) continue;
        bool inverted = false;
        for (const Tuple& loc : locs) {
            if (is_inverted_f(loc)) {
                inverted = true;
                break;
            }
        }
        if (inverted) {
            ++s.n_skipped_inverted;
            continue;
        }

        Eigen::VectorXd g(3);
        const Eigen::VectorXd x = m_vertex_attribute[vid].m_posf;
        energy_for(vertex_region(vid)).gradient(x, g);
        const double gn = g.norm();

        if (!band_vertex_is_reachable(vid)) {
            s.max_pinned = std::max(s.max_pinned, gn);
            ++s.n_pinned;
            continue;
        }

        s.max_reachable = std::max(s.max_reachable, gn);
        s.max_at_vertex = std::max(s.max_at_vertex, gn);
        sum_reachable += gn;
        ++s.n_reachable;
    }

    if (include_face_samples) {
        for (const Tuple& f : get_faces()) {
            if (!face_is_offset_surface_live(f)) continue;
            const auto vs = get_face_vids(f);
            const bool gating = band_vertex_is_reachable(vs[0]) &&
                                band_vertex_is_reachable(vs[1]) && band_vertex_is_reachable(vs[2]);
            const std::optional<Tuple> opp = f.switch_tetrahedron(*this);
            size_t band = f.tid(*this);
            if (!cell_is_offset_band(band) && opp) band = opp->tid(*this);
            const int region = band < m_cell_region.size() ? m_cell_region[band] : -1;
            for_each_offset_face_sample(f, [&](const Vector3d& q, double, double, double) {
                ++s.n_face_samples;
                if (!gating) return;
                Eigen::VectorXd g(3);
                energy_for(region).gradient(Eigen::VectorXd(q), g);
                s.max_in_face = std::max(s.max_in_face, g.norm());
            });
        }
    }

    s.avg_reachable = (s.n_reachable > 0) ? sum_reachable / s.n_reachable : 0.;
    return s;
}

TopoOffsetTetMesh::EnergyCriterion TopoOffsetTetMesh::energy_criterion()
{
    EnergyCriterion s;
    // THE bar, as a length: a face is resolved when its RMS relative error is within it, and a
    // vertex placed when its own relative error is. One key for both since 2026-09-24. The
    // offset surface's envelope (envelope_size) holds the front in a TetWild pass alone and is
    // not an accuracy.
    s.bar_length = m_offset_params.front_conv;
    const auto front = [&](const size_t vid) {
        return m_vertex_extra[vid].m_is_on_offset && m_vertex_attribute[vid].m_is_rounded;
    };
    std::vector<char> placed(vert_capacity(), 0);
    // front_measure "vertex_ring": the ring measure is accumulated per corner from the face
    // loop's own face_offset_term() calls below, so it judges exactly the face numbers the face
    // mode does, every face weighted equally. See EnergyCriterion::ring_exit.
    s.ring_exit = m_offset_params.front_measure == "vertex_ring";
    // ring_n: the faces summed at each vertex, the denominator of every ring mean.
    std::vector<double> ring_sum;
    std::vector<size_t> ring_n;
    std::vector<char> ring_bad;
    if (s.ring_exit) {
        ring_sum.assign(vert_capacity(), 0.);
        ring_n.assign(vert_capacity(), 0);
        ring_bad.assign(vert_capacity(), 0);
    }
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!front(vid)) continue;
        // gn is the vertex's convergence measure over the one bar; rho the
        // length residual_length(), its actual distance to the level set. rho is
        // reported and gates measurability, NOT placement: front_placed_by_ratio() is the one
        // notion, and it reads gn. See the declaration for what qualifying the sag test's corners
        // by rho instead used to cost.
        const Vector3d p = m_vertex_attribute[vid].m_posf;
        const double rho = front_vertex_residual_length(vid);
        const double gn = front_vertex_conv_ratio(vid);
        if (!std::isfinite(rho) || !std::isfinite(gn)) {
            ++s.n_unmeasurable;
            continue;
        }
        if (front_placed_by_ratio(gn)) {
            placed[vid] = 1;
        } else {
            ++s.n_unplaced;
        }
        ++s.n_vertices;
        s.sum_vertex += gn;
        if (gn > s.max_vertex) {
            s.max_vertex = gn;
            s.worst_vid = vid;
        }
    }
    // The resolution test is per FACE, sampled over its interior lattice: on a surface the
    // interpolation error peaks inside the face, and a chord test on the edges alone misses it.
    for (const auto& f : offset_surface_faces()) {
        const size_t va = f[0], vb = f[1], vc = f[2];
        if (!front(va) || !front(vb) || !front(vc)) continue;
        // ONE call feeds both jobs: this number is the loop's exit test (max_face / faces_ok(),
        // see converged()) AND what decides which faces the refinement is handed. Under
        // front_measure "vertex_ring" it feeds both through the ring measure instead. gn is
        // the root of the face term, the figure the logs print.
        const double term = face_offset_term(va, vb, vc);
        const double gn = term < 0. ? -1. : std::sqrt(term); // the sag / bar_length
        if (gn < 0.) {
            ++s.n_unmeasurable;
            if (s.ring_exit) ring_bad[va] = ring_bad[vb] = ring_bad[vc] = 1;
            continue;
        }
        const Vector3d& pa = m_vertex_attribute[va].m_posf;
        const Vector3d& pb = m_vertex_attribute[vb].m_posf;
        const Vector3d& pc = m_vertex_attribute[vc].m_posf;
        const Vector3d centroid = (pa + pb + pc) / 3.;
        if (s.ring_exit) {
            for (const size_t u : {va, vb, vc}) {
                ring_sum[u] += term;
                ++ring_n[u];
            }
        }
        // The longest edge is the chord the target is derived from.
        const std::array<std::pair<size_t, size_t>, 3> es = {{{va, vb}, {vb, vc}, {vc, va}}};
        size_t la = va, lb = vb;
        double len = 0.;
        for (const auto& [u, w] : es) {
            const double d = (m_vertex_attribute[u].m_posf - m_vertex_attribute[w].m_posf).norm();
            if (d > len) {
                len = d;
                la = u;
                lb = w;
            }
        }
        ++s.n_faces;
        s.sum_face += gn;
        if (gn > s.max_face) {
            s.max_face = gn;
        }
        if (gn > s.bar) {
            ++s.n_faces_over;
            const bool corners_placed = placed[va] && placed[vb] && placed[vc];
            if (corners_placed) ++s.n_faces_over_placed;
            // Every face over the bar is refinable, placed or not. There is no placement gate:
            // under one unified measure the corners can be held off the level set BY the sag of
            // the very faces such a gate would refuse to refine, leaving the loop no lever.
            // `n_faces_over_placed` and `max_face_placed` are the PLACED subset, for reporting
            // only. Under the ring measure no face is handed to the refinement: the vertices
            // are, after this loop.
            if (!s.ring_exit) {
                // Refinable only if the rule can still lower a target; judged against the MAX of
                // the three scalars (the 2D twin uses the max of its chord's two).
                const double l = std::max(m_params.l, 1e-300);
                const double s_floor = std::max(
                    m_offset_params.min_sizing_scalar,
                    m_offset_params.min_edge_length / l);
                const size_t lc = (la != va && lb != va) ? va : ((la != vb && lb != vb) ? vb : vc);
                const double have = std::max(
                    {m_vertex_attribute[va].m_sizing_scalar,
                     m_vertex_attribute[vb].m_sizing_scalar,
                     m_vertex_attribute[vc].m_sizing_scalar});
                // How short this face's chord would have to become, as a sizing scalar: the
                // chord rule inverts the sagitta's power law to a length. Refinement itself is
                // the halving in refine_front_by_halving(), but the question asked here is the
                // same one either way -- is there any target left below what the corners
                // already carry, or are they at the floor.
                const double target =
                    front_chord_target(la, lb, len, gn * s.bar_length, s.bar_length);
                const double sn =
                    std::clamp(target / l, s_floor, m_offset_params.max_sizing_scalar);
                if (sn < have) {
                    s.refinable.push_back({la, lb, lc, gn * s.bar_length, len});
                } else {
                    // Not handed to the refinement, for one of two reasons. Either the corners
                    // are at the sizing floor (have <= s_floor): nothing can refine the face and
                    // it blocks the exit for good. Or the chord target, which is at most half
                    // the longest edge, is not below have x l: that edge is then at least twice
                    // the largest target length at the corners, long enough for the split
                    // pass's length gate, which shortens it with no lowering at all.
                    ++s.n_at_floor;
                    s.floor_scalar = s_floor;
                    s.floor_from_min_edge_length =
                        m_offset_params.min_edge_length / l > m_offset_params.min_sizing_scalar;
                    if (gn > s.max_face_at_floor) {
                        s.max_face_at_floor = gn;
                        s.worst_at_floor_centroid = centroid;
                        s.worst_at_floor_scalar = have;
                    }
                    if (have <= s_floor) {
                        ++s.n_corners_at_floor;
                        if (gn > s.max_face_corners_at_floor) {
                            s.max_face_corners_at_floor = gn;
                            s.worst_corners_at_floor_centroid = centroid;
                        }
                    }
                }
                if (corners_placed && gn > s.max_face_placed) {
                    s.max_face_placed = gn;
                    s.worst_placed_centroid = centroid;
                }
            }
        }
    }
    if (s.ring_exit) {
        // The ring measure and its refinement. A vertex over the bar is refinable while the
        // halving can still lower its own scalar -- the floor rule of refine_front_by_halving(),
        // nothing else: no chord target, no placement gate. One at the floor blocks the exit.
        const double l = std::max(m_params.l, 1e-300);
        const double s_floor =
            std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / l);
        for (const Tuple& v : get_vertices()) {
            const size_t vid = v.vid(*this);
            if (!front(vid)) continue;
            if (ring_bad[vid]) {
                ++s.n_rings_unmeasurable;
                continue;
            }
            if (ring_n[vid] == 0) continue; // no measured offset face: no ring to judge
            const double r = std::sqrt(ring_sum[vid] / double(ring_n[vid]));
            ++s.n_rings;
            s.sum_ring += r;
            if (r > s.max_ring) {
                s.max_ring = r;
                s.worst_ring_vid = vid;
            }
            if (!(r > s.bar)) continue;
            ++s.n_rings_over;
            const double have = m_vertex_attribute[vid].m_sizing_scalar;
            if (have > s_floor) {
                s.refinable_vertices.push_back(vid);
            } else {
                ++s.n_rings_at_floor;
                s.floor_scalar = s_floor;
                s.floor_from_min_edge_length =
                    m_offset_params.min_edge_length / l > m_offset_params.min_sizing_scalar;
                if (r > s.max_ring_at_floor) {
                    s.max_ring_at_floor = r;
                    s.worst_ring_at_floor_pos = m_vertex_attribute[vid].m_posf;
                }
            }
        }
    }
    return s;
}

std::string TopoOffsetTetMesh::EnergyCriterion::sizing_floor_fact() const
{
    if (ring_exit) {
        if (n_rings_at_floor == 0) return "";
        const char* origin =
            floor_from_min_edge_length ? "min_edge_length / l" : "min_sizing_scalar";
        return fmt::format(
            "{} front vertex(es) with the {} over the bar have their sizing scalar at the "
            "sizing floor {:.4g} (from {}), so they cannot be refined and the loop cannot "
            "converge on them: worst {:.4}x the bar at ({:.4}, {:.4}, {:.4})",
            n_rings_at_floor,
            ring_name(),
            floor_scalar,
            origin,
            max_ring_at_floor,
            worst_ring_at_floor_pos.x(),
            worst_ring_at_floor_pos.y(),
            worst_ring_at_floor_pos.z());
    }
    if (n_at_floor == 0) return "";
    const char* origin = floor_from_min_edge_length ? "min_edge_length / l" : "min_sizing_scalar";
    if (n_corners_at_floor > 0) {
        return fmt::format(
            "{} offset face(s) over the bar have every corner at the sizing floor {:.4g} (from "
            "{}), so they cannot be refined and the loop cannot converge on them: worst {:.4}x "
            "the bar at centroid ({:.4}, {:.4}, {:.4}). Of the {} face(s) over the bar with no "
            "chord target below the largest sizing scalar at their corners, the other {} have a "
            "longest edge at least twice that target length, which the split pass shortens",
            n_corners_at_floor,
            floor_scalar,
            origin,
            max_face_corners_at_floor,
            worst_corners_at_floor_centroid.x(),
            worst_corners_at_floor_centroid.y(),
            worst_corners_at_floor_centroid.z(),
            n_at_floor,
            n_at_floor - n_corners_at_floor);
    }
    return fmt::format(
        "{} offset face(s) over the bar have no chord target below the largest sizing scalar at "
        "their corners, and none has its corners at the sizing floor {:.4g} (from {}): each has "
        "a longest edge at least twice the largest target length at its corners, which the "
        "split pass shortens, so no refinement is needed; worst {:.4}x the bar at centroid "
        "({:.4}, {:.4}, {:.4}), corner scalar {:.4g}",
        n_at_floor,
        floor_scalar,
        origin,
        max_face_at_floor,
        worst_at_floor_centroid.x(),
        worst_at_floor_centroid.y(),
        worst_at_floor_centroid.z(),
        worst_at_floor_scalar);
}

double TopoOffsetTetMesh::front_gradient_linf()
{
    double worst = 0.;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!m_vertex_extra[vid].m_is_on_offset || !m_vertex_attribute[vid].m_is_rounded) continue;
        // The objective's normal gradient, always: the vertex's convergence ratio is its relative
        // error, not a stationarity measure, so it would not be the "gradient reference" the log
        // calls this. As in 2D.
        worst = std::max(worst, front_vertex_normal_gradient(vid));
    }
    return worst;
}

double TopoOffsetTetMesh::front_vertex_conv_ratio(const size_t vid) const
{
    // THE measure at one point, against THE bar: the vertex's own relative error
    // |relative_residual(x)| over front_conv_frac(), i.e. its distance to the level set along the
    // field over front_conv (see face_offset_term()). This is the face term's order-0 stencil
    // evaluated at a single corner, which is what makes the vertex test and the face test one
    // test rather than two.
    const OffsetPotential3D& pot = potential_for(vid);
    const double level = pot.target_level();
    if (!(level > 0.)) return std::numeric_limits<double>::infinity();
    const double r = front_vertex_relative_residual(vid);
    if (!std::isfinite(r)) return std::numeric_limits<double>::infinity();
    const double bar = m_offset_params.front_conv_frac();
    if (!(bar > 0.)) return std::numeric_limits<double>::infinity();
    return std::abs(r) / bar;
}

// ---------------------------------------------------------------------------------------------
// The front readings: a front face's band cell, and the field at a front vertex.
// ---------------------------------------------------------------------------------------------

int64_t TopoOffsetTetMesh::front_face_band_cell(const size_t a, const size_t b, const size_t c)
    const
{
    for (const size_t tid : get_one_ring_tids_for_vertex(a)) {
        if (!cell_is_offset_band(tid)) continue;
        const auto t = oriented_tet_vids(tid);
        if (std::find(t.begin(), t.end(), b) != t.end() &&
            std::find(t.begin(), t.end(), c) != t.end()) {
            return int64_t(tid);
        }
    }
    return -1;
}

double TopoOffsetTetMesh::front_vertex_relative_residual(const size_t vid) const
{
    return potential_for(vid).relative_residual(m_vertex_attribute[vid].m_posf);
}

double TopoOffsetTetMesh::front_vertex_residual_length(const size_t vid) const
{
    return potential_for(vid).residual_length(m_vertex_attribute[vid].m_posf);
}

Vector3d TopoOffsetTetMesh::front_vertex_field_gradient(const size_t vid) const
{
    return potential_for(vid).gradient(m_vertex_attribute[vid].m_posf);
}

void TopoOffsetTetMesh::assign_band_regions(const bool log)
{
    // See m_region_potentials. A flood fill over the band cells, seeded from every band cell
    // with an input-complex vertex, whose piece is read off the per-piece envelopes (the nearest
    // piece to a vertex ON the complex is its own, at distance 0). As in 2D, a cell reached from
    // two regions and a vertex on cells of two regions read -2 and fall back to the union field.
    m_cell_region.assign(tet_capacity(), -1);
    m_vertex_region.assign(vert_capacity(), -1);
    if (m_n_regions <= 1 || m_region_potentials.empty() || m_region_envelopes.empty()) return;
    const auto region_at = [&](const Vector3d& p) -> int {
        int best = -1;
        double best_d = std::numeric_limits<double>::infinity();
        for (size_t r = 0; r < m_region_envelopes.size(); ++r) {
            const double d = m_region_envelopes[r]->squared_distance(p);
            if (d < best_d) {
                best_d = d;
                best = int(r);
            }
        }
        return best;
    };
    std::vector<size_t> queue;
    std::vector<int> piece_of_vertex(vert_capacity(), -3); // -3: not looked up yet
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (!cell_is_offset_band(tid)) continue;
        for (const size_t v : oriented_tet_vids(tid)) {
            if (m_vertex_extra[v].label != 1) continue;
            if (piece_of_vertex[v] == -3) {
                piece_of_vertex[v] = region_at(m_vertex_attribute[v].m_posf);
            }
            const int r = piece_of_vertex[v];
            if (r < 0) continue;
            if (m_cell_region[tid] == -1) {
                m_cell_region[tid] = r;
                queue.push_back(tid);
            } else if (m_cell_region[tid] >= 0 && m_cell_region[tid] != r) {
                m_cell_region[tid] = -2;
            }
        }
    }
    while (!queue.empty()) {
        const size_t t = queue.back();
        queue.pop_back();
        const int r = m_cell_region[t];
        if (r < 0) continue;
        for (int j = 0; j < 4; ++j) {
            const std::optional<Tuple> opp = tuple_from_face(t, j).switch_tetrahedron(*this);
            if (!opp) continue;
            const size_t g = opp->tid(*this);
            if (!cell_is_offset_band(g)) continue;
            if (m_cell_region[g] == -1) {
                m_cell_region[g] = r;
                queue.push_back(g);
            } else if (m_cell_region[g] >= 0 && m_cell_region[g] != r) {
                m_cell_region[g] = -2;
            }
        }
    }
    std::vector<size_t> n_cells(size_t(m_n_regions), 0);
    size_t n_mixed_cells = 0, n_unreached = 0, n_mixed_verts = 0;
    for (size_t t = 0; t < m_cell_region.size(); ++t) {
        if (!tuple_from_tet(t).is_valid(*this) || !cell_is_offset_band(t)) continue;
        const int r = m_cell_region[t];
        if (r == -2) {
            ++n_mixed_cells;
            continue;
        }
        if (r < 0) {
            ++n_unreached;
            continue;
        }
        ++n_cells[size_t(r)];
        for (const size_t v : oriented_tet_vids(t)) {
            if (m_vertex_region[v] == -1) {
                m_vertex_region[v] = r;
            } else if (m_vertex_region[v] >= 0 && m_vertex_region[v] != r) {
                m_vertex_region[v] = -2;
                ++n_mixed_verts;
            }
        }
    }
    if (!log) return;
    std::string per;
    for (size_t r = 0; r < n_cells.size(); ++r)
        per += fmt::format("{}{}", r ? " / " : "", n_cells[r]);
    if (n_mixed_cells > 0 || n_unreached > 0 || n_mixed_verts > 0) {
        logger().warn(
            "\t[regions] band cells per region {} | {} cells reached from TWO regions, {} reached "
            "from none, {} vertices on cells of two regions -- all fall back to the union field",
            per,
            n_mixed_cells,
            n_unreached,
            n_mixed_verts);
    } else {
        logger().info("\t[regions] band cells per region {}", per);
    }
}

double TopoOffsetTetMesh::front_chord_target(
    const size_t va,
    const size_t vb,
    const double len,
    const double sag,
    const double bar) const
{
    // 3/4 L (bar / sag)^(1/p), capped at L/2, with the exponent p measured from how the level
    // set turns across the chord: 2 on a smooth level set, 1 where the chord straddles a kink.
    // See the 2D twin for the derivation. The sag fraction a halving leaves is ratio = 2^-p, so
    // p = -log2(ratio): 1/4 -> 2, 1/2 -> 1. (The 2D twin writes -1 / log2(ratio), which maps
    // 1/4 to 0.5 and 1/2 to 1 and so over-refines every smooth chord by (sag / bar)^(1..2)
    // instead of the square root -- affordable on a curve, cubic on a surface. Not mirrored.)
    double p = 2.;
    const OffsetPotential3D& pot = potential_for_edge(va, vb);
    const Vector3d pa = m_vertex_attribute[va].m_posf, pb = m_vertex_attribute[vb].m_posf;
    const Vector3d ga = pot.gradient(pa), gb = pot.gradient(pb), gm = pot.gradient(0.5 * (pa + pb));
    const double na = ga.norm(), nb = gb.norm(), nm = gm.norm();
    if (std::isfinite(na) && na > 0. && std::isfinite(nb) && nb > 0. && std::isfinite(nm) &&
        nm > 0.) {
        const Vector3d ua = ga / na, ub = gb / nb, um = gm / nm;
        const auto turn = [](const Vector3d& u, const Vector3d& v) {
            return std::atan2(u.cross(v).norm(), u.dot(v));
        };
        const double phi = turn(ua, ub);
        if (phi > 0.) {
            const double ratio =
                std::clamp(0.5 * std::max(turn(ua, um), turn(um, ub)) / phi, 0.25, 0.5);
            p = -std::log2(ratio);
        }
    }
    return std::min(0.75 * len * std::pow(bar / sag, 1. / p), 0.5 * len);
}

size_t TopoOffsetTetMesh::refine_front_by_halving(
    const std::vector<EnergyCriterion::Refinable>& faces)
{
    // The corners, face by face in the order given; the vertex form halves each once.
    std::vector<size_t> corners;
    corners.reserve(3 * faces.size());
    for (const EnergyCriterion::Refinable& r : faces) {
        for (const size_t v : {r.a, r.b, r.c}) corners.push_back(v);
    }
    return refine_front_by_halving(corners);
}

size_t TopoOffsetTetMesh::refine_front_by_halving(const std::vector<size_t>& vertices)
{
    const double l = std::max(m_params.l, 1e-300);
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / l);
    // Each vertex is halved once per call: the first entry that names it does the halving and
    // marks it, so a vertex shared by several refinable faces is not halved several times.
    std::vector<size_t> changed;
    std::vector<char> done(vert_capacity(), 0);
    for (const size_t v : vertices) {
        if (done[v]) continue;
        done[v] = 1;
        double& sc = m_vertex_attribute[v].m_sizing_scalar;
        const double sn = std::max(0.5 * sc, s_floor);
        if (sn < sc) {
            sc = sn;
            changed.push_back(v);
        }
    }
    grade_sizing(m_offset_params.sizing_gradation, changed);
    return changed.size();
}

size_t TopoOffsetTetMesh::offset_stuck_refine(
    const std::vector<std::array<size_t, 3>>& faces,
    size_t* n_edges)
{
    // See the declaration: refine_sizing_around_worst()'s forced split and sizing refinement, on
    // the faces out of convergence instead of the worst cells, the sizing through the front's own
    // halving. The forced split obeys stuck_refine_force_split, as TetWild's does.
    std::vector<size_t> corners;
    corners.reserve(3 * faces.size());
    size_t queued = 0;
    for (const auto& f : faces) {
        if (m_params.stuck_refine_force_split) {
            queued += m_force_split_edges
                          .insert(
                              wmtk::utils::longest_edge(
                                  f,
                                  [this](size_t vid) -> const Vector3d& {
                                      return m_vertex_attribute[vid].m_posf;
                                  }))
                          .second;
        }
        corners.insert(corners.end(), f.begin(), f.end());
    }
    if (n_edges) *n_edges = queued;
    return refine_front_by_halving(corners);
}

bool TopoOffsetTetMesh::convergence_turn(const int turn, StuckState& s, const bool refine_now)
{
    // See the declaration: mesh_improvement()'s exit and stall response, on the two bounded
    // quantities -- the max AMIPS over the cells against stop_energy, and the max stencil-point
    // offset error over every unique stencil point of the offset surface against
    // front_conv_frac()^2. Each cell and point is evaluated on the mesh's threads and reduced on
    // this one.
    const TopoOffsetTetMesh& self = *this;
    const double a_bar = m_params.stop_energy;
    const double o_bar = front_offset_bar();

    std::vector<double> q(tet_capacity(), 0.);
    diagnostic_parallel_for(q.size(), kDiagnosticChunk, [&](const size_t b, const size_t e) {
        for (size_t t = b; t < e; ++t) {
            if (self.tuple_from_tet(t).is_valid(self)) q[t] = self.loop_cell_quality(t);
        }
    });
    double a3 = 0.;
    for (const double x : q) a3 = std::max(a3, x);
    const double max_amips = std::cbrt(a3);

    // Every stencil point of the offset surface ONCE (StencilPoint): each face's points named,
    // sorted, the first entry of each name evaluated. A face is out of convergence when any of
    // its points is at or over the bound -- so a corner or an edge point out marks every face
    // around it -- and every face out goes to the offset stuck-refine.
    const std::vector<std::array<size_t, 3>> faces = offset_surface_faces();
    std::vector<std::pair<StencilPoint, size_t>> named; // a point, a face it is on
    {
        std::vector<StencilPoint> fp;
        for (size_t k = 0; k < faces.size(); ++k) {
            fp.clear();
            face_stencil_points(faces[k][0], faces[k][1], faces[k][2], fp);
            for (const StencilPoint& p : fp) named.emplace_back(p, k);
        }
    }
    std::sort(named.begin(), named.end(), [](const auto& x, const auto& y) {
        return x.first < y.first || (x.first == y.first && x.second < y.second);
    });
    std::vector<size_t> first; // each unique point's first entry in `named`
    for (size_t i = 0; i < named.size(); ++i) {
        if (i == 0 || !(named[i].first == named[i - 1].first)) first.push_back(i);
    }
    std::vector<double> pe(first.size(), 0.);
    diagnostic_parallel_for(first.size(), kDiagnosticChunk, [&](const size_t b, const size_t e) {
        for (size_t u = b; u < e; ++u) pe[u] = self.stencil_point_error(named[first[u]].first);
    });
    // The floor of refine_front_by_halving(), for the count of out-of-convergence corners at it.
    const double l = std::max(m_params.l, 1e-300);
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / l);
    double max_o = 0.;
    size_t n_unmeasurable = 0, n_floor = 0, n_points_out = 0;
    std::vector<char> face_out(faces.size(), 0);
    for (size_t u = 0; u < first.size(); ++u) {
        const double e = pe[u];
        if (e < 0.) {
            ++n_unmeasurable;
            continue;
        }
        max_o = std::max(max_o, e);
        if (e < o_bar) continue;
        ++n_points_out;
        const size_t end = u + 1 < first.size() ? first[u + 1] : named.size();
        for (size_t i = first[u]; i < end; ++i) face_out[named[i].second] = 1;
    }
    std::vector<std::array<size_t, 3>> out;
    std::vector<char> seen(vert_capacity(), 0);
    for (size_t k = 0; k < faces.size(); ++k) {
        if (!face_out[k]) continue;
        out.push_back(faces[k]);
        for (const size_t v : faces[k]) {
            if (seen[v]) continue;
            seen[v] = 1;
            n_floor += m_vertex_attribute[v].m_sizing_scalar <= s_floor;
        }
    }
    const bool converged = max_amips < a_bar && max_o < o_bar && n_unmeasurable == 0;

    // The stall response, once per metric, in mesh_improvement()'s order: a cooldown turn only
    // counts down; otherwise, from the second turn on, a metric at or over its bound whose fall
    // stalled refines. AMIPS first: refine_sizing_around_worst() clears m_force_split_edges,
    // offset_stuck_refine() appends to it.
    const double eps = m_params.stuck_refine_stall_eps;
    const auto stuck = [&](int& cooldown,
                           const bool over,
                           const double prev,
                           const double cur,
                           const double target,
                           const auto& refine) -> std::string {
        if (converged) return "converged";
        if (!refine_now) return "off (this loop does not refine)";
        if (cooldown > 0) {
            --cooldown;
            return "cooldown";
        }
        if (!over) return "within its bound";
        if (turn <= 1) return "first turn";
        const double fall = prev - cur, bar = eps * (cur - target);
        if (!metric_stalled(prev, cur, target, eps)) {
            return fmt::format("falling ({:.4g} > {:g} x {:.4g})", fall, eps, cur - target);
        }
        cooldown = std::max(0, m_params.stuck_refine_cooldown);
        return fmt::format("STALLED (fell {:.4g} <= {:.4g}) -> {}", fall, bar, refine());
    };
    const std::string a_state =
        stuck(s.cooldown_amips, !(max_amips < a_bar), s.prev_amips, max_amips, a_bar, [&]() {
            const size_t n = refine_sizing_around_worst(max_amips);
            return fmt::format(
                "{} vertices lowered, {} edges queued for a forced split",
                n,
                m_force_split_edges.size());
        });
    const std::string o_state =
        stuck(s.cooldown_offset, !out.empty(), s.prev_offset, max_o, o_bar, [&]() {
            size_t n_edges = 0;
            const size_t n = offset_stuck_refine(out, &n_edges);
            return fmt::format(
                "{} faces: {} longest edges queued for a forced split, {} vertices halved",
                out.size(),
                n_edges,
                n);
        });
    s.prev_amips = max_amips;
    s.prev_offset = max_o;

    logger().log(
        n_unmeasurable > 0 ? spdlog::level::warn : spdlog::level::info,
        "\t[convergence] turn {}: converged {} | max AMIPS {:.6g} (stop {:g}) | max "
        "stencil-point offset error {:.6g} (bound front_conv_frac^2 = {:.6g}; {:.4g}x the bar "
        "in distance) over {} stencil points (stencil_order {}) of {} offset faces, {} points "
        "out of convergence on {} faces ({} of their corners at the sizing floor {:.4g}), {} "
        "unmeasurable{} | global offset error {:.10g} (diagnostic) | stuck-refine A: {} | "
        "stuck-refine O: {}",
        turn,
        converged ? "yes" : "no",
        max_amips,
        a_bar,
        max_o,
        o_bar,
        o_bar > 0. ? std::sqrt(max_o / o_bar) : std::numeric_limits<double>::infinity(),
        first.size(),
        m_offset_params.stencil_order,
        faces.size(),
        n_points_out,
        out.size(),
        n_floor,
        s_floor,
        n_unmeasurable,
        n_unmeasurable > 0 ? " (they block the exit)" : "",
        offset_error(),
        a_state,
        o_state);
    return converged;
}

void TopoOffsetTetMesh::grade_sizing(double grade, const std::vector<size_t>& seeds)
{
    if (seeds.empty()) return;
    if (m_offset_params.sizing_gradation_mode == "distance") {
        grade_sizing_by_distance(seeds);
    } else {
        gradation_smooth_sizing(grade, seeds);
    }
}

size_t TopoOffsetTetMesh::grade_sizing_by_distance(const std::vector<size_t>& seeds)
{
    // Ported statement for statement from tetwild::TetWild::adjust_sizing_field (the attic/
    // directory, since deleted; read attic/app/tetwild/TetWild.cpp at 2371c13fe8): the same R,
    // the same ramp, the same breadth-first walk that stops at R, the same floor. Two parts of
    // that function are NOT ported on purpose:
    //   - the seeds are not multiplied. TetWild's seeds are the worst tets' vertices and the
    //     0.5 at dist 0 IS their refinement; here the caller has just set each seed's scalar to
    //     the value it wants, and halving it again would refine the seed twice.
    //   - the 1.5x recovery TetWild applies to every vertex outside the ball. That is TetWild's
    //     stall response coarsening the field back, not gradation; here it would undo the
    //     front's resolution on every call.
    // TetWild finds the nearest seed with geogram's nearest-neighbour search; a uniform grid of
    // cell size R does the same job exactly for the only question asked, "which seed within R
    // is nearest", without the dependency.
    if (seeds.empty()) return 0;
    const double l = std::max(m_params.l, 1e-16);
    const double R = 1.8 * l;
    const double refine_scalar = 0.5;
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / l);

    std::vector<char> is_seed(vert_capacity(), 0);
    std::vector<Vector3d> pts;
    pts.reserve(seeds.size());
    for (const size_t v : seeds) {
        if (is_seed[v]) continue;
        is_seed[v] = 1;
        pts.push_back(m_vertex_attribute[v].m_posf);
    }
    // grid of cell size R: every seed within R of a query lies in the query's cell or one of
    // its 26 neighbours.
    Vector3d lo = pts[0];
    for (const Vector3d& p : pts) lo = lo.cwiseMin(p);
    auto cell_of = [&](const Vector3d& p) {
        return std::array<int64_t, 3>{
            static_cast<int64_t>(std::floor((p[0] - lo[0]) / R)),
            static_cast<int64_t>(std::floor((p[1] - lo[1]) / R)),
            static_cast<int64_t>(std::floor((p[2] - lo[2]) / R))};
    };
    std::map<std::array<int64_t, 3>, std::vector<size_t>> grid;
    for (size_t i = 0; i < pts.size(); ++i) grid[cell_of(pts[i])].push_back(i);
    auto nearest_seed_dist = [&](const Vector3d& p) {
        const auto c = cell_of(p);
        double best2 = std::numeric_limits<double>::infinity();
        for (int64_t dx = -1; dx <= 1; ++dx)
            for (int64_t dy = -1; dy <= 1; ++dy)
                for (int64_t dz = -1; dz <= 1; ++dz) {
                    const auto it = grid.find({c[0] + dx, c[1] + dy, c[2] + dz});
                    if (it == grid.end()) continue;
                    for (const size_t i : it->second)
                        best2 = std::min(best2, (p - pts[i]).squaredNorm());
                }
        return std::sqrt(std::max(best2, 0.));
    };

    std::vector<double> scale_multipliers(vert_capacity(), 1.0);
    std::vector<char> visited(vert_capacity(), 0);
    std::queue<size_t> v_queue;
    for (const size_t v : seeds) v_queue.push(v);
    std::vector<size_t> cache_one_ring;
    size_t n_reached = 0;
    while (!v_queue.empty()) {
        const size_t vid = v_queue.front();
        v_queue.pop();
        if (visited[vid]) continue;
        visited[vid] = 1;
        const double dist = nearest_seed_dist(m_vertex_attribute[vid].m_posf);
        if (dist > R) continue; // outside the R-ball: not graded, and the walk stops here
        ++n_reached;
        scale_multipliers[vid] = std::min(
            scale_multipliers[vid],
            dist / R * (1 - refine_scalar) + refine_scalar); // linear interpolate
        for (const size_t n_vid : get_one_ring_vids_for_vertex_adj(vid, cache_one_ring)) {
            if (visited[n_vid]) continue;
            v_queue.push(n_vid);
        }
    }

    size_t n_lowered = 0;
    size_t n_floored = 0;
    for (size_t vid = 0; vid < vert_capacity(); ++vid) {
        if (!visited[vid] || is_seed[vid] || scale_multipliers[vid] >= 1.) continue;
        double& sc = m_vertex_attribute[vid].m_sizing_scalar;
        double ns = sc * scale_multipliers[vid];
        if (ns < s_floor) {
            ns = s_floor;
            ++n_floored;
        }
        if (ns < sc) {
            sc = ns;
            ++n_lowered;
        }
    }
    logger().info(
        "\t[gradation] distance (TetWild): {} seeds, {} vertices within R = 1.8 l = {:.6g} of "
        "one, {} lowered by the 0.5 .. 1 ramp ({} at the floor {:.6g})",
        pts.size(),
        n_reached,
        R,
        n_lowered,
        n_floored,
        s_floor);
    return n_lowered;
}

void TopoOffsetTetMesh::check_offset_within_support(const char* when) const
{
    report_outside_support(when, residual_split());
}

void TopoOffsetTetMesh::report_outside_support(const char* when, const DistanceSplit& s) const
{
    if (s.n_outside_support == 0) return;

    log_and_throw_error(
        "{}: {} offset-surface vertices have left the smooth offset potential's support "
        "(dhat = {} = offset_dhat_factor x target_distance {}). The worst is vertex {} at "
        "Euclidean distance {} from the input complex, which is {:.2f}x target_distance. Out "
        "there Phi is identically zero WITH a zero gradient: the smoothing term gives those "
        "vertices no direction back, their residual saturates instead of growing, and the "
        "sizing field refines around vertices nothing can move. Raise offset_dhat_factor if "
        "the offset legitimately has to travel that far, or reduce target_distance.",
        when,
        s.n_outside_support,
        m_offset_potential->dhat(),
        m_offset_params.target_distance,
        s.worst_outside_vid,
        s.worst_outside_dist,
        s.worst_outside_dist / std::max(m_offset_params.target_distance, 1e-16));
}

std::pair<double, double> TopoOffsetTetMesh::compute_distance_deviation() const
{
    const std::vector<bool> on_band = band_vertex_mask();

    double max_dist = 0.0;
    double sum_dist = 0.0;
    int n_verts = 0;
    m_worst_dist_vid = static_cast<size_t>(-1);
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid]) continue;
        ++n_verts;
        const double dist = band_vertex_distance_error(vid);
        if (dist > max_dist) {
            max_dist = dist;
            m_worst_dist_vid = vid;
        }
        sum_dist += dist;
    }
    const double avg_dist = (n_verts > 0) ? sum_dist / n_verts : 0.0;
    return std::make_pair(max_dist, avg_dist);
}

TopoOffsetTetMesh::DistanceSplit TopoOffsetTetMesh::distance_deviation_split() const
{
    const std::vector<bool> on_band = band_vertex_mask();

    DistanceSplit s;
    double sum_reachable = 0.;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid]) continue;
        const double err = band_vertex_distance_error(vid);
        if (band_vertex_is_reachable(vid)) {
            s.max_reachable = std::max(s.max_reachable, err);
            sum_reachable += err;
            ++s.n_reachable;
        } else {
            s.max_pinned = std::max(s.max_pinned, err);
            ++s.n_pinned;
        }
    }
    s.avg_reachable = (s.n_reachable > 0) ? sum_reachable / s.n_reachable : 0.;
    return s;
}

double TopoOffsetTetMesh::cell_quality_rel(const size_t tid) const
{
    // cell_quality stores AMIPS^3 and stop_energy is a bar on AMIPS, so the cube root is not
    // optional.
    return std::cbrt(cell_quality(tid)) / std::max(m_params.stop_energy, 1e-16);
}

double TopoOffsetTetMesh::amips_rel_at_face(const Tuple& f) const
{
    double q = cell_quality_rel(f.tid(*this));
    if (const auto opp = f.switch_tetrahedron(*this)) {
        q = std::max(q, cell_quality_rel(opp->tid(*this)));
    }
    return q;
}

double TopoOffsetTetMesh::face_criterion_rel(const Tuple& f) const
{
    // The max of the face's AMIPS over stop_energy and, on a live offset face, the root of its
    // face_offset_term() -- the loop's own face measure, in units of the bar. Sorted corners, as
    // tet_energy() reads the term. An unmeasurable face fails.
    const double score = amips_rel_at_face(f);
    if (!face_is_offset_surface_live(f)) return score;
    std::array<size_t, 3> v = get_face_vids(f);
    std::sort(v.begin(), v.end());
    const double term = face_offset_term(v[0], v[1], v[2]);
    if (!(term >= 0.)) return std::numeric_limits<double>::infinity();
    return std::max(score, std::sqrt(term));
}

size_t TopoOffsetTetMesh::refine_sizing_around_worst(const double max_metric)
{
    // TetWildMesh::refine_sizing_around_worst verbatim -- ranked by element quality, clamped the
    // same way, seeding the same force-split edges. Two callers: mesh_improvement() in a TetWild
    // pass, and the loop's AMIPS stuck-refine (convergence_turn()).
    const int n_rings = std::max(0, m_params.stuck_refine_rings);
    const double filter_energy = std::min(std::max(max_metric / 100., m_params.stop_energy), 100.);

    // The quality is AMIPS^3, so the energy "max energy" refers to is its cube root. In a TetWild
    // pass loop_cell_quality() is cell_quality(); in the loop, plastic AMIPS under use_rest_pose,
    // what the loop's exit reads.
    const auto worst = wmtk::utils::select_worst_cells(
        tet_capacity(),
        [this](size_t tid) { return tuple_from_tet(tid).is_valid(*this); },
        [this](size_t tid) { return std::cbrt(loop_cell_quality(tid)); },
        filter_energy,
        m_params.stuck_refine_num_worst);
    if (worst.empty()) {
        return 0;
    }

    log_stuck_refine_census(max_metric, filter_energy);
    log_refine_block_census(fmt::format("stuck call {}", m_stuck_calls), filter_energy);

    m_force_split_edges.clear();
    if (m_params.stuck_refine_force_split) {
        for (const auto& [unused_score, tid] : worst) {
            m_force_split_edges.insert(
                wmtk::utils::longest_edge(
                    oriented_tet_vids(tid),
                    [this](size_t vid) -> const Vector3d& {
                        return m_vertex_attribute[vid].m_posf;
                    }));
        }
    }

    std::vector<size_t> seeds;
    seeds.reserve(4 * worst.size());
    for (const auto& [unused_score, tid] : worst) {
        for (const size_t v : oriented_tet_vids(tid)) seeds.push_back(v);
    }
    const auto region = wmtk::utils::grow_vertex_region(seeds, n_rings, [this](size_t v) {
        return get_one_ring_vids_for_vertex_adj(v);
    });

    const auto refined = wmtk::utils::apply_sizing_refinement(
        region,
        m_params.stuck_refine_factor,
        m_params.stuck_refine_min_scalar,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; });
    grade_sizing(m_params.stuck_refine_gradation, refined);

    logger().info(
        "[stuck-refine A] worst {} tets (max energy {:.4}, filter {:.4}), refined {} of {} "
        "region vertices",
        worst.size(),
        max_metric,
        filter_energy,
        refined.size(),
        region.size());
    return refined.size();
}

void TopoOffsetTetMesh::log_worst_dist_vertex() const
{
    {
        const DistanceSplit r = residual_split();
        const DistanceSplit d = distance_deviation_split();
        logger().info(
            "\tband split (phi residual): {} reachable (max {:.6}, avg {:.6}) | {} PINNED "
            "(max {:.6})",
            r.n_reachable,
            r.max_reachable,
            r.avg_reachable,
            r.n_pinned,
            r.max_pinned);
        logger().info(
            "\tband split (euclidean dist err): {} reachable (max {:.6}, avg {:.6}) | {} PINNED "
            "(max {:.6})",
            d.n_reachable,
            d.max_reachable,
            d.avg_reachable,
            d.n_pinned,
            d.max_pinned);
    }

    const size_t vid = m_worst_dist_vid;
    if (vid == static_cast<size_t>(-1)) return;

    // The distance band_vertex_distance_error() chose the vertex by, printed beside it.
    const Vector3d p = m_vertex_attribute[vid].m_posf;
    const double d = input_complex_distance(p);

    // Every face incident to vid, classified.
    const auto& ve = m_vertex_extra[vid];
    int n_offset_f = 0, n_region_f = 0, n_bbox_f = 0;
    std::set<size_t> seen_fids;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        const auto tet_vids = oriented_tet_vids(tid);
        for (int skip = 0; skip < 4; ++skip) {
            if (tet_vids[size_t(skip)] == vid) continue;
            const auto [face_tuple, fid] = tuple_from_face(face_corners_from(tet_vids, skip));
            if (!seen_fids.insert(fid).second) continue;
            n_offset_f += face_is_offset_surface_live(face_tuple);
            n_region_f += face_is_region(fid);
            n_bbox_f += (m_face_attribute[fid].m_is_bbox_fs >= 0);
        }
    }
    logger().info(
        "\tworst-dist vertex {}: pos ({:.6}, {:.6}, {:.6}) dist {:.6} target {:.6} err {:.6}",
        vid,
        p[0],
        p[1],
        p[2],
        d,
        m_offset_params.target_distance,
        std::abs(d - m_offset_params.target_distance));
    logger().info(
        "\t  flags: on_offset {} on_input {} on_region {} on_bbox {} rounded {} | held {} "
        "| incident faces: {} offset, {} region, {} bbox | phi {:.6} (level {:.6}), "
        "residual {:.6}, containment envelope {}",
        ve.m_is_on_offset,
        ve.m_is_on_input,
        ve.m_is_on_region,
        !m_vertex_attribute[vid].on_bbox_faces.empty(),
        m_vertex_attribute[vid].m_is_rounded,
        vertex_is_held(vid),
        n_offset_f,
        n_region_f,
        n_bbox_f,
        potential_for(vid).value(p),
        potential_for(vid).target_level(),
        potential_for(vid).residual_length(p),
        smoothing_containment_envelope(vid) ? "yes" : "none");
    const char* fate = "TetWild's smooth_after(): AMIPS, no offset term";
    if (!m_vertex_attribute[vid].m_is_rounded) {
        fate = "REFUSED by smooth_before: not rounded";
    } else if (m_tetwild_pass && ve.m_is_on_offset) {
        fate =
            "a TetWild pass: TetWild's smooth_after() (raw AMIPS sum), held in the offset envelope";
    } else if (ve.m_is_on_offset) {
        fate = "the front smoother: AMIPS plus the offset terms (smooth_front_vertex)";
    }
    logger().info("\t  smoothing fate: {}", fate);

    const auto tags_to_string = [](const CellTag& tags) {
        std::string s = "{";
        for (const int64_t t : tags) {
            if (s.size() > 1) s += ",";
            s += std::to_string(t);
        }
        return s + "}";
    };
    seen_fids.clear();
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        const auto tet_vids = oriented_tet_vids(tid);
        for (int skip = 0; skip < 4; ++skip) {
            if (tet_vids[size_t(skip)] == vid) continue;
            const auto [ft, fid] = tuple_from_face(face_corners_from(tet_vids, skip));
            if (!seen_fids.insert(fid).second) continue;
            const char* cls = "untracked";
            if (m_face_attribute[fid].m_is_surface_fs) {
                cls = m_face_attribute[fid].m_surface_class == OFFSET_SURFACE_CLASS ? "OFFSET"
                                                                                    : "REGION";
            } else if (m_face_attribute[fid].m_is_bbox_fs >= 0) {
                cls = "bbox";
            }
            const auto fv = get_face_vids(ft);
            const std::optional<Tuple> opp = ft.switch_tetrahedron(*this);
            if (!opp) {
                logger().info(
                    "\t  face [{}, {}, {}]: class {} (domain boundary, one tet)",
                    fv[0],
                    fv[1],
                    fv[2],
                    cls);
                continue;
            }
            const size_t ta = ft.tid(*this), tb = opp->tid(*this);
            logger().info(
                "\t  face [{}, {}, {}]: class {} | tets {} tags {} label {} band {} | {} tags {} "
                "label {} band {}",
                fv[0],
                fv[1],
                fv[2],
                cls,
                ta,
                tags_to_string(m_tet_attribute[ta].tag),
                m_tet_attribute[ta].label,
                cell_is_offset_band(ta),
                tb,
                tags_to_string(m_tet_attribute[tb].tag),
                m_tet_attribute[tb].label,
                cell_is_offset_band(tb));
        }
    }
}

bool TopoOffsetTetMesh::face_is_offset_surface_live(const Tuple& f) const
{
    const size_t ta = f.tid(*this);
    // Defence in depth behind the two walks that feed this: a default-constructed Tuple carries
    // m_global_tid == size_t(-1), which is the same sentinel TetMesh::Tuple::is_valid() tests
    // first, and switch_tetrahedron() below would index m_tet_connectivity with it. Refusing it
    // here means a caller that forgets to check a lookup gets `false`, not a wild read.
    if (ta == std::numeric_limits<size_t>::max()) {
        ++m_offset_face_invalid_tuple;
        return false;
    }
    const std::optional<Tuple> opp = f.switch_tetrahedron(*this);
    if (!opp) {
        // Domain boundary. A band cell here means the band was clipped by the bounding box, and
        // that face is offset surface -- its vertices can never reach the target distance, which
        // is precisely the thing that must be measured rather than hidden.
        return cell_is_offset_band(ta);
    }
    const size_t tb = opp->tid(*this);
    const bool a = cell_is_offset_band(ta), b = cell_is_offset_band(tb);
    if (a == b) return false; // both in the band, or neither: not the band's surface
    // The band's inner interface, against the input complex it wraps, sits at distance 0 by
    // construction and would drag the reported error to target_distance everywhere.
    return !cell_is_input_complex(a ? tb : ta);
}

bool TopoOffsetTetMesh::offset_face_normal_out_of_band(const Tuple& f, Vector3d& n) const
{
    // See the declaration. Ported unchanged from the fold work (wmtk-pinned-split, 2026-10-24).
    const auto o = f.switch_tetrahedron(*this);
    if (!o) return false;
    const size_t t0 = f.tid(*this), t1 = o->tid(*this);
    const size_t tb = cell_is_offset_band(t0) ? t0 : cell_is_offset_band(t1) ? t1 : size_t(-1);
    if (tb == size_t(-1)) return false;
    const std::array<size_t, 3> fv = get_face_vids(f);
    size_t opp = 0;
    for (const size_t v : oriented_tet_vids(tb)) {
        if (v != fv[0] && v != fv[1] && v != fv[2]) opp = v;
    }
    const Vector3d& p0 = m_vertex_attribute[fv[0]].m_posf;
    n = (m_vertex_attribute[fv[1]].m_posf - p0).cross(m_vertex_attribute[fv[2]].m_posf - p0);
    if (!(n.norm() > 0)) return false;
    n.normalize();
    if (n.dot(p0 - m_vertex_attribute[opp].m_posf) < 0) n = -n;
    return true;
}

size_t TopoOffsetTetMesh::count_real_folds(std::vector<char>* corners) const
{
    // Each live offset face once, its normal out of the band; then the edges with exactly two.
    // Ported unchanged from the fold work (wmtk-pinned-split, 2026-10-24).
    std::map<std::array<size_t, 2>, std::vector<std::pair<std::array<size_t, 3>, Vector3d>>>
        by_edge;
    size_t vmax = 0;
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        Vector3d n;
        if (!offset_face_normal_out_of_band(f, n)) continue;
        const std::array<size_t, 3> fv = get_face_vids(f);
        for (int i = 0; i < 3; ++i) {
            const size_t a = fv[i], b = fv[(i + 1) % 3];
            by_edge[{{std::min(a, b), std::max(a, b)}}].push_back({fv, n});
            vmax = std::max(vmax, std::max(a, b));
        }
    }
    if (corners) corners->assign(vmax + 1, 0);
    size_t folds = 0;
    for (const auto& [e, fs] : by_edge) {
        if (fs.size() != 2 || fs[0].second.dot(fs[1].second) >= kCos150) continue;
        ++folds;
        if (!corners) continue;
        for (const auto& [fv, n] : fs) {
            for (const size_t v : fv) (*corners)[v] = 1;
        }
    }
    return folds;
}

bool TopoOffsetTetMesh::edge_is_offset_surface_live(const size_t a, const size_t b) const
{
    for (const size_t tid : get_incident_tids_for_edge(a, b)) {
        const auto vs = oriented_tet_vids(tid);
        for (const size_t c : vs) {
            if (c == a || c == b) continue;
            const auto found = try_tuple_from_face({{a, b, c}});
            if (found && face_is_offset_surface_live(std::get<0>(*found))) return true;
        }
    }
    return false;
}

std::vector<std::array<size_t, 3>> TopoOffsetTetMesh::offset_surface_faces() const
{
    std::set<std::array<size_t, 3>> faces;
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        const auto vs = get_face_vids(f);
        std::array<size_t, 3> t{{vs[0], vs[1], vs[2]}};
        std::sort(t.begin(), t.end());
        faces.insert(t);
    }
    return std::vector<std::array<size_t, 3>>(faces.begin(), faces.end());
}

std::vector<TopoOffsetTetMesh::Tuple> TopoOffsetTetMesh::offset_surface_faces_live_at(
    const size_t vid) const
{
    std::vector<Tuple> result;
    std::set<size_t> seen;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        const auto tv = oriented_tet_vids(tid);
        for (int skip = 0; skip < 4; ++skip) {
            if (tv[size_t(skip)] == vid) continue;
            // try_ rather than the asserting tuple_from_face: see the note in
            // vertex_has_live_offset_face() below and m_offset_face_lookup_misses.
            const auto found = try_tuple_from_face(face_corners_from(tv, skip));
            if (!found) {
                ++m_offset_face_lookup_misses;
                continue;
            }
            const auto& [ft, fid] = *found;
            if (!seen.insert(fid).second) continue;
            if (face_is_offset_surface_live(ft)) result.push_back(ft);
        }
    }
    return result;
}

template <typename F>
bool TopoOffsetTetMesh::for_each_live_offset_face_at(const size_t vid, F&& fn) const
{
    // Reads vid's own tet list and the tets in it, nothing else. Every face through vid is shared
    // by at most two tets, and both contain vid, so both are in vid's list: pairing the faces
    // within the list finds the tet across each one without looking up any other vertex's list.
    // A face with one tet in the list is on the domain boundary. The verdict per face is
    // face_is_offset_surface_live()'s: live when exactly one side is band and the other is not
    // the input complex, or when a band tet's face is on the domain boundary.
    //
    // Why not try_tuple_from_face(): it reads the tet lists of the face's other two corners,
    // which for a vertex of a collapsed edge's link lie two steps from the edge. The collapse
    // pass claims only about 2-ring(v1) u N(v2), by design (TetMesh.h, "Ring lockers -- NOT
    // balls"), so those lists could belong to another thread's collapse: ThreadSanitizer,
    // 2026-09-30, 26 of 36 races on the cube at AMIPS weight 1e-4 and 22 of 47 at w = 1, and a
    // segfault in about 1 run in 30 at 10 threads. vid's own list is always claimed.
    const std::vector<size_t>& ring = get_one_ring_tids_for_vertex(vid);
    struct Side
    {
        std::array<size_t, 3> face;
        size_t tid;
    };
    std::vector<Side> sides;
    sides.reserve(3 * ring.size());
    for (const size_t tid : ring) {
        const auto tv = oriented_tet_vids(tid);
        for (int skip = 0; skip < 4; ++skip) {
            if (tv[size_t(skip)] == vid) continue; // the face opposite vid does not contain it
            std::array<size_t, 3> f = face_corners_from(tv, skip);
            std::sort(f.begin(), f.end());
            sides.push_back({f, tid});
        }
    }
    std::sort(sides.begin(), sides.end(), [](const Side& a, const Side& b) {
        return a.face != b.face ? a.face < b.face : a.tid < b.tid;
    });
    for (size_t i = 0; i < sides.size();) {
        size_t j = i + 1;
        while (j < sides.size() && sides[j].face == sides[i].face) ++j;
        const size_t ta = sides[i].tid;
        if (j - i == 1) {
            // a band face on the domain boundary
            if (cell_is_offset_band(ta) && fn(sides[i].face, ta)) return true;
        } else {
            const size_t tb = sides[i + 1].tid;
            const bool a = cell_is_offset_band(ta), b = cell_is_offset_band(tb);
            if (a != b && !cell_is_input_complex(a ? tb : ta) && fn(sides[i].face, a ? ta : tb)) {
                return true;
            }
        }
        i = j;
    }
    return false;
}

bool TopoOffsetTetMesh::vertex_has_live_offset_face(const size_t vid) const
{
    // See for_each_live_offset_face_at(): any face at all.
    return for_each_live_offset_face_at(vid, [](const std::array<size_t, 3>&, size_t) {
        return true;
    });
}

void TopoOffsetTetMesh::folded_front_faces_at(
    const size_t vid,
    std::set<std::array<size_t, 3>>& out) const
{
    // See the declaration.
    for_each_live_offset_face_at(vid, [&](const std::array<size_t, 3>& f, const size_t band) {
        if (front_face_field_angle_in(f[0], f[1], f[2], band) > m_offset_params.foldover_angle) {
            out.insert(f);
        }
        return false;
    });
}

bool TopoOffsetTetMesh::face_folded_in(
    const size_t a,
    const size_t b,
    const size_t c,
    const std::vector<size_t>& cells) const
{
    // See the declaration.
    for (const size_t t : cells) {
        if (!cell_is_offset_band(t)) continue;
        const auto tv = oriented_tet_vids(t);
        const auto has = [&](const size_t v) {
            return std::find(tv.begin(), tv.end(), v) != tv.end();
        };
        if (!has(a) || !has(b) || !has(c)) continue;
        return front_face_field_angle_in(a, b, c, t) > m_offset_params.foldover_angle;
    }
    return false;
}

void TopoOffsetTetMesh::refresh_offset_membership(const size_t vid)
{
    m_vertex_extra[vid].m_is_on_offset = vertex_has_live_offset_face(vid);
}

std::pair<size_t, size_t> TopoOffsetTetMesh::offset_membership_mismatches() const
{
    size_t flagged_not_live = 0, live_not_flagged = 0;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        const bool live = vertex_has_live_offset_face(vid);
        const bool flag = m_vertex_extra[vid].m_is_on_offset;
        if (flag && !live) ++flagged_not_live;
        if (live && !flag) ++live_not_flagged;
    }
    return {flagged_not_live, live_not_flagged};
}

void TopoOffsetTetMesh::check_offset_membership(const char* when) const
{
    if (!m_params.perform_sanity_checks) return;
    const auto [flagged_not_live, live_not_flagged] = offset_membership_mismatches();
    logger().info(
        "\t[sanity] offset membership @ {}: flagged but not on the surface {}, on the surface but "
        "not flagged {}",
        when,
        flagged_not_live,
        live_not_flagged);
    if (flagged_not_live != 0 || live_not_flagged != 0) {
        log_and_throw_error(
            "offset membership is out of step @ {}: {} vertices carry m_is_on_offset with no live "
            "offset-surface face, {} have one without the flag. The propagation in "
            "split_after_cells / collapse_after_vertex / swap_after_cells missed a case.",
            when,
            flagged_not_live,
            live_not_flagged);
    }
}

void TopoOffsetTetMesh::report_offset_face_lookup_misses(const char* when) const
{
    const long long misses = m_offset_face_lookup_misses.load();
    const long long invalid = m_offset_face_invalid_tuple.load();
    if (misses == 0 && invalid == 0) return;
    logger().warn(
        "\t[offset face lookup] {}: {} face(s) asked for by offset_surface_faces_live_at() were "
        "not in the connectivity, {} invalid tuple(s) refused by face_is_offset_surface_live (run "
        "totals). That walk reads past what the pass locks, so at num_threads > 0 this is a stale "
        "read. See m_offset_face_lookup_misses.",
        when,
        misses,
        invalid);
}

bool TopoOffsetTetMesh::front_face_out_normal(
    const size_t a,
    const size_t b,
    const size_t c,
    Vector3d& n) const
{
    // See the declaration.
    const int64_t bc = front_face_band_cell(a, b, c);
    return bc >= 0 && front_face_out_normal_in(a, b, c, size_t(bc), n);
}

bool TopoOffsetTetMesh::front_face_out_normal_in(
    const size_t a,
    const size_t b,
    const size_t c,
    const size_t band_tid,
    Vector3d& n) const
{
    // See the declaration.
    const Vector3d& pa = m_vertex_attribute[a].m_posf;
    const Vector3d& pb = m_vertex_attribute[b].m_posf;
    const Vector3d& pc = m_vertex_attribute[c].m_posf;
    n = (pb - pa).cross(pc - pa);
    if (!(n.norm() > 0.)) return false;
    for (const size_t v : oriented_tet_vids(band_tid)) {
        if (v == a || v == b || v == c) continue;
        if (n.dot(pa - m_vertex_attribute[v].m_posf) < 0.) n = -n;
    }
    return true;
}

double TopoOffsetTetMesh::front_face_field_angle(const size_t a, const size_t b, const size_t c)
    const
{
    // See the declaration.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (!m_input_complex_envelope) return nan;
    Vector3d n;
    if (!front_face_out_normal(a, b, c, n)) return nan;
    return front_face_angle_to_field(a, b, c, n);
}

double TopoOffsetTetMesh::front_face_field_angle_in(
    const size_t a,
    const size_t b,
    const size_t c,
    const size_t band_tid) const
{
    // See the declaration.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (!m_input_complex_envelope) return nan;
    Vector3d n;
    if (!front_face_out_normal_in(a, b, c, band_tid, n)) return nan;
    return front_face_angle_to_field(a, b, c, n);
}

double TopoOffsetTetMesh::front_face_angle_to_field(
    const size_t a,
    const size_t b,
    const size_t c,
    const Vector3d& n) const
{
    // See the declaration.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (!m_input_complex_envelope) return nan;
    const Vector3d& pa = m_vertex_attribute[a].m_posf;
    const Vector3d& pb = m_vertex_attribute[b].m_posf;
    const Vector3d& pc = m_vertex_attribute[c].m_posf;
    const Vector3d x = (pa + pb + pc) / 3.;
    const Envelope& env = *m_input_complex_envelope;
    const int64_t prim = nearest_primitive(env, m_input_complex_pad, x);
    if (prim < 0) return nan;
    Vector3d g;
    env.primitive_distance(prim, x, &g);
    if (!(g.norm() > 0.)) return nan;
    // atan2 of the cross and dot products: accurate at every angle, unlike acos near 0 and 180.
    return std::atan2(n.cross(g).norm(), n.dot(g)) * 180. / M_PI;
}

TopoOffsetTetMesh::FoldCensus TopoOffsetTetMesh::fold_census() const
{
    // See the declaration.
    FoldCensus c;
    for (const auto& f : offset_surface_faces()) {
        ++c.n_faces;
        const double ang = front_face_field_angle(f[0], f[1], f[2]);
        if (!std::isfinite(ang)) {
            ++c.n_unmeasured;
            continue;
        }
        if (!(ang > m_offset_params.foldover_angle)) continue;
        ++c.n_fold;
        if (ang > c.worst) {
            c.worst = ang;
            c.worst_at = (m_vertex_attribute[f[0]].m_posf + m_vertex_attribute[f[1]].m_posf +
                          m_vertex_attribute[f[2]].m_posf) /
                         3.;
        }
    }
    return c;
}

const OffsetPotential3D& TopoOffsetTetMesh::potential_for_face(const Tuple& f) const
{
    const size_t ta = f.tid(*this);
    const std::optional<Tuple> opp = f.switch_tetrahedron(*this);
    size_t band = ta;
    if (!cell_is_offset_band(ta) && opp && cell_is_offset_band(opp->tid(*this)))
        band = opp->tid(*this);
    return potential_for_region(band < m_cell_region.size() ? m_cell_region[band] : -1);
}

void TopoOffsetTetMesh::check_no_vertex_on_both_surfaces(const char* when) const
{
    // A vertex on both surfaces is unsatisfiable: at distance 0 from the input complex and
    // required to sit at target_distance from it. The geometry decides, not the flags, which are
    // over-broad. As in 2D.
    std::vector<size_t> both;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!m_vertex_extra[vid].m_is_on_offset || !m_vertex_extra[vid].m_is_on_input) {
            continue;
        }
        if (input_complex_distance(m_vertex_attribute[vid].m_posf) >
            m_offset_params.envelope_size) {
            continue;
        }
        both.push_back(vid);
    }
    if (both.empty()) {
        return;
    }

    const size_t n_show = std::min<size_t>(both.size(), 8);
    std::string detail;
    for (size_t i = 0; i < n_show; ++i) {
        const size_t vid = both[i];
        const double d = input_complex_distance(m_vertex_attribute[vid].m_posf);
        detail += fmt::format("{}{} (dist to input {:.6g})", i ? ", " : "", vid, d);
    }
    log_and_throw_error(
        "[{}] {} vertices are on BOTH the input complex and the offset surface. Such a vertex is "
        "at distance 0 from the input and is asked to be at target_distance {} from it at the "
        "same time, so the optimization cannot place it and the offset surface through it cannot "
        "converge. This is a construction defect, not an optimization failure. Offending "
        "vertices: {}{}",
        when,
        both.size(),
        m_offset_params.target_distance,
        detail,
        both.size() > n_show ? ", ..." : "");
}

void TopoOffsetTetMesh::build_offset_envelope()
{
    // See m_offset_envelope: the front as the loop left it, a closed manifold surface by now,
    // so it is one triangle set and no face of it is shared with anything else.
    std::vector<Eigen::Vector3i> tris;
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        const auto vs = get_face_vids(f);
        tris.emplace_back(int(vs[0]), int(vs[1]), int(vs[2]));
    }
    if (tris.empty()) {
        m_offset_envelope = nullptr;
        logger().warn("\t[offset envelope] no offset-surface faces; the envelope is empty");
        return;
    }

    std::vector<Eigen::Vector3d> verts(vert_capacity());
    for (size_t i = 0; i < vert_capacity(); ++i) {
        verts[i] = m_vertex_attribute[i].m_posf;
    }

    // envelope_size, as init() resolved it: the eps of every held surface's envelope, the offset
    // surface's too (its own key, offset_envelope, went on 2026-10-20).
    // optimize_construction's pass alone holds the front in a wider envelope, half the distance
    // the march traced it to (m_construction_envelope_eps, 0 at every other time).
    const bool construction_envelope = m_construction_envelope_eps > 0.;
    const double eps = construction_envelope ? m_construction_envelope_eps
                                             : std::max(m_offset_params.envelope_size, 1e-12);

    m_offset_envelope = std::make_shared<Envelope>(/*exact=*/true);
    m_offset_envelope->init(verts, tris, eps);
    logger().info(
        "\t[offset envelope] built: {} faces, {} (eps {:.6g} = {})",
        tris.size(),
        m_offset_envelope->use_exact ? "EXACT" : "sampled",
        eps,
        construction_envelope
            ? std::string("half the traced distance, optimize_construction's pass only")
            : fmt::format(
                  "envelope_size, {:.4} x the bbox diagonal",
                  m_offset_params.envelope_size_rel));
}

namespace {
/// Cheap existence test for a companion frame; <filesystem> is not used in this component.
bool debug_frame_file_exists(const std::string& p)
{
    std::ifstream f(p);
    return f.good();
}

/// DEBUG_output only. Splice a VTK FieldData string array carrying this frame's pass label into
/// a .vtu paraviewo has just closed, so ParaView can display it per timestep: add an Annotate
/// Attribute Data filter, association Field Data, array frame_label.
///
/// Why splice rather than write it properly: paraviewo's VTUWriter exposes only numeric point
/// and cell fields (Eigen::MatrixXd), with no FieldData and no string support, and it is a
/// third-party dependency outside this component. The .vtu is XML, so the block goes in here.
/// format="ascii" keeps it out of the appended-data section, so the binary offsets paraviewo
/// already wrote stay valid. VTK encodes a string as its character codes, space separated and
/// null terminated, which also makes the label XML-safe whatever it contains.
///
/// The <UnstructuredGrid> anchor sits in the first few hundred bytes, so only the head is held
/// in memory and the body -- tens of megabytes on a large frame -- is streamed through.
bool inject_frame_label(const std::string& path, const std::string& label)
{
    static const std::string anchor = "<UnstructuredGrid>";
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    std::string head(4096, '\0');
    in.read(&head[0], static_cast<std::streamsize>(head.size()));
    head.resize(static_cast<size_t>(in.gcount()));
    const size_t at = head.find(anchor);
    if (at == std::string::npos) return false;
    const size_t cut = at + anchor.size();

    std::string codes;
    for (const char c : label) {
        codes += std::to_string(static_cast<unsigned>(static_cast<unsigned char>(c))) + " ";
    }
    codes += "0";

    const std::string tmp = path + ".lbl";
    {
        std::ofstream out(tmp, std::ios::binary);
        if (!out) return false;
        out.write(head.data(), static_cast<std::streamsize>(cut));
        out << "\n  <FieldData>\n    <Array type=\"String\" Name=\"frame_label\" "
               "NumberOfTuples=\"1\" format=\"ascii\">\n      "
            << codes << "\n    </Array>\n  </FieldData>";
        out.write(head.data() + cut, static_cast<std::streamsize>(head.size() - cut));
        std::vector<char> buf(size_t(1) << 16);
        while (in.read(buf.data(), static_cast<std::streamsize>(buf.size())) || in.gcount() > 0) {
            out.write(buf.data(), in.gcount());
        }
        if (!out) return false;
    }
    in.close();
    if (std::rename(tmp.c_str(), path.c_str()) == 0) return true;
    std::remove(tmp.c_str()); // leave the frame paraviewo wrote rather than a half-named file
    return false;
}
} // namespace

void TopoOffsetTetMesh::append_frame_label(const size_t idx, const std::string& label) const
{
    std::ofstream f(
        m_offset_params.output_path + "_frames.txt",
        idx == 0 ? std::ios::trunc : std::ios::app);
    if (f) f << fmt::format("{:05d}\t{}\n", idx, label);
}

void TopoOffsetTetMesh::write_debug_frame(const std::string& label)
{
    const size_t idx = m_debug_seq++;
    append_frame_label(idx, label);
    if (m_offset_potential) {
        // E (the AMIPS term) at this frame (see global_energy()), one line per frame. The first
        // column after the label, once the band term, is 0 so older readers keep their columns.
        double amips = 0.;
        for (const Tuple& t : get_tets()) {
            const auto vids = oriented_tet_vids(t.tid(*this));
            const double amips3 = get_quality(t);
            amips +=
                amips3 >= MAX_ENERGY ? MAX_ENERGY : amips_term_weight() * vol_amips3(vids, amips3);
        }
        std::ofstream f(
            m_offset_params.output_path + "_energy.txt",
            idx == 0 ? std::ios::trunc : std::ios::app);
        if (f) f << fmt::format("{:05d}\t{}\t{:.17g}\t{:.17g}\n", idx, label, 0., amips);
    }
    const std::string base = m_offset_params.output_path + fmt::format("_{:05d}", idx);
    // The mesh is read here, on the loop's thread; the four files are encoded and written on a
    // background thread from the gathered arrays alone, while the loop goes on. The loop waits
    // only when the previous frame's files are still being written. MEASURED 2026-10-09, cube,
    // 3 turns: writing frames serially was 27% of the wall time at 10 threads (3.2 of 12.0 s) and
    // 10% serial, the writing being serial while the passes are parallel.
    VtuFrame frame = gather_vtu_frame(base);
    m_front_solve_log.clear(); // gathered; the next frame shows the solves after this one
    wait_debug_frame_write();
    if (m_debug_timeline.labels.size() <= idx) m_debug_timeline.labels.resize(idx + 1);
    m_debug_timeline.labels[idx] = label;
    m_debug_frame_write = std::async(
        std::launch::async,
        [frame = std::move(frame),
         timeline = std::move(m_debug_timeline),
         out = m_offset_params.output_path,
         base,
         label,
         idx]() mutable {
            write_vtu_frame(frame);
            // Record what this frame actually wrote, then refresh the ParaView collections.
            // Which companions exist is dimension-specific and some are conditional, so they are
            // discovered from disk rather than hard-coded here.
            for (const char* sfx : {"", "_surf", "_off", "_edge", "_front"}) {
                const std::string p = base + sfx + ".vtu";
                if (!debug_frame_file_exists(p)) continue;
                // The label goes INTO the frame as FieldData, not only into the .pvd: a .pvd
                // DataSet's name= attribute does reach the reader, but as a vtkCharArray, which
                // ParaView's annotation renders as the first character's numeric code rather than
                // the text, and an XML comment is discarded outright. See inject_frame_label().
                inject_frame_label(p, label);
                timeline.pvd_series[sfx].push_back(idx);
            }
            write_debug_pvd(out, timeline);
            return std::move(timeline);
        });
}

void TopoOffsetTetMesh::wait_debug_frame_write()
{
    if (m_debug_frame_write.valid()) m_debug_timeline = m_debug_frame_write.get();
}

void TopoOffsetTetMesh::write_debug_pvd(const std::string& out, const DebugTimeline& timeline)
{
    // DEBUG_output only. ParaView detects a file series only when the frame index sits
    // IMMEDIATELY before the extension. The main frames are <output>_NNNNN.vtu and group fine,
    // but every companion is <output>_NNNNN_off.vtu -- index in the middle, suffix after it --
    // so ParaView opens each companion as its own dataset instead of one time series. A .pvd
    // collection names the files explicitly, which sidesteps the naming rule entirely.
    // Rewritten after EVERY frame, not once at the end: these runs are killed often, and a
    // killed run should still leave a series that opens.
    // file= is resolved relative to the .pvd, so it carries the bare name, not output_path.
    const std::string stem = out.substr(out.find_last_of("/\\") + 1);
    for (const auto& [sfx, idxs] : timeline.pvd_series) {
        if (idxs.size() < 2) continue; // a single frame is not a series
        std::ofstream f(out + (sfx.empty() ? std::string("_main") : sfx) + ".pvd", std::ios::trunc);
        if (!f) continue;
        f << "<?xml version=\"1.0\"?>\n"
             "<VTKFile type=\"Collection\" version=\"0.1\" byte_order=\"LittleEndian\">\n"
             "  <Collection>\n";
        for (const size_t i : idxs) {
            f << fmt::format(
                "    <DataSet timestep=\"{}\" group=\"\" part=\"0\" file=\"{}_{:05d}{}.vtu\"/>",
                i,
                stem,
                i,
                sfx);
            // The frame's label, so the .pvd also says which pass produced each timestep. A
            // label containing "--" would close the XML comment early, so it is left out.
            const std::string lab = i < timeline.labels.size() ? timeline.labels[i] : "";
            if (!lab.empty() && lab.find("--") == std::string::npos) {
                f << fmt::format("  <!-- {} -->", lab);
            }
            f << "\n";
        }
        f << "  </Collection>\n</VTKFile>\n";
    }
}

void TopoOffsetTetMesh::loop_group(const std::array<int, 4>& ops, const std::string& where)
{
    // See the declaration. Off: the engine's group as before.
    if (!m_offset_params.front_pocket_split) {
        local_operations(ops);
        return;
    }
    // EXPERIMENTAL_front_pocket_split: the same passes, one call for the operation passes and one
    // per smoothing pass, as repulsion_rounds runs its groups. Besides the splits this moves the
    // rounding sweep from after the k-th smoothing pass to after each, and adds each call's energy
    // and time lines. Each smoothing pass smooths the vertices of the turn's block (m_smooth_set).
    if (ops[0] > 0 || ops[1] > 0 || ops[2] > 0) local_operations({{ops[0], ops[1], ops[2], 0}});
    // EXPERIMENTAL_label_cleanup: after the operation passes (none in pre_smooth), before the
    // condition-1 splits of the first smoothing pass. A flip with f = 1 or 2 faces on the other
    // label leaves the cell with all four corners on the front -- a pinned cell, which those
    // splits handle -- and one with f = 3 removes a pinned cell (the flipped cell had all four
    // corners on the front; afterwards three); the smoothing pass then moves the new front in the
    // same turn. Under interleaved_smoothing this is once per group, else once per turn (per
    // block).
    if (m_offset_params.label_cleanup != "off" && (ops[0] > 0 || ops[1] > 0 || ops[2] > 0)) {
        label_cleanup(where);
        // DEBUG_output: the clean-up's own frame, so its flips are seen apart from the condition-1
        // splits and the smoothing pass that follow it.
        if (m_offset_params.debug_output) {
            const std::string name = m_debug_pass_name;
            m_debug_pass_name = "label_cleanup";
            write_optimization_debug_output("debug_label_cleanup");
            m_debug_pass_name = name;
        }
    }
    for (int p = 0; p < ops[3]; ++p) {
        front_pocket_split(fmt::format("{}, before smoothing pass {}", where, p + 1));
        local_operations({{0, 0, 0, 1}});
    }
}

bool TopoOffsetTetMesh::trace_collapse_begin(const Tuple& t)
{
    const auto& ids = m_offset_params.debug_trace_collapse;
    if (ids.empty()) return false;
    if (m_trace_collapse.open) trace_collapse_end("left open (refused before)");
    const size_t v1 = t.vid(*this), v2 = t.switch_vertex(*this).vid(*this);
    if (std::find(ids.begin(), ids.end(), v1) == ids.end() &&
        std::find(ids.begin(), ids.end(), v2) == ids.end()) {
        return false;
    }
    TraceCollapse& a = m_trace_collapse;
    a = TraceCollapse();
    a.open = true;
    a.v1 = v1;
    a.v2 = v2;
    a.length = (m_vertex_attribute[v1].m_posf - m_vertex_attribute[v2].m_posf).norm();
    for (size_t r = 0; r < a.before.size(); ++r) a.before[r] = m_collapse_reject[r].load();
    return true;
}

void TopoOffsetTetMesh::trace_collapse_end(const char* outcome)
{
    TraceCollapse& a = m_trace_collapse;
    if (!a.open) return;
    a.open = false;
    std::string reasons;
    for (size_t r = 0; r < a.before.size(); ++r) {
        const long n = m_collapse_reject[r].load() - a.before[r];
        if (n > 0) reasons += fmt::format(" {} {}", collapse_reject_name(CollapseReject(r)), n);
    }
    logger().info(
        "\t[trace collapse] turn {}{}: {} -> {} ({} -> {}) length {:.6g} | {} |{}{}",
        m_round,
        dry_run() ? " (dry run)" : "",
        a.v1,
        a.v2,
        m_vertex_extra[a.v1].m_is_on_offset ? "front" : "off front",
        m_vertex_extra[a.v2].m_is_on_offset ? "front" : "off front",
        a.length,
        outcome,
        reasons.empty() ? std::string(" no counter raised") : reasons,
        a.energy ? fmt::format(
                       " | front max rule: stencil-point offset max {:.6g} -> {:.6g} (bound "
                       "{:.6g}), AMIPS^3 max {:.6g} -> {:.6g} (bound {:.6g})",
                       a.e_before,
                       a.e_after,
                       front_offset_bar(),
                       a.a_before,
                       a.a_after,
                       front_amips_bar())
                 : std::string());
}

void TopoOffsetTetMesh::front_pocket_split(const std::string& where)
{
    // loop_group() calls this only with the key on; the key is tested here too so that the slot
    // alone is a no-op with it off (the unit test drives the slot directly).
    if (!m_offset_params.front_pocket_split || m_tetwild_pass) return;
    const auto len2 = [this](const std::array<size_t, 2>& e) {
        return (m_vertex_attribute[e[0]].m_posf - m_vertex_attribute[e[1]].m_posf).squaredNorm();
    };
    // A pinned cell: its corners sorted (it names the cell across the round's splits), the edges
    // to split it on, longest first (ties by vertex ids), and whether all six of its edges are on
    // the offset surface.
    struct Pinned
    {
        std::array<size_t, 4> vids;
        std::vector<std::array<size_t, 2>> edges;
        bool six = false;
    };
    // One scan of the live cells. `no_face`: pinned cells with all four faces on the offset
    // surface, which have no edge to split; `six` and `band`, when given: pinned cells with all
    // six edges on it, and band cells with all four corners on the front.
    const auto scan = [&](size_t& no_face, size_t* six, size_t* band) {
        std::vector<Pinned> out;
        no_face = 0;
        for (const Tuple& t : get_tets()) {
            const size_t tid = t.tid(*this);
            const std::array<size_t, 4> vs = oriented_tet_vids(tid);
            if (!std::all_of(vs.begin(), vs.end(), [this](const size_t v) {
                    return m_vertex_extra[v].m_is_on_offset;
                })) {
                continue;
            }
            if (cell_is_input_complex(tid)) continue;
            // A band cell with all four corners on the front is the mirror case: at the band
            // term's optimum its corners lie on the level set too, so it is the hull of four
            // points on it -- a sliver, flattened outright where the curvature puts that hull on
            // the background's side. Split the same way (an edge on no offset face is inside the
            // band, so its midpoint is off the front); counted apart in the log.
            if (cell_is_offset_band(tid) && band) ++*band;
            Pinned p;
            p.vids = vs;
            std::sort(p.vids.begin(), p.vids.end());
            for (int i = 0; i < 4; ++i) {
                for (int j = i + 1; j < 4; ++j) {
                    const std::array<size_t, 2> e{{p.vids[i], p.vids[j]}};
                    if (!edge_is_offset_surface_live(e[0], e[1])) p.edges.push_back(e);
                }
            }
            if (p.edges.empty()) {
                // All six edges on the offset surface, so every edge midpoint would be on the
                // front. Then split the edges of the cell's faces that are not offset faces,
                // longest first: such a face has its three edges on the offset surface, and
                // splitting one, (a, b) of face (a, b, c), at m makes the edge (m, c), whose
                // faces are the two halves of (a, b, c), between the same two labels as it, and
                // one new face inside each cell that shared it, so it is off the offset surface.
                // Both children of this cell have it, and the next round splits it as above: a
                // vertex off the front at (a + b + 2c) / 4 on that face, a corner of all four of
                // the cell's descendants. That is simplicial_embedding()'s face split, made of
                // the loop's own edge splits; the component's face split hooks are the
                // construction's (they write no m_is_on_offset, no rest, no exact fallback).
                // A cell with all four faces on the offset surface is a one-cell component of
                // its label; a split would leave its boundary, every corner of it on the front,
                // as it is, so it is counted only. MEASURED before this (cube_in, target 0.3,
                // 7 turns): 176 such cells over the passes (31 distinct; 102 band, 74 not), every
                // one with exactly three faces on the offset surface, their shared corner on
                // exactly three offset faces and in no other cell of the cell's label -- a cell
                // alone under a front vertex of valence 3 -- and all 176 left unsplit.
                p.six = true;
                if (six) ++*six;
                for (int j = 0; j < 4; ++j) {
                    const Tuple f = tuple_from_face(tid, j);
                    if (face_is_offset_surface_live(f)) continue;
                    const std::array<size_t, 3> fv = get_face_vids(f);
                    for (int i = 0; i < 3; ++i) {
                        const size_t u = fv[i], w = fv[(i + 1) % 3];
                        const std::array<size_t, 2> e{{std::min(u, w), std::max(u, w)}};
                        if (std::find(p.edges.begin(), p.edges.end(), e) == p.edges.end()) {
                            p.edges.push_back(e);
                        }
                    }
                }
                if (p.edges.empty()) {
                    ++no_face;
                    continue;
                }
            }
            std::sort(p.edges.begin(), p.edges.end(), [&](const auto& a, const auto& b) {
                const double la = len2(a), lb = len2(b);
                return la != lb ? la > lb : a < b;
            });
            out.push_back(std::move(p));
        }
        return out;
    };
    // Whether the cell on these sorted corners is still live: an earlier split of the round may
    // have split one of its edges, and so the cell, already.
    const auto alive = [this](const std::array<size_t, 4>& s) {
        for (const size_t tid : get_incident_tids_for_edge(s[0], s[1])) {
            std::array<size_t, 4> vs = oriented_tet_vids(tid);
            std::sort(vs.begin(), vs.end());
            if (vs == s) return true;
        }
        return false;
    };

    // EXPERIMENTAL_front_spanned_split: the spanned edges and faces (see the spec), split each
    // round before the pinned cells, lowest dimension first. A face is listed only when its three
    // edges are on the offset surface; one with an edge off it is split by that edge's split.
    const auto on_front = [this](const size_t v) { return m_vertex_extra[v].m_is_on_offset; };
    const auto off_input = [this](const std::vector<size_t>& tids) {
        return !tids.empty() && std::none_of(tids.begin(), tids.end(), [this](const size_t t) {
            return cell_is_input_complex(t);
        });
    };
    struct SpannedFace
    {
        std::array<size_t, 3> vids; // sorted
        std::vector<std::array<size_t, 2>> edges; // longest first
    };
    const auto scan_spanned = [&](std::vector<std::array<size_t, 2>>& edges,
                                  std::vector<SpannedFace>& faces) {
        edges.clear();
        faces.clear();
        if (!m_offset_params.front_spanned_split) return;
        for (const Tuple& e : get_edges()) {
            const size_t a = e.vid(*this), b = e.switch_vertex(*this).vid(*this);
            if (!on_front(a) || !on_front(b) || edge_is_offset_surface_live(a, b)) continue;
            if (!off_input(get_incident_tids_for_edge(a, b))) continue;
            edges.push_back({{std::min(a, b), std::max(a, b)}});
        }
        for (const Tuple& f : get_faces()) {
            std::array<size_t, 3> fv = get_face_vids(f);
            if (!std::all_of(fv.begin(), fv.end(), on_front) || face_is_offset_surface_live(f)) {
                continue;
            }
            if (!edge_is_offset_surface_live(fv[0], fv[1]) ||
                !edge_is_offset_surface_live(fv[1], fv[2]) ||
                !edge_is_offset_surface_live(fv[0], fv[2])) {
                continue;
            }
            std::vector<size_t> tids{f.tid(*this)};
            if (const auto o = f.switch_tetrahedron(*this)) tids.push_back(o->tid(*this));
            if (!off_input(tids)) continue;
            SpannedFace s;
            std::sort(fv.begin(), fv.end());
            s.vids = fv;
            s.edges = {{{fv[0], fv[1]}}, {{fv[1], fv[2]}}, {{fv[0], fv[2]}}};
            std::sort(s.edges.begin(), s.edges.end(), [&](const auto& x, const auto& y) {
                const double lx = len2(x), ly = len2(y);
                return lx != ly ? lx > ly : x < y;
            });
            faces.push_back(std::move(s));
        }
    };
    const auto face_alive = [this](const std::array<size_t, 3>& s) {
        for (const size_t tid : get_incident_tids_for_edge(s[0], s[1])) {
            const std::array<size_t, 4> vs = oriented_tet_vids(tid);
            if (std::find(vs.begin(), vs.end(), s[2]) != vs.end()) return true;
        }
        return false;
    };
    const auto split_one = [this](const std::array<size_t, 2>& e) {
        const Tuple t = tuple_from_edge(e);
        reserve_free_slots(cell_slot_bound(t), 1); // serial: grow rather than refuse
        std::vector<Tuple> new_edges;
        m_pocket_splitting = true; // no longest-edge wait (split_edge_before())
        const bool ok = split_edge(t, new_edges);
        m_pocket_splitting = false;
        return ok;
    };

    size_t n_band = 0, n_six = 0, no_face = 0;
    std::vector<Pinned> pinned = scan(no_face, &n_six, &n_band);
    std::vector<std::array<size_t, 2>> sp_edges;
    std::vector<SpannedFace> sp_faces;
    scan_spanned(sp_edges, sp_faces);
    const size_t found = pinned.size() + no_face, found_no_face = no_face;
    const size_t found_edges = sp_edges.size(), found_faces = sp_faces.size();
    size_t n_split = 0, n_refused = 0, n_split_edges = 0, n_split_faces = 0;
    const long waits0 = m_split_order_waits.load();
    int rounds = 0;
    bool cap = false;
    while (!pinned.empty() || !sp_edges.empty() || !sp_faces.empty()) {
        if (rounds == kPocketMaxRounds) {
            cap = true;
            break;
        }
        ++rounds;
        size_t split_now = 0;
        for (const std::array<size_t, 2>& e : sp_edges) {
            if (get_incident_tids_for_edge(e[0], e[1]).empty()) continue; // split already
            if (split_one(e)) {
                ++split_now;
                ++n_split_edges;
            } else {
                ++n_refused;
            }
        }
        for (const SpannedFace& s : sp_faces) {
            if (!face_alive(s.vids)) continue;
            for (const std::array<size_t, 2>& e : s.edges) {
                if (split_one(e)) {
                    ++split_now;
                    ++n_split_faces;
                    break;
                }
                ++n_refused;
            }
        }
        for (const Pinned& p : pinned) {
            if (!alive(p.vids)) continue;
            for (const std::array<size_t, 2>& e : p.edges) {
                if (split_one(e)) {
                    ++split_now;
                    break;
                }
                ++n_refused;
            }
        }
        n_split += split_now;
        // Every cell a split of an edge off the offset surface makes has the off-front midpoint
        // as a corner; a split of an edge on it (a cell with all six edges there) leaves two
        // children with one edge off it. So a rescan finds those children and the cells whose
        // splits were all refused; with none committed it finds the same ones.
        if (split_now == 0) break;
        pinned = scan(no_face, nullptr, nullptr);
        scan_spanned(sp_edges, sp_faces);
    }
    if (cap) {
        logger().warn(
            "\t[pocket split] {}: the cap of {} rounds stopped the rescans with {} pinned cells "
            "left that have an edge to split",
            where,
            kPocketMaxRounds,
            pinned.size());
    }
    const size_t left_six_split =
        size_t(std::count_if(pinned.begin(), pinned.end(), [](const Pinned& p) { return p.six; }));
    logger().info(
        "\t[pocket split] {}: pinned cells (outside the input complex, all four corners on the "
        "front) {}, of which {} with all six edges on the offset surface ({} with all four faces "
        "on it) | edges split {} in {} round(s), split attempts refused {} ({} by the "
        "longest-edge order) | left: {} with all six edges on the offset surface ({} with all "
        "four faces on it, not split; {} every split refused, or the cap), {} with an edge off "
        "it (every split refused, or the cap) | of the pinned cells found, band cells {}{}",
        where,
        found,
        n_six,
        found_no_face,
        n_split,
        rounds,
        n_refused,
        m_split_order_waits.load() - waits0,
        no_face + left_six_split,
        no_face,
        left_six_split,
        pinned.size() - left_six_split,
        n_band,
        m_offset_params.front_spanned_split
            ? fmt::format(
                  " | spanned edges found {}, split {}; spanned faces (three edges on the offset "
                  "surface) found {}, edges split for them {}; left {} edges, {} faces",
                  found_edges,
                  n_split_edges,
                  found_faces,
                  n_split_faces,
                  sp_edges.size(),
                  sp_faces.size())
            : std::string());
}

void TopoOffsetTetMesh::label_cleanup(const std::string& where)
{
    // EXPERIMENTAL_label_cleanup (see the spec and the declaration). MEASURED before it (cube_in,
    // target 0.45, 5% tolerance, 30-run batches): some runs end with thin non-band cells left
    // beyond the level set -- one with 117 such cells reaching 5.6 tolerances out -- where E is
    // 1.2e-7 above its optimum against a settle bar of 3.5e-5, so nothing removes them.
    const std::string& mode = m_offset_params.label_cleanup;
    if (mode == "off" || m_tetwild_pass) return;
    // "one_sided" alone on this branch (Parameters refuses the others): its two bounds of d read
    // the input envelope (min_max_corner_distance(), nearest_primitive()).
    const Envelope& env = *m_input_complex_envelope;
    const double delta = m_offset_params.target_distance;
    const auto label_of = [this](const size_t tid) { return m_tet_attribute[tid].label; };

    // A cell with a front face: one shared with a cell of the other of the two labels, band (2)
    // and non-band (0), neither of them the input complex (1).
    const auto has_front_face = [&](const size_t tid) {
        const int l = label_of(tid);
        if (l == 1) return false;
        for (int j = 0; j < 4; ++j) {
            const std::optional<Tuple> o = tuple_from_face(tid, j).switch_tetrahedron(*this);
            if (o && label_of(o->tid(*this)) != 1 && label_of(o->tid(*this)) != l) return true;
        }
        return false;
    };

    // THE SIDE TEST: whether t's own corners put it on the other label's side. It reads nothing
    // the clean-up changes (no vertex moves), so a cell's answer is the same in every pass and a
    // flipped cell is never asked to flip back: a cell flips at most once, and the passes end.
    //   one_sided: non-band -> band iff U(t) < delta, U the min over input triangles P of the max
    //     of d_P over t's corners (min_max_corner_distance()), an upper bound of d on t (d <= d_P,
    //     d_P convex), so d < delta on all of t. band -> non-band iff L(t) > delta,
    //     L = d(c) - max_k |v_k - c|, c the centroid: a lower bound of d on t (d is 1-Lipschitz,
    //     and the point of t farthest from c is a corner). L <= d <= U: the two exclude each other.
    //   (Uday's "energy" and "one_sided_tips" decide by the sign of the band term, which this
    //   branch does not have; they are refused.)
    const auto wants_flip = [&](const size_t tid) {
        const std::array<size_t, 4> vs = oriented_tet_vids(tid);
        const bool band = cell_is_offset_band(tid);
        std::array<Vector3d, 4> p;
        for (int k = 0; k < 4; ++k) p[size_t(k)] = m_vertex_attribute[vs[size_t(k)]].m_posf;
        if (!band) return min_max_corner_distance(env, m_input_complex_pad, p.data(), 4) < delta;
        const Vector3d c = (p[0] + p[1] + p[2] + p[3]) / 4.;
        double r = 0.;
        for (const Vector3d& q : p) r = std::max(r, (q - c).norm());
        const double dc = env.primitive_distance(nearest_primitive(env, m_input_complex_pad, c), c);
        return dc - r > delta;
    };

    // ELIGIBILITY: two labels only, so that the topology test sees everything t touches. Every
    // cell sharing a vertex with t is band or non-band (no input-complex cell), not plastic (its
    // AMIPS term would read the label), and of the one tag of its label there (the flipped cell
    // takes the other label's, `tag`). And no corner of t is on anything else: the input complex
    // (vertex label 1, which also catches a sheet, wire or point complex without cells), another
    // tracked surface (vertex_is_on_region(): a tag boundary or the box), or the mesh boundary (a
    // face through it with no cell beyond) -- which implies that no face of t is on the mesh
    // boundary. The corner conditions are there because the test below reads cells only: at a
    // corner on the wall, a flip the test allows can still join the front to the wall at that one
    // vertex.
    const auto eligible = [&](const size_t tid, CellTag& tag) {
        const std::array<size_t, 4> vs = oriented_tet_vids(tid);
        std::array<const CellTag*, 3> tag_of{{nullptr, nullptr, nullptr}}; // by label
        for (const size_t v : vs) {
            if (m_vertex_extra[v].label == 1 || vertex_is_on_region(v)) return false;
            for (const size_t s : get_one_ring_tids_for_vertex(v)) {
                const int l = label_of(s);
                if ((l != 0 && l != 2) || cell_is_plastic(s)) return false;
                const CellTag& tg = m_tet_attribute[s].tag;
                if (!tag_of[size_t(l)]) {
                    tag_of[size_t(l)] = &tg;
                } else if (*tag_of[size_t(l)] != tg) {
                    return false;
                }
                for (int k = 0; k < 4; ++k) {
                    const Tuple f = tuple_from_face(s, k);
                    if (f.switch_tetrahedron(*this)) continue;
                    const std::array<size_t, 3> fv = get_face_vids(f);
                    if (std::find(fv.begin(), fv.end(), v) != fv.end()) return false;
                }
            }
        }
        const int other = label_of(tid) == 2 ? 0 : 2;
        tag = tag_of[size_t(other)] ? *tag_of[size_t(other)] : CellTag{};
        return true;
    };

    // THE TOPOLOGY TEST. f = the faces of t shared with a cell of the other label, the one t
    // would take:
    //   f = 0: refused (t would start a new piece, or the front would pinch at an edge or vertex);
    //   f = 1: refused if the corner opposite that face is in a cell of the other label;
    //   f = 2: the two faces share an edge; refused if the edge on neither face (the two corners
    //          opposite them) is in a cell of the other label;
    //   f = 3: allowed (the fourth face becomes front);
    //   f = 4: refused (t is a whole piece by itself; the flip deletes it).
    // An allowed flip has the other label meet t in exactly its f faces, a disk, so taking t in
    // keeps that label's topology. t's own label, which loses t, meets t in its 4 - f faces and
    // also, for f = 2, in the edge the two other-label faces share, for f = 3 in their common
    // corner, when that is in another cell of t's label. Not tested: on a manifold front it
    // cannot happen (it is a front edge on more than two front faces, or a front vertex where t's
    // label meets twice); such flips are counted (`odd`), and the component check below catches a
    // split. Returns f.
    const auto topology = [&](const size_t tid, bool& allow, bool& odd) {
        const std::array<size_t, 4> vs = oriented_tet_vids(tid);
        const int self = label_of(tid), other = self == 2 ? 0 : 2;
        std::vector<size_t> opp_other, opp_self; // the corner opposite each face, by its far side
        for (int j = 0; j < 4; ++j) {
            const Tuple f = tuple_from_face(tid, j);
            const std::array<size_t, 3> fv = get_face_vids(f);
            size_t opp = vs[0];
            for (const size_t v : vs) {
                if (std::find(fv.begin(), fv.end(), v) == fv.end()) opp = v;
            }
            const std::optional<Tuple> o = f.switch_tetrahedron(*this); // eligibility: present
            (o && label_of(o->tid(*this)) == other ? opp_other : opp_self).push_back(opp);
        }
        const auto at_vertex = [&](const size_t v, const int l) {
            for (const size_t s : get_one_ring_tids_for_vertex(v)) {
                if (s != tid && label_of(s) == l) return true;
            }
            return false;
        };
        const auto at_edge = [&](const size_t a, const size_t b, const int l) {
            for (const size_t s : get_incident_tids_for_edge(a, b)) {
                if (s != tid && label_of(s) == l) return true;
            }
            return false;
        };
        const int f = int(opp_other.size());
        allow = false;
        odd = false;
        if (f == 1) allow = !at_vertex(opp_other[0], other);
        if (f == 2) {
            allow = !at_edge(opp_other[0], opp_other[1], other);
            odd = at_edge(opp_self[0], opp_self[1], self);
        }
        if (f == 3) {
            allow = true;
            odd = at_vertex(opp_self[0], self);
        }
        return f;
    };

    // A flip, and everything derived from t's label, which lives on t's closure: the closed labels
    // of its faces, edges and corners (2 where some cell holding it is band, else 0: eligibility
    // keeps 1 out of the closure), the cached class of its four faces -- the only faces whose two
    // sides change -- and the front flags of its four corners, the only vertices on those faces.
    // The base's union flag m_is_on_surface is raised for a corner that joins the front, as the
    // split and the collapse raise it; it is never cleared (vertex_is_held()).
    const auto any_band = [this](const std::vector<size_t>& tids) {
        return std::any_of(tids.begin(), tids.end(), [this](const size_t s) {
            return cell_is_offset_band(s);
        });
    };
    const auto flip = [&](const size_t tid, const CellTag& tag) {
        m_tet_attribute[tid].label = label_of(tid) == 2 ? 0 : 2;
        m_tet_attribute[tid].tag = tag;
        for (int j = 0; j < 4; ++j) {
            const Tuple f = tuple_from_face(tid, j);
            const size_t fid = f.fid(*this);
            std::vector<size_t> sides{tid};
            if (const std::optional<Tuple> o = f.switch_tetrahedron(*this)) {
                sides.push_back(o->tid(*this));
            }
            int& lf = m_face_extra[fid].label;
            if (lf != 1) lf = any_band(sides) ? 2 : 0;
            if (face_is_offset_surface_live(f)) {
                m_face_attribute[fid].m_is_surface_fs = true;
                m_face_attribute[fid].m_surface_class = OFFSET_SURFACE_CLASS;
            } else if (m_face_attribute[fid].m_surface_class == OFFSET_SURFACE_CLASS) {
                m_face_attribute[fid].m_is_surface_fs = false;
                m_face_attribute[fid].m_surface_class = 0; // left at 0 off the surface
            }
        }
        for (int j = 0; j < 6; ++j) {
            const Tuple e = tuple_from_edge(tid, j);
            int& le = m_edge_attribute[e.eid(*this)].label;
            if (le != 1) {
                const size_t a = e.vid(*this), b = e.switch_vertex(*this).vid(*this);
                le = any_band(get_incident_tids_for_edge(a, b)) ? 2 : 0;
            }
        }
        for (const size_t v : oriented_tet_vids(tid)) {
            int& lv = m_vertex_extra[v].label;
            if (lv != 1) lv = any_band(get_one_ring_tids_for_vertex(v)) ? 2 : 0;
            refresh_offset_membership(v);
            if (m_vertex_extra[v].m_is_on_offset) m_vertex_attribute[v].m_is_on_surface = true;
        }
    };

    // THE INVARIANT'S MEASURES: connected components of one label's cells under face adjacency,
    // and front edges on more than two front faces (where the label changes more than twice
    // around the edge).
    const auto label_components = [this](const int l) {
        std::vector<size_t> parent(tet_capacity());
        std::iota(parent.begin(), parent.end(), size_t(0));
        const auto root = [&parent](size_t x) {
            while (parent[x] != x) {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            return x;
        };
        size_t n = 0;
        for (const Tuple& t : get_tets()) {
            const size_t tid = t.tid(*this);
            if (m_tet_attribute[tid].label != l) continue;
            ++n;
            for (int j = 0; j < 4; ++j) {
                const std::optional<Tuple> o = tuple_from_face(tid, j).switch_tetrahedron(*this);
                if (!o || m_tet_attribute[o->tid(*this)].label != l) continue;
                const size_t a = root(tid), b = root(o->tid(*this));
                if (a != b) {
                    parent[a] = b;
                    --n;
                }
            }
        }
        return n;
    };
    const auto nonmanifold_front_edges = [this]() {
        std::map<std::array<size_t, 2>, int> n;
        for (const std::array<size_t, 3>& f : offset_surface_faces()) { // sorted triples
            ++n[std::array<size_t, 2>{{f[0], f[1]}}];
            ++n[std::array<size_t, 2>{{f[1], f[2]}}];
            ++n[std::array<size_t, 2>{{f[0], f[2]}}];
        }
        size_t c = 0;
        for (const auto& ek : n) c += ek.second > 2 ? 1 : 0;
        return c;
    };

    const double e0 = offset_error();
    const size_t band0 = label_components(2), non0 = label_components(0);
    const size_t nm0 = nonmanifold_front_edges();
    std::vector<char> flipped(tet_capacity(), 0);
    std::vector<size_t> corners; // of the flipped cells
    size_t to_band = 0, to_non = 0, n_odd = 0;
    int passes = 0;
    // The last pass's verdicts: that pass flips nothing, so they are what the clean-up leaves.
    size_t n_cand = 0, n_want = 0, n_skip = 0;
    std::array<size_t, 5> n_refused{}; // by f
    for (;;) {
        ++passes;
        std::vector<size_t> cands;
        for (const Tuple& t : get_tets()) {
            if (has_front_face(t.tid(*this))) cands.push_back(t.tid(*this));
        }
        std::sort(cands.begin(), cands.end());
        n_cand = cands.size();
        n_want = 0;
        n_skip = 0;
        n_refused.fill(0);
        size_t n_now = 0;
        // The three tests are pure reads, so the order they are asked in decides only what is
        // counted: the side test first, so that the counts are of cells on the other side.
        for (const size_t tid : cands) {
            if (!wants_flip(tid)) continue;
            ++n_want;
            CellTag tag;
            if (!eligible(tid, tag)) {
                ++n_skip;
                continue;
            }
            bool allow = false, odd = false;
            const int f = topology(tid, allow, odd);
            if (!allow) {
                ++n_refused[size_t(f)];
                continue;
            }
            if (flipped[tid]) {
                log_and_throw_error(
                    "[label cleanup] {}: cell {} flips a second time; its side test reads its own "
                    "corners alone, which the clean-up does not move",
                    where,
                    tid);
            }
            // The topology test reads the label the cell joins only; on a manifold front the old
            // label cannot meet the cell again (Uday 2026-10-26: throw rather than count it).
            // MEASURED before the throw: 0 such flips in every run (cube_in 0.45, both modes).
            if (odd) {
                log_and_throw_error(
                    "[label cleanup] {}: cell {} (f = {}) passes the topology test, but its old "
                    "label meets it again at a front edge on more than two front faces or a front "
                    "vertex, which the test does not read",
                    where,
                    tid,
                    f);
            }
            flipped[tid] = 1;
            ++(cell_is_offset_band(tid) ? to_non : to_band);
            n_odd += odd ? 1 : 0;
            flip(tid, tag);
            for (const size_t v : oriented_tet_vids(tid)) corners.push_back(v);
            ++n_now;
        }
        if (n_now == 0) break;
    }
    double e1 = e0;
    size_t band1 = band0, non1 = non0, nm1 = nm0;
    if (to_band + to_non > 0) {
        // What reads the front beyond the flipped cells' closure: the stored vertex order (the
        // collapse's link condition), from the tracked faces at the vertex, which changed only at
        // the flipped cells' corners; the band-region map, rebuilt as each turn rebuilds it;
        // DEBUG_crossings' snapshot, taken here so that the smoothing pass is compared with the
        // front it starts from.
        wmtk::vector_unique(corners);
        for (const size_t v : corners) m_vertex_attribute[v].m_order = compute_vertex_order(v);
        assign_band_regions(/*log=*/false);
        if (m_offset_params.debug_crossings && m_round > 0) {
            crossing_snapshot("label cleanup", true, true);
        }
        check_offset_membership("label cleanup"); // perform_sanity_checks only
        e1 = offset_error();
        band1 = label_components(2);
        non1 = label_components(0);
        nm1 = nonmanifold_front_edges();
    }
    logger().info(
        "\t[label cleanup] {}: {} | {} pass(es), flipped non-band -> band {}, band -> non-band {} "
        "({} of them with f = 2 or 3 where the old label meets the cell again, at a front edge on "
        "more than two front faces or a front vertex, not tested) | left after the last pass: {} "
        "cells with a front face, {} on the other side by the side test, of which {} skipped for "
        "eligibility, refused by topology f=0 {}, f=1 {}, f=2 {}, f=4 {} | global offset error "
        "{:.9g} -> {:.9g} | "
        "components (face adjacency) band {} -> {}, non-band {} -> {} | non-manifold front edges "
        "(on more than two front faces) {} -> {}",
        where,
        mode,
        passes,
        to_band,
        to_non,
        n_odd,
        n_cand,
        n_want,
        n_skip,
        n_refused[0],
        n_refused[1],
        n_refused[2],
        n_refused[4],
        e0,
        e1,
        band0,
        band1,
        non0,
        non1,
        nm0,
        nm1);
    if (band1 != band0 || non1 != non0) {
        log_and_throw_error(
            "[label cleanup] {}: the flips changed the topology -- band components {} -> {}, "
            "non-band components {} -> {} -- which the topology test forbids, a bug ({} flips had "
            "the old label meeting the cell again, which the test does not read)",
            where,
            band0,
            band1,
            non0,
            non1,
            n_odd);
    }
}

bool TopoOffsetTetMesh::pinned_cell(const std::array<size_t, 4>& vids, const int label) const
{
    // As front_pocket_split()'s scan: label 1 is the input complex (cell_is_input_complex()).
    if (label == 1) return false;
    return std::all_of(vids.begin(), vids.end(), [this](const size_t v) {
        return m_vertex_extra[v].m_is_on_offset;
    });
}

void TopoOffsetTetMesh::pinned_count_collapse(const size_t v1_id, const size_t v2_id)
{
    // See m_pinned_counts. Only v1 off the front onto v2 on it: with v1 on the front a cell it
    // turns pinned was pinned already, and with v2 off it none is. The flags read are the ones
    // before the collapse, which re-derives v2's and the edge link's (refresh_offset_membership()).
    // MEASURED (2026-10-23, cube_in at target 0.3 with and without front_pocket_split, input1 at
    // 0.7 turn 1): of 986 committed collapses tested, 330 flagged, the flags after the commit
    // made a cell of the changed ones pinned for exactly the 330.
    if (dry_run()) return;
    if (m_vertex_extra[v1_id].m_is_on_offset || !m_vertex_extra[v2_id].m_is_on_offset) return;
    PinnedCounts& c = m_pinned_counts[m_tetwild_pass ? 1 : 0][size_t(OpKind::collapse)];
    ++c.seen;
    for (const size_t tid : get_one_ring_tids_for_vertex(v1_id)) {
        std::array<size_t, 4> vs = oriented_tet_vids(tid);
        if (std::find(vs.begin(), vs.end(), v2_id) != vs.end()) continue; // the collapse deletes it
        std::replace(vs.begin(), vs.end(), v1_id, v2_id);
        if (pinned_cell(vs, m_tet_attribute[tid].label)) {
            ++c.pinned;
            pinned_op().pinned = true;
            return;
        }
    }
}

void TopoOffsetTetMesh::pinned_count_swap(const std::vector<size_t>& tids)
{
    // See m_pinned_counts. Called once swap_capture_tag() has taken the ring's one label, which
    // every new cell takes. A 4-4 / 5-6 only records it here; its cases are pinned_count_case()'s.
    if (dry_run()) return;
    PinnedOp& op = pinned_op();
    op.interior = true;
    op.label = m_swap_label.local();
    const OpKind k = current_op_kind();
    if (k != OpKind::swap_32 && k != OpKind::swap_face) return;
    PinnedCounts& c = m_pinned_counts[m_tetwild_pass ? 1 : 0][size_t(k)];
    ++c.seen;
    // The ring's corners in every cell of it -- the edge (a, b) of a 3-2, the face (v0, v1, v2)
    // of a face swap -- and the rest: the link p, q, r of the 3-2, the apexes v3, v4.
    std::vector<std::pair<size_t, size_t>> count; // (vertex, cells holding it)
    for (const size_t tid : tids) {
        for (const size_t v : oriented_tet_vids(tid)) {
            auto it = std::find_if(count.begin(), count.end(), [v](const auto& p) {
                return p.first == v;
            });
            if (it == count.end())
                count.emplace_back(v, 1);
            else
                ++it->second;
        }
    }
    std::vector<size_t> all, rest;
    for (const auto& [v, n] : count) (n == tids.size() ? all : rest).push_back(v);
    std::vector<std::array<size_t, 4>> cells;
    if (k == OpKind::swap_32) {
        assert(all.size() == 2 && rest.size() == 3);
        for (const size_t a : all) cells.push_back({{a, rest[0], rest[1], rest[2]}});
    } else {
        assert(all.size() == 3 && rest.size() == 2);
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                cells.push_back({{rest[0], rest[1], all[size_t(i)], all[size_t(j)]}});
            }
        }
    }
    for (const std::array<size_t, 4>& vs : cells) {
        if (pinned_cell(vs, op.label)) {
            ++c.pinned;
            op.pinned = true;
            return;
        }
    }
}

void TopoOffsetTetMesh::pinned_count_case(const std::vector<std::array<size_t, 4>>& tets)
{
    // See m_pinned_counts. One candidate case of an interior 4-4 / 5-6; the swap's totals are
    // added at its outcome (pinned_count_outcome()).
    PinnedOp& op = pinned_op();
    if (dry_run() || !op.interior) return;
    ++op.cases;
    for (const std::array<size_t, 4>& vs : tets) {
        if (pinned_cell(vs, op.label)) {
            ++op.cases_pinned;
            return;
        }
    }
}

void TopoOffsetTetMesh::pinned_count_outcome(const OpKind k, const OpEvent e) const
{
    // See m_pinned_counts. A real attempt has one outcome event after its `attempt`, so the swap's
    // cases are added once; a dry run tested nothing (cases 0, pinned false).
    PinnedOp& op = pinned_op();
    if (e == OpEvent::attempt) {
        op = PinnedOp{};
        return;
    }
    PinnedCounts& c = m_pinned_counts[m_tetwild_pass ? 1 : 0][size_t(k)];
    const bool committed = e == OpEvent::committed;
    if (k == OpKind::swap_44 || k == OpKind::swap_56) {
        if (op.cases == 0) return;
        ++c.seen;
        c.cases += op.cases;
        c.cases_pinned += op.cases_pinned;
        if (op.cases_pinned > 0) {
            ++c.pinned;
            if (committed) ++c.pinned_committed;
        }
        if (op.cases_pinned == op.cases) {
            ++c.all_pinned;
            if (committed) ++c.all_pinned_committed;
        }
        op.cases = 0;
    } else if (committed && op.pinned) {
        ++c.pinned_committed;
    }
}

void TopoOffsetTetMesh::pinned_counts_reset(const int pass)
{
    for (PinnedCounts& c : m_pinned_counts[size_t(pass)]) {
        c.seen = 0;
        c.pinned = 0;
        c.pinned_committed = 0;
        c.cases = 0;
        c.cases_pinned = 0;
        c.all_pinned = 0;
        c.all_pinned_committed = 0;
    }
    for (size_t k = 0; k < size_t(OpKind::COUNT); ++k) {
        m_pinned_ops0[k] = {
            {op_event_count(OpKind(k), OpEvent::attempt),
             op_event_count(OpKind(k), OpEvent::screened),
             op_event_count(OpKind(k), OpEvent::committed)}};
    }
}

std::string TopoOffsetTetMesh::pinned_counts_report(const int pass) const
{
    // The operations' own totals over the same window (the [ops accounting] events): attempts
    // include a screened pass's dry runs, which `screened` counts when they passed.
    const auto totals = [this](const OpKind k) {
        const auto& t0 = m_pinned_ops0[size_t(k)];
        return fmt::format(
            "[attempts {}, screened {}, committed {}]",
            op_event_count(k, OpEvent::attempt) - t0[0],
            op_event_count(k, OpEvent::screened) - t0[1],
            op_event_count(k, OpEvent::committed) - t0[2]);
    };
    const auto& P = m_pinned_counts[size_t(pass)];
    std::string out;
    {
        const PinnedCounts& c = P[size_t(OpKind::collapse)];
        out += fmt::format(
            "collapse, v1 off the front onto v2 on it: {}, would make a pinned cell {} (committed "
            "{}) {}",
            c.seen.load(),
            c.pinned.load(),
            c.pinned_committed.load(),
            totals(OpKind::collapse));
    }
    for (const OpKind k : {OpKind::swap_32, OpKind::swap_face}) {
        const PinnedCounts& c = P[size_t(k)];
        out += fmt::format(
            " | {} interior: {}, would make a pinned cell {} (committed {}) {}",
            op_kind_name(k),
            c.seen.load(),
            c.pinned.load(),
            c.pinned_committed.load(),
            totals(k));
    }
    for (const OpKind k : {OpKind::swap_44, OpKind::swap_56}) {
        const PinnedCounts& c = P[size_t(k)];
        out += fmt::format(
            " | {} interior with a case scored: {}, cases {}, cases with a pinned cell (dropped) "
            "{}, swaps with a case dropped {} (committed {}), with every case dropped {} "
            "(committed {}) {}",
            op_kind_name(k),
            c.seen.load(),
            c.cases.load(),
            c.cases_pinned.load(),
            c.pinned.load(),
            c.pinned_committed.load(),
            c.all_pinned.load(),
            c.all_pinned_committed.load(),
            totals(k));
    }
    return out;
}

bool TopoOffsetTetMesh::optimize_offset_loop(
    const bool final_stage,
    const bool refine,
    const std::string& label,
    const int max_turns,
    const bool fold_free)
{
    // One loop: TetWild's operation groups (split / collapse / swap, each followed by smoothing)
    // with the front placed by the offset objective inside the smoothing passes. No envelope
    // holds the front in the loop, neither the operations nor the smoothing; only a TetWild pass
    // holds it to one (build_offset_envelope()).
    const int rounds = std::max(1, m_offset_params.max_rounds);
    const int a_iters = std::max(1, m_offset_params.max_iterations);
    check_no_vertex_on_both_surfaces("construction");
    audit_surface_containment("construction");
    needle_scan("after construction, before the loop");
    assign_band_regions();
    m_front_gradient_reference = front_gradient_linf();
    logger().info(
        "\tLOOP: TetWild's operation groups with the front placed inside their "
        "smoothing passes | front energy-gradient reference {:.6g} | ONE criterion: the RMS "
        "relative error over a stencil_order {} stencil ({} points per face) against front_conv "
        "{:.6g} ({:.6g} x the configured target_distance)",
        m_front_gradient_reference,
        m_offset_params.stencil_order,
        stencil_points_per_face(),
        m_offset_params.front_conv,
        m_offset_params.front_conv_rel);
    logger().info(
        "\t[offset envelope] the loop's operations and smoothing do not hold the offset surface "
        "to an envelope; a TetWild pass does (eps envelope_size {:.6g})",
        m_offset_params.envelope_size);
    (void)rounds;
    // max_turns > 0: growth_mode "smart"'s attempt (smart_growth_attempts) in place of max_rounds.
    const int budget = max_turns > 0 ? max_turns : std::max(1, m_offset_params.max_rounds);
    const char* budget_name = max_turns > 0 ? "smart_growth_attempts" : "max_rounds";
    // One turn is TetWild's operation groups, run here rather than through mesh_improvement().
    // Since 2026-10-26 its stall response is here too, once for AMIPS and once for the offset error
    // (convergence_turn(), under EXPERIMENTAL_settle_refinement).
    // interleaved_smoothing true is TetWild's shape of a turn: three groups, each one operation
    // pass followed by k smoothing passes. With it false, this component's default, a turn is ONE
    // group -- split, collapse and swap back to back -- followed by one smoothing block of
    // num_smoothing_passes. Why the switch reaches the loop: measured on the
    // deliverable cube at target_distance_rel 1e-2 / front_conv_rel 1e-4, of the six smoothing
    // passes per turn only the first after the split moved the front from turn 5 on; the other five
    // never moved a vertex across the bar, moved the background under 2% of its target edge length,
    // and cost 63% of the turn (43 of 68 s). What the combined group changes, to be measured and
    // not argued: the collapse and swap energy rules judge a front the split pass has not been
    // placed since.
    const bool interleaved = m_params.interleaved_smoothing;
    const int k = std::max(
        1,
        interleaved ? m_params.interleaved_smoothing_passes : m_params.num_smoothing_passes);
    const std::vector<std::array<int, 4>> groups =
        interleaved
            ? std::vector<std::array<int, 4>>{{{1, 0, 0, k}}, {{0, 1, 0, k}}, {{0, 0, 1, k}}}
            : std::vector<std::array<int, 4>>{{{1, 1, 1, k}}};
    const std::vector<const char*> group_names =
        interleaved ? std::vector<const char*>{"split", "collapse", "swap"}
                    : std::vector<const char*>{"ops"};
    logger().info(
        "\tTurn shape: {}, run twice per turn -- a FRONT block whose smoothing moves only the "
        "front vertices the loop places (the offset error objective), then a BACKGROUND block "
        "whose smoothing moves every other vertex (TetWild's smoother)",
        interleaved
            ? fmt::format(
                  "INTERLEAVED -- split, collapse, swap, each followed by {} smoothing "
                  "pass(es)",
                  k)
            : fmt::format(
                  "COMBINED -- split, collapse, swap back to back, then {} smoothing pass(es) "
                  "(interleaved_smoothing false)",
                  k));
    compute_vertex_partition_morton();
    // One turn of grace after the field is lowered: refine_front_by_halving() lowers sizing
    // scalars at the end of a turn, and the split pass
    // that realizes them does not run until the NEXT turn.
    if (m_offset_params.pre_smooth) {
        // One smoothing block on the constructed mesh before turn 1's split pass: the same
        // block every operation group is followed by. Frames are r0S*.
        m_round = 0;
        logger().info(
            "\t[pre_smooth] one smoothing block before turn 1: {} interleaved smoothing pass(es)",
            k);
        loop_group({{0, 0, 0, k}}, "pre_smooth");
    }
    op_accounting_reset(); // the [ops accounting] lines are per turn, from turn 1's first op
    m_split_order_waits = 0; // and so are the [split order] lines
    m_split_off_longest = 0;
    // EXPERIMENTAL_settle_refinement: the max-based exit and the two stuck-refines
    // (convergence_turn()); the energy criterion's refinable sets, its sizing-floor warnings and
    // its exit do not act. stuck carries the stall tests' previous maxima and cooldowns.
    const bool settle = m_offset_params.settle_refinement;
    StuckState stuck = stuck_state_start();
    // Forced splits are vertex ids queued for the next split pass; ids left by an earlier pass or
    // loop (a TetWild pass's last stall, a smart growth attempt restored from its snapshot) name
    // other edges now.
    m_force_split_edges.clear();
    // THE TWO BLOCKS OF A TURN (Spencer, 2026-10-26): the operation groups run twice, first with
    // smoothing on the front vertices the loop places alone -- the offset surface is placed, which
    // may make bad cells -- then with smoothing on every other vertex alone, which cleans the
    // mesh up around it (m_smooth_set, smooth_before()). Back to every vertex on any way out.
    struct SmoothSetReset
    {
        SmoothSet& s;
        ~SmoothSetReset() { s = SmoothSet::All; }
    } smooth_set_reset{m_smooth_set};
    const std::array<std::pair<SmoothSet, const char*>, 2> blocks = {
        {{SmoothSet::Front, "front"}, {SmoothSet::Background, "background"}}};
    for (int it = 0; it < budget; ++it) {
        m_round = it + 1;
        m_iterations_used = it + 1;
        for (const auto& [block_set, block_name] : blocks) {
            m_smooth_set = block_set;
            const size_t skipped0 = m_smooth_set_skipped.load();
            for (size_t gi = 0; gi < groups.size(); ++gi) {
                ++m_smooth_group; // see VertexExtra::m_front_change
                // E before and after each group, the descent check (a collapse or swap group may
                // not raise it; a split group may).
                const double energy_before_group = global_energy();
                const int vol_c0 = m_volume_reject_collapse.load(),
                          vol_s0 = m_volume_reject_swap.load(),
                          vol_p0 = m_volume_reject_split.load();
                const int pl_c0 = m_plastic_reject_collapse.load(),
                          pl_s0 = m_plastic_reject_swap.load();
                const int fold_c0 = m_fold_reject_collapse.load(),
                          fold_s0 = m_fold_reject_swap.load(),
                          fold_m0 = m_fold_reject_smooth.load();
                if (gi == 1) needle_scan("collapse pass");
                if (!interleaved) needle_scan("combined ops pass");
                // [energy step] lines: the group's baseline here, then one per smoothing pass from
                // smoothing_pass_done() (see log_energy_step()).
                if (m_offset_params.debug_pinned_counts) pinned_counts_reset(0);
                m_energy_step_turn = it + 1;
                m_energy_step_group = group_names[gi];
                m_energy_step_first = true;
                log_energy_step("start");
                // loop_group(): the group's passes, with EXPERIMENTAL_front_pocket_split's splits
                // before each smoothing pass and EXPERIMENTAL_label_cleanup after the operation
                // passes (both off: local_operations(groups[gi]) as before).
                loop_group(
                    groups[gi],
                    fmt::format("turn {} {} block {} group", it + 1, block_name, group_names[gi]));
                m_energy_step_turn = 0;
                // Per group, so a containment violation is attributed to the pass that made it
                // rather than found at the end of the run. Same gate as the shared sanity check.
                if (m_params.perform_sanity_checks) {
                    audit_surface_containment(
                        fmt::format("turn {} after {}", it + 1, group_names[gi]));
                }
                {
                    const double energy_after_group = global_energy();
                    logger().info(
                        "\t[energy] turn {} {} group: E {:.10g} -> {:.10g} ({:+.4g}) | {} block | "
                        "refused by the front max rule (stencil-point offset error / AMIPS): {} "
                        "collapses, {} flips; {} splits refused by the split gate",
                        it + 1,
                        group_names[gi],
                        energy_before_group,
                        energy_after_group,
                        energy_after_group - energy_before_group,
                        block_name,
                        m_volume_reject_collapse.load() - vol_c0,
                        m_volume_reject_swap.load() - vol_s0,
                        m_volume_reject_split.load() - vol_p0);
                    if (m_offset_params.debug_pinned_counts) {
                        logger().info(
                            "\t[pinned would-refuse] turn {} {} block {} group: {}",
                            it + 1,
                            block_name,
                            group_names[gi],
                            pinned_counts_report(0));
                    }
                    if (fold_reject_active()) {
                        logger().info(
                            "\t[op reject] turn {} {} group: refused for raising the folded offset "
                            "faces (growth_mode individual_op_reject): {} collapses, {} "
                            "offset-surface "
                            "flips, {} smoothing moves",
                            it + 1,
                            group_names[gi],
                            m_fold_reject_collapse.load() - fold_c0,
                            m_fold_reject_swap.load() - fold_s0,
                            m_fold_reject_smooth.load() - fold_m0);
                    }
                    if (m_plastic_active) {
                        logger().info(
                            "\t[plastic] turn {} {} group: refused by the plastic max rule "
                            "(use_rest_pose, plastic AMIPS): {} collapses, {} swaps",
                            it + 1,
                            group_names[gi],
                            m_plastic_reject_collapse.load() - pl_c0,
                            m_plastic_reject_swap.load() - pl_s0);
                    }
                }
            }
            logger().info(
                "\t[block] turn {} {} block done: its smoothing passes left {} vertex visits to "
                "the "
                "other block",
                it + 1,
                block_name,
                m_smooth_set_skipped.load() - skipped0);
        }
        m_smooth_set = SmoothSet::All;
        consolidate_mesh();
        m_cross_valid = false; // DEBUG_crossings: consolidation renumbered the vertices
        assign_band_regions();
        const double amips = std::get<0>(optimization_quality_stats());
        const double bar = optimization_stop_metric();
        const EnergyCriterion ec = energy_criterion();
        const Vector3d wx = ec.worst_vid != static_cast<size_t>(-1)
                                ? m_vertex_attribute[ec.worst_vid].m_posf
                                : Vector3d::Zero();
        if (ec.ring_exit) {
            // front_measure "vertex_ring": the ring measure is the exit test; the face measure
            // and the vertex measure are the same numbers the face mode prints, as diagnostics.
            const Vector3d rx = ec.worst_ring_vid != static_cast<size_t>(-1)
                                    ? m_vertex_attribute[ec.worst_ring_vid].m_posf
                                    : Vector3d::Zero();
            logger().info(
                "======== turn {} / {}: max AMIPS {:.4} (stop {:.4}) | {} max "
                "{:.4}x the bar (avg {:.4}x) (worst v{} at ({:.4}, {:.4}, {:.4})) over {} "
                "front vertices, {} rings unmeasurable, {} unmeasurable in all (the exit test) | "
                "diagnostic: faces max {:.4}x (avg {:.4}x), {} faces over the bar of {}; front "
                "vertices max {:.4}x (avg {:.4}x), {} not placed of {} | vertices over the bar: "
                "{}, refinable {} (at the sizing floor {}) ========",
                it + 1,
                budget,
                amips,
                bar,
                ec.ring_name(),
                ec.max_ring,
                ec.avg_ring(),
                ec.worst_ring_vid,
                rx.x(),
                rx.y(),
                rx.z(),
                ec.n_rings,
                ec.n_rings_unmeasurable,
                ec.n_unmeasurable,
                ec.max_face,
                ec.avg_face(),
                ec.n_faces_over,
                ec.n_faces,
                ec.max_vertex,
                ec.avg_vertex(),
                ec.n_unplaced,
                ec.n_vertices,
                ec.n_rings_over,
                ec.refinable_vertices.size(),
                ec.n_rings_at_floor);
        } else {
            logger().info(
                "======== turn {} / {}: max AMIPS {:.4} (stop {:.4}) | front vertices "
                "max {:.4}x the bar (avg {:.4}x) (worst v{} at ({:.4}, {:.4}, {:.4})) "
                "(diagnostic), faces max {:.4}x (avg {:.4}x), {} unmeasurable (the exit test) | {} "
                "vertices, {} faces | faces over the bar: {}, of which {} with all corners placed "
                "(worst {:.4}x, centroid ({:.4}, {:.4}, {:.4})) | refinable faces {} (at the "
                "sizing floor {}) ========",
                it + 1,
                budget,
                amips,
                bar,
                ec.max_vertex,
                ec.avg_vertex(),
                ec.worst_vid,
                wx.x(),
                wx.y(),
                wx.z(),
                ec.max_face,
                ec.avg_face(),
                ec.n_unmeasurable,
                ec.n_vertices,
                ec.n_faces,
                ec.n_faces_over,
                ec.n_faces_over_placed,
                ec.max_face_placed,
                ec.worst_placed_centroid.x(),
                ec.worst_placed_centroid.y(),
                ec.worst_placed_centroid.z(),
                ec.refinable.size(),
                ec.n_at_floor);
        }
        // Faces over the bar with every corner at the sizing floor block the exit (they are over
        // the bar) and no refinement will ever take them, so a run that keeps them never
        // converges: a warning, every turn they exist. The turn line's "at the sizing floor"
        // count also holds faces whose corners are ABOVE the floor and whose longest edge the
        // split pass will shorten -- on the deliverable cube at target_distance_rel 1e-2 /
        // front_conv_rel 1e-4, every face it counted in turns 2, 4 and 5 (2, 2 and 16) was of
        // that kind, with corner scalars down to 0.0625 against a floor of 0.002 -- so the same
        // line says which, at info when none is at the floor. A turn with neither says nothing
        // beyond its turn line.
        //
        // Under the ring measure the same line names the VERTICES over the bar at the floor,
        // always a warning: the vertex form of the halving has no chord rule, so every vertex it
        // cannot take is one nothing will ever take.
        if (!settle && ec.n_at_floor > 0) {
            logger().log(
                ec.n_corners_at_floor > 0 ? spdlog::level::warn : spdlog::level::info,
                "\t[sizing floor] turn {}: {}",
                it + 1,
                ec.sizing_floor_fact());
        }
        if (!settle && ec.ring_exit && ec.n_rings_at_floor > 0) {
            logger().warn("\t[sizing floor] turn {}: {}", it + 1, ec.sizing_floor_fact());
        }
        // Every place a proposed swap can be turned down, counted for this turn. See
        // TetOptimizerMesh::SwapReject: this is instrumentation for why an offset-surface flip
        // is never accepted, and the counters are reset each turn so the line is per-turn.
        logger().info("\t[swap reject] turn {}: {}", it + 1, swap_reject_report());
        swap_counters_reset();
        // Every split, collapse and swap this turn attempted, each ended at exactly one named
        // place; see TetOptimizerMesh::op_accounting_report(). A line warns when a refusal was
        // made that no counter named -- that is a silent refusal site, i.e. a defect in the
        // accounting, and it should not exist.
        for (const OpKind k :
             {OpKind::split,
              OpKind::collapse,
              OpKind::swap_32,
              OpKind::swap_44,
              OpKind::swap_56,
              OpKind::swap_face}) {
            logger().log(
                op_accounting_unexplained(k) ? spdlog::level::warn : spdlog::level::info,
                "\t[ops accounting] turn {} {}",
                it + 1,
                op_accounting_report(k));
        }
        op_accounting_reset();
        // See split_edge_before(): splits that waited for a longer edge, and splits that
        // committed off the longest edge of an incident tet anyway -- every longer edge was under
        // the gate, which the order does not forbid. That count is NOT the threads defect: the
        // halving grades the sizing field, and the serial pass commits the same kind from turn 2
        // on (cube 1e-2 / 1e-4: 0, 379, 1577, 4573, 10539, 10843, 8298 per turn).
        logger().info(
            "\t[split order] turn {}: waited {} | committed off the longest edge {}",
            it + 1,
            m_split_order_waits.exchange(0),
            m_split_off_longest.exchange(0));
        logger().info("\t[flip funnel] turn {}: {}", it + 1, flip_funnel_report());
        flip_funnel_reset();
        // perform_sanity_checks only: m_is_on_offset against the labels, whole mesh. Free when
        // the key is off, which is the default.
        check_offset_membership(fmt::format("turn {}", it + 1).c_str());
        // Not gated on the key: silent unless a face lookup actually missed this run.
        report_offset_face_lookup_misses(fmt::format("turn {}", it + 1).c_str());
        if (!settle && !refine &&
            (!ec.refinable.empty() || (ec.ring_exit && ec.n_rings_over > 0))) {
            logger().info(
                "\t[resolution] turn {}: {} -- no refinement ({} front vertex(es) / {} face(s) "
                "over the bar left as they are)",
                it + 1,
                label,
                ec.n_rings_over,
                ec.refinable.size());
        }
        if (!settle && refine && !ec.refinable.empty()) {
            // Refinement is the halving, and only the halving: every refinable face has the
            // sizing scalar at its corners halved.
            const size_t n = refine_front_by_halving(ec.refinable);
            logger().info(
                "\t[resolution] turn {}: {} front face(s) {} whose RMS "
                "relative error over {} stencil point(s) is over the bar (worst placed {:.4}x, "
                "centroid ({:.4}, {:.4}, {:.4})) -> sizing scalar halved at {} vertices",
                it + 1,
                ec.refinable.size(),
                "(placed or not)",
                stencil_points_per_face(),
                ec.max_face_placed,
                ec.worst_placed_centroid.x(),
                ec.worst_placed_centroid.y(),
                ec.worst_placed_centroid.z(),
                n);
        }
        if (!settle && refine && ec.ring_exit && ec.n_rings_over > 0) {
            // front_measure "vertex_ring": the halving takes each vertex over the bar, that
            // vertex alone. refinable is empty in this mode, so the face line above is silent.
            const size_t n = refine_front_by_halving(ec.refinable_vertices);
            logger().info(
                "\t[resolution] turn {}: {} front vertex(es) whose {} (the RMS of the face "
                "measures of its incident offset faces, each face weighted equally) is over the "
                "bar, {} of them at the sizing floor (worst {:.4}x) -> sizing scalar halved at {} "
                "vertices",
                it + 1,
                ec.n_rings_over,
                ec.ring_name(),
                ec.n_rings_at_floor,
                ec.max_ring,
                n);
        }
        // EXPERIMENTAL_settle_refinement: the max-based exit and the two stuck-refines, after the
        // consolidation (the forced splits are vertex ids); true when the loop may exit, which then
        // replaces EnergyCriterion::converged() below.
        const bool settle_exit = settle && convergence_turn(it + 1, stuck, refine);
        // The turn's "end" frame is written HERE, after the refinement, not before it: it is the
        // turn's final state, so what it carries is the sizing field the refinement just lowered.
        // 3D ONLY; 2D still writes its end frame before the refinement. See .claude/CLAUDE.md.
        if (m_offset_params.debug_output) {
            write_optimization_debug_output(fmt::format("end_{}S", it + 1));
        }
        // Termination: every offset face's measure within the bar and nothing unmeasurable
        // (EnergyCriterion::converged()) -- then quality with the front held in its envelope
        // (below). The face measure is the one quantity the smoothing minimises and the refinement
        // reads; the vertex measure is reported on the turn line and in the verdict, and tested
        // nowhere. Nothing refinable is implied, a refinable face being over the bar. Under
        // front_measure "vertex_ring" the tested measure is every front vertex's ring measure
        // instead, and the face measure joins the vertex measure as a diagnostic
        // (EnergyCriterion::ring_exit).
        //
        // The loop exits on the FIRST turn that meets the criterion. It used to additionally
        // demand that the previous turn lowered no sizing scalar -- one turn of hysteresis,
        // which given that the exit required an empty refinable set amounted to two
        // consecutive turns without a lowering, the second of them converged. That was dropped
        // because it cost many turns waiting for two to line up, and TetWild's loop takes the
        // same choice: it breaks the moment its max energy is under stop_energy, that number
        // being a property of the mesh it is holding, where refinable is a request for work on
        // the next turn.
        //
        // EXPERIMENTAL_settle_refinement: convergence_turn()'s exit (max AMIPS below stop_energy,
        // every offset face's stencil-point offset error below front_conv_frac()^2) in place of
        // converged(); the criterion's measures are then diagnostics.
        bool resolved = settle ? settle_exit : ec.converged();
        // EXPERIMENTAL_exit_no_folds: no exit while a real fold remains (count_real_folds()). The
        // vertex exit tests the front's vertices only; a flat sheet of inner cells within
        // front_conv of the level set passes it. MEASURED (cube_in at 0.45, front_conv 5%, 30
        // runs, 8 threads): one run exited with an inner region of 0.5% of the target's volume,
        // every front point within 0.76 front_conv, 5 real folds; 5-7 of each batch's 28-30
        // exits had 1-5 real folds.
        if (m_offset_params.exit_no_folds) {
            const size_t folds = count_real_folds();
            logger().info(
                "\t[exit folds] turn {}: real folds {}{}",
                it + 1,
                folds,
                resolved && folds > 0 ? " -- the exit test passes; held for the folds" : "");
            if (folds > 0) resolved = false;
        }
        // fold_free (growth_mode "smart"): a converged loop with a folded offset face has failed,
        // judged here, before the verdict and the final TetWild pass, which it does not get.
        if (resolved && fold_free) {
            const FoldCensus fc = fold_census();
            if (fc.n_fold > 0) {
                logger().warn(
                    "[{}] converged at target_distance {:.6g} after {} turn(s), but {} of {} "
                    "offset faces are folded (more than foldover_angle {} degrees from grad d; the "
                    "worst {:.4g} degrees, at ({:.4}, {:.4}, {:.4})): not fold-free",
                    label,
                    m_offset_params.target_distance,
                    it + 1,
                    fc.n_fold,
                    fc.n_faces,
                    m_offset_params.foldover_angle,
                    fc.worst,
                    fc.worst_at[0],
                    fc.worst_at[1],
                    fc.worst_at[2]);
                return false;
            }
        }
        if (resolved && !final_stage) {
            // The init_optimize loop ends here; the final pass and the verdict belong to the
            // ordinary loop that follows it.
            logger().info(
                "[{}] converged at target_distance {:.6g} after {} turn(s): {}{} max {:.4}x the "
                "bar over {} front vertices, max AMIPS {:.4}",
                label,
                m_offset_params.target_distance,
                it + 1,
                settle ? "max AMIPS and every stencil-point offset error within their bounds "
                         "(EXPERIMENTAL_settle_refinement); diagnostic: "
                       : "",
                ec.ring_exit ? std::string(ec.ring_name()) : std::string("faces"),
                (ec.ring_exit ? ec.max_ring : ec.max_face) / ec.bar,
                ec.n_vertices,
                amips);
            return true;
        }
        if (resolved) {
            m_energy_verdict = ec;
            m_converged = true;
            // Provisional: the final pass below overwrites both when it runs. The verdict at the
            // end of optimize_offset() requires this AND the front's.
            m_quality_max_amips = amips;
            m_quality_converged = amips < bar;
            if (settle) {
                logger().info(
                    "The front is resolved after {} iteration(s): max AMIPS below stop_energy "
                    "and every offset face's stencil-point offset error below front_conv_frac^2 "
                    "(EXPERIMENTAL_settle_refinement); offset faces max {:.4}x with {} over the "
                    "bar (diagnostic); front vertices max {:.4}x (diagnostic); max AMIPS {:.4} "
                    "against stop {:.4}",
                    it + 1,
                    ec.max_face / ec.bar,
                    ec.n_faces_over,
                    ec.max_vertex / ec.bar,
                    amips,
                    bar);
            } else if (ec.ring_exit) {
                logger().info(
                    "The front is resolved after {} iteration(s): every front "
                    "vertex's {} within the bar (rings max {:.4}x), nothing unmeasurable; offset "
                    "faces max {:.4}x with {} over the bar (diagnostic); front vertices max {:.4}x "
                    "(diagnostic); max AMIPS {:.4} against stop {:.4}",
                    it + 1,
                    ec.ring_name(),
                    ec.max_ring / ec.bar,
                    ec.max_face / ec.bar,
                    ec.n_faces_over,
                    ec.max_vertex / ec.bar,
                    amips,
                    bar);
            } else {
                logger().info(
                    "The front is resolved after {} iteration(s): every offset face "
                    "within the bar (faces max {:.4}x), nothing unmeasurable; front vertices max "
                    "{:.4}x (diagnostic); max AMIPS {:.4} against stop {:.4}",
                    it + 1,
                    ec.max_face / ec.bar,
                    ec.max_vertex / ec.bar,
                    amips,
                    bar);
            }
            if (amips >= bar) {
                logger().info(
                    "======== final pass, TetWild with the front held in its envelope: max AMIPS "
                    "{:.6g} >= stop_energy {} "
                    "========",
                    amips,
                    m_params.stop_energy);
                m_round = it + 2;
                const double final_amips = tetwild_pass("final pass", a_iters);
                m_quality_max_amips = final_amips;
                m_quality_converged = final_amips < m_params.stop_energy;
                logger().log(
                    m_quality_converged ? spdlog::level::info : spdlog::level::warn,
                    "\t[final pass] max element quality {:.4} (stop {:.4}) -> {}",
                    final_amips,
                    optimization_stop_metric(),
                    m_quality_converged ? "ok" : "STILL OVER: the run does not converge");
                // Under DEBUG_remote_output the "final" frame at the end of optimize_offset() is
                // this mesh.
                if (m_offset_params.debug_output && !remote_frames()) {
                    write_optimization_debug_output(fmt::format("end_{}F", it + 2));
                }
            }
            return true;
        }
    }
    logger().warn(
        "The loop did not converge in {} turns ({}){}",
        budget,
        budget_name,
        label.empty() ? std::string()
                      : fmt::format(
                            " -- {} at target_distance {:.6g}",
                            label,
                            m_offset_params.target_distance));
    return false;
}

double TopoOffsetTetMesh::tetwild_pass(const char* what, const int max_its)
{
    // The final envelopes -- the front's envelope, and m_envelope widened to every region boundary
    // as placement left them (build_final_envelopes()). TetWild's own rules (m_op_rules: the
    // smoother's raw AMIPS sum, the operations' max of AMIPS^3), no plastic medium
    // (AMIPS against the regular tet); the front smoothed inside its envelope like any held vertex.
    build_final_envelopes();
    // A loop's last stuck-refine may have queued forced splits by vertex ids the pass's mesh has
    // renumbered since; the pass's own stall response queues its own.
    m_force_split_edges.clear();
    m_tetwild_pass = true;
    // TetWild's own rules for the operations and the smoother (m_op_rules).
    const OpRules rules_was = m_op_rules;
    m_op_rules = OpRules::TetWild;
    const bool plastic_was = m_plastic_active;
    m_plastic_active = false;
    // DEBUG_remote_output: the pass's start frame; its iterations' frames come from
    // optimization_iteration_done().
    if (remote_frames()) {
        write_debug_frame(fmt::format("{}r{}F_start", m_frame_prefix, m_round));
    }
    if (m_offset_params.debug_pinned_counts) pinned_counts_reset(1);
    mesh_improvement(max_its);
    logger().info(
        "\t[split order] {}: waited {} | committed off the longest edge {}",
        what,
        m_split_order_waits.exchange(0),
        m_split_off_longest.exchange(0));
    // DEBUG_pinned_counts: the whole pass, its iterations together (mesh_improvement() runs them).
    if (m_offset_params.debug_pinned_counts) {
        logger().info("\t[pinned would-refuse] {}: {}", what, pinned_counts_report(1));
    }
    m_plastic_active = plastic_was;
    m_op_rules = rules_was;
    m_tetwild_pass = false;
    release_final_envelopes();
    assign_band_regions();
    return std::get<0>(optimization_quality_stats());
}

void TopoOffsetTetMesh::optimize_construction_pass()
{
    // See the declaration and the spec doc (optimize_construction).
    const auto t0 = std::chrono::steady_clock::now();
    // The sizing field at 1 everywhere. Nothing before this pass lowers it (the march's splits
    // carry it, at 1), so a vertex that needs resetting here is unexpected: counted and warned.
    size_t n_reset = 0;
    for (const Tuple& v : get_vertices()) {
        double& s = m_vertex_attribute[v.vid(*this)].m_sizing_scalar;
        if (s != 1.) {
            s = 1.;
            ++n_reset;
        }
    }
    if (n_reset > 0) {
        logger().warn(
            "\t[optimize construction] {} vertices had a sizing scalar other than 1; set to 1",
            n_reset);
    }
    // The offset surface's envelope in this pass only: half the distance the march traced it to
    // (d_0 when growth_mode applies, else target_distance); every other tracked
    // surface keeps m_envelope (envelope_size). Nothing marched: envelope_size for it too.
    m_construction_envelope_eps = m_construction_distance > 0. ? 0.5 * m_construction_distance : 0.;
    const double amips0 = std::get<0>(optimization_quality_stats());
    logger().info(
        "======== [optimize construction] TetWild on the mesh as constructed, from the sizing "
        "field at 1, under TetWild's own rules (raw AMIPS sum, max-AMIPS guards; no band term), "
        "every tracked surface held (the offset surface to eps {:.6g}, half the traced distance "
        "{:.6g}; the rest to eps {:.6g}): max AMIPS {:.6g}, stop_energy {} | #V {}, #T {} ========",
        m_construction_envelope_eps > 0. ? m_construction_envelope_eps
                                         : m_offset_params.envelope_size,
        m_construction_distance,
        m_envelope_eps,
        amips0,
        m_params.stop_energy,
        vertex_size(),
        tet_size());
    log_energy_step("start");
    m_frame_prefix = "c";
    m_round = 0;
    const double amips1 =
        tetwild_pass("optimize construction", std::max(1, m_offset_params.max_iterations));
    m_construction_envelope_eps = 0.;
    // The loop's setup, redone on what the pass left: a compact mesh, the band regions on it, the
    // vertex order, and under use_rest_pose the rests the loop measures plastic cells against.
    consolidate_mesh();
    assign_band_regions();
    init_vertex_order();
    stamp_plastic_rests();
    log_energy_step("tetwild");
    if (m_params.debug_output) write_optimization_debug_output("end_construction_tetwild");
    m_frame_prefix.clear();
    double s_min = std::numeric_limits<double>::infinity(), s_max = 0.;
    for (const Tuple& v : get_vertices()) {
        const double s = m_vertex_attribute[v.vid(*this)].m_sizing_scalar;
        s_min = std::min(s_min, s);
        s_max = std::max(s_max, s);
    }
    const bool under = amips1 < m_params.stop_energy;
    logger().log(
        under ? spdlog::level::info : spdlog::level::warn,
        "\t[optimize construction] max AMIPS {:.6g} -> {:.6g} (stop {}){} | #V {}, #T {} | sizing "
        "scalar {:.6g} .. {:.6g} | {:.3g} s",
        amips0,
        amips1,
        m_params.stop_energy,
        under ? "" : ", STILL OVER stop_energy",
        vertex_size(),
        tet_size(),
        s_min,
        s_max,
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
}

TopoOffsetTetMesh::MeshSnapshot TopoOffsetTetMesh::take_snapshot()
{
    // See the declaration.
    if (m_track_feature_edges || m_track_feature_points) {
        log_and_throw_error(
            "take_snapshot: feature-edge or feature-point tracking is on, and the image does not "
            "hold their collections");
    }
    consolidate_mesh();
    MeshSnapshot s;
    s.n_vertices = vert_capacity();
    const size_t n_tets = tet_capacity();
    s.tets.resize(n_tets);
    for (size_t t = 0; t < n_tets; ++t) s.tets[t] = oriented_tet_vids(t);
    // The live slots of a collection, [0, n): consolidate_mesh() packed them there.
    const auto head = [](const auto& c, const size_t n) {
        return std::vector(c.m_attributes.begin(), c.m_attributes.begin() + std::ptrdiff_t(n));
    };
    s.vertex = head(m_vertex_attribute, s.n_vertices);
    s.vertex_extra = head(m_vertex_extra, s.n_vertices);
    s.face = head(m_face_attribute, 4 * n_tets);
    s.face_extra = head(m_face_extra, 4 * n_tets);
    s.edge = head(m_edge_attribute, 6 * n_tets);
    s.tet = head(m_tet_attribute, n_tets);
    restore_snapshot(s);
    return s;
}

void TopoOffsetTetMesh::restore_snapshot(const MeshSnapshot& s)
{
    // See the declaration. init_with_isolated_vertices() leaves every collection cleared and
    // sized to the new capacity, at least the live count, so the copies fit.
    init_with_isolated_vertices(s.n_vertices, s.tets);
    const auto put = [](auto& c, const auto& v) {
        std::copy(v.begin(), v.end(), c.m_attributes.begin());
    };
    put(m_vertex_attribute, s.vertex);
    put(m_vertex_extra, s.vertex_extra);
    put(m_face_attribute, s.face);
    put(m_face_extra, s.face_extra);
    put(m_edge_attribute, s.edge);
    put(m_tet_attribute, s.tet);
    assign_band_regions(/*log=*/false);
}

double TopoOffsetTetMesh::growth_stage_target(
    const double d0,
    const double delta,
    const int k,
    const int n)
{
    return k >= n ? delta : d0 + double(k) * (delta - d0) / double(n);
}

void TopoOffsetTetMesh::linear_growth()
{
    // See the declaration and the spec doc (growth_mode "linear"). Between stages only
    // target_distance changes, and what init_offset_potential() builds on it: the field, the
    // per-piece fields, the exact integral's reach. Every other reader of target_distance (the
    // band term, the settle test, the measures, the front smoother) asks for it when it runs, and
    // no pass is running here.
    const double delta = m_offset_params.target_distance;
    const double d0 = m_growth_start;
    // growth_mode "individual_op_reject": d_0, then target_distance, nothing between.
    const int n = m_offset_params.growth_mode == "linear" ? m_offset_params.linear_growth_steps : 1;
    const auto set_target = [&](const double d) {
        m_offset_params.target_distance = d;
        init_offset_potential();
    };
    std::string done; // the stages before the last, for the last one's header
    for (int k = 0; k < n; ++k) {
        set_target(growth_stage_target(d0, delta, k, n));
        m_frame_prefix = fmt::format("g{}", k);
        logger().info(
            "======== [incremental growth] stage {} / {}: target_distance {:.6g} ({:.4g}x "
            "target_distance {:.6g}){} ========",
            k,
            n,
            m_offset_params.target_distance,
            m_offset_params.target_distance / delta,
            delta,
            k == 0 ? ", the distance the march traced to" : "");
        // Parameters::init() refuses front_conv above target_distance itself; a stage's target
        // below it is not refused, only said.
        if (m_offset_params.front_conv > m_offset_params.target_distance) {
            logger().warn(
                "[incremental growth] stage {} / {}: front_conv {:.6g} exceeds the stage's "
                "target_distance {:.6g}, so its tests read nearly every front vertex as within "
                "the bar",
                k,
                n,
                m_offset_params.front_conv,
                m_offset_params.target_distance);
        }
        const bool converged = optimize_offset_loop(
            /*final_stage=*/false,
            /*refine=*/true,
            fmt::format("incremental growth stage {} / {}", k, n));
        // The loop sets m_iterations_used every turn, and a stage before the last has no final
        // pass to reset it: its turns.
        done += fmt::format(
            "{}stage {} at {:.6g} in {} turn(s)",
            k == 0 ? "" : ", ",
            k,
            m_offset_params.target_distance,
            m_iterations_used);
        m_converged = false;
        m_energy_verdict.reset();
        if (!converged) {
            m_frame_prefix.clear();
            // The verdict and the report measure this mesh against target_distance itself.
            set_target(delta);
            logger().warn(
                "[incremental growth] stage {} / {} at target_distance {:.6g} did not converge in "
                "max_rounds {} turns: the run stops here, stages {}..{} not run; the verdict "
                "measures this mesh against target_distance {:.6g}",
                k,
                n,
                growth_stage_target(d0, delta, k, n),
                m_offset_params.max_rounds,
                k + 1,
                n,
                delta);
            return;
        }
    }
    m_frame_prefix.clear();
    set_target(delta);
    logger().info(
        "======== [incremental growth] stage {} / {}: target_distance {:.6g}, the ordinary loop "
        "(converged before it: {}) ========",
        n,
        n,
        delta,
        done);
    optimize_offset_loop();
}

void TopoOffsetTetMesh::smart_growth()
{
    // See the declaration and the spec doc (growth_mode "smart"). As in linear_growth(), only
    // target_distance changes between loops, and what init_offset_potential() builds on it.
    const auto t0 = std::chrono::steady_clock::now();
    const double delta = m_offset_params.target_distance;
    const double d0 = m_growth_start;
    const int attempts = m_offset_params.smart_growth_attempts;
    const auto set_target = [&](const double d) {
        m_offset_params.target_distance = d;
        init_offset_potential();
    };
    // Parameters::init() refuses front_conv above target_distance itself; a smaller loop's target
    // below it is not refused, only said.
    const auto warn_front_conv = [&](const std::string& what) {
        if (m_offset_params.front_conv > m_offset_params.target_distance) {
            logger().warn(
                "[smart growth] {}: front_conv {:.6g} exceeds its target_distance {:.6g}, so its "
                "tests read nearly every front vertex as within the bar",
                what,
                m_offset_params.front_conv,
                m_offset_params.target_distance);
        }
    };

    // Level 0: the loop at d_0, from the front as the march left it.
    set_target(d0);
    m_frame_prefix = "s0";
    logger().info(
        "======== [smart growth] level 0: target_distance {:.6g} ({:.4g}x target_distance {:.6g}), "
        "the distance the march traced to; up to max_rounds {} turns ========",
        d0,
        d0 / delta,
        delta,
        m_offset_params.max_rounds);
    warn_front_conv("level 0");
    if (!optimize_offset_loop(/*final_stage=*/false, /*refine=*/true, "smart growth level 0")) {
        m_frame_prefix.clear();
        log_and_throw_error(
            "[smart growth] level 0 at the traced distance {:.6g} did not converge in max_rounds "
            "{} "
            "turns",
            d0,
            m_offset_params.max_rounds);
    }
    m_converged = false;
    m_energy_verdict.reset();
    std::string history = fmt::format("level 0 at {:.6g} in {} turn(s)", d0, m_iterations_used);
    MeshSnapshot snap = take_snapshot();
    logger().info(
        "[smart growth] level 0 converged at target_distance {:.6g}; the mesh is stored (#V {}, "
        "#T {})",
        d0,
        vertex_size(),
        tet_size());
    SmartGrowthSchedule sched{d0, delta, m_offset_params.smart_growth_max_fails};
    int n_reverted = 0;
    for (;;) {
        // j == 0 is target_distance itself, set exactly (not by the formula).
        const bool at_delta = sched.j == 0;
        const double t = at_delta ? delta : sched.target();
        const int level = sched.level, attempt = sched.j + 1;
        const std::string label = fmt::format("smart growth level {} attempt {}", level, attempt);
        set_target(t);
        m_frame_prefix = fmt::format("s{}a{}", level, attempt);
        logger().info(
            "======== [smart growth] level {} attempt {}: target_distance {:.6g} ({:.4g}x "
            "target_distance {:.6g}), from the converged {:.6g}{}; up to smart_growth_attempts {} "
            "turns, converged and fold-free to succeed ========",
            level,
            attempt,
            t,
            t / delta,
            delta,
            sched.base,
            at_delta ? ", the final stage" : "",
            attempts);
        warn_front_conv(label);
        const bool ok = optimize_offset_loop(
            /*final_stage=*/at_delta,
            /*refine=*/true,
            label,
            attempts,
            /*fold_free=*/true);
        const int turns = m_iterations_used;
        const double base = sched.base;
        switch (sched.report(ok)) {
        case SmartGrowthSchedule::Next::Done:
            m_frame_prefix.clear();
            logger().info(
                "[smart growth] target_distance {:.6g} reached: {}, level {} at it in {} turn(s); "
                "{} failed attempt(s) reverted; {:.3g} s",
                delta,
                history,
                level,
                turns,
                n_reverted,
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
            return;
        case SmartGrowthSchedule::Next::Advance:
            m_converged = false;
            m_energy_verdict.reset();
            history += fmt::format(", level {} at {:.6g} in {} turn(s)", level, t, turns);
            snap = take_snapshot();
            logger().info(
                "[smart growth] level {} converged at target_distance {:.6g} (attempt {}); the "
                "mesh is stored (#V {}, #T {}), the next attempt is at target_distance {:.6g}",
                level,
                t,
                attempt,
                vertex_size(),
                tet_size(),
                delta);
            break;
        case SmartGrowthSchedule::Next::Retry:
        case SmartGrowthSchedule::Next::GiveUp:
            ++n_reverted;
            m_converged = false;
            m_energy_verdict.reset();
            restore_snapshot(snap);
            logger().warn(
                "[smart growth] level {} attempt {} at target_distance {:.6g} failed (not "
                "converged in {} turn(s), or folded; see above): the mesh is back at the "
                "converged {:.6g} (#V {}, #T {}){}",
                level,
                attempt,
                t,
                turns,
                base,
                vertex_size(),
                tet_size(),
                sched.j < m_offset_params.smart_growth_max_fails
                    ? fmt::format("; next attempt at {:.6g}", sched.target())
                    : std::string());
            if (sched.j >= m_offset_params.smart_growth_max_fails) {
                m_frame_prefix.clear();
                log_and_throw_error(
                    "[smart growth] {} attempts from the converged distance {:.6g} failed "
                    "(smart_growth_max_fails), the last at target_distance {:.6g}; converged "
                    "before: {}",
                    m_offset_params.smart_growth_max_fails,
                    base,
                    t,
                    history);
            }
            break;
        }
    }
}

void TopoOffsetTetMesh::optimize_offset(const std::filesystem::path& output_file)
{
    logger().info("Optimizing offset (3D)...");

    // From here on every edge split is an optimization split, run by the shared engine.
    m_edge_split_mode = EdgeSplitMode::Optimization;

    // label the offset surface, and with it the vertices the optimization places
    logger().info("\tLabel offset faces...");
    label_offset_boundary();
    // The baseline the propagation has to hold from here on. label_offset_boundary() marks the
    // flag from m_face_extra's construction labels; this asks the live cell-label test the same
    // question, so a disagreement at turn 0 would mean the two disagree about what the offset
    // surface IS, before any operation has run.
    check_offset_membership("construction");
    report_offset_face_lookup_misses("construction");

    init_vertex_order();

    // use_rest_pose: the plastic medium (cell_is_plastic()) -- every cell outside the band and
    // the input complex measures its AMIPS against its rest shape, stamped here so the first
    // operations already have one. Off, every cell is measured against the regular tet.
    m_plastic_active = m_offset_params.use_rest_pose;
    stamp_plastic_rests();
    if (m_plastic_active) {
        logger().info(
            "[plastic] use_rest_pose true: AMIPS of every cell outside the band and the input "
            "complex against its rest shape: stamped now, after every smoothing pass, and on every "
            "cell a split or swap creates or a collapse changes; all-plastic operations decided "
            "as exact ties; off in the final pass");
    } else {
        logger().info("[plastic] use_rest_pose false: equilateral AMIPS for every cell");
    }

    // The front as constructed must already be inside the potential's support.
    check_offset_within_support("Offset as constructed");

    logger().info(
        "\tOffset criterion: |grad (Phi - c)^2 . n| <= (front_conv / target_distance) {} x "
        "max|grad (Phi - c)^2 . n| over the band AS CONSTRUCTED, with n the unit normal from "
        "the offset surface's own normal (Voronoi-weighted at vertices, the face's own inside "
        "a face). Measured over every band vertex and {} stencil point(s) "
        "per band face; the reference is reported next, before the loop starts.",
        m_offset_params.front_conv_frac(),
        stencil_points_per_face());

    // No sizing seed here: the loop starts from the field as construction left it, which with
    // no pre-optimization pass is 1.0 everywhere unless the input itself carried a scalar. The
    // front's resolution comes from the refinement rule once it is placed.
    {
        double s_min = std::numeric_limits<double>::infinity(), s_max = 0.;
        for (const Tuple& v : get_vertices()) {
            const double s = m_vertex_attribute[v.vid(*this)].m_sizing_scalar;
            s_min = std::min(s_min, s);
            s_max = std::max(s_max, s);
        }
        logger().info(
            "[sizing] the loop starts from the sizing field as is (whatever construction left): "
            "scalar {:.6g} .. {:.6g}",
            s_min,
            s_max);
    }
    // Unconditional: write_vtu() must not be the only consolidate here (see the 2D twin). No
    // frame here: nothing changes the mesh between this point and the "construction" frame the
    // optimization writes first, so one would duplicate the other.
    consolidate_mesh();

    iter_cnt_split = 0;
    iter_cnt_collapse = 0;
    iter_cnt_collapse_offset_removed = 0;
    iter_cnt_swap = 0;
    m_smooth_trace.reset();
    optimization_metrics.clear();

    // Frame 0 is the mesh as constructed, before the optimization touches it.
    if (remote_frames()) {
        write_debug_frame("construction"); // DEBUG_remote_output's first frame
    } else if (m_params.debug_output) {
        m_debug_pass_name = "construction";
        write_optimization_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
    }

    // optimize_construction: TetWild on the mesh as constructed, from the sizing field at 1, the
    // offset surface held to half the traced distance (optimize_construction_pass()).
    if (m_offset_params.optimize_construction) optimize_construction_pass();

    if (m_init_optimize) {
        // init_optimize (see marching_tets()): the loop first runs to
        // convergence under stencil_order without refinement, the sizing field left as it is, and
        // without the final pass or the verdict; then the ordinary loop below, with its
        // refinement. target_distance is not touched.
        m_frame_prefix = "i";
        logger().info(
            "======== [init_optimize] target_distance {:.6g}, stencil_order {}, no "
            "refinement ========",
            m_offset_params.target_distance,
            m_offset_params.stencil_order);
        optimize_offset_loop(/*final_stage=*/false, /*refine=*/false, "init_optimize");
        m_frame_prefix.clear();
        m_converged = false;
        m_energy_verdict.reset();
        logger().info(
            "======== [init_optimize] done; the loop at target_distance {:.6g}, "
            "stencil_order {}, with refinement ========",
            m_offset_params.target_distance,
            m_offset_params.stencil_order);
    }

    // growth_mode (see marching_tets()): "linear" in stages, the last of them the ordinary loop at
    // target_distance; "smart" in attempts, the one that reaches target_distance its final stage.
    if (m_growth_start > 0. && m_offset_params.growth_mode == "smart") {
        smart_growth();
    } else if (m_growth_start > 0.) {
        linear_growth();
    } else {
        optimize_offset_loop();
    }

    log_smooth_trace();
    logger().info(
        "splits = {} (offset-edge: {} offered -> {} accepted)  |  collapses = {} ({} removed an "
        "offset vertex, {} refused by the offset criterion)  |  swaps = {} ({} refused by the "
        "offset criterion)",
        iter_cnt_split.load(),
        iter_cnt_split_offset_before.load(),
        iter_cnt_split_offset.load(),
        iter_cnt_collapse.load(),
        iter_cnt_collapse_offset_removed.load(),
        iter_cnt_collapse_offset_reject.load(),
        iter_cnt_swap.load(),
        iter_cnt_swap_offset_reject.load());

    // Final metrics and the convergence verdict, one entry for the whole run.
    assign_band_regions();
    const auto [max_dist, avg_dist] = compute_distance_deviation();
    const DistanceSplit r = residual_split();
    const GradientSplit g = gradient_split();
    logger().info(
        "placement gradient (at band vertices): max {} (avg {}) | in-face diagnostic {} ({} face "
        "samples) | {} reachable, {} pinned (max {}), {} skipped ({} unrounded, {} inverted ring)",
        g.max_reachable,
        g.avg_reachable,
        g.max_in_face,
        g.n_face_samples,
        g.n_reachable,
        g.n_pinned,
        g.max_pinned,
        g.n_skipped_unrounded + g.n_skipped_inverted,
        g.n_skipped_unrounded,
        g.n_skipped_inverted);
    logger().info(
        "phi residual (diagnostic, absolute model units): max {} (avg {}) vs front_conv {} | at "
        "vertices {}, inside faces {} | {} samples, {} pinned vertices || euclid dist err: max {} "
        "| avg {}",
        r.max_reachable,
        r.avg_reachable,
        m_offset_params.front_conv,
        r.max_at_vertex,
        r.max_in_face,
        r.n_reachable,
        r.n_pinned,
        max_dist,
        avg_dist);
    optimization_metrics.push_back(
        {{max_dist,
          avg_dist,
          r.max_reachable,
          r.avg_reachable,
          g.max_reachable,
          g.avg_reachable,
          g.max_at_vertex,
          g.max_in_face}});
    log_worst_dist_vertex();

    bool front_ok = false;
    std::string floor_fact; // quoted again by throw_on_nonconvergence below
    std::string quality; // and so is this
    // The quality half of the verdict is judged only once the front is resolved: the loop sets
    // m_quality_* at convergence or after the final pass. A run that ends with the front
    // unresolved never judged it, and printing the defaults read "final quality ok: max AMIPS 0"
    // on every run stopped at max_rounds (the slot model at 7 turns, 2026-10-06).
    const auto quality_fact = [&](const bool judged) {
        return judged ? fmt::format(
                            "final quality {}: max AMIPS {:.4} vs stop_energy {}",
                            m_quality_converged ? "ok" : "OVER",
                            m_quality_max_amips,
                            m_params.stop_energy)
                      : fmt::format(
                            "final quality not judged (the front is not resolved): max AMIPS "
                            "{:.4} now vs stop_energy {}",
                            std::get<0>(optimization_quality_stats()),
                            m_params.stop_energy);
    };
    {
        // Measured at convergence when the loop converged (see m_energy_verdict), else now.
        const EnergyCriterion ec = m_energy_verdict ? *m_energy_verdict : energy_criterion();
        // Two criteria, both required: the front resolved -- the loop's own exit test,
        // EnergyCriterion::converged() -- and the final quality under stop_energy (the finishing
        // pass's verdict; see m_quality_converged). The vertex numbers are printed as a
        // diagnostic and decide nothing.
        //
        // EXPERIMENTAL_settle_refinement: the front half is the loop's exit (convergence_turn()),
        // the one place that sets m_energy_verdict under the key; the criterion's measures and its
        // sizing-floor fact (about refinable sets that do not act) are diagnostics.
        const bool settle = m_offset_params.settle_refinement;
        front_ok = settle ? m_energy_verdict.has_value() : ec.converged();
        m_converged = front_ok && m_quality_converged;
        floor_fact = settle ? std::string() : ec.sizing_floor_fact();
        quality = quality_fact(front_ok);
        if (settle) {
            logger().info(
                "[convergence] verdict: front {} -- tested: the loop's exit (max AMIPS below "
                "stop_energy, every offset face's stencil-point offset error below "
                "front_conv_frac^2); the measures on the next line are diagnostics",
                front_ok ? "resolved" : "NOT resolved");
        }
        if (ec.ring_exit) {
            // front_measure "vertex_ring": the ring measure is what was tested; the face and
            // vertex measures are the diagnostics, the face one naming how many faces are still
            // over the bar at the verdict.
            logger().log(
                m_converged ? spdlog::level::info : spdlog::level::warn,
                "{}{}: front {} -- tested (every front vertex's {} within the bar, nothing "
                "unmeasurable): {} rings max {:.4}x the bar (avg {:.4}x), {} rings "
                "unmeasurable, {} unmeasurable in all | diagnostic, not tested: {} faces max "
                "{:.4}x the bar (avg {:.4}x), {} faces over the bar; {} front vertices max {:.4}x "
                "the bar (avg {:.4}x), {} | vertices to resolve {} (at the sizing floor {}) | "
                "front_conv {:.4} || {}{}{}",
                m_converged ? "Converged" : "Optimization did not converge",
                m_energy_verdict ? " (front measured at convergence, before the finishing pass)"
                                 : "",
                front_ok ? "resolved" : "NOT resolved",
                ec.ring_name(),
                ec.n_rings,
                ec.max_ring,
                ec.avg_ring(),
                ec.n_rings_unmeasurable,
                ec.n_unmeasurable,
                ec.n_faces,
                ec.max_face,
                ec.avg_face(),
                ec.n_faces_over,
                ec.n_vertices,
                ec.max_vertex,
                ec.avg_vertex(),
                ec.vertices_ok() ? std::string("all placed")
                                 : fmt::format("{} not placed", ec.n_unplaced),
                ec.refinable_vertices.size(),
                ec.n_rings_at_floor,
                m_offset_params.front_conv,
                quality,
                floor_fact.empty() ? "" : " || ",
                floor_fact);
        } else {
            logger().log(
                m_converged ? spdlog::level::info : spdlog::level::warn,
                "{}{}: front {} -- tested (every face within the bar, nothing unmeasurable): {} "
                "faces max {:.4}x the bar (avg {:.4}x), {} unmeasurable | diagnostic, not "
                "tested: {} front vertices max {:.4}x the bar (avg {:.4}x), {} | faces to resolve "
                "{} (at the sizing floor {}) | front_conv {:.4} || {}{}{}",
                m_converged ? "Converged" : "Optimization did not converge",
                m_energy_verdict ? " (front measured at convergence, before the finishing pass)"
                                 : "",
                front_ok ? "resolved" : "NOT resolved",
                ec.n_faces,
                ec.max_face,
                ec.avg_face(),
                ec.n_unmeasurable,
                ec.n_vertices,
                ec.max_vertex,
                ec.avg_vertex(),
                ec.vertices_ok() ? std::string("all placed")
                                 : fmt::format("{} not placed", ec.n_unplaced),
                ec.refinable.size(),
                ec.n_at_floor,
                m_offset_params.front_conv,
                quality,
                floor_fact.empty() ? "" : " || ",
                floor_fact);
        }
    }

    // DEBUG_remote_output: the final mesh, the one the driver writes.
    if (remote_frames()) write_debug_frame("final");

    // Foldovers on the offset surface, checked UNCONDITIONALLY -- a fold is a defect in the
    // delivered mesh, not a debug curiosity, so it is reported whether or not debug_output wrote
    // the per-face field. Run here, at the end of optimize_offset, so it describes the mesh the
    // driver is about to write: after the finishing pass, not at the loop's verdict. Reported for
    // a run that did not converge too. A face is a foldover when its normal, out of the band, is
    // more than foldover_angle from the field's normal (front_face_field_angle()).
    {
        const FoldCensus fc = fold_census();
        if (fc.n_fold > 0) {
            logger().warn(
                "[foldover] {} of {} offset faces are folded: their normal, out of the band, is "
                "more "
                "than foldover_angle {} degrees from the field's normal (grad d at the centroid); "
                "the worst {:.4g} degrees, at ({:.4}, {:.4}, {:.4}). {} face(s) not measurable. {}",
                fc.n_fold,
                fc.n_faces,
                m_offset_params.foldover_angle,
                fc.worst,
                fc.worst_at[0],
                fc.worst_at[1],
                fc.worst_at[2],
                fc.n_unmeasured,
                m_offset_params.debug_output
                    ? "The per-face flag offset_foldover is on the debug frames' _off files."
                    : "Set DEBUG_output to get the per-face offset_foldover field.");
        }
    }

    // Escalate to a hard failure if the caller asked for it, AFTER the warnings above so the log
    // still names which criterion missed before the throw.
    if (!m_converged && m_offset_params.throw_on_nonconvergence) {
        log_and_throw_error(
            "Optimization did not converge and throw_on_nonconvergence is set: front {} (every "
            "{} within the bar, nothing unmeasurable), {}. Ran {} of {} iterations; see the "
            "warnings above.{}{}",
            front_ok ? "resolved" : "NOT resolved",
            m_offset_params.front_measure == "vertex_ring" ? "front vertex's ring measure" : "face",
            quality,
            optimization_metrics.size(),
            m_offset_params.max_iterations,
            floor_fact.empty() ? "" : " ",
            floor_fact);
    }
}

} // namespace wmtk::components::topological_offset
