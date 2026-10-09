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
#include <vector>

namespace wmtk {
/// Defined in src/wmtk/TetOptimizerMeshSwaps.cpp and declared in no engine header. The face swap's
/// before-hook ends with it; TopoOffsetTetMesh::swap_face_before() duplicates that hook.
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
    const Vector3d& pc,
    double* mean_error,
    double* remainder) const
{
    // THE face measure of the exit and the refinement: the MEAN over the face's stencil of the
    // squared relative error, in units of the tolerance. At every stencil point q of
    // for_each_face_sample():
    //
    //     r(q) = pot.relative_residual(q) / front_conv_frac()
    //          = (signed distance from q to the level set, along the field) / front_conv
    //
    // and the face's term is mean of r^2, so 1 exactly at the bar; its root is the "Nx the bar"
    // figure the logs print. The energy does not read it (tet_energy()).
    //
    // WHY THE FIELD IS ASKED, not (Phi(q) - c)/c formed here. That relative FIELD error is the
    // relative distance error only for a field linear in the distance, as the euclidean one is.
    // For the smooth field it is the distance error times delta |dPhi/dd| / c = 3.44 at the
    // default offset_dhat_factor 2. relative_residual() is the distance to the level set along
    // the field over target_distance for both fields.
    const double level = pot.target_level();
    if (!(level > 0.)) return -1.;
    // Sums weighted by each point's quadrature weight w (1 unless EXPERIMENTAL_quadratic_stencil,
    // see for_each_face_sample()); wsum their total.
    double sum = 0., sum1 = 0., wsum = 0.;
    size_t n = 0;
    bool unmeasurable = false;
    // For the remainder: the normal equations of the (weighted) least-squares fit
    // r ~ c . (wa, wb, wc), the function linear on the face, in the samples' barycentric
    // coordinates.
    Eigen::Matrix3d lam_lam = Eigen::Matrix3d::Zero();
    Vector3d lam_r = Vector3d::Zero();
    for_each_face_sample(
        pa,
        pb,
        pc,
        [&](const Vector3d& q, double wa, double wb, double wc, double w) {
            if (unmeasurable) return;
            const double r = pot.relative_residual(q);
            if (!std::isfinite(r)) {
                unmeasurable = true;
                return;
            }
            sum += w * r * r;
            sum1 += w * r;
            wsum += w;
            ++n;
            if (remainder) {
                const Vector3d lam(wa, wb, wc);
                lam_lam += w * (lam * lam.transpose());
                lam_r += (w * r) * lam;
            }
        });
    // n == 0 only when stencil_order < 0, which the spec's min refuses; an unmeasurable sample
    // reads the whole face unmeasurable.
    if (unmeasurable || n == 0) return -1.;
    if (!(m_offset_params.front_conv_frac() > 0.)) return std::numeric_limits<double>::infinity();
    if (mean_error) *mean_error = (sum1 / wsum) / m_offset_params.front_conv_frac();
    if (remainder) {
        // The corners are among the samples, each with weight >= 1, so lam_lam >= I and the fit
        // is unique. Residual sum of squares = sum w r^2 - lam_r . c; clamped at 0 against
        // round-off.
        const Vector3d c = lam_lam.ldlt().solve(lam_r);
        *remainder = offset_term_weight() * std::max(sum - lam_r.dot(c), 0.) / wsum;
    }
    return offset_term_weight() * (sum / wsum);
}

double TopoOffsetTetMesh::face_offset_term(
    const size_t a,
    const size_t b,
    const size_t c,
    double* mean_error,
    double* remainder) const
{
    return face_offset_term(
        potential_for_edge(a, b),
        m_vertex_attribute[a].m_posf,
        m_vertex_attribute[b].m_posf,
        m_vertex_attribute[c].m_posf,
        mean_error,
        remainder);
}

double TopoOffsetTetMesh::cell_vol_amips3(const size_t tid) const
{
    // See the declaration. The rest's corners are in the oriented order, as the cell's are.
    const auto vs = oriented_tet_vids(tid);
    std::array<Vector3d, 4> p;
    for (int k = 0; k < 4; ++k) p[size_t(k)] = m_vertex_attribute[vs[size_t(k)]].m_posf;
    const TetAttributes& ta = m_tet_attribute[tid];
    if (cell_is_plastic(tid) && ta.rest_valid) {
        Eigen::Matrix3d R;
        for (int k = 1; k < 4; ++k) R.col(k - 1) = ta.rest_pos[size_t(k)] - ta.rest_pos[0];
        // A rest that holds no shape (a cell stamped degenerate) falls back to the regular tet.
        if (R.determinant() > 0.) return VolAMIPSEnergy3D::value_of(p, R);
    }
    return vol_amips3(vs);
}

double TopoOffsetTetMesh::vol_amips3(const std::array<size_t, 4>& vids) const
{
    const double a3 = TetOptimizerMesh::get_quality(vids);
    if (!(a3 < MAX_ENERGY)) return std::numeric_limits<double>::infinity();
    double tr = 0.;
    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            tr += (m_vertex_attribute[vids[size_t(i)]].m_posf -
                   m_vertex_attribute[vids[size_t(j)]].m_posf)
                      .squaredNorm();
        }
    }
    tr *= 0.5;
    return std::sqrt(2.) / 12. * tr * std::sqrt(tr) * std::sqrt(a3);
}

double TopoOffsetTetMesh::band_cell_vd(
    const std::array<size_t, 4>& vids,
    const int64_t stored_tri,
    int64_t* best) const
{
    if (m_n_regions > 1) {
        log_and_throw_error(
            "the energy's D(t) reads one input region, and this input has {}",
            m_n_regions);
    }
    if (!m_band_tris) log_and_throw_error("band_cell_vd(): D(t)'s triangles were not built");
    const double delta = m_offset_params.target_distance;
    std::array<Vector3d, 4> p;
    for (int j = 0; j < 4; ++j) p[size_t(j)] = m_vertex_attribute[vids[size_t(j)]].m_posf;
    const double vol = std::abs((p[1] - p[0]).dot((p[2] - p[0]).cross(p[3] - p[0]))) / 6.;
    std::array<int64_t, 5> cand;
    for (int j = 0; j < 4; ++j) cand[size_t(j)] = m_band_tris->nearest(p[size_t(j)]);
    cand[4] = stored_tri;
    double m = std::numeric_limits<double>::infinity();
    int64_t arg = -1;
    for (size_t i = 0; i < cand.size(); ++i) {
        const int64_t P = cand[i];
        if (P < 0 || std::find(cand.begin(), cand.begin() + i, P) != cand.begin() + i) continue;
        double s = 0.;
        for (const Vector3d& q : p) s += (m_band_tris->distance(P, q) - delta) / delta;
        if (s / 4. < m) m = s / 4., arg = P;
    }
    if (best) *best = arg;
    return vol * m;
}

double TopoOffsetTetMesh::tet_energy(const size_t tid) const
{
    // E_T(t) = V_t ( w A(t)^3 / SE^3 + [t in B] (1 - w) D(t) ); see the declaration.
    const double va = cell_vol_amips3(tid);
    if (!std::isfinite(va)) return MAX_ENERGY;
    double e = amips_weight() * va;
    if (cell_is_offset_band(tid)) {
        e += band_weight() * band_cell_vd(oriented_tet_vids(tid), m_tet_attribute[tid].band_tri);
    }
    return std::isfinite(e) ? std::min(e, MAX_ENERGY) : MAX_ENERGY;
}

double TopoOffsetTetMesh::energy_sum(const std::vector<size_t>& tids) const
{
    double s = 0.;
    for (const size_t tid : tids) {
        const double e = tet_energy(tid);
        if (e >= MAX_ENERGY) return MAX_ENERGY;
        s += e;
    }
    return s;
}

double TopoOffsetTetMesh::total_energy() const
{
    double s = 0.;
    for (const Tuple& t : get_tets()) s += tet_energy(t.tid(*this));
    return s;
}

void TopoOffsetTetMesh::energy_parts(double& amips, double& band) const
{
    amips = 0.;
    band = 0.;
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        amips += amips_weight() * cell_vol_amips3(tid);
        if (cell_is_offset_band(tid) && m_band_tris) {
            band +=
                band_weight() * band_cell_vd(oriented_tet_vids(tid), m_tet_attribute[tid].band_tri);
        }
    }
}

void TopoOffsetTetMesh::log_energy_step(const char* step) const
{
    double amips = 0., band = 0.;
    energy_parts(amips, band);
    logger().info(
        "\t[energy step] turn {} {}: E {:.12g} | AMIPS term {:.12g} | band term {:.12g}",
        m_round,
        step,
        total_energy(),
        amips,
        band);
}

void TopoOffsetTetMesh::refresh_band_tris()
{
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (!cell_is_offset_band(tid)) continue;
        int64_t best = -1;
        band_cell_vd(oriented_tet_vids(tid), m_tet_attribute[tid].band_tri, &best);
        m_tet_attribute[tid].band_tri = best;
    }
}

void TopoOffsetTetMesh::store_band_minimisers(const std::vector<size_t>& tids, const int64_t extra)
{
    if (!m_band_tris) return;
    for (const size_t tid : tids) {
        if (!cell_is_offset_band(tid)) continue;
        const auto vs = oriented_tet_vids(tid);
        int64_t b1 = -1, b2 = -1;
        const double v1 = band_cell_vd(vs, m_tet_attribute[tid].band_tri, &b1);
        const double v2 = extra >= 0 ? band_cell_vd(vs, extra, &b2) : v1;
        m_tet_attribute[tid].band_tri = (extra >= 0 && v2 < v1) ? b2 : b1;
    }
}

int TopoOffsetTetMesh::candidate_label(const std::array<size_t, 4>& vids)
{
    const SwapRecord& rec = m_swap_record.local();
    if (!rec.flip) return m_swap_label.local();
    const SwapSurfaceSides& sides = m_swap_sides.local();
    for (const size_t v : vids) {
        const auto it = sides.by_vertex.find(v);
        if (it != sides.by_vertex.end()) return it->second.second;
    }
    return -1;
}

double TopoOffsetTetMesh::candidate_energy(const std::array<size_t, 4>& vids)
{
    // See the declaration and SwapRecord: E_T of a cell that does not exist yet, read from its
    // corners and the label swap_after_cells() would give it.
    const SwapRecord& rec = m_swap_record.local();
    for (const size_t v : vids) {
        if (rec.active && std::binary_search(rec.verts.begin(), rec.verts.end(), v)) continue;
        log_and_throw_error(
            "TopoOffsetTetMesh::candidate_energy: ({}, {}, {}, {}) is not made of the vertices of "
            "the swap in flight ({}) -- the engine scored a cell its swap's before-hook did not "
            "record",
            vids[0],
            vids[1],
            vids[2],
            vids[3],
            rec.active ? "vertex " + std::to_string(v) + " is not one" : "there is none");
    }
    const int label = candidate_label(vids);
    // A plastic cell a swap creates is stamped at creation: its rest is its own shape, A = 3.
    const bool plastic = m_plastic_active && label == 0;
    const double regular = vol_amips3(vids);
    if (!std::isfinite(regular)) return MAX_ENERGY;
    // V from the same AMIPS: V = V AMIPS^3 / AMIPS^3 (vol_amips3()).
    const double va = plastic ? 27. * regular / TetOptimizerMesh::get_quality(vids) : regular;
    double e = amips_weight() * va;
    if (label == 2) e += band_weight() * band_cell_vd(vids);
    return std::isfinite(e) ? std::min(e, MAX_ENERGY) : MAX_ENERGY;
}

void TopoOffsetTetMesh::swap_record_fill(const std::vector<size_t>& tids, const bool flip)
{
    SwapRecord& rec = m_swap_record.local();
    rec.clear();
    for (const size_t tid : tids) {
        for (const size_t v : oriented_tet_vids(tid)) rec.verts.push_back(v);
    }
    wmtk::vector_unique(rec.verts);
    rec.active = true;
    rec.flip = flip;
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
    // Clear whatever the last surface flip on this thread left behind: the flags outlive one
    // operation, and an interior swap is no flip of the offset surface.
    {
        SwapSurfaceSides& sides = m_swap_sides.local();
        sides.worthwhile = false;
        sides.saw_case = false;
    }
    m_swap_record.local().clear(); // see SwapRecord
    if (!swap_capture_tag(tids)) return false;
    // THE SWAP GUARD's before-half: the sum of E_T over the cells the swap replaces, T_b. The new
    // cells fill exactly their region of space and nothing else changes; swap_after_cells() and
    // the case search compare against it.
    m_swap_energy_before.local() = energy_sum(tids);
    // What the 4-4 / 5-6 case search and the face swap's gate score candidate cells from; see
    // SwapRecord. After the capture, whose single label every new cell takes. A 3-2 has no case
    // search: the guard judges it on the real cells in swap_after_cells().
    if (current_op_kind() != OpKind::swap_32) swap_record_fill(tids, false);
    return true;
}

bool TopoOffsetTetMesh::swap_before_surface(
    const std::vector<size_t>& tids,
    const size_t a,
    const size_t b,
    const size_t c,
    const size_t d)
{
    // Re-decided below, for this flip alone. Cleared first, so a refusal on any path below
    // leaves nothing behind for the funnel. swap_before_interior() clears them too.
    {
        SwapSurfaceSides& sides = m_swap_sides.local();
        sides.worthwhile = false;
        sides.saw_case = false;
    }
    m_swap_record.local().clear(); // see SwapRecord

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
    // followed through the rest of the swap by [flip funnel].
    const bool offset_flip =
        face_is_offset_surface_live(ftup_abc) && face_is_offset_surface_live(ftup_abd);
    if (offset_flip) {
        SwapSurfaceSides& sides = m_swap_sides.local();
        sides.worthwhile = true;
        sides.kind = static_cast<int>(tids.size());
        ++funnel_offered;
        if (sides.kind >= 3 && sides.kind <= 5) ++funnel_kind[size_t(sides.kind - 3)];
    }
    // THE SWAP GUARD's before-half, as in swap_before_interior(): the replaced cells' sum of E_T.
    // A flip changes no cell outside its ring -- E_T is a volume integral, so a cell beyond a face
    // the flip hands to the other side keeps its energy.
    m_swap_energy_before.local() = energy_sum(tids);
    // What the 4-4 / 5-6 case search scores candidate cells from; see SwapRecord. After
    // swap_capture_surface_sides(), whose sides the new cells take.
    if (current_op_kind() != OpKind::swap_32)
        swap_record_fill(tids, true); // as in swap_before_interior()

    // Non-offset surface flips are not refused categorically: the shared swap checks both new
    // triangles with surface_triangle_is_outside(), which holds a held face to m_envelope, and
    // that envelope is the geometric constraint. The class-match and same-boundary refusals above
    // are the topology half.
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

void TopoOffsetTetMesh::stamp_rest_cell(const size_t tid)
{
    if (!cell_is_plastic(tid)) return;
    const auto vs = oriented_tet_vids(tid);
    TetAttributes& x = m_tet_attribute[tid];
    for (int i = 0; i < 4; ++i) x.rest_pos[size_t(i)] = m_vertex_attribute[vs[size_t(i)]].m_posf;
    x.rest_valid = true;
}

void TopoOffsetTetMesh::stamp_plastic_rests()
{
    if (!m_plastic_active) return;
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (!cell_is_plastic(tid)) continue;
        const auto vs = oriented_tet_vids(tid);
        TetAttributes& x = m_tet_attribute[tid];
        for (int i = 0; i < 4; ++i) {
            x.rest_pos[size_t(i)] = m_vertex_attribute[vs[size_t(i)]].m_posf;
        }
        x.rest_valid = true;
    }
}

void TopoOffsetTetMesh::smooth_passes(const int k)
{
    // The rests are stamped once before the block, not between its passes, so the block's
    // rest-shape term resists everything the block moves. A no-op off the plastic medium.
    stamp_plastic_rests();
    log_energy_step("stamp");
    for (int i = 0; i < k; ++i) {
        local_operations({{0, 0, 0, 1}});
        log_energy_step("smooth");
    }
}

TopoOffsetTetMesh::CollapseSets TopoOffsetTetMesh::collapse_sets(const size_t v1, const size_t v2)
    const
{
    CollapseSets s;
    s.before = get_one_ring_tids_for_vertex(v1);
    for (const size_t tid : s.before) {
        const auto vs = oriented_tet_vids(tid);
        if (std::find(vs.begin(), vs.end(), v2) == vs.end()) s.after.push_back(tid);
    }
    return s;
}

double TopoOffsetTetMesh::swap_edge_44_energy(
    const std::vector<std::array<size_t, 4>>& tets,
    const int op_case)
{
    // See the declaration: sums of E_T, double::max() for an inverted candidate as the base does.
    // The current cells are the ring the before-half already summed (swap_before_interior() /
    // swap_before_surface(), the same cells the engine passes here as old_tets_conn), so the
    // score is read, not recomputed.
    double e = 0.;
    if (op_case == 0) {
        e = m_swap_energy_before.local();
    } else {
        // Every candidate cell's inversion test before any candidate's energy.
        bool inverted = false;
        for (const std::array<size_t, 4>& vids : tets) inverted = inverted || is_inverted(vids);
        if (inverted) {
            e = std::numeric_limits<double>::max();
        } else {
            for (const std::array<size_t, 4>& vids : tets) e += candidate_energy(vids);
        }
    }
    if (op_case >= 1) {
        // The engine takes the strictly lowest candidate and does not say which: the lowest is
        // what it commits, if it commits (see SwapRecord::scored_energy).
        SwapRecord& rec = m_swap_record.local();
        rec.scored = true;
        rec.scored_energy = std::min(rec.scored_energy, e);
    }
    SwapSurfaceSides& sides = m_swap_sides.local();
    if (!sides.worthwhile) return e;
    if (op_case == 0) {
        sides.case0_energy = e; // the current cells: the score every case has to beat
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

bool TopoOffsetTetMesh::swap_face_before(const Tuple& t)
{
    // DUPLICATED from TetOptimizerMesh::swap_face_before() (src/wmtk/TetOptimizerMeshSwaps.cpp),
    // step for step and reject kind for reject kind, because the engine has no energy hook for
    // the face swap's gate. Three changes, marked CHANGED: swap_before_interior() runs before the
    // gate instead of after it, since it fills the record candidate_energy() reads; the gate is
    // the E_T guard on the sum over the three new cells; and every new cell is tested for
    // inversion before any is scored. See the declaration.
    //
    // Every refusal below is counted in the per-kind table alone (swap_reject_kind_only()), so
    // the edge-swap line swap_reject_report() prints is untouched by face swaps.
    if (!TetMesh::swap_face_before(t)) {
        return swap_reject_kind_only(SwapReject::base_before);
    }

    auto& cache = swap_cache.local();
    cache.is_surface_flip = false;

    const SmartTuple tt(*this, t);

    const size_t fid = tt.fid();
    if (m_face_attribute[fid].m_is_surface_fs || m_face_attribute[fid].m_is_bbox_fs >= 0) {
        return swap_reject_kind_only(
            m_face_attribute[fid].m_is_surface_fs ? SwapReject::face_tracked_surface
                                                  : SwapReject::face_tracked_bbox);
    }
    const auto oppo_tet = tt.switch_tetrahedron();
    assert(oppo_tet.has_value() && "Should not swap boundary.");

    const size_t t0 = tt.tid();
    const size_t t1 = oppo_tet.value().tid();
    const std::vector<size_t> twotets{t0, t1};

    // CHANGED: before the gate (the base calls it after), with the same reject kind.
    if (!swap_before_interior(twotets)) {
        return swap_reject_kind_only(SwapReject::interior_hook);
    }

    // CHANGED: the sum of E_T over the three new cells, from the record, strictly below the two
    // cells' sum (swap_before_interior() has just taken it), in place of the stored AMIPS^3 and
    // get_quality().
    double scored = 0.;
    {
        const auto t1_vids = oriented_tet_vids(t1);

        const size_t v0 = tt.vid();
        const size_t v1 = tt.switch_vertex().vid();
        const size_t v2 = tt.switch_edge().switch_vertex().vid();
        const size_t v3 = tt.switch_face().switch_edge().switch_vertex().vid();

        const std::array<size_t, 3> tri{{v0, v1, v2}};

        std::vector<std::array<size_t, 4>> new_tets(3);
        for (size_t i = 0; i < 3; i++) {
            new_tets[i] = t1_vids;
            wmtk::array_replace_inline(new_tets[i], tri[i], v3);
        }
        // CHANGED: every new cell's inversion test before any new cell's energy.
        for (const std::array<size_t, 4>& new_tet : new_tets) {
            if (is_inverted(new_tet)) return swap_reject_kind_only(SwapReject::face_inverted);
        }
        for (const std::array<size_t, 4>& new_tet : new_tets) scored += candidate_energy(new_tet);
        if (!energy_lowers(scored, m_swap_energy_before.local())) {
            ++iter_cnt_swap_energy_reject;
            return swap_reject_kind_only(SwapReject::face_not_better);
        }
    }
    // perform_sanity_checks: the three cells' sum is what swap_after_cells() must read.
    SwapRecord& rec = m_swap_record.local();
    rec.scored = true;
    rec.scored_energy = scored;

    face_attribute_tracker(*this, twotets, m_face_attribute, cache.changed_faces);
    return true;
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

void TopoOffsetTetMesh::swap_scoring_check(const std::vector<size_t>& tids, const double after)
{
    // See SwapRecord::scored_energy. The case search and the face gate decided on the record's
    // prediction; the rule below reads the real cells. A difference is a cell scored on a number
    // it does not have -- a record that predicted a face wrong, or a field read differently.
    const SwapRecord& rec = m_swap_record.local();
    if (!rec.scored) return; // a 3-2: nothing was scored before it existed
    ++m_swap_scoring_checked;
    const double scale = std::max(std::abs(after), std::abs(rec.scored_energy));
    if (std::abs(after - rec.scored_energy) <= 1e-9 * scale) return; // a NaN is a mismatch
    const long long n = m_swap_scoring_mismatch++;
    if (n >= 8) return;
    logger().warn(
        "\t[swap scoring] {}: the {} new cells it picked (first tet {}) scored an energy sum "
        "{:.17g} before they existed and read {:.17g} on the mesh (rel {:.3g})",
        op_kind_name(current_op_kind()),
        tids.size(),
        tids.empty() ? size_t(0) : tids.front(),
        rec.scored_energy,
        after,
        std::abs(after - rec.scored_energy) / scale);
}

bool TopoOffsetTetMesh::swap_after_cells(const std::vector<size_t>& tids, bool is_surface_flip)
{
    // THE SWAP GUARD for every swap -- interior edge swaps, flips of a tracked surface, face swaps
    // (the energy spec): the sum of E_T over the cells the swap made must be STRICTLY below the
    // sum over the cells it replaced, as swap_before_interior() / swap_before_surface() cached it.
    // A swap moves no vertex and changes no cell outside its ring, so the two sums differ by
    // exactly the swap's change of E. Applied HERE on the real cells, once their labels are
    // written below. The 4-4 / 5-6 case search and the face swap's gate apply it earlier, to cells
    // that do not exist yet (candidate_energy()), and under perform_sanity_checks the two are
    // compared here. Strict, so a swap pass terminates.
    //
    // A refusal is rolled back by the engine. It is counted as app_sag_raised, the engine's name
    // for this after-hook refusal (SwapReject lives in the engine, which this does not change);
    // a face swap counts in the per-kind table alone, as the engine counts its own refusals.
    const auto energy_lowered = [&]() {
        const double after = energy_sum(tids);
        if (m_params.perform_sanity_checks) swap_scoring_check(tids, after);
        if (energy_lowers(after, m_swap_energy_before.local())) return true;
        ++iter_cnt_swap_energy_reject;
        if (current_op_kind() == OpKind::swap_face) {
            return swap_reject_kind_only(SwapReject::app_sag_raised);
        }
        return swap_reject(SwapReject::app_sag_raised);
    };

    if (!is_surface_flip) {
        // Interior: one tag and one label for the whole ring, captured by swap_capture_tag().
        const CellTag& tag = m_swap_tag.local();
        const int label = m_swap_label.local();
        for (const size_t t : tids) {
            m_tet_attribute[t].tag = tag;
            m_tet_attribute[t].label = label;
            m_tet_attribute[t].band_tri = -1; // as the swap's candidates were scored
            // A cell the swap creates is stamped at creation (its slot's old rest belongs to
            // another cell), before the guard reads it.
            stamp_rest_cell(t);
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
        m_tet_attribute[t].band_tri = -1; // as the swap's candidates were scored
        stamp_rest_cell(t); // created by the swap: stamped at creation, as above
    }
    if (sides.worthwhile) ++funnel_energy;
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
        return false;
    }
    const size_t v2_id = collapse_cache.local().v2_id;
    // The guard has run by now, inside the base's call above (collapse_after_connectivity(),
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
    if (!TetOptimizerMesh::collapse_edge_before(t)) {
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
        return collapse_reject(CollapseReject::app_substructure_link);
    }
    // The guard's before-half is taken in collapse_before_vertex().
    return true;
}

bool TopoOffsetTetMesh::collapse_before_vertex(
    const size_t v1_id,
    const size_t v2_id,
    const double edge_length)
{
    // THE cell sets of this collapse (CollapseSets), taken first and kept for every check after
    // this one, the after-hooks included.
    CollapseSets& sets = m_collapse_sets.local();
    sets = collapse_sets(v1_id, v2_id);
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
    // EXPERIMENTAL_no_collapse_length_gate lifts it: the energy rule alone judges the collapse.
    if (!m_collapse_limit_length && VE[v1_id].m_is_on_offset &&
        !m_offset_params.no_collapse_length_gate) {
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

    // THE COLLAPSE GUARD's before-half, last, so only a candidate every other test admitted pays
    // for it: the sum of E_T over the before cells, T_b. The after cells occupy the same region
    // of space and nothing outside it changes; collapse_after_connectivity() compares.
    m_collapse_energy_before.local() = energy_sum(sets.before);
    return true;
}

bool TopoOffsetTetMesh::collapse_after_connectivity(
    const size_t,
    const size_t v2_id,
    const std::vector<std::array<size_t, 2>>&)
{
    // THE COLLAPSE GUARD (the energy spec): the sum of E_T over the collapse's after cells (v1's
    // ring minus v2's, now holding v2 in v1's place) finite and not above the sum over its
    // before cells (v1's ring), CollapseSets, taken in collapse_before_vertex();
    // energy_not_raised().
    // They cover the same region of space and the mesh outside it is unchanged, so this is the
    // collapse's change of E.
    //
    // HERE: the connectivity is final, the cells the collapse keeps keep their slots and labels,
    // and a collapse moves no vertex, so the energy is read from the mesh as it now is. Refused
    // here, the base counts after_connectivity and rolls back.
    const CollapseSets& sets = m_collapse_sets.local();
    if (!energy_not_raised(energy_sum(sets.after), m_collapse_energy_before.local())) {
        ++iter_cnt_collapse_energy_reject;
        return false;
    }
    // Coarsening keeps an absolute bar besides, because it runs after the loop and trades
    // elements for nothing but the promise that the result is still good. As in 2D.
    // Over the live offset faces at v2 of the after cells: the faces the collapse reshaped.
    if (m_coarsen_mode && m_offset_potential) {
        double after = 0.;
        std::vector<std::array<size_t, 3>> seen;
        for (const size_t tid : sets.after) {
            for (int j = 0; j < 4; ++j) {
                const Tuple f = tuple_from_face(tid, j);
                std::array<size_t, 3> vs = get_face_vids(f);
                if (std::find(vs.begin(), vs.end(), v2_id) == vs.end()) continue;
                if (!face_is_offset_surface_live(f)) continue;
                std::sort(vs.begin(), vs.end());
                if (std::find(seen.begin(), seen.end(), vs) != seen.end()) continue;
                seen.push_back(vs);
                after = std::max(after, face_criterion_rel(f));
            }
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
    m_vertex_extra[v2_id].m_is_on_input =
        m_vertex_extra.at(v1_id).m_is_on_input || m_vertex_extra.at(v2_id).m_is_on_input;
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

    // Every cell at the midpoint was created by this split and the snapshot copy gave each the
    // parent's rest -- stamp at creation, or a plastic child measures itself against a tet twice
    // its size. Before the split guard reads the children (split_edge_after()).
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
    ++m_smooth_trace.attempted;
    // The final pass does not move the front: it is converged by then, and nothing follows to put
    // it back.
    if (m_freeze_front && m_vertex_extra[vid].m_is_on_offset) return false;

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
    solver->stop_criteria().relGradNorm = kSmoothRelGradNormTol;
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

    // THE smoother, for every vertex (smooth_vertex()). A front vertex reaches here only outside
    // the final pass: smooth_before() refuses it while m_freeze_front is set.
    const bool ok = smooth_vertex(t);
    if (ok && ve.m_is_on_offset) ++m_smooth_trace.offset_accepted;
    return ok;
}

double TopoOffsetTetMesh::front_vertex_normal_gradient(const size_t vid) const
{
    // ||grad E_V|| at the vertex's current position (vertex_energy()), taken along the move
    // direction where there is one.
    const auto energy = vertex_energy(vid);
    if (!energy) return std::numeric_limits<double>::infinity();
    const Vector3d x = m_vertex_attribute[vid].m_posf;
    Eigen::VectorXd xv = x, g(3);
    energy->gradient(xv, g);
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
    // yes/no, and a face resting on the skin of its tube is the one the next operation pushes
    // out. Both envelopes are real SampleEnvelopes, so squared_distance() is safe on them.
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
    const auto farthest = [](const SampleEnvelope& env, const std::array<Vector3d, 3>& p, int k) {
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
        const std::shared_ptr<SampleEnvelope> env = surface_envelope_for_face(vids);
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
    std::vector<double> sum(vert_capacity(), 0.), wsum(vert_capacity(), 0.);
    std::vector<size_t> n(vert_capacity(), 0);
    std::vector<char> bad(vert_capacity(), 0);
    for (const auto& f : offset_surface_faces()) {
        if (!front(f[0]) || !front(f[1]) || !front(f[2])) continue;
        const double term = face_offset_term(f[0], f[1], f[2]);
        const double wf = ring_face_weight(f[0], f[1], f[2]);
        for (const size_t u : f) {
            if (term < 0.) {
                bad[u] = 1;
            } else {
                sum[u] += wf * term;
                wsum[u] += wf;
                ++n[u];
            }
        }
    }
    std::vector<double> r(vert_capacity(), std::numeric_limits<double>::quiet_NaN());
    for (size_t v = 0; v < vert_capacity(); ++v) {
        if (!bad[v] && n[v] > 0) r[v] = std::sqrt(sum[v] / wsum[v]);
    }
    return r;
}

double TopoOffsetTetMesh::ring_measure_at(const size_t vid) const
{
    const auto front = [&](const size_t u) {
        return m_vertex_extra[u].m_is_on_offset && m_vertex_attribute[u].m_is_rounded;
    };
    double sum = 0., wsum = 0.;
    size_t n = 0;
    for (const Tuple& ft : offset_surface_faces_live_at(vid)) {
        const auto f = get_face_vids(ft);
        if (!front(f[0]) || !front(f[1]) || !front(f[2])) continue;
        const double term = face_offset_term(f[0], f[1], f[2]);
        if (term < 0.) return std::numeric_limits<double>::quiet_NaN();
        const double wf = ring_face_weight(f[0], f[1], f[2]);
        sum += wf * term;
        wsum += wf;
        ++n;
    }
    return n > 0 ? std::sqrt(sum / wsum) : std::numeric_limits<double>::quiet_NaN();
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
    // Per pass, after the base's own "newton, smooth_after" line (every vertex off the front;
    // smooth_vertex() records it there).
    logger().info("\tnewton, front: {}", m_newton_front.to_string());
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
        "\tfront vertices: {} attempted -> {} accepted",
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
        if (m_input_complex_bvh) {
            cand.dist = (cand.c - m_input_complex_bvh->nearest_point(cand.c)).norm();
        }
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
                        const std::shared_ptr<SampleEnvelope> env =
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
    return std::abs(m_input_complex_bvh->dist(VectorXd(p)) - m_offset_params.target_distance);
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
                const double d = m_input_complex_bvh->dist(VectorXd(p));
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
    // vertex placed when its own relative error is. One key for both since 2026-09-24. See
    // offset_envelope_rel for the leash on the operations, which is not an accuracy and which
    // startup requires to be no wider than this.
    s.tube = m_offset_params.front_conv;
    const auto front = [&](const size_t vid) {
        return m_vertex_extra[vid].m_is_on_offset && m_vertex_attribute[vid].m_is_rounded;
    };
    std::vector<char> placed(vert_capacity(), 0);
    // front_measure "vertex_ring": the ring measure is accumulated per corner from the face
    // loop's own face_offset_term() calls below, so it judges exactly the face numbers the face
    // mode does, every face weighted by ring_face_weight() (1, or its area under
    // EXPERIMENTAL_area_weighted_ring). See EnergyCriterion::ring_exit.
    s.ring_exit = m_offset_params.front_measure == "vertex_ring";
    s.unreachable_exit = s.ring_exit && m_offset_params.unreachable_exit;
    // EXPERIMENTAL_unreachable_exit: sums over the ring of the faces' mean errors and remainders.
    std::vector<double> ring_mean, ring_rem;
    if (s.unreachable_exit) {
        ring_mean.assign(vert_capacity(), 0.);
        ring_rem.assign(vert_capacity(), 0.);
    }
    // ring_w: the sum of the faces' ring_face_weight(), the denominator of every ring mean.
    std::vector<double> ring_sum, ring_w;
    std::vector<size_t> ring_n;
    std::vector<char> ring_bad;
    if (s.ring_exit) {
        ring_sum.assign(vert_capacity(), 0.);
        ring_w.assign(vert_capacity(), 0.);
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
        double mean_e = 0., rem = 0.;
        const double term = face_offset_term(
            va,
            vb,
            vc,
            s.unreachable_exit ? &mean_e : nullptr,
            s.unreachable_exit ? &rem : nullptr);
        const double gn = term < 0. ? -1. : std::sqrt(term); // the sag / the tube
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
            const double wf = ring_face_weight(va, vb, vc);
            for (const size_t u : {va, vb, vc}) {
                ring_sum[u] += wf * term;
                ring_w[u] += wf;
                ++ring_n[u];
                if (s.unreachable_exit) {
                    ring_mean[u] += wf * mean_e;
                    ring_rem[u] += wf * rem;
                }
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
                const double target = front_chord_target(la, lb, len, gn * s.tube, s.tube);
                const double sn =
                    std::clamp(target / l, s_floor, m_offset_params.max_sizing_scalar);
                if (sn < have) {
                    s.refinable.push_back({la, lb, lc, gn * s.tube, len});
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
            const double r = std::sqrt(ring_sum[vid] / ring_w[vid]);
            ++s.n_rings;
            s.sum_ring += r;
            if (r > s.max_ring) {
                s.max_ring = r;
                s.worst_ring_vid = vid;
            }
            if (!(r > s.bar)) continue; // within the bar (rings_ok(): max_ring <= bar)
            ++s.n_rings_over;
            if (s.unreachable_exit) {
                // W, the mean over the ring's faces of their remainders: the part of the error's
                // variation over each face that no linear function on the face matches, the part
                // a finer front can follow. A linear variation is either a tilt smoothing removes
                // or, where the level set is out of reach, the distance rising along a front that
                // cannot follow it; refining only shrinks the rings there (slot_small, target
                // 1.35, the channel above the plate: 86-99.5% of the variance was linear, and the
                // variance rule refined 177, 371, 882, 2184 vertices in turns 1-4 where W refines
                // 111, 134, 25, 0 on the same frames). A vertex whose W is within the bar is not
                // refined, and passes when smoothing no longer moves it: its solve in the turn's
                // last smoothing passes (the latest operation group) changed its own error by at
                // most the bar. With no front solve in that group there is nothing to judge, and
                // it does not pass.
                const double mean = ring_mean[vid] / ring_w[vid];
                const double w_rem = ring_rem[vid] / ring_w[vid];
                if (!(w_rem > s.bar * s.bar)) {
                    const VertexExtra& ve = m_vertex_extra[vid];
                    const bool solved = ve.m_front_change_group == m_smooth_group;
                    if (!solved) ++s.n_rings_unsolved;
                    const double change =
                        solved ? ve.m_front_change : std::numeric_limits<double>::infinity();
                    if (change <= s.bar) {
                        ++s.n_rings_unreached;
                        s.max_unreached_mean = std::max(s.max_unreached_mean, std::abs(mean));
                    } else {
                        ++s.n_rings_unsettled;
                        if (s.n_rings_unsettled == 1 || change > s.max_unsettled_change) {
                            s.max_unsettled_change = change;
                            s.worst_unsettled_pos = m_vertex_attribute[vid].m_posf;
                        }
                    }
                    continue;
                }
            }
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

std::string TopoOffsetTetMesh::EnergyCriterion::unreached_fact() const
{
    if (!unreachable_exit || n_rings_unreached == 0) return "";
    return fmt::format(
        " or, at {} vertex(es) over it, the level set out of reach (mean error up to {:.4} bars)",
        n_rings_unreached,
        max_unreached_mean);
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
    // with an input-complex vertex, whose piece is read off the per-piece BVHs (the nearest
    // piece to a vertex ON the complex is its own, at distance 0). As in 2D, a cell reached from
    // two regions and a vertex on cells of two regions read -2 and fall back to the union field.
    m_cell_region.assign(tet_capacity(), -1);
    m_vertex_region.assign(vert_capacity(), -1);
    if (m_n_regions <= 1 || m_region_potentials.empty() || m_region_bvhs.empty()) return;
    const auto region_at = [&](const Vector3d& p) -> int {
        int best = -1;
        double best_d = std::numeric_limits<double>::infinity();
        for (size_t r = 0; r < m_region_bvhs.size(); ++r) {
            const double d = m_region_bvhs[r]->squared_dist(VectorXd(p));
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

void TopoOffsetTetMesh::log_front_profile(const size_t vid)
{
    if (vid == static_cast<size_t>(-1) || vid >= m_vertex_attribute.size() || !m_offset_potential)
        return;
    const int region = vertex_region(vid);
    const std::shared_ptr<const OffsetPotential3D> pot = potential_ptr_for(vid);
    const Vector3d x0 = m_vertex_attribute[vid].m_posf;
    Vector3d g = pot->gradient(x0);
    if (!(g.norm() > 0.) || !g.allFinite()) return;
    const Vector3d n = g / g.norm();
    auto stencil = vertex_energy(vid);
    if (!stencil) return; // no valid cell at vid
    OffsetEnergy3D own_point(pot, offset_term_weight(), true, true);
    const double delta = m_offset_params.target_distance;
    logger().info(
        "[front profile] worst vertex {} at ({:.5}, {:.5}, {:.5}), region {}, along the field "
        "direction n = ({:.4}, {:.4}, {:.4}); columns: s/delta | own-point term (the vertex's "
        "squared residual at its own position, in units of the bar) | E_V minus own point "
        "| E_V (vertex_energy(): the one-ring's sum of E_T)",
        vid,
        x0.x(),
        x0.y(),
        x0.z(),
        region,
        n.x(),
        n.y(),
        n.z());
    for (int k = -10; k <= 10; ++k) {
        const double sd = 0.05 * k;
        const Vector3d x = x0 + sd * delta * n;
        Eigen::VectorXd xv(3);
        xv << x.x(), x.y(), x.z();
        const double F = stencil->value(xv);
        const double Fo = own_point.value(xv);
        logger().info("[front profile] {:+.2f} | {:.6g} | {:.6g} | {:.6g}", sd, Fo, F - Fo, F);
    }
}

double TopoOffsetTetMesh::front_chord_target(
    const size_t va,
    const size_t vb,
    const double len,
    const double sag,
    const double tube) const
{
    // 3/4 L (tube / sag)^(1/p), capped at L/2, with the exponent p measured from how the level
    // set turns across the chord: 2 on a smooth level set, 1 where the chord straddles a kink.
    // See the 2D twin for the derivation. The sag fraction a halving leaves is ratio = 2^-p, so
    // p = -log2(ratio): 1/4 -> 2, 1/2 -> 1. (The 2D twin writes -1 / log2(ratio), which maps
    // 1/4 to 0.5 and 1/2 to 1 and so over-refines every smooth chord by (sag / tube)^(1..2)
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
    return std::min(0.75 * len * std::pow(tube / sag, 1. / p), 0.5 * len);
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

TopoOffsetTetMesh::SmoothingProgress TopoOffsetTetMesh::smoothing_progress(
    const std::vector<Vector3d>& before)
{
    SmoothingProgress s;
    const double l = std::max(m_params.l, 1e-16);
    // A front vertex's STEP against the one bar; the background uses its own sizing target below.
    const double tube = m_offset_params.front_conv;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        const Vector3d& x = m_vertex_attribute[vid].m_posf;
        const double step = vid < before.size() ? (x - before[vid]).norm() : 0.;
        const bool front =
            m_vertex_extra[vid].m_is_on_offset && m_vertex_attribute[vid].m_is_rounded;
        if (!front) {
            ++s.n_background;
            const double target = std::max(m_vertex_attribute[vid].m_sizing_scalar * l, 1e-300);
            const double r = step / target;
            if (r > s.background_max_step) {
                s.background_max_step = r;
                s.background_worst_vid = vid;
            }
            continue;
        }
        if (tube > 0.) s.front_max_step = std::max(s.front_max_step, step / tube);
        const double gn = front_vertex_conv_ratio(vid);
        if (!std::isfinite(gn)) {
            ++s.n_front_unmeasurable;
            continue;
        }
        ++s.n_front;
        if (gn > s.front_max_ratio) {
            s.front_max_ratio = gn;
            s.front_worst_vid = vid;
        }
    }
    return s;
}

void TopoOffsetTetMesh::smooth_group_to_convergence(const char* group_name)
{
    const int max_passes = std::max(1, m_offset_params.adaptive_smoothing_max_passes);
    const double stall_rel = m_offset_params.adaptive_smoothing_stall_rel;
    const double step_rel = m_offset_params.adaptive_smoothing_step_rel;
    std::vector<Vector3d> before;
    double prev_front = std::numeric_limits<double>::infinity();
    // The plastic rests are stamped once before the block, as before every smoothing block.
    stamp_plastic_rests();
    for (int p = 0; p < max_passes; ++p) {
        before.assign(vert_capacity(), Vector3d::Zero());
        for (const Tuple& v : get_vertices()) {
            const size_t vid = v.vid(*this);
            before[vid] = m_vertex_attribute[vid].m_posf;
        }
        // One pass through the same entry the fixed count used: the sweep, rounding, the
        // quality log and update_attributes(). The tube is NOT rebuilt between passes; the
        // group's caller rebuilds it once, as with the fixed count.
        local_operations({{0, 0, 0, 1}});
        const SmoothingProgress s = smoothing_progress(before);
        const bool front_converged = s.n_front == 0 || front_placed_by_ratio(s.front_max_ratio);
        const bool front_stalled = !front_converged && std::isfinite(prev_front) &&
                                   s.front_max_ratio > (1. - stall_rel) * prev_front;
        const bool background_settled = s.background_max_step <= step_rel;
        const char* front_verdict =
            front_converged ? "converged" : (front_stalled ? "stalled" : "moving");
        logger().info(
            "\t[smoothing {} pass {}/{}] front: max ratio {:.4} (prev {:.4}) at v{}, {} measured "
            "+ {} unmeasurable, max step {:.4} x tube -> {} | background: max step "
            "{:.4} x target edge at v{} over {} vertices -> {}",
            group_name,
            p + 1,
            max_passes,
            s.front_max_ratio,
            prev_front,
            s.front_worst_vid,
            s.n_front,
            s.n_front_unmeasurable,
            s.front_max_step,
            front_verdict,
            s.background_max_step,
            s.background_worst_vid,
            s.n_background,
            background_settled ? "settled" : "moving");
        if ((front_converged || front_stalled) && background_settled) break;
        prev_front = s.front_max_ratio;
    }
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
    // Ported statement for statement from tetwild::TetWild::adjust_sizing_field
    // (attic/app/tetwild/TetWild.cpp): the same R, the same ramp, the same breadth-first walk
    // that stops at R, the same floor. Two parts of that function are NOT ported on purpose:
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
    // same way, seeding the same force-split edges. Final pass only, by construction:
    // mesh_improvement() is this function's one caller, and the driver only ever runs that for
    // the frozen-front finishing pass.
    const int n_rings = std::max(0, m_params.stuck_refine_rings);
    const double filter_energy = std::min(std::max(max_metric / 100., m_params.stop_energy), 100.);

    // cell_quality is AMIPS^3, so the energy "max energy" refers to is its cube root.
    const auto worst = wmtk::utils::select_worst_cells(
        tet_capacity(),
        [this](size_t tid) { return tuple_from_tet(tid).is_valid(*this); },
        [this](size_t tid) { return std::cbrt(cell_quality(tid)); },
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

    const Vector3d p = m_vertex_attribute[vid].m_posf;
    const double d = (p - m_input_complex_bvh->nearest_point(p)).norm();

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
    const char* fate = "smooth_vertex(): the one-ring's sum of E_T";
    if (!m_vertex_attribute[vid].m_is_rounded) {
        fate = "REFUSED by smooth_before: not rounded";
    } else if (m_freeze_front && ve.m_is_on_offset) {
        fate = "REFUSED by smooth_before: front frozen in the final pass";
    } else if (ve.m_is_on_offset) {
        fate = "smooth_vertex(): the one-ring's sum of E_T, the band term included";
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

bool TopoOffsetTetMesh::vertex_has_live_offset_face(const size_t vid) const
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
            if (cell_is_offset_band(ta)) return true; // band face on the domain boundary
        } else {
            const size_t tb = sides[i + 1].tid;
            const bool a = cell_is_offset_band(ta), b = cell_is_offset_band(tb);
            if (a != b && !cell_is_input_complex(a ? tb : ta)) return true;
        }
        i = j;
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

/// The outer-angle threshold, in degrees, above which an offset-surface edge counts as folded
/// over: the angle between its two faces measured through ONE of the two sides. 180 is flat and
/// 360 is the two faces exactly on top of each other. Because the two sides sum to 360, "over
/// 330 on one side" is the same statement as "under 30 unsigned", which is what the code tests --
/// see offset_surface_foldover_labels() for why the side is not determined.
/// Optimize2d.cpp carries the same constant; the two values must stay equal.
static constexpr double FOLDOVER_OUTER_ANGLE_DEG = 330.;

std::vector<char> TopoOffsetTetMesh::offset_surface_foldover_labels() const
{
    std::vector<char> fold(vert_capacity(), 0);

    // Every live offset face against each of its three edges, so an edge arrives with the faces
    // that actually carry it. A std::map keyed on the sorted vertex pair: the surface is a small
    // part of the mesh and this runs once per debug frame.
    struct EdgeFaces
    {
        int n = 0;
        std::array<size_t, 2> opposite{{0, 0}}; // the two faces' third vertices
    };
    std::map<std::array<size_t, 2>, EdgeFaces> edges;
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        const auto fv = get_face_vids(f);
        for (int i = 0; i < 3; ++i) {
            const size_t a = fv[i], b = fv[(i + 1) % 3], c = fv[(i + 2) % 3];
            std::array<size_t, 2> key{{a, b}};
            if (key[0] > key[1]) std::swap(key[0], key[1]);
            EdgeFaces& ef = edges[key];
            if (ef.n < 2) ef.opposite[size_t(ef.n)] = c;
            ++ef.n; // counted past 2 on purpose, so a non-manifold edge can be recognised
        }
    }

    // A fold is the two faces lying on top of each other, and WHICH side is pinched is not part
    // of it. MEASURED, on the cube at target_distance_rel 5e-2 with the construction probe off
    // and the alignment term off: at every folded edge in that run it is the BACKGROUND that is
    // pinched, not the band -- the two band tets sit at angles like 92 and 265 degrees around
    // the edge, on either side of a sliver of outside a fraction of a degree wide. Measuring
    // through the background there gives 0.67 degrees, not 359.33. So the side is deliberately
    // not determined: the two sides sum to 360, "over 355 through one side" is exactly "under 5
    // unsigned", and testing the unsigned angle catches the fold whichever wedge collapsed.
    // This also drops the band-apex orientation the first version needed, which was the part
    // that could be got wrong.
    const double coincidence_deg = 360. - FOLDOVER_OUTER_ANGLE_DEG;
    for (const auto& [e, ef] : edges) {
        // Not two faces means the angle is not defined -- a domain-boundary rim, or a
        // non-manifold edge. Not measurable is not a fold.
        if (ef.n != 2) continue;
        const Vector3d pa = m_vertex_attribute[e[0]].m_posf;
        const Vector3d pb = m_vertex_attribute[e[1]].m_posf;
        Vector3d dir = pb - pa;
        const double elen = dir.norm();
        if (!(elen > 0.)) continue;
        dir /= elen;
        // Each face's in-plane direction away from the edge, so the angle between them is the
        // angle around the edge and not the angle between two arbitrary chords.
        const auto perp = [&](const size_t opp_vid, Vector3d& u) -> bool {
            const Vector3d w = m_vertex_attribute[opp_vid].m_posf - pa;
            u = w - w.dot(dir) * dir;
            const double len = u.norm();
            if (!(len > 0.) || !std::isfinite(len)) return false;
            u /= len;
            return true;
        };
        Vector3d u0, u1;
        if (!perp(ef.opposite[0], u0) || !perp(ef.opposite[1], u1)) continue;
        const double ang = std::acos(std::clamp(u0.dot(u1), -1., 1.)) * 180. / M_PI; // [0, 180]
        if (ang < coincidence_deg) {
            fold[e[0]] = 1;
            fold[e[1]] = 1;
        }
    }
    return fold;
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
        if (m_input_complex_bvh->dist(VectorXd(m_vertex_attribute[vid].m_posf)) >
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
        const double d = m_input_complex_bvh->dist(VectorXd(m_vertex_attribute[vid].m_posf));
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

    // The leash as init() resolved it: absolute if the config gave one, else offset_envelope_rel
    // x the bbox diagonal. Referenced to the BOX, not to target_distance, since 2026-09-24. As
    // in 2D.
    const double eps = std::max(m_offset_params.offset_envelope, 1e-12);

    m_offset_envelope = std::make_shared<SampleEnvelope>(/*exact=*/true);
    m_offset_envelope->init(verts, tris, eps);
    logger().info(
        "\t[offset envelope] built: {} faces, {} (eps {:.6g} = offset_envelope, "
        "{:.4} x the bbox diagonal)",
        tris.size(),
        m_offset_envelope->use_exact ? "EXACT" : "sampled",
        eps,
        m_offset_params.offset_envelope_rel);
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
    const std::string base = m_offset_params.output_path + fmt::format("_{:05d}", idx);
    write_vtu(base);
    m_front_solve_log.clear(); // written; the next frame shows the solves after this one
    // Record what this frame actually wrote, then refresh the ParaView collections. Which
    // companions exist is dimension-specific and some are conditional, so they are discovered
    // from disk rather than hard-coded here.
    if (m_debug_frame_labels.size() <= idx) m_debug_frame_labels.resize(idx + 1);
    m_debug_frame_labels[idx] = label;
    for (const char* sfx : {"", "_surf", "_off", "_edge", "_front"}) {
        const std::string p = base + sfx + ".vtu";
        if (!debug_frame_file_exists(p)) continue;
        // The label goes INTO the frame as FieldData, not only into the .pvd: a .pvd DataSet's
        // name= attribute does reach the reader, but as a vtkCharArray, which ParaView's
        // annotation renders as the first character's numeric code rather than the text, and an
        // XML comment is discarded outright. See inject_frame_label().
        inject_frame_label(p, label);
        m_debug_pvd_series[sfx].push_back(idx);
    }
    write_debug_pvd();
}

void TopoOffsetTetMesh::write_debug_pvd() const
{
    // DEBUG_output only. ParaView detects a file series only when the frame index sits
    // IMMEDIATELY before the extension. The main frames are <output>_NNNNN.vtu and group fine,
    // but every companion is <output>_NNNNN_off.vtu -- index in the middle, suffix after it --
    // so ParaView opens each companion as its own dataset instead of one time series. A .pvd
    // collection names the files explicitly, which sidesteps the naming rule entirely.
    // Rewritten after EVERY frame, not once at the end: these runs are killed often, and a
    // killed run should still leave a series that opens.
    const std::string& out = m_offset_params.output_path;
    // file= is resolved relative to the .pvd, so it carries the bare name, not output_path.
    const std::string stem = out.substr(out.find_last_of("/\\") + 1);
    for (const auto& [sfx, idxs] : m_debug_pvd_series) {
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
            const std::string lab = i < m_debug_frame_labels.size() ? m_debug_frame_labels[i] : "";
            if (!lab.empty() && lab.find("--") == std::string::npos) {
                f << fmt::format("  <!-- {} -->", lab);
            }
            f << "\n";
        }
        f << "  </Collection>\n</VTKFile>\n";
    }
}

bool TopoOffsetTetMesh::optimize_offset_loop()
{
    // One loop: TetWild's operation groups (split / collapse / swap, each followed by smoothing)
    // with the front placed by the offset objective inside the smoothing passes. No offset tube
    // holds the front in the loop, neither the operations nor the smoothing; only the frozen-front
    // final pass is held to one (build_offset_envelope()).
    const int rounds = std::max(1, m_offset_params.max_rounds);
    const int a_iters = std::max(1, m_offset_params.max_iterations);
    logger().info(
        "\t[energy] E_T(t) = V_t (w A(t)^3 / SE^3 + [t in band] (1 - w) D(t)), w = w_amips {:.6g}, "
        "SE = stop_energy {:.6g}, A against {} "
        "| "
        "split and collapse: the sum of E_T over the cells they change must be finite and not "
        "rise; swap: finite and strictly fall | smoothing: every vertex minimises its one-ring's "
        "sum of E_T, every move kept only if that sum is finite and does not rise",
        m_offset_params.w_amips,
        m_params.stop_energy,
        m_offset_params.use_rest_pose
            ? "the rest shape outside the band and the input complex (use_rest_pose)"
            : "the regular tet everywhere (use_rest_pose false)");
    check_no_vertex_on_both_surfaces("construction");
    check_no_vertex_on_both_surfaces("construction");
    audit_surface_containment("construction");
    needle_scan("after construction, before the loop");
    assign_band_regions();
    m_front_gradient_reference = front_gradient_linf();
    logger().info(
        "\tLOOP: TetWild's operation groups with the front placed inside their "
        "smoothing passes | front energy-gradient reference {:.6g} | ONE criterion: the RMS "
        "relative error over a stencil_order {} stencil ({} points per face) against front_conv "
        "{:.6g} ({:.6g} x the bbox diagonal)",
        m_front_gradient_reference,
        m_offset_params.stencil_order,
        stencil_points_per_face(),
        m_offset_params.front_conv,
        m_offset_params.front_conv_rel);
    logger().info(
        "\t[offset envelope] the loop's operations and smoothing do not hold the offset surface "
        "to an envelope; the final pass does (eps {:.6g})",
        m_offset_params.offset_envelope);
    (void)rounds;
    const int budget = std::max(1, m_offset_params.max_rounds);
    // One turn is TetWild's operation groups, run here rather than through mesh_improvement().
    // What mesh_improvement() adds and is left out here on purpose is its stall response, which
    // refines around the worst elements: a moving front stretches cells by design.
    // k is the fixed count when adaptive_smoothing is off; on, each group smooths until the
    // front and the background settle (smooth_group_to_convergence()).
    // interleaved_smoothing true is TetWild's shape of a turn: three groups, each one operation
    // pass followed by k smoothing passes. With it false, this component's default, a turn is ONE
    // group -- split, collapse and swap back to back -- followed by one smoothing block of
    // num_smoothing_passes (or adaptive). Why the switch reaches the loop: measured on the
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
        "\tTurn shape: {}",
        interleaved
            ? fmt::format(
                  "INTERLEAVED -- split, collapse, swap, each followed by {}",
                  m_offset_params.adaptive_smoothing ? std::string("adaptive smoothing")
                                                     : fmt::format("{} smoothing pass(es)", k))
            : fmt::format(
                  "COMBINED -- split, collapse, swap back to back, then {} (interleaved_smoothing "
                  "false)",
                  m_offset_params.adaptive_smoothing ? std::string("adaptive smoothing")
                                                     : fmt::format("{} smoothing pass(es)", k)));
    compute_vertex_partition_morton();
    // One turn of grace after the field is lowered: refine_front_by_halving() lowers sizing
    // scalars at the end of a turn, and the split pass
    // that realizes them does not run until the NEXT turn.
    if (m_offset_params.pre_smooth) {
        // One smoothing block on the constructed mesh before turn 1's split pass: the same
        // block every operation group is followed by, with the same bookkeeping around it
        // (plastic rests stamped before each pass). Frames are r0S*.
        m_round = 0;
        logger().info(
            "\t[pre_smooth] one smoothing block before turn 1: {}",
            m_offset_params.adaptive_smoothing
                ? std::string("adaptive smoothing")
                : fmt::format("{} interleaved smoothing pass(es)", k));
        if (m_offset_params.adaptive_smoothing) {
            smooth_group_to_convergence("pre_smooth");
        } else {
            smooth_passes(k);
        }
    }
    op_accounting_reset(); // the [ops accounting] lines are per turn, from turn 1's first op
    m_split_order_waits = 0; // and so are the [split order] lines
    m_split_off_longest = 0;
    m_swap_scoring_checked = 0; // and the [swap scoring] lines
    m_swap_scoring_mismatch = 0;
    for (int it = 0; it < budget; ++it) {
        m_round = it + 1;
        m_iterations_used = it + 1;
        const int energy_p0 = iter_cnt_split_energy_reject.load();
        const int energy_c0 = iter_cnt_collapse_energy_reject.load();
        const int energy_s0 = iter_cnt_swap_energy_reject.load();
        for (size_t gi = 0; gi < groups.size(); ++gi) {
            ++m_smooth_group; // EXPERIMENTAL_unreachable_exit: see VertexExtra::m_front_change
            // E before and after each group: the descent check (no operation and no smoothing move
            // raises it, so neither may a group; the restamp before it changes E's plastic part).
            stamp_plastic_rests(); // plastic: every operation block starts from its own shape
            // Every band cell stores its D(t) minimiser before the split pass, so the pass's
            // children can use their parent's (E unchanged).
            if (gi == 0 && m_band_tris) refresh_band_tris();
            const double energy_before_group = total_energy();
            log_energy_step("stamp");
            double energy_after_ops = energy_before_group;
            if (gi == 1) needle_scan("collapse pass");
            if (!interleaved) needle_scan("combined ops pass");
            if (m_offset_params.adaptive_smoothing) {
                // The group's operations alone, then its smoothing pass by pass until the front
                // and the background have settled -- see smooth_group_to_convergence().
                local_operations(
                    {{groups[gi][0], groups[gi][1], groups[gi][2], 0}},
                    !m_offset_params.no_collapse_length_gate);
                energy_after_ops = total_energy();
                log_energy_step(group_names[gi]);
                smooth_group_to_convergence(group_names[gi]);
            } else {
                // The group's operations, then its k smoothing passes, each stamped first.
                local_operations(
                    {{groups[gi][0], groups[gi][1], groups[gi][2], 0}},
                    !m_offset_params.no_collapse_length_gate);
                energy_after_ops = total_energy();
                log_energy_step(group_names[gi]);
                smooth_passes(groups[gi][3]);
            }
            // Per group, so a containment violation is attributed to the pass that made it
            // rather than found at the end of the run. Same gate as the shared sanity check.
            if (m_params.perform_sanity_checks) {
                audit_surface_containment(fmt::format("turn {} after {}", it + 1, group_names[gi]));
            }
            const double energy_after_group = total_energy();
            logger().info(
                "\t[energy] turn {} {} group: E {:.10g} -> {:.10g} ({:+.4g}; operations {:+.4g}, "
                "smoothing {:+.4g})",
                it + 1,
                group_names[gi],
                energy_before_group,
                energy_after_group,
                energy_after_group - energy_before_group,
                energy_after_ops - energy_before_group,
                energy_after_group - energy_after_ops);
        }
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
            if (ec.unreachable_exit) {
                logger().info(
                    "\t[unreachable exit] turn {}: of {} front vertices over the bar, {} with the "
                    "remainder over the bar ({} refinable, {} at the sizing floor); {} with the "
                    "remainder "
                    "within it pass, the level set out of reach (mean error up to {:.4} bars); {} "
                    "with the remainder within it still move and block the exit (own-error change "
                    "in "
                    "the last smoothing passes up to {:.4} bars, worst at ({:.4}, {:.4}, {:.4}); "
                    "{} of them not solved in those passes)",
                    it + 1,
                    ec.n_rings_over,
                    ec.refinable_vertices.size() + ec.n_rings_at_floor,
                    ec.refinable_vertices.size(),
                    ec.n_rings_at_floor,
                    ec.n_rings_unreached,
                    ec.max_unreached_mean,
                    ec.n_rings_unsettled,
                    ec.max_unsettled_change,
                    ec.worst_unsettled_pos.x(),
                    ec.worst_unsettled_pos.y(),
                    ec.worst_unsettled_pos.z(),
                    ec.n_rings_unsolved);
            }
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
        if (ec.n_at_floor > 0) {
            logger().log(
                ec.n_corners_at_floor > 0 ? spdlog::level::warn : spdlog::level::info,
                "\t[sizing floor] turn {}: {}",
                it + 1,
                ec.sizing_floor_fact());
        }
        if (ec.ring_exit && ec.n_rings_at_floor > 0) {
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
        logger().info(
            "\t[energy guard] turn {}: refused {} split(s), {} collapse(s) and {} swap(s): the "
            "sum of E_T over the cells they change not finite, or above (swap: not strictly below)",
            it + 1,
            iter_cnt_split_energy_reject.load() - energy_p0,
            iter_cnt_collapse_energy_reject.load() - energy_c0,
            iter_cnt_swap_energy_reject.load() - energy_s0);
        // perform_sanity_checks only: the case search's and the face gate's scores of the cells
        // they picked, against the same cells read on the mesh (swap_scoring_check()). Each
        // mismatch warned as it happened, the first 8 per turn.
        if (m_params.perform_sanity_checks) {
            const long long checked = m_swap_scoring_checked.exchange(0);
            const long long bad = m_swap_scoring_mismatch.exchange(0);
            logger().log(
                bad > 0 ? spdlog::level::warn : spdlog::level::info,
                "\t[swap scoring] turn {}: {} of {} swap(s) scored before their cells existed "
                "read differently on the mesh{}",
                it + 1,
                bad,
                checked,
                bad > 8 ? fmt::format(" ({} not listed)", bad - 8) : std::string());
        }
        // EXPERIMENTAL_no_refinement: no halving below; EXPERIMENTAL_no_exit: no exit after it.
        const bool refine_now = !m_offset_params.no_refinement;
        const bool exit_now = !m_offset_params.no_exit;
        if (!refine_now && (!ec.refinable.empty() || (ec.ring_exit && ec.n_rings_over > 0))) {
            logger().info(
                "\t[resolution] turn {}: EXPERIMENTAL_no_refinement -- no refinement ({} front "
                "vertex(es) / {} face(s) over the bar left as they are)",
                it + 1,
                ec.n_rings_over,
                ec.refinable.size());
        }
        if (refine_now && !ec.refinable.empty()) {
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
        // Under EXPERIMENTAL_unreachable_exit only the vertices whose remainder is over the bar are
        // refinable; the rest of those over the bar are on the [unreachable exit] line.
        const size_t n_to_refine = ec.unreachable_exit
                                       ? ec.refinable_vertices.size() + ec.n_rings_at_floor
                                       : ec.n_rings_over;
        if (refine_now && ec.ring_exit && n_to_refine > 0) {
            // front_measure "vertex_ring": the halving takes each vertex over the bar, that
            // vertex alone. refinable is empty in this mode, so the face line above is silent.
            const size_t n = refine_front_by_halving(ec.refinable_vertices);
            logger().info(
                "\t[resolution] turn {}: {} front vertex(es) whose {} (the RMS of the face "
                "measures of its incident offset faces, each face weighted equally) is over the "
                "bar, {} of them at the sizing floor (worst {:.4}x) -> sizing scalar halved at {} "
                "vertices",
                it + 1,
                n_to_refine,
                ec.ring_name(),
                ec.n_rings_at_floor,
                ec.max_ring,
                n);
        }
        // The turn's "end" frame is written HERE, after the refinement, not before it: it is the
        // turn's final state, so what it carries is the sizing field the halving just lowered.
        // 3D ONLY; 2D still writes its end frame before the refinement. See .claude/CLAUDE.md.
        if (m_offset_params.debug_output) {
            write_optimization_debug_output(fmt::format("end_{}S", it + 1));
        }
        // Termination: every offset face's measure within the bar and nothing unmeasurable
        // (EnergyCriterion::converged()) -- then quality with the front frozen (below). The face
        // measure is the one quantity the smoothing minimises and the refinement reads; the
        // vertex measure is reported on the turn line and in the verdict, and tested nowhere.
        // Nothing refinable is implied, a refinable face being over the bar. Under front_measure
        // "vertex_ring" the tested measure is every front vertex's ring measure instead, and the
        // face measure joins the vertex measure as a diagnostic (EnergyCriterion::ring_exit).
        //
        // The loop exits on the FIRST turn that meets the criterion. It used to additionally
        // demand that the previous turn lowered no sizing scalar -- one turn of hysteresis,
        // which given that the exit required an empty refinable set amounted to two
        // consecutive turns without a lowering, the second of them converged. That was dropped
        // because it cost many turns waiting for two to line up, and TetWild's loop takes the
        // same choice: it breaks the moment its max energy is under stop_energy, that number
        // being a property of the mesh it is holding, where refinable is a request for work on
        // the next turn.
        if (exit_now && ec.converged()) {
            m_energy_verdict = ec;
            m_converged = true;
            // Provisional: the final pass below overwrites both when it runs. The verdict at the
            // end of optimize_offset() requires this AND the front's.
            m_quality_max_amips = amips;
            m_quality_converged = amips < bar;
            if (ec.ring_exit) {
                logger().info(
                    "The front is resolved after {} iteration(s): every front "
                    "vertex's {} within the bar{} (rings max {:.4}x), nothing unmeasurable; offset "
                    "faces max {:.4}x with {} over the bar (diagnostic); front vertices max {:.4}x "
                    "(diagnostic); max AMIPS {:.4} against stop {:.4}",
                    it + 1,
                    ec.ring_name(),
                    ec.unreached_fact(),
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
                    "======== final pass, front frozen: max AMIPS {:.6g} >= stop_energy {} "
                    "========",
                    amips,
                    m_params.stop_energy);
                m_round = it + 2;
                // The final pass's envelopes -- the front's tube, and m_envelope widened to every
                // region boundary as placement left them (build_final_envelopes()). E_T still
                // guards every operation and smoothing move, every cell elastic: no plastic medium,
                // so A is against the regular tet, the quality the pass is driving down.
                build_final_envelopes();
                m_freeze_front = true;
                const bool plastic_was = m_plastic_active;
                m_plastic_active = false;
                mesh_improvement(a_iters);
                logger().info(
                    "\t[split order] final pass: waited {} | committed off the longest edge {}",
                    m_split_order_waits.exchange(0),
                    m_split_off_longest.exchange(0));
                m_plastic_active = plastic_was;
                m_freeze_front = false;
                release_final_envelopes();
                assign_band_regions();
                const double final_amips = std::get<0>(optimization_quality_stats());
                m_quality_max_amips = final_amips;
                m_quality_converged = final_amips < m_params.stop_energy;
                logger().log(
                    m_quality_converged ? spdlog::level::info : spdlog::level::warn,
                    "\t[final pass] max element quality {:.4} (stop {:.4}) -> {}",
                    final_amips,
                    optimization_stop_metric(),
                    m_quality_converged ? "ok" : "STILL OVER: the run does not converge");
                if (m_offset_params.debug_output) {
                    write_optimization_debug_output(fmt::format("end_{}F", it + 2));
                }
            }
            return true;
        }
    }
    logger().warn("The loop did not converge in {} turns (max_rounds)", budget);
    log_front_profile(energy_criterion().worst_vid);
    return false;
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

    // The plastic medium (cell_is_plastic()), under use_rest_pose: every cell outside the band and
    // the input complex measures its AMIPS against its rest shape, stamped here so the first
    // operations already have one. Off, every cell is measured against the regular tet.
    m_plastic_active = m_offset_params.use_rest_pose;
    stamp_plastic_rests();
    if (m_plastic_active) {
        logger().info(
            "[plastic] use_rest_pose true: AMIPS of every cell outside the band and the input "
            "complex against its rest shape: stamped now, before every operation group and every "
            "block of smoothing passes, and at creation for cells a split or swap makes");
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
    iter_cnt_split_energy_reject = 0;
    iter_cnt_collapse_energy_reject = 0;
    iter_cnt_swap_energy_reject = 0;
    m_smooth_trace.reset();
    optimization_metrics.clear();

    // Frame 0 is the mesh as constructed, before the optimization touches it.
    if (m_params.debug_output) {
        m_debug_pass_name = "construction";
        write_optimization_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
    }

    optimize_offset_loop();

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
    logger().info(
        "energy guard: {} splits, {} collapses and {} swaps refused: the sum of E_T over the "
        "cells they change not finite, or above (swap: not strictly below)",
        iter_cnt_split_energy_reject.load(),
        iter_cnt_collapse_energy_reject.load(),
        iter_cnt_swap_energy_reject.load());

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
        front_ok = ec.converged();
        m_converged = front_ok && m_quality_converged;
        floor_fact = ec.sizing_floor_fact();
        quality = quality_fact(front_ok);
        if (ec.ring_exit) {
            // front_measure "vertex_ring": the ring measure is what was tested; the face and
            // vertex measures are the diagnostics, the face one naming how many faces are still
            // over the bar at the verdict.
            logger().log(
                m_converged ? spdlog::level::info : spdlog::level::warn,
                "{}{}: front {} -- tested (every front vertex's {} within the bar{}, nothing "
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
                ec.unreached_fact(),
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

    // Collapsed foldovers on the offset surface, checked UNCONDITIONALLY -- a fold is a defect in
    // the delivered mesh, not a debug curiosity, so it is reported whether or not debug_output
    // wrote the per-vertex field. Run here, at the end of optimize_offset, so it describes the
    // mesh the driver is about to write: after the finishing pass, not at the loop's verdict.
    // Reported for a run that did not converge too, where a fold is if anything more likely.
    // See offset_surface_foldover_labels() for what the angle is and why its side is not
    // determined. Costs one pass over the live offset simplices, once, with no field evaluation.
    {
        const std::vector<char> fold = offset_surface_foldover_labels();
        size_t n_fold = 0;
        size_t first = std::numeric_limits<size_t>::max();
        for (size_t vid = 0; vid < fold.size(); ++vid) {
            if (!fold[vid]) continue;
            ++n_fold;
            if (first == std::numeric_limits<size_t>::max()) first = vid;
        }
        if (n_fold > 0) {
            const auto& p = m_vertex_attribute[first].m_posf;
            logger().warn(
                "[foldover] the offset surface is folded back on itself at {} vertex(es): {} "
                "meet within {} degrees of coincident (over {} degrees through one side). "
                "First at v{} ({:.4}, {:.4}{}). {}",
                n_fold,
                "two faces of an offset-surface edge",
                360. - FOLDOVER_OUTER_ANGLE_DEG,
                FOLDOVER_OUTER_ANGLE_DEG,
                first,
                p[0],
                p[1],
                fmt::format(", {:.4}", p[2]),
                m_offset_params.debug_output
                    ? "The per-vertex flag offset_foldover is on the debug frames."
                    : "Set DEBUG_output to get the per-vertex offset_foldover field.");
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
