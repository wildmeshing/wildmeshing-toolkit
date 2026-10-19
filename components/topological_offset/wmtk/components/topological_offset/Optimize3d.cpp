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
    if (!(m_offset_params.front_conv_frac() > 0.)) return std::numeric_limits<double>::infinity();
    const double dn = double(n);
    return offset_term_weight() * (sum / dn);
}

double TopoOffsetTetMesh::face_offset_term(const size_t a, const size_t b, const size_t c) const
{
    return face_offset_term(
        potential_for_edge(a, b),
        m_vertex_attribute[a].m_posf,
        m_vertex_attribute[b].m_posf,
        m_vertex_attribute[c].m_posf);
}

double TopoOffsetTetMesh::tet_energy(const size_t tid, const bool band_term) const
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
        double e = band_amips_weight() * v;
        if (m_repulsion_potential) e += repulsion_cell_term(vids);
        return e;
    }
    // A TetWild pass scores every cell by its AMIPS term alone (m_tetwild_pass).
    return cell_energy(vids, a3, band_term && !m_tetwild_pass && cell_is_offset_band(tid));
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
    // E = int_B (d - delta) / delta dV (as the band term's upper bound) + (w / s^3) sum vol
    // AMIPS^3: every live tet's tet_energy() once.
    double s = 0.;
    for (const Tuple& t : get_tets()) s += tet_energy(t.tid(*this));
    return s;
}

double TopoOffsetTetMesh::offset_energy() const
{
    // int_B (d - delta) / delta dV as band_cell_term() reads it, the band cells only. The settle
    // test reads this, not E: E's AMIPS term changes wherever the mesh is improved, the front
    // moving or not. Measured 2026-10-14 (serial, Geneva settings): splits raised E through the
    // AMIPS term by 0.7-3.6 bar energies per turn on the cube outward.
    double s = 0.;
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (cell_is_offset_band(tid)) s += band_cell_term(oriented_tet_vids(tid));
    }
    return s;
}

void TopoOffsetTetMesh::log_energy_step(const char* step) const
{
    // See the declaration. A band cell is never plastic (cell_is_plastic()), so its tet_energy()
    // is cell_energy(vids, A^3, true): the AMIPS part (plus the repulsion term, 0 in the loop)
    // and band_cell_term(), summed here apart rather than calling both tet_energy() and
    // band_cell_term(), which would evaluate the band term twice.
    double amips = 0., band = 0.;
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (!cell_is_offset_band(tid)) {
            amips += tet_energy(tid);
            continue;
        }
        const std::array<size_t, 4> vids = oriented_tet_vids(tid);
        const double a3 = get_quality(t);
        if (a3 >= MAX_ENERGY) {
            amips += MAX_ENERGY;
            continue;
        }
        amips += cell_energy(vids, a3, false);
        band += band_cell_term(vids);
    }
    logger().info(
        "\t[energy step] turn {} {}: E {:.12g} | AMIPS term {:.12g} | band term {:.12g}",
        m_energy_step_turn,
        step,
        amips + band,
        amips,
        band);
}

double TopoOffsetTetMesh::tet_energy_sum(const std::vector<size_t>& tids, const bool band_term)
    const
{
    double s = 0.;
    for (const size_t tid : tids) s += tet_energy(tid, band_term);
    return s;
}

double TopoOffsetTetMesh::candidate_energy(const std::array<size_t, 4>& vids)
{
    // The candidate's label as swap_after_cells() will write it: the ring's single label for an
    // interior swap, the side of a ring vertex it contains for a surface flip.
    const SwapRecord& rec = m_swap_record.local();
    int label = -1;
    if (!rec.flip) {
        label = m_swap_label.local();
    } else {
        const SwapSurfaceSides& sides = m_swap_sides.local();
        for (const size_t v : vids) {
            const auto it = sides.by_vertex.find(v);
            if (it != sides.by_vertex.end()) {
                label = it->second.second;
                break;
            }
        }
    }
    const double a3 = get_quality(vids);
    // use_rest_pose: a plastic cell the swap creates is stamped at creation (swap_after_cells()),
    // so it reads A = 3 against its own shape -- scored with tet_energy()'s arithmetic for such a
    // cell, value_of() with the rest equal to the cell itself.
    if (m_plastic_active && label != 1 && label != 2 && a3 < MAX_ENERGY) {
        std::array<Vector3d, 4> p;
        for (int k = 0; k < 4; ++k) p[size_t(k)] = m_vertex_attribute[vids[size_t(k)]].m_posf;
        Eigen::Matrix3d R;
        for (int k = 1; k < 4; ++k) R.col(k - 1) = p[size_t(k)] - p[0];
        const double v = VolAMIPSEnergy3D::value_of(p, R);
        if (!std::isfinite(v)) return MAX_ENERGY;
        double e = band_amips_weight() * v;
        if (m_repulsion_potential) e += repulsion_cell_term(vids);
        return e;
    }
    // A swap whose band term cancels leaves it out (tet_energy(), BAND TERM THAT CANCELS), as a
    // TetWild pass leaves it out of every cell.
    return cell_energy(vids, a3, label == 2 && !rec.band_cancels && !m_tetwild_pass);
}

void TopoOffsetTetMesh::swap_record_fill(const bool flip)
{
    // See SwapRecord: what the candidates' labels are read from.
    SwapRecord& rec = m_swap_record.local();
    rec.clear();
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
    m_swap_record.local().clear(); // see SwapRecord
    if (!swap_capture_tag(tids)) return false;
    // The energy rule's before-half (see swap_after_cells()): the sum over the ring. A cell's
    // energy is its own corners' and label's, so no cell outside the ring changes energy.
    // use_rest_pose: every new cell takes the ring's one label; an all-plastic swap is an exact
    // tie (plastic_tie()), which the strict rule refuses -- outright, before any case is scored.
    if (plastic_tie(tids, {m_swap_label.local()})) {
        ++m_plastic_tie_swap;
        return false;
    }
    // Without the band term when it cancels (tet_energy(), BAND TERM THAT CANCELS): a ring of
    // band cells has label 2, the label every new cell takes.
    const bool band_cancels = band_term_cancels(tids);
    m_swap_energy_before.local() = tet_energy_sum(tids, !band_cancels);
    // What the 4-4 / 5-6 case search and the face swap's gate read candidate cells' labels from;
    // see SwapRecord. After the capture, whose single label every new cell takes.
    // A 3-2 has no case search and no gate on cells that do not exist yet: the rule judges it
    // on the real cells in swap_after_cells(), so it needs no record beyond band_cancels.
    if (current_op_kind() != OpKind::swap_32) swap_record_fill(false);
    m_swap_record.local().band_cancels = band_cancels; // after the fill, which clears it
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
    // followed through the rest of the swap by [flip funnel]. It is judged by the energy rule like
    // every other swap (swap_after_cells()); no rule of its own.
    if (face_is_offset_surface_live(ftup_abc) && face_is_offset_surface_live(ftup_abd)) {
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

    // The energy rule's before-half (see swap_after_cells()): the sum over the cells the flip
    // replaces. A cell's energy is its own corners' and label's, so the cells beyond the ring --
    // a 3-2's cells beyond (a,c,d) and (b,c,d) among them, whose faces it hands to the other
    // side -- keep theirs.
    // use_rest_pose: each new cell takes a side captured from the ring's vertices; an all-plastic
    // flip is an exact tie (plastic_tie()), which the strict rule refuses -- outright.
    {
        std::vector<int> after_labels;
        for (const auto& [v, side] : m_swap_sides.local().by_vertex) {
            after_labels.push_back(side.second);
        }
        if (plastic_tie(tids, after_labels)) {
            ++m_plastic_tie_swap;
            return swap_reject(SwapReject::app_sag_raised);
        }
    }
    // Without the band term when it cancels (tet_energy(), BAND TERM THAT CANCELS): a ring of
    // band cells has only band sides, and every new cell takes one or the flip is refused.
    const bool band_cancels = band_term_cancels(tids);
    m_swap_energy_before.local() = tet_energy_sum(tids, !band_cancels);
    // What the 4-4 / 5-6 case search reads candidate cells' labels from; see SwapRecord. After
    // swap_capture_surface_sides(), whose sides the new cells take.
    if (current_op_kind() != OpKind::swap_32)
        swap_record_fill(true); // as in swap_before_interior()
    m_swap_record.local().band_cancels = band_cancels; // after the fill, which clears it

    // Non-offset surface flips are not refused categorically: the shared swap checks both new
    // triangles with surface_triangle_is_outside(), which holds a held face to m_envelope, and
    // that envelope is the geometric constraint. The class-match and same-boundary refusals above
    // are the topology half. Placement accuracy is the
    // smoothing passes' job and the energy rule's.
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
    auto elastic = std::make_shared<CubedAMIPSEnergy3D>(std::move(cells), band_amips_weight());
    if (plastic.empty()) return elastic;
    auto sum = std::make_shared<optimization::EnergySum>();
    sum->add_energy(elastic);
    sum->add_energy(std::make_shared<VolAMIPSEnergy3D>(std::move(plastic), band_amips_weight()));
    return sum;
}

bool TopoOffsetTetMesh::smooth_nonfront_vertex(const Tuple& t)
{
    // See the declaration. TetOptimizerMesh::smooth_after()'s options, two changes: the objective
    // comes from smoothing_extra_energy() (w_amips 0: the smoother adds no AMIPS term of its own),
    // and the engine's AMIPS^3 veto is replaced by the energy's, on the engine's scope, which
    // includes vertices on a surface.
    const size_t vid = t.vid(*this);
    optimization::SmoothVertexOptions opts;
    opts.w_amips = 0.;
    opts.w_envelope = m_params.w_envelope;
    opts.s_amips = m_s_amips;
    opts.s_envelope = m_s_envelope;
    opts.two_stage = false;
    opts.smoothing_mode = m_params.smoothing_mode == "exact"
                              ? optimization::SmoothVertexOptions::SmoothingMode::Exact
                              : optimization::SmoothVertexOptions::SmoothingMode::Projected;
    opts.project_line_search_steps = m_params.project_line_search_steps;
    opts.project_line_search_nested_steps = m_params.project_line_search_nested_steps;
    opts.quality_veto = false;
    opts.max_step = damping_step(vid); // EXPERIMENTAL_damping; infinity when off
    const Vector3d x0 = std::as_const(m_vertex_attribute)[vid].m_posf;
    // The rule is E itself: the sum of tet_energy() over the ring may not rise. A vertex held on a
    // surface takes the engine's projected step, whose acceptance is that rule already
    // (smoothing_ring_energy(): strictly below), so it needs no veto here. Every other vertex is
    // vetoed after its solve: the solve descends the vertex's part of E, but a vertex held on the
    // input descends only the AMIPS part (see smoothing_extra_energy()).
    const bool projected =
        opts.smoothing_mode == optimization::SmoothVertexOptions::SmoothingMode::Projected &&
        m_vertex_attribute[vid].m_is_on_surface && smoothing_energy_envelope(vid) != nullptr;
    const std::vector<size_t> ring = get_one_ring_tids_for_vertex(vid);
    const double before = projected ? 0. : tet_energy_sum(ring);
    // BAND TERM THAT CANCELS (tet_energy()) for the move: the ring's cells tile the star of vid,
    // whose link does not move, and smooth_vertex_3d() refuses a move that inverts one, so a
    // ring of band cells clear of the input's bisectors at the old position and at the new one
    // keeps B, and the veto compares the sums without the band term. The full before-sum is
    // kept for a move that leaves the clear region. MEASURED (cube 1e-2 / 1e-4, front_flux, 15
    // rounds, serial): 412,850 of 1,962,638 non-front vetoes had such a ring, |dB| <= 1.6e-12
    // delta V over it (rounding; 16% exactly 0), and reading dB as 0 flipped 0 of them. Front
    // vertices (a background cell in every ring) and held vertices (an input cell) never have one.
    const bool cancels_before = !projected && band_term_cancels(ring);
    const double before_no_band = cancels_before ? tet_energy_sum(ring, false) : 0.;
    if (!optimization::smooth_vertex_3d(
            *this,
            t,
            opts,
            m_solver.local(),
            &m_smooth_rejects,
            &m_newton)) {
        return false;
    }
    if (projected) {
        count_damped_move(vid, x0, opts.max_step);
        return true;
    }
    const bool cancels = cancels_before && band_term_cancels(ring);
    const double after = tet_energy_sum(ring, !cancels);
    if (!(after <= (cancels ? before_no_band : before))) { // a NaN refuses; a tie passes
        --m_smooth_rejects.accepted;
        ++m_smooth_rejects.quality;
        return false;
    }
    count_damped_move(vid, x0, opts.max_step);
    return true;
}

double TopoOffsetTetMesh::damping_step(const size_t vid) const
{
    // See the declaration. Every edge at vid lies in a cell of its one-ring, so the ring's corners
    // other than vid are exactly its neighbours.
    if (!m_offset_params.damping) return std::numeric_limits<double>::infinity();
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
    if (!m_offset_params.damping) return;
    ++m_damping_moves;
    const double moved = (std::as_const(m_vertex_attribute)[vid].m_posf - x0).norm();
    if (moved >= max_step * (1. - 1e-9)) ++m_damping_bounded;
}

double TopoOffsetTetMesh::swap_edge_44_energy(
    const std::vector<std::array<size_t, 4>>& tets,
    const int op_case)
{
    // See the declaration. The sum of the energy over the cells, double::max() for an inverted
    // one as the base does: the current cells are real and read from the mesh, a candidate's do
    // not exist yet and are scored by candidate_energy().
    double e = -1.;
    if (op_case == 0) {
        // The old cells are the ring the before-half already measured (swap_before_interior() /
        // swap_before_surface() -> tet_energy_sum(tids), the same cells the engine passes here as
        // old_tets_conn), so the score is read, not recomputed: one evaluation per operation.
        e = m_swap_energy_before.local();
    } else {
        // Every candidate cell's inversion test before any candidate's energy: a case with an
        // inverted cell scores double::max(), as the base scores it.
        bool inverted = false;
        for (const std::array<size_t, 4>& vids : tets) {
            if (is_inverted(vids)) {
                inverted = true;
                break;
            }
        }
        if (inverted) {
            e = std::numeric_limits<double>::max();
        } else {
            e = 0.; // the candidates' sum
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
    if (!sides.worthwhile || dry_run()) return e; // the funnel follows real attempts
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
    // the face swap's gate (one is a planned follow-up; this override goes with it). Three
    // changes, marked CHANGED: swap_before_interior() runs before the gate instead of after it,
    // since it takes the two cells' sum and fills the record candidate_energy() reads; the gate
    // is on the energy's sum; and every new cell is tested for inversion before any is scored.
    // See the declaration.
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

    // CHANGED: the sum of the energy over the two cells, and over the three new cells
    // (candidate_energy()), in place of the stored AMIPS^3 and get_quality(). The two cells' sum
    // was taken by swap_before_interior() just above; it is read, not evaluated a second time.
    double scored = -1.;
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
        // CHANGED: every new cell's inversion test before any new cell's energy (the base tests
        // and scores cell by cell), as in swap_edge_44_energy(); an inverted cell now reports
        // face_inverted even when an earlier cell would have reported face_not_better.
        for (const std::array<size_t, 4>& new_tet : new_tets) {
            if (is_inverted(new_tet)) {
                return swap_reject_kind_only(SwapReject::face_inverted);
            }
        }
        // The three cells' sum must fall strictly below the two cells' (a NaN refuses).
        scored = 0.;
        for (const std::array<size_t, 4>& new_tet : new_tets) scored += candidate_energy(new_tet);
        if (!(scored < m_swap_energy_before.local())) {
            return swap_reject_kind_only(SwapReject::face_not_better);
        }
    }
    // perform_sanity_checks: the sum over the three cells is what swap_after_cells() must read.
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
        "\t[swap scoring] {}: the {} new cells it picked (first tet {}) scored energy "
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
    // m_face_extra, as the base's tracker_assign_after() does m_face_attribute after this hook:
    // a new cell's face that was a face of the old cells keeps its value, every other is reset.
    // A refusal below rolls it back with the rest. The edge label likewise; both are closed once
    // the new cells' labels are written below (close_labels()).
    face_extra_assign(tids, m_face_extra_swap.local());
    edge_label_assign(tids, m_edge_label_swap.local());
    // THE ENERGY RULE for every swap -- interior edge swaps, flips of a tracked surface, face
    // swaps (see tet_energy()): the sum of tet_energy() over the cells the swap made, their labels
    // written below, must be STRICTLY below the sum over the cells it replaced, as
    // swap_before_interior() / swap_before_surface() took it (a NaN refuses). The cells beyond the
    // ring keep their corners and labels and so their energy: this is the swap's whole change of
    // E. Applied HERE on the real cells, and this is THE rule. The 4-4 / 5-6 case search and the
    // face swap's gate apply it earlier, to cells that do not exist yet (candidate_energy(); see
    // tet_energy(), CANDIDATE CELLS), and under perform_sanity_checks the two are compared here. A
    // swap moves no vertex, so both sums are read at the same positions.
    //
    // It is TetWild's swap rule, strictness included, on the energy instead of AMIPS; the
    // engine's AMIPS form of it, swap_quality_allowed(), admits everything. Strict because a
    // swap pass has to terminate: every accepted swap strictly lowers E. Measured without it, when
    // flips of the offset surface were judged on an ABSOLUTE bar (new cells under stop_energy,
    // until 2026-09-25) and a sag rule that let a tie pass: ties on the cube's flat sides (42.8%
    // of the offset faces sag <= 1e-12 at the 5e-2 cube) flipped back and forth -- a 5e-2 run sat
    // in one pass for 25 minutes, and a 1e-3 pass accepted 3020000 flips, 99.997% of them winning
    // under 1e-12 of the bar. The absolute bar existed because strict improvement of AMIPS alone
    // blocked the flips that cut the front's error: whole runs on the deliverable cube accepted no
    // surface flip at all (one pass: 0 of 7636), and at the cube 1e-3, 93.5% of the valence-4
    // surface edges whose flip would cut the sag raised max AMIPS. The energy charges that error
    // too, so such a flip can pass while AMIPS rises.
    //
    // A refusal is rolled back by the engine. It is counted as app_sag_raised, the engine's name
    // for this after-hook refusal (SwapReject lives in the engine, which this does not change);
    // a face swap counts in the per-kind table alone, as the engine counts its own refusals.
    const auto energy_lowered = [&]() {
        // without the band term when the before-sum left it out (BAND TERM THAT CANCELS)
        const double after_sum = tet_energy_sum(tids, !m_swap_record.local().band_cancels);
        if (m_params.perform_sanity_checks) swap_scoring_check(tids, after_sum);
        if (after_sum < m_swap_energy_before.local()) return true;
        ++m_volume_reject_swap;
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
            // use_rest_pose: a cell the swap creates is stamped at creation, before the energy
            // rule reads it (candidate_energy() scores it the same way).
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
        return false;
    }
    const size_t v2_id = collapse_cache.local().v2_id;
    // repulsion_rounds: the complex must stay simplicially embedded (repulsion_embedding_kept());
    // every cell the collapse made is in the survivor's ring.
    if (m_repulsion_potential && !repulsion_embedding_kept(get_one_ring_tids_for_vertex(v2_id))) {
        ++m_repulsion_embed_refused;
        return false;
    }
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
    // The energy rule's before-half is taken in collapse_before_vertex().
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
    // The energy rule's before-half (see CollapseVolume): the sum of tet_energy() over v1's ring,
    // the cells the collapse changes, and v2's other cells, which it does not. Not in coarsening,
    // where the engine skips its own collapse rule as well.
    if (!m_coarsen_mode) {
        std::vector<size_t> ring1 = get_one_ring_tids_for_vertex(v1_id);
        CollapseVolume& cv = m_collapse_volume.local();
        // Without the band term when it cancels (tet_energy(), BAND TERM THAT CANCELS): the new
        // cells are ring1's cells without v2, in their own slots with their own labels.
        cv.band_cancels = band_term_cancels(ring1);
        cv.before = tet_energy_sum(ring1, !cv.band_cancels);
        // use_rest_pose: the cells the collapse changes are v1's cells without v2, each keeping
        // its label; when they and the whole ring are plastic, the collapse is an exact tie
        // (plastic_tie()).
        std::vector<int> after_labels;
        for (const size_t tid : ring1) {
            const auto vs = oriented_tet_vids(tid);
            if (std::find(vs.begin(), vs.end(), v2_id) == vs.end()) {
                after_labels.push_back(m_tet_attribute[tid].label);
            }
        }
        cv.tie = plastic_tie(ring1, after_labels);
        std::sort(ring1.begin(), ring1.end());
        cv.v2_only.clear();
        for (const size_t tid : get_one_ring_tids_for_vertex(v2_id)) {
            if (!std::binary_search(ring1.begin(), ring1.end(), tid)) cv.v2_only.push_back(tid);
        }
        std::sort(cv.v2_only.begin(), cv.v2_only.end());
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

    return true;
}

bool TopoOffsetTetMesh::collapse_after_connectivity(
    const size_t,
    const size_t v2_id,
    const std::vector<std::array<size_t, 2>>&)
{
    // THE ENERGY RULE for a collapse (see tet_energy()): the sum of tet_energy() over the cells
    // the collapse changed -- the survivor's ring less v2's old cells outside v1's ring, which
    // keep their slots, corners and labels -- may not exceed the sum over v1's old ring
    // (collapse_before_vertex()). A tie passes, as in TetWild's collapse rule, which
    // collapse_quality_allowed() switches off; a NaN refuses.
    //
    // HERE: the connectivity is final, the cells the collapse keeps keep their slots and labels,
    // and a collapse moves no vertex, so the energy is read from the mesh as it now is. Not in
    // collapse_edge_after(), after the base returned: a refusal there has no after-hook reason
    // to be counted under -- CollapseReject lives in the engine, which this does not change, and
    // its app_ops_guard is a before-hook reason, so the [ops accounting] line would report the
    // refusals unattributed. Refused here, the base counts after_connectivity and rolls back.
    //
    // Not in coarsening, where the engine skips its own collapse rule as well and judges the
    // region after re-smoothing.
    //
    // use_rest_pose: every cell the collapse changed is stamped first, at the change and before
    // the rule reads it, as split and swap stamp the cells they make. An exact tie
    // (plastic_tie(), decided in collapse_before_vertex()) is not gated by the energy; it is
    // still refused for a cell the engine cannot score. Coarsening runs in the final pass, where
    // the plastic medium is off.
    if (!m_coarsen_mode) {
        const CollapseVolume& cv = m_collapse_volume.local();
        double after = 0.;
        bool scoreable = true;
        for (const size_t tid : get_one_ring_tids_for_vertex(v2_id)) {
            if (std::binary_search(cv.v2_only.begin(), cv.v2_only.end(), tid)) continue;
            stamp_rest_cell(tid);
            if (cv.tie) {
                scoreable = scoreable && get_quality(tuple_from_tet(tid)) < MAX_ENERGY;
            } else {
                after += tet_energy(tid, !cv.band_cancels);
            }
        }
        if (cv.tie ? !scoreable : !(after <= cv.before)) { // a NaN refuses
            ++m_volume_reject_collapse;
            return false;
        }
        if (cv.tie) ++m_plastic_tie_collapse;
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
    ++m_smooth_trace.attempted;
    const size_t vid = t.vid(*this);

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

    // The repulsion passes before the march (repulsion_smoothing()); false at every other time.
    if (is_repulsion_vertex(vid)) return smooth_repulsion_vertex(t);

    // A front vertex goes through the shared smoother -- same solver, line search and accept
    // tests as every other vertex -- with the offset's options: its objective carries the offset
    // terms (smoothing_extra_energy), and the veto is on the ring's sum of tet_energy() instead
    // of AMIPS, since a front vertex must be able to worsen its ring's shape on the way to the
    // level set but not the energy, which charges both (see smooth_front_vertex()). Every other
    // vertex minimises the same energy (smooth_nonfront_vertex()) -- and in a TetWild pass the
    // front too, as a vertex held in its tube (smoothing_envelope()) whose energy is AMIPS alone.
    if (ve.m_is_on_offset && !m_tetwild_pass) {
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
    // yes/no, and a face resting on the skin of its tube is the one the next operation pushes
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
    if (m_offset_params.damping) {
        const size_t bounded = m_damping_bounded.exchange(0);
        const size_t moves = m_damping_moves.exchange(0);
        logger().info(
            "\t[damping] {} of {} accepted smoothing moves ended on the bound ({} x the shortest "
            "edge at the vertex)",
            bounded,
            moves,
            m_offset_params.damping_factor);
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
    auto band = front_energy(vid);
    const double delta = m_offset_params.target_distance;
    logger().info(
        "[front profile] worst vertex {} at ({:.5}, {:.5}, {:.5}), region {}, along the field "
        "direction n = ({:.4}, {:.4}, {:.4}); columns: s/delta | band term of the vertex's band "
        "cells (front_energy(), no AMIPS)",
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
        logger().info("[front profile] {:+.2f} | {:.6g}", sd, band->value(xv));
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

bool TopoOffsetTetMesh::settle_refinement_turn(
    const int turn,
    double& e_prev,
    const bool refine_now)
{
    const double delta = m_offset_params.target_distance;
    const double eps = m_offset_params.front_conv;
    const double e_now = offset_energy(); // no AMIPS: see offset_energy()
    const double de = e_prev - e_now;
    e_prev = e_now;
    const std::vector<std::array<size_t, 3>> faces = offset_surface_faces();
    double area = 0.;
    for (const auto& f : faces) {
        const Vector3d& pa = m_vertex_attribute[f[0]].m_posf;
        area += 0.5 * (m_vertex_attribute[f[1]].m_posf - pa)
                          .cross(m_vertex_attribute[f[2]].m_posf - pa)
                          .norm();
    }
    // The bar energy: the offset term's excess for a front of area A lying eps beyond the level
    // set, per unit area int_0^eps s / delta ds = eps^2 / (2 delta). A turn whose E fell by less
    // has settled: the front no longer moves by a bar.
    const double bar_energy = eps * eps * area / (2. * delta);
    const bool settled = de < bar_energy;
    // The floor of refine_front_by_halving().
    const double l = std::max(m_params.l, 1e-300);
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / l);
    size_t n_marked = 0, n_floor = 0, n_halved = 0, n_over_max = 0, n_over_rms = 0;
    size_t n_over_vertex = 0, n_front_vertices = 0, n_gap_cells = 0, n_gap_over = 0;
    double max_inside = -std::numeric_limits<double>::infinity(), max_outside = max_inside;
    double max_rms = 0., max_vertex = 0., max_gap = 0.;
    const bool rms_mode = m_offset_params.settle_mark == "rms";
    const bool vertex_mode = m_offset_params.settle_mark == "vertex";
    if (settled) {
        // Per front face f, how far a point of f lies from the level set: inside_f = delta - min
        // over f of d, exact (min over P of the triangle-triangle distance); outside_f = max over f
        // of d, minus delta, exact up to rounding (ExactBandIntegral::max_distance(): f cut by the
        // input's nearest-feature regions, the max at a piece's vertex; an upper bound only on
        // pieces with curved region boundaries, rule (a)). A face with either over eps has its
        // corners halved. The former outside_f, min over P of the max of d_P at f's corners (one
        // triangle's convex d_P), overshoots by about s^2 / (2 delta) where f straddles a coplanar
        // input edge by s: MEASURED, cube outward, turn 28, all 8 faces it marked were 1.06-1.26
        // bars over by it and -0.016..+0.020 bars by the exact max; they alone blocked the exit.
        // settle_mark "rms" and "vertex" only log outside_f and keep the former bound (the exact
        // structure exists for them only under band_volume_rule "exact").
        std::vector<size_t> corners;
        for (const auto& f : faces) {
            const std::array<Vector3d, 3> p = {
                {m_vertex_attribute[f[0]].m_posf,
                 m_vertex_attribute[f[1]].m_posf,
                 m_vertex_attribute[f[2]].m_posf}};
            const double inside = delta - min_triangle_distance(
                                              *m_input_complex_envelope,
                                              m_input_complex_pad,
                                              p[0],
                                              p[1],
                                              p[2]);
            const double outside =
                (m_band_exact && !rms_mode && !vertex_mode ? m_band_exact->max_distance(p)
                                                           : min_max_corner_distance(
                                                                 *m_input_complex_envelope,
                                                                 m_input_complex_pad,
                                                                 p.data(),
                                                                 3)) -
                delta;
            max_inside = std::max(max_inside, inside / eps);
            max_outside = std::max(max_outside, outside / eps);
            const bool over_max = std::max(inside, outside) > eps;
            // EXPERIMENTAL_settle_mark "rms": the mean over f of (d - delta)^2 cos(theta) by the
            // 3-edge-midpoint rule, theta between f's normal out of the band and grad d (signed),
            // against front_conv^2 -- X_f against the bar energy of area(f), the area cancelling.
            // The normal is oriented by the band cell carrying f (its fourth corner is in the
            // band).
            Vector3d n = (p[1] - p[0]).cross(p[2] - p[0]);
            const int64_t bc = front_face_band_cell(f[0], f[1], f[2]);
            if (bc >= 0) {
                for (const size_t v : oriented_tet_vids(size_t(bc))) {
                    if (v == f[0] || v == f[1] || v == f[2]) continue;
                    if (n.dot(p[0] - m_vertex_attribute[v].m_posf) < 0.) n = -n;
                }
            }
            const double q = face_mean_sq_cos(
                *m_input_complex_envelope,
                m_input_complex_pad,
                p[0],
                p[1],
                p[2],
                n.normalized(),
                delta);
            const bool over_rms = bc >= 0 && q > eps * eps;
            max_rms = std::max(max_rms, std::sqrt(std::max(q, 0.)) / eps);
            n_over_max += over_max;
            n_over_rms += over_rms;
            if (vertex_mode || !(rms_mode ? over_rms : over_max)) continue;
            ++n_marked;
            bool at_floor = true;
            for (const size_t v : f) {
                corners.push_back(v);
                at_floor = at_floor && m_vertex_attribute[v].m_sizing_scalar <= s_floor;
            }
            if (at_floor) ++n_floor;
        }
        // EXPERIMENTAL_settle_mark "vertex": a front vertex whose own distance from the level set,
        // |d(v) - delta|, exceeds front_conv (d over all input triangles, exact at the vertex). It
        // says nothing about the points of a face between its corners.
        std::vector<size_t> front_vids;
        for (const auto& f : faces) front_vids.insert(front_vids.end(), f.begin(), f.end());
        wmtk::vector_unique(front_vids);
        n_front_vertices = front_vids.size();
        for (const size_t v : front_vids) {
            const Vector3d& x = m_vertex_attribute[v].m_posf;
            const Envelope& env = *m_input_complex_envelope;
            const double err = std::abs(
                env.primitive_distance(nearest_primitive(env, m_input_complex_pad, x), x) - delta);
            max_vertex = std::max(max_vertex, err / eps);
            if (!(err > eps)) continue;
            ++n_over_vertex;
            if (!vertex_mode) continue;
            ++n_marked;
            corners.push_back(v);
            if (m_vertex_attribute[v].m_sizing_scalar <= s_floor) ++n_floor;
        }
        // EXPERIMENTAL_gap_refinement: a band cell with a front corner whose corner bound may lie
        // more than a bar above its true integral: md - d(centroid) > front_conv, md the corner
        // mean of the bounding triangle's distance. Where d is convex on the cell the true mean of
        // d is at least d(centroid) (Jensen), so Vol (md - d(c)) / delta bounds the gap. The gap
        // depends on the corners' positions, and the smoother descends it with the energy: on the
        // cylinder outward (2026-10-09) every fold formed by a move that lowered the bound while
        // the true integral plus AMIPS rose.
        if (m_offset_params.gap_refinement) {
            std::vector<size_t> cells;
            for (const size_t v : front_vids) {
                for (const size_t t : get_one_ring_tids_for_vertex(v)) {
                    if (cell_is_offset_band(t)) cells.push_back(t);
                }
            }
            wmtk::vector_unique(cells);
            n_gap_cells = cells.size();
            for (const size_t t : cells) {
                const auto vs = oriented_tet_vids(t);
                std::array<Vector3d, 4> p;
                for (int j = 0; j < 4; ++j) p[size_t(j)] = m_vertex_attribute[vs[size_t(j)]].m_posf;
                int64_t best = -1;
                const double md = min_mean_distance(
                    *m_input_complex_envelope,
                    m_input_complex_pad,
                    p.data(),
                    4,
                    best);
                const Vector3d ctr = 0.25 * (p[0] + p[1] + p[2] + p[3]);
                const Envelope& env = *m_input_complex_envelope;
                const double dc =
                    env.primitive_distance(nearest_primitive(env, m_input_complex_pad, ctr), ctr);
                max_gap = std::max(max_gap, (md - dc) / eps);
                if (!(md - dc > eps)) continue;
                ++n_gap_over;
                bool at_floor = true;
                for (const size_t v : vs) {
                    corners.push_back(v);
                    at_floor = at_floor && m_vertex_attribute[v].m_sizing_scalar <= s_floor;
                }
                if (at_floor) ++n_floor;
            }
        }
        if (refine_now && !corners.empty()) n_halved = refine_front_by_halving(corners);
    }
    // A marked face with every corner at the floor is one nothing will refine: it blocks the exit
    // for good, hence the warning, every turn such faces exist.
    logger().log(
        n_floor > 0 ? spdlog::level::warn : spdlog::level::info,
        "\t[settle] turn {}: offset energy {:.10g}, its fall {:.4g}, bar energy b {:.4g} (front "
        "area {:.6g}), "
        "settled {}{}",
        turn,
        e_now,
        de,
        bar_energy,
        area,
        settled ? "yes" : "no",
        settled ? fmt::format(
                      " | marked {} {} of {} by the {} test (max test: {} faces over, max "
                      "inside {:.4} bars, max outside {:.4} bars; rms test: {} faces over, max "
                      "{:.4} bars; vertex test: {} vertices over, max {:.4} bars{}), {}, {} marked "
                      "{} at the sizing floor {:.4g}{}",
                      n_marked,
                      vertex_mode ? "vertices" : "faces",
                      vertex_mode ? fmt::format("{} front vertices", n_front_vertices)
                                  : fmt::format("{}", faces.size()),
                      m_offset_params.settle_mark,
                      n_over_max,
                      max_inside,
                      max_outside,
                      n_over_rms,
                      max_rms,
                      n_over_vertex,
                      max_vertex,
                      m_offset_params.gap_refinement
                          ? fmt::format(
                                "; gap test: {} of {} band cells at the front over, max {:.4} bars "
                                "(refined)",
                                n_gap_over,
                                n_gap_cells,
                                max_gap)
                          : std::string(),
                      refine_now ? fmt::format("halved {} vertices", n_halved)
                                 : std::string("no halving (this loop does not refine)"),
                      n_floor,
                      vertex_mode ? "vertices" : "faces with every corner",
                      s_floor,
                      n_floor > 0 ? " (they cannot be refined and block the exit)" : "")
                : std::string(" -- no face test, no refinement, no exit: the front still moves"));
    return settled && n_marked == 0 && n_gap_over == 0;
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
    // same way, seeding the same force-split edges. TetWild passes only, by construction:
    // mesh_improvement() is this function's one caller, and only tetwild_pass() runs that.
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
        fate = "a TetWild pass: AMIPS alone, held in the offset tube (smooth_nonfront_vertex)";
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
    //
    // The faces are found from the band side: a live offset face has exactly one band cell
    // (face_is_offset_surface_live() is true only where its two cells differ, or for a band
    // cell's face on the domain boundary), so the band cells' faces list each one exactly once,
    // at one switch_tetrahedron() per band-cell face -- not a canonical-id lookup for every face
    // of the mesh before the same test. MEASURED 2026-10-09 on the cube (22k tets): 9 of the 40
    // ms a debug frame spent reading the mesh, before. The order the faces arrive in does not
    // change the labels: an edge keeps the two faces' third vertices, and the angle between them
    // is symmetric in the two.
    struct EdgeFaces
    {
        int n = 0;
        std::array<size_t, 2> opposite{{0, 0}}; // the two faces' third vertices
    };
    std::map<std::array<size_t, 2>, EdgeFaces> edges;
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (!cell_is_offset_band(tid)) continue;
        for (int j = 0; j < 4; ++j) {
            const Tuple f = tuple_from_face(tid, j);
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

    // The leash as init() resolved it: absolute if the config gave one, else offset_envelope_rel
    // x the bbox diagonal. Referenced to the BOX, not to target_distance, since 2026-09-24. As
    // in 2D.
    const double eps = std::max(m_offset_params.offset_envelope, 1e-12);

    m_offset_envelope = std::make_shared<Envelope>(/*exact=*/true);
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
    if (m_offset_potential) {
        // E's two parts at this frame (see global_energy()), one line per frame.
        double vol = 0., amips = 0.;
        for (const Tuple& t : get_tets()) {
            const size_t tid = t.tid(*this);
            const auto vids = oriented_tet_vids(tid);
            const double amips3 = get_quality(t);
            amips +=
                amips3 >= MAX_ENERGY ? MAX_ENERGY : band_amips_weight() * vol_amips3(vids, amips3);
            if (cell_is_offset_band(tid)) vol += band_cell_term(vids);
        }
        std::ofstream f(
            m_offset_params.output_path + "_energy.txt",
            idx == 0 ? std::ios::trunc : std::ios::app);
        if (f) f << fmt::format("{:05d}\t{}\t{:.17g}\t{:.17g}\n", idx, label, vol, amips);
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

bool TopoOffsetTetMesh::optimize_offset_loop(
    const bool final_stage,
    const bool refine,
    const std::string& label)
{
    // One loop: TetWild's operation groups (split / collapse / swap, each followed by smoothing)
    // with the front placed by the offset objective inside the smoothing passes. No offset tube
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
        "\tTurn shape: {}",
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
        local_operations({{0, 0, 0, k}});
    }
    op_accounting_reset(); // the [ops accounting] lines are per turn, from turn 1's first op
    m_split_order_waits = 0; // and so are the [split order] lines
    m_split_off_longest = 0;
    m_swap_scoring_checked = 0; // and the [swap scoring] lines
    m_swap_scoring_mismatch = 0;
    // EXPERIMENTAL_settle_refinement: the settle test refines and exits (settle_refinement_turn());
    // the energy criterion's refinable sets, its sizing-floor warnings and its exit do not act.
    // settle_e_prev is the offset energy at the end of the previous turn, for turn 1 before its
    // first group.
    const bool settle = m_offset_params.settle_refinement;
    double settle_e_prev = 0.;
    // EXPERIMENTAL_laplacian: each turn's factor (m_laplacian_factor) from the offset energy's
    // change over the turns before it, as the settle test measures it: 1 for turns 1 and 2, then
    // min(1, |dE_{t-1}| / |dE_1|). lap_e_start is the offset energy at the running turn's start.
    const bool laplacian = m_offset_params.laplacian > 0.;
    double lap_e_start = 0., lap_de_first = 0., lap_de_last = 0.;
    m_laplacian_factor = 1.;
    for (int it = 0; it < budget; ++it) {
        m_round = it + 1;
        m_iterations_used = it + 1;
        if (laplacian) {
            if (it == 0) {
                logger().info(
                    "\t[laplacian] turn 1: factor 1 (lambda {}; no change measured yet)",
                    m_offset_params.laplacian);
            } else {
                logger().info(
                    "\t[laplacian] turn {}: factor {:.6g} (lambda {}; |dE| of turn {} {:.6g}, of "
                    "turn 1 {:.6g})",
                    it + 1,
                    m_laplacian_factor,
                    m_offset_params.laplacian,
                    it,
                    lap_de_last,
                    lap_de_first);
            }
        }
        for (size_t gi = 0; gi < groups.size(); ++gi) {
            ++m_smooth_group; // see VertexExtra::m_front_change
            // E before and after each group, the descent check (a collapse or swap group may not
            // raise it; a split group may).
            // The exact rules' value table, sized to the mesh as it is now (see
            // ExactBandIntegral::reserve_values()).
            if (m_band_exact) m_band_exact->reserve_values(2 * tet_capacity());
            const double energy_before_group = global_energy();
            if (settle && it == 0 && gi == 0) settle_e_prev = offset_energy();
            if (laplacian && it == 0 && gi == 0) {
                lap_e_start = settle ? settle_e_prev : offset_energy();
            }
            const int vol_c0 = m_volume_reject_collapse.load(),
                      vol_s0 = m_volume_reject_swap.load(), vol_p0 = m_volume_reject_split.load();
            const int tie_c0 = m_plastic_tie_collapse.load(), tie_s0 = m_plastic_tie_swap.load();
            if (gi == 1) needle_scan("collapse pass");
            if (!interleaved) needle_scan("combined ops pass");
            // [energy step] lines: the group's baseline here, then one per smoothing pass from
            // smoothing_pass_done() (see log_energy_step()).
            m_energy_step_turn = it + 1;
            m_energy_step_group = group_names[gi];
            m_energy_step_first = true;
            log_energy_step("start");
            local_operations(groups[gi]);
            m_energy_step_turn = 0;
            // Per group, so a containment violation is attributed to the pass that made it
            // rather than found at the end of the run. Same gate as the shared sanity check.
            if (m_params.perform_sanity_checks) {
                audit_surface_containment(fmt::format("turn {} after {}", it + 1, group_names[gi]));
            }
            {
                const double energy_after_group = global_energy();
                logger().info(
                    "\t[band volume energy] turn {} {} group: E {:.10g} -> {:.10g} ({:+.4g}) | "
                    "refused for raising E over the changed cells (the only rule): {} collapses, "
                    "{} swaps, {} splits",
                    it + 1,
                    group_names[gi],
                    energy_before_group,
                    energy_after_group,
                    energy_after_group - energy_before_group,
                    m_volume_reject_collapse.load() - vol_c0,
                    m_volume_reject_swap.load() - vol_s0,
                    m_volume_reject_split.load() - vol_p0);
                if (m_plastic_active) {
                    logger().info(
                        "\t[plastic tie] turn {} {} group: {} swaps refused outright, {} collapses "
                        "not gated by the energy (all-plastic, plastic_tie())",
                        it + 1,
                        group_names[gi],
                        m_plastic_tie_swap.load() - tie_s0,
                        m_plastic_tie_collapse.load() - tie_c0);
                }
            }
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
        // EXPERIMENTAL_settle_refinement: the settle test, the face test and the halving; true
        // when the loop may exit, which then replaces EnergyCriterion::converged() below.
        const bool settle_exit = settle && settle_refinement_turn(it + 1, settle_e_prev, refine);
        // EXPERIMENTAL_laplacian: this turn's change of the offset energy sets the next turn's
        // factor. settle_refinement_turn() has just moved settle_e_prev to the energy now.
        if (laplacian) {
            const double e_end = settle ? settle_e_prev : offset_energy();
            lap_de_last = std::abs(lap_e_start - e_end);
            if (it == 0) lap_de_first = lap_de_last;
            m_laplacian_factor = lap_de_first > 0. ? std::min(1., lap_de_last / lap_de_first)
                                                   : (lap_de_last > 0. ? 1. : 0.);
            lap_e_start = e_end;
        }
        // The turn's "end" frame is written HERE, after the refinement, not before it: it is the
        // turn's final state, so what it carries is the sizing field the halving just lowered.
        // 3D ONLY; 2D still writes its end frame before the refinement. See .claude/CLAUDE.md.
        if (m_offset_params.debug_output) {
            write_optimization_debug_output(fmt::format("end_{}S", it + 1));
        }
        // Termination: every offset face's measure within the bar and nothing unmeasurable
        // (EnergyCriterion::converged()) -- then quality with the front held in its tube (below).
        // The face measure is the one quantity the smoothing minimises and the refinement reads;
        // the vertex measure is reported on the turn line and in the verdict, and tested nowhere.
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
        //
        // EXPERIMENTAL_settle_refinement: the settle exit (settled, no front face more than
        // front_conv from the level set) in place of converged(); the criterion's measures are
        // then diagnostics.
        const bool resolved = settle ? settle_exit : ec.converged();
        if (resolved && !final_stage) {
            // The init_optimize loop ends here; the final pass and the verdict belong to the
            // ordinary loop that follows it.
            logger().info(
                "[{}] converged at target_distance {:.6g} after {} turn(s): {}{} max {:.4}x the "
                "bar over {} front vertices, max AMIPS {:.4}",
                label,
                m_offset_params.target_distance,
                it + 1,
                settle ? "settled, no front face over the bar (EXPERIMENTAL_settle_refinement); "
                         "diagnostic: "
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
                    "The front is resolved after {} iteration(s): the turn settled and no front "
                    "face is more than front_conv from the level set "
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
                    "======== final pass, TetWild with the front held in its tube: max AMIPS "
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
                if (m_offset_params.debug_output) {
                    write_optimization_debug_output(fmt::format("end_{}F", it + 2));
                }
            }
            return true;
        }
    }
    logger().warn(
        "The loop did not converge in {} turns (max_rounds){}",
        budget,
        final_stage ? std::string()
                    : fmt::format(
                          " -- {} at target_distance {:.6g}",
                          label,
                          m_offset_params.target_distance));
    log_front_profile(energy_criterion().worst_vid);
    return false;
}

double TopoOffsetTetMesh::tetwild_pass(const char* what, const int max_its)
{
    // The final envelopes -- the front's tube, and m_envelope widened to every region boundary
    // as placement left them (build_final_envelopes()). Every cell's energy its AMIPS term alone
    // (m_tetwild_pass: no band term; no plastic medium, so AMIPS is against the regular tet), the
    // quality the pass is driving down; the front smoothed inside its tube like any held vertex.
    build_final_envelopes();
    m_tetwild_pass = true;
    const bool plastic_was = m_plastic_active;
    m_plastic_active = false;
    mesh_improvement(max_its);
    logger().info(
        "\t[split order] {}: waited {} | committed off the longest edge {}",
        what,
        m_split_order_waits.exchange(0),
        m_split_off_longest.exchange(0));
    m_plastic_active = plastic_was;
    m_tetwild_pass = false;
    release_final_envelopes();
    assign_band_regions();
    return std::get<0>(optimization_quality_stats());
}

void TopoOffsetTetMesh::construction_tetwild_run()
{
    // See the declaration and the spec doc (EXPERIMENTAL_construction_tetwild_run).
    const auto t0 = std::chrono::steady_clock::now();
    const double amips0 = std::get<0>(optimization_quality_stats());
    logger().info(
        "======== [construction tetwild] TetWild on the mesh as constructed, every tracked "
        "surface held (the offset surface to eps {:.6g}, the rest to eps {:.6g}): max AMIPS "
        "{:.6g}, stop_energy {} | #V {}, #T {} ========",
        m_offset_params.offset_envelope,
        m_envelope_eps,
        amips0,
        m_params.stop_energy,
        vertex_size(),
        tet_size());
    log_energy_step("start");
    m_frame_prefix = "c";
    m_round = 0;
    const double amips1 =
        tetwild_pass("construction tetwild", std::max(1, m_offset_params.max_iterations));
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
        "\t[construction tetwild] max AMIPS {:.6g} -> {:.6g} (stop {}){} | #V {}, #T {} | sizing "
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

double TopoOffsetTetMesh::growth_stage_target(
    const double d0,
    const double delta,
    const int k,
    const int n)
{
    return k >= n ? delta : d0 + double(k) * (delta - d0) / double(n);
}

void TopoOffsetTetMesh::incremental_growth()
{
    // See the declaration and the spec doc (EXPERIMENTAL_incremental_growth). Between stages only
    // target_distance changes, and what init_offset_potential() builds on it: the field, the
    // per-piece fields, the exact integral's reach. Every other reader of target_distance (the
    // band term, the settle test, the measures, the front smoother) asks for it when it runs, and
    // no pass is running here.
    const double delta = m_offset_params.target_distance;
    const double d0 = m_growth_start;
    const int n = m_offset_params.incremental_growth_steps;
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
    if (m_params.debug_output) {
        m_debug_pass_name = "construction";
        write_optimization_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
    }

    if (m_offset_params.construction_tetwild_run) construction_tetwild_run();

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

    // EXPERIMENTAL_incremental_growth (see marching_tets()): the loop in stages, the last of them
    // the ordinary loop at target_distance.
    if (m_growth_start > 0.) {
        incremental_growth();
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
        // EXPERIMENTAL_settle_refinement: the front half is the loop's settle exit, the one place
        // that sets m_energy_verdict under the key; the criterion's measures and its sizing-floor
        // fact (about refinable sets that do not act) are diagnostics.
        const bool settle = m_offset_params.settle_refinement;
        front_ok = settle ? m_energy_verdict.has_value() : ec.converged();
        m_converged = front_ok && m_quality_converged;
        floor_fact = settle ? std::string() : ec.sizing_floor_fact();
        quality = quality_fact(front_ok);
        if (settle) {
            logger().info(
                "[settle] verdict: front {} -- tested: the loop's settle exit (a settled turn with "
                "no front face more than front_conv from the level set); the measures on the next "
                "line are diagnostics",
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
