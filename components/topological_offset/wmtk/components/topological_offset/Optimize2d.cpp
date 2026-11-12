#include <wmtk/utils/AMIPS2D.h>
#include "TopoOffsetTriMesh.h"

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SizingField.hpp>
#include <wmtk/utils/predicates.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * The 2D optimization phase. The operations themselves -- split, collapse, the quality-driven
 * edge flip, and the driver that sequences them -- are wmtk::TriOptimizerMesh's. What is here
 * is only what the offset knows: where its two tracked surfaces are, and how a vertex on the
 * offset boundary is allowed to move.
 */

bool TopoOffsetTriMesh::edge_is_on_surface(const std::array<size_t, 2>& vids) const
{
    const auto [t, eid] = tuple_from_edge(vids);
    if (eid == static_cast<size_t>(-1) || !t.is_valid(*this)) {
        return false;
    }
    // The edge's own tracking record, never a comparison of the incident faces' tag sets: the
    // band replaces the tags of every face it grows through, so a region boundary the band
    // swallowed has identical tags on both sides and a tag comparison would call it untracked.
    // m_is_surface_fs cannot go quiet that way -- it is assigned once from the input partition
    // and propagated by split, collapse and swap alike.
    //
    // Spelled out rather than delegating to is_edge_on_surface(vids) so the tuple lookup the
    // guard above already paid for is reused.
    return m_edge_attribute[eid].m_is_surface_fs;
}

bool TopoOffsetTriMesh::vertex_is_on_surface(const size_t vid) const
{
    for (const Tuple& e : get_one_ring_edges_for_vertex(vid)) {
        const size_t a = e.vid(*this);
        const size_t b = e.switch_vertex(*this).vid(*this);
        if (edge_is_on_surface({{a, b}})) return true;
    }
    return false;
}

bool TopoOffsetTriMesh::face_in_region(const size_t fid) const
{
    return m_face_extra[fid].label != 0; // 1 = input complex, 2 = offset band
}

bool TopoOffsetTriMesh::face_is_input_complex(const size_t fid) const
{
    return m_face_extra[fid].label == 1;
}

void TopoOffsetTriMesh::label_offset_boundary()
{
    // Runs once at the top of the optimization, never again -- as in 3D. It only upgrades the
    // offset boundary to its own class: the face labels are construction data the optimization
    // does not propagate, and init_surfaces_and_boundaries() classifies the region boundaries and
    // the wall once from the input partition, which is what the per-tag envelopes are keyed on.
    // Region and input edges keep class 0 and are envelope-checked by the shared operations
    // exactly as in triwild and simwild.

    // Face quality, which the shared operations read and keep up to date from here on.
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        m_face_attribute[fid].m_quality = get_quality(fid);
    }

    size_t n_off = 0, n_wall_band = 0;
    for (const Tuple& e : get_edges()) {
        const size_t eid = e.eid(*this);
        if (m_edge_extra[eid].label != 2) {
            continue; // edge not on the offset, skip
        }
        const std::optional<Tuple> opp = e.switch_face(*this);
        if (!opp) {
            // The band ran into the domain wall. With no second face this cannot be classified
            // as offset boundary -- and must not be, or the wall segment would lose the per-tag
            // containment that keeps the box a box.
            ++n_wall_band;
            continue;
        }
        if (m_face_extra[e.fid(*this)].label == m_face_extra[opp->fid(*this)].label) {
            continue; // edge not between different labels, skip
        }

        m_edge_attribute[eid].m_is_surface_fs = true;
        m_edge_attribute[eid].m_surface_class = OFFSET_SURFACE_CLASS;
        ++n_off;

        for (const size_t vid : {e.vid(*this), e.switch_vertex(*this).vid(*this)}) {
            m_vertex_extra[vid].m_is_on_offset = true;
            // The base's union flag, and the one the shared operations actually read.
            m_vertex_attribute[vid].m_is_on_surface = true;
        }
    }

    size_t n_reg = 0, n_box = 0;
    for (const Tuple& e : get_edges()) {
        const size_t eid = e.eid(*this);
        n_reg += edge_is_region(eid);
        n_box += (m_edge_attribute[eid].m_is_bbox_fs >= 0);
    }
    logger().info(
        "\ttracked edges: {} offset boundary, {} region boundary (input complex included), {} "
        "bbox | {} band edges lying ON the wall",
        n_off,
        n_reg,
        n_box,
        n_wall_band);
}

bool TopoOffsetTriMesh::swap_edge_before(const Tuple& t)
{
    // The base refuses every tracked edge (is_edge_on_surface()), the offset boundary included,
    // so a 2D swap never flips the front: what reaches past this call is an interior flip, and
    // the only offset rule that applies to it is the energy rule (see swap_edge_after()).
    if (!TriOptimizerMesh::swap_edge_before(t)) {
        return false;
    }

    // A wall edge needs no special case: it is m_is_surface_fs-tracked (the base refuses those),
    // has no second face (the `!opp` test below), and its endpoints' mask differs from any
    // interior apex pair (the junction rule at the bottom).

    // The flip replaces (a,b) with (c,d), the two apexes of the incident triangles.
    const std::optional<Tuple> opp = t.switch_face(*this);
    if (!opp) return false;
    const size_t c = t.switch_edge(*this).switch_vertex(*this).vid(*this);
    const size_t d = opp->switch_edge(*this).switch_vertex(*this).vid(*this);
    if (c == d) return false;

    // Already joined? Then the flip would duplicate that edge.
    for (const Tuple& e : get_one_ring_edges_for_vertex(tuple_from_vertex(c))) {
        const size_t nb = (e.vid(*this) == c) ? e.switch_vertex(*this).vid(*this) : e.vid(*this);
        if (nb == d) return false;
    }

    // A flip across a junction would detach the new diagonal from one of the boundaries the old
    // edge lay on: refuse when the endpoints' masks differ from the apexes'. 3D twin: the
    // face_mask comparison in swap_before_surface().
    if (edge_mask({{t.vid(*this), t.switch_vertex(*this).vid(*this)}}) != edge_mask({{c, d}})) {
        return false;
    }

    // The energy rule's before-half (see swap_edge_after()): the sum of tri_energy() over the two
    // faces the flip replaces. A face's energy is its own corners' and label's, so no face
    // outside the two changes energy. Without the band term when it cancels (tri_energy(), BAND
    // TERM THAT CANCELS): both faces share one label, which the new faces keep. Last, so only a
    // flip every other test admitted pays for it. As in 3D (swap_before_interior()).
    const std::vector<size_t> fids = {t.fid(*this), opp->fid(*this)};
    const bool band_cancels = band_term_cancels(fids);
    m_swap_band_cancels.local() = band_cancels;
    m_swap_energy_before.local() = tri_energy_sum(fids, !band_cancels);
    // "exact_swept": the band faces the flip replaces, with their pieces (swap_swept_change())
    if (swept_rule()) {
        auto& old = m_swap_old_band.local();
        old.clear();
        for (const size_t fid : fids) {
            if (!face_is_offset_band(fid)) continue;
            const std::array<size_t, 3> vs = oriented_tri_vids(fid);
            old.push_back({vs, place_piece(vs.data(), 3)});
        }
    }
    return true;
}

void TopoOffsetTriMesh::warn_if_offset_reaches_domain_boundary() const
{
    // A band edge with no opposite face lies ON the domain boundary: the band ran out of room
    // before reaching target_distance. Counted in vertices as well as edges because the vertices
    // are what is frozen and what the distance metric drops.
    size_t n_edges = 0, n_verts = 0;
    std::vector<bool> counted(vert_capacity(), false);
    for (const Tuple& e : get_edges()) {
        if (e.switch_face(*this)) continue; // interior edge; the band has room here
        if (!face_is_offset_band(e.fid(*this))) continue;
        ++n_edges;
        for (const size_t v : {e.vid(*this), e.switch_vertex(*this).vid(*this)}) {
            if (!counted[v]) {
                counted[v] = true;
                ++n_verts;
            }
        }
    }
    if (n_edges == 0) return;

    logger().warn(
        "Offset band reaches the domain boundary: {} band edges ({} vertices) lie ON the "
        "bounding box. target_distance ({}) exceeds the clearance between the input complex and "
        "the box, so the offset is CLIPPED there and cannot reach the target distance -- those "
        "vertices are on the frozen bounding box and no operation may move them. They are also "
        "excluded from max_dist_err / avg_dist_err, which have no opposite face to measure "
        "across, so the reported distance error UNDER-REPORTS the true error. Reduce "
        "target_distance, or pad the background mesh.",
        n_edges,
        n_verts,
        m_offset_params.target_distance);
}

std::shared_ptr<Envelope> TopoOffsetTriMesh::envelope_for_mask(uint64_t mask) const
{
    if (mask == 0) return nullptr;
    if ((mask & (mask - 1)) == 0) {
        // Single bit: the member envelope itself -- a real Envelope, safe on every path
        // including the pull. Linear scan; the tag count is tiny.
        for (const auto& [tag, env] : m_tag_envelopes) {
            const auto it = m_tag_bit.find(tag);
            if (it != m_tag_bit.end() && (mask >> it->second) == 1) return env;
        }
        return nullptr; // a bit whose tag never got an envelope (no boundary edges at init)
    }
    // Several bits: the memoized intersection. Lazy and mutex-guarded because containment
    // queries run concurrently under kPartition; creation is rare (a handful of junction masks
    // per model), so the lock is uncontended in steady state.
    {
        std::lock_guard<std::mutex> lock(m_isect_mutex);
        const auto it = m_isect_cache.find(mask);
        if (it != m_isect_cache.end()) return it->second;
    }
    std::vector<std::shared_ptr<Envelope>> members;
    for (const auto& [tag, env] : m_tag_envelopes) {
        const auto it = m_tag_bit.find(tag);
        if (it != m_tag_bit.end() && (mask & (uint64_t(1) << it->second))) {
            members.push_back(env);
        }
    }
    std::shared_ptr<Envelope> isect;
    if (members.empty()) {
        isect = nullptr; // every bit dangled; nothing to contain in
    } else if (members.size() == 1) {
        isect = members.front(); // the other bits dangled; degrade to the one real tube
    } else {
        isect = std::make_shared<IntersectionEnvelope>(std::move(members));
    }
    std::lock_guard<std::mutex> lock(m_isect_mutex);
    m_isect_cache.emplace(mask, isect);
    return isect;
}

std::shared_ptr<Envelope> TopoOffsetTriMesh::containment_for(
    const uint64_t region_mask,
    const bool on_offset) const
{
    // The region side first, and OUTSIDE the lock: envelope_for_mask() takes m_isect_mutex
    // itself and std::mutex is not recursive, so calling it while holding the lock below
    // deadlocks. Nothing here re-enters it after the lock is taken.
    const std::shared_ptr<Envelope> region = envelope_for_mask(region_mask);

    // The offset side, for the operations of the frozen-front final pass only: in the loop the
    // offset boundary is held to no envelope, by the operations or the smoother (see
    // smoothing_containment_envelope()), since placing the front is what moves it. Null until
    // the final pass builds it. As in 3D.
    const bool hold_offset = on_offset && m_offset_envelope != nullptr && m_freeze_front;

    if (!hold_offset) return region; // may itself be null: nothing contains this simplex
    if (!region) return m_offset_envelope;

    // On both: the intersection, inside every tube it lies on. Memoized per region mask;
    // rebuild_offset_envelope() clears the map, so an entry can never outlive its tube.
    {
        std::lock_guard<std::mutex> lock(m_isect_mutex);
        const auto it = m_offset_isect_cache.find(region_mask);
        if (it != m_offset_isect_cache.end()) return it->second;
    }
    std::shared_ptr<Envelope> isect = std::make_shared<IntersectionEnvelope>(
        std::vector<std::shared_ptr<Envelope>>{region, m_offset_envelope});
    std::lock_guard<std::mutex> lock(m_isect_mutex);
    return m_offset_isect_cache.emplace(region_mask, std::move(isect)).first->second;
}

bool TopoOffsetTriMesh::face_is_deformable(const size_t fid) const
{
    // deform_others: the whole medium outside the band deforms, the input complex's interior
    // included -- its boundary is what the complex tube holds. Same set as face_is_plastic().
    return face_is_plastic(fid);
}

bool TopoOffsetTriMesh::face_is_released_band(const size_t fid) const
{
    // A band cell that is released material: every tag besides the offset output tag belongs to
    // a released object, and there is at least one such tag. A pure band cell carries only the
    // output tag and stays the front's freely-reshaped working material; a cell shared with a
    // held object is excluded. Read only by the front placement objective and the rest stamping,
    // so the front does work against released material where it pushes through the overlap while
    // the band's interior smoothing stays equilateral.
    if (m_deform_tags.empty()) return false;
    if (m_face_extra[fid].label != 2) return false;
    bool has_released = false;
    for (const int64_t t : m_face_attribute[fid].tags) {
        if (m_offset_output_tag_ids.count(t)) continue;
        if (m_deform_tags.count(t) == 0) return false;
        has_released = true;
    }
    return has_released;
}

void TopoOffsetTriMesh::stamp_rest_face(const size_t fid)
{
    if (!face_is_plastic(fid) && !face_is_deformable(fid) && !face_is_released_band(fid)) return;
    const auto vs = oriented_tri_vids(fid);
    FaceExtra2d& x = m_face_extra[fid];
    for (int i = 0; i < 3; ++i) x.rest_pos[i] = m_vertex_attribute[vs[i]].m_posf;
    x.rest_valid = true;
}

void TopoOffsetTriMesh::stamp_plastic_rests()
{
    if (!m_plastic_active) return;
    size_t n = 0;
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        if (!face_is_plastic(fid) && !face_is_released_band(fid)) continue;
        const auto vs = oriented_tri_vids(fid);
        FaceExtra2d& x = m_face_extra[fid];
        for (int i = 0; i < 3; ++i) x.rest_pos[i] = m_vertex_attribute[vs[i]].m_posf;
        x.rest_valid = true;
        ++n;
    }
    (void)n;
}

bool TopoOffsetTriMesh::smooth_plastic_vertex(const Tuple& t)
{
    // The plastic medium's smoothing: rest-shape AMIPS over the one-ring and nothing else. Rest
    // is the shape at the group's start (stamp_plastic_rests), so the term resists only the
    // increment. No equilateral term -- quality in the medium belongs to the operation passes --
    // and no quality veto, which would freeze the flow. Accept on exact inversion of the ring.
    const size_t vid = t.vid(*this);
    const std::vector<size_t> ring = get_one_ring_fids_for_vertex(t);
    for (const size_t fid : ring) {
        if (is_inverted_f(fid)) {
            ++m_smooth_rejects.already_inverted;
            return false;
        }
    }
    std::vector<RestAMIPSEnergy2D::Cell> cells;
    for (const size_t fid : ring) {
        const FaceExtra2d& fx = m_face_extra[fid];
        if (!face_is_plastic(fid) || !fx.rest_valid) continue;
        const auto vs = oriented_tri_vids(fid);
        int k = 0;
        while (k < 3 && vs[k] != vid) ++k;
        if (k == 3) continue;
        RestAMIPSEnergy2D::Cell c;
        // Neighbours: read through the const smoothing_position(), see
        // TriOptimizerMesh::smooth_after().
        c.q1 = smoothing_position(vs[(k + 1) % 3]);
        c.q2 = smoothing_position(vs[(k + 2) % 3]);
        Eigen::Matrix2d R;
        R.col(0) = fx.rest_pos[(k + 1) % 3] - fx.rest_pos[k];
        R.col(1) = fx.rest_pos[(k + 2) % 3] - fx.rest_pos[k];
        const double det = R.determinant();
        if (!(det > 0.)) continue;
        c.rest_inv = R.inverse();
        cells.push_back(c);
    }
    if (cells.empty()) return false;
    auto energy = std::make_shared<RestAMIPSEnergy2D>(std::move(cells), 1.0);
    // The thread's shared solver, criteria set by smoothing_solver().
    polysolve::nonlinear::Solver& solver = smoothing_solver();
    const Vector2d x0 = m_vertex_attribute[vid].m_posf;
    Eigen::VectorXd x = x0;
    bool threw = false;
    try {
        solver.minimize(*energy, x);
    } catch (const std::exception&) {
        threw = true;
    }
    m_newton_plastic.record(solver, threw);
    set_vertex_position(vid, Vector2d(x));
    for (const size_t fid : ring) {
        if (is_inverted(fid)) {
            set_vertex_position(vid, x0);
            ++m_smooth_rejects.inverted;
            return false;
        }
    }
    for (const size_t fid : ring) m_face_attribute[fid].m_quality = get_quality(fid);
    ++m_smooth_rejects.accepted;
    m_released_tube_dirty.store(true, std::memory_order_release); // the boundary may have moved
    return true;
}

void TopoOffsetTriMesh::release_deformable_regions()
{
    // deform_others: from here on the only region-class envelopes are the domain wall and the
    // input complex boundary (EnvelopeSetup::WallComplex), and every other tag region is
    // released: it deforms as plastic medium, see face_is_plastic(). The released set is every
    // input tag the selection does not name, ambient included; it drives face_is_released_band()
    // and the diagnostics. The tubes and the masks come from build_boundary_envelopes().
    std::set<int64_t> source_tags;
    if (!m_offset_params.offsets.empty() && m_offset_params.offsets[0].expression) {
        for (const int64_t t : m_offset_params.offsets[0].expression->tags_involved()) {
            source_tags.insert(t);
        }
    }
    m_source_tags = source_tags;
    m_deform_tags.clear();
    for (const auto& [tag, name] : m_tag_id_to_name) {
        if (source_tags.count(tag) || m_offset_output_tag_ids.count(tag)) continue;
        m_deform_tags.insert(tag);
    }
    build_boundary_envelopes("deform_others", EnvelopeSetup::WallComplex);
    // No released tube: a released boundary is held by nothing, in the operations too.
    m_released_envelope = nullptr;
    m_released_tube_dirty.store(false, std::memory_order_release);

    std::string released;
    for (const int64_t t : m_deform_tags) released += " " + envelope_key_name(t);
    logger().info(
        "[deform_others] released:{} | held: the domain wall and the input complex boundary "
        "({} tubes); every face outside the band is plastic",
        released,
        m_tag_envelopes.size());
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTriMesh::rest_energy_for_vertex(
    const size_t vid) const
{
    // Off with the plastic medium: the final pass minimises equilateral AMIPS alone.
    if (!m_plastic_active) return nullptr;
    std::vector<RestAMIPSEnergy2D::Cell> cells;
    for (const size_t fid : get_one_ring_fids_for_vertex(tuple_from_vertex(vid))) {
        // Released-band cells too: a released object's boundary inside the band has band-labeled
        // ring cells, which face_is_deformable() skips, so without this the boundary is smoothed
        // by pure triangle quality instead of getting the same treatment as the object's boundary
        // outside the band.
        if (!face_is_deformable(fid) && !face_is_released_band(fid)) continue;
        const FaceExtra2d& fx = m_face_extra[fid];
        if (!fx.rest_valid) continue;
        const auto vs = oriented_tri_vids(fid);
        int k = 0;
        while (k < 3 && vs[k] != vid) ++k;
        if (k == 3) continue;
        RestAMIPSEnergy2D::Cell c;
        c.q1 = m_vertex_attribute[vs[(k + 1) % 3]].m_posf;
        c.q2 = m_vertex_attribute[vs[(k + 2) % 3]].m_posf;
        Eigen::Matrix2d R;
        R.col(0) = fx.rest_pos[(k + 1) % 3] - fx.rest_pos[k];
        R.col(1) = fx.rest_pos[(k + 2) % 3] - fx.rest_pos[k];
        const double det = R.determinant();
        if (!(det > 0.)) continue; // a degenerate or inverted rest holds no shape to preserve
        c.rest_inv = R.inverse();
        cells.push_back(c);
    }
    if (cells.empty()) return nullptr;
    // The AMIPS factor the smoother gives this vertex (smoother_amips_weight()), so the rest
    // term and the equilateral quality term it sums with sit at 1:1. As in 3D.
    const double w = smoother_amips_weight(vid);
    return std::make_shared<RestAMIPSEnergy2D>(std::move(cells), w);
}

bool TopoOffsetTriMesh::swap_edge_after(const Tuple& t)
{
    if (!TriOptimizerMesh::swap_edge_after(t)) {
        return false;
    }
    // THE ENERGY RULE for a swap (see tri_energy()): the sum of tri_energy() over the two faces
    // the flip made must be STRICTLY below the sum over the two it replaced, as
    // swap_edge_before() took it (a NaN refuses). The faces beyond the two keep their corners and
    // labels and so their energy: this is the swap's whole change of E. A swap moves no vertex,
    // so both sums are read at the same positions. TriWild's swap rule, strictness included, on
    // the energy instead of AMIPS; strict because a swap pass has to terminate: every accepted
    // swap strictly lowers E. The engine's AMIPS form of it, swap_quality_allowed(), admits
    // everything. A refusal is rolled back by the engine. The 3D twin is swap_after_cells();
    // see there for the measurements behind strictness and the sum. Labels need no writing here:
    // the two new faces take the two old faces' slots, which hold the one label both had.
    const std::optional<Tuple> opp = t.switch_face(*this);
    std::vector<size_t> made{t.fid(*this)};
    if (opp) made.push_back(opp->fid(*this));
    // repulsion_rounds: the complex must stay simplicially embedded (repulsion_embedding_kept()).
    if (m_repulsion_potential && !repulsion_embedding_kept(made)) {
        ++m_repulsion_embed_refused;
        return false;
    }
    // "exact_swept": the two sums, without the band term (band_term_cancels()), plus the band
    // term's change (swap_swept_change()).
    double after = tri_energy_sum(made, !m_swap_band_cancels.local());
    if (swept_rule()) after += swap_swept_change(made);
    if (!(after < m_swap_energy_before.local())) {
        ++iter_cnt_swap_energy_reject; // a NaN refuses
        return false;
    }
    // deform_others: a swap rewires exactly these two faces; their rest is stale.
    for (const size_t fid : made) stamp_rest_face(fid);
    ++iter_cnt_swap;
    return true;
}

bool TopoOffsetTriMesh::collapse_edge_after(const Tuple& t)
{
    const size_t v2_id = collapse_cache.local().v2_id;
    // growth_mode "individual_op_reject": the edges through the survivor are every offset edge
    // the collapse changed (it moves no vertex); refused, the engine rolls the collapse back, as
    // for the energy rule below and in the same place, before the base's hook. As in 3D
    // (collapse_after_connectivity()).
    if (const int before = m_collapse_turned_before.local(); before >= 0) {
        std::set<std::array<size_t, 2>> f;
        turned_front_edges_at(v2_id, f);
        if (int(f.size()) > before) {
            ++m_turned_reject_collapse;
            return false;
        }
    }
    // THE ENERGY RULE for a collapse (see tri_energy()): the sum of tri_energy() over the faces
    // the collapse changed -- the survivor's ring less v2's old faces outside v1's ring, which
    // keep their slots, corners and labels -- may not exceed the sum over v1's old ring
    // (collapse_before_vertex()). A tie passes, as in TriWild's collapse rule, which
    // collapse_quality_allowed() switches off; a NaN refuses.
    //
    // HERE, first, before the base's after-hook: TriMesh::collapse_edge() has committed the
    // connectivity (v1 retired, v2 kept), the faces the collapse keeps keep their slots and
    // labels, and a collapse moves no vertex, so the energy is read from the mesh as it now is.
    // Before the base, not after it, because the base's hook ends in collapse_after_vertex(),
    // which counts the collapse as done: a refusal after it would be counted as a collapse and
    // then rolled back. The 3D twin applies the rule in collapse_after_connectivity(), an engine
    // hook 2D does not have; a refusal here returns false and the engine rolls the collapse back.
    //
    // Not in coarsening, where the engine skips its own collapse rule as well and judges the
    // region after re-smoothing.
    if (!m_coarsen_mode) {
        const CollapseVolume& cv = m_collapse_volume.local();
        double after = 0.;
        for (const size_t fid : get_one_ring_fids_for_vertex(v2_id)) {
            if (std::binary_search(cv.v2_only.begin(), cv.v2_only.end(), fid)) continue;
            after += tri_energy(fid, !cv.band_cancels);
        }
        if (swept_rule()) { // "exact_swept": the band term's change
            // v1's faces that v2 joins keep their slots: the change of their pieces
            // (band_relabel_change()) reads them in v2's ring, where v2's other faces have none
            after += band_swept_sum(cv.swept_x1, cv.swept_x2, cv.swept_edges) +
                     band_relabel_change(get_one_ring_fids_for_vertex(v2_id), cv.swept_pieces);
        }
        if (!(after <= cv.before)) { // a NaN refuses
            ++iter_cnt_collapse_energy_reject;
            return false;
        }
    }
    // repulsion_rounds: the complex must stay simplicially embedded (repulsion_embedding_kept());
    // every face the collapse made is in the survivor's ring. Before the base's hook for the
    // reason the energy rule is.
    if (m_repulsion_potential && !repulsion_embedding_kept(get_one_ring_fids_for_vertex(v2_id))) {
        ++m_repulsion_embed_refused;
        return false;
    }
    if (!TriOptimizerMesh::collapse_edge_after(t)) {
        return false;
    }
    // Coarsening keeps an absolute bar besides, because it runs after the loop and trades
    // elements for nothing but the promise that the result is still good. As in 3D.
    // face_criterion_rel() is the max of AMIPS over stop_energy and the chord measure.
    if (m_coarsen_mode && m_offset_potential) {
        double after = 0.;
        for (const size_t fid : get_one_ring_fids_for_vertex(v2_id)) {
            after = std::max(after, face_criterion_rel(fid));
        }
        if (after > 1.0) {
            ++iter_cnt_collapse_offset_reject;
            return false;
        }
    }
    if (!m_offset_params.sizing_collapse_min) { // see collapse_edge_before()
        m_vertex_attribute[v2_id].m_sizing_scalar = m_collapse_survivor_sizing.local();
    }
    // deform_others: every surviving face at the survivor changed shape (v1 became v2); their
    // rest is stale.
    for (const size_t fid : get_one_ring_fids_for_vertex(v2_id)) {
        stamp_rest_face(fid);
    }
    return true;
}

bool TopoOffsetTriMesh::collapse_edge_before(const Tuple& t)
{
    // The collapse length gate lives in the shared pass, which filters the candidate list against
    // collapsing_l2 scaled by the endpoints' sizing scalars.
    if (!TriOptimizerMesh::collapse_edge_before(t)) {
        return false;
    }
    // The survivor's own sizing scalar, for sizing_collapse_min = false: the base collapse
    // overwrites it with the min of the two, and collapse_edge_after() puts it back.
    // collapse_cache is the base's, filled by the call above; v2 survives.
    m_collapse_survivor_sizing.local() =
        m_vertex_attribute[collapse_cache.local().v2_id].m_sizing_scalar;
    // Applied unconditionally, where the base asks only when both endpoints already sit on a
    // tracked simplex: the offset region is a thin band, so a collapse with one endpoint in the
    // interior can still pinch its two sides together while every tracked surface survives.
    if (!substructure_link_condition(t)) {
        return false;
    }
    // growth_mode "individual_op_reject": the turned edges through the two ends, for
    // collapse_edge_after(); -1 (no rule) when neither end is on the front. As in 3D.
    {
        const auto& cc = collapse_cache.local();
        int& turned = m_collapse_turned_before.local();
        turned = -1;
        if (turned_reject_active() &&
            (m_vertex_extra[cc.v1_id].m_is_on_offset || m_vertex_extra[cc.v2_id].m_is_on_offset)) {
            std::set<std::array<size_t, 2>> f;
            turned_front_edges_at(cc.v1_id, f);
            turned_front_edges_at(cc.v2_id, f);
            turned = int(f.size());
        }
    }
    // The energy rule's before-half is taken in collapse_before_vertex(), which the engine calls
    // before its scoring loop, so that collapse_quality_allowed() can refuse on it early.
    return true;
}

std::vector<TopoOffsetTriMesh::Tuple> TopoOffsetTriMesh::offset_surface_edges_live_at(
    const size_t vid) const
{
    std::vector<Tuple> result;
    std::set<size_t> seen;
    for (const Tuple& e : get_one_ring_edges_for_vertex(tuple_from_vertex(vid))) {
        if (!seen.insert(e.eid(*this)).second) continue;
        if (edge_is_offset_surface_live(e)) result.push_back(e);
    }
    return result;
}

bool TopoOffsetTriMesh::collapse_before_vertex(const size_t v1_id, const size_t v2_id)
{
    // The energy rule's before-half (see CollapseVolume): the sum of tri_energy() over v1's ring,
    // the faces the collapse changes, and v2's other faces, which it does not. Not in
    // coarsening, where the engine skips its own collapse rule as well. As in 3D.
    if (!m_coarsen_mode) {
        std::vector<size_t> ring1 = get_one_ring_fids_for_vertex(v1_id);
        CollapseVolume& cv = m_collapse_volume.local();
        // Without the band term when it cancels (tri_energy(), BAND TERM THAT CANCELS): the new
        // faces are ring1's faces without v2, in their own slots with their own labels.
        cv.band_cancels = band_term_cancels(ring1);
        cv.before = tri_energy_sum(ring1, !cv.band_cancels);
        // "exact_swept": v1's boundary edges of each B_r, for the band term's change, v1 moving
        // onto v2 in all its faces (EXACT SWEPT CHANGE; the faces through v2 go flat), and the
        // pieces of v1's band faces. The collapse changes no face's label: v1's other faces keep
        // theirs in their own slots; their pieces can change, v2 replacing v1 among their
        // corners (band_relabel_change()). As 3D.
        cv.swept_edges.clear();
        cv.swept_pieces.clear();
        if (swept_rule()) {
            cv.swept_x1 = m_vertex_attribute[v1_id].m_posf;
            cv.swept_x2 = m_vertex_attribute[v2_id].m_posf;
            band_swept_edges(v1_id, cv.swept_x1, cv.swept_edges);
            cv.swept_pieces = band_face_pieces(ring1);
        }
        std::sort(ring1.begin(), ring1.end());
        cv.v2_only.clear();
        for (const size_t fid : get_one_ring_fids_for_vertex(v2_id)) {
            if (!std::binary_search(ring1.begin(), ring1.end(), fid)) cv.v2_only.push_back(fid);
        }
        std::sort(cv.v2_only.begin(), cv.v2_only.end());
    }
    // Diagnostic: the flattest face this collapse is about to reshape, read back by
    // record_flatness() in collapse_after_vertex().
    {
        double f = 1.;
        for (const size_t fid : get_one_ring_fids_for_vertex(v1_id)) {
            f = std::min(f, face_flatness(fid));
        }
        m_collapse_parent_flatness.local() = f;
    }

    // The link of the edge about to be collapsed: the only vertices besides v2 whose offset
    // membership this collapse can change. Captured here because the edge no longer exists in
    // collapse_after_vertex(), which is where the refresh runs. See m_collapse_edge_link.
    {
        std::vector<size_t>& link = m_collapse_edge_link.local();
        link.clear();
        for (const size_t fid : get_incident_fids_for_edge(v1_id, v2_id)) {
            for (const size_t w : oriented_tri_vids(fid)) {
                if (w != v1_id && w != v2_id) link.push_back(w);
            }
        }
        wmtk::vector_unique(link);
    }

    const auto& VE = m_vertex_extra;

    // v1 is the vertex the collapse removes; it merges into v2, which keeps its position. A
    // vertex on any tracked boundary -- input complex, region boundary, domain wall alike -- may
    // be removed provided it merges onto a vertex of the same class (the per-class rules below;
    // the base's on_bbox_faces subset rule for the wall), the result stays inside its tags'
    // envelopes, and the substructure link condition survives. That is TriWild's rule for its
    // input surface, applied uniformly; the wall gets no special refusal.

    // Never both surfaces on one vertex: such a vertex sits at distance 0 from the input complex
    // and is asked to sit at target_distance from it at once. Refused here, and asserted
    // independently by check_no_vertex_on_both_surfaces().
    {
        const bool input = VE[v1_id].m_is_on_input || VE[v2_id].m_is_on_input;
        const bool offset = VE[v1_id].m_is_on_offset || VE[v2_id].m_is_on_offset;
        if (input && offset) {
            return false;
        }
    }

    // The front is always length-limited, whatever the pass says: it deliberately has no
    // envelope while it moves, so its sizing field is the only thing bounding its resolution.
    if (!m_collapse_limit_length && VE[v1_id].m_is_on_offset) {
        return false;
    }

    // The base only knows that both endpoints are on SOME tracked surface. A vertex may not leave
    // the particular surface it belongs to, and each class is checked separately because a vertex
    // can be on more than one: satisfying the union is not enough.
    if (VE[v1_id].m_is_on_input && !VE[v2_id].m_is_on_input) {
        return false;
    }
    if (VE[v1_id].m_is_on_offset && !VE[v2_id].m_is_on_offset) {
        return false;
    }
    if (VE[v1_id].m_is_on_region && !VE[v2_id].m_is_on_region) {
        return false;
    }
    return true;
}

void TopoOffsetTriMesh::collapse_pass_begin()
{
    needle_scan("collapse pass");
}

void TopoOffsetTriMesh::collapse_after_vertex(const size_t v1_id, const size_t v2_id)
{
    // Diagnostic. Runs after the collapse is committed, so what it sees is real; the survivor's
    // ring is every face the collapse reshaped.
    for (const size_t fid : get_one_ring_fids_for_vertex(v2_id)) {
        if (get_quality(fid) >= kNeedleQuality) report_needle("COLLAPSE", fid, -1.);
        record_flatness("COLLAPSE", m_collapse_parent_flatness.local(), fid);
    }

    if (m_vertex_extra.at(v1_id).m_is_on_offset) ++iter_cnt_collapse_offset_removed;

    // The base ORs its own m_is_on_surface, which is the union of the two; these say which.
    m_vertex_extra[v2_id].m_is_on_input =
        m_vertex_extra.at(v1_id).m_is_on_input || m_vertex_extra.at(v2_id).m_is_on_input;
    // The offset half is NOT an OR: it is re-derived from the labels, for v2 and for the link of
    // the edge that just died. An OR can only ever add the flag, so a collapse that takes a
    // vertex off the offset boundary would leave it flagged for the rest of the run -- counted
    // by energy_criterion(), smoothed as a front vertex, and unable to satisfy a criterion that
    // measures its distance to a level set it is no longer on. See refresh_offset_membership().
    // As in 3D.
    refresh_offset_membership(v2_id);
    for (const size_t w : m_collapse_edge_link.local()) {
        if (w == v1_id || w == v2_id) continue;
        refresh_offset_membership(w);
    }
    m_vertex_extra[v2_id].m_is_on_region =
        m_vertex_extra.at(v1_id).m_is_on_region || m_vertex_extra.at(v2_id).m_is_on_region;
    // The survivor now carries both vertices' geometry, so it lies on the union of their
    // boundaries. See VertexExtra2d::m_boundary_mask.
    m_vertex_extra[v2_id].m_boundary_mask |= m_vertex_extra.at(v1_id).m_boundary_mask;
    // The construction labels of the survivor and the link, the vertices whose triangles changed
    // (carried_vertex_label()).
    m_vertex_extra[v2_id].label = carried_vertex_label(v2_id);
    for (const size_t w : m_collapse_edge_link.local()) {
        if (w == v1_id || w == v2_id) continue;
        m_vertex_extra[w].label = carried_vertex_label(w);
    }

    // The base calls this only once a collapse has actually gone through.
    ++iter_cnt_collapse;
}

void TopoOffsetTriMesh::split_after_vertex(const size_t v_id)
{
    // The new vertex's classification is set in split_adjust_position(), which the base calls
    // BEFORE its own containment check; this hook runs after it, and the check dispatches on
    // exactly those bits. Only the birth epoch is set here.
    //
    // Read by the needle diagnostics. Assigned rather than OR'd because v_id may be a
    // recycled slot carrying a dead vertex's bits.
    m_vertex_extra[v_id].m_born_epoch = m_op_epoch;
    // repulsion_rounds, before the march: the construction label, which says which vertices are
    // in the input complex and so which edges the march will split (is_marched_edge()). The AND
    // never misses a midpoint in the complex -- an edge in it has both ends in it -- and only
    // over-marks the midpoint of an edge that joins two complex vertices off the complex, which
    // makes the split pass refuse more, never less. repulsion_smoothing() recomputes every label
    // from the tags after the pass (relabel_input_complex()). As in 3D.
    if (m_repulsion_potential) {
        const auto& c = m_opt_split_cache.local();
        m_vertex_extra[v_id].label =
            (m_vertex_extra[c.v1_id].label != 0 && m_vertex_extra[c.v2_id].label != 0) ? 1 : 0;
    }

    // Diagnostic, see the header. Every face incident to the midpoint was created by this split,
    // so a MAX_ENERGY face here is one this split manufactured; a split is never refused on
    // quality, so nothing upstream would have stopped it.
    const auto& sc = m_opt_split_cache.local();
    const double parent_q = sc.parent_q_max;
    for (const size_t fid : get_one_ring_fids_for_vertex(tuple_from_vertex(v_id))) {
        const double q = get_quality(fid);
        if (q >= MAX_ENERGY) ++m_deg_split_created;
        if (q >= kNeedleQuality) report_needle("SPLIT", fid, parent_q);
        record_flatness("SPLIT", sc.parent_flatness, fid);
    }

    // The children's region labels are set in split_adjust_position(), early enough for the
    // split's own containment check to see them.

    // deform_others: every face at the midpoint was created by this split and the snapshot copy
    // gave each the parent's rest -- re-stamp, or a child measures itself against a triangle
    // twice its size (see FaceExtra2d::rest_valid).
    for (const size_t fid : get_one_ring_fids_for_vertex(tuple_from_vertex(v_id))) {
        stamp_rest_face(fid);
    }
}

bool TopoOffsetTriMesh::split_adjust_position(const size_t v_id, const std::vector<Tuple>&)
{
    // Carry each parent's construction label onto its two children. Not automatic, because the
    // children land in fresh fid slots, whose m_face_extra defaults to label 0 (or, for a
    // recycled slot, to the previous occupant's label). split_edge_before() records the parents'
    // labels keyed by APEX -- the vertex opposite the split edge, shared by both children of a
    // parent and by no other parent -- the same key TriOptimizerMesh::split_edge_after resolves
    // its own FaceAttributes cache with, so the label cannot land anywhere the tags did not.
    //
    // Safe before the base's own checks: m_face_extra is registered with m_face_attr_group, so a
    // refused split rolls it back. Idempotent too, since two children reuse their parent's fid.
    const auto& c = m_opt_split_cache.local();
    if (c.face_label.empty()) return true; // a marching-mode split; those set labels themselves

    // The new vertex's classification, and it must be set here. The base's split_edge_after()
    // asks surface_segment_is_outside() about both new segments BEFORE it calls
    // split_after_vertex(), and that dispatch reads the endpoints' own bits, so setting them in
    // the later hook leaves the check reading a recycled slot -- all-false for a fresh one, which
    // makes the dispatch return nullptr and the containment check a silent no-op.
    //
    // Same rollback safety as the face labels below: m_vertex_extra is registered with
    // m_vertex_attr_group (see the constructor), so a refused split undoes these too.
    const auto& e = split_cache.local().old_e_attrs;
    m_vertex_extra[v_id].m_is_on_offset =
        e.m_is_surface_fs && e.m_surface_class == OFFSET_SURFACE_CLASS;
    // Class 0 covers the input complex AND every other region boundary, so the two flags are
    // narrowed from the ENDPOINTS rather than read off the class: a midpoint is on the complex
    // only if the whole edge was, which is the same AND rule the boundary mask follows.
    m_vertex_extra[v_id].m_is_on_input =
        m_vertex_extra[c.v1_id].m_is_on_input && m_vertex_extra[c.v2_id].m_is_on_input;
    // ... but m_is_on_region is the split edge's own class, not an endpoint AND: a bare AND
    // over-claims on a chord whose two ends happen to share a region, which is what the mask gate
    // exists to prevent. 3D reads is_edge_on_region() for the same reason.
    m_vertex_extra[v_id].m_is_on_region =
        e.m_is_surface_fs && e.m_surface_class != OFFSET_SURFACE_CLASS;
    // Assigned, not OR'd -- v_id may be a recycled slot carrying a dead vertex's bits. The bits
    // are the endpoints' mask AND, captured in split_edge_before(); the class gate right above is
    // what keeps a chord's midpoint maskless, and the mask is inert on a vertex with no region.
    m_vertex_extra[v_id].m_boundary_mask =
        m_vertex_extra[v_id].m_is_on_region ? c.edge_bits : uint64_t(0);

    for (const size_t endpoint : {c.v1_id, c.v2_id}) {
        const simplex::Edge new_edge(endpoint, v_id);
        for (const size_t fid : get_incident_fids_for_edge(endpoint, v_id)) {
            const size_t apex = simplex_from_face(fid).opposite_vertex(new_edge).id();
            const auto it = c.face_label.find(apex);
            if (it == c.face_label.end()) continue; // unreachable; leave the slot alone
            m_face_extra[fid].label = it->second;
        }
    }
    // The construction label, from the flags and face labels just set (carried_vertex_label()).
    m_vertex_extra[v_id].label = carried_vertex_label(v_id);
    return true; // the position itself is the base's business, and it is happy with it
}

bool TopoOffsetTriMesh::smooth_before(const Tuple& t)
{
    ++m_smooth_trace.attempted;
    const size_t vid = t.vid(*this);
    // The final pass does not move the front: it is converged by then, its smoothing there
    // would be AMIPS alone with the front free anywhere inside the offset tube, and nothing
    // follows to put it back.
    if (m_freeze_front && m_vertex_extra[vid].m_is_on_offset) return false;

    // Diagnostic, recorded for every visit; only visits whose ring already holds a needle are
    // counted, and smooth_after() reads this back.
    auto& pre = m_needle_pre.local();
    pre = {ring_max_quality(vid), m_vertex_attribute[vid].m_posf};
    if (pre.first >= kNeedleQuality) ++m_needle_smooth_offered;

    // The base's smooth_before minus its bounding-box refusal, which is why this does not call
    // it: the base freezes every vertex on the domain wall. Here the wall is a region boundary
    // held in ambient's tag envelope like any other, so its vertices are smoothed and the
    // containment check decides whether the move survives. As in 3D.
    //
    // Rounding still has to happen, and its failure still refuses the move.
    const bool rounded_now = round(t);
    if (!m_vertex_attribute[vid].m_is_rounded && !rounded_now) {
        ++m_smooth_trace.before_unrounded;
        return false;
    }

    return true;
}

polysolve::nonlinear::Solver& TopoOffsetTriMesh::smoothing_solver()
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

bool TopoOffsetTriMesh::smooth_vertex_2d_counted(
    const Tuple& t,
    const optimization::SmoothVertexOptions& opts,
    optimization::NewtonCounters& newton,
    bool* solved)
{
    // smooth_vertex_2d() refuses before solving exactly when a ring face is inverted in floats,
    // and otherwise solves once (two_stage is off on every path of this component), so this
    // precheck says whether a solve ran without touching the engine. The 3D engine records the
    // solve itself, through smooth_vertex_3d()'s NewtonCounters argument.
    bool will_solve = true;
    for (const size_t fid : get_one_ring_fids_for_vertex(t)) {
        if (is_inverted_f(fid)) {
            will_solve = false;
            break;
        }
    }
    const bool ok =
        optimization::smooth_vertex_2d(*this, t, opts, m_solver.local(), &m_smooth_rejects);
    if (will_solve) newton.record(*m_solver.local(), /*did_throw=*/false); // see m_newton
    if (solved) *solved = will_solve;
    return ok;
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTriMesh::amips3_energy(
    const size_t vid) const
{
    // See the declaration. The faces exactly as the shared smoother assembles them for its own
    // AMIPS term (smooth_vertex_2d()): the moving vertex first, winding preserved. 3D's plastic
    // cells (use_rest_pose, VolAMIPSEnergy3D against a stamped rest) have no twin: no 2D face's
    // energy reads a rest (tri_energy()).
    std::vector<std::array<double, 6>> cells;
    for (const size_t fid : get_one_ring_fids_for_vertex(vid)) {
        const std::array<size_t, 3> vs = oriented_tri_vids(fid);
        int k = 0;
        while (k < 3 && vs[size_t(k)] != vid) ++k;
        if (k == 3) continue;
        std::array<double, 6> c;
        for (int i = 0; i < 3; ++i) {
            const Vector2d& p = m_vertex_attribute[vs[size_t((k + i) % 3)]].m_posf;
            c[size_t(2 * i)] = p[0];
            c[size_t(2 * i + 1)] = p[1];
        }
        cells.push_back(c);
    }
    // (w / s^3) sum area AMIPS^3, the energy's AMIPS term -- an area, as the band term is.
    return std::make_shared<CubedAMIPSEnergy2D>(std::move(cells), band_amips_weight());
}

bool TopoOffsetTriMesh::smooth_nonfront_vertex(const Tuple& t)
{
    // See the declaration. TriOptimizerMesh::smooth_after()'s options, two changes: the objective
    // comes from smoothing_extra_energy() (w_amips 0: the smoother adds no AMIPS term of its own),
    // and the engine's AMIPS veto is replaced by the energy's, on the engine's scope, which
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
    // The rule is E itself: the sum of tri_energy() over the ring may not rise (a tie passes, a
    // NaN refuses). A vertex held on a region boundary takes the engine's projected step, whose
    // acceptance is that rule already (smoothing_ring_energy(): strictly below), so it needs no
    // veto here; the engine's own condition (smooth_vertex_2d(): a pull envelope under the
    // projected mode; 3D also asks m_is_on_surface, which its engine asks). Every other vertex is
    // vetoed after its solve. A smoothing move changes no label and moves one vertex, so the ring
    // holds every face whose energy it can change. As 3D's.
    const std::vector<size_t> ring = get_one_ring_fids_for_vertex(vid);
    const Vector2d x0 = m_vertex_attribute[vid].m_posf;
    const bool projected =
        opts.smoothing_mode == optimization::SmoothVertexOptions::SmoothingMode::Projected &&
        smoothing_energy_envelope(vid) != nullptr;
    // "exact_swept": the projected step compares smoothing_ring_energy(), which takes the band
    // term's change from the position at its first call on this vertex; every other move is
    // accepted by swept_accept() (EXACT SWEPT CHANGE), from the ring's AMIPS part before
    const bool swept = swept_rule();
    if (swept) m_swept_stored.local().vid = vid;
    const double amips_before = swept && !projected ? tri_energy_sum(ring, false) : 0.;
    const double before = projected || swept ? 0. : tri_energy_sum(ring);
    // BAND TERM THAT CANCELS (tri_energy()) for the move, tested at the old position and the new
    // one, as 3D's smooth_nonfront_vertex(); false in 2D (band_term_cancels()) but under
    // "exact_swept", which compares its own sums above.
    const bool cancels_before = !projected && !swept && band_term_cancels(ring);
    const double before_no_band = cancels_before ? tri_energy_sum(ring, false) : 0.;
    if (!smooth_vertex_2d_counted(t, opts, m_newton)) {
        return false;
    }
    if (projected) return true;
    if (swept) {
        if (!swept_accept(vid, ring, x0, amips_before, m_swept_nonfront)) {
            --m_smooth_rejects.accepted;
            ++m_smooth_rejects.quality;
            return false;
        }
        return true;
    }
    const bool cancels = cancels_before && band_term_cancels(ring);
    const double after = tri_energy_sum(ring, !cancels);
    if (!(after <= (cancels ? before_no_band : before))) { // a NaN refuses; a tie passes
        --m_smooth_rejects.accepted;
        ++m_smooth_rejects.quality;
        return false;
    }
    return true;
}

bool TopoOffsetTriMesh::smooth_after(const Tuple& t)
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
                // Per-event forensic detail at INFO, like report_needle(); the population sweep
                // is what warns.
                logger().info(
                    "[needle-smooth #{}] vid {} ring max {:.6g} -> {:.6g} ({:.3g}x) | moved "
                    "{:.6g} | input {} offset {} region {} mask 0x{:x} | pos ({:.17g}, {:.17g})",
                    m_needle_smooth_reports.load(),
                    vid,
                    pre.first,
                    after,
                    after / std::max(pre.first, 1e-300),
                    moved,
                    ve.m_is_on_input,
                    ve.m_is_on_offset,
                    ve.m_is_on_region,
                    ve.m_boundary_mask,
                    m_vertex_attribute[vid].m_posf[0],
                    m_vertex_attribute[vid].m_posf[1]);
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

    // The plastic medium: a vertex whose whole ring is plastic and which no envelope holds flows
    // under rest-shape AMIPS alone (see smooth_plastic_vertex). Wall vertices, and vertices
    // touching the band or the complex, keep the standard path so the interfaces stay under the
    // usual rules. As in 3D.
    if (m_plastic_active && !ve.m_is_on_offset && !smoothing_containment_envelope(vid)) {
        bool all_plastic = true;
        for (const size_t fid : get_one_ring_fids_for_vertex(t)) {
            if (!face_is_plastic(fid)) {
                all_plastic = false;
                break;
            }
        }
        if (all_plastic) {
            const bool okp = smooth_plastic_vertex(t);
            ++m_smooth_trace.interior_attempted;
            return okp;
        }
    }

    // A front vertex goes through the shared smoother -- same solver, line search and accept
    // tests as every other vertex -- with the offset's options: its objective carries the offset
    // terms (smoothing_extra_energy), and the veto is on tri_energy() instead of AMIPS, since a
    // front vertex must be able to worsen its ring's shape on the way to the level set but not
    // the energy, which charges both (see smooth_front_vertex()). A front vertex reaches here
    // only outside the final pass: smooth_before() refuses it while m_freeze_front is set. Every
    // other vertex goes through smooth_nonfront_vertex(), vetoed on the same sum. As in 3D.
    if (ve.m_is_on_offset) {
        // growth_mode "individual_op_reject": the turned offset edges through vid before the
        // move, which may not raise them (every edge it changes goes through vid). As in 3D, but
        // around smooth_front_vertex() rather than after its veto inside it, so that the 2D
        // smoother's own edits (the exact_swept step) merge with this one: the same test at the
        // same point of the move, after the veto passed. A move refused here was counted by
        // DEBUG_crossings (log-only) and marked the released tube dirty (a needless refresh).
        int turned_before = -1;
        if (turned_reject_active()) {
            std::set<std::array<size_t, 2>> f;
            turned_front_edges_at(vid, f);
            turned_before = int(f.size());
        }
        bool ok = smooth_front_vertex(t);
        if (ok && turned_before >= 0) {
            std::set<std::array<size_t, 2>> f;
            turned_front_edges_at(vid, f);
            if (int(f.size()) > turned_before) {
                ++m_turned_reject_smooth;
                --m_smooth_rejects.accepted;
                ++m_smooth_rejects.quality;
                ok = false;
            }
        }
        if (ok) ++m_smooth_trace.offset_accepted;
        return ok;
    }
    return smooth_nonfront_vertex(t);
}

double TopoOffsetTriMesh::front_vertex_normal_gradient(const size_t vid) const
{
    // ||grad F|| at the vertex's current position, F the objective smooth_front_vertex()
    // minimises, taken along the move direction n: |grad F . n| (see below).
    const Vector2d x = m_vertex_attribute[vid].m_posf;
    Eigen::VectorXd xv = x, g(2);
    front_objective(vid)->gradient(xv, g);
    if (!g.allFinite()) return std::numeric_limits<double>::infinity();
    // Along the field normal whatever the placement mode: the test asks whether the front is where
    // the field wants it. Tangential motion is the flat direction of the energy -- only w x AMIPS
    // acts along the front -- where a tiny gradient means a large Newton step.
    const Vector2d n = front_vertex_move_direction(vid);
    if (n.squaredNorm() > 0.) return std::abs(n.dot(Vector2d(g)));
    return g.norm();
}

std::vector<double> TopoOffsetTriMesh::front_ring_measures() const
{
    // energy_criterion()'s ring measure, chord for chord: one edge_offset_term() per front chord
    // with two front ends, added to each end; an unmeasurable chord leaves its ends without a
    // ring measure. As in 3D.
    const auto front = [&](const size_t vid) {
        return m_vertex_extra[vid].m_is_on_offset && m_vertex_attribute[vid].m_is_rounded;
    };
    std::vector<double> sum(vert_capacity(), 0.);
    std::vector<size_t> n(vert_capacity(), 0);
    std::vector<char> bad(vert_capacity(), 0);
    for (const auto& e : offset_surface_edges()) {
        if (!front(e[0]) || !front(e[1])) continue;
        const double term = edge_offset_term(e[0], e[1]);
        for (const size_t u : e) {
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

double TopoOffsetTriMesh::ring_measure_at(const size_t vid) const
{
    const auto front = [&](const size_t u) {
        return m_vertex_extra[u].m_is_on_offset && m_vertex_attribute[u].m_is_rounded;
    };
    double sum = 0.;
    size_t n = 0;
    for (const Tuple& et : offset_surface_edges_live_at(vid)) {
        const auto e = get_edge_vids(et);
        if (!front(e[0]) || !front(e[1])) continue;
        const double term = edge_offset_term(e[0], e[1]);
        if (term < 0.) return std::numeric_limits<double>::quiet_NaN();
        sum += term;
        ++n;
    }
    return n > 0 ? std::sqrt(sum / double(n)) : std::numeric_limits<double>::quiet_NaN();
}

void TopoOffsetTriMesh::crossing_snapshot(
    const std::string& pass,
    const bool compare,
    const bool match_positions)
{
    std::vector<double> r = front_ring_measures();
    std::vector<Vector2d> pos(vert_capacity());
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

void TopoOffsetTriMesh::update_attributes()
{
    TriOptimizerMesh::update_attributes();
    // DEBUG_held_topology: after every pass of the base (split, collapse, swap, smoothing), once
    // the baseline is taken. As in 3D.
    if (m_held_topology) held_topology_check(m_debug_pass_name);
    if (!m_offset_params.debug_crossings || m_round <= 0) return;
    const std::string& p = m_debug_pass_name;
    if (p == "split" || p == "collapse" || p == "swap") {
        crossing_snapshot(p, true, true);
    } else if (!m_cross_valid) {
        crossing_snapshot("start", false, false);
    }
}

void TopoOffsetTriMesh::log_smoothing_pass_accounting()
{
    // Per pass, after the engine's own "\tsmooth:" line: the background's Newton line (which the
    // 3D engine prints itself), then the 3D component's lines. The plastic line only when the
    // plastic medium solved anything.
    logger().info("\tnewton, smooth_after: {}", m_newton.to_string());
    logger().info("\tnewton, front: {}", m_newton_front.to_string());
    logger().info(
        "\tfront veto: fired {} of {} front moves that reached it (ring sum of tri_energy rose)",
        m_front_veto_fired.exchange(0),
        m_front_veto_asked.exchange(0));
    if (swept_rule()) {
        // "exact_swept": swept_accept()'s outcomes, front and other moves apart, as 3D's
        for (auto* c : {&m_swept_front, &m_swept_nonfront}) {
            logger().info(
                "\t[exact_swept] {} moves: {} solves, accepted at full step {}, after 1 / 2 / 3 / "
                "4 halvings {} / {} / {} / {}, refused {}",
                c == &m_swept_front ? "front" : "other",
                c->solves.exchange(0),
                c->accepted[0].exchange(0),
                c->accepted[1].exchange(0),
                c->accepted[2].exchange(0),
                c->accepted[3].exchange(0),
                c->accepted[4].exchange(0),
                c->refused.exchange(0));
        }
        logger().info(
            "\t[exact_swept] search model derivatives: {} evaluations, {} with an edge part "
            "labelled by rule (a) (a curved bisector), {} from the swept triangles, {} at x0 with "
            "their Hessian, {} from the ring's band faces (front_edge_derivs() threw)",
            m_swept_derivs.evals.exchange(0),
            m_swept_derivs.rule_a.exchange(0),
            m_swept_derivs.swept_tris.exchange(0),
            m_swept_derivs.swept_x0.exchange(0),
            m_swept_derivs.ring_cells.exchange(0));
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
    if (m_newton_plastic.solves() > 0) {
        logger().info("\tnewton, plastic: {}", m_newton_plastic.to_string());
    }
    m_newton.reset();
    m_newton_front.reset();
    m_newton_plastic.reset();
    if (m_offset_params.debug_crossings && m_round > 0) crossing_snapshot("smooth", true, false);
}

void TopoOffsetTriMesh::optimization_debug_checkpoint()
{
    // The 2D engine calls this after every smoothing pass (smooth_all_vertices()) and nowhere
    // else on this component's paths: the operation passes write their frame directly. So this is
    // where the 3D engine's per-pass hook goes, before the frame the base default writes.
    log_smoothing_pass_accounting();
    TriOptimizerMesh::optimization_debug_checkpoint();
}

void TopoOffsetTriMesh::audit_surface_containment(const std::string& when) const
{
    // See the declaration: the shared sanity check says an edge is outside, this says which
    // envelope refused it and by how much, because offset-class and region-class are different
    // bugs with different fixes.
    struct Bad
    {
        size_t a = 0, b = 0;
        uint64_t mask = 0;
        bool offset_class = false;
        double worst_d = 0.; ///< furthest ALONG-SEGMENT distance to a real member tube
        double worst_end_d = 0.; ///< furthest ENDPOINT distance -- 0 means both ends are inside
        double worst_u = 0.; ///< where along the segment worst_d sits; ~0.5 means a chord bulge
        double len = 0.;
        int worst_tag = -1;
    };
    std::vector<Bad> bad;
    size_t n_tracked = 0, n_offset_class = 0, n_region_class = 0, n_other = 0;
    size_t bad_offset = 0, bad_region = 0, bad_other = 0;

    for (const Tuple& e : get_edges()) {
        const size_t eid = e.eid(*this);
        if (!m_edge_attribute[eid].m_is_surface_fs) continue;
        ++n_tracked;
        const std::array<size_t, 2> vids = {{e.vid(*this), e.switch_vertex(*this).vid(*this)}};
        // The edge's own stored class is ground truth here: this sweep runs between operations,
        // so no attribute slot is mid-write. Classifying by `mask != 0` instead repeats the
        // endpoint-AND over-claim and files offset edges under "region".
        const bool is_offset = edge_is_offset(eid);
        const uint64_t mask = is_offset ? uint64_t(0) : edge_mask(vids);
        if (is_offset)
            ++n_offset_class;
        else if (mask != 0)
            ++n_region_class;
        else
            ++n_other;

        // Exactly the dispatch the sanity check uses, so this cannot disagree with it.
        if (!surface_segment_is_outside(vids[0], vids[1], m_edge_attribute[eid])) continue;

        Bad r;
        r.a = vids[0];
        r.b = vids[1];
        r.mask = mask;
        r.offset_class = is_offset;
        r.len = (m_vertex_attribute[vids[1]].m_posf - m_vertex_attribute[vids[0]].m_posf).norm();
        if (r.offset_class)
            ++bad_offset;
        else if (mask != 0)
            ++bad_region;
        else
            ++bad_other;

        // How far outside, per real member -- never the composite. Sampled along the segment and
        // not just at the endpoints, because that is what is_outside(edge) does and it is the
        // distinction being drawn: endpoints at distance 0 with a large interior maximum is a
        // chord spanning boundary the tube follows around, not a boundary edge that has drifted.
        const auto probe = [&](const std::shared_ptr<Envelope>& env, int tag) {
            if (!env) return;
            for (const size_t v : vids) {
                const double d =
                    std::sqrt(std::max(env->squared_distance(m_vertex_attribute[v].m_posf), 0.));
                if (d > r.worst_end_d) r.worst_end_d = d;
            }
            constexpr int kSamples = 16;
            const Vector2d& pa = m_vertex_attribute[vids[0]].m_posf;
            const Vector2d& pb = m_vertex_attribute[vids[1]].m_posf;
            for (int k = 0; k <= kSamples; ++k) {
                const double u = double(k) / kSamples;
                const double d =
                    std::sqrt(std::max(env->squared_distance(Vector2d(pa + u * (pb - pa))), 0.));
                if (d > r.worst_d) {
                    r.worst_d = d;
                    r.worst_tag = tag;
                    r.worst_u = u;
                }
            }
        };
        if (mask != 0) {
            for (const auto& [tag, env] : m_tag_envelopes) {
                const auto it = m_tag_bit.find(tag);
                if (it != m_tag_bit.end() && (mask & (uint64_t(1) << it->second))) probe(env, tag);
            }
        } else if (r.offset_class) {
            probe(m_offset_envelope, -1);
        }
        bad.push_back(r);
    }

    if (bad.empty()) {
        logger().info(
            "\t[containment {}] clean: 0 of {} tracked edges outside ({} offset-class, {} "
            "region-class, {} neither)",
            when,
            n_tracked,
            n_offset_class,
            n_region_class,
            n_other);
        return;
    }

    logger().warn(
        "\t[containment {}] {} of {} tracked edges are OUTSIDE their envelope: {} OFFSET-class "
        "(the offset tube), {} REGION-class (a tag tube / junction intersection), {} neither "
        "| population: {} offset-class, {} region-class, {} neither",
        when,
        bad.size(),
        n_tracked,
        bad_offset,
        bad_region,
        bad_other,
        n_offset_class,
        n_region_class,
        n_other);

    std::sort(bad.begin(), bad.end(), [](const Bad& x, const Bad& y) {
        return x.worst_d > y.worst_d;
    });
    const size_t show = std::min<size_t>(bad.size(), 8);
    for (size_t i = 0; i < show; ++i) {
        const Bad& r = bad[i];
        logger().warn(
            "\t  [{}] edge [{}, {}] mask 0x{:x} len {:.6g} | ({:.6g}, {:.6g}) -- ({:.6g}, {:.6g}) "
            "| OUT BY {:.6g} at u={:.3g} along the segment; endpoints out by {:.6g}{}",
            r.offset_class ? "offset" : (r.mask ? "region" : "other "),
            r.a,
            r.b,
            r.mask,
            r.len,
            m_vertex_attribute[r.a].m_posf.x(),
            m_vertex_attribute[r.a].m_posf.y(),
            m_vertex_attribute[r.b].m_posf.x(),
            m_vertex_attribute[r.b].m_posf.y(),
            r.worst_d,
            r.worst_u,
            r.worst_end_d,
            r.worst_tag >= 0 ? fmt::format(" (tag {})", r.worst_tag) : std::string());
        // Why it is tracked and why it claims that mask: the per-endpoint bits the AND is taken
        // over, and the edge's own stored class. A chord over-claim shows as two endpoints that
        // each legitimately carry the bit, joined by an edge whose own class is not region.
        const auto& ea = m_vertex_extra[r.a];
        const auto& eb = m_vertex_extra[r.b];
        const auto [tup, eid] = tuple_from_edge({{r.a, r.b}});
        logger().warn(
            "\t      endpoints: v{} mask 0x{:x} on_region {} on_offset {} on_input {} | v{} mask "
            "0x{:x} on_region {} on_offset {} on_input {} || edge: surface_fs {} region {} offset "
            "{} label {} | live boundary bits 0x{:x}",
            r.a,
            ea.m_boundary_mask,
            ea.m_is_on_region,
            ea.m_is_on_offset,
            ea.m_is_on_input,
            r.b,
            eb.m_boundary_mask,
            eb.m_is_on_region,
            eb.m_is_on_offset,
            eb.m_is_on_input,
            m_edge_attribute[eid].m_is_surface_fs,
            edge_is_region(eid),
            edge_is_offset(eid),
            m_edge_extra[eid].label,
            edge_boundary_bits(tup));
    }
}

void TopoOffsetTriMesh::log_region_edge_mask_health(const std::string& when) const
{
    // Two counts, one invariant and one expectation. The invariant is on the stored masks: every
    // tracked region edge must dispatch to an envelope, so edge_mask() of its endpoints -- the
    // expression surface_envelope_for_edge() uses -- must be nonzero, and a zero is a propagation
    // hole. The expectation is that the LIVE bits go quiet: the band replaces the tags of every
    // face it grows through, which is why the masks must be propagated, not rederived.
    int n_region = 0, n_unmasked = 0, n_released = 0, n_live_dead = 0, n_wall = 0;
    int n_band = 0, n_outside = 0, n_mixed = 0, n_ends_offset = 0, n_ends_input = 0;
    size_t worst = size_t(-1);
    for (const Tuple& e : get_edges()) {
        const size_t eid = e.eid(*this);
        if (!edge_is_region(eid)) continue;
        ++n_region;
        if (!e.switch_face(*this)) ++n_wall;
        if (edge_boundary_bits(e) == 0) ++n_live_dead;
        const size_t va = e.vid(*this), vb = e.switch_vertex(*this).vid(*this);
        if (edge_mask({va, vb}) != 0) continue;
        // A quiet edge bounds nothing any more: the band retagged both its sides, so there is no
        // boundary left to hold and a zero mask on it is moot, not a violation.
        if (edge_boundary_bits(e) == 0) continue;
        // A released boundary is freed on purpose -- deform_others zeroes its endpoint masks by
        // this same symmetric-difference test -- so a zero mask there is the feature, not a
        // propagation hole. Everything else still violates the invariant and is counted below.
        if (!m_deform_tags.empty()) {
            if (const std::optional<Tuple> opp0 = e.switch_face(*this)) {
                CellTag edge_tags;
                const auto& t0 = m_face_attribute[e.fid(*this)].tags;
                const auto& t1 = m_face_attribute[opp0->fid(*this)].tags;
                std::set_symmetric_difference(
                    t0.begin(),
                    t0.end(),
                    t1.begin(),
                    t1.end(),
                    std::inserter(edge_tags, edge_tags.begin()));
                bool released_here = false;
                for (const int64_t t : edge_tags) {
                    if (m_deform_tags.count(t)) released_here = true;
                }
                if (released_here) {
                    ++n_released;
                    continue;
                }
            }
        }
        ++n_unmasked;
        if (worst == size_t(-1)) worst = eid;
        // The first few in full: what is an uncontained region edge? Endpoint masks say which end
        // lost the chain; positions say where it sits (on the complex, at the front, ...).
        if (n_unmasked <= 6) {
            const auto& A = m_vertex_attribute[va];
            const auto& B = m_vertex_attribute[vb];
            const auto& EA = m_vertex_extra[va];
            const auto& EB = m_vertex_extra[vb];
            const std::optional<Tuple> opp2 = e.switch_face(*this);
            logger().warn(
                "\t    unmasked e{}: v{}(mask 0x{:x} in/reg/off {}{}{} bbox {}) at "
                "({:.4g},{:.4g})  --  v{}(mask 0x{:x} in/reg/off {}{}{} bbox {}) at "
                "({:.4g},{:.4g}) | labels {} vs {}",
                eid,
                va,
                EA.m_boundary_mask,
                int(EA.m_is_on_input),
                int(EA.m_is_on_region),
                int(EA.m_is_on_offset),
                A.on_bbox_faces.size(),
                A.m_posf.x(),
                A.m_posf.y(),
                vb,
                EB.m_boundary_mask,
                int(EB.m_is_on_input),
                int(EB.m_is_on_region),
                int(EB.m_is_on_offset),
                B.on_bbox_faces.size(),
                B.m_posf.x(),
                B.m_posf.y(),
                m_face_extra[e.fid(*this)].label,
                opp2 ? std::to_string(m_face_extra[opp2->fid(*this)].label) : std::string("-"));
        }
        // Where the unmasked ones sit, which is what says who created them.
        const size_t f0 = e.fid(*this);
        const std::optional<Tuple> opp = e.switch_face(*this);
        const bool b0 = face_is_offset_band(f0);
        const bool b1 = opp && face_is_offset_band(opp->fid(*this));
        if (b0 && b1) {
            ++n_band;
        } else if (!b0 && !b1) {
            ++n_outside;
        } else {
            ++n_mixed;
        }
        const size_t a = e.vid(*this), b = e.switch_vertex(*this).vid(*this);
        if (m_vertex_extra[a].m_is_on_offset && m_vertex_extra[b].m_is_on_offset) ++n_ends_offset;
        if (m_vertex_extra[a].m_is_on_input && m_vertex_extra[b].m_is_on_input) ++n_ends_input;
    }
    logger().info(
        "\t[envelope health @ {}] {} region-boundary edges tracked ({} on the wall) | {} freed "
        "by deform_others (released boundaries; expected) | {} with a ZERO stored mask (the "
        "invariant; must be 0) | {} with quiet LIVE bits (expected once the band retags the "
        "faces it grew through)",
        when,
        n_region,
        n_wall,
        n_released,
        n_unmasked,
        n_live_dead);

    // What the tags actually are, band versus not: a region boundary swallowed by the band loses
    // its tag difference, and its envelope with it.
    {
        std::map<std::string, std::pair<int, int>> hist; // tag set -> (band faces, other faces)
        for (const Tuple& f : get_faces()) {
            const size_t fid = f.fid(*this);
            std::string key;
            for (const int64_t t : m_face_attribute[fid].tags) {
                key += (key.empty() ? "" : ",") + std::to_string(t);
            }
            if (key.empty()) key = "-";
            auto& e = hist[key];
            (face_is_offset_band(fid) ? e.first : e.second) += 1;
        }
        std::string tags;
        for (const auto& [k, v] : hist) {
            tags += fmt::format(
                "{}[{}] band {} / other {}",
                tags.empty() ? "" : " | ",
                k,
                v.first,
                v.second);
        }
        std::string bits;
        for (const auto& [t, b] : m_tag_bit) {
            bits += fmt::format("{}{}->bit{}", bits.empty() ? "" : " ", t, b);
        }
        logger()
            .info("\t[envelope health @ {}] faces by tag set: {} | tag bits: {}", when, tags, bits);
    }
    if (n_unmasked > 0) {
        logger().warn(
            "\t[envelope health @ {}] {} of {} tracked region-boundary edges ({:.1f}%) are "
            "contained by NOTHING (released boundaries already excluded): their endpoints' "
            "stored masks AND to zero, so surface_envelope_for_edge() has no envelope to hold "
            "them to. Either a propagation hole, or collateral of deform_others' vertex freeing "
            "(a freed vertex shared with a KEPT boundary takes that boundary's edges with it).",
            when,
            n_unmasked,
            n_region,
            100.0 * double(n_unmasked) / double(std::max(n_region, 1)));
        logger().warn(
            "\t[envelope health @ {}] of those {}: {} lie between two BAND faces, {} between two "
            "non-band faces, {} straddle the band edge | {} have both ends on the offset, {} both "
            "ends on the input complex | first is e{}",
            when,
            n_unmasked,
            n_band,
            n_outside,
            n_mixed,
            n_ends_offset,
            n_ends_input,
            worst);
    }
}

void TopoOffsetTriMesh::log_smooth_trace() const
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


void TopoOffsetTriMesh::log_refine_block_census(const std::string& when, const double filter_energy)
    const
{
    // See the declaration for what each verdict means and for the two shortcuts taken.
    enum Verdict { kShort = 0, kValence, kContain, kFree, kNVerdict };
    static const char* kName[kNVerdict] = {"short", "valence", "contain", "free"};

    const double l = std::max(m_params.l, 1e-16);
    const size_t val_thresh = m_params.split_high_valence_threshold > 0
                                  ? size_t(m_params.split_high_valence_threshold)
                                  : std::numeric_limits<size_t>::max();

    std::array<size_t, kNVerdict> edge_hist{};
    std::array<size_t, kNVerdict> face_first{}; ///< a face's BEST edge -- its actual prospect
    size_t n_bad = 0, n_inverted = 0, n_any_free = 0;
    size_t n_contain_offset = 0, n_contain_region = 0; ///< which envelope did the refusing

    /// One exemplar per face-level verdict: the worst-quality face that got it.
    struct Ex
    {
        double q = -1.;
        size_t fid = 0;
        Vector2d c = Vector2d::Zero();
        double dist = -1., phi_over_c = -1., sizing = 0.;
        std::array<double, 3> len{}, gate{};
        std::array<int, 3> verd{{-1, -1, -1}};
        bool inverted = false;
        int label = -1;
    };
    std::array<Ex, kNVerdict> ex;

    const double c_level = m_offset_potential ? m_offset_potential->target_level() : 0.;

    for (size_t fid = 0; fid < tri_capacity(); ++fid) {
        if (!tuple_from_tri(fid).is_valid(*this)) continue;
        const double q = m_face_attribute[fid].m_quality;
        if (!(q >= filter_energy)) continue;
        ++n_bad;

        const auto vs = oriented_tri_vids(fid);
        const bool inv = is_inverted(fid);
        if (inv) ++n_inverted;

        Ex cand;
        cand.q = q;
        cand.fid = fid;
        cand.inverted = inv;
        cand.label = m_face_extra[fid].label;
        cand.sizing = std::numeric_limits<double>::max();
        for (const size_t v : vs) {
            cand.c += m_vertex_attribute[v].m_posf / 3.;
            cand.sizing = std::min(cand.sizing, m_vertex_attribute[v].m_sizing_scalar);
        }
        if (m_input_complex_bvh) {
            const Vector3d foot = m_input_complex_bvh->nearest_point(cand.c);
            cand.dist = (cand.c - Vector2d(foot.x(), foot.y())).norm();
        }
        if (m_offset_potential && c_level > 0.) {
            // Several entries of offsets: the face's piece's field over its own level, each piece
            // at its own distance. As in 3D.
            const OffsetPotential2D& pot = potential_for_face(fid);
            cand.phi_over_c = pot.value(cand.c) /
                              (m_offset_params.offsets.size() > 1 ? pot.target_level() : c_level);
        }

        int best = kShort; // the LEAST blocked of the three edges; `free` is best of all
        for (int k = 0; k < 3; ++k) {
            const size_t a = vs[k], b = vs[(k + 1) % 3];
            const Vector2d& pa = m_vertex_attribute[a].m_posf;
            const Vector2d& pb = m_vertex_attribute[b].m_posf;
            const double len2 = (pb - pa).squaredNorm();
            const double sr = 0.5 * (m_vertex_attribute[a].m_sizing_scalar +
                                     m_vertex_attribute[b].m_sizing_scalar);
            const double gate2 = m_params.splitting_l2 * sr * sr;
            cand.len[k] = std::sqrt(len2);
            cand.gate[k] = std::sqrt(std::max(gate2, 0.));

            Verdict v;
            if (len2 < gate2) {
                v = kShort;
            } else if (vertex_valence(vs[(k + 2) % 3]) > val_thresh) {
                v = kValence;
            } else {
                // The child segments' envelope IS the parent's -- see the declaration.
                const auto [_, eid] = tuple_from_edge({{a, b}});
                v = kFree;
                if (m_edge_attribute[eid].m_is_surface_fs) {
                    const std::shared_ptr<Envelope> env =
                        surface_envelope_for_edge({{a, b}}, m_edge_attribute[eid]);
                    if (env) {
                        const Vector2d mid = 0.5 * (pa + pb);
                        if (env->is_outside(std::array<Vector2d, 2>{{pa, mid}}) ||
                            env->is_outside(std::array<Vector2d, 2>{{mid, pb}})) {
                            v = kContain;
                            if (edge_is_offset(eid))
                                ++n_contain_offset;
                            else
                                ++n_contain_region;
                        }
                    }
                }
            }
            cand.verd[k] = int(v);
            if (v == kFree)
                best = kFree;
            else if (best != kFree && v > best)
                best = v;
            ++edge_hist[v];
        }
        ++face_first[best];
        if (best == kFree) ++n_any_free;
        if (cand.q > ex[best].q) ex[best] = cand;
    }

    if (n_bad == 0) {
        logger().info("\t[refine-block {}] no element at or above {:.4g}", when, filter_energy);
        return;
    }

    std::string faces, edges;
    for (int v = 0; v < kNVerdict; ++v) {
        if (face_first[v])
            faces += fmt::format("{}{} {}", faces.empty() ? "" : ", ", face_first[v], kName[v]);
        if (edge_hist[v])
            edges += fmt::format("{}{} {}", edges.empty() ? "" : ", ", edge_hist[v], kName[v]);
    }
    logger().info(
        "\t[refine-block {}] {} elements >= {:.4g} ({} exactly inverted) | best edge per element: "
        "{} | all {} edges: {} | containment refusals by class: {} offset, {} region | "
        "target l {:.6g}, split needs length >= {:.4g} x mean sizing",
        when,
        n_bad,
        filter_energy,
        n_inverted,
        faces,
        3 * n_bad,
        edges,
        n_contain_offset,
        n_contain_region,
        l,
        std::sqrt(std::max(m_params.splitting_l2, 0.)));

    // n_any_free is the load-bearing number: elements no gate is blocking.
    logger().info(
        "\t[refine-block {}] {} of {} bad elements have at least one splittable edge -- for those "
        "the gates are NOT the obstacle",
        when,
        n_any_free,
        n_bad);

    for (int v = 0; v < kNVerdict; ++v) {
        const Ex& e = ex[v];
        if (e.q < 0.) continue;
        // INFO like the census headlines above: this is diagnostic detail, not a defect claim.
        logger().info(
            "\t  worst [{}]: f{} q {:.4g}{} label {} at ({:.6g}, {:.6g}) | dist to complex {:.6g} "
            "= {:.4g}x delta | Phi/c {:.6g} | min sizing {:.6g} = {:.4g}x l | edges "
            "len/gate {:.4g}/{:.4g} [{}], {:.4g}/{:.4g} [{}], {:.4g}/{:.4g} [{}]",
            kName[v],
            e.fid,
            e.q,
            e.inverted ? " INVERTED" : "",
            e.label,
            e.c.x(),
            e.c.y(),
            e.dist,
            e.dist / std::max(m_offset_params.target_distance, 1e-16),
            e.phi_over_c,
            e.sizing,
            e.sizing / l,
            e.len[0],
            e.gate[0],
            kName[e.verd[0]],
            e.len[1],
            e.gate[1],
            kName[e.verd[1]],
            e.len[2],
            e.gate[2],
            kName[e.verd[2]]);
    }
}

void TopoOffsetTriMesh::log_stuck_refine_census(const double max_metric, const double filter_energy)
{
    // See the declaration for what this is for. Diagnostic only -- it reads the mesh and writes
    // nothing but m_stuck_prev_cells and the log.
    ++m_stuck_calls;

    const double l = std::max(m_params.l, 1e-16);
    const double cell = l / 10.;

    size_t n_faces = 0, n_over_filter = 0, n_max = 0;
    size_t n_exact_inverted = 0, n_float_only = 0, n_unrounded = 0;
    std::array<size_t, 3> by_class{{0, 0, 0}}; // ambient / input complex / band
    size_t n_below_gate = 0, n_at_floor = 0;
    std::vector<double> areas, shortest, aspects;
    std::vector<size_t> max_fids;
    std::set<std::pair<long, long>> cells;

    for (size_t fid = 0; fid < tri_capacity(); ++fid) {
        if (!tuple_from_tri(fid).is_valid(*this)) continue;
        ++n_faces;
        const double q = m_face_attribute[fid].m_quality;
        if (q >= filter_energy) ++n_over_filter;
        if (q < MAX_ENERGY) continue;
        ++n_max;
        max_fids.push_back(fid);

        const auto vs = oriented_tri_vids(fid);

        // Why it scores MAX_ENERGY. is_inverted() is exact for the coordinates the vertices
        // actually carry; is_inverted_f() uses m_posf alone. A face inverted in float but not
        // exactly is a valid triangle whose double area underflowed -- refining it produces two
        // more of the same, which is the distinction this census exists for.
        const bool exact_bad = is_inverted(fid);
        const bool float_bad = is_inverted_f(fid);
        if (exact_bad)
            ++n_exact_inverted;
        else if (float_bad)
            ++n_float_only;
        bool any_unrounded = false;
        for (const size_t v : vs) any_unrounded |= !m_vertex_attribute[v].m_is_rounded;
        if (any_unrounded) ++n_unrounded;

        const int lab = m_face_extra[fid].label;
        by_class[lab >= 0 && lab <= 2 ? size_t(lab) : size_t(0)]++;

        const Vector2d& a = m_vertex_attribute[vs[0]].m_posf;
        const Vector2d& b = m_vertex_attribute[vs[1]].m_posf;
        const Vector2d& c = m_vertex_attribute[vs[2]].m_posf;
        areas.push_back(std::abs((b - a)[0] * (c - a)[1] - (b - a)[1] * (c - a)[0]) / 2.);
        const double e0 = (b - a).norm(), e1 = (c - b).norm(), e2 = (a - c).norm();
        const double lo = std::min({e0, e1, e2}), hi = std::max({e0, e1, e2});
        shortest.push_back(lo);
        aspects.push_back(lo > 0. ? hi / lo : std::numeric_limits<double>::infinity());

        // Can the next pass even split it? The base's gate is length^2 > (l * sbar)^2 * 16/9,
        // sbar the mean of the endpoints' scalars, so a face whose longest edge is already under
        // that is not a split candidate however far the sizing field is driven down.
        double sbar_hi = 0.;
        for (const size_t v : vs) sbar_hi += m_vertex_attribute[v].m_sizing_scalar;
        sbar_hi /= 3.;
        if (hi <= l * sbar_hi * 4. / 3.) ++n_below_gate;
        bool at_floor = true;
        for (const size_t v : vs)
            at_floor &= m_vertex_attribute[v].m_sizing_scalar <=
                        m_params.stuck_refine_min_scalar * (1. + 1e-9);
        if (at_floor) ++n_at_floor;

        const Vector2d ctr = (a + b + c) / 3.;
        cells.insert({long(std::floor(ctr[0] / cell)), long(std::floor(ctr[1] / cell))});
    }

    if (n_max == 0) {
        logger().info(
            "[stuck-census #{}] {} faces, {} at or over filter {:.4}, NONE at MAX_ENERGY -- the "
            "stall is merely-bad elements, not degenerate ones (max metric {:.4})",
            m_stuck_calls,
            n_faces,
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

    // Connected clusters among the MAX_ENERGY faces, by shared edge. Union-find over that set
    // only: the question being asked is how many clumps there are.
    std::unordered_map<size_t, size_t> idx_of;
    for (size_t i = 0; i < max_fids.size(); ++i) idx_of[max_fids[i]] = i;
    std::vector<size_t> parent(max_fids.size());
    std::iota(parent.begin(), parent.end(), size_t(0));
    std::function<size_t(size_t)> find = [&](size_t x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    };
    std::map<std::pair<size_t, size_t>, size_t> edge_owner;
    for (size_t i = 0; i < max_fids.size(); ++i) {
        const auto vs = oriented_tri_vids(max_fids[i]);
        for (int k = 0; k < 3; ++k) {
            size_t u = vs[k], w = vs[(k + 1) % 3];
            if (u > w) std::swap(u, w);
            auto it = edge_owner.find({u, w});
            if (it == edge_owner.end()) {
                edge_owner[{u, w}] = i;
            } else {
                const size_t ra = find(it->second), rb = find(i);
                if (ra != rb) parent[ra] = rb;
            }
        }
    }
    std::unordered_map<size_t, size_t> comp_size;
    for (size_t i = 0; i < max_fids.size(); ++i) comp_size[find(i)]++;
    size_t largest = 0;
    for (const auto& [root, sz] : comp_size) largest = std::max(largest, sz);

    size_t overlap = 0;
    for (const auto& c : cells)
        if (m_stuck_prev_cells.count(c)) ++overlap;
    const double overlap_pct =
        m_stuck_prev_cells.empty() ? 0. : 100. * double(overlap) / double(cells.size());

    logger().info(
        "[stuck-census #{}] {} faces | {} at/over filter {:.4} | {} at MAX_ENERGY ({:.2f}%)",
        m_stuck_calls,
        n_faces,
        n_over_filter,
        filter_energy,
        n_max,
        100. * double(n_max) / double(std::max<size_t>(n_faces, 1)));
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
        "largest {} faces | grid cells {} ({:.1f}% shared with the previous census)",
        m_stuck_calls,
        by_class[0],
        by_class[1],
        by_class[2],
        comp_size.size(),
        largest,
        cells.size(),
        overlap_pct);
    logger().info(
        "[stuck-census #{}]   geometry: area med {:.6g} (min {:.6g}), shortest edge med {:.6g}, "
        "aspect med {:.6g} | target l {:.6g}",
        m_stuck_calls,
        med(areas),
        areas.front(),
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
        "[stuck-census #{}]   created since the last census: by SPLIT {} needle faces (a split "
        "is never refused on quality)",
        m_stuck_calls,
        split_created - m_deg_prev_split_created);
    m_deg_prev_split_created = split_created;

    m_stuck_prev_cells = std::move(cells);
}


void TopoOffsetTriMesh::report_needle(const char* op, const size_t fid, const double parent_q) const
{
    // See the declaration. First kNeedleReports only; everything after that is the force-split
    // loop repeating itself.
    if (m_needle_reports.fetch_add(1) >= kNeedleReports) return;

    const auto vs = oriented_tri_vids(fid);
    const Vector2d& a = m_vertex_attribute[vs[0]].m_posf;
    const Vector2d& b = m_vertex_attribute[vs[1]].m_posf;
    const Vector2d& c = m_vertex_attribute[vs[2]].m_posf;
    const double area = ((b - a)[0] * (c - a)[1] - (b - a)[1] * (c - a)[0]) / 2.;
    const double e0 = (b - a).norm(), e1 = (c - b).norm(), e2 = (a - c).norm();

    std::string per_vertex;
    for (int k = 0; k < 3; ++k) {
        const size_t v = vs[k];
        const auto& x = m_vertex_extra[v];
        per_vertex += fmt::format(
            "\n\t    v{} id {} ({:.17g}, {:.17g}) input {} offset {} region {} mask 0x{:x} "
            "epoch {} rounded {} sizing {:.6g}",
            k,
            v,
            m_vertex_attribute[v].m_posf[0],
            m_vertex_attribute[v].m_posf[1],
            x.m_is_on_input,
            x.m_is_on_offset,
            x.m_is_on_region,
            x.m_boundary_mask,
            x.m_born_epoch,
            m_vertex_attribute[v].m_is_rounded,
            m_vertex_attribute[v].m_sizing_scalar);
    }
    // Per-event forensic detail at INFO: a warning is reserved for a defect that exists when it is
    // reported. These lines only narrate births; the needle-scan population sweeps do the warning.
    logger().info(
        "[needle #{}] created at {} | fid {} label {} | area {:.6g} | edges {:.6g} {:.6g} {:.6g} "
        "| parent AMIPS {} | is_inverted {} is_inverted_f {} | epoch {}{}",
        m_needle_reports.load(),
        op,
        fid,
        m_face_extra[fid].label,
        area,
        e0,
        e1,
        e2,
        parent_q < 0. ? std::string("n/a") : fmt::format("{:.6g}", parent_q),
        is_inverted(fid),
        is_inverted_f(fid),
        m_op_epoch,
        per_vertex);
}


void TopoOffsetTriMesh::needle_scan(const char* when) const
{
    size_t n = 0;
    double worst_area = std::numeric_limits<double>::max();
    size_t worst_fid = 0;
    std::array<size_t, 3> by_class{{0, 0, 0}};
    for (size_t fid = 0; fid < tri_capacity(); ++fid) {
        if (!tuple_from_tri(fid).is_valid(*this)) continue;
        if (get_quality(fid) < kNeedleQuality) continue;
        ++n;
        const int lab = m_face_extra[fid].label;
        by_class[lab >= 0 && lab <= 2 ? size_t(lab) : size_t(0)]++;
        const auto vs = oriented_tri_vids(fid);
        const Vector2d& a = m_vertex_attribute[vs[0]].m_posf;
        const Vector2d& b = m_vertex_attribute[vs[1]].m_posf;
        const Vector2d& c = m_vertex_attribute[vs[2]].m_posf;
        const double area = std::abs((b - a)[0] * (c - a)[1] - (b - a)[1] * (c - a)[0]) / 2.;
        if (area < worst_area) {
            worst_area = area;
            worst_fid = fid;
        }
    }
    if (n == 0) {
        logger().info("[needle-scan] {}: NONE", when);
        return;
    }
    logger().warn(
        "[needle-scan] {}: {} faces over AMIPS {:g} (ambient {}, input complex {}, band {}), "
        "smallest area {:.6g} at fid {}",
        when,
        n,
        kNeedleQuality,
        by_class[0],
        by_class[1],
        by_class[2],
        worst_area,
        worst_fid);
    needle_forensics();
    logger().info(
        "[needle-smooth] cumulative: {} visits with a needle in the ring | {} produced a "
        "candidate | {} actually repaired it | {} did not move the vertex at all",
        m_needle_smooth_offered.load(),
        m_needle_smooth_reached.load(),
        m_needle_smooth_fixed.load(),
        m_needle_smooth_stationary.load());
    report_needle("scan", worst_fid, -1.);
}


double TopoOffsetTriMesh::ring_max_quality(const size_t vid) const
{
    double m = -1.;
    for (const size_t fid : get_one_ring_fids_for_vertex(tuple_from_vertex(vid))) {
        m = std::max(m, get_quality(fid));
    }
    return m;
}


double TopoOffsetTriMesh::face_flatness(const size_t fid) const
{
    const auto vs = oriented_tri_vids(fid);
    const Vector2d& a = m_vertex_attribute[vs[0]].m_posf;
    const Vector2d& b = m_vertex_attribute[vs[1]].m_posf;
    const Vector2d& c = m_vertex_attribute[vs[2]].m_posf;
    const double twice_area = std::abs((b - a)[0] * (c - a)[1] - (b - a)[1] * (c - a)[0]);
    const double lmax = std::max({(b - a).norm(), (c - b).norm(), (a - c).norm()});
    return lmax > 0. ? twice_area / (lmax * lmax) : 0.;
}


void TopoOffsetTriMesh::record_flatness(
    const char* op,
    const double parent_flat,
    const size_t child_fid) const
{
    const double child = face_flatness(child_fid);
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
    // Only the creations are logged: a flat child of a flat parent is understood multiplication,
    // a flat child of a healthy parent is the event being hunted.
    if (from_healthy && m_flat_genesis_reports.fetch_add(1) < 10) {
        const auto vs = oriented_tri_vids(child_fid);
        std::string vtx;
        for (int k = 0; k < 3; ++k) {
            const auto& x = m_vertex_extra[vs[k]];
            vtx += fmt::format(
                "\n\t    v{} id {} ({:.17g}, {:.17g}) epoch {} input {} region {} mask 0x{:x}",
                k,
                vs[k],
                m_vertex_attribute[vs[k]].m_posf[0],
                m_vertex_attribute[vs[k]].m_posf[1],
                x.m_born_epoch,
                x.m_is_on_input,
                x.m_is_on_region,
                x.m_boundary_mask);
        }
        // Per-event forensic detail at INFO -- see report_needle(); the flat-population sweep is
        // the warning when flat faces exist now.
        logger().info(
            "[genesis #{}] {} turned a HEALTHY face into a flat one: flatness {:.6g} -> {:.6g} "
            "(threshold {:g}) | fid {} label {} | AMIPS {:.6g}{}",
            m_flat_genesis_reports.load(),
            op,
            parent_flat,
            child,
            kFlatThreshold,
            child_fid,
            m_face_extra[child_fid].label,
            get_quality(child_fid),
            vtx);
    }
}


void TopoOffsetTriMesh::needle_forensics() const
{
    const double l = std::max(m_params.l, 1e-16);
    const double coll_c = std::sqrt(std::max(m_params.collapsing_l2, 0.)); // = 4/5 l
    const double split_c = std::sqrt(std::max(m_params.splitting_l2, 0.)); // = 4/3 l

    // ---- the flattest faces, and every gate on every one of their edges ----
    std::vector<std::pair<double, size_t>> flat;
    for (size_t fid = 0; fid < tri_capacity(); ++fid) {
        if (!tuple_from_tri(fid).is_valid(*this)) continue;
        const double f = face_flatness(fid);
        if (f < kFlatThreshold) flat.emplace_back(f, fid);
    }
    std::sort(flat.begin(), flat.end());
    if (flat.empty()) {
        logger().info("[forensics] 0 faces flatter than {:g}", kFlatThreshold);
    }
    // A nonempty population is a defect that exists NOW: warn. Empty is the healthy report.
    if (!flat.empty())
        logger().warn(
            "[forensics] {} faces flatter than {:g} | gates: collapse 4/5*l = {:.6g}, split 4/3*l "
            "= "
            "{:.6g}, both scaled by the edge's mean sizing scalar",
            flat.size(),
            kFlatThreshold,
            coll_c,
            split_c);

    const size_t show = std::min<size_t>(flat.size(), 4);
    for (size_t i = 0; i < show; ++i) {
        const size_t fid = flat[i].second;
        const auto vs = oriented_tri_vids(fid);
        logger().warn(
            "[forensics] face {} flatness {:.4g} AMIPS {:.6g} label {}",
            fid,
            flat[i].first,
            get_quality(fid),
            m_face_extra[fid].label);
        for (int k = 0; k < 3; ++k) {
            const size_t u = vs[k], w = vs[(k + 1) % 3];
            const double len = (m_vertex_attribute[u].m_posf - m_vertex_attribute[w].m_posf).norm();
            const double sbar =
                (m_vertex_attribute[u].m_sizing_scalar + m_vertex_attribute[w].m_sizing_scalar) /
                2.;
            const auto got = try_tuple_from_edge({{u, w}});
            std::string swap_info = "edge not found";
            if (got) {
                const Tuple& et = std::get<0>(*got);
                const bool surf = is_edge_on_surface(et);
                swap_info = fmt::format(
                    "on_surface {} (swap {})",
                    surf,
                    surf ? "REFUSED outright" : "allowed");
            }
            logger().warn(
                "[forensics]   edge {}-{} len {:.6g} | collapse gate {:.6g} -> {} | split gate "
                "{:.6g} -> {} | force-split queued {} | {}",
                u,
                w,
                len,
                coll_c * sbar,
                len <= coll_c * sbar ? "offered" : "NEVER OFFERED (too long)",
                split_c * sbar,
                len > split_c * sbar ? "SPLIT CANDIDATE" : "too short",
                is_force_split_edge(u, w),
                swap_info);
        }
    }

    // ---- coincident vertices ----
    const double eps = 1e-9 * l;
    std::unordered_map<long long, std::vector<size_t>> cells;
    const auto key = [&](const Vector2d& p) {
        return (long long)(std::llround(p[0] / (eps * 10.))) * 1000003LL +
               (long long)(std::llround(p[1] / (eps * 10.)));
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
                const bool shares = try_tuple_from_edge({{group[i], group[j]}}).has_value();
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
    // Coincident pairs are a defect that exists NOW: warn. None is the healthy report.
    if (n_pairs > 0)
        logger().warn(
            "[forensics] coincident vertices (closer than {:.3g}): {} pairs, {} of them NOT joined "
            "by an edge (no collapse can reach those). {}",
            eps,
            n_pairs,
            n_pairs_no_edge,
            first.empty() ? "none" : first);
    logger().info(
        "[forensics] genesis tally: flat faces made from a HEALTHY parent -- split {}, collapse "
        "{} | flat-from-flat (multiplication) {}",
        m_flat_created_split.load(),
        m_flat_created_collapse.load(),
        m_flat_worsened_split.load());
}

bool TopoOffsetTriMesh::face_is_offset_band(const size_t fid) const
{
    return m_face_extra[fid].label == 2;
}


std::vector<bool> TopoOffsetTriMesh::band_vertex_mask() const
{
    // Read from the TAGS, not from m_vertex_extra[].m_is_on_offset: the flag is refreshed by
    // label_offset_boundary() only once per iteration, while the shared operations maintain the
    // tags continuously. Only the OUTER surface of the band, the one that is supposed to sit at
    // target_distance -- the inner interface where the band wraps the complex is by construction
    // at distance 0, so including it makes the max error identically target_distance on every
    // input and the test can never pass. 3D draws the same line.
    std::vector<bool> on_band(vert_capacity(), false);
    for (const Tuple& e : get_edges()) {
        const std::optional<Tuple> opp = e.switch_face(*this);
        if (!opp) {
            // A band edge on the domain boundary is still the band's outer surface, and its
            // vertices are still supposed to sit at target_distance. The rule below compares two
            // incident faces and there is only one here, but the missing side is outside the
            // domain, which is trivially neither band nor input complex.
            //
            // Skipping these let a clipped offset report a healthy error: exactly the clipped
            // vertices -- the frozen ones nothing can fix -- were the ones dropped from the
            // measurement, and a wholly clipped band left no measured edges at all.
            if (face_is_offset_band(e.fid(*this))) {
                on_band[e.vid(*this)] = true;
                on_band[e.switch_vertex(*this).vid(*this)] = true;
            }
            continue;
        }
        const size_t fa = e.fid(*this), fb = opp->fid(*this);
        const bool a = face_is_offset_band(fa), b = face_is_offset_band(fb);
        if (a == b) continue; // both inside the band or both outside it
        // the face across the interface must be the plain background mesh, not the complex
        if (face_is_input_complex(a ? fb : fa)) continue;
        on_band[e.vid(*this)] = true;
        on_band[e.switch_vertex(*this).vid(*this)] = true;
    }
    return on_band;
}

double TopoOffsetTriMesh::band_vertex_distance_error(const size_t vid) const
{
    // dist() pads a 2D point to 3D itself, and in 2D the complex's triangles lie in the
    // z = 0 plane, so a point inside the input complex reports 0 -- the same convention the
    // 3D version gets from its inside-any-tet check. Several entries of offsets: at the vertex's
    // piece's target distance, from its own piece -- its integral's distance (band_exact_for(),
    // 3D's envelope_for(): 2D builds no per-piece envelope). As in 3D.
    const Vector2d p = m_vertex_attribute[vid].m_posf;
    if (m_offset_params.offsets.size() <= 1 || !m_band_exact)
        return std::abs(m_input_complex_bvh->dist(VectorXd(p)) - m_offset_params.target_distance);
    const int r = place_piece(&vid, 1);
    return std::abs(band_exact_for(r)->distance(p) - region_delta(r));
}

double TopoOffsetTriMesh::band_vertex_residual(const size_t vid) const
{
    // How far this vertex is from the level set Phi = c, as a LENGTH: the offset's own error, as
    // opposed to band_vertex_distance_error()'s Euclidean diagnostic.
    return potential_for(vid).residual_length(m_vertex_attribute[vid].m_posf);
}

TopoOffsetTriMesh::EdgeSamples TopoOffsetTriMesh::offset_edge_samples(const Tuple& e) const
{
    EdgeSamples s;
    if (m_offset_params.stencil_order < 0) return s;
    const size_t va = e.vid(*this), vb = e.switch_vertex(*this).vid(*this);
    if (!band_vertex_is_reachable(va) || !band_vertex_is_reachable(vb)) return s;
    const OffsetPotential2D& pot = potential_for_edge(va, vb);
    for_each_offset_edge_sample(e, [&](const Vector2d& q, double, double) {
        const double r = pot.residual_length(q);
        s.max = std::max(s.max, r);
        s.sum += r;
        ++s.n;
    });
    return s;
}

TopoOffsetTriMesh::DistanceSplit TopoOffsetTriMesh::residual_split() const
{
    // The band's Phi residual. Every front vertex and every edge sample counts toward the driving
    // max, pinned ones (on the domain wall) included: a pinned vertex far from the level set is a
    // real error in the offset the run returns. The reachable/pinned split is kept as attribution
    // -- when the max comes from a pinned vertex the remedy is construction (domain size), not
    // more optimization. Same as 3D.
    const std::vector<bool> on_band = band_vertex_mask();

    DistanceSplit s;
    double sum_reachable = 0.;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid]) continue;
        const Vector2d p = m_vertex_attribute[vid].m_posf;
        const double err = potential_for(vid).residual_length(p);
        s.max_reachable = std::max(s.max_reachable, err);
        s.max_at_vertex = std::max(s.max_at_vertex, err);
        sum_reachable += err;
        ++s.n_reachable;
        if (band_vertex_is_reachable(vid)) {
            // The runaway guard's measurement, taken here rather than in its own traversal: this
            // loop already visits exactly the vertices it cares about, and Phi is the expensive
            // part. report_outside_support() turns this into the error.
            if (!potential_for(vid).within_support(p)) {
                ++s.n_outside_support;
                const double d = m_input_complex_bvh->dist(VectorXd(p));
                if (d > s.worst_outside_dist) {
                    s.worst_outside_dist = d;
                    s.worst_outside_vid = vid;
                }
            }
        } else {
            // Attribution only: the vertex already counted toward the max and the average
            // above; this records that the count includes n_pinned vertices nothing can move.
            s.max_pinned = std::max(s.max_pinned, err);
            ++s.n_pinned;
        }
    }
    // ... and the same measurement ALONG the band, which is what stops a boundary whose
    // vertices sit on the level set but whose edges cut across it from reading as converged.
    for (const Tuple& e : get_edges()) {
        if (!edge_is_offset_surface_live(e)) continue;
        const EdgeSamples es = offset_edge_samples(e);
        if (es.n == 0) continue;
        s.max_reachable = std::max(s.max_reachable, es.max);
        s.max_in_edge = std::max(s.max_in_edge, es.max);
        sum_reachable += es.sum;
        s.n_reachable += es.n;
    }

    s.avg_reachable = (s.n_reachable > 0) ? sum_reachable / s.n_reachable : 0.;
    return s;
}

TopoOffsetTriMesh::GradientSplit TopoOffsetTriMesh::gradient_split(
    const bool include_edge_samples) const
{
    // ||grad (Phi(x) - c)^2|| at every band vertex, on the field the vertex is placed on, plus
    // the edge-interior half on the same lattice the residual is sampled on. A diagnostic: the
    // loop exits on energy_criterion(). Weight 1 deliberately: an absolute bound in length units,
    // which a tuning weight would scale. Same as 3D.
    const std::vector<bool> on_band = band_vertex_mask();
    // One energy per region field, plus the union for a vertex with no region: a vertex is
    // measured against the field it is placed on. See potential_for().
    std::vector<std::unique_ptr<OffsetEnergy2D>> energies;
    for (const auto& rp : m_region_potentials)
        energies.push_back(std::make_unique<OffsetEnergy2D>(rp, 1.0, true, true));
    OffsetEnergy2D union_energy(m_offset_potential, 1.0, true, true);
    const auto energy_for = [&](const int region) -> OffsetEnergy2D& {
        return (region >= 0 && size_t(region) < energies.size()) ? *energies[size_t(region)]
                                                                 : union_energy;
    };

    GradientSplit s;
    double sum_reachable = 0.;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid]) continue;
        if (!m_vertex_extra[vid].m_is_on_offset) continue;

        // Skipped, not measured: a vertex the smoother declines to place this pass has a gradient
        // that is not part of the fixed point. Counted so a run cannot report convergence over a
        // band it never fully measured.
        if (!m_vertex_attribute[vid].m_is_rounded) {
            ++s.n_skipped_unrounded;
            continue;
        }
        const std::vector<size_t>& locs = get_one_ring_fids_for_vertex(vid);
        if (locs.empty()) continue;
        bool inverted = false;
        for (const size_t fid : locs) {
            if (is_inverted_f(fid)) {
                inverted = true;
                break;
            }
        }
        if (inverted) {
            ++s.n_skipped_inverted;
            continue;
        }

        Eigen::VectorXd g(2);
        const Eigen::VectorXd x = m_vertex_attribute[vid].m_posf;
        energy_for(vertex_region(vid)).gradient(x, g);
        const double gn = g.norm();

        // Pinned vertices are reported, not gated -- the one place this criterion deliberately
        // parts company with residual_split(). A residual is a statement about the BOUNDARY; a
        // gradient is a statement about the ITERATION, and folding in a vertex no sweep can move
        // would make convergence unreachable by construction.
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

    // ... and along the band's edges, which is the OTHER HALF of the criterion, not a diagnostic
    // beside it. A sample is not a variable, so no placement reduces it -- but refinement does,
    // and excluding the samples let a run declare convergence with a visibly polygonal front.
    //
    // Two things make the comparison mean anything: the same quantity as at the vertices (the full
    // norm, not the smaller normal projection, which would face a bar calibrated for something
    // else), and reachability -- only edges with BOTH ends reachable may gate, since a chord to a
    // pinned vertex inherits an error no refinement fixes. The rest are counted, never gate.
    if (include_edge_samples) {
        for (const Tuple& e : get_edges()) {
            if (!edge_is_offset_surface_live(e)) continue;
            const size_t va = e.vid(*this), vb = e.switch_vertex(*this).vid(*this);
            const bool gating = band_vertex_is_reachable(va) && band_vertex_is_reachable(vb);
            for_each_offset_edge_sample(e, [&](const Vector2d& q, double, double) {
                ++s.n_edge_samples;
                if (!gating) return;
                Eigen::VectorXd g(2);
                energy_for(edge_region(va, vb)).gradient(Eigen::VectorXd(q), g);
                s.max_in_edge = std::max(s.max_in_edge, g.norm());
            });
        }
    }

    s.avg_reachable = (s.n_reachable > 0) ? sum_reachable / s.n_reachable : 0.;
    return s;
}


TopoOffsetTriMesh::EnergyCriterion TopoOffsetTriMesh::energy_criterion()
{
    EnergyCriterion s;
    // THE bar, as a length: a chord is resolved when its RMS relative error is within it, and a
    // vertex placed when its own relative error is. See offset_envelope_rel for the leash on the
    // operations, which is not an accuracy and which startup requires to be no wider than this.
    s.tube = m_offset_params.front_conv;
    const auto front = [&](const size_t vid) {
        return m_vertex_extra[vid].m_is_on_offset && m_vertex_attribute[vid].m_is_rounded;
    };
    std::vector<char> placed(vert_capacity(), 0);
    // front_measure "vertex_ring": the ring measure is accumulated per end from the chord loop's
    // own edge_offset_term() calls below, so it judges exactly the chord numbers the face mode
    // does, every chord weighted equally. See EnergyCriterion::ring_exit.
    s.ring_exit = m_offset_params.front_measure == "vertex_ring";
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
        // gn is the vertex's convergence measure over the one bar; rho the length
        // residual_length(), its actual distance to the level set. rho is reported and gates
        // measurability, NOT placement: front_placed_by_ratio() is the one notion, and it reads gn.
        const Vector2d p = m_vertex_attribute[vid].m_posf;
        const double rho = potential_for(vid).residual_length(p);
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
    // The resolution test is per CHORD, sampled over its stencil, ends included (the 3D twin
    // samples each face over its stencil).
    for (const auto& e : offset_surface_edges()) {
        const size_t va = e[0], vb = e[1];
        if (!front(va) || !front(vb)) continue;
        // ONE call feeds both jobs: this number is the loop's exit test (max_edge / edges_ok(),
        // see converged()) AND what decides which chords the refinement is handed. Under
        // front_measure "vertex_ring" it feeds both through the ring measure instead. gn is the
        // root of the chord term, the figure the logs print.
        const double term = edge_offset_term(va, vb);
        const double gn = term < 0. ? -1. : std::sqrt(term);
        if (gn < 0.) {
            ++s.n_unmeasurable;
            if (s.ring_exit) ring_bad[va] = ring_bad[vb] = 1;
            continue;
        }
        const Vector2d& pa = m_vertex_attribute[va].m_posf;
        const Vector2d& pb = m_vertex_attribute[vb].m_posf;
        const Vector2d mid = 0.5 * (pa + pb);
        const double len = (pa - pb).norm();
        if (s.ring_exit) {
            for (const size_t u : {va, vb}) {
                ring_sum[u] += term;
                ++ring_n[u];
            }
        }
        ++s.n_edges;
        s.sum_edge += gn;
        if (gn > s.max_edge) {
            s.max_edge = gn;
        }
        if (gn > s.bar) {
            ++s.n_edges_over;
            const bool ends_placed = placed[va] && placed[vb];
            if (ends_placed) ++s.n_edges_over_placed;
            // Every chord over the bar is refinable, placed or not: under one unified measure
            // the ends can be held off the level set BY the error of the very chords a placement
            // gate would refuse to refine. The placed subset is for reporting only. Under the
            // ring measure no chord is handed to the refinement: the vertices are, after this
            // loop. As in 3D.
            if (!s.ring_exit) {
                // Refinable only if the rule can still lower a target; judged against the MAX of
                // the two scalars.
                const double l = std::max(m_params.l, 1e-300);
                const double s_floor = std::max(
                    m_offset_params.min_sizing_scalar,
                    m_offset_params.min_edge_length / l);
                const double have = std::max(
                    m_vertex_attribute[va].m_sizing_scalar,
                    m_vertex_attribute[vb].m_sizing_scalar);
                // How short this chord would have to become, as a sizing scalar: the chord rule
                // inverts the sagitta's power law to a length. Refinement itself is the halving
                // in refine_front_by_halving(), but the question asked here is the same either
                // way -- is there any target left below what the ends already carry, or are they
                // at the floor.
                const double target = front_chord_target(va, vb, len, gn * s.tube, s.tube);
                const double sn =
                    std::clamp(target / l, s_floor, m_offset_params.max_sizing_scalar);
                if (sn < have) {
                    s.refinable.push_back({va, vb, gn * s.tube, len});
                } else {
                    // Not handed to the refinement, for one of two reasons: either the ends are
                    // at the sizing floor (have <= s_floor) and nothing can refine the chord, or
                    // the chord target, at most half the chord, is not below have x l, so the
                    // chord is at least twice the larger target length at its ends and the split
                    // pass's length gate takes it.
                    ++s.n_at_floor;
                    s.floor_scalar = s_floor;
                    s.floor_from_min_edge_length =
                        m_offset_params.min_edge_length / l > m_offset_params.min_sizing_scalar;
                    if (gn > s.max_edge_at_floor) {
                        s.max_edge_at_floor = gn;
                        s.worst_at_floor_mid = mid;
                        s.worst_at_floor_scalar = have;
                    }
                    if (have <= s_floor) {
                        ++s.n_corners_at_floor;
                        if (gn > s.max_edge_corners_at_floor) {
                            s.max_edge_corners_at_floor = gn;
                            s.worst_corners_at_floor_mid = mid;
                        }
                    }
                }
                if (ends_placed && gn > s.max_edge_placed) {
                    s.max_edge_placed = gn;
                    s.worst_placed_mid = mid;
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
            if (ring_n[vid] == 0) continue; // no measured front chord: no ring to judge
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

std::string TopoOffsetTriMesh::EnergyCriterion::sizing_floor_fact() const
{
    if (ring_exit) {
        if (n_rings_at_floor == 0) return "";
        const char* origin =
            floor_from_min_edge_length ? "min_edge_length / l" : "min_sizing_scalar";
        return fmt::format(
            "{} front vertex(es) with the {} over the bar have their sizing scalar at the "
            "sizing floor {:.4g} (from {}), so they cannot be refined and the loop cannot "
            "converge on them: worst {:.4}x the bar at ({:.4}, {:.4})",
            n_rings_at_floor,
            ring_name(),
            floor_scalar,
            origin,
            max_ring_at_floor,
            worst_ring_at_floor_pos.x(),
            worst_ring_at_floor_pos.y());
    }
    if (n_at_floor == 0) return "";
    const char* origin = floor_from_min_edge_length ? "min_edge_length / l" : "min_sizing_scalar";
    if (n_corners_at_floor > 0) {
        return fmt::format(
            "{} front chord(s) over the bar have both ends at the sizing floor {:.4g} (from {}), "
            "so they cannot be refined and the loop cannot converge on them: worst {:.4}x the "
            "bar at midpoint ({:.4}, {:.4}). Of the {} chord(s) over the bar with no chord target "
            "below the larger sizing scalar at their ends, the other {} are at least twice that "
            "target length, which the split pass shortens",
            n_corners_at_floor,
            floor_scalar,
            origin,
            max_edge_corners_at_floor,
            worst_corners_at_floor_mid.x(),
            worst_corners_at_floor_mid.y(),
            n_at_floor,
            n_at_floor - n_corners_at_floor);
    }
    return fmt::format(
        "{} front chord(s) over the bar have no chord target below the larger sizing scalar at "
        "their ends, and none has its ends at the sizing floor {:.4g} (from {}): each is at "
        "least twice the larger target length at its ends, which the split pass shortens, so no "
        "refinement is needed; worst {:.4}x the bar at midpoint ({:.4}, {:.4}), end scalar {:.4g}",
        n_at_floor,
        floor_scalar,
        origin,
        max_edge_at_floor,
        worst_at_floor_mid.x(),
        worst_at_floor_mid.y(),
        worst_at_floor_scalar);
}

double TopoOffsetTriMesh::front_gradient_linf()
{
    double worst = 0.;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!m_vertex_extra[vid].m_is_on_offset || !m_vertex_attribute[vid].m_is_rounded) continue;
        // The objective's normal gradient, always: the vertex's convergence ratio is its relative
        // error, not a stationarity measure, so it would not be the "gradient reference" the log
        // calls this. As in 3D.
        worst = std::max(worst, front_vertex_normal_gradient(vid));
    }
    return worst;
}

double TopoOffsetTriMesh::front_vertex_conv_ratio(const size_t vid) const
{
    // THE measure at one point, against THE bar: the vertex's own relative error
    // |relative_residual(x)| over front_conv_frac(), i.e. its distance to the level set along the
    // field over front_conv (see edge_offset_term()). This is the chord term's order-0 stencil
    // evaluated at a single end, which is what makes the vertex test and the chord test one test
    // rather than two. As in 3D.
    const OffsetPotential2D& pot = potential_for(vid);
    const double level = pot.target_level();
    if (!(level > 0.)) return std::numeric_limits<double>::infinity();
    const double r = pot.relative_residual(m_vertex_attribute[vid].m_posf);
    if (!std::isfinite(r)) return std::numeric_limits<double>::infinity();
    const double bar = m_offset_params.offsets.size() > 1 ? front_conv_frac_of(place_piece(&vid, 1))
                                                          : m_offset_params.front_conv_frac();
    if (!(bar > 0.)) return std::numeric_limits<double>::infinity();
    return std::abs(r) / bar;
}

double TopoOffsetTriMesh::edge_offset_term(
    const OffsetPotential2D& pot,
    const Vector2d& pa,
    const Vector2d& pb,
    const int piece) const
{
    // THE chord measure: the MEAN over the chord's stencil of the squared relative error, in
    // units of the tolerance. At every stencil point q of for_each_edge_sample():
    //
    //     r(q) = pot.relative_residual(q) / front_conv_frac()
    //          = (signed distance from q to the level set, along the field) / front_conv
    //
    // and the chord's term is mean of r^2, so 1 exactly at the bar; its root is the "Nx the bar"
    // figure the logs print. One arithmetic for every reader: the exit and refinement
    // (energy_criterion()) and the debug frames; not the energy (tri_energy()) nor the smoother's
    // objective, as in 3D. The
    // stencil contains the chord's ENDS, where a distance to the level set is that vertex's own
    // placement error, so this one number covers both the vertex and the chord. The 3D twin is
    // face_offset_term(); see there for why the field is asked rather than (Phi(q) - c)/c formed
    // here.
    const double level = pot.target_level();
    if (!(level > 0.)) return -1.;
    double sum = 0.;
    size_t n = 0;
    bool unmeasurable = false;
    for_each_edge_sample(pa, pb, [&](const Vector2d& q, double, double) {
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
    // reads the whole chord unmeasurable.
    if (unmeasurable || n == 0) return -1.;
    if (!(front_conv_frac_of(piece) > 0.)) return std::numeric_limits<double>::infinity();
    return offset_term_weight(piece) * (sum / double(n));
}

double TopoOffsetTriMesh::edge_offset_term(const size_t a, const size_t b) const
{
    // Several entries of offsets: the chord's piece's bar (one entry: the run's). As in 3D.
    const std::array<size_t, 2> e{{a, b}};
    const int piece = m_offset_params.offsets.size() > 1 ? place_piece(e.data(), 2) : -1;
    return edge_offset_term(
        potential_for_edge(a, b),
        m_vertex_attribute[a].m_posf,
        m_vertex_attribute[b].m_posf,
        piece);
}

double TopoOffsetTriMesh::tri_energy(const size_t fid, const bool band_term) const
{
    // See the declaration. get_quality() is the base's: AMIPS2D or MAX_ENERGY.
    return cell_energy(
        oriented_tri_vids(fid),
        get_quality(fid),
        band_term && face_is_offset_band(fid));
}

double TopoOffsetTriMesh::tri_energy_sum(const std::vector<size_t>& fids, const bool band_term)
    const
{
    double s = 0.;
    for (const size_t fid : fids) s += tri_energy(fid, band_term);
    return s;
}

double TopoOffsetTriMesh::band_swept_change(
    const size_t vid,
    const Vector2d& x0,
    const Vector2d& x1,
    ExactBandIntegral2D::Stats* st) const
{
    // See EXACT SWEPT CHANGE.
    if (x0 == x1) return 0.;
    std::vector<SweptEdgesOfPiece> pieces;
    band_swept_edges(vid, x0, pieces);
    return band_swept_sum(x0, x1, pieces, st);
}

void TopoOffsetTriMesh::band_swept_edges(
    const size_t vid,
    const Vector2d& x0,
    std::vector<SweptEdgesOfPiece>& pieces) const
{
    // Each boundary edge of B_r through vid belongs to one face of B_r, the face across it not
    // one (EXACT SWEPT CHANGE, several entries). As 3D's band_swept_faces().
    using wmtk::utils::predicates::orient2d;
    using wmtk::utils::predicates::Orientation;
    pieces.clear();
    for (const size_t fid : get_one_ring_fids_for_vertex(vid)) {
        if (!face_is_offset_band(fid)) continue;
        const std::array<size_t, 3> vs = oriented_tri_vids(fid);
        int k = 0;
        while (k < 3 && vs[size_t(k)] != vid) ++k;
        if (k == 3) continue;
        const int r = place_piece(vs.data(), 3);
        for (int j = 1; j <= 2; ++j) { // the edge (vid, a), p the face's third corner
            const size_t a = vs[size_t((k + j) % 3)], p = vs[size_t((k + 3 - j) % 3)];
            bool inside = false;
            for (const size_t o : get_incident_fids_for_edge(vid, a)) {
                if (o == fid || !face_is_offset_band(o)) continue;
                if (place_piece(oriented_tri_vids(o).data(), 3) == r) inside = true; // inside B_r
            }
            if (inside) continue;
            // the pair ordered with its normal rot_cw(second - first) out of B_r, away from the
            // face's third corner p: p on the pair's left, orient2d positive
            const Orientation side =
                orient2d(x0, m_vertex_attribute[a].m_posf, m_vertex_attribute[p].m_posf);
            if (side == Orientation::COLLINEAR) continue; // a flat face at x0 bounds nothing
            auto it = std::find_if(pieces.begin(), pieces.end(), [&](const SweptEdgesOfPiece& q) {
                return q.piece == r;
            });
            if (it == pieces.end()) it = pieces.insert(pieces.end(), SweptEdgesOfPiece{r, {}});
            it->edges.push_back({m_vertex_attribute[a].m_posf, side == Orientation::POSITIVE});
        }
    }
}

double TopoOffsetTriMesh::band_swept_sum(
    const Vector2d& x0,
    const Vector2d& x1,
    const std::vector<SweptEdgesOfPiece>& pieces,
    ExactBandIntegral2D::Stats* st) const
{
    // the function the smoother's search model evaluates too (BandVolumeEnergy2D::set_swept()),
    // per piece
    double s = 0.;
    for (const SweptEdgesOfPiece& q : pieces) {
        s += BandVolumeEnergy2D::swept_change(
            *band_exact_for(q.piece),
            region_delta(q.piece),
            x0,
            x1,
            q.edges,
            st);
    }
    return s;
}

std::vector<std::pair<size_t, int>> TopoOffsetTriMesh::band_face_pieces(
    const std::vector<size_t>& fids) const
{
    std::vector<std::pair<size_t, int>> out;
    for (const size_t fid : fids) {
        if (face_is_offset_band(fid))
            out.push_back({fid, place_piece(oriented_tri_vids(fid).data(), 3)});
    }
    std::sort(out.begin(), out.end());
    return out;
}

double TopoOffsetTriMesh::band_relabel_change(
    const std::vector<size_t>& fids,
    const std::vector<std::pair<size_t, int>>& before) const
{
    // See the declaration and EXACT SWEPT CHANGE, several entries.
    if (m_offset_params.offsets.size() <= 1) return 0.;
    double s = 0.;
    for (const size_t fid : fids) {
        if (!face_is_offset_band(fid)) continue;
        const auto it = std::lower_bound(
            before.begin(),
            before.end(),
            std::pair<size_t, int>{fid, std::numeric_limits<int>::min()});
        if (it == before.end() || it->first != fid) continue;
        const std::array<size_t, 3> vs = oriented_tri_vids(fid);
        const int r = place_piece(vs.data(), 3);
        if (r != it->second) s += band_cell_term(vs, r) - band_cell_term(vs, it->second);
    }
    return s;
}

double TopoOffsetTriMesh::swap_swept_change(const std::vector<size_t>& made) const
{
    // See the declaration.
    const auto& old = m_swap_old_band.local();
    std::vector<std::pair<std::array<size_t, 3>, int>> nb;
    for (const size_t fid : made) {
        if (!face_is_offset_band(fid)) continue;
        const std::array<size_t, 3> vs = oriented_tri_vids(fid);
        nb.push_back({vs, place_piece(vs.data(), 3)});
    }
    const int r = !old.empty() ? old.front().second : nb.empty() ? -1 : nb.front().second;
    bool one = true;
    for (const auto& t : old) one = one && t.second == r;
    for (const auto& t : nb) one = one && t.second == r;
    if (one) return 0.;
    double s = 0.;
    for (const auto& t : nb) s += band_cell_term(t.first, t.second);
    for (const auto& t : old) s -= band_cell_term(t.first, t.second);
    return s;
}

bool TopoOffsetTriMesh::swept_accept(
    const size_t vid,
    const std::vector<size_t>& ring,
    const Vector2d& x0,
    const double amips_before,
    SweptCounters& c)
{
    // See the declaration.
    ++c.solves;
    const Vector2d x1 = m_vertex_attribute[vid].m_posf;
    // band_swept_change()'s edges, the same at every halving (band_swept_edges() reads vid's
    // position only for the pieces), and the ring's pieces, read with vid at x0, where the move
    // starts (place_piece() reads a face's first corner's position where no corner has a piece)
    std::vector<SweptEdgesOfPiece> pieces;
    m_vertex_attribute[vid].m_posf = x0;
    band_swept_edges(vid, x0, pieces);
    const std::vector<std::pair<size_t, int>> ring_pieces = band_face_pieces(ring);
    m_vertex_attribute[vid].m_posf = x1;
    const std::vector<std::shared_ptr<const BandVolumeEnergy2D>>& models = m_swept_model.local();
    for (int k = 0; k <= kSweptHalvings; ++k) {
        if (k > 0) {
            m_vertex_attribute[vid].m_posf = x0 + std::ldexp(1., -k) * (x1 - x0);
            m_vertex_attribute[vid].set_pos_to_posf();
            bool inverted = false;
            for (const size_t f : ring) inverted = inverted || is_inverted(f);
            if (inverted) continue;
            for (const size_t f : ring) m_face_attribute[f].m_quality = get_quality(f);
        }
        const double dA = tri_energy_sum(ring, false) - amips_before;
        const Vector2d& x = m_vertex_attribute[vid].m_posf;
        double dB = band_relabel_change(ring, ring_pieces);
        for (size_t i = 0; i < pieces.size(); ++i) {
            const SweptEdgesOfPiece& q = pieces[i];
            const ExactBandIntegral2D& eb = *band_exact_for(q.piece);
            const double delta = region_delta(q.piece);
            double d;
            if (!(i < models.size() && models[i] &&
                  models[i]->swept_value_at(eb, delta, x0, x, q.edges, d))) {
                d = BandVolumeEnergy2D::swept_change(eb, delta, x0, x, q.edges);
            }
            dB += d;
        }
        if (dA + dB <= 0.) { // a NaN refuses; a tie passes
            ++c.accepted[size_t(k)];
            return true;
        }
    }
    ++c.refused;
    return false;
}

double TopoOffsetTriMesh::global_energy() const
{
    // E = int_B (d - delta) / delta dA (exact, or as the corner bound) + (w / s^3) sum area
    // AMIPS^3: every live face's tri_energy() once. As in 3D.
    double s = 0.;
    for (const Tuple& f : get_faces()) s += tri_energy(f.fid(*this));
    return s;
}

double TopoOffsetTriMesh::offset_energy() const
{
    // See the declaration: band_cell_term() over the band faces only. As in 3D.
    double s = 0.;
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        if (face_is_offset_band(fid)) s += band_cell_term(oriented_tri_vids(fid));
    }
    return s;
}

std::pair<double, double> TopoOffsetTriMesh::energy_parts() const
{
    // See the declaration. A face's tri_energy() is cell_energy(vids, A, band): its AMIPS part
    // (plus the repulsion term, 0 in the loop) and, on a band face, band_cell_term().
    double amips = 0., band = 0.;
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        if (!face_is_offset_band(fid)) {
            amips += tri_energy(fid);
            continue;
        }
        const std::array<size_t, 3> vids = oriented_tri_vids(fid);
        const double a = get_quality(fid);
        if (a >= MAX_ENERGY) {
            amips += MAX_ENERGY;
            continue;
        }
        amips += cell_energy(vids, a, false);
        band += band_cell_term(vids);
    }
    return {amips, band};
}

void TopoOffsetTriMesh::log_energy_step(const int turn, const char* step) const
{
    const auto [amips, band] = energy_parts();
    logger().info(
        "\t[energy step] turn {} {}: E {:.12g} | AMIPS term {:.12g} | band term {:.12g}",
        turn,
        step,
        amips + band,
        amips,
        band);
}

int TopoOffsetTriMesh::nearest_piece(const Vector2d& p) const
{
    // See the declaration.
    if (!m_input_complex_bvh) return -1;
    Vector2d foot, seg_normal;
    bool on_corner = false;
    int feature = -1;
    m_input_complex_bvh->nearest_point_feature(p, foot, on_corner, seg_normal, feature);
    if (feature < 0) return -1;
    const std::vector<int64_t>& src = on_corner ? m_phi_vert_region : m_phi_seg_region;
    return size_t(feature) < src.size() ? int(src[size_t(feature)]) : -1;
}

int TopoOffsetTriMesh::place_piece(const size_t* vids, const size_t n) const
{
    // See the declaration. As in 3D, m_face_region in place of m_cell_region.
    if (m_offset_params.offsets.size() <= 1 || m_region_potentials.empty()) return -1;
    if (m_region_potentials.size() == 1) return 0;
    int best = -1;
    double best_delta = std::numeric_limits<double>::infinity();
    const auto consider = [&](const int r) {
        if (r >= 0 && region_delta(r) < best_delta) {
            best_delta = region_delta(r);
            best = r;
        }
    };
    for (size_t i = 0; i < n; ++i) {
        const int r = vertex_region(vids[i]);
        if (r == -2) {
            for (const size_t f : get_one_ring_fids_for_vertex(vids[i]))
                consider(f < m_face_region.size() ? m_face_region[f] : -1);
        } else {
            consider(r);
        }
    }
    if (best >= 0) return best;
    return nearest_piece(m_vertex_attribute[vids[0]].m_posf);
}

void TopoOffsetTriMesh::assign_band_regions(const bool log)
{
    // See m_region_potentials. A flood fill over the band faces, seeded from every band face
    // that shares an edge with an input-complex face, with that face's region. A band face
    // reached with two different regions (the bands merged) and a vertex whose band faces
    // disagree read -2 and fall back to the union field -- reported, never silent.
    m_face_region.assign(tri_capacity(), -1);
    m_vertex_region.assign(vert_capacity(), -1);
    if (m_n_regions <= 1 || m_region_potentials.empty()) return;
    // Which piece the seed belongs to, read geometrically off the captured complex: the shared
    // edge lies on the input complex, so its midpoint is at distance 0 from its own piece and at
    // the pieces' separation from any other. Re-derived rather than carried, because nothing
    // propagates a region index through split and collapse -- the same reason
    // classify_curve_edges() re-derives on_curve.
    const auto region_at = [&](const Vector2d& p) { return nearest_piece(p); };
    std::vector<size_t> queue;
    // The seed is a complex VERTEX (label 1) on a band face, not a band face across an edge from
    // a complex FACE: the face rule seeds nothing on a curve or point complex, which has no
    // faces, so the whole band would fall back to the union field. For a region complex the
    // vertex rule seeds exactly the same faces, since a band face on a region's boundary edge is
    // incident to that edge's endpoints. The vertex sits ON the complex, so the query below is at
    // distance 0 from its own piece.
    std::vector<int> piece_of_vertex(vert_capacity(), -3); // -3: not looked up yet
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        if (!face_is_offset_band(fid)) continue;
        for (const size_t v : oriented_tri_vids(fid)) {
            if (m_vertex_extra[v].label != 1) continue;
            if (piece_of_vertex[v] == -3) {
                piece_of_vertex[v] = region_at(m_vertex_attribute[v].m_posf);
            }
            const int r = piece_of_vertex[v];
            if (r < 0) continue;
            if (m_face_region[fid] == -1) {
                m_face_region[fid] = r;
                queue.push_back(fid);
            } else if (m_face_region[fid] >= 0 && m_face_region[fid] != r) {
                m_face_region[fid] = -2;
            }
        }
    }
    while (!queue.empty()) {
        const size_t f = queue.back();
        queue.pop_back();
        const int r = m_face_region[f];
        if (r < 0) continue;
        for (int j = 0; j < 3; ++j) {
            const std::optional<Tuple> opp = tuple_from_edge(f, j).switch_face(*this);
            if (!opp) continue;
            const size_t g = opp->fid(*this);
            if (!face_is_offset_band(g)) continue;
            if (m_face_region[g] == -1) {
                m_face_region[g] = r;
                queue.push_back(g);
            } else if (m_face_region[g] >= 0 && m_face_region[g] != r) {
                m_face_region[g] = -2;
            }
        }
    }
    std::vector<size_t> n_faces(size_t(m_n_regions), 0);
    size_t n_mixed_faces = 0, n_unreached = 0, n_mixed_verts = 0;
    for (size_t f = 0; f < m_face_region.size(); ++f) {
        if (!tuple_from_tri(f).is_valid(*this) || !face_is_offset_band(f)) continue;
        const int r = m_face_region[f];
        if (r == -2) {
            ++n_mixed_faces;
            continue;
        }
        if (r < 0) {
            ++n_unreached;
            continue;
        }
        ++n_faces[size_t(r)];
        for (const size_t v : oriented_tri_vids(f)) {
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
    for (size_t r = 0; r < n_faces.size(); ++r)
        per += fmt::format("{}{}", r ? " / " : "", n_faces[r]);
    if (n_mixed_faces > 0 || n_unreached > 0 || n_mixed_verts > 0) {
        logger().warn(
            "\t[regions] band faces per region {} | {} faces reached from TWO regions, {} reached "
            "from none, {} vertices on faces of two regions -- all fall back to the union field",
            per,
            n_mixed_faces,
            n_unreached,
            n_mixed_verts);
    } else {
        logger().info("\t[regions] band faces per region {}", per);
    }
}

void TopoOffsetTriMesh::log_front_profile(const size_t vid)
{
    // Diagnostic: the band term of one vertex's band faces along its normal, at 21 points across
    // +-delta/2. Logged once, on non-convergence, for the worst vertex. As in 3D.
    if (vid == static_cast<size_t>(-1) || vid >= m_vertex_attribute.size() || !m_offset_potential)
        return;
    const int region = vertex_region(vid);
    const std::shared_ptr<const OffsetPotential2D> pot = potential_ptr_for(vid);
    const Vector2d x0 = m_vertex_attribute[vid].m_posf;
    Vector2d g = pot->gradient(x0);
    if (!(g.norm() > 0.) || !g.allFinite()) return;
    const Vector2d n = g / g.norm(); // toward the input for the smooth field, away for Euclidean
    auto band = front_energy(vid);
    const double delta = vertex_delta(vid); // the vertex's piece's (one entry: the run's)
    logger().info(
        "[front profile] worst vertex {} at ({:.5}, {:.5}), region {}, along the field direction "
        "n = ({:.4}, {:.4}); columns: s/delta | band term of the vertex's band faces "
        "(front_energy(), no AMIPS)",
        vid,
        x0.x(),
        x0.y(),
        region,
        n.x(),
        n.y());
    for (int k = -10; k <= 10; ++k) {
        const double sd = 0.05 * k;
        const Vector2d x = x0 + sd * delta * n;
        Eigen::VectorXd xv(2);
        xv << x.x(), x.y();
        logger().info("[front profile] {:+.2f} | {:.6g}", sd, band->value(xv));
    }
}

double TopoOffsetTriMesh::front_chord_target(
    const size_t va,
    const size_t vb,
    const double len,
    const double sag,
    const double tube) const
{
    // The length that resolves this chord: 3/4 L (tube / sag)^(1/p), capped at L/2, with the
    // exponent p measured rather than assumed. p is how fast the sag falls when the chord is
    // halved -- 2 on a smooth level set, where sag = L^2 / (8 rho), but 1 where the chord
    // straddles a kink of the level set (for the Euclidean field, the medial axis of an input
    // corner): there the turn between the endpoint gradients is a fixed jump halving does not
    // shrink, so the sag falls only like L.
    //
    // p from the turn, with no threshold: phi is the turn between the endpoint gradients, phi_a
    // and phi_b the turns each half would carry (the midpoint gradient splits it). A smooth arc
    // splits the turn evenly, so a half's sag is a quarter; a kink puts the whole jump in one
    // half, whose sag is a half. So ratio = (max(phi_a, phi_b) / phi) / 2 is the predicted sag
    // fraction, in [1/4, 1/2], and p = -log2(ratio) maps it back: 1/4 -> 2, 1/2 -> 1. (Until
    // the 3D port was mirrored back this read -1 / log2(ratio), which maps 1/4 to 0.5 and 1/2
    // to 1 and so over-refined every smooth chord by (sag / tube)^(1..2) instead of the square
    // root.) Anything degenerate falls back to 2. Same as 3D.
    double p = 2.;
    const OffsetPotential2D& pot = potential_for_edge(va, vb);
    const Vector2d pa = m_vertex_attribute[va].m_posf, pb = m_vertex_attribute[vb].m_posf;
    const Vector2d ga = pot.gradient(pa), gb = pot.gradient(pb), gm = pot.gradient(0.5 * (pa + pb));
    const double na = ga.norm(), nb = gb.norm(), nm = gm.norm();
    if (std::isfinite(na) && na > 0. && std::isfinite(nb) && nb > 0. && std::isfinite(nm) &&
        nm > 0.) {
        const Vector2d ua = ga / na, ub = gb / nb, um = gm / nm;
        const auto turn = [](const Vector2d& u, const Vector2d& v) {
            return std::atan2(std::abs(u.x() * v.y() - u.y() * v.x()), u.dot(v));
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

size_t TopoOffsetTriMesh::refine_front_by_halving(
    const std::vector<EnergyCriterion::Refinable>& edges)
{
    // The ends, edge by edge in the order given; the vertex form halves each once.
    std::vector<size_t> ends;
    ends.reserve(2 * edges.size());
    for (const EnergyCriterion::Refinable& r : edges) {
        for (const size_t v : {r.a, r.b}) ends.push_back(v);
    }
    return refine_front_by_halving(ends);
}

size_t TopoOffsetTriMesh::refine_front_by_halving(const std::vector<size_t>& vertices)
{
    const double l = std::max(m_params.l, 1e-300);
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / l);
    // Each vertex is halved once per call: the first entry that names it does the halving and
    // marks it, so a vertex shared by several refinable edges is not halved several times.
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

double TopoOffsetTriMesh::edge_stencil_max_sq(const std::array<size_t, 2>& e) const
{
    // See the declaration. (d - delta)^2 = (delta r)^2, r the potential's relative_residual() =
    // (d - delta) / delta. Several entries of offsets: r on the chord's piece's field, in the
    // run's units (run_units_scale()). As in 3D.
    if (!m_offset_potential) return 0.;
    const double delta = run_units_scale(e, m_offset_params.target_distance);
    const OffsetPotential2D& pot = potential_for_edge(e[0], e[1]);
    double m = 0.;
    bool unmeasurable = false;
    for_each_edge_sample(
        m_vertex_attribute[e[0]].m_posf,
        m_vertex_attribute[e[1]].m_posf,
        [&](const Vector2d& q, double, double) {
            if (unmeasurable) return;
            const double r = pot.relative_residual(q);
            if (!std::isfinite(r)) {
                unmeasurable = true;
                return;
            }
            m = std::max(m, (delta * r) * (delta * r));
        });
    return unmeasurable ? std::numeric_limits<double>::infinity() : m;
}

bool TopoOffsetTriMesh::settle_refinement_turn(
    const int turn,
    double& e_prev,
    const bool refine_now)
{
    // See the declaration; TopoOffsetTetMesh::settle_refinement_turn() one dimension down, whose
    // measurements motivate each test.
    if (!m_band_exact) {
        log_and_throw_error(
            "EXPERIMENTAL_settle_refinement in 2D reads the distance to the input through the "
            "exact band integral, which this run did not build (init_band_exact())");
    }
    const double delta = m_offset_params.target_distance;
    const double eps = m_offset_params.front_conv;
    const double e_now = settle_metric(); // no AMIPS: see offset_energy()
    const double de = e_prev - e_now;
    e_prev = e_now;
    const std::vector<std::array<size_t, 2>> edges = offset_surface_edges();
    double length = 0.;
    for (const auto& e : edges) {
        length += (m_vertex_attribute[e[1]].m_posf - m_vertex_attribute[e[0]].m_posf).norm();
    }
    // The bar energy: the offset term's excess for a front of length L lying eps beyond the level
    // set, per unit length int_0^eps s / delta ds = eps^2 / (2 delta). A turn whose E fell by less
    // has settled: the front no longer moves by a bar. As in 3D, its area A for L.
    const double bar_energy = eps * eps * length / (2. * delta);
    const bool settled = de < bar_energy;
    m_turn_settled = settled; // check_level_set_reachable()
    // The floor of refine_front_by_halving().
    const double l = std::max(m_params.l, 1e-300);
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / l);
    size_t n_marked = 0, n_floor = 0, n_halved = 0, n_over_max = 0, n_over_rms = 0;
    size_t n_over_vertex = 0, n_front_vertices = 0, n_gap_cells = 0, n_gap_over = 0;
    double max_inside = -std::numeric_limits<double>::infinity(), max_outside = max_inside;
    double max_rms = 0., max_vertex = 0., max_gap = 0.;
    const bool rms_mode = m_offset_params.settle_mark == "rms";
    const bool edge_mode = m_offset_params.settle_mark == "vertex_edge";
    const bool split_mode = m_offset_params.settle_mark == "vertex_split";
    const bool vertex_mode = m_offset_params.settle_mark == "vertex" || edge_mode || split_mode;
    const bool stencil_mode = m_offset_params.settle_mark == "stencil";
    size_t n_over_stencil = 0;
    double max_stencil = 0.;
    size_t n_edge_over = 0, n_edge_floor = 0, n_off_chords = 0;
    double max_edge = 0., max_chord = 0.;
    std::vector<std::array<size_t, 2>> to_split;
    if (settled) {
        // Per front chord f, how far a point of f lies from the level set: inside_f = delta - min
        // over f of d, exact (ExactBandIntegral2D::min_distance()); outside_f = max over f of d,
        // minus delta, exact up to rounding (ExactBandIntegral2D::max_distance(): f cut by the
        // input's nearest-feature regions, the max at a piece's end; an upper bound only on
        // rule-(a) pieces). A chord with either over eps has its ends halved. settle_mark "rms",
        // "vertex" and "stencil" only log outside_f and keep the former bound (distance_bound()).
        // Each test at its place's own target distance and bar (region_delta(), front_conv_of()):
        // with one entry, the run's.
        std::vector<size_t> corners;
        for (const auto& f : edges) {
            const int piece_f = place_piece(f.data(), 2);
            const double delta_f = region_delta(piece_f);
            const double eps_f = front_conv_of(piece_f);
            const ExactBandIntegral2D& eb_f = *band_exact_for(piece_f);
            const std::array<Vector2d, 2> p = {
                {m_vertex_attribute[f[0]].m_posf, m_vertex_attribute[f[1]].m_posf}};
            const double inside = delta_f - eb_f.min_distance(p);
            const double outside =
                (!rms_mode && !vertex_mode && !stencil_mode ? eb_f.max_distance(p)
                                                            : eb_f.distance_bound(p)) -
                delta_f;
            max_inside = std::max(max_inside, inside / eps_f);
            max_outside = std::max(max_outside, outside / eps_f);
            const bool over_max = std::max(inside, outside) > eps_f;
            // EXPERIMENTAL_settle_mark "rms": the mean over f of (d - delta)^2 cos(theta) by the
            // 2-point Gauss rule, theta between f's normal out of the band and grad d (signed),
            // against front_conv^2. The normal is oriented by the band face carrying f
            // (front_edge_out_normal()).
            Vector2d n;
            const bool has_band = front_edge_out_normal(f[0], f[1], n);
            double q = 0.;
            if (has_band) {
                const double h = 0.5 / std::sqrt(3.);
                for (const double w : {0.5 - h, 0.5 + h}) {
                    Vector2d g;
                    const double dx = eb_f.distance(Vector2d((1. - w) * p[0] + w * p[1]), g);
                    q += 0.5 * (dx - delta_f) * (dx - delta_f) * n.dot(g);
                }
            }
            const bool over_rms = has_band && q > eps_f * eps_f;
            max_rms = std::max(max_rms, std::sqrt(std::max(q, 0.)) / eps_f);
            n_over_max += over_max;
            n_over_rms += over_rms;
            // EXPERIMENTAL_settle_mark "stencil": a point of f's stencil at or beyond front_conv,
            // (d - delta)^2 >= front_conv^2, or unmeasurable.
            bool over_stencil = false;
            if (stencil_mode) {
                const double m = edge_stencil_max_sq(f);
                max_stencil = std::max(max_stencil, std::sqrt(m) / eps);
                over_stencil = !(m < eps * eps);
                n_over_stencil += over_stencil;
            }
            if (vertex_mode || !(stencil_mode ? over_stencil : rms_mode ? over_rms : over_max)) {
                continue;
            }
            ++n_marked;
            bool at_floor = true;
            for (const size_t v : f) {
                corners.push_back(v);
                at_floor = at_floor && m_vertex_attribute[v].m_sizing_scalar <= s_floor;
            }
            if (at_floor) ++n_floor;
        }
        // EXPERIMENTAL_settle_mark "vertex": a front vertex whose own distance from the level set,
        // |d(v) - delta|, exceeds front_conv (d exact at the vertex). It says nothing about the
        // points of a chord between its ends.
        std::vector<size_t> front_vids;
        for (const auto& f : edges) front_vids.insert(front_vids.end(), f.begin(), f.end());
        wmtk::vector_unique(front_vids);
        n_front_vertices = front_vids.size();
        const auto level_set_error = [&](const Vector2d& x, const int piece) {
            return std::abs(band_exact_for(piece)->distance(x) - region_delta(piece));
        };
        std::vector<char> vertex_over(edge_mode || split_mode ? vert_capacity() : 0, 0);
        for (const size_t v : front_vids) {
            const int piece_v = place_piece(&v, 1);
            const double eps_v = front_conv_of(piece_v);
            const double err = level_set_error(m_vertex_attribute[v].m_posf, piece_v);
            max_vertex = std::max(max_vertex, err / eps_v);
            if (!(err > eps_v)) continue;
            ++n_over_vertex;
            if (!vertex_mode) continue;
            ++n_marked;
            corners.push_back(v);
            if (edge_mode || split_mode) vertex_over[v] = 1;
            if (m_vertex_attribute[v].m_sizing_scalar <= s_floor) ++n_floor;
        }
        // The chords with both ends within front_conv of the level set and the midpoint not:
        // such a chord spans a stretch of the level set its ends do not see, which the vertex test
        // passes for good. EXPERIMENTAL_settle_mark "vertex_edge" halves their ends,
        // "vertex_split" lists them to be split; neither holds the exit, which stays the vertex
        // test. See the 3D twin for the measurements (t100033 inward at 0.5 delta_1, 10%).
        const auto off_midpoint = [&](const std::array<size_t, 2>& e, double& ratio) {
            if (vertex_over[e[0]] || vertex_over[e[1]]) return false;
            const int piece_e = place_piece(e.data(), 2);
            const double eps_e = front_conv_of(piece_e);
            const double err = level_set_error(
                0.5 * (m_vertex_attribute[e[0]].m_posf + m_vertex_attribute[e[1]].m_posf),
                piece_e);
            ratio = std::max(ratio, err / eps_e);
            return err > eps_e;
        };
        if (edge_mode) {
            for (const auto& e : edges) {
                if (!off_midpoint(e, max_edge)) continue;
                ++n_edge_over;
                bool at_floor = true;
                for (const size_t v : e) {
                    corners.push_back(v);
                    at_floor = at_floor && m_vertex_attribute[v].m_sizing_scalar <= s_floor;
                }
                if (at_floor) ++n_edge_floor;
            }
        }
        // "vertex_split": the chord itself is split (3D: the edge of the face with the midpoint
        // farthest from the level set; a chord's one edge is itself, its centroid its midpoint).
        if (split_mode) {
            for (const auto& e : edges) {
                if (!off_midpoint(e, max_chord)) continue;
                ++n_off_chords;
                to_split.push_back(e);
            }
        }
        // EXPERIMENTAL_gap_refinement: a band face with a front corner whose corner bound may lie
        // more than a bar above its true integral: md - d(centroid) > front_conv, md the corner
        // mean of the bounding primitive's distance. Where d is convex on the face the true mean
        // of d is at least d(centroid) (Jensen), so area (md - d(c)) / delta bounds the gap. As in
        // 3D (the cylinder outward, 2026-10-09).
        if (m_offset_params.gap_refinement) {
            std::vector<size_t> cells;
            for (const size_t v : front_vids) {
                for (const size_t t : get_one_ring_fids_for_vertex(v)) {
                    if (face_is_offset_band(t)) cells.push_back(t);
                }
            }
            wmtk::vector_unique(cells);
            n_gap_cells = cells.size();
            for (const size_t t : cells) {
                const auto vs = oriented_tri_vids(t);
                std::array<Vector2d, 3> p;
                for (int j = 0; j < 3; ++j) p[size_t(j)] = m_vertex_attribute[vs[size_t(j)]].m_posf;
                int64_t best = -1;
                const int piece_t = place_piece(vs.data(), 3);
                const ExactBandIntegral2D& eb = *band_exact_for(piece_t);
                const double md = eb.min_mean_distance(p.data(), 3, best);
                const double dc = eb.distance(Vector2d((p[0] + p[1] + p[2]) / 3.));
                const double eps_t = front_conv_of(piece_t);
                max_gap = std::max(max_gap, (md - dc) / eps_t);
                if (!(md - dc > eps_t)) continue;
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
        if (refine_now) m_settle_split_edges = to_split;
    }
    // A marked chord with both ends at the floor is one nothing will refine: it blocks the exit
    // for good, hence the warning, every turn such chords exist.
    logger().log(
        n_floor > 0 ? spdlog::level::warn : spdlog::level::info,
        "\t[settle] turn {}: offset energy {:.10g}, its fall {:.4g}, bar energy b {:.4g} (front "
        "length {:.6g}), settled {}{}",
        turn,
        e_now,
        de,
        bar_energy,
        length,
        settled ? "yes" : "no",
        settled ? fmt::format(
                      " | marked {} {} of {} by the {} test (max test: {} chords over, max "
                      "inside {:.4} bars, max outside {:.4} bars; rms test: {} chords over, max "
                      "{:.4} bars; vertex test: {} vertices over, max {:.4} bars{}{}), {}, {} "
                      "marked {} at the sizing floor {:.4g}{}",
                      n_marked,
                      vertex_mode ? "vertices" : "chords",
                      vertex_mode ? fmt::format("{} front vertices", n_front_vertices)
                                  : fmt::format("{}", edges.size()),
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
                                "; gap test: {} of {} band faces at the front over, max {:.4} "
                                "bars (refined)",
                                n_gap_over,
                                n_gap_cells,
                                max_gap)
                          : std::string(),
                      stencil_mode ? fmt::format(
                                         "; stencil test: {} chords over, max {:.4} bars",
                                         n_over_stencil,
                                         max_stencil)
                                   : std::string(),
                      refine_now ? fmt::format("halved {} vertices", n_halved)
                                 : std::string("no halving (this loop does not refine)"),
                      n_floor,
                      vertex_mode ? "vertices" : "chords with both ends",
                      s_floor,
                      n_floor > 0 ? " (they cannot be refined and block the exit)" : "")
                : std::string(" -- no chord test, no refinement, no exit: the front still moves"));
    if (edge_mode && settled) {
        logger().info(
            "\t[settle edge] turn {}: {} of {} front chords with both ends within front_conv and "
            "the midpoint over (max {:.4} bars), their ends refined, {} with both ends at the "
            "sizing floor; they do not hold the exit",
            turn,
            n_edge_over,
            edges.size(),
            max_edge,
            n_edge_floor);
    }
    // The listed chords do not hold the exit, which stays the vertex test: they are split only if
    // the run goes on (the next turn), and a run that exits leaves them unsplit.
    if (split_mode && settled) {
        logger().info(
            "\t[settle split] turn {}: {} front chords with both ends within front_conv and the "
            "midpoint over (max {:.4} bars) | {} chords to split before the next turn's first "
            "smoothing pass; they do not hold the exit",
            turn,
            n_off_chords,
            max_chord,
            to_split.size());
    }
    return settled && n_marked == 0 && n_gap_over == 0;
}

TopoOffsetTriMesh::SmoothingProgress TopoOffsetTriMesh::smoothing_progress(
    const std::vector<Vector2d>& before)
{
    SmoothingProgress s;
    const double l = std::max(m_params.l, 1e-16);
    // A front vertex's STEP against the one bar; the background uses its own sizing target below.
    const double tube = m_offset_params.front_conv;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        const Vector2d& x = m_vertex_attribute[vid].m_posf;
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

void TopoOffsetTriMesh::smooth_group_to_convergence(const char* group_name)
{
    // The adaptive_smoothing keys were removed on the 3D-only branch spenbert/offsets-cleanup
    // (2026-10-13); their old defaults are hard-wired here until the 2D port.
    const int max_passes = /* adaptive_smoothing_max_passes */ 20;
    const double stall_rel = /* adaptive_smoothing_stall_rel */ 0.1;
    const double step_rel = /* adaptive_smoothing_step_rel */ 0.01;
    std::vector<Vector2d> before;
    double prev_front = std::numeric_limits<double>::infinity();
    for (int p = 0; p < max_passes; ++p) {
        before.assign(vert_capacity(), Vector2d::Zero());
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

void TopoOffsetTriMesh::grade_sizing(double grade, const std::vector<size_t>& seeds)
{
    if (seeds.empty()) return;
    if (m_offset_params.sizing_gradation_mode == "distance") {
        grade_sizing_by_distance(seeds);
    } else {
        gradation_smooth_sizing(grade, seeds);
    }
}

size_t TopoOffsetTriMesh::grade_sizing_by_distance(const std::vector<size_t>& seeds)
{
    // Ported statement for statement from tetwild::TetWild::adjust_sizing_field (the attic/
    // directory, since deleted; read attic/app/tetwild/TetWild.cpp at 2371c13fe8), one
    // dimension down: the same R, the same ramp, the same breadth-first walk that stops at R,
    // the same floor. Two parts of that function are NOT ported on purpose:
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
    std::vector<Vector2d> pts;
    pts.reserve(seeds.size());
    for (const size_t v : seeds) {
        if (is_seed[v]) continue;
        is_seed[v] = 1;
        pts.push_back(m_vertex_attribute[v].m_posf);
    }
    // grid of cell size R: every seed within R of a query lies in the query's cell or one of
    // its 8 neighbours.
    Vector2d lo = pts[0];
    for (const Vector2d& p : pts) lo = lo.cwiseMin(p);
    auto cell_of = [&](const Vector2d& p) {
        return std::array<int64_t, 2>{
            static_cast<int64_t>(std::floor((p[0] - lo[0]) / R)),
            static_cast<int64_t>(std::floor((p[1] - lo[1]) / R))};
    };
    std::map<std::array<int64_t, 2>, std::vector<size_t>> grid;
    for (size_t i = 0; i < pts.size(); ++i) grid[cell_of(pts[i])].push_back(i);
    auto nearest_seed_dist = [&](const Vector2d& p) {
        const auto c = cell_of(p);
        double best2 = std::numeric_limits<double>::infinity();
        for (int64_t dx = -1; dx <= 1; ++dx)
            for (int64_t dy = -1; dy <= 1; ++dy) {
                const auto it = grid.find({c[0] + dx, c[1] + dy});
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
        for (const size_t n_vid : get_one_ring_vids_for_vertex_duplicate(vid)) {
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

void TopoOffsetTriMesh::check_offset_within_support(const char* when) const
{
    report_outside_support(when, residual_split());
}

void TopoOffsetTriMesh::report_outside_support(const char* when, const DistanceSplit& s) const
{
    if (s.n_outside_support == 0) return;

    log_and_throw_error(
        "{}: {} offset-boundary vertices have left the smooth offset potential's support "
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

std::pair<double, double> TopoOffsetTriMesh::compute_distance_deviation() const
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

TopoOffsetTriMesh::DistanceSplit TopoOffsetTriMesh::distance_deviation_split() const
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

double TopoOffsetTriMesh::face_criterion_rel(const size_t fid) const
{
    // The max of the face's AMIPS over stop_energy and, on each live chord it carries, the root of
    // the chord's edge_offset_term() -- the loop's own chord measure, in units of the bar. Sorted
    // ends, so a chord reads the same from either face. An unmeasurable chord fails. As in 3D.
    double score = quality_rel(fid);
    for (int j = 0; j < 3; ++j) {
        const Tuple e = tuple_from_edge(fid, j);
        if (!edge_is_offset_surface_live(e)) continue;
        std::array<size_t, 2> v = get_edge_vids(e);
        if (v[0] > v[1]) std::swap(v[0], v[1]);
        const double term = edge_offset_term(v[0], v[1]);
        if (!(term >= 0.)) return std::numeric_limits<double>::infinity();
        score = std::max(score, std::sqrt(term));
    }
    return score;
}

size_t TopoOffsetTriMesh::refine_sizing_around_worst(const double max_metric)
{
    // TriWildMesh::refine_sizing_around_worst verbatim -- ranked by m_face_attribute[].m_quality,
    // filtered against stop_energy, seeding the same force-split edges. Final pass only, by
    // construction: mesh_improvement() is this function's one caller, and the driver only ever
    // runs that for the frozen-front finishing pass.
    //
    // Note the units: max_metric arrives from mesh_improvement() as whatever
    // optimization_quality_stats() returned, which is the engine's absolute AMIPS, and the score
    // it is compared against is absolute too.
    const int n_rings = std::max(0, m_params.stuck_refine_rings);

    // Clamped above exactly as TriWild does: without it a single degenerate face (quality
    // MAX_ENERGY) sets filter_energy astronomically high and select_worst_cells then picks
    // out only the degenerate faces, so refinement stops fixing the merely-bad ones.
    const double filter_energy = std::min(std::max(max_metric / 100., m_params.stop_energy), 100.);

    // m_quality is the AMIPS2D energy itself, so no cube root (unlike tetwild/simwild).
    const auto worst = wmtk::utils::select_worst_cells(
        tri_capacity(),
        [this](size_t fid) { return tuple_from_tri(fid).is_valid(*this); },
        [this](size_t fid) { return m_face_attribute[fid].m_quality; },
        filter_energy,
        m_params.stuck_refine_num_worst);
    if (worst.empty()) {
        return 0;
    }

    log_stuck_refine_census(max_metric, filter_energy);
    // What is blocking the fix, next to what is broken. Same filter, so the two censuses cover
    // the same element set and can be read together.
    log_refine_block_census(fmt::format("stuck call {}", m_stuck_calls), filter_energy);

    // Force-split: the longest edge of each selected face, split once next pass regardless of
    // the length gate, WITHOUT touching the sizing field. This is what unsticks a face whose
    // edges are already shorter than their target.
    m_force_split_edges.clear();
    if (m_params.stuck_refine_force_split) {
        for (const auto& [unused_score, fid] : worst) {
            m_force_split_edges.insert(
                wmtk::utils::longest_edge(
                    oriented_tri_vids(fid),
                    [this](size_t vid) -> const Vector2d& {
                        return m_vertex_attribute[vid].m_posf;
                    }));
        }
    }

    std::vector<size_t> seeds;
    seeds.reserve(3 * worst.size());
    for (const auto& [unused_score, fid] : worst) {
        for (const size_t v : oriented_tri_vids(fid)) seeds.push_back(v);
    }
    const auto region = wmtk::utils::grow_vertex_region(seeds, n_rings, [this](size_t v) {
        return get_one_ring_vids_for_vertex_duplicate(v);
    });

    const auto refined = wmtk::utils::apply_sizing_refinement(
        region,
        m_params.stuck_refine_factor,
        m_params.stuck_refine_min_scalar,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; });
    grade_sizing(m_params.stuck_refine_gradation, refined);

    logger().info(
        "[stuck-refine A] worst {} tris (max energy {:.4}, filter {:.4}), refined {} of {} "
        "region vertices",
        worst.size(),
        max_metric,
        filter_energy,
        refined.size(),
        region.size());
    return refined.size();
}

void TopoOffsetTriMesh::log_worst_dist_vertex() const
{
    // The band split first: how much of the error is the optimizer's to fix. The loop and the
    // sizing field only see the reachable half, so a run whose reported max looks bad but whose
    // reachable max is fine is a construction problem, not an optimization one. Both quantities
    // are reported -- the Phi residual the loop converges on, and the Euclidean distance, which
    // says how far the smoothed offset ended up from the exact one.
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

    const Vector2d p = m_vertex_attribute[vid].m_posf;
    const Vector3d near3 = m_input_complex_bvh->nearest_point(VectorXd(p));
    const double d = (p - Vector2d(near3[0], near3[1])).norm();

    // Every gate between this vertex and a corrective move, in the order smoothing hits them.
    const auto& ve = m_vertex_extra[vid];
    int n_offset_e = 0, n_region_e = 0, n_bbox_e = 0;
    for (const Tuple& e : get_one_ring_edges_for_vertex(tuple_from_vertex(vid))) {
        const size_t eid = e.eid(*this);
        n_offset_e += edge_is_offset_surface_live(e);
        n_region_e += edge_is_region(eid);
        n_bbox_e += (m_edge_attribute[eid].m_is_bbox_fs >= 0);
    }
    logger().info(
        "\tworst-dist vertex {}: pos ({:.6}, {:.6}) dist {:.6} target {:.6} err {:.6}",
        vid,
        p[0],
        p[1],
        d,
        vertex_delta(vid),
        std::abs(d - vertex_delta(vid)));
    logger().info(
        "\t  flags: on_offset {} on_input {} on_region {} on_bbox {} rounded {} | boundary mask "
        "{:#x} | incident edges: {} offset, {} region, {} bbox | phi {:.6} (level {:.6}), "
        "residual {:.6}, containment envelope {}",
        ve.m_is_on_offset,
        ve.m_is_on_input,
        ve.m_is_on_region,
        !m_vertex_attribute[vid].on_bbox_faces.empty(),
        m_vertex_attribute[vid].m_is_rounded,
        vertex_boundary_mask(vid),
        n_offset_e,
        n_region_e,
        n_bbox_e,
        potential_for(vid).value(p),
        potential_for(vid).target_level(),
        potential_for(vid).residual_length(p),
        smoothing_containment_envelope(vid) ? "yes" : "none");
    // Which objective the smoother would give it, and whether it is refused before reaching one.
    const char* fate = "TriWild's smooth_after(): AMIPS, no offset term";
    if (!m_vertex_attribute[vid].m_is_rounded) {
        fate = "REFUSED by smooth_before: not rounded";
    } else if (m_freeze_front && ve.m_is_on_offset) {
        fate = "REFUSED by smooth_before: front frozen in the final pass";
    } else if (ve.m_is_on_offset) {
        fate = "the front smoother: AMIPS plus the offset terms (smooth_front_vertex)";
    }
    logger().info("\t  smoothing fate: {}", fate);

    // Every incident edge, with the tag sets of the two faces across it: the ground truth the
    // classification is derived from. label is a 3-way coarsening of these sets, so the pair says
    // exactly why an edge landed in the class it did.
    const auto tags_to_string = [](const CellTag& tags) {
        std::string s = "{";
        for (const int64_t t : tags) {
            if (s.size() > 1) s += ",";
            s += std::to_string(t);
        }
        return s + "}";
    };
    for (const Tuple& e : get_one_ring_edges_for_vertex(tuple_from_vertex(vid))) {
        const size_t eid = e.eid(*this);
        const size_t nb = (e.vid(*this) == vid) ? e.switch_vertex(*this).vid(*this) : e.vid(*this);
        const std::optional<Tuple> opp = e.switch_face(*this);
        const char* cls = "untracked";
        if (m_edge_attribute[eid].m_is_surface_fs) {
            cls =
                m_edge_attribute[eid].m_surface_class == OFFSET_SURFACE_CLASS ? "OFFSET" : "REGION";
        } else if (m_edge_attribute[eid].m_is_bbox_fs >= 0) {
            cls = "bbox";
        }
        if (!opp) {
            logger().info("\t  edge ->{}: class {} (domain boundary, one face)", nb, cls);
            continue;
        }
        const size_t fa = e.fid(*this), fb = opp->fid(*this);
        logger().info(
            "\t  edge ->{}: class {} | faces {} tags {} label {} band {} | {} tags {} label {} "
            "band {}",
            nb,
            cls,
            fa,
            tags_to_string(m_face_attribute[fa].tags),
            m_face_extra[fa].label,
            face_is_offset_band(fa),
            fb,
            tags_to_string(m_face_attribute[fb].tags),
            m_face_extra[fb].label,
            face_is_offset_band(fb));
    }
}

bool TopoOffsetTriMesh::edge_is_offset_surface_live(const Tuple& e) const
{
    // The band's OUTER boundary, recomputed from the labels on every call: the operations that
    // ask this run between one label_offset_boundary() and the next. A band face meeting a face
    // that is neither band nor input complex.
    const size_t fa = e.fid(*this);
    const std::optional<Tuple> opp = e.switch_face(*this);
    if (!opp) {
        // Domain boundary. A band face here means the band was clipped by the bounding box, and
        // that edge is offset boundary -- its vertices can never reach the target distance, which
        // is precisely the thing that must be measured rather than hidden. As in 3D.
        return face_is_offset_band(fa);
    }
    const size_t fb = opp->fid(*this);
    const bool a = face_is_offset_band(fa), b = face_is_offset_band(fb);
    if (a == b) return false; // both in the band, or neither: not the band's boundary
    // The band's inner interface, against the input complex it wraps, sits at distance 0 by
    // construction and would drag the reported error to target_distance everywhere.
    return !face_is_input_complex(a ? fb : fa);
}

std::vector<std::array<size_t, 2>> TopoOffsetTriMesh::offset_surface_edges() const
{
    std::set<std::array<size_t, 2>> edges;
    for (const Tuple& e : get_edges()) {
        if (!edge_is_offset_surface_live(e)) continue;
        std::array<size_t, 2> v = get_edge_vids(e);
        if (v[0] > v[1]) std::swap(v[0], v[1]);
        edges.insert(v);
    }
    return std::vector<std::array<size_t, 2>>(edges.begin(), edges.end());
}

bool TopoOffsetTriMesh::vertex_has_live_offset_edge(const size_t vid) const
{
    // Reads vid's own face list and the faces in it, nothing else: every edge through vid is
    // shared by at most two faces, and both contain vid, so pairing the edges within the list
    // finds the face across each one without looking up any other vertex's list. An edge with
    // one face in the list is on the domain boundary. The verdict per edge is
    // edge_is_offset_surface_live()'s. As in 3D.
    const std::vector<size_t>& ring = get_one_ring_fids_for_vertex(vid);
    struct Side
    {
        size_t other; ///< the edge's other end
        size_t fid;
    };
    std::vector<Side> sides;
    sides.reserve(2 * ring.size());
    for (const size_t fid : ring) {
        for (const size_t w : oriented_tri_vids(fid)) {
            if (w != vid) sides.push_back({w, fid});
        }
    }
    std::sort(sides.begin(), sides.end(), [](const Side& a, const Side& b) {
        return a.other != b.other ? a.other < b.other : a.fid < b.fid;
    });
    for (size_t i = 0; i < sides.size();) {
        size_t j = i + 1;
        while (j < sides.size() && sides[j].other == sides[i].other) ++j;
        const size_t fa = sides[i].fid;
        if (j - i == 1) {
            if (face_is_offset_band(fa)) return true; // band edge on the domain boundary
        } else {
            const size_t fb = sides[i + 1].fid;
            const bool a = face_is_offset_band(fa), b = face_is_offset_band(fb);
            if (a != b && !face_is_input_complex(a ? fb : fa)) return true;
        }
        i = j;
    }
    return false;
}

void TopoOffsetTriMesh::refresh_offset_membership(const size_t vid)
{
    m_vertex_extra[vid].m_is_on_offset = vertex_has_live_offset_edge(vid);
}

std::pair<size_t, size_t> TopoOffsetTriMesh::offset_membership_mismatches() const
{
    size_t flagged_not_live = 0, live_not_flagged = 0;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        const bool live = vertex_has_live_offset_edge(vid);
        const bool flag = m_vertex_extra[vid].m_is_on_offset;
        if (flag && !live) ++flagged_not_live;
        if (live && !flag) ++live_not_flagged;
    }
    return {flagged_not_live, live_not_flagged};
}

void TopoOffsetTriMesh::check_offset_membership(const char* when) const
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
            "offset-boundary edge, {} have one without the flag. The propagation in "
            "split_adjust_position / collapse_after_vertex missed a case.",
            when,
            flagged_not_live,
            live_not_flagged);
    }
}

int64_t TopoOffsetTriMesh::front_edge_band_face(const size_t a, const size_t b) const
{
    for (const size_t fid : get_incident_fids_for_edge(a, b)) {
        if (face_is_offset_band(fid)) return int64_t(fid);
    }
    return -1;
}

bool TopoOffsetTriMesh::front_edge_out_normal(const size_t a, const size_t b, Vector2d& n) const
{
    // See the declaration. As 3D's front_face_out_normal().
    const int64_t bf = front_edge_band_face(a, b);
    if (bf < 0) return false;
    return front_edge_out_normal_in(a, b, size_t(bf), n);
}

bool TopoOffsetTriMesh::front_edge_out_normal_in(
    const size_t a,
    const size_t b,
    const size_t band_fid,
    Vector2d& n) const
{
    // See the declaration. As 3D's front_face_out_normal_in().
    const Vector2d& pa = m_vertex_attribute[a].m_posf;
    const Vector2d d = m_vertex_attribute[b].m_posf - pa;
    n = Vector2d(-d.y(), d.x());
    if (!(n.norm() > 0.)) return false;
    n.normalize();
    for (const size_t v : oriented_tri_vids(band_fid)) {
        if (v == a || v == b) continue;
        if (n.dot(pa - m_vertex_attribute[v].m_posf) < 0.) n = -n;
    }
    return true;
}

double TopoOffsetTriMesh::front_edge_field_angle(const size_t a, const size_t b) const
{
    // See the declaration. As 3D's front_face_field_angle().
    const int64_t bf = front_edge_band_face(a, b);
    if (bf < 0) return std::numeric_limits<double>::quiet_NaN();
    return front_edge_field_angle_in(a, b, size_t(bf));
}

double TopoOffsetTriMesh::front_edge_field_angle_in(
    const size_t a,
    const size_t b,
    const size_t band_fid) const
{
    // See the declaration. As 3D's front_face_field_angle_in() and front_face_angle_to_field().
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (!m_band_exact) return nan;
    Vector2d n;
    if (!front_edge_out_normal_in(a, b, band_fid, n)) return nan;
    const Vector2d x = 0.5 * (m_vertex_attribute[a].m_posf + m_vertex_attribute[b].m_posf);
    // The edge's piece's integral (place_piece(); the whole complex's with one entry).
    const std::array<size_t, 2> e{{a, b}};
    Vector2d g;
    band_exact_for(place_piece(e.data(), 2))->distance(x, g);
    if (!(g.norm() > 0.)) return nan;
    // atan2 of the cross and dot products: accurate at every angle, unlike acos near 0 and 180.
    return std::atan2(std::abs(n.x() * g.y() - n.y() * g.x()), n.dot(g)) * 180. / M_PI;
}

TopoOffsetTriMesh::FoldCount TopoOffsetTriMesh::count_folds(
    std::vector<size_t>* fold_vertices) const
{
    // Each live offset edge once, its normal out of the band; then the vertices with exactly two,
    // sorted by fold_test(). As 3D's count_folds(), one dimension down.
    std::map<size_t, std::vector<std::pair<std::array<size_t, 2>, Vector2d>>> by_vertex;
    for (const Tuple& e : get_edges()) {
        if (!edge_is_offset_surface_live(e) || !e.switch_face(*this)) continue;
        const size_t a = e.vid(*this), b = e.switch_vertex(*this).vid(*this);
        Vector2d n;
        if (!front_edge_out_normal(a, b, n)) continue;
        const std::array<size_t, 2> ev{{a, b}};
        by_vertex[a].push_back({ev, n});
        by_vertex[b].push_back({ev, n});
    }
    FoldCount c;
    for (const auto& [v, es] : by_vertex) {
        if (es.size() != 2) continue;
        double ang[2] = {0., 0.};
        const auto field_angle = [&](const int k) {
            const auto& ev = es[size_t(k)].first;
            return ang[k] = front_edge_field_angle(ev[0], ev[1]);
        };
        const FoldTest t =
            fold_test(es[0].second, es[1].second, field_angle, m_offset_params.foldover_angle);
        if (t == FoldTest::none) continue;
        if (t == FoldTest::ridge_vertex) {
            ++c.ridge_vertices;
            c.ridge_max_angle = std::max({c.ridge_max_angle, ang[0], ang[1]});
            continue;
        }
        ++c.folds;
        if (fold_vertices) fold_vertices->push_back(v);
    }
    return c;
}

TopoOffsetTriMesh::TurnedEdgeCensus TopoOffsetTriMesh::turned_edge_census() const
{
    // See the declaration. As 3D's turned_face_census().
    TurnedEdgeCensus c;
    for (const auto& e : offset_surface_edges()) {
        ++c.n_edges;
        const double ang = front_edge_field_angle(e[0], e[1]);
        if (!std::isfinite(ang)) {
            ++c.n_unmeasured;
            continue;
        }
        if (!(ang > m_offset_params.foldover_angle)) continue;
        ++c.n_turned;
        if (ang > c.worst) {
            c.worst = ang;
            c.worst_at = 0.5 * (m_vertex_attribute[e[0]].m_posf + m_vertex_attribute[e[1]].m_posf);
        }
    }
    return c;
}

void TopoOffsetTriMesh::turned_front_edges_at(
    const size_t vid,
    std::set<std::array<size_t, 2>>& out) const
{
    // See the declaration. The live offset edges through vid found as vertex_has_live_offset_edge()
    // finds them, pairing the edges within vid's face list: an edge with one face in the list is
    // on the domain boundary, the band's when that face is a band face.
    const std::vector<size_t>& ring = get_one_ring_fids_for_vertex(vid);
    struct Side
    {
        size_t other; ///< the edge's other end
        size_t fid;
    };
    std::vector<Side> sides;
    sides.reserve(2 * ring.size());
    for (const size_t fid : ring) {
        for (const size_t w : oriented_tri_vids(fid)) {
            if (w != vid) sides.push_back({w, fid});
        }
    }
    std::sort(sides.begin(), sides.end(), [](const Side& a, const Side& b) {
        return a.other != b.other ? a.other < b.other : a.fid < b.fid;
    });
    for (size_t i = 0; i < sides.size();) {
        size_t j = i + 1;
        while (j < sides.size() && sides[j].other == sides[i].other) ++j;
        const size_t fa = sides[i].fid;
        int64_t band = -1;
        if (j - i == 1) {
            if (face_is_offset_band(fa)) band = int64_t(fa);
        } else {
            const size_t fb = sides[i + 1].fid;
            const bool a = face_is_offset_band(fa), b = face_is_offset_band(fb);
            if (a != b && !face_is_input_complex(a ? fb : fa)) band = int64_t(a ? fa : fb);
        }
        if (band >= 0 && front_edge_field_angle_in(vid, sides[i].other, size_t(band)) >
                             m_offset_params.foldover_angle) {
            out.insert({{std::min(vid, sides[i].other), std::max(vid, sides[i].other)}});
        }
        i = j;
    }
}

std::vector<char> TopoOffsetTriMesh::offset_surface_foldover_labels() const
{
    // See the declaration.
    std::vector<char> fold(vert_capacity(), 0);
    std::vector<size_t> vs;
    count_folds(&vs);
    for (const size_t v : vs) fold[v] = 1;
    return fold;
}

void TopoOffsetTriMesh::check_no_vertex_on_both_surfaces(const char* when) const
{
    // A vertex on both surfaces is unsatisfiable: it sits at distance 0 from the input complex,
    // where Phi diverges, and is at the same time required to sit at target_distance from it, so
    // no placement, no smoothing and no refinement can fix it. The rest of the component steps
    // around such a vertex (dropped from the gradient metric, booked under max_pinned), which is
    // right for the measurement and wrong as the only response -- hence this check.
    //
    // On the domain boundary is different and deliberately not checked: constrained, not
    // contradictory. Checked after every phase and not only at construction, because
    // collapse_after_vertex() ORs the flags and can create the state from two fine vertices.
    std::vector<size_t> both;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!m_vertex_extra[vid].m_is_on_offset || !m_vertex_extra[vid].m_is_on_input) {
            continue;
        }
        // The geometry decides, not the flags. m_is_on_input is over-broad -- the split propagates
        // it onto new vertices and the collapse ORs it onto survivors -- so a vertex can carry it
        // while sitting a full target_distance from the complex, which is exactly where the offset
        // wants it. Unsatisfiable is a geometric fact: smoothing_position_is_allowed() holds an
        // input-complex vertex within envelope_size of the complex and the offset asks it to reach
        // target_distance, and those two demands contradict each other only when the vertex really
        // is on the complex.
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
        "[{}] {} vertices are on BOTH the input complex and the offset boundary. Such a vertex is "
        "at distance 0 from the input and is asked to be at target_distance {} from it at the "
        "same time, so the optimization cannot place it and the offset through it cannot "
        "converge. This is a construction defect, not an optimization failure. Offending "
        "vertices: {}{}",
        when,
        both.size(),
        m_offset_params.target_distance,
        detail,
        both.size() > n_show ? ", ..." : "");
}

bool TopoOffsetTriMesh::edge_borders_released_boundary(const Tuple& e) const
{
    // The same test release_deformable_regions() freed vertices by: the incident faces' CURRENT
    // tag symmetric difference contains a released tag and no source tag. Kept here rather than
    // cached on the edge so the classification always reflects tags as they are now.
    const std::optional<Tuple> opp = e.switch_face(*this);
    if (!opp) return false; // the domain wall
    CellTag edge_tags;
    const auto& t0 = m_face_attribute[e.fid(*this)].tags;
    const auto& t1 = m_face_attribute[opp->fid(*this)].tags;
    std::set_symmetric_difference(
        t0.begin(),
        t0.end(),
        t1.begin(),
        t1.end(),
        std::inserter(edge_tags, edge_tags.begin()));
    bool released_here = false;
    for (const int64_t t : edge_tags) {
        if (m_source_tags.count(t)) return false;
        if (m_deform_tags.count(t)) released_here = true;
    }
    return released_here;
}

std::shared_ptr<Envelope> TopoOffsetTriMesh::released_envelope() const
{
    // deform_others' ops-only tube: a tube around the CURRENT released boundaries, consulted by
    // surface_envelope_for_edge() -- the dispatch every operation containment check comes through
    // and no smoothing path does. Releasing an object frees its boundary of every envelope so
    // smoothing can carry it; without this the operations would be free too, and the collapse pass
    // distorts a released boundary at will while the plastic re-stamp makes each distortion the
    // new rest. eps is the offset tube's: no operation may degrade a tracked boundary by more than
    // the accuracy tube, released or not.
    //
    // Lazy, on a dirty flag the smoothing accepts set, never rebuilt on a fixed cadence: this tube
    // holds a boundary smoothing is SUPPOSED to move, so any fixed schedule leaves a window where
    // the tube lags the boundary it was built from and the engine's sanity sweep reports false
    // alarms. Rebuilding on first query after a smoothing accept makes every consumer judge
    // against the boundary as it is now. Operations never set the flag, so op-by-op drift cannot
    // recenter its own container.
    // WallComplex holds a released boundary with nothing, the operations included.
    if (envelope_setup() == EnvelopeSetup::WallComplex) return nullptr;
    if (m_deform_tags.empty()) return nullptr;
    std::lock_guard<std::mutex> lock(m_released_mutex);
    if (!m_released_tube_dirty.load(std::memory_order_acquire)) return m_released_envelope;
    // Never rebuild mid-operation: a containment query can arrive between an operation's before-
    // and after-hooks -- a split child's check runs before its attributes are written, a
    // collapse's between tentative state and the rollback decision -- and a tube built from that
    // mesh reads recycled slots and tentative geometry. Deferring keeps the group-start tube for
    // the whole operation, which is the semantics anyway: an operation is judged against the shape
    // as of the moment no operation was in flight. `recording` is the engine's
    // rollback-protection flag, thread-local, true exactly inside an operation.
    // const_cast: enumerable_thread_specific::local() has no const overload; this is a read.
    if (const_cast<TopoOffsetTriMesh*>(this)->m_vertex_attribute.recording.local()) {
        return m_released_envelope;
    }
    std::vector<Eigen::Vector2i> segs;
    for (const Tuple& e : get_edges()) {
        if (!m_edge_attribute[e.eid(*this)].m_is_surface_fs) continue;
        if (!edge_borders_released_boundary(e)) continue;
        segs.emplace_back(int(e.vid(*this)), int(e.switch_vertex(*this).vid(*this)));
    }
    if (segs.empty()) {
        m_released_envelope = nullptr;
    } else {
        std::vector<Eigen::Vector2d> verts(vert_capacity());
        for (size_t i = 0; i < vert_capacity(); ++i) {
            verts[i] = m_vertex_attribute[i].m_posf;
        }
        const double eps = std::max(m_offset_params.offset_envelope, 1e-12);
        m_released_envelope = std::make_shared<Envelope>(/*exact=*/true);
        m_released_envelope->init(verts, segs, eps);
    }
    m_released_tube_dirty.store(false, std::memory_order_release);
    return m_released_envelope;
}

void TopoOffsetTriMesh::refresh_released_envelope()
{
    // The released boundaries' ops-only tube: mark and rebuild NOW, at this consistent moment,
    // between passes -- released_envelope() never rebuilds mid-operation.
    m_released_tube_dirty.store(true, std::memory_order_release);
    released_envelope();
}

void TopoOffsetTriMesh::rebuild_offset_envelope()
{
    refresh_released_envelope();
    // First, and on every path out of here including the empty one: each entry is an
    // IntersectionEnvelope holding the tube this call is about to replace.
    {
        std::lock_guard<std::mutex> lock(m_isect_mutex);
        m_offset_isect_cache.clear();
    }

    std::vector<Eigen::Vector2i> segs;
    for (const Tuple& e : get_edges()) {
        if (!edge_is_offset_surface_live(e)) continue;
        segs.emplace_back(int(e.vid(*this)), int(e.switch_vertex(*this).vid(*this)));
    }
    if (segs.empty()) {
        m_offset_envelope = nullptr;
        logger().warn("\t[offset envelope] no offset-boundary segments; the envelope is empty");
        return;
    }

    std::vector<Eigen::Vector2d> verts(vert_capacity());
    for (size_t i = 0; i < vert_capacity(); ++i) {
        verts[i] = m_vertex_attribute[i].m_posf;
    }

    // The leash as init() resolved it: absolute if the config gave one, else offset_envelope_rel
    // x the bbox diagonal. Referenced to the BOX, not to target_distance, since 2026-09-24. As
    // in 3D.
    const double eps = std::max(m_offset_params.offset_envelope, 1e-12);

    m_offset_envelope = std::make_shared<Envelope>(/*exact=*/true); // see the tag envelopes
    m_offset_envelope->init(verts, segs, eps);
    logger().info(
        "\t[offset envelope] rebuilt: {} segments, {} (eps {:.6g} = offset_envelope, "
        "{:.4} x the bbox diagonal)",
        segs.size(),
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

void TopoOffsetTriMesh::append_frame_label(const size_t idx, const std::string& label) const
{
    std::ofstream f(
        m_offset_params.output_path + "_frames.txt",
        idx == 0 ? std::ios::trunc : std::ios::app);
    if (f) f << fmt::format("{:05d}\t{}\n", idx, label);
}

void TopoOffsetTriMesh::write_debug_frame(const std::string& label)
{
    if (!m_params.output_enabled) return;
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

void TopoOffsetTriMesh::write_debug_pvd() const
{
    if (!m_params.output_enabled) return;
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

bool TopoOffsetTriMesh::held_object_edge(const Tuple& e) const
{
    // See the declaration; the 2D twin of TopoOffsetTetMesh::held_object_face(). An edge of a
    // face of the input complex, where 3D asks the face's own label: 2D's operations carry the
    // face labels, not the edge labels. An isolated edge of the complex is a tracked boundary (a
    // tag boundary or the curve group, init_surfaces_and_boundaries()) or on the mesh boundary.
    const std::optional<Tuple> o = e.switch_face(*this);
    if (!o) return true; // the mesh boundary
    if (m_face_extra[e.fid(*this)].label == 1 || m_face_extra[o->fid(*this)].label == 1)
        return true;
    return edge_is_region(e.eid(*this));
}

bool TopoOffsetTriMesh::held_object_vertex(const size_t v) const
{
    // See the declaration; the 2D twin of TopoOffsetTetMesh::held_object_vertex() and
    // held_object_edge(), whose held edges are 2D's held vertices. On the input complex by
    // m_is_on_input, which 2D's operations carry (3D: the vertex's label).
    if (m_vertex_extra[v].m_is_on_input) return true;
    if (m_track_feature_points &&
        m_feature_point_attribute[v].m_feature_point_id != NO_FEATURE_POINT) {
        return true;
    }
    for (const Tuple& e : get_one_ring_edges_for_vertex(v)) {
        if (held_object_edge(e)) return true;
    }
    return false;
}

std::array<int64_t, 4> TopoOffsetTriMesh::held_topology_signature(
    std::map<std::array<double, 2>, std::array<bool, 2>>* vertex_touch,
    std::array<std::array<int64_t, 4>, 4>* by_kind) const
{
    // See the declaration; the 2D twin of TopoOffsetTetMesh::held_topology_signature(). The held
    // simplices by sorted corners: the held edges with their corners, the input-complex and
    // anchored vertices.
    std::map<std::vector<size_t>, size_t> index;
    std::vector<std::vector<size_t>> simplices;
    // the kinds a simplex belongs to: 1 the input complex, 2 the held points (3D: the held curves
    // and points; 2D's held curves, the curve group, are codimension 1 and kind 4), 4 the region
    // boundaries, 8 the box -- a corner of a held edge takes its kind
    std::vector<uint8_t> kinds;
    const auto add = [&](std::vector<size_t> sigma, const uint8_t kind) {
        std::sort(sigma.begin(), sigma.end());
        const auto [it, fresh] = index.emplace(sigma, simplices.size());
        if (fresh) {
            simplices.push_back(sigma);
            kinds.push_back(0);
        }
        kinds[it->second] |= kind;
    };
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        for (int k = 0; k < 3; ++k) {
            const Tuple e = tuple_from_edge(fid, size_t(k));
            if (!held_object_edge(e)) continue;
            const std::array<size_t, 2> ev = get_edge_vids(e);
            const std::optional<Tuple> o = e.switch_face(*this);
            const uint8_t kind =
                !o                                                                         ? 8
                : (m_face_extra[fid].label == 1 || m_face_extra[o->fid(*this)].label == 1) ? 1
                                                                                           : 4;
            add({ev[0], ev[1]}, kind);
            add({ev[0]}, kind);
            add({ev[1]}, kind);
        }
        for (const size_t v : oriented_tri_vids(fid)) {
            const uint8_t kind =
                (m_vertex_extra[v].m_is_on_input ? 1 : 0) |
                (m_track_feature_points &&
                         m_feature_point_attribute[v].m_feature_point_id != NO_FEATURE_POINT
                     ? 2
                     : 0);
            if (kind) add({v}, kind);
        }
    }
    // Each simplex's chambers: its triangles, joined across the edges through it that are not
    // held. A held edge is its own chamber wall, so each of its triangles is a chamber.
    std::vector<std::map<size_t, int>> chamber_of(simplices.size());
    std::vector<std::vector<std::array<bool, 2>>> touched(simplices.size()); // band, non-band
    std::vector<size_t> first(simplices.size() + 1, 0); // element ids: first[i] + chamber
    for (size_t i = 0; i < simplices.size(); ++i) {
        const std::vector<size_t>& sigma = simplices[i];
        const std::vector<size_t> star = sigma.size() == 1
                                             ? get_one_ring_fids_for_vertex(sigma[0])
                                             : get_incident_fids_for_edge(sigma[0], sigma[1]);
        for (const size_t c0 : star) {
            if (chamber_of[i].count(c0)) continue;
            const int k = int(touched[i].size());
            touched[i].push_back({{false, false}});
            std::vector<size_t> stack{c0};
            chamber_of[i][c0] = k;
            while (!stack.empty()) {
                const size_t c = stack.back();
                stack.pop_back();
                const int l = m_face_extra[c].label;
                touched[i][size_t(k)][0] = touched[i][size_t(k)][0] || l == 2;
                touched[i][size_t(k)][1] = touched[i][size_t(k)][1] || l == 0;
                for (int j = 0; j < 3; ++j) {
                    const Tuple e = tuple_from_edge(c, size_t(j));
                    const std::array<size_t, 2> ev = get_edge_vids(e);
                    const bool through = std::all_of(sigma.begin(), sigma.end(), [&](size_t x) {
                        return ev[0] == x || ev[1] == x;
                    });
                    if (!through || held_object_edge(e)) continue;
                    const size_t n = e.switch_face(*this)->fid(*this);
                    if (chamber_of[i].emplace(n, k).second) stack.push_back(n);
                }
            }
        }
        first[i + 1] = first[i] + touched[i].size();
        if (vertex_touch && sigma.size() == 1) {
            const Vector2d& x = m_vertex_attribute[sigma[0]].m_posf;
            std::array<bool, 2>& t = (*vertex_touch)[{{x[0], x[1]}}];
            for (const auto& k : touched[i]) {
                t[0] = t[0] || k[0];
                t[1] = t[1] || k[1];
            }
        }
    }
    // Per label: union-find over the touched elements, each edge joined to its corners' elements
    // (the corner's chamber holding the edge's triangle); components and the Euler
    // characteristic.
    const auto compute = [&](const uint8_t mask) {
        std::array<int64_t, 4> out{};
        for (int label = 0; label < 2; ++label) {
            std::vector<size_t> root(first.back());
            std::iota(root.begin(), root.end(), size_t(0));
            const auto find = [&root](size_t x) {
                while (root[x] != x) x = root[x] = root[root[x]];
                return x;
            };
            int64_t chi = 0;
            for (size_t i = 0; i < simplices.size(); ++i) {
                const std::vector<size_t>& sigma = simplices[i];
                if (!(kinds[i] & mask)) continue;
                for (size_t k = 0; k < touched[i].size(); ++k) {
                    if (!touched[i][k][size_t(label)]) continue;
                    chi += sigma.size() == 2 ? -1 : 1;
                    if (sigma.size() == 1) continue;
                    size_t cell = chamber_of[i].begin()->first; // the chamber's triangle
                    for (const auto& [fid, kk] : chamber_of[i]) {
                        if (size_t(kk) == k) {
                            cell = fid;
                            break;
                        }
                    }
                    for (const size_t x : sigma) {
                        const auto it = index.find(std::vector<size_t>{x});
                        if (it == index.end()) continue;
                        const size_t r = it->second;
                        if (!(kinds[r] & mask)) continue;
                        const int kr = chamber_of[r].at(cell);
                        if (!touched[r][size_t(kr)][size_t(label)]) continue;
                        root[find(first[i] + k)] = find(first[r] + size_t(kr));
                    }
                }
            }
            int64_t b0 = 0;
            for (size_t i = 0; i < simplices.size(); ++i) {
                if (!(kinds[i] & mask)) continue;
                for (size_t k = 0; k < touched[i].size(); ++k) {
                    if (touched[i][k][size_t(label)] && find(first[i] + k) == first[i] + k) ++b0;
                }
            }
            out[size_t(2 * label)] = b0;
            out[size_t(2 * label + 1)] = chi;
            // DEBUG_held_topology, held points, the band: each component's vertex count and
            // centroid, logged where the topology changed.
            if (mask == 2 && label == 0 && m_offset_params.debug_held_topology) {
                std::map<size_t, std::pair<size_t, Vector2d>> comp;
                for (size_t i = 0; i < simplices.size(); ++i) {
                    if (!(kinds[i] & mask) || simplices[i].size() != 1) continue;
                    for (size_t k = 0; k < touched[i].size(); ++k) {
                        if (!touched[i][k][0]) continue;
                        auto& [n, c] =
                            comp.try_emplace(find(first[i] + k), 0, Vector2d::Zero()).first->second;
                        ++n;
                        c += m_vertex_attribute[simplices[i][0]].m_posf;
                    }
                }
                m_held_curve_counts = {{0, 0}};
                for (size_t i = 0; i < simplices.size(); ++i) {
                    if (kinds[i] & 2) ++m_held_curve_counts[simplices[i].size() == 1 ? 0 : 1];
                }
                m_held_curve_band_components.clear();
                for (const auto& [r, nc] : comp) {
                    m_held_curve_band_components.emplace_back(
                        nc.first,
                        nc.second / double(nc.first));
                }
            }
        }
        return out;
    };
    if (by_kind) {
        for (int k = 0; k < 4; ++k) (*by_kind)[size_t(k)] = compute(uint8_t(1 << k));
    }
    return compute(0xFF);
}

void TopoOffsetTriMesh::held_topology_check(const std::string& pass)
{
    // See the declaration; the 2D twin of TopoOffsetTetMesh::held_topology_check(), without its
    // stale order-2 tags: 2D has no order-2 edge tags (an order-2 simplex of a curve is a vertex,
    // anchored by hold_curves_and_points()) and its construction untracks nothing (3D's
    // untrack_covered_boundaries()), so no anchor can go stale.
    if (!m_offset_params.debug_held_topology) return;
    std::map<std::array<double, 2>, std::array<bool, 2>> touch;
    std::array<std::array<int64_t, 4>, 4> by_kind{};
    const std::array<int64_t, 4> s = held_topology_signature(&touch, &by_kind);
    if (!m_held_topology) {
        logger().info(
            "\t[held topology] {}: baseline -- the band touches the held objects in {} "
            "component(s), Euler characteristic {}; the non-band in {}, {}",
            pass,
            s[0],
            s[1],
            s[2],
            s[3]);
    } else if (s != *m_held_topology) {
        ++m_held_topology_changes;
        const std::array<int64_t, 4>& o = *m_held_topology;
        logger().warn(
            "\t[held topology] {} (turn {}): the held objects' contact topology changed -- band "
            "{} -> {} component(s), Euler characteristic {} -> {}; non-band {} -> {}, {} -> {} "
            "({} change(s) so far)",
            pass,
            m_round,
            o[0],
            s[0],
            o[1],
            s[1],
            o[2],
            s[2],
            o[3],
            s[3],
            m_held_topology_changes);
        // Where: the held vertices whose touch changed, matched by position (a collapse moves
        // one vertex onto another, so the survivor keeps a position the map had), and those
        // gone or new.
        size_t n_changed = 0, n_gone = 0, n_new = 0;
        std::string at;
        for (const auto& [x, t] : touch) {
            const auto it = m_held_vertex_touch.find(x);
            if (it == m_held_vertex_touch.end()) {
                ++n_new;
                continue;
            }
            if (it->second == t) continue;
            if (++n_changed <= 6) {
                at += fmt::format(
                    " ({:.4g}, {:.4g}) band {}->{} non-band {}->{};",
                    x[0],
                    x[1],
                    int(it->second[0]),
                    int(t[0]),
                    int(it->second[1]),
                    int(t[1]));
            }
        }
        for (const auto& [x, t] : m_held_vertex_touch) n_gone += touch.count(x) ? 0 : 1;
        logger().warn(
            "\t[held topology] {}: {} held vertices changed touch,{} {} gone, {} new",
            pass,
            n_changed,
            at,
            n_gone,
            n_new);
        const char* kind_names[4] = {
            "input complex",
            "held points",
            "region boundaries and the curve group",
            "box"};
        for (int k = 0; k < 4; ++k) {
            const auto& a = m_held_topology_by_kind[size_t(k)];
            const auto& b = by_kind[size_t(k)];
            if (a == b) continue;
            if (k == 1) {
                std::string where;
                for (const auto& [n, c] : m_held_curve_band_components) {
                    where += fmt::format(" {} at ({:.3g}, {:.3g});", n, c[0], c[1]);
                }
                logger().warn(
                    "\t[held topology] {}: the band's components on held points, vertices and "
                    "centroid:{} | held point vertices {} -> {}",
                    pass,
                    where,
                    m_held_curve_counts_before[0],
                    m_held_curve_counts[0]);
            }
            logger().warn(
                "\t[held topology] {}: {} -- band {} -> {} component(s), Euler characteristic {} "
                "-> {}; non-band {} -> {}, {} -> {}",
                pass,
                kind_names[k],
                a[0],
                b[0],
                a[1],
                b[1],
                a[2],
                b[2],
                a[3],
                b[3]);
        }
    }
    m_held_topology = s;
    m_held_topology_by_kind = by_kind;
    m_held_curve_counts_before = m_held_curve_counts;
    m_held_vertex_touch = std::move(touch);
}

void TopoOffsetTriMesh::check_level_set_reachable(const std::string& when)
{
    // See the declaration. As in 3D, word for word.
    if (m_unreachable_cells == 0) return;
    const std::string msg = fmt::format(
        "[held objects] {}: the level set is out of reach without the band touching a held object "
        "it did not touch as constructed -- the label clean-up keeps {} non-band cell(s) wholly "
        "inside the level set out of the band, the first at {} (throw_on_unreachable_level_set "
        "false warns and goes on)",
        when,
        m_unreachable_cells,
        m_unreachable_where);
    if (m_offset_params.throw_on_unreachable_level_set) log_and_throw_error("{}", msg);
    logger().warn("\t{}", msg);
}

void TopoOffsetTriMesh::label_cleanup(const std::string& where)
{
    // EXPERIMENTAL_label_cleanup (see the spec and the declaration): the 2D twin of
    // TopoOffsetTetMesh::label_cleanup(), whose measurements motivate it (cube_in, target 0.45,
    // 5% tolerance: thin non-band cells left beyond the level set, E 1.2e-7 above its optimum
    // against a settle bar of 3.5e-5, so nothing removed them). Not in the final pass
    // (m_freeze_front; 3D: a TetWild pass), which loop_group() does not run anyway.
    const std::string& mode = m_offset_params.label_cleanup;
    if (mode == "off" || m_freeze_front) return;
    const bool tips = mode == "one_sided_tips";
    const bool at_target = mode == "one_sided_at_target";
    // "energy_decrease" reads the "surface" rule's face term (3D: label_flip_surface_change()),
    // which 2D has not: refused. "one_sided_tips" and "one_sided_at_target" flip by the sign of a
    // triangle's band term b(t) (band_cell_term()), which tri_energy() carries since the energy
    // step of the 2D lock-step port (see wants_flip()).
    if (mode == "energy_decrease") {
        log_and_throw_error(
            "EXPERIMENTAL_label_cleanup \"energy_decrease\" in 2D: it flips by the \"surface\" "
            "rule's face term, which has no 2D port; 2D runs \"one_sided\", \"one_sided_tips\", "
            "\"one_sided_at_target\" or \"off\"");
    }
    // one_sided reads d through the exact integral's searches (init_offset_potential() builds it).
    if (!m_band_exact) {
        log_and_throw_error(
            "EXPERIMENTAL_label_cleanup \"one_sided\" reads the distance to the input through the "
            "exact band integral, which this run did not build: it needs "
            "EXPERIMENTAL_band_volume_rule \"corner_bound\", \"exact\", \"exact_swept\" or "
            "\"surface\", EXPERIMENTAL_settle_mark \"max\", or DEBUG_output");
    }
    const auto label_of = [this](const size_t fid) { return m_face_extra[fid].label; };

    // A triangle with a front edge: one shared with a triangle of the other of the two labels,
    // band (2) and non-band (0), neither of them the input complex (1).
    const auto has_front_face = [&](const size_t fid) {
        const int l = label_of(fid);
        if (l == 1) return false;
        for (int j = 0; j < 3; ++j) {
            const std::optional<Tuple> o = tuple_from_edge(fid, size_t(j)).switch_face(*this);
            if (o && label_of(o->fid(*this)) != 1 && label_of(o->fid(*this)) != l) return true;
        }
        return false;
    };

    // THE SIDE TEST: whether t's own corners put it on the other label's side. It reads nothing
    // the clean-up changes (no vertex moves), so a triangle's answer is the same in every pass
    // and a flipped triangle is never asked to flip back: it flips at most once, and the passes
    // end. As in 3D:
    //   one_sided: non-band -> band iff U(t) < delta, U = ExactBandIntegral2D::distance_bound(),
    //     the min over the input's segments and points P of the max of d_P over t's corners, an
    //     upper bound of d on t (d <= d_P, d_P convex), so d < delta on all of t. band -> non-band
    //     iff L(t) > delta, L = d(c) - max_k |v_k - c|, c the centroid: a lower bound of d on t (d
    //     is 1-Lipschitz, and the point of t farthest from c is a corner). L <= d <= U: the two
    //     exclude each other.
    //   one_sided_tips: one_sided, and also a TIP -- a triangle with two edges on the other label
    //     (3D: three faces) whose three corners are all within front_conv of the level set --
    //     flips iff that strictly lowers E. An eligible triangle is not plastic, so tri_energy()
    //     is cell_energy(): the AMIPS term (w / s^3) area A^3, from its corners and quality alone,
    //     plus band_cell_term() when it is band (the repulsion term is 0 in the loop). Every other
    //     triangle's energy reads its own corners and label. So the flip changes E by exactly
    //     +b(t) (to band) or -b(t) (to non-band): non-band -> band iff b(t) < 0, band -> non-band
    //     iff b(t) > 0. A triangle at MAX_ENERGY is MAX_ENERGY under either label, and no flip
    //     lowers E there. See 3D for the measurements; the passes still end, every flip of either
    //     kind agreeing with the sign of b(t).
    //   one_sided_at_target: one_sided, and also any triangle with a front edge (1 or 2 edges on
    //     the other label; 3D: 1 to 3 faces) whose three corners are all within front_conv of the
    //     level set, by the tip rule's sign of b(t). As in 3D.
    const auto wants_flip = [&](const size_t fid) {
        const std::array<size_t, 3> vs = oriented_tri_vids(fid);
        const bool band = face_is_offset_band(fid);
        // The triangle's piece's target distance, its front_conv (front_conv_rel of it) and its
        // integral (one entry: the run's and the whole complex's).
        const int piece = place_piece(vs.data(), 3);
        const double delta = region_delta(piece);
        const double front_conv = front_conv_of(piece);
        const ExactBandIntegral2D& band_exact = *band_exact_for(piece);
        std::array<Vector2d, 3> p;
        for (int k = 0; k < 3; ++k) p[size_t(k)] = m_vertex_attribute[vs[size_t(k)]].m_posf;
        if (!band && band_exact.distance_bound(p) < delta) return true;
        if (band) {
            const Vector2d c = (p[0] + p[1] + p[2]) / 3.;
            double r = 0.;
            for (const Vector2d& q : p) r = std::max(r, (q - c).norm());
            if (band_exact.distance(c) - r > delta) return true;
        }
        if (!tips && !at_target) return false;
        const int other = band ? 0 : 2;
        int edges = 0;
        for (int j = 0; j < 3; ++j) {
            const std::optional<Tuple> o = tuple_from_edge(fid, size_t(j)).switch_face(*this);
            if (o && label_of(o->fid(*this)) == other) ++edges;
        }
        if (tips ? edges != 2 : (edges < 1 || edges > 2)) return false;
        for (const Vector2d& q : p) {
            if (!(std::abs(band_exact.distance(q) - delta) <= front_conv)) return false;
        }
        if (get_quality(fid) >= MAX_ENERGY) return false;
        const double b = band_cell_term(vs);
        return band ? b > 0. : b < 0.;
    };

    // ELIGIBILITY: two labels only, so that the topology test sees everything t touches. Every
    // triangle sharing a vertex with t is band or non-band (no input-complex triangle) and not
    // plastic (its AMIPS term would read the label). And no corner of t is where a held boundary
    // -- a tag boundary, the curve group or the box (vertex_is_on_region()) -- and the front
    // already meet (m_is_on_offset): only the construction makes such contact, where its edges
    // between band and non-band are both held and front and carry the offset class, which the
    // held objects' test cannot see. Every other corner, on the input complex, a held point, a
    // held boundary or the box, the held objects' test decides. As in 3D (2026-11-10).
    const auto eligible = [&](const size_t fid) {
        for (const size_t v : oriented_tri_vids(fid)) {
            if (vertex_is_on_region(v) && m_vertex_extra[v].m_is_on_offset) return false;
            for (const size_t s : get_one_ring_fids_for_vertex(v)) {
                const int l = label_of(s);
                if ((l != 0 && l != 2) || face_is_plastic(s)) return false;
            }
        }
        return true;
    };

    // THE TOPOLOGY TEST, 3D's one dimension down. f = the edges of t shared with a triangle of
    // the other label, the one t would take:
    //   f = 0: refused (t would start a new piece, or the front would pinch at a vertex);
    //   f = 1: refused if the corner opposite that edge is in a triangle of the other label;
    //   f = 2: allowed (the third edge becomes front);
    //   f = 3: refused (t is a whole piece by itself; the flip deletes it).
    // An allowed flip has the other label meet t in exactly its f edges, an arc, so taking t in
    // keeps that label's topology. 3D tests, for f = 1 and 2, the simplex of t off the other
    // label's faces (the opposite corner, the edge joining the two opposite corners); in 2D only
    // f = 1 has one, and f = 2 is 3D's f = 3. t's own label, which loses t, meets t in its 3 - f
    // edges and also, for f = 2, in the common corner of the two other-label edges when that is
    // in another triangle of t's label. Not tested: on a manifold front it cannot happen (a front
    // vertex where t's label meets twice); such a flip throws (`odd`), and the component check
    // below catches a split. Returns f.
    const auto topology = [&](const size_t fid, bool& allow, bool& odd) {
        const std::array<size_t, 3> vs = oriented_tri_vids(fid);
        const int self = label_of(fid), other = self == 2 ? 0 : 2;
        std::vector<size_t> opp_other, opp_self; // the corner opposite each edge, by its far side
        for (int j = 0; j < 3; ++j) {
            const Tuple e = tuple_from_edge(fid, size_t(j));
            const std::array<size_t, 2> ev = get_edge_vids(e);
            size_t opp = vs[0];
            for (const size_t v : vs) {
                if (v != ev[0] && v != ev[1]) opp = v;
            }
            const std::optional<Tuple> o = e.switch_face(*this);
            (o && label_of(o->fid(*this)) == other ? opp_other : opp_self).push_back(opp);
        }
        const auto at_vertex = [&](const size_t v, const int l) {
            for (const size_t s : get_one_ring_fids_for_vertex(v)) {
                if (s != fid && label_of(s) == l) return true;
            }
            return false;
        };
        const int f = int(opp_other.size());
        allow = false;
        odd = false;
        if (f == 1) allow = !at_vertex(opp_other[0], other);
        if (f == 2) {
            allow = true;
            odd = at_vertex(opp_self[0], self);
        }
        return f;
    };

    // THE HELD OBJECTS' TEST, 3D's (2026-11-10) one dimension down. The invariant: the
    // optimization keeps the topology the construction gave the band, the non-band AND every
    // held object -- the input complex, the held points (anchors), the region boundaries and the
    // curve group, the box -- as if the front moved by a deformation of the domain that maps each
    // held object onto itself. The topology test above keeps the first two; this keeps the rest.
    // A held simplex sigma of t's closure has CHAMBERS: the triangles around it, joined across
    // the edges through sigma that are not held (a held edge's two sides are two chambers). Per
    // chamber, sigma is touched by a label when the chamber has a triangle of it. Switching t from
    // label s to o changes only the chamber holding t, for the held simplices of t's closure X:
    // o touches each of them after, and s leaves those whose chamber had no other triangle of s.
    // Allowed iff both changes are simple:
    //   the set o touches grows by X attached along A, the part of X o touched before: allowed
    //     iff A -> X is a homotopy equivalence (per component of X, A meets it, connected, with
    //     the same Euler characteristic) -- a held point o did not touch, alone in X, fails;
    //   the set s touches loses Y, the part of X only t gave it: allowed iff cl(Y) \ Y -> cl(Y)
    //     is one -- a held point whose last triangle of s is t fails.
    // X lies on t's boundary, a circle (3D: a sphere), so its parts are graphs, and for graphs
    // components and Euler characteristic decide the homotopy type exactly. So a held point's
    // chamber status never changes, a crossing of the front with a held curve or boundary moves
    // along it but is never born or lost, and the front never newly touches a held object: it
    // stops a layer of triangles short of one it did not touch as constructed. MEASURED in 3D
    // before it (held point q 0.25 from p, target 0.3): turn 1's clean-up took 6 of q's 30 cells
    // into the band, so q joined the front; under exact_swept turn 11 took q's last non-band cell
    // and the band enclosed q.
    const auto held_face = [&](const Tuple& e) { return held_object_edge(e); };
    // The tag the flipped triangle takes, of the label it joins. The band: the one tag set of the
    // band triangles at t's corners (two pieces' bands, of two tag sets, refuse). The non-band:
    // that of t's non-band edge neighbours across edges that are not held (a held edge's far side
    // is another region); they must agree, and there must be one. As in 3D.
    const auto joined_tag = [&](const size_t fid, CellTag& tag) {
        const int o = label_of(fid) == 2 ? 0 : 2;
        const CellTag* found = nullptr;
        if (o == 2) {
            for (const size_t v : oriented_tri_vids(fid)) {
                for (const size_t s : get_one_ring_fids_for_vertex(v)) {
                    if (label_of(s) != 2) continue;
                    const CellTag& tg = m_face_attribute[s].tags;
                    if (found && *found != tg) return false;
                    found = &tg;
                }
            }
        } else {
            for (int k = 0; k < 3; ++k) {
                const Tuple e = tuple_from_edge(fid, size_t(k));
                if (held_face(e)) continue;
                const size_t n = e.switch_face(*this)->fid(*this);
                if (label_of(n) != o) continue;
                const CellTag& tg = m_face_attribute[n].tags;
                if (found && *found != tg) return false;
                found = &tg;
            }
        }
        if (!found) return false;
        tag = *found;
        return true;
    };
    const auto edge_has = [](const std::array<size_t, 2>& ev, const size_t x) {
        return ev[0] == x || ev[1] == x;
    };
    // The chamber of sigma holding t: whether it has a triangle of label o, and one of label s
    // other than t.
    const auto chamber = [&](const size_t fid,
                             const std::vector<size_t>& sigma,
                             const int s,
                             const int o,
                             bool& o_in,
                             bool& s_other) {
        o_in = s_other = false;
        std::vector<size_t> seen{fid}, stack{fid};
        while (!stack.empty()) {
            const size_t c = stack.back();
            stack.pop_back();
            if (c != fid) {
                o_in = o_in || label_of(c) == o;
                s_other = s_other || label_of(c) == s;
            }
            for (int k = 0; k < 3; ++k) {
                const Tuple e = tuple_from_edge(c, size_t(k));
                const std::array<size_t, 2> ev = get_edge_vids(e);
                if (!std::all_of(sigma.begin(), sigma.end(), [&](size_t x) {
                        return edge_has(ev, x);
                    })) {
                    continue;
                }
                if (held_face(e)) continue;
                const size_t n = e.switch_face(*this)->fid(*this);
                if (std::find(seen.begin(), seen.end(), n) != seen.end()) continue;
                seen.push_back(n);
                stack.push_back(n);
            }
        }
    };
    const auto held_allows = [&](const size_t fid) {
        const std::array<size_t, 3> vs = oriented_tri_vids(fid);
        const int s = label_of(fid), o = s == 2 ? 0 : 2;
        // X: the held simplices of t's closure, each as the local indices (0..2) of its corners
        // -- closed, as a held edge's corners are held.
        std::vector<std::vector<int>> X;
        for (int i = 0; i < 3; ++i) {
            if (held_object_vertex(vs[size_t(i)])) X.push_back({i});
        }
        for (int k = 0; k < 3; ++k) {
            const Tuple e = tuple_from_edge(fid, size_t(k));
            if (!held_face(e)) continue;
            const std::array<size_t, 2> ev = get_edge_vids(e);
            std::vector<int> c;
            for (int i = 0; i < 3; ++i) {
                if (edge_has(ev, vs[size_t(i)])) c.push_back(i);
            }
            X.push_back(c);
        }
        if (X.empty()) return true;
        std::vector<char> touched(X.size()), loses(X.size());
        for (size_t x = 0; x < X.size(); ++x) {
            std::vector<size_t> sigma;
            for (const int i : X[x]) sigma.push_back(vs[size_t(i)]);
            bool o_in = false, s_other = false;
            chamber(fid, sigma, s, o, o_in, s_other);
            touched[x] = o_in;
            loses[x] = !s_other;
        }
        // Whether the closed part P of the closed set Q (flags over X) includes into Q as a
        // homotopy equivalence: per component of Q, P meets it, is connected there, and has the
        // same Euler characteristic.
        const auto equivalent = [&](const std::vector<char>& inQ, const std::vector<char>& inP) {
            const auto components = [&](const std::vector<char>& in) {
                std::array<int, 3> r{{0, 1, 2}};
                const auto root = [&r](int a) {
                    while (r[size_t(a)] != a) a = r[size_t(a)];
                    return a;
                };
                for (size_t x = 0; x < X.size(); ++x) {
                    if (in[x] && X[x].size() == 2) r[size_t(root(X[x][0]))] = root(X[x][1]);
                }
                for (int a = 0; a < 3; ++a) r[size_t(a)] = root(a);
                return r;
            };
            const std::array<int, 3> rq = components(inQ), rp = components(inP);
            for (int c = 0; c < 3; ++c) {
                bool is_root = false;
                for (size_t x = 0; x < X.size(); ++x) {
                    is_root = is_root || (inQ[x] && X[x].size() == 1 && rq[size_t(X[x][0])] == c);
                }
                if (!is_root) continue;
                int chi_q = 0, chi_p = 0, p_root = -1;
                bool p_connected = true;
                for (size_t x = 0; x < X.size(); ++x) {
                    if (!inQ[x] || rq[size_t(X[x][0])] != c) continue;
                    const int sign = X[x].size() == 2 ? -1 : 1;
                    chi_q += sign;
                    if (!inP[x]) continue;
                    chi_p += sign;
                    if (X[x].size() == 1) {
                        const int pr = rp[size_t(X[x][0])];
                        if (p_root < 0) p_root = pr;
                        p_connected = p_connected && pr == p_root;
                    }
                }
                if (p_root < 0 || !p_connected || chi_p != chi_q) return false;
            }
            return true;
        };
        const std::vector<char> all(X.size(), 1);
        if (std::find(touched.begin(), touched.end(), 0) != touched.end() &&
            !equivalent(all, touched)) {
            return false;
        }
        if (std::find(loses.begin(), loses.end(), 1) != loses.end()) {
            std::vector<char> closure(X.size(), 0), rest(X.size(), 0);
            for (size_t y = 0; y < X.size(); ++y) {
                if (!loses[y]) continue;
                for (size_t x = 0; x < X.size(); ++x) {
                    const bool face_of_y = std::all_of(X[x].begin(), X[x].end(), [&](int i) {
                        return std::find(X[y].begin(), X[y].end(), i) != X[y].end();
                    });
                    if (face_of_y) closure[x] = 1;
                }
            }
            for (size_t x = 0; x < X.size(); ++x) rest[x] = closure[x] && !loses[x];
            if (!equivalent(closure, rest)) return false;
        }
        return true;
    };

    // The one-sided side test's non-band half (wants_flip()): d < the piece's target on all of
    // t, so the band reaches the level set only through t. A triangle held objects keep out, so,
    // makes the level set out of reach (check_level_set_reachable()). As in 3D.
    const auto inside_level_set = [&](const size_t fid) {
        const std::array<size_t, 3> vs = oriented_tri_vids(fid);
        const int piece = place_piece(vs.data(), 3);
        const std::shared_ptr<const ExactBandIntegral2D>& be = band_exact_for(piece);
        if (!be) return false;
        std::array<Vector2d, 3> p;
        for (int k = 0; k < 3; ++k) p[size_t(k)] = m_vertex_attribute[vs[size_t(k)]].m_posf;
        return be->distance_bound(p) < region_delta(piece);
    };
    // A held vertex's kind and position, for the message. 2D tracks the curve group as it
    // tracks a region boundary (init_surfaces_and_boundaries()), and nothing carries which of
    // the two an edge is through the operations, so they are one kind here.
    const auto describe_held = [&](const size_t v) -> std::string {
        const bool anchored = m_track_feature_points &&
                              m_feature_point_attribute[v].m_feature_point_id != NO_FEATURE_POINT;
        const char* kind = m_vertex_extra[v].m_is_on_input                ? "the input complex"
                           : anchored                                     ? "an anchored point"
                           : !m_vertex_attribute[v].on_bbox_faces.empty() ? "the box"
                           : m_vertex_extra[v].m_is_on_region ? "a region boundary or a held curve"
                                                              : "the mesh boundary";
        const Vector2d& x = m_vertex_attribute[v].m_posf;
        return fmt::format("{} at ({:.4g}, {:.4g})", kind, x[0], x[1]);
    };
    // How deep inside the level set t's held corners lie: the max over them of the piece's target
    // minus d, and the deepest one's kind and position. Within front_conv of the level set a front
    // passing outside the held object is already within the bar, so only a held object deeper
    // than that makes the level set out of reach. As in 3D (MEASURED there: double_sphere with
    // tag_1 held, a refused cell's held corner 0.001 inside against front_conv 0.005 threw at
    // turn 5 before the depth rule; the held point, 0.05 inside against 0.03, throws).
    const auto held_depth = [&](const size_t fid) {
        const std::array<size_t, 3> vs = oriented_tri_vids(fid);
        const int piece = place_piece(vs.data(), 3);
        const std::shared_ptr<const ExactBandIntegral2D>& be = band_exact_for(piece);
        double depth = -std::numeric_limits<double>::infinity();
        size_t deepest = vs[0];
        for (const size_t v : vs) {
            if (!be || !held_object_vertex(v)) continue;
            const double dv = region_delta(piece) - be->distance(m_vertex_attribute[v].m_posf);
            if (dv > depth) {
                depth = dv;
                deepest = v;
            }
        }
        return std::pair<double, std::string>(
            depth,
            fmt::format("{}, {:.4g} inside the level set", describe_held(deepest), depth));
    };

    // A flip, and everything derived from t's label, which lives on t's closure: the closed labels
    // of its edges and corners (2 where some triangle holding it is band, else 0; a simplex of
    // the input complex keeps its 1), the front class of its three edges -- the only edges whose
    // two sides change -- and the front flags of its three corners, the only vertices on those
    // edges. The base's union flag m_is_on_surface is raised for a corner that joins the front, as
    // label_offset_boundary() raises it; it is never cleared. As in 3D; 2D's vertex order is read
    // live (TriMesh::get_order_of_vertex()), so there is no stored order to re-derive.
    const auto any_band = [this](const std::vector<size_t>& fids) {
        return std::any_of(fids.begin(), fids.end(), [this](const size_t s) {
            return face_is_offset_band(s);
        });
    };
    const auto flip = [&](const size_t fid, const CellTag& tag) {
        // A held edge of t becoming front would be contact with a held object the held objects'
        // test forbids, and its class rewritten below: a bug, so it throws.
        std::array<bool, 3> front_before{};
        for (int j = 0; j < 3; ++j) {
            front_before[size_t(j)] = edge_is_offset_surface_live(tuple_from_edge(fid, size_t(j)));
        }
        m_face_extra[fid].label = label_of(fid) == 2 ? 0 : 2;
        m_face_attribute[fid].tags = tag;
        for (int j = 0; j < 3; ++j) {
            const Tuple e = tuple_from_edge(fid, size_t(j));
            const size_t eid = e.eid(*this);
            if (!front_before[size_t(j)] && held_face(e) && edge_is_offset_surface_live(e)) {
                log_and_throw_error(
                    "[label cleanup] {}: triangle {} makes held edge {} front, which the held "
                    "objects' test forbids",
                    where,
                    fid,
                    eid);
            }
            int& le = m_edge_extra[eid].label;
            if (le != 1) {
                const std::array<size_t, 2> ev = get_edge_vids(e);
                le = any_band(get_incident_fids_for_edge(ev[0], ev[1])) ? 2 : 0;
            }
            if (edge_is_offset_surface_live(e)) {
                m_edge_attribute[eid].m_is_surface_fs = true;
                m_edge_attribute[eid].m_surface_class = OFFSET_SURFACE_CLASS;
            } else if (m_edge_attribute[eid].m_surface_class == OFFSET_SURFACE_CLASS) {
                m_edge_attribute[eid].m_is_surface_fs = false;
                m_edge_attribute[eid].m_surface_class = 0; // left at 0 off the surface
            }
        }
        for (const size_t v : oriented_tri_vids(fid)) {
            int& lv = m_vertex_extra[v].label;
            if (lv != 1) lv = any_band(get_one_ring_fids_for_vertex(v)) ? 2 : 0;
            refresh_offset_membership(v);
            if (m_vertex_extra[v].m_is_on_offset) m_vertex_attribute[v].m_is_on_surface = true;
        }
    };

    // THE INVARIANT'S MEASURES: connected components of one label's triangles under edge
    // adjacency, and front vertices on more than two front edges (where the label changes more
    // than twice around the vertex; 3D: front edges on more than two front faces).
    const auto label_components = [this](const int l) {
        std::vector<size_t> parent(tri_capacity());
        std::iota(parent.begin(), parent.end(), size_t(0));
        const auto root = [&parent](size_t x) {
            while (parent[x] != x) {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            return x;
        };
        size_t n = 0;
        for (const Tuple& t : get_faces()) {
            const size_t fid = t.fid(*this);
            if (m_face_extra[fid].label != l) continue;
            ++n;
            for (int j = 0; j < 3; ++j) {
                const std::optional<Tuple> o = tuple_from_edge(fid, size_t(j)).switch_face(*this);
                if (!o || m_face_extra[o->fid(*this)].label != l) continue;
                const size_t a = root(fid), b = root(o->fid(*this));
                if (a != b) {
                    parent[a] = b;
                    --n;
                }
            }
        }
        return n;
    };
    const auto nonmanifold_front_vertices = [this]() {
        std::map<size_t, int> n;
        for (const std::array<size_t, 2>& e : offset_surface_edges()) { // sorted pairs
            ++n[e[0]];
            ++n[e[1]];
        }
        size_t c = 0;
        for (const auto& vk : n) c += vk.second > 2 ? 1 : 0;
        return c;
    };
    const double e0 = global_energy();
    const size_t band0 = label_components(2), non0 = label_components(0);
    const size_t nm0 = nonmanifold_front_vertices();
    std::vector<char> flipped(tri_capacity(), 0);
    size_t to_band = 0, to_non = 0, n_odd = 0;
    int passes = 0;
    // The last pass's verdicts: that pass flips nothing, so they are what the clean-up leaves.
    size_t n_cand = 0, n_want = 0, n_skip = 0, n_refused_held = 0, n_refused_tag = 0;
    size_t n_unreachable = 0; // of n_refused_held: non-band and wholly inside the level set
    std::string unreachable_where;
    std::array<size_t, 4> n_refused{}; // by f
    for (;;) {
        ++passes;
        std::vector<size_t> cands;
        for (const Tuple& t : get_faces()) {
            if (has_front_face(t.fid(*this))) cands.push_back(t.fid(*this));
        }
        std::sort(cands.begin(), cands.end());
        n_cand = cands.size();
        n_want = 0;
        n_skip = 0;
        n_refused_held = 0;
        n_refused_tag = 0;
        n_unreachable = 0;
        unreachable_where.clear();
        n_refused.fill(0);
        size_t n_now = 0;
        // The tests are pure reads, so the order they are asked in decides only what is counted:
        // the side test first, so that the counts are of triangles on the other side.
        for (const size_t fid : cands) {
            if (!wants_flip(fid)) continue;
            ++n_want;
            if (!eligible(fid)) {
                ++n_skip;
                continue;
            }
            bool allow = false, odd = false;
            const int f = topology(fid, allow, odd);
            if (!allow) {
                ++n_refused[size_t(f)];
                continue;
            }
            if (!held_allows(fid)) {
                ++n_refused_held;
                if (!face_is_offset_band(fid) && inside_level_set(fid)) {
                    const auto [depth, at] = held_depth(fid);
                    const std::array<size_t, 3> vs = oriented_tri_vids(fid);
                    if (depth > front_conv_of(place_piece(vs.data(), 3)) && n_unreachable++ == 0) {
                        unreachable_where = at;
                    }
                }
                continue;
            }
            CellTag tag;
            if (!joined_tag(fid, tag)) {
                ++n_refused_tag;
                continue;
            }
            if (flipped[fid]) {
                log_and_throw_error(
                    "[label cleanup] {}: triangle {} flips a second time; its side test reads its "
                    "own corners alone, which the clean-up does not move",
                    where,
                    fid);
            }
            // The topology test reads the label the triangle joins only; on a manifold front the
            // old label cannot meet the triangle again (as in 3D: throw rather than count it).
            if (odd) {
                log_and_throw_error(
                    "[label cleanup] {}: triangle {} (f = {}) passes the topology test, but its "
                    "old "
                    "label meets it again at a front vertex, which the test does not read",
                    where,
                    fid,
                    f);
            }
            flipped[fid] = 1;
            ++(face_is_offset_band(fid) ? to_non : to_band);
            n_odd += odd ? 1 : 0;
            flip(fid, tag);
            ++n_now;
        }
        if (n_now == 0) break;
    }
    m_unreachable_cells = n_unreachable;
    m_unreachable_where = unreachable_where;
    double e1 = e0;
    size_t band1 = band0, non1 = non0, nm1 = nm0;
    if (to_band + to_non > 0) {
        // What reads the front beyond the flipped triangles' closure: the band-region map,
        // rebuilt as each turn rebuilds it; DEBUG_crossings' snapshot, taken here so that the
        // smoothing pass is compared with the front it starts from.
        assign_band_regions(/*log=*/false);
        if (m_offset_params.debug_crossings && m_round > 0) {
            crossing_snapshot("label cleanup", true, true);
        }
        check_offset_membership("label cleanup"); // perform_sanity_checks only
        e1 = global_energy();
        band1 = label_components(2);
        non1 = label_components(0);
        nm1 = nonmanifold_front_vertices();
    }
    logger().info(
        "\t[label cleanup] {}: {} | {} pass(es), flipped non-band -> band {}, band -> non-band {} "
        "({} of them with f = 2 where the old label meets the triangle again, at a front vertex, "
        "not tested) | left after the last pass: {} triangles with a front edge, {} on the other "
        "side by the side test, of which {} skipped for eligibility, refused by topology f=0 {}, "
        "f=1 {}, f=3 {}, by the held objects {} ({} wholly inside the level set), for no single "
        "tag {} | E {:.9g} -> {:.9g} | components (edge adjacency) band {} -> {}, non-band {} -> "
        "{} | non-manifold front vertices (on more than two front edges) {} -> {}",
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
        n_refused[3],
        n_refused_held,
        n_unreachable,
        n_refused_tag,
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
            "the old label meeting the triangle again, which the test does not read)",
            where,
            band0,
            band1,
            non0,
            non1,
            n_odd);
    }
}

void TopoOffsetTriMesh::loop_group(const std::array<int, 4>& ops, const std::string& where)
{
    // See the declaration; TopoOffsetTetMesh::loop_group(), the same passes in the same order.
    if (!m_offset_params.front_pocket_split) {
        local_operations(ops);
        return;
    }
    // EXPERIMENTAL_front_pocket_split: the same passes, one call for the operation passes and one
    // per smoothing pass. Besides the splits this moves the rounding sweep from after the k-th
    // smoothing pass to after each, and adds each call's lines. As in 3D.
    if (ops[0] > 0 || ops[1] > 0 || ops[2] > 0) local_operations({{ops[0], ops[1], ops[2], 0}});
    // EXPERIMENTAL_label_cleanup: after the operation passes (none in pre_smooth), before the
    // pocket splits of the first smoothing pass. A flip with f = 1 edge on the other label (3D: 1
    // or 2 faces) leaves the triangle with all three corners on the front -- a pinned cell, which
    // those splits handle -- and a triangle flipped with f = 2 (3D's f = 3) was a pinned cell
    // before its flip; the smoothing pass then moves the new front in the same turn. Under
    // interleaved_smoothing this is once per group, else once per turn. As in 3D.
    if (m_offset_params.label_cleanup != "off" && (ops[0] > 0 || ops[1] > 0 || ops[2] > 0)) {
        label_cleanup(where);
        held_topology_check("label_cleanup");
        // DEBUG_output: the clean-up's own frame, so its flips are seen apart from the pocket
        // splits and the smoothing pass that follow it. As in 3D.
        if (m_offset_params.debug_output) {
            const std::string name = m_debug_pass_name;
            m_debug_pass_name = "label_cleanup";
            write_smoothing_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
            m_debug_pass_name = name;
        }
    }
    for (int p = 0; p < ops[3]; ++p) {
        front_pocket_split(fmt::format("{}, before smoothing pass {}", where, p + 1));
        held_topology_check("pocket_split");
        local_operations({{0, 0, 0, 1}});
    }
}

void TopoOffsetTriMesh::front_pocket_split(const std::string& where)
{
    // See the declaration; TopoOffsetTetMesh::front_pocket_split() one dimension down, whose
    // measurements motivate it (cube_in at target 0.3: 174 folds, 152 on a pinned cell, without
    // it; resolved at turn 3 with it). loop_group() calls this only with the key on; the key is
    // tested here too so that the slot alone is a no-op with it off (the unit test drives the
    // slot directly). Not in the final pass (m_freeze_front; 3D: a TetWild pass), which
    // loop_group() does not run anyway.
    if (!m_offset_params.front_pocket_split || m_freeze_front) return;
    const auto len2 = [this](const std::array<size_t, 2>& e) {
        return (m_vertex_attribute[e[0]].m_posf - m_vertex_attribute[e[1]].m_posf).squaredNorm();
    };
    const auto on_front = [this](const size_t v) { return m_vertex_extra[v].m_is_on_offset; };
    const auto longest_first = [&](const std::array<size_t, 2>& a, const std::array<size_t, 2>& b) {
        const double la = len2(a), lb = len2(b);
        return la != lb ? la > lb : a < b;
    };
    // A pinned cell: its corners sorted (it names the triangle across the round's splits), and
    // the edges to split it on, longest first (ties by vertex ids).
    struct Pinned
    {
        std::array<size_t, 3> vids;
        std::vector<std::array<size_t, 2>> edges;
    };
    // One scan of the live triangles. `no_edge`: pinned cells with all three edges on the front,
    // which have no edge to split; `band`, when given: band triangles with all three corners on
    // the front. 3D's count of cells with all six edges on the offset surface has no twin (see
    // the declaration).
    const auto scan = [&](size_t& no_edge, size_t* band) {
        std::vector<Pinned> out;
        no_edge = 0;
        for (const Tuple& t : get_faces()) {
            const size_t fid = t.fid(*this);
            const std::array<size_t, 3> vs = oriented_tri_vids(fid);
            if (!std::all_of(vs.begin(), vs.end(), on_front)) continue;
            if (face_is_input_complex(fid)) continue;
            // A band triangle with all three corners on the front is the mirror case: at the
            // offset term's optimum its corners lie on the level set too. Split the same way (an
            // edge that is not a front edge is inside the band, so its midpoint is off the
            // front); counted apart in the log. As in 3D.
            if (face_is_offset_band(fid) && band) ++*band;
            Pinned p;
            p.vids = vs;
            std::sort(p.vids.begin(), p.vids.end());
            for (size_t j = 0; j < 3; ++j) {
                const Tuple e = tuple_from_edge(fid, j);
                if (edge_is_offset_surface_live(e)) continue;
                std::array<size_t, 2> ev = get_edge_vids(e);
                if (ev[0] > ev[1]) std::swap(ev[0], ev[1]);
                p.edges.push_back(ev);
            }
            if (p.edges.empty()) {
                // All three edges on the front: a one-triangle component of its label. A split
                // would leave its boundary, every corner of it on the front, as it is, so it is
                // counted only, as 3D counts its cell with all four faces on the offset surface.
                ++no_edge;
                continue;
            }
            std::sort(p.edges.begin(), p.edges.end(), longest_first);
            out.push_back(std::move(p));
        }
        return out;
    };
    // Whether the triangle on these sorted corners is still live: an earlier split of the round
    // may have split one of its edges, and so the triangle, already.
    const auto alive = [this](const std::array<size_t, 3>& s) {
        for (const size_t fid : get_incident_fids_for_edge(s[0], s[1])) {
            std::array<size_t, 3> vs = oriented_tri_vids(fid);
            std::sort(vs.begin(), vs.end());
            if (vs == s) return true;
        }
        return false;
    };
    // EXPERIMENTAL_front_spanned_split: the spanned edges (see the declaration), split each round
    // before the pinned cells, lowest dimension first. 3D's spanned faces, listed when their
    // three edges are on the offset surface, are 2D's spanned edges themselves.
    const auto off_input = [this](const std::vector<size_t>& fids) {
        return !fids.empty() && std::none_of(fids.begin(), fids.end(), [this](const size_t f) {
            return face_is_input_complex(f);
        });
    };
    const auto scan_spanned = [&](std::vector<std::array<size_t, 2>>& edges) {
        edges.clear();
        if (!m_offset_params.front_spanned_split) return;
        for (const Tuple& e : get_edges()) {
            const size_t a = e.vid(*this), b = e.switch_vertex(*this).vid(*this);
            if (!on_front(a) || !on_front(b) || edge_is_offset_surface_live(e)) continue;
            if (!off_input(get_incident_fids_for_edge(a, b))) continue;
            edges.push_back({{std::min(a, b), std::max(a, b)}});
        }
    };
    // The loop's own split (split_edge(): labels, tags, sizing and rests as in the split pass).
    // 3D skips its longest-edge order here (m_pocket_splitting); 2D's split has no such order.
    const auto split_one = [this](const std::array<size_t, 2>& e) {
        const Tuple t = std::get<0>(tuple_from_edge(e));
        reserve_free_slots(cell_slot_bound(t), 1); // serial: grow rather than refuse
        std::vector<Tuple> new_tris;
        return split_edge(t, new_tris);
    };
    // EXPERIMENTAL_settle_mark "vertex_split": the chords the last settled turn listed
    // (settle_refinement_turn()), split here, before the pinned cells, so that the smoothing pass
    // that follows moves the new front vertices in the same turn. Split at the end of the settled
    // turn instead, the next turn's collapse pass undid them before any smoothing had moved them:
    // MEASURED in 3D (t100033 inward at 0.5 delta_1, 10%, 1 thread). A chord this turn's
    // operation passes removed (split, or collapsed with an end, whose face list the collapse
    // clears) is skipped; the next settled turn tests it again. The ids hold, as in 3D: the mesh
    // is consolidated before the settle test, and a vertex made since takes a new slot (SlotPool
    // hands out slots past the live ones; only consolidation renumbers).
    if (!m_settle_split_edges.empty()) {
        size_t n_listed = m_settle_split_edges.size(), n_done = 0, n_gone = 0, n_no = 0;
        for (const std::array<size_t, 2>& e : m_settle_split_edges) {
            if (get_incident_fids_for_edge(e[0], e[1]).empty()) {
                ++n_gone;
                continue;
            }
            split_one(e) ? ++n_done : ++n_no;
        }
        m_settle_split_edges.clear();
        logger().info(
            "\t[settle split] {}: of the {} chords the last settled turn listed, split {}, gone "
            "(removed by this turn's operation passes) {}, refused {}",
            where,
            n_listed,
            n_done,
            n_gone,
            n_no);
    }
    size_t n_band = 0, no_edge = 0;
    std::vector<Pinned> pinned = scan(no_edge, &n_band);
    std::vector<std::array<size_t, 2>> sp_edges;
    scan_spanned(sp_edges);
    const size_t found = pinned.size() + no_edge, found_no_edge = no_edge;
    const size_t found_edges = sp_edges.size();
    size_t n_split = 0, n_refused = 0, n_split_edges = 0;
    int rounds = 0;
    bool cap = false;
    while (!pinned.empty() || !sp_edges.empty()) {
        if (rounds == kPocketMaxRounds) {
            cap = true;
            break;
        }
        ++rounds;
        size_t split_now = 0;
        for (const std::array<size_t, 2>& e : sp_edges) {
            if (get_incident_fids_for_edge(e[0], e[1]).empty()) continue; // split already
            if (split_one(e)) {
                ++split_now;
                ++n_split_edges;
            } else {
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
        // Every triangle a split of an edge off the front makes has the off-front midpoint as a
        // corner, so a rescan finds the cells and edges whose splits were all refused; with none
        // committed it finds the same ones.
        if (split_now == 0) break;
        pinned = scan(no_edge, nullptr);
        scan_spanned(sp_edges);
    }
    if (cap) {
        logger().warn(
            "\t[pocket split] {}: the cap of {} rounds stopped the rescans with {} pinned cells "
            "left that have an edge to split",
            where,
            kPocketMaxRounds,
            pinned.size());
    }
    logger().info(
        "\t[pocket split] {}: pinned cells (outside the input complex, all three corners on the "
        "front) {}, of which {} with all three edges on the front | edges split {} in {} "
        "round(s), split attempts refused {} | left: {} with all three edges on the front (not "
        "split), {} with an edge off it (every split refused, or the cap) | of the pinned cells "
        "found, band cells {}{}",
        where,
        found,
        found_no_edge,
        n_split,
        rounds,
        n_refused,
        no_edge,
        pinned.size(),
        n_band,
        m_offset_params.front_spanned_split
            ? fmt::format(
                  " | spanned edges (both ends on the front, not a front edge) found {}, split "
                  "{}; left {}",
                  found_edges,
                  n_split_edges,
                  sp_edges.size())
            : std::string());
}

bool TopoOffsetTriMesh::optimize_offset_loop(
    const bool final_stage,
    const bool refine,
    const std::string& label,
    const int max_turns,
    const bool turned_free)
{
    // One loop: TriWild's operation groups (split / collapse / swap, each followed by smoothing)
    // with the front placed by the offset objective inside the smoothing passes. No offset tube
    // holds the front in the loop, neither the operations nor the smoothing; only the frozen-front
    // final pass is held to one (containment_for()). As in 3D.
    const int a_iters = std::max(1, m_offset_params.max_iterations);
    check_no_vertex_on_both_surfaces("construction");
    log_region_edge_mask_health("construction");
    audit_surface_containment("construction");
    // Each loop judges its own target (the init_optimize loop, growth_mode's stages). As in 3D.
    m_unreachable_cells = 0;
    needle_scan("after construction, before the loop");
    assign_band_regions();
    m_front_gradient_reference = front_gradient_linf();
    logger().info(
        "\tLOOP: TriWild's operation groups with the front placed inside their "
        "smoothing passes | front energy-gradient reference {:.6g} | ONE criterion: the RMS "
        "relative error over a stencil_order {} stencil ({} points per chord) against front_conv "
        "{:.6g} ({:.6g} x the bbox diagonal)",
        m_front_gradient_reference,
        m_offset_params.stencil_order,
        stencil_points_per_edge(),
        m_offset_params.front_conv,
        m_offset_params.front_conv_rel);
    logger().info(
        "\t[offset envelope] the loop's operations and smoothing do not hold the offset boundary "
        "to an envelope; the final pass does (eps {:.6g})",
        m_offset_params.offset_envelope);
    // max_turns > 0: growth_mode "smart"'s attempt (smart_growth_attempts) in place of max_rounds.
    // As in 3D.
    const int budget = max_turns > 0 ? max_turns : std::max(1, m_offset_params.max_rounds);
    const char* budget_name = max_turns > 0 ? "smart_growth_attempts" : "max_rounds";
    // One turn is TriWild's operation groups, run here rather than through mesh_improvement() so
    // the released-boundary tube can be refreshed after every group. What mesh_improvement()
    // adds and is left out here on purpose is its stall response, which refines around the
    // worst elements: a moving front stretches cells by design.
    // k is the fixed count when adaptive_smoothing is off; on, each group smooths until the
    // front and the background settle (smooth_group_to_convergence()).
    // interleaved_smoothing true is TriWild's shape of a turn: three groups, each one operation
    // pass followed by k smoothing passes. With it false a turn is ONE group -- split, collapse
    // and swap back to back -- followed by one smoothing block of num_smoothing_passes (or
    // adaptive). See the 3D twin for the measurement behind the switch.
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
                  /* adaptive_smoothing */ false ? std::string("adaptive smoothing")
                                                 : fmt::format("{} smoothing pass(es)", k))
            : fmt::format(
                  "COMBINED -- split, collapse, swap back to back, then {} (interleaved_smoothing "
                  "false)",
                  /* adaptive_smoothing */ false ? std::string("adaptive smoothing")
                                                 : fmt::format("{} smoothing pass(es)", k)));
    partition_mesh_morton();
    if (m_offset_params.pre_smooth) {
        // One smoothing block on the constructed mesh before turn 1's split pass: the same
        // block every operation group is followed by, with the same bookkeeping around it
        // (plastic rests stamped before, the released tube refreshed after). Frames are r0S*.
        m_round = 0;
        refresh_released_envelope();
        stamp_plastic_rests();
        logger().info(
            "\t[pre_smooth] one smoothing block before turn 1: {}",
            /* adaptive_smoothing */ false ? std::string("adaptive smoothing")
                                           : fmt::format("{} interleaved smoothing pass(es)", k));
        if (/* adaptive_smoothing */ false) {
            smooth_group_to_convergence("pre_smooth");
        } else {
            // Through loop_group(), as in 3D: the pocket splits precede each of its smoothing
            // passes too.
            loop_group({{0, 0, 0, k}}, "pre_smooth");
        }
        refresh_released_envelope();
    }
    // EXPERIMENTAL_settle_refinement: the settle test refines and exits (settle_refinement_turn());
    // the energy criterion's refinable sets, its sizing-floor warnings and its exit do not act.
    // settle_e_prev is the offset energy at the end of the previous turn, for turn 1 before its
    // first group. As in 3D.
    const bool settle = m_offset_params.settle_refinement;
    double settle_e_prev = 0.;
    for (int it = 0; it < budget; ++it) {
        m_round = it + 1;
        m_iterations_used = it + 1;
        refresh_released_envelope();
        for (size_t gi = 0; gi < groups.size(); ++gi) {
            stamp_plastic_rests(); // plastic: each group resists only its own increment
            // E before and after each group, the descent check (a collapse or swap group may not
            // raise it; a split group may). As in 3D.
            const double energy_before_group = global_energy();
            if (settle && it == 0 && gi == 0) settle_e_prev = settle_metric();
            const int energy_c0 = iter_cnt_collapse_energy_reject.load();
            const int energy_s0 = iter_cnt_swap_energy_reject.load();
            const int energy_p0 = iter_cnt_split_energy_reject.load();
            const int fold_c0 = m_turned_reject_collapse.load(),
                      fold_m0 = m_turned_reject_smooth.load();
            if (gi == 1) needle_scan("collapse pass");
            if (!interleaved) needle_scan("combined ops pass");
            log_energy_step(it + 1, "start");
            if (/* adaptive_smoothing */ false) {
                // The group's operations alone, then its smoothing pass by pass until the front
                // and the background have settled -- see smooth_group_to_convergence().
                local_operations({{groups[gi][0], groups[gi][1], groups[gi][2], 0}});
                smooth_group_to_convergence(group_names[gi]);
            } else {
                // The operation passes, EXPERIMENTAL_label_cleanup, and the pocket splits before
                // each smoothing pass. As in 3D.
                loop_group(groups[gi], fmt::format("turn {} {} group", it + 1, group_names[gi]));
            }
            refresh_released_envelope(); // the smoothing in this group moved the boundaries
            // Per group, so a containment violation is attributed to the pass that made it
            // rather than found at the end of the run. Same gate as the shared sanity check.
            if (m_params.perform_sanity_checks) {
                audit_surface_containment(fmt::format("turn {} after {}", it + 1, group_names[gi]));
            }
            const double energy_after_group = global_energy();
            logger().info(
                "\t[band volume energy] turn {} {} group: E {:.10g} -> {:.10g} ({:+.4g}) | refused "
                "by the energy rule over the faces they change: {} collapse(s) for raising E, {} "
                "swap(s) and {} split(s) (EXPERIMENTAL_split_energy_gate) for not strictly "
                "lowering it ({} / {} / {} in the run so far)",
                it + 1,
                group_names[gi],
                energy_before_group,
                energy_after_group,
                energy_after_group - energy_before_group,
                iter_cnt_collapse_energy_reject.load() - energy_c0,
                iter_cnt_swap_energy_reject.load() - energy_s0,
                iter_cnt_split_energy_reject.load() - energy_p0,
                iter_cnt_collapse_energy_reject.load(),
                iter_cnt_swap_energy_reject.load(),
                iter_cnt_split_energy_reject.load());
            if (turned_reject_active()) {
                // As in 3D, without its flips (turned_reject_active()).
                logger().info(
                    "\t[op reject] turn {} {} group: refused for raising the offset edges turned "
                    "from the field (growth_mode individual_op_reject): {} collapses, {} "
                    "smoothing moves",
                    it + 1,
                    group_names[gi],
                    m_turned_reject_collapse.load() - fold_c0,
                    m_turned_reject_smooth.load() - fold_m0);
            }
        }
        consolidate_mesh();
        m_cross_valid = false; // DEBUG_crossings: consolidation renumbered the vertices
        assign_band_regions();
        const double amips = std::get<0>(optimization_quality_stats());
        const double bar = optimization_stop_metric();
        const EnergyCriterion ec = energy_criterion();
        const Vector2d wx = ec.worst_vid != static_cast<size_t>(-1)
                                ? m_vertex_attribute[ec.worst_vid].m_posf
                                : Vector2d::Zero();
        if (ec.ring_exit) {
            // front_measure "vertex_ring": the ring measure is the exit test; the chord measure
            // and the vertex measure are the same numbers the face mode prints, as diagnostics.
            const Vector2d rx = ec.worst_ring_vid != static_cast<size_t>(-1)
                                    ? m_vertex_attribute[ec.worst_ring_vid].m_posf
                                    : Vector2d::Zero();
            logger().info(
                "======== turn {} / {}: max AMIPS {:.4} (stop {:.4}) | {} max "
                "{:.4}x the bar (avg {:.4}x) (worst v{} at ({:.4}, {:.4})) over {} "
                "front vertices, {} rings unmeasurable, {} unmeasurable in all (the exit test) | "
                "diagnostic: chords max {:.4}x (avg {:.4}x), {} chords over the bar of {}; front "
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
                ec.n_rings,
                ec.n_rings_unmeasurable,
                ec.n_unmeasurable,
                ec.max_edge,
                ec.avg_edge(),
                ec.n_edges_over,
                ec.n_edges,
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
                "max {:.4}x the bar (avg {:.4}x) (worst v{} at ({:.4}, {:.4})) "
                "(diagnostic), chords max {:.4}x (avg {:.4}x), {} unmeasurable (the exit test) | "
                "{} vertices, {} chords | chords over the bar: {}, of which {} with both ends "
                "placed (worst {:.4}x, midpoint ({:.4}, {:.4})) | refinable chords {} (at the "
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
                ec.max_edge,
                ec.avg_edge(),
                ec.n_unmeasurable,
                ec.n_vertices,
                ec.n_edges,
                ec.n_edges_over,
                ec.n_edges_over_placed,
                ec.max_edge_placed,
                ec.worst_placed_mid.x(),
                ec.worst_placed_mid.y(),
                ec.refinable.size(),
                ec.n_at_floor);
        }
        // Chords over the bar with both ends at the sizing floor block the exit (they are over
        // the bar) and no refinement will ever take them, so a run that keeps them never
        // converges: a warning, every turn they exist. The turn line's "at the sizing floor"
        // count also holds chords whose ends are above the floor and which the split pass will
        // shorten, so the same line says which, at info when none is at the floor.
        //
        // Under the ring measure the same line names the VERTICES over the bar at the floor,
        // always a warning: the vertex form of the halving has no chord rule, so every vertex it
        // cannot take is one nothing will ever take. As in 3D.
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
        // The 3D loop logs four more lines here -- [swap reject], [ops accounting], [split order]
        // and [flip funnel] -- from instrumentation that lives in the 3D engine (TetOptimizerMesh)
        // and in 3D-only operations (surface flips, longest-edge split order); the 2D engine has
        // none of it, so 2D has no such lines.
        //
        // perform_sanity_checks only: m_is_on_offset against the labels, whole mesh. Free when
        // the key is off, which is the default.
        check_offset_membership(fmt::format("turn {}", it + 1).c_str());
        if (!settle && !refine &&
            (!ec.refinable.empty() || (ec.ring_exit && ec.n_rings_over > 0))) {
            logger().info(
                "\t[resolution] turn {}: {} -- no refinement ({} front vertex(es) / {} chord(s) "
                "over the bar left as they are)",
                it + 1,
                label,
                ec.n_rings_over,
                ec.refinable.size());
        }
        if (!settle && refine && !ec.refinable.empty()) {
            // Refinement is the halving, and only the halving: every refinable chord has the
            // sizing scalar at its ends halved.
            const size_t n = refine_front_by_halving(ec.refinable);
            logger().info(
                "\t[resolution] turn {}: {} front chord(s) {} whose RMS "
                "relative error over {} stencil point(s) is over the bar (worst placed {:.4}x, "
                "midpoint ({:.4}, {:.4})) -> sizing scalar halved at {} vertices",
                it + 1,
                ec.refinable.size(),
                "(placed or not)",
                stencil_points_per_edge(),
                ec.max_edge_placed,
                ec.worst_placed_mid.x(),
                ec.worst_placed_mid.y(),
                n);
        }
        if (!settle && refine && ec.ring_exit && ec.n_rings_over > 0) {
            // front_measure "vertex_ring": the halving takes each vertex over the bar, that
            // vertex alone. refinable is empty in this mode, so the chord line above is silent.
            const size_t n = refine_front_by_halving(ec.refinable_vertices);
            logger().info(
                "\t[resolution] turn {}: {} front vertex(es) whose {} (the RMS of the chord "
                "measures of its incident front chords, each chord weighted equally) is over the "
                "bar, {} of them at the sizing floor (worst {:.4}x) -> sizing scalar halved at {} "
                "vertices",
                it + 1,
                ec.n_rings_over,
                ec.ring_name(),
                ec.n_rings_at_floor,
                ec.max_ring,
                n);
        }
        // EXPERIMENTAL_settle_refinement: the settle test, the chord test and the halving; true
        // when the loop may exit, which then replaces EnergyCriterion::converged() below.
        const bool settle_exit = settle && settle_refinement_turn(it + 1, settle_e_prev, refine);
        // At a settled turn the last clean-up's refusals are no longer the front on its way.
        if (settle && m_turn_settled) check_level_set_reachable(fmt::format("turn {}", it + 1));
        // The turn's "end" frame is written HERE, after the refinement, not before it: it is the
        // turn's final state, so what it carries is the sizing field the halving just lowered.
        if (m_offset_params.debug_output) {
            write_smoothing_debug_output(fmt::format("end_{}S", it + 1));
        }
        // Termination: every front chord's measure within the bar and nothing unmeasurable
        // (EnergyCriterion::converged()) -- then quality with the front frozen (below). Under
        // front_measure "vertex_ring" the tested measure is every front vertex's ring measure
        // instead. The loop exits on the FIRST turn that meets the criterion, as TriWild's loop
        // breaks the moment its max energy is under stop_energy. As in 3D.
        //
        // EXPERIMENTAL_settle_refinement: the settle exit (settled, nothing marked) in place of
        // converged(); the criterion's measures are then diagnostics. As in 3D.
        bool resolved = settle ? settle_exit : ec.converged();
        // EXPERIMENTAL_exit_no_folds: no exit while a fold remains (fold_test(), through
        // count_folds()): a vertex with two offset edges whose normals out of the band are within
        // foldover_angle of opposite, one of the two more than foldover_angle from the field's
        // normal at its midpoint (front_edge_field_angle()); an edge that cannot be measured
        // counts as turned, so the exit is never released on what was not measured. Two opposite
        // edges that both follow the field are a ridge vertex, the front following a corner of
        // the level set sharper than foldover_angle, and do not hold the exit. As in 3D, whose
        // measurements motivate both rules (cube_in at 0.45: a run exited with a flat sheet of
        // inner cells and 5 real folds; the 10-26 batch: holding for every opposite pair held 4
        // runs to the turn cap on ridges of the level set). The counts are logged every turn, the
        // key on or off.
        {
            const FoldCount fc = count_folds();
            logger().info(
                "\t[exit folds] turn {}: folds {}{}",
                it + 1,
                fc.folds,
                resolved && fc.folds > 0
                    ? (m_offset_params.exit_no_folds
                           ? " -- the exit test passes; held for the folds"
                           : " -- the exit test passes; EXPERIMENTAL_exit_no_folds is off")
                    : "");
            if (fc.ridge_vertices > 0) {
                logger().info(
                    "\t[exit ridge vertices] turn {}: {} vertices with two opposite offset edges "
                    "both within foldover_angle {} degrees of the field (not folds); their edges "
                    "reach {:.4} degrees",
                    it + 1,
                    fc.ridge_vertices,
                    m_offset_params.foldover_angle,
                    fc.ridge_max_angle);
            }
            if (m_offset_params.exit_no_folds && fc.folds > 0) resolved = false;
        }
        // turned_free (growth_mode "smart"): a converged loop with an offset edge turned from the
        // field has failed, judged here, before the verdict and the final pass, which it does not
        // get. As in 3D.
        if (resolved && turned_free) {
            const TurnedEdgeCensus fc = turned_edge_census();
            if (fc.n_turned > 0) {
                logger().warn(
                    "[{}] converged at target_distance {:.6g} after {} turn(s), but {} of {} "
                    "offset edges are turned from the field (more than foldover_angle {} degrees "
                    "from grad d; the worst {:.4g} degrees, at ({:.4}, {:.4}))",
                    label,
                    m_offset_params.target_distance,
                    it + 1,
                    fc.n_turned,
                    fc.n_edges,
                    m_offset_params.foldover_angle,
                    fc.worst,
                    fc.worst_at[0],
                    fc.worst_at[1]);
                refresh_released_envelope();
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
                settle ? "settled, no front chord over the bar (EXPERIMENTAL_settle_refinement); "
                         "diagnostic: "
                       : "",
                ec.ring_exit ? std::string(ec.ring_name()) : std::string("chords"),
                (ec.ring_exit ? ec.max_ring : ec.max_edge) / ec.bar,
                ec.n_vertices,
                amips);
            refresh_released_envelope();
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
                    "chord is more than front_conv from the level set "
                    "(EXPERIMENTAL_settle_refinement); front chords max {:.4}x with {} over the "
                    "bar (diagnostic); front vertices max {:.4}x (diagnostic); max AMIPS {:.4} "
                    "against stop {:.4}",
                    it + 1,
                    ec.max_edge / ec.bar,
                    ec.n_edges_over,
                    ec.max_vertex / ec.bar,
                    amips,
                    bar);
            } else if (ec.ring_exit) {
                logger().info(
                    "The front is resolved after {} iteration(s): every front "
                    "vertex's {} within the bar (rings max {:.4}x), nothing unmeasurable; front "
                    "chords max {:.4}x with {} over the bar (diagnostic); front vertices max "
                    "{:.4}x (diagnostic); max AMIPS {:.4} against stop {:.4}",
                    it + 1,
                    ec.ring_name(),
                    ec.max_ring / ec.bar,
                    ec.max_edge / ec.bar,
                    ec.n_edges_over,
                    ec.max_vertex / ec.bar,
                    amips,
                    bar);
            } else {
                logger().info(
                    "The front is resolved after {} iteration(s): every front chord "
                    "within the bar (chords max {:.4}x), nothing unmeasurable; front vertices max "
                    "{:.4}x (diagnostic); max AMIPS {:.4} against stop {:.4}",
                    it + 1,
                    ec.max_edge / ec.bar,
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
                // The front's tube, built from the front as placement left it and held for the
                // entire pass; the region boundaries stay held in the tubes and masks the load
                // built (m_tag_envelopes, never rebuilt), as 3D's final pass adds the front's
                // envelope alone (build_final_envelopes(), 2b8de047ba). Equilateral AMIPS alone:
                // the plastic vertex path and the rest-shape term are both off. Until 2026-11-11
                // the region tubes and the vertex masks were rebuilt here from the tags and
                // on_curve, which no operation carries: MEASURED on the mesh the loop leaves (p
                // offset to 0.3, a held curve-group curve 0.5 from it, 3 to 8 turns, 5 runs), the
                // rebuild put 1 to 4 of the curve's 7 to 9 tracked edges in its tube and took the
                // curve bit off 1 to 8 of its 8 to 10 vertices; after 3 turns 4 of its 8 edges
                // were held by nothing.
                rebuild_offset_envelope();
                m_freeze_front = true;
                const bool plastic_was = m_plastic_active;
                m_plastic_active = false;
                mesh_improvement(a_iters);
                m_plastic_active = plastic_was;
                m_freeze_front = false;
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
                    write_smoothing_debug_output(fmt::format("end_{}F", it + 2));
                }
            }
            refresh_released_envelope();
            return true;
        }
    }
    check_level_set_reachable("the loop's end");
    logger().warn(
        "The loop did not converge in {} turns ({}){}",
        budget,
        budget_name,
        label.empty() ? std::string()
                      : fmt::format(
                            " -- {} at target_distance {:.6g}",
                            label,
                            m_offset_params.target_distance));
    log_front_profile(energy_criterion().worst_vid);
    return false;
}

TopoOffsetTriMesh::MeshSnapshot TopoOffsetTriMesh::take_snapshot()
{
    // See the declaration.
    if (m_track_feature_points) {
        log_and_throw_error(
            "take_snapshot: feature-point tracking is on, and the image does not hold its "
            "collection");
    }
    consolidate_mesh();
    MeshSnapshot s;
    s.n_vertices = vert_capacity();
    const size_t n_faces = tri_capacity();
    s.faces.resize(n_faces);
    for (size_t f = 0; f < n_faces; ++f) s.faces[f] = oriented_tri_vids(f);
    // The live slots of a collection, [0, n): consolidate_mesh() packed them there.
    const auto head = [](const auto& c, const size_t n) {
        return std::vector(c.m_attributes.begin(), c.m_attributes.begin() + std::ptrdiff_t(n));
    };
    s.vertex = head(m_vertex_attribute, s.n_vertices);
    s.vertex_extra = head(m_vertex_extra, s.n_vertices);
    s.edge = head(m_edge_attribute, 3 * n_faces);
    s.edge_extra = head(m_edge_extra, 3 * n_faces);
    s.face = head(m_face_attribute, n_faces);
    s.face_extra = head(m_face_extra, n_faces);
    restore_snapshot(s);
    return s;
}

void TopoOffsetTriMesh::restore_snapshot(const MeshSnapshot& s)
{
    // See the declaration. init_with_isolated_vertices() leaves every collection cleared and
    // sized to the new capacity, at least the live count, so the copies fit.
    init_with_isolated_vertices(s.n_vertices, s.faces);
    const auto put = [](auto& c, const auto& v) {
        std::copy(v.begin(), v.end(), c.m_attributes.begin());
    };
    put(m_vertex_attribute, s.vertex);
    put(m_vertex_extra, s.vertex_extra);
    put(m_edge_attribute, s.edge);
    put(m_edge_extra, s.edge_extra);
    put(m_face_attribute, s.face);
    put(m_face_extra, s.face_extra);
    assign_band_regions(/*log=*/false);
}

double TopoOffsetTriMesh::growth_stage_target(
    const double d0,
    const double delta,
    const int k,
    const int n)
{
    return k >= n ? delta : d0 + double(k) * (delta - d0) / double(n);
}

void TopoOffsetTriMesh::linear_growth()
{
    // See the declaration and the spec doc (growth_mode "linear"). Between stages only
    // target_distance changes, and what init_offset_potential() builds on it: the field, the
    // per-piece fields, the exact integral. Every other reader of target_distance (the band term,
    // the settle test, the measures, the front smoother) asks for it when it runs, and no pass is
    // running here. As in 3D.
    const double delta = m_offset_params.target_distance;
    const double d0 = m_growth_start;
    // growth_mode "individual_op_reject": d_0, then target_distance, nothing between.
    const int n = m_offset_params.growth_mode == "linear" ? m_offset_params.linear_growth_steps : 1;
    const auto set_target = [&](const double d) {
        m_offset_params.target_distance = d;
        init_offset_potential();
    };
    // Several entries of offsets (m_growth_starts): stage k at each entry's own stage target, from
    // its d_0 to its target_distance (region_delta() reads them, m_offset_levels); the run's
    // target the smallest. One entry: target_distance itself.
    const bool several = !m_growth_starts.empty();
    const auto set_stage = [&](const int k) {
        if (!several) {
            set_target(growth_stage_target(d0, delta, k, n));
            return;
        }
        m_offset_levels.resize(m_growth_starts.size());
        for (size_t e = 0; e < m_growth_starts.size(); ++e) {
            m_offset_levels[e] = growth_stage_target(
                m_growth_starts[e],
                m_offset_params.offsets[e].target_distance,
                k,
                n);
            logger().info(
                "\t[incremental growth] stage {} / {}: offsets[{}] at {:.6g} (its target_distance "
                "{:.6g})",
                k,
                n,
                e,
                m_offset_levels[e],
                m_offset_params.offsets[e].target_distance);
        }
        set_target(*std::min_element(m_offset_levels.begin(), m_offset_levels.end()));
    };
    const auto set_final = [&]() {
        m_offset_levels.clear();
        set_target(delta);
    };
    std::string done; // the stages before the last, for the last one's header
    for (int k = 0; k < n; ++k) {
        set_stage(k);
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
            set_final();
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
    set_final();
    logger().info(
        "======== [incremental growth] stage {} / {}: target_distance {:.6g}, the ordinary loop "
        "(converged before it: {}) ========",
        n,
        n,
        delta,
        done);
    optimize_offset_loop();
}

void TopoOffsetTriMesh::smart_growth()
{
    // See the declaration and the spec doc (growth_mode "smart"). As in linear_growth(), only
    // target_distance changes between loops, and what init_offset_potential() builds on it. As in
    // 3D.
    const auto t0 = std::chrono::steady_clock::now();
    const double delta = m_offset_params.target_distance;
    const double d0 = m_growth_start;
    const int attempts = m_offset_params.smart_growth_attempts;
    const auto set_target = [&](const double d) {
        m_offset_params.target_distance = d;
        init_offset_potential();
    };
    // Several entries of offsets (m_growth_starts): one schedule per entry, from its d_0 to its
    // target_distance, advanced in lockstep with sched below (the same successes and failures,
    // so the same j and level): each loop at each entry's own target (m_offset_levels), the run's
    // the smallest. One entry: sched alone.
    const bool several = !m_growth_starts.empty();
    std::vector<SmartGrowthSchedule> entry_sched;
    for (size_t e = 0; e < m_growth_starts.size(); ++e) {
        entry_sched.push_back(
            {m_growth_starts[e],
             m_offset_params.offsets[e].target_distance,
             m_offset_params.smart_growth_max_fails});
    }
    const auto set_targets = [&](const double d, const bool level0) {
        if (!several) {
            set_target(d);
            return;
        }
        m_offset_levels.resize(entry_sched.size());
        for (size_t e = 0; e < entry_sched.size(); ++e) {
            const SmartGrowthSchedule& es = entry_sched[e];
            m_offset_levels[e] = level0 ? m_growth_starts[e] : es.j == 0 ? es.delta : es.target();
            logger().info(
                "\t[smart growth] offsets[{}] at {:.6g} (its target_distance {:.6g})",
                e,
                m_offset_levels[e],
                es.delta);
        }
        set_target(*std::min_element(m_offset_levels.begin(), m_offset_levels.end()));
    };
    const auto report_entries = [&](const bool ok) {
        for (SmartGrowthSchedule& es : entry_sched) es.report(ok);
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
    set_targets(d0, /*level0=*/true);
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
            "{} turns",
            d0,
            m_offset_params.max_rounds);
    }
    m_converged = false;
    m_energy_verdict.reset();
    std::string history = fmt::format("level 0 at {:.6g} in {} turn(s)", d0, m_iterations_used);
    MeshSnapshot snap = take_snapshot();
    logger().info(
        "[smart growth] level 0 converged at target_distance {:.6g}; the mesh is stored (#V {}, "
        "#F {})",
        d0,
        get_vertices().size(),
        get_faces().size());
    SmartGrowthSchedule sched{d0, delta, m_offset_params.smart_growth_max_fails};
    int n_reverted = 0;
    for (;;) {
        // j == 0 is target_distance itself, set exactly (not by the formula).
        const bool at_delta = sched.j == 0;
        const double t = at_delta ? delta : sched.target();
        const int level = sched.level, attempt = sched.j + 1;
        const std::string label = fmt::format("smart growth level {} attempt {}", level, attempt);
        set_targets(t, /*level0=*/false);
        m_frame_prefix = fmt::format("s{}a{}", level, attempt);
        logger().info(
            "======== [smart growth] level {} attempt {}: target_distance {:.6g} ({:.4g}x "
            "target_distance {:.6g}), from the converged {:.6g}{}; up to smart_growth_attempts {} "
            "turns, converged and free of turned edges to succeed ========",
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
            /*turned_free=*/true);
        const int turns = m_iterations_used;
        const double base = sched.base;
        report_entries(ok);
        switch (sched.report(ok)) {
        case SmartGrowthSchedule::Next::Done:
            m_frame_prefix.clear();
            m_offset_levels.clear(); // each entry at its target_distance
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
                "mesh is stored (#V {}, #F {}), the next attempt is at target_distance {:.6g}",
                level,
                t,
                attempt,
                get_vertices().size(),
                get_faces().size(),
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
                "converged in {} turn(s), or with turned edges; see above): the mesh is back at "
                "the converged {:.6g} (#V {}, #F {}){}",
                level,
                attempt,
                t,
                turns,
                base,
                get_vertices().size(),
                get_faces().size(),
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

void TopoOffsetTriMesh::optimize_offset(const std::filesystem::path& output_file)
{
    logger().info("Optimizing offset (2D)...");

    // From here on every edge split is an optimization split, run by the shared engine. The
    // marching-triangles placement mode requires one endpoint inside the offset and one outside,
    // which does not hold for an arbitrary long edge. split_edge_before/after dispatch on this.
    m_edge_split_mode = EdgeSplitMode::Optimization;

    // label the offset boundary, and with it the vertices the optimization places
    logger().info("\tLabel offset edges...");
    label_offset_boundary();
    // The baseline the propagation has to hold from here on: label_offset_boundary() marks the
    // flag from the construction labels; this asks the live label test the same question.
    check_offset_membership("construction");
    held_topology_check("construction"); // DEBUG_held_topology's baseline

    // deform_others: from here on, other input regions deform instead of being envelope-held.
    // deform_others was removed on the 3D-only branch spenbert/offsets-cleanup (2026-10-13); its
    // default, false, is hard-wired here until the 2D port.
    if (/* deform_others */ false) {
        release_deformable_regions();
        if (!m_deform_tags.empty()) {
            m_plastic_active = true;
            stamp_plastic_rests();
        }
    }

    // The released-boundary tube, from the boundaries as released. The offset envelope is only
    // built for the final pass.
    refresh_released_envelope();

    // The front as constructed must already be inside the potential's support.
    check_offset_within_support("Offset as constructed");

    logger().info(
        "\tOffset criterion: |grad (Phi - c)^2 . n| <= (front_conv / target_distance) {} x "
        "max|grad (Phi - c)^2 . n| over the band AS CONSTRUCTED, with n the unit normal from "
        "the offset surface's own normal (Voronoi-weighted at vertices, the edge's own inside "
        "an edge). Measured over every band vertex and {} stencil point(s) "
        "per band edge; the reference is reported next, before the loop starts.",
        m_offset_params.front_conv_frac(),
        stencil_points_per_edge());

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

    // Unconditional: write_vtu() must not be the only consolidate here. No frame here: nothing
    // changes the mesh between this point and the "construction" frame the optimization writes
    // first, so one would duplicate the other.
    consolidate_mesh();

    iter_cnt_split = 0;
    iter_cnt_collapse = 0;
    iter_cnt_collapse_offset_removed = 0;
    iter_cnt_swap = 0;
    iter_cnt_collapse_energy_reject = 0;
    iter_cnt_swap_energy_reject = 0;
    iter_cnt_split_energy_reject = 0;
    m_smooth_trace.reset();
    optimization_metrics.clear();

    // Frame 0 is the mesh as constructed, before the optimization touches it.
    if (m_params.debug_output) {
        m_debug_pass_name = "construction";
        write_smoothing_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
    }

    if (m_init_optimize) {
        // init_optimize (see marching_tris()): the loop first runs to convergence under
        // stencil_order without refinement, the sizing field left as it is, and without the final
        // pass or the verdict; then the ordinary loop below, with its refinement.
        // target_distance is not touched. As in 3D.
        m_frame_prefix = "i";
        // EXPERIMENTAL_init_optimize_stencil_order >= 0: this loop alone runs at that order
        // (smoothing energy, energy rules, exit test), restored for the loop below. Switched
        // here, between the loops, where no pass is running.
        const int stencil_order = m_offset_params.stencil_order;
        if (m_offset_params.init_optimize_stencil_order >= 0) {
            m_offset_params.stencil_order = m_offset_params.init_optimize_stencil_order;
        }
        logger().info(
            "======== [init_optimize] target_distance {:.6g}, stencil_order {}, no "
            "refinement ========",
            m_offset_params.target_distance,
            m_offset_params.stencil_order);
        optimize_offset_loop(/*final_stage=*/false, /*refine=*/false, "init_optimize");
        m_offset_params.stencil_order = stencil_order;
        m_frame_prefix.clear();
        m_converged = false;
        m_energy_verdict.reset();
        logger().info(
            "======== [init_optimize] done; the loop at target_distance {:.6g}, "
            "stencil_order {}, with refinement ========",
            m_offset_params.target_distance,
            m_offset_params.stencil_order);
    }
    // growth_mode (see marching_tris()): "linear" in stages, the last of them the ordinary loop at
    // target_distance; "smart" in attempts, the one that reaches target_distance its final stage.
    // As in 3D.
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
    logger().info(
        "energy rule: {} collapses refused for raising E over the faces they change, {} swaps "
        "and {} splits (EXPERIMENTAL_split_energy_gate) for not strictly lowering it",
        iter_cnt_collapse_energy_reject.load(),
        iter_cnt_swap_energy_reject.load(),
        iter_cnt_split_energy_reject.load());

    // Final metrics and the convergence verdict, one entry for the whole run.
    assign_band_regions();
    const auto [max_dist, avg_dist] = compute_distance_deviation();
    const DistanceSplit r = residual_split();
    const GradientSplit g = gradient_split();
    logger().info(
        "placement gradient (at band vertices): max {} (avg {}) | in-edge diagnostic {} ({} edge "
        "samples) | {} reachable, {} pinned (max {}), {} skipped ({} unrounded, {} inverted ring)",
        g.max_reachable,
        g.avg_reachable,
        g.max_in_edge,
        g.n_edge_samples,
        g.n_reachable,
        g.n_pinned,
        g.max_pinned,
        g.n_skipped_unrounded + g.n_skipped_inverted,
        g.n_skipped_unrounded,
        g.n_skipped_inverted);
    logger().info(
        "phi residual (diagnostic, absolute model units): max {} (avg {}) vs front_conv {} | at "
        "vertices {}, inside edges {} | {} samples, {} pinned vertices || euclid dist err: max {} "
        "| avg {}",
        r.max_reachable,
        r.avg_reachable,
        m_offset_params.front_conv,
        r.max_at_vertex,
        r.max_in_edge,
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
          g.max_in_edge}});
    log_worst_dist_vertex();

    bool front_ok = false;
    std::string floor_fact; // quoted again by throw_on_nonconvergence below
    std::string quality; // and so is this
    // As in 3D: the quality half of the verdict is judged only once the front is resolved, and
    // printing the unjudged defaults read "final quality ok: max AMIPS 0" on every run stopped at
    // max_rounds.
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
        // fact (about refinable sets that do not act) are diagnostics. As in 3D.
        const bool settle = m_offset_params.settle_refinement;
        front_ok = settle ? m_energy_verdict.has_value() : ec.converged();
        m_converged = front_ok && m_quality_converged;
        floor_fact = settle ? std::string() : ec.sizing_floor_fact();
        quality = quality_fact(front_ok);
        if (settle) {
            logger().info(
                "[settle] verdict: front {} -- tested: the loop's settle exit (a settled turn with "
                "no front chord more than front_conv from the level set); the measures on the next "
                "line are diagnostics",
                front_ok ? "resolved" : "NOT resolved");
        }
        if (ec.ring_exit) {
            // front_measure "vertex_ring": the ring measure is what was tested; the chord and
            // vertex measures are the diagnostics.
            logger().log(
                m_converged ? spdlog::level::info : spdlog::level::warn,
                "{}{}: front {} -- tested (every front vertex's {} within the bar, nothing "
                "unmeasurable): {} rings max {:.4}x the bar (avg {:.4}x), {} rings "
                "unmeasurable, {} unmeasurable in all | diagnostic, not tested: {} chords max "
                "{:.4}x the bar (avg {:.4}x), {} chords over the bar; {} front vertices max {:.4}x "
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
                ec.n_edges,
                ec.max_edge,
                ec.avg_edge(),
                ec.n_edges_over,
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
                "{}{}: front {} -- tested (every chord within the bar, nothing unmeasurable): {} "
                "chords max {:.4}x the bar (avg {:.4}x), {} unmeasurable | diagnostic, not "
                "tested: {} front vertices max {:.4}x the bar (avg {:.4}x), {} | chords to "
                "resolve {} (at the sizing floor {}) | front_conv {:.4} || {}{}{}",
                m_converged ? "Converged" : "Optimization did not converge",
                m_energy_verdict ? " (front measured at convergence, before the finishing pass)"
                                 : "",
                front_ok ? "resolved" : "NOT resolved",
                ec.n_edges,
                ec.max_edge,
                ec.avg_edge(),
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

    // Offset edges turned from the field, checked UNCONDITIONALLY -- a defect in the delivered
    // mesh, not a debug curiosity, so it is reported whether or not debug_output is on. Run here,
    // at the end of optimize_offset, so it describes the mesh the driver is about to write: after
    // the finishing pass, not at the loop's verdict. Reported for a run that did not converge too.
    // An edge is turned when its normal, out of the band, is more than foldover_angle from the
    // field's normal (front_edge_turned()). As 3D's [turned faces], but for its pointer to the
    // debug frames' per-face field_angle_turned, which 2D's frames do not write per edge. Until the
    // growth step of the 2D lock-step port this block counted the folds (fold_test()) as
    // [foldover], the [exit folds] line's count.
    {
        const TurnedEdgeCensus fc = turned_edge_census();
        if (fc.n_turned > 0) {
            logger().warn(
                "[turned edges] {} of {} offset edges are turned from the field: their normal, "
                "out of the band, is more than foldover_angle {} degrees from the field's normal "
                "(grad d at the midpoint); the worst {:.4g} degrees, at ({:.4}, {:.4}). {} edge(s) "
                "not measurable.",
                fc.n_turned,
                fc.n_edges,
                m_offset_params.foldover_angle,
                fc.worst,
                fc.worst_at[0],
                fc.worst_at[1],
                fc.n_unmeasured);
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
            m_offset_params.front_measure == "vertex_ring" ? "front vertex's ring measure"
                                                           : "chord",
            quality,
            optimization_metrics.size(),
            m_offset_params.max_iterations,
            floor_fact.empty() ? "" : " ",
            floor_fact);
    }
}

} // namespace wmtk::components::topological_offset
