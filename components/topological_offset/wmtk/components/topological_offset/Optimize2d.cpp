#include <wmtk/utils/AMIPS2D.h>
#include "TopoOffsetTriMesh.h"

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SizingField.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <set>

namespace wmtk::components::topological_offset {

/**
 * The 2D optimization phase. The operations themselves -- split, collapse, the quality-driven
 * edge flip, and the driver that sequences them -- are wmtk::TriOptimizerMesh's. What is here
 * is only what the offset knows: where its two tracked surfaces are, and how a vertex on the
 * offset boundary is allowed to move.
 */

namespace {
/// Diagnostic-only running maximum over a smoothing pass, which is run in parallel.
void atomic_max(std::atomic<long long>& target, long long value)
{
    long long cur = target.load();
    while (value > cur && !target.compare_exchange_weak(cur, value)) {
    }
}
} // namespace

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
    // No offset criterion here and none in the `after` hook: the envelope is Phase A's
    // constraint and the offset criterion belongs to Phase B. Same as 3D -- see Swap.cpp.
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

std::shared_ptr<SampleEnvelope> TopoOffsetTriMesh::envelope_for_mask(uint64_t mask) const
{
    if (mask == 0) return nullptr;
    if ((mask & (mask - 1)) == 0) {
        // Single bit: the member envelope itself -- a real SampleEnvelope, safe on every path
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
    std::vector<std::shared_ptr<SampleEnvelope>> members;
    for (const auto& [tag, env] : m_tag_envelopes) {
        const auto it = m_tag_bit.find(tag);
        if (it != m_tag_bit.end() && (mask & (uint64_t(1) << it->second))) {
            members.push_back(env);
        }
    }
    std::shared_ptr<SampleEnvelope> isect;
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

std::shared_ptr<SampleEnvelope> TopoOffsetTriMesh::containment_for(
    const uint64_t region_mask,
    const bool on_offset) const
{
    // The region side first, and OUTSIDE the lock: envelope_for_mask() takes m_isect_mutex
    // itself and std::mutex is not recursive, so calling it while holding the lock below
    // deadlocks. Nothing here re-enters it after the lock is taken.
    const std::shared_ptr<SampleEnvelope> region = envelope_for_mask(region_mask);

    // The offset side, Phase A only: Phase B has to move the front, and a tube around where it
    // currently sits would cap how far it can travel. Null before the first rebuild.
    const bool hold_offset = on_offset && m_phase == OptPhase::A && m_offset_envelope != nullptr;

    if (!hold_offset) return region; // may itself be null: nothing contains this simplex
    if (!region) return m_offset_envelope;

    // On both: the intersection, inside every tube it lies on. Memoized per region mask;
    // rebuild_offset_envelope() clears the map, so an entry can never outlive its tube.
    {
        std::lock_guard<std::mutex> lock(m_isect_mutex);
        const auto it = m_offset_isect_cache.find(region_mask);
        if (it != m_offset_isect_cache.end()) return it->second;
    }
    std::shared_ptr<SampleEnvelope> isect = std::make_shared<IntersectionEnvelope>(
        std::vector<std::shared_ptr<SampleEnvelope>>{region, m_offset_envelope});
    std::lock_guard<std::mutex> lock(m_isect_mutex);
    return m_offset_isect_cache.emplace(region_mask, std::move(isect)).first->second;
}

bool TopoOffsetTriMesh::project_into_containment(const size_t vid, Vector2d& x) const
{
    const uint64_t mask = vertex_boundary_mask(vid);
    if (mask == 0) return true; // nothing holds this vertex; any position is valid

    // The real members, never envelope_for_mask()'s composite -- see the declaration for why
    // nearest_point() on an IntersectionEnvelope dereferences a null BVH.
    std::vector<const SampleEnvelope*> members;
    for (const auto& [tag, env] : m_tag_envelopes) {
        const auto it = m_tag_bit.find(tag);
        if (it != m_tag_bit.end() && (mask & (uint64_t(1) << it->second))) {
            members.push_back(env.get());
        }
    }
    if (members.empty()) return true; // every bit dangled: no tube was ever built for them

    // Alternating projection, worst violation first. A single tube needs one round, since
    // nearest_point() lands x on the curve and the tube contains its own curve; a junction needs
    // a few, converging toward the point where its curves meet.
    constexpr int kMaxRounds = 8;
    for (int round = 0; round < kMaxRounds; ++round) {
        const SampleEnvelope* worst = nullptr;
        double worst_d2 = -1.;
        for (const SampleEnvelope* e : members) {
            if (!e->is_outside(x)) continue;
            const double d2 = e->squared_distance(x);
            if (d2 > worst_d2) {
                worst_d2 = d2;
                worst = e;
            }
        }
        if (!worst) return true; // inside every tube it lies on
        Vector2d proj = x;
        worst->nearest_point(x, proj);
        if (!proj.allFinite()) return false;
        x = proj;
    }

    // Did not settle: two tubes that do not actually intersect near x. A construction question,
    // not something to paper over by committing the last iterate -- the caller keeps the entry
    // position instead.
    for (const SampleEnvelope* e : members) {
        if (e->is_outside(x)) return false;
    }
    return true;
}

bool TopoOffsetTriMesh::face_is_deformable(const size_t fid) const
{
    if (m_deform_tags.empty()) return false;
    if (m_face_extra[fid].label != 0) return false;
    const auto& tags = m_face_attribute[fid].tags;
    if (tags.empty()) return false;
    for (const int64_t t : tags) {
        if (m_deform_tags.count(t) == 0) return false;
    }
    return true;
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
    const std::vector<size_t>& ring = get_one_ring_fids_for_vertex(t);
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
        c.q1 = m_vertex_attribute[vs[(k + 1) % 3]].m_posf;
        c.q2 = m_vertex_attribute[vs[(k + 2) % 3]].m_posf;
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
    auto& solver = m_solver.local();
    if (!solver) {
        solver = polysolve::nonlinear::Solver::create(
            optimization::basic_nonlinear_solver_params,
            optimization::basic_linear_solver_params,
            1,
            opt_logger());
    }
    const Vector2d x0 = smoothing_position(vid);
    Eigen::VectorXd x = x0;
    try {
        solver->minimize(*energy, x);
    } catch (const std::exception&) {
    }
    set_smoothing_position(vid, Vector2d(x));
    for (const size_t fid : ring) {
        if (is_inverted(fid)) {
            set_smoothing_position(vid, x0);
            ++m_smooth_rejects.inverted;
            return false;
        }
    }
    ++m_smooth_rejects.accepted;
    m_released_tube_dirty.store(true, std::memory_order_release); // the boundary may have moved
    return true;
}

void TopoOffsetTriMesh::release_deformable_regions()
{
    // Held: any tag on an input-complex face or on a wall face, and the curve group. Everything
    // else with an envelope is an object the offset merely shares the scene with, and
    // deform_others releases it -- tube dropped, faces given a rest shape, smoothing deforming it
    // against RestAMIPSEnergy2D instead of a tube refusing every move. Dangling mask bits are
    // already the envelope machinery's normal case, so the queries need no change.
    //
    // Never released, whatever else it touches: every tag the offset_selection expression names.
    // The complex is defined by that expression, so those tags are the geometry the offset
    // measures from, faces of their own or not. Do not go back to a heuristic over complex faces;
    // measured worse -- see git history of this file.
    std::set<int64_t> kept;
    std::set<int64_t> source_tags;
    if (m_offset_params.offset_selection) {
        for (const int64_t t : m_offset_params.offset_selection->tags_involved()) {
            kept.insert(t);
            source_tags.insert(t);
        }
    }
    for (const Tuple& e : get_edges()) {
        if (e.switch_face(*this)) continue; // wall edges only
        for (const int64_t t : m_face_attribute[e.fid(*this)].tags) kept.insert(t);
    }
    if (m_curve_tag >= 0) kept.insert(m_curve_tag);
    // protected_tags is not consulted here, and the two keys must stay orthogonal: protected_tags
    // is a tagging decision (whether a band cell overwrites the object's tag or carries both) and
    // deform_others is a geometry decision (whether other objects may deform).

    m_deform_tags.clear();
    for (const auto& [tag, env] : m_tag_envelopes) {
        if (kept.count(tag) == 0) m_deform_tags.insert(tag);
    }
    m_source_tags = source_tags; // the ops-only tube's classification applies the same rule
    if (m_deform_tags.empty()) {
        logger().info("[deform_others] nothing to release: every tagged region is held");
        return;
    }

    // No tube is ever edited: releasing a boundary is a property of its vertices' masks (the loop
    // below), and every tag's envelope, released tags included, keeps its as-loaded segments. A
    // tube held by nobody constrains nothing, while a tube that no longer covers geometry whose
    // bits some vertex still carries measures that vertex against the far side of the scene. Do
    // not re-add either edit -- dropping released segments from kept tags' tubes, or reducing a
    // released tag's own tube; both measured worse -- see git history of this file.
    std::string released;
    for (const int64_t t : m_deform_tags) {
        released += " " + m_tag_id_to_name.at(t);
    }

    // Edit the masks in place, never recompute them: a mask is seeded at construction and
    // propagated by every operation since (a split ANDs its endpoints, a collapse ORs them), so
    // it carries history no rebuild from the current edge set can reproduce. The loop below
    // reduces a freed vertex's mask to its source bits -- held by released geometry alone it ends
    // at 0, while a junction vertex where the released boundary crosses the source keeps the
    // source's hold, so the source's own crossing edges stay contained.
    uint64_t released_bits = 0;
    for (const int64_t t : m_deform_tags) {
        const auto it = m_tag_bit.find(t);
        if (it != m_tag_bit.end()) released_bits |= (uint64_t(1) << it->second);
    }
    if (released_bits) {
        // Free a released boundary's vertices down to their source bits. Clearing only the
        // released tag's bit leaves them pinned in the other side's tube (an interface edge
        // contributes bits for both of its sides); clearing the source bits too leaves the
        // source's own crossing edges contained by nothing.
        //
        // Never freed: an edge that also borders a source tag (it is the source's own boundary),
        // an edge both of whose faces carry a source tag (complex-internal, and the front cannot
        // enter the complex, so it must stay where it was loaded), or a wall vertex.
        uint64_t source_bits = 0;
        for (const int64_t t : source_tags) {
            const auto it = m_tag_bit.find(t);
            if (it != m_tag_bit.end()) source_bits |= (uint64_t(1) << it->second);
        }
        const auto face_has_source = [&](const size_t fid) {
            for (const int64_t t : m_face_attribute[fid].tags) {
                if (source_tags.count(t)) return true;
            }
            return false;
        };
        size_t n_freed = 0;
        for (const Tuple& e : get_edges()) {
            const size_t eid = e.eid(*this);
            if (!m_edge_attribute[eid].m_is_surface_fs) continue;
            const std::optional<Tuple> f_opp = e.switch_face(*this);
            if (!f_opp) continue; // the domain wall
            CellTag edge_tags;
            const auto& t0 = m_face_attribute[e.fid(*this)].tags;
            const auto& t1 = m_face_attribute[f_opp->fid(*this)].tags;
            std::set_symmetric_difference(
                t0.begin(),
                t0.end(),
                t1.begin(),
                t1.end(),
                std::inserter(edge_tags, edge_tags.begin()));
            bool released_here = false, touches_source = false;
            for (const int64_t t : edge_tags) {
                if (m_deform_tags.count(t)) released_here = true;
                if (source_tags.count(t)) touches_source = true;
            }
            if (!released_here || touches_source) continue;
            if (face_has_source(e.fid(*this)) && face_has_source(f_opp->fid(*this))) {
                continue; // complex-internal: anchored, never freed
            }
            for (const size_t v : {e.vid(*this), e.switch_vertex(*this).vid(*this)}) {
                if (!m_vertex_attribute[v].on_bbox_faces.empty()) continue; // wall stays held
                const uint64_t kept = m_vertex_extra[v].m_boundary_mask & source_bits;
                if (m_vertex_extra[v].m_boundary_mask != kept) ++n_freed;
                m_vertex_extra[v].m_boundary_mask = kept;
            }
        }
        logger().info(
            "[deform_others] {} boundary vertices freed down to their source bits",
            n_freed);
    }

    size_t n_faces = 0;
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        if (face_is_deformable(fid)) {
            stamp_rest_face(fid);
            ++n_faces;
        }
    }
    logger().info(
        "[deform_others] released:{} | {} deformable faces stamped with their rest shape; "
        "held: {} envelopes",
        released,
        n_faces,
        m_tag_envelopes.size());
    m_released_tube_dirty.store(true, std::memory_order_release);
    released_envelope(); // built here, at a consistent moment, not at some mid-pass first query
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTriMesh::rest_energy_for_vertex(
    const size_t vid) const
{
    if (m_deform_tags.empty()) return nullptr;
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
    // The shared smoother's own AMIPS factor, so the rest term and the equilateral quality
    // term it sums with sit at 1:1. Deliberately not a spec key.
    const double w = m_params.w_amips > 0 ? m_s_amips * m_params.w_amips : 1.0;
    return std::make_shared<RestAMIPSEnergy2D>(std::move(cells), w);
}

int64_t TopoOffsetTriMesh::tangent_curve_tag(const size_t vid, const Vector2d& x) const
{
    const uint64_t mask = vertex_boundary_mask(vid);
    if (mask == 0) return -1;
    int64_t best = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (const auto& [tag, env] : m_tag_envelopes) {
        const auto it = m_tag_bit.find(tag);
        if (it == m_tag_bit.end() || !(mask & (uint64_t(1) << it->second))) continue;
        if (m_tag_polyline.find(tag) == m_tag_polyline.end()) continue;
        const double d2 = env->squared_distance(x); // a REAL member; never the composite
        if (d2 < best_d2) {
            best_d2 = d2;
            best = tag;
        }
    }
    return best;
}

bool TopoOffsetTriMesh::curve_tangent(
    const int64_t tag,
    const Vector2d& x,
    const Vector2d& prefer,
    Vector2d& tau) const
{
    const auto env_it = m_tag_envelopes.find(tag);
    const auto pl_it = m_tag_polyline.find(tag);
    if (env_it == m_tag_envelopes.end() || pl_it == m_tag_polyline.end()) return false;

    Vector2d foot;
    bool on_corner = false;
    Vector2d seg_normal = Vector2d::Zero();
    int feature_id = -1;
    env_it->second->nearest_point_feature(x, foot, on_corner, seg_normal, feature_id);

    if (!on_corner) {
        // Segment interior: one tangent, the segment's own direction. seg_normal is unit and its
        // sign is arbitrary, so the quarter turn is too -- the caller's `prefer` fixes it.
        if (!seg_normal.allFinite() || seg_normal.norm() <= 0.) return false;
        tau = Vector2d(-seg_normal.y(), seg_normal.x()).normalized();
        if (tau.dot(prefer) < 0.) tau = -tau;
        return true;
    }

    // A polyline VERTEX: the tangent is two-valued. Take the incident segment pointing most
    // nearly along `prefer` -- the one-sided derivative in the direction the search wants to go.
    const TagPolyline2d& pl = pl_it->second;
    if (feature_id < 0 || feature_id >= int(pl.at_vertex.size())) return false;
    double best = -std::numeric_limits<double>::infinity();
    bool found = false;
    for (const int seg : pl.at_vertex[feature_id]) {
        const int a = pl.E[seg][0], b = pl.E[seg][1];
        const int far = (a == feature_id) ? b : a;
        Vector2d d = m_env_polyline_V[far] - m_env_polyline_V[feature_id];
        const double len = d.norm();
        if (!(len > 0.)) continue;
        d /= len;
        const double align = d.dot(prefer);
        if (align > best) {
            best = align;
            tau = d;
            found = true;
        }
    }
    return found;
}

bool TopoOffsetTriMesh::walk_along_curve(
    const int64_t tag,
    const Vector2d& x,
    const double s,
    Vector2d& out) const
{
    const auto env_it = m_tag_envelopes.find(tag);
    const auto pl_it = m_tag_polyline.find(tag);
    if (env_it == m_tag_envelopes.end() || pl_it == m_tag_polyline.end()) return false;
    const TagPolyline2d& pl = pl_it->second;

    // Start on the curve, at the foot rather than at x: the vertex sits inside a tube of
    // half-width eps, so it is generally a little off the polyline, and arclength is defined only
    // on it. This is also what makes the walk idempotent.
    Vector2d foot;
    bool on_corner = false;
    Vector2d seg_normal = Vector2d::Zero();
    int feature_id = -1;
    env_it->second->nearest_point_feature(x, foot, on_corner, seg_normal, feature_id);
    out = foot;
    double remaining = std::abs(s);
    if (!(remaining > 0.)) return true;

    // The direction to travel, resolved at the foot.
    Vector2d dir;
    {
        Vector2d probe;
        if (!curve_tangent(tag, x, Vector2d(1., 0.), probe)) return false;
        dir = (s >= 0.) ? probe : Vector2d(-probe);
    }

    // Which segment we are on, and which polyline vertex we are heading toward.
    int seg = -1;
    if (on_corner) {
        if (feature_id < 0 || feature_id >= int(pl.at_vertex.size())) return false;
        for (const int cand : pl.at_vertex[feature_id]) {
            const int a = pl.E[cand][0], b = pl.E[cand][1];
            const int far = (a == feature_id) ? b : a;
            const Vector2d d = m_env_polyline_V[far] - m_env_polyline_V[feature_id];
            if (d.norm() > 0. && d.normalized().dot(dir) > 0.9999) {
                seg = cand;
                break;
            }
        }
        if (seg < 0) return false; // dir does not leave this corner along any segment
    } else {
        seg = feature_id;
        if (seg < 0 || seg >= int(pl.E.size())) return false;
    }

    // March: walk to the end of the current segment in `dir`, or land inside it if the budget
    // runs out first. A shared vertex with exactly two incident segments has one continuation;
    // anything else -- an open end, three curves meeting -- stops the walk, because guessing a
    // continuation would slide the vertex onto a different curve.
    constexpr int kMaxSegments = 4096; // a walk this long is a bug, not a long boundary
    for (int step = 0; step < kMaxSegments; ++step) {
        const int a = pl.E[seg][0], b = pl.E[seg][1];
        const Vector2d pa = m_env_polyline_V[a], pb = m_env_polyline_V[b];
        Vector2d sd = pb - pa;
        const double slen = sd.norm();
        if (!(slen > 0.)) return false;
        sd /= slen;
        const int ahead_id = (sd.dot(dir) >= 0.) ? b : a;
        const Vector2d ahead = m_env_polyline_V[ahead_id];
        const double to_end = (ahead - out).norm();

        if (remaining <= to_end) {
            const Vector2d unit = (to_end > 0.) ? Vector2d((ahead - out) / to_end) : dir;
            out += remaining * unit;
            return true;
        }
        remaining -= to_end;
        out = ahead;

        // Continue across the polyline vertex.
        if (ahead_id < 0 || ahead_id >= int(pl.at_vertex.size())) return false;
        const auto& inc = pl.at_vertex[ahead_id];
        if (inc.size() != 2) return false; // open end or a genuine junction: stop here
        const int next = (inc[0] == seg) ? inc[1] : inc[0];
        if (next == seg) return false;
        const int na = pl.E[next][0], nb = pl.E[next][1];
        const int nfar = (na == ahead_id) ? nb : na;
        Vector2d nd = m_env_polyline_V[nfar] - m_env_polyline_V[ahead_id];
        if (!(nd.norm() > 0.)) return false;
        dir = nd.normalized();
        seg = next;
    }
    return false;
}

bool TopoOffsetTriMesh::swap_edge_after(const Tuple& t)
{
    if (!TriOptimizerMesh::swap_edge_after(t)) {
        return false;
    }
    // No offset criterion for surface flips: the shared swap has already checked both new
    // segments against their envelopes (see swap_edge_before()).
    // deform_others: a swap rewires exactly these two faces; their rest is stale.
    stamp_rest_face(t.fid(*this));
    if (const std::optional<Tuple> opp = t.switch_face(*this)) stamp_rest_face(opp->fid(*this));
    ++iter_cnt_swap;
    return true;
}

bool TopoOffsetTriMesh::collapse_edge_after(const Tuple& t)
{
    if (!TriOptimizerMesh::collapse_edge_after(t)) {
        return false;
    }
    if (!m_offset_params.sizing_collapse_min) { // see collapse_edge_before()
        m_vertex_attribute[collapse_cache.local().v2_id].m_sizing_scalar =
            m_collapse_survivor_sizing.local();
    }
    // No offset criterion in the loop: Phase A holds the front in m_offset_envelope, so the
    // shared pass's containment check enforces it with the tolerance semantics TriWild's input
    // gets, and Phase B re-places the front every round. Same as 3D.
    //
    // Coarsening keeps an absolute bar: it runs after the loop, so a collapse leaving any offset
    // face over tolerance is a regression with fewer elements, not a saving. face_criterion_rel()
    // is the max of AMIPS and the offset residual, each over its own tolerance.
    if (m_coarsen_mode && m_offset_potential) {
        double after = 0.;
        for (const size_t fid : get_one_ring_fids_for_vertex(t)) {
            after = std::max(after, face_criterion_rel(fid));
        }
        if (after > 1.0) {
            ++iter_cnt_collapse_offset_reject;
            return false;
        }
    }
    // deform_others: every surviving face at the survivor changed shape (v1 became v2);
    // their rest is stale.
    for (const size_t fid : get_one_ring_fids_for_vertex(t)) {
        stamp_rest_face(fid);
    }
    return true;
}

bool TopoOffsetTriMesh::collapse_edge_before(const Tuple& t)
{
    if (!TriOptimizerMesh::collapse_edge_before(t)) {
        return false;
    }
    // The survivor's own sizing scalar, for sizing_collapse_min = false: the base collapse
    // overwrites it with the min of the two, and collapse_edge_after() puts it back.
    // collapse_cache is the base's, filled by the call above; v2 survives.
    m_collapse_survivor_sizing.local() =
        m_vertex_attribute[collapse_cache.local().v2_id].m_sizing_scalar;
    // Unconditionally, not just when both endpoints are already on a tracked simplex: a
    // topological offset needs the link condition preserved for the mesh AND every substructure.
    if (!substructure_link_condition(t)) {
        return false;
    }
    return true;
}

bool TopoOffsetTriMesh::collapse_before_vertex(const size_t v1_id, const size_t v2_id)
{
    // Diagnostic: the flattest face this collapse is about to reshape, read back by
    // record_flatness() in collapse_after_vertex().
    {
        double f = 1.;
        for (const size_t fid : get_one_ring_fids_for_vertex(tuple_from_vertex(v1_id))) {
            f = std::min(f, face_flatness(fid));
        }
        m_collapse_parent_flatness.local() = f;
    }

    const auto& VE = m_vertex_extra;

    // v1 is the vertex the collapse removes; it merges into v2, which keeps its position. A
    // vertex on any tracked boundary -- input complex, region boundary, domain wall alike -- may
    // be removed provided it merges onto a vertex of the same class (the per-class rules below;
    // the base's on_bbox_faces subset rule for the wall), the result stays inside its tags'
    // envelopes, and the substructure link condition survives. That is TriWild's rule for its
    // input surface, applied uniformly; the wall gets no special refusal.

    // Never both surfaces on one vertex: such a vertex sits at distance 0 from the input complex
    // and is asked to sit at target_distance from it at once, so the front through it can never
    // converge. Construction cannot make one; a collapse can, since collapse_after_vertex() ORs
    // the flags. Refused here, and asserted independently by check_no_vertex_on_both_surfaces()
    // after construction and after every Phase A.
    {
        const bool input = VE[v1_id].m_is_on_input || VE[v2_id].m_is_on_input;
        const bool offset = VE[v1_id].m_is_on_offset || VE[v2_id].m_is_on_offset;
        if (input && offset) {
            return false;
        }
    }

    // The front is always length-limited, whatever the pass says. Every other tracked surface is
    // held by its tags' envelopes, which bound how far it can be decimated whatever the length
    // gate does; the front deliberately has no envelope in Phase B -- it is the surface the
    // optimization exists to move -- so its sizing field is the only thing bounding its
    // resolution, and a pass with the length gate off would remove that too.
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
    for (const size_t fid : get_one_ring_fids_for_vertex(tuple_from_vertex(v2_id))) {
        if (get_quality(fid) >= kNeedleQuality) report_needle("COLLAPSE", fid, -1.);
        record_flatness("COLLAPSE", m_collapse_parent_flatness.local(), fid);
    }

    if (m_vertex_extra.at(v1_id).m_is_on_offset) ++iter_cnt_collapse_offset_removed;
    // Churn: v1 is the vertex being removed, so if a split created it this collapse undoes that
    // split. Same epoch means the collapse pass immediately following its own split pass took it
    // straight back out.
    {
        const uint32_t born = m_vertex_extra.at(v1_id).m_born_epoch;
        if (born != 0) {
            ++iter_cnt_recollapsed;
            if (born == m_op_epoch) ++iter_cnt_recollapsed_same_pass;
        }
    }

    // The base ORs its own m_is_on_surface, which is the union of the two; these say which.
    m_vertex_extra[v2_id].m_is_on_input =
        m_vertex_extra.at(v1_id).m_is_on_input || m_vertex_extra.at(v2_id).m_is_on_input;
    m_vertex_extra[v2_id].m_is_on_offset =
        m_vertex_extra.at(v1_id).m_is_on_offset || m_vertex_extra.at(v2_id).m_is_on_offset;
    m_vertex_extra[v2_id].m_is_on_region =
        m_vertex_extra.at(v1_id).m_is_on_region || m_vertex_extra.at(v2_id).m_is_on_region;
    // The survivor now carries both vertices' geometry, so it lies on the union of their
    // boundaries. See VertexExtra2d::m_boundary_mask.
    m_vertex_extra[v2_id].m_boundary_mask |= m_vertex_extra.at(v1_id).m_boundary_mask;

    // The base calls this only once a collapse has actually gone through: the 2D equivalent of
    // the 3D counter's home in collapse_edge_after().
    ++iter_cnt_collapse;
}

void TopoOffsetTriMesh::split_after_vertex(const size_t v_id)
{
    // The new vertex's classification is set in split_adjust_position(), which the base calls
    // BEFORE its own containment check; this hook runs after it, and the check dispatches on
    // exactly those bits. Only the birth epoch is set here.
    //
    // Churn instrumentation, read only by collapse_after_vertex(). Assigned rather than OR'd
    // because v_id may be a recycled slot carrying a dead vertex's bits.
    m_vertex_extra[v_id].m_born_epoch = m_op_epoch;
    if (m_op_epoch != 0) ++iter_cnt_split_born;

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
    return true; // the position itself is the base's business, and it is happy with it
}

bool TopoOffsetTriMesh::smooth_before(const Tuple& t)
{
    ++m_smooth_trace.attempted;
    const size_t vid = t.vid(*this);
    // The final Phase A does not move the front: it is converged by then, its smoothing there
    // would be AMIPS alone with the front free anywhere inside the offset tube, and no Phase B
    // follows to put it back. The pass may still split, collapse and swap around the front and
    // smooth everything else.
    if (m_freeze_front && m_vertex_extra[vid].m_is_on_offset) return false;

    // Diagnostic, recorded for every visit; only visits whose ring already holds a needle are
    // counted, and smooth_after() reads this back.
    auto& pre = m_needle_pre.local();
    pre = {ring_max_quality(vid), m_vertex_attribute[vid].m_posf};
    if (pre.first >= kNeedleQuality) ++m_needle_smooth_offered;

    // The base's smooth_before minus its bounding-box refusal, which is why this does not call
    // it. The base freezes the domain wall; here the wall is a region boundary held in ambient's
    // tag envelope like any other, so its vertices are smoothed and the containment check decides
    // whether the move survives. What keeps the wall a wall is that check, not immobility. Same
    // as 3D -- see TopoOffsetTetMesh::smooth_before().
    //
    // Rounding still has to happen, and its failure still refuses the move.
    const bool rounded_now = round(t);
    if (!m_vertex_attribute[vid].m_is_rounded && !rounded_now) {
        ++m_smooth_trace.before_unrounded;
        return false;
    }

    return true;
}

bool TopoOffsetTriMesh::smooth_after(const Tuple& t)
{
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

    // The plastic medium: a vertex whose whole ring is plastic and which no envelope holds flows
    // under rest-shape AMIPS alone (see smooth_plastic_vertex). Wall vertices, and vertices
    // touching the band or the complex, keep the standard path so the interfaces stay under the
    // usual rules -- a released boundary included: the plastic rest is re-stamped every group, so
    // this path cannot preserve a boundary's shape across the run, and it drops the quality term,
    // the boundary's only regularization. Preserving a released boundary's shape needs
    // cross-group memory, which does not exist yet.
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

    // Phase B: a front vertex goes through the shared smoother -- same solver, line search and
    // accept tests as every other vertex -- with the offset's options: its objective carries the
    // offset terms (smoothing_extra_energy) and there is no quality veto, since a front vertex
    // must be able to worsen its ring on the way to the level set. Shape is Phase A's job, and
    // every other vertex in both phases is TriWild's smooth_after() unchanged.
    if (phase_places_front() && ve.m_is_on_offset) {
        const bool ok = smooth_front_vertex_phase_b(t);
        if (ok) ++m_smooth_trace.offset_accepted;
        return ok;
    }
    // Phase A is TriWild: the shared smoother, with the front held by m_offset_envelope and
    // carrying no offset term of its own.
    const double before = ve.m_is_on_offset ? band_vertex_residual(vid) : 0.;
    const bool ok = TriOptimizerMesh::smooth_after(t);
    if (!ve.m_is_on_offset) {
        return ok;
    }
    const double after = band_vertex_residual(vid);
    if (ok) ++m_smooth_trace.offset_accepted;

    // The residuals are read BEFORE the caller's rollback, so `after` is the position the
    // smoother proposed rather than the one kept -- which separates "no better place" from "found
    // one, refused by the accept checks". Read the two numbers next to the accepted count.
    const auto nano = [](double x) { return static_cast<long long>(std::min(x, 1e9) * 1e9); };
    m_smooth_trace.res_before_nano += nano(before);
    m_smooth_trace.res_after_nano += nano(after);
    atomic_max(m_smooth_trace.res_max_before_nano, nano(before));
    atomic_max(m_smooth_trace.res_max_after_nano, nano(after));
    return ok;
}

Vector2d TopoOffsetTriMesh::offset_vertex_normal(const size_t vid) const
{
    // See the declaration: the single definition of an offset vertex's normal. The direction the
    // offset grew along, taken from the geometry rather than the mesh -- the BVH is the same
    // structure Phi queries, so the foot point is the exact nearest point on the complex as
    // loaded. Flips discontinuously across the medial axis, where the nearest feature changes.
    if (m_input_complex_bvh) {
        const Vector2d x = m_vertex_attribute[vid].m_posf;
        const Vector3d foot = m_input_complex_bvh->nearest_point(x);
        const Vector2d d(x.x() - foot.x(), x.y() - foot.y());
        const double len = d.norm();
        if (len > 0.) return d / len;
    }
    return Vector2d::Zero();
}

double TopoOffsetTriMesh::front_vertex_normal_gradient(const size_t vid) const
{
    // ||grad F|| at the vertex's current position, F the objective smooth_front_vertex_phase_b()
    // minimises: stationarity of the 2-D solve. Under front_normal_projection the unknown is the
    // position along the normal, so the derivative along it, |grad F . n|.
    const Vector2d x = m_vertex_attribute[vid].m_posf;
    Eigen::VectorXd xv = x, g(2);
    phase_b_front_objective(vid, x)->gradient(xv, g);
    if (!g.allFinite()) return std::numeric_limits<double>::infinity();
    // Along the field normal whatever the placement mode: the test asks whether the front is where
    // the field wants it. Tangential motion is the flat direction of the energy -- only w x AMIPS
    // acts along the front -- where a tiny gradient means a large Newton step.
    const Vector2d n = front_vertex_move_direction(vid);
    if (n.squaredNorm() > 0.) return std::abs(n.dot(Vector2d(g)));
    return g.norm();
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
        if (!surface_segment_is_outside(vids[0], vids[1])) continue;

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
        const auto probe = [&](const std::shared_ptr<SampleEnvelope>& env, int tag) {
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
        "(the Phase A pin), {} REGION-class (a tag tube / junction intersection), {} neither "
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
        "\tsmooth trace: attempted {} | before: bbox {}, unrounded {}, phase-B wrong class {}, "
        "phase-B envelope-held background {} / ON-OFFSET {} | reached the smoother: {} on the "
        "offset boundary, {} elsewhere ({} of them on another region boundary) | ({})",
        s.attempted.load(),
        s.before_bbox.load(),
        s.before_unrounded.load(),
        s.before_phase_b_not_offset.load(),
        s.before_phase_b_enveloped_background.load(),
        s.before_phase_b_enveloped_offset.load(),
        s.offset_attempted.load(),
        s.interior_attempted.load(),
        s.region_attempted.load(),
        m_smooth_rejects.to_string());
    // The envelope-held offset vertices, which this scheme does not place at all. Warned, not
    // merely counted: they are left where Phase A put them, and band_vertex_is_reachable() books
    // them PINNED, so they stop gating convergence and are reported separately.
    if (s.before_phase_b_enveloped_offset.load() > 0) {
        logger().warn(
            "\t{} Phase B offset visits were SKIPPED AND PINNED: the vertex is on the offset "
            "boundary AND held by an envelope (a tag region boundary, or the domain wall), which "
            "requires it to be within envelope_size of the input boundary and at target_distance "
            "from the complex at once. Not satisfiable, so it is left where Phase A put it and "
            "excluded from max_reachable rather than chased -- read the PINNED half of the band "
            "measures below for what that is costing. This is a RETREAT: the placement exists "
            "(commented out in smooth_before) and is the thing to restore. See OPEN PROBLEMS in "
            ".claude/CLAUDE.md.",
            s.before_phase_b_enveloped_offset.load());
    }
    // The one thing here that IS a defect: a vertex outside the envelope that is supposed to
    // contain it. The projection pulls it back, but it should not have been outside.
    if (m_placement_env_entry_outside.load() > 0) {
        logger().warn(
            "\t{} constrained placements found the vertex ALREADY outside its own envelope on "
            "entry. The invariant is 0 -- this says construction or Phase A is leaving offset "
            "vertices outside their region tube. The projection pulls them back in, so the "
            "placement still runs, but the violation upstream is real.",
            m_placement_env_entry_outside.load());
    }
    const int n = std::max(1, s.offset_attempted.load());
    logger().info(
        "\toffset term: {} attempted -> {} accepted | phi residual over them: avg {:.6} -> "
        "{:.6}, max {:.6} -> {:.6}",
        s.offset_attempted.load(),
        s.offset_accepted.load(),
        double(s.res_before_nano.load()) / n * 1e-9,
        double(s.res_after_nano.load()) / n * 1e-9,
        double(s.res_max_before_nano.load()) * 1e-9,
        double(s.res_max_after_nano.load()) * 1e-9);
}


void TopoOffsetTriMesh::pre_optimize_input_mesh()
{
    // See the declaration for why this exists. Everything here is Phase A on a mesh that has no
    // offset in it yet.
    const OptPhase saved_phase = m_phase;
    const EdgeSplitMode saved_mode = m_edge_split_mode;

    // Phase A's units. optimization_quality_stats() and optimization_stop_metric() both branch on
    // m_phase and must agree: Phase A is absolute AMIPS against stop_energy, which is what
    // refine_sizing_around_worst() derives its filter from. Phase B's normalized pair would also
    // dereference m_offset_potential, which does not exist yet.
    m_phase = OptPhase::A;
    // The split hook branches on this. Midpoint is the construction path (simplicial embedding
    // and marching, which carry their own labels); the shared engine's splits must take the
    // Optimization path or they would be treated as marching splits.
    m_edge_split_mode = EdgeSplitMode::Optimization;

    // The sizing field: target_distance on the input-complex BOUNDARY, graded outward. That
    // boundary is the curve Phi measures from and the curve the marching wraps the band around,
    // so asking for delta there makes the band one delta-scale cell thick, which is what puts the
    // constructed offset near delta from the complex. The boundary and not the whole complex: a
    // filled complex's interior vertices carry the same label, and resolving them at delta would
    // multiply the mesh for geometry the offset never measures against. An edge is on that
    // boundary when exactly one incident face carries the input-complex label -- the rule that
    // produced m_phi_E.
    const double l_target = std::max(m_params.l, 1e-16);
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / l_target);
    const double s_input = std::clamp(
        m_offset_params.target_distance / l_target,
        s_floor,
        m_offset_params.max_sizing_scalar);
    // Which sizing field this pass runs against -- see pre_optimize_sizing_from_edges.
    if (m_offset_params.pre_optimize_sizing_from_edges) {
        // "Keep the resolution you have": every vertex takes the mean of its own incident edge
        // lengths, the rule init_offset_sizing_field() uses on the front. target_distance never
        // enters, so this pass improves element quality without refining anything toward delta
        // and the constructed offset lands wherever the input mesh's own scale puts it.
        size_t n_set = 0;
        double sum_h = 0.;
        for (const Tuple& v : get_vertices()) {
            const size_t vid = v.vid(*this);
            double sum_len = 0.;
            int n = 0;
            for (const size_t nb : get_one_ring_vids_for_vertex_duplicate(vid)) {
                sum_len += (m_vertex_attribute[vid].m_posf - m_vertex_attribute[nb].m_posf).norm();
                ++n;
            }
            if (n == 0) continue;
            const double h_local = sum_len / n;
            m_vertex_attribute[vid].m_sizing_scalar =
                std::clamp(h_local / l_target, s_floor, m_offset_params.max_sizing_scalar);
            sum_h += h_local;
            ++n_set;
        }
        logger().info(
            "[pre-optimize] sizing field: {} vertices seeded from their OWN incident edge "
            "lengths (mean {:.6g} = {:.4g} l); target_distance {} does not enter",
            n_set,
            n_set ? sum_h / double(n_set) : 0.,
            n_set ? (sum_h / double(n_set)) / l_target : 0.,
            m_offset_params.target_distance);
    } else {
        // The gradation: gradation_smooth_sizing(), the shared ring BFS, and nothing else. Do not
        // re-add the rejected alternatives -- a Lipschitz ramp in distance (with or without a
        // plateau), one-ring Jacobi averaging, tetwild's init_sizing_field() distance BFS, or
        // triwild's multiplicative grow-and-refine recipe; all measured worse -- see git history
        // of this file.
        std::vector<size_t> seeds;
        for (const Tuple& e : get_edges()) {
            const std::optional<Tuple> opp = e.switch_face(*this);
            const bool in_a = m_face_extra[e.fid(*this)].label == 1;
            const bool in_b = opp ? (m_face_extra[opp->fid(*this)].label == 1) : false;
            if (in_a == in_b) continue; // interior to the complex, or interior to the background
            for (const size_t vid : {e.vid(*this), e.switch_vertex(*this).vid(*this)}) {
                double& sc = m_vertex_attribute[vid].m_sizing_scalar;
                sc = std::min(sc, s_input);
                seeds.push_back(vid);
            }
        }
        // A curve complex has no label-1 faces, so the edge test above seeds nothing on it and the
        // pre-pass would be a silent no-op, leaving the band one coarse input cell thick. The
        // complex's vertices carry the label whatever its dimension: seed those too, but ONLY
        // where the complex is a curve -- a region labels its interior vertices as well, and
        // seeding those refines the whole region. A vertex with a label-1 face in its ring belongs
        // to a region, whose boundary the edge loop above already seeded.
        for (const Tuple& v : get_vertices()) {
            const size_t vid = v.vid(*this);
            if (m_vertex_extra[vid].label != 1) continue;
            bool region = false;
            for (const size_t fid : get_one_ring_fids_for_vertex(v)) {
                if (m_face_extra[fid].label == 1) {
                    region = true;
                    break;
                }
            }
            if (region) continue;
            double& sc = m_vertex_attribute[vid].m_sizing_scalar;
            sc = std::min(sc, s_input);
            seeds.push_back(vid);
        }
        wmtk::vector_unique(seeds);
        if (!seeds.empty()) {
            gradation_smooth_sizing(m_offset_params.sizing_gradation, seeds);
        }
        logger().info(
            "[pre-optimize] sizing seed: {} input-complex vertices at scalar {:.6g} "
            "(= target_distance {} / l {:.6g}), graded outward at {}x per ring",
            seeds.size(),
            s_input,
            m_offset_params.target_distance,
            l_target,
            m_offset_params.sizing_gradation);
    }

    // The seeded field, before a single operation runs.
    if (m_offset_params.debug_output) {
        write_vtu(m_offset_params.output_path + "_seeded");
    }

    const double before = std::get<0>(optimization_quality_stats());
    logger().info(
        "[pre-optimize] TriWild over the input mesh: {} vertices, {} faces, max element quality "
        "{:.4} (stop {:.4}), held by the per-tag region envelopes only",
        get_vertices().size(),
        get_faces().size(),
        before,
        optimization_stop_metric());

    mesh_improvement(std::max(1, m_offset_params.max_iterations));

    const double after = std::get<0>(optimization_quality_stats());
    logger().info(
        "[pre-optimize] done: {} vertices, {} faces, max element quality {:.4} -> {:.4}",
        get_vertices().size(),
        get_faces().size(),
        before,
        after);

    m_edge_split_mode = saved_mode;
    m_phase = saved_phase;
    consolidate_mesh();

    // Re-derive the construction labels, because the optimization does not maintain them. No
    // operation propagates VertexExtra2d::label, and marching_tris() decides which edges to split
    // from exactly that label -- this pass is the first optimization that runs BEFORE the
    // marching, so leaving it stale gives a wrong crossing set and a band with holes.
    //
    // Re-derived rather than propagated: the label is a function of the FACE TAGS, which the base
    // does propagate through split and collapse, and label_input_complex() is the authority on
    // that function. Cleared first because it only ever writes 1 and has no path back to 0.
    for (const Tuple& v : get_vertices()) m_vertex_extra[v.vid(*this)].label = 0;
    for (const Tuple& e : get_edges()) m_edge_extra[e.eid(*this)].label = 0;
    for (const Tuple& f : get_faces()) m_face_extra[f.fid(*this)].label = 0;
    label_input_complex();

    // The input complex is NOT re-extracted: the driver builds m_input_complex_bvh -- and with it
    // m_phi_V/E/P, the arrays init_offset_potential() hands to Phi -- once before
    // execute_offset(), and that one extraction serves the whole run. Do not re-extract here;
    // init_input_complex_bvh() collects the closure of the label-1 faces, so the interior vertices
    // this pass created would enter m_phi_V and move Phi. Measured worse -- see git history.
    //
    // The cost is that Phi measures the polygon the mesh had on load while the mesh carries the
    // refined one: a systematic chord sagitta, growing with the input's coarseness and curvature.
    // The fix, if it is ever needed, is to re-extract the boundary curve only, not the closure.
    needle_scan("after the pre-pass");
}

void TopoOffsetTriMesh::init_offset_sizing_field()
{
    // Paper Sec. 5.3.3, Step 1: the sizing field "is defined on each edge of the offset mesh and
    // is initialized with the current length of each edge."
    //
    // The base's default scalar of 1 means a target of m_params.l, a fraction of the bounding box
    // diagonal and far longer than the front's own edges, so starting there makes every front edge
    // a collapse candidate on the first pass and the front is decimated before the metrics below
    // get to speak. Starting from the current lengths keeps the resolution the mesh has. The field
    // is per-vertex here, so a vertex takes the mean of its incident front edges.
    const double l = std::max(m_params.l, 1e-16);
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / l);

    double raw_sum = 0.;
    int n_seeded = 0;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        double sum_len = 0.;
        int n = 0;
        for (const Tuple& e : get_one_ring_edges_for_vertex(v)) {
            if (!edge_is_offset_surface_live(e)) continue;
            const size_t nb =
                (e.vid(*this) == vid) ? e.switch_vertex(*this).vid(*this) : e.vid(*this);
            sum_len += (m_vertex_attribute[vid].m_posf - m_vertex_attribute[nb].m_posf).norm();
            ++n;
        }
        // Every vertex is seeded, not just the front's: a vertex with no incident front edge falls
        // back to its whole one-ring. Leaving the background at the base target -- far coarser
        // than the mesh construction produced -- is what makes a bare collapse pass destructive,
        // since the gate is edge length against the target at its endpoints and would then mark
        // essentially every interior edge as collapsible.
        if (n == 0) {
            for (const size_t nb : get_one_ring_vids_for_vertex_duplicate(vid)) {
                sum_len += (m_vertex_attribute[vid].m_posf - m_vertex_attribute[nb].m_posf).norm();
                ++n;
            }
            if (n == 0) continue; // isolated vertex; nothing to measure
            m_vertex_attribute[vid].m_sizing_scalar =
                std::clamp((sum_len / n) / l, s_floor, m_offset_params.max_sizing_scalar);
            continue;
        }
        raw_sum += sum_len / n;
        ++n_seeded;
        m_vertex_attribute[vid].m_sizing_scalar =
            std::clamp((sum_len / n) / l, s_floor, m_offset_params.max_sizing_scalar);
    }
    logger().info(
        "\tOffset sizing seed: {} vertices, mean incident length {:.6} -> target {:.6} "
        "(base l {:.6}, l_min {:.6} = {} x target_distance {}, scalar floor {:.6})",
        n_seeded,
        n_seeded > 0 ? raw_sum / n_seeded : 0.,
        std::max(n_seeded > 0 ? raw_sum / n_seeded : 0., m_offset_params.min_edge_length),
        l,
        m_offset_params.min_edge_length,
        m_offset_params.min_edge_length_rel,
        m_offset_params.target_distance,
        s_floor);

    // The front's resolution follows from the tolerance and the level set's curvature, set once
    // here. A chord of length L over curvature radius rho misses it by L^2 / (8 rho) and the front
    // must stay within eps x delta of the level set (eps = offset_envelope_rel), so
    // L <= sqrt(8 eps delta rho). rho is floored at delta: on the convex side the level set's
    // curvature never exceeds 1 / delta, and where it does -- a concave crease at the input's
    // medial axis, a seam between two fronts -- no chord resolves it and the vertex test decides.
    {
        const double delta = m_offset_params.target_distance;
        const double eps = m_offset_params.offset_envelope_rel;
        std::vector<size_t> changed;
        double L_min = std::numeric_limits<double>::infinity(), L_max = 0.;
        size_t n_flat = 0;
        for (const Tuple& v : get_vertices()) {
            const size_t vid = v.vid(*this);
            if (!m_vertex_extra[vid].m_is_on_offset) continue;
            const Vector2d x = m_vertex_attribute[vid].m_posf;
            const OffsetPotential2D& pot = potential_for(vid);
            const Vector2d g = pot.gradient(x);
            const Eigen::Matrix2d H = pot.hessian(x);
            const double gn = g.norm();
            // Curvature of the level set of Phi through x.
            double rho = delta;
            if (gn > 0. && g.allFinite() && H.allFinite()) {
                const double k = (g.x() * g.x() * H(1, 1) - 2. * g.x() * g.y() * H(0, 1) +
                                  g.y() * g.y() * H(0, 0)) /
                                 (gn * gn * gn);
                if (std::isfinite(k) && std::abs(k) > 0.)
                    rho = std::max(1. / std::abs(k), delta);
                else
                    rho = std::numeric_limits<double>::infinity();
            }
            if (!std::isfinite(rho)) {
                ++n_flat;
                continue; // a straight level set: the seeded resolution stands
            }
            // 3/4: Phase A splits an edge only once it is longer than 4/3 of its target, so the
            // target is 3/4 of the chord the tube allows.
            const double L = 0.75 * std::sqrt(8. * eps * delta * rho);
            double& sc = m_vertex_attribute[vid].m_sizing_scalar;
            const double ns = std::clamp(L / l, s_floor, m_offset_params.max_sizing_scalar);
            if (ns < sc) {
                sc = ns;
                changed.push_back(vid);
            }
            const double Lc = std::min(L, sc * l); // what the vertex actually gets
            L_min = std::min(L_min, Lc);
            L_max = std::max(L_max, Lc);
        }
        if (!changed.empty()) gradation_smooth_sizing(m_offset_params.sizing_gradation, changed);
        logger().info(
            "\tFront resolution from the tolerance: L = 3/4 sqrt(8 eps delta rho), rho >= delta "
            "-> {:.6g} .. {:.6g} ({:.3g} .. {:.3g} x delta) at eps {}; {} front vertices "
            "tightened, {} on a straight level set left at the seed",
            std::isfinite(L_min) ? L_min : 0.,
            L_max,
            std::isfinite(L_min) ? L_min / delta : 0.,
            L_max / delta,
            eps,
            changed.size(),
            n_flat);
    }
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
            cand.phi_over_c = potential_for_face(fid).value(cand.c) / c_level;
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
                    const std::shared_ptr<SampleEnvelope> env = surface_envelope_for_edge({{a, b}});
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

    const std::array<size_t, 6> now{
        {m_deg_split_created.load(),
         m_deg_collapse_offered.load(),
         m_deg_collapse_allowed.load(),
         m_deg_collapse_by_ringmax.load(),
         m_deg_collapse_by_stop.load(),
         m_deg_collapse_by_unrounded.load()}};
    logger().info(
        "[stuck-census #{}]   created since the last census: by SPLIT {} needle faces (a split "
        "is never refused on quality) | by COLLAPSE {} of {} offered at MAX_ENERGY, admitted by "
        "ring_max {} / stop_energy {} / unrounded {}",
        m_stuck_calls,
        now[0] - m_deg_prev_counts[0],
        now[2] - m_deg_prev_counts[2],
        now[1] - m_deg_prev_counts[1],
        now[3] - m_deg_prev_counts[3],
        now[4] - m_deg_prev_counts[4],
        now[5] - m_deg_prev_counts[5]);
    m_deg_prev_counts = now;

    m_stuck_prev_cells = std::move(cells);
}


bool TopoOffsetTriMesh::collapse_quality_allowed(
    const size_t v1,
    const size_t v2,
    const double q,
    const double ring_max) const
{
    // Pure instrumentation: the base's rule -- TriWild's -- is returned unchanged, so Phase A is
    // TriWild's mesh_improvement() with no offset-specific admission. See the header for why the
    // `q <= ring_max` clause is the one to watch. Do not swap in 3D's rule (`q <= ring_max` alone,
    // without the base's "any result under stop_energy" clause); measured worse -- see git history
    // of this file. Seam folds that 3D's rule would refuse remain a known open issue.
    const bool allowed = TriOptimizerMesh::collapse_quality_allowed(v1, v2, q, ring_max);
    if (q >= MAX_ENERGY) {
        ++m_deg_collapse_offered;
        if (allowed) {
            ++m_deg_collapse_allowed;
            if (!m_vertex_attribute.at(v1).m_is_rounded) {
                ++m_deg_collapse_by_unrounded;
            } else if (q <= m_params.stop_energy) {
                ++m_deg_collapse_by_stop;
            } else if (q <= ring_max) {
                ++m_deg_collapse_by_ringmax;
            }
        }
    }
    return allowed;
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
                const double sw = swap_weight(et);
                swap_info = fmt::format(
                    "on_surface {} (swap {}) | swap_weight {:.6g} (pass needs > 1e-5 -> {})",
                    surf,
                    surf ? "REFUSED outright" : "allowed",
                    sw,
                    sw > 1e-5 ? "would swap" : "REFUSED");
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
    // 3D version gets from its inside-any-tet check.
    const Vector2d p = m_vertex_attribute[vid].m_posf;
    return std::abs(m_input_complex_bvh->dist(VectorXd(p)) - m_offset_params.target_distance);
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
    const int k = m_offset_params.offset_residual_samples;
    if (k <= 0) return s;

    const size_t va = e.vid(*this), vb = e.switch_vertex(*this).vid(*this);
    if (!band_vertex_is_reachable(va) || !band_vertex_is_reachable(vb)) return s;

    const Vector2d pa = m_vertex_attribute[va].m_posf;
    const Vector2d pb = m_vertex_attribute[vb].m_posf;
    for (int i = 1; i <= k; ++i) {
        const double t = double(i) / (k + 1);
        const double r = potential_for_edge(va, vb).residual_length(pa + t * (pb - pa));
        s.max = std::max(s.max, r);
        s.sum += r;
        ++s.n;
    }
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
        if (es.n == 0) continue; // no samples asked for (offset_residual_samples <= 0)
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
    // The convergence criterion: ||grad (Phi(x) - c)^2|| at every band vertex -- the full norm of
    // the gradient of the same objective Phase B's local solves minimise, measured at the
    // variables the optimizer owns and compared against offset_gradient_tolerance(). Identical to
    // the Phase B local stop by construction, so "every visit finishes immediately" and "the run
    // has converged" are one statement. Weight 1 deliberately: an absolute bound in length units,
    // which a tuning weight would scale.
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

    // The normal-aligned companion, |grad E . n| with n the offset vertex normal. NOT the deciding
    // measure: only motion along n moves the surface off the level set, so it says how misplaced
    // the surface is, while the running tests are the full norm, which a local solve can zero. The
    // same projection serves the in-edge chord diagnostic on the edge's own normal. A degenerate
    // site has no normal; there the full norm stands in, conservative since |g . n| <= |g|.
    const auto project = [](const Eigen::VectorXd& g, const Vector2d& n) -> double {
        return (n.squaredNorm() > 0.) ? std::abs(g.head<2>().dot(n)) : g.norm();
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
        s.max_normal_aligned =
            std::max(s.max_normal_aligned, project(g, offset_vertex_normal(vid)));

        // Pinned vertices are reported, not gated -- the one place this criterion deliberately
        // parts company with residual_split(). A residual is a statement about the BOUNDARY; a
        // gradient is a statement about the ITERATION, and folding in a vertex no sweep can move
        // would make convergence unreachable by construction.
        if (!band_vertex_is_reachable(vid)) {
            s.max_pinned = std::max(s.max_pinned, gn);
            ++s.n_pinned;
            continue;
        }

        if (gn > s.max_reachable) {
            s.max_reachable = gn;
            s.worst_vid = vid;
        }
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
    // pinned vertex inherits an error no refinement fixes. The rest go to max_in_edge_pinned.
    if (include_edge_samples) {
        for (const Tuple& e : get_edges()) {
            if (!edge_is_offset_surface_live(e)) continue;
            const size_t va = e.vid(*this), vb = e.switch_vertex(*this).vid(*this);
            const bool gating = band_vertex_is_reachable(va) && band_vertex_is_reachable(vb);
            for_each_offset_edge_sample(e, [&](const Vector2d& q) {
                Eigen::VectorXd g(2);
                energy_for(edge_region(va, vb)).gradient(Eigen::VectorXd(q), g);
                const double q_full = g.norm();
                if (gating) {
                    s.max_in_edge = std::max(s.max_in_edge, q_full);
                } else {
                    s.max_in_edge_pinned = std::max(s.max_in_edge_pinned, q_full);
                }
                ++s.n_edge_samples;
            });
        }
    }

    s.avg_reachable = (s.n_reachable > 0) ? sum_reachable / s.n_reachable : 0.;
    return s;
}

double TopoOffsetTriMesh::edge_interpolation_residual(const Tuple& e) const
{
    const size_t a = e.vid(*this), b = e.switch_vertex(*this).vid(*this);
    const OffsetPotential2D& pot = potential_for_edge(a, b);
    const double c = pot.target_level();
    if (!(c > 0.)) return -1.;
    // The value, not the gradient: the gradient of ((Phi - c) / c)^2 is (2 / c) r grad Phi with
    // r = (Phi - c) / c, so interpolating it across an edge measures r x (the turn of grad Phi),
    // which at a pressed seam reads far larger than the geometry warrants. The second difference
    // of r cancels a uniform press and keeps only how far the level set curves away from the
    // chord, which halving the edge does reduce (~L^2). The factor 2 w / c puts it in the
    // gradient's units, so the vertex bar applies unchanged.
    const auto r_at = [&](const Vector2d& p) { return (pot.value(p) - c) / c; };
    const Vector2d pa = m_vertex_attribute[a].m_posf, pb = m_vertex_attribute[b].m_posf;
    const double ra = r_at(pa), rb = r_at(pb), rm = r_at(0.5 * (pa + pb));
    if (!std::isfinite(ra) || !std::isfinite(rb) || !std::isfinite(rm)) return -1.;
    return 2. * (1. - m_params.w_amips) / c * std::abs(rm - 0.5 * (ra + rb));
}


TopoOffsetTriMesh::EnergyCriterion TopoOffsetTriMesh::energy_criterion()
{
    EnergyCriterion s;
    const OptPhase saved = m_phase;
    m_phase = OptPhase::B; // the objective's offset terms exist only in Phase B
    // The convergence length, front_conv_rel x delta: what "placed" means for a vertex and
    // "resolved" for a chord. See edge_conv_ratio() for the role split -- offset_envelope_rel is
    // the leash on the operations, this is the accuracy, and startup requires leash <= accuracy.
    s.tube = m_offset_params.front_conv_rel * m_offset_params.target_distance;
    const auto placed = [&](const size_t vid) {
        return m_vertex_extra[vid].m_is_on_offset && m_vertex_attribute[vid].m_is_rounded;
    };
    std::vector<char> on_level(vert_capacity(), 0);
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!placed(vid)) continue;
        // The single phase's states -- see EnergyCriterion. rho is the reference-slope length
        // residual_length(), the measure the sag test qualifies its ends with. "Stopped" is the
        // objective's stationarity -- the remaining 1-D Newton step along the normal, in length,
        // at most the tube -- not the net displacement: at a pressed seam the two fronts take
        // turns pushing the one-cell strip and every vertex drifts a little for ever, while the
        // objective is stationary there.
        const Vector2d p = m_vertex_attribute[vid].m_posf;
        const double rho = potential_for(vid).residual_length(p);
        const double gn = front_vertex_conv_ratio(vid); // Newton step / (front_conv_rel x delta)
        const double newton_len =
            gn * m_offset_params.front_conv_rel * m_offset_params.target_distance;
        if (!std::isfinite(rho) || !std::isfinite(gn)) {
            ++s.n_unmeasurable;
        } else if (rho <= s.tube) {
            ++s.n_placed;
            on_level[vid] = 1;
        } else if (newton_len > s.tube) {
            ++s.n_travelling;
        } else if (front_vertex_touches_other(vid) || !m_offset_params.front_alignment_energy) {
            // Stationary off its level set. With the alignment term off the objective is the
            // offset term and w x AMIPS alone, so only a press can hold a vertex short of its
            // level set -- another front (touching), or the interior of a strip two cells thick.
            // With the alignment term on, a stationary vertex touching nothing is the term's own
            // bias: stuck.
            ++s.n_pressed_on;
            if (front_vertex_touches_other(vid)) ++s.n_pressed_touching;
        } else {
            ++s.n_stuck;
            if (rho > s.worst_stuck_rho) {
                s.worst_stuck_rho = rho;
                s.worst_stuck_vid = vid;
            }
        }
        // The alternating loop's vertex test, kept for it and for the log.
        if (vid < m_placement_pressed.size() && m_placement_pressed[vid]) {
            ++s.n_pressed; // constrained minimum: grad F is the constraint force, not a residual
            continue;
        }
        if (!std::isfinite(gn)) continue;
        ++s.n_vertices;
        if (gn > s.max_vertex) {
            s.max_vertex = gn;
            s.worst_vid = vid;
        }
    }
    for (const Tuple& e : get_edges()) {
        if (!edge_is_offset_surface_live(e)) continue;
        const size_t va = e.vid(*this), vb = e.switch_vertex(*this).vid(*this);
        if (!placed(va) || !placed(vb)) continue;
        if ((va < m_placement_pressed.size() && m_placement_pressed[va]) ||
            (vb < m_placement_pressed.size() && m_placement_pressed[vb])) {
            ++s.n_edges_pressed; // the seam: a constrained pair, not a resolution question
            continue;
        }
        const double gn = edge_conv_ratio(e); // sag / tube
        if (gn < 0.) {
            ++s.n_unmeasurable;
            continue;
        }
        ++s.n_edges;
        if (gn > s.max_edge) {
            s.max_edge = gn;
            s.worst_edge_mid =
                0.5 * (m_vertex_attribute[va].m_posf + m_vertex_attribute[vb].m_posf);
            s.worst_edge_len =
                (m_vertex_attribute[va].m_posf - m_vertex_attribute[vb].m_posf).norm();
        }
        if (gn > s.bar) {
            ++s.n_edges_over;
            if (on_level[va] && on_level[vb]) {
                ++s.n_edges_over_on_level;
                const double len =
                    (m_vertex_attribute[va].m_posf - m_vertex_attribute[vb].m_posf).norm();
                // Refinable only if the rule can still lower a target: at the sizing floor
                // (min_sizing_scalar, min_edge_length) the chord cannot be resolved within the
                // tube at this resolution -- a crease of the Euclidean level set, an inward offset
                // meeting itself -- so it is reported at-floor, not looped on.
                //
                // Against the MAX of the two scalars, not the min: split and collapse both judge
                // an edge by the MEAN of its endpoint scalars, so a min-based test excuses a chord
                // as at-floor the moment its fine end reaches sn while its coarse end keeps
                // splitting and having the children collapsed back. Judged by the max the edge
                // stays refinable while either end is coarser than the sag demands, so the coarse
                // end comes down and the split sticks; seam edges have both ends at the floor, so
                // max == min and they are still excused. Do not attack this cycle with
                // sizing_collapse_min or with min in the shared collapse length gate; both
                // measured worse -- see git history of this file.
                const double l = std::max(m_params.l, 1e-300);
                const double s_floor = std::max(
                    m_offset_params.min_sizing_scalar,
                    m_offset_params.min_edge_length / l);
                // The same rule refine_front_from_sag() applies, so the classification and
                // the refinement cannot disagree about what this chord needs.
                const double target = front_chord_target(va, vb, len, gn * s.tube, s.tube);
                const double sn =
                    std::clamp(target / l, s_floor, m_offset_params.max_sizing_scalar);
                const double have = std::max(
                    m_vertex_attribute[va].m_sizing_scalar,
                    m_vertex_attribute[vb].m_sizing_scalar);
                if (sn < have) {
                    s.refinable.push_back({va, vb, gn * s.tube, len});
                } else {
                    ++s.n_at_floor;
                }
                if (gn > s.max_edge_on_level) {
                    s.max_edge_on_level = gn;
                    s.worst_on_level_mid =
                        0.5 * (m_vertex_attribute[va].m_posf + m_vertex_attribute[vb].m_posf);
                }
            }
        }
    }
    m_phase = saved;
    return s;
}

double TopoOffsetTriMesh::phase_b_front_gradient_linf()
{
    // The vertex convergence measure over the front vertices Phase B places, as a ratio to its
    // bar (1 = bar); see front_vertex_conv_ratio(). Under gradient_norm_rel, before the
    // reference is measured, this is the raw |n . grad F| (the reference itself).
    double worst = 0.;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!m_vertex_extra[vid].m_is_on_offset || !m_vertex_attribute[vid].m_is_rounded) continue;
        if (vid < m_placement_pressed.size() && m_placement_pressed[vid])
            continue; // constrained minimum
        const double gn = m_front_gradient_reference > 0. ||
                                  m_offset_params.front_conv_criterion != "gradient_norm_rel"
                              ? front_vertex_conv_ratio(vid)
                              : front_vertex_normal_gradient(vid);
        if (gn > worst) {
            worst = gn;
            m_front_gradient_worst_vid = vid;
        }
    }
    return worst;
}

double TopoOffsetTriMesh::front_vertex_conv_ratio(const size_t vid) const
{
    const double rel = m_offset_params.front_conv_rel;
    const std::string& crit = m_offset_params.front_conv_criterion;
    if (crit == "gradient_norm_rel") {
        const double bar = rel * m_front_gradient_reference;
        return bar > 0. ? front_vertex_normal_gradient(vid) / bar
                        : std::numeric_limits<double>::infinity();
    }
    const Vector2d x = m_vertex_attribute[vid].m_posf;
    Eigen::VectorXd xv = x, g(2);
    Eigen::MatrixXd H(2, 2);
    const auto prob = phase_b_front_objective(vid, x);
    prob->gradient(xv, g);
    prob->hessian(xv, H);
    if (!g.allFinite() || !H.allFinite()) return std::numeric_limits<double>::infinity();
    Vector2d n =
        front_vertex_move_direction(vid); // the field normal, or the boundary tangent where held
    if (!(n.squaredNorm() > 0.)) n = g.normalized();
    if (!(n.squaredNorm() > 0.)) return std::numeric_limits<double>::infinity();
    const double gn = n.dot(Vector2d(g)), h = n.dot(H * n);
    if (!(h > 0.)) return gn == 0. ? 0. : std::numeric_limits<double>::infinity();
    if (crit == "step_size_rel") {
        return std::abs(gn / h) / (rel * m_offset_params.target_distance);
    }
    // decrement: half of g_n^2 / h, the energy the next Newton step still gains, against rel x F
    const double F = prob->value(xv);
    return F > 0. ? (0.5 * gn * gn / h) / (rel * F) : std::numeric_limits<double>::infinity();
}

double TopoOffsetTriMesh::edge_conv_ratio(const Tuple& e) const
{
    const double r = edge_interpolation_residual(e); // (2 w / c) |r(m) - mean r|, -1 unmeasurable
    if (r < 0.) return r;
    const double rel = m_offset_params.front_conv_rel;
    if (m_offset_params.front_conv_criterion == "gradient_norm_rel") {
        const double bar = rel * m_front_gradient_reference;
        return bar > 0. ? r / bar : std::numeric_limits<double>::infinity();
    }
    // Resolution is front_conv_rel's business: it is THE accuracy -- the vertex bar and this
    // resolution threshold alike -- while offset_envelope_rel is only the leash on the operation
    // passes. The startup check requires offset_envelope_rel <= front_conv_rel, since an accuracy
    // finer than the leash is unreachable: operations free to dent the front by more than the sag
    // threshold mint new refinable edges every turn.
    //
    // As a LENGTH: the sagitta of Phi over the chord, |Phi(m) - mean Phi|, divided by |grad Phi|
    // at the midpoint. The dimensionless form this replaced meant a tighter tolerance for the
    // smooth field than for the Euclidean one at the same number. Reported, not gated -- the
    // front's resolution is set from the tolerance in init_offset_sizing_field().
    const size_t a = e.vid(*this), b = e.switch_vertex(*this).vid(*this);
    const OffsetPotential2D& pot = potential_for_edge(a, b);
    const Vector2d pa = m_vertex_attribute[a].m_posf, pb = m_vertex_attribute[b].m_posf;
    const Vector2d m = 0.5 * (pa + pb);
    const double gn = pot.gradient(m).norm();
    if (!(gn > 0.) || !std::isfinite(gn)) return -1.;
    const double sag = std::abs(pot.value(m) - 0.5 * (pot.value(pa) + pot.value(pb))) / gn;
    return sag / (m_offset_params.front_conv_rel * m_offset_params.target_distance);
}

void TopoOffsetTriMesh::assign_band_regions()
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
    const auto region_at = [&](const Vector2d& p) -> int {
        Vector2d foot, seg_normal;
        bool on_corner = false;
        int feature = -1;
        m_input_complex_bvh->nearest_point_feature(p, foot, on_corner, seg_normal, feature);
        if (feature < 0) return -1;
        const std::vector<int64_t>& src = on_corner ? m_phi_vert_region : m_phi_seg_region;
        return size_t(feature) < src.size() ? int(src[size_t(feature)]) : -1;
    };
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
    // Diagnostic: the front objective of one vertex along its normal, its offset term alone
    // against the total, at 21 points across +-delta/2 -- to see whether a vertex that will not
    // place sits at a minimum the alignment/AMIPS terms create or is blocked by the line search.
    // Logged once, on non-convergence, for the worst vertex.
    if (vid == static_cast<size_t>(-1) || vid >= m_vertex_attribute.size() || !m_offset_potential)
        return;
    const int region = vertex_region(vid);
    const std::shared_ptr<const OffsetPotential2D> pot = potential_ptr_for(vid);
    const Vector2d x0 = m_vertex_attribute[vid].m_posf;
    Vector2d g = pot->gradient(x0);
    if (!(g.norm() > 0.) || !g.allFinite()) return;
    const Vector2d n = g / g.norm(); // toward the input for the smooth field, away for Euclidean
    const OptPhase saved = m_phase;
    m_phase = OptPhase::B;
    auto total = phase_b_front_energy(vid, pot);
    OffsetEnergy2D offset_only(pot, 1. - m_params.w_amips, true, true);
    const double delta = m_offset_params.target_distance;
    logger().info(
        "[front profile] worst vertex {} at ({:.5}, {:.5}), region {}, along the field direction "
        "n = ({:.4}, {:.4}); columns: s/delta | offset term | rest (alignment + w AMIPS) | total",
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
        const double F = total->value(xv);
        const double Fo = offset_only.value(xv);
        logger().info("[front profile] {:+.2f} | {:.6g} | {:.6g} | {:.6g}", sd, Fo, F - Fo, F);
    }
    m_phase = saved;
}

bool TopoOffsetTriMesh::front_vertex_touches_other(const size_t vid) const
{
    // Through a BACKGROUND triangle (the strip between two fronts, or between a front and a
    // wall), one of whose other vertices is on the input, on a region boundary, or on a front
    // this vertex is not joined to by a front edge -- another band's front, or this band's own
    // front across a slot. A neighbour along the same front does not count: the triangle behind
    // a front edge is the band's own.
    std::set<size_t> along;
    for (const Tuple& e : get_one_ring_edges_for_vertex(vid)) {
        if (!edge_is_offset(e.eid(*this))) continue;
        const size_t va = e.vid(*this), vb = e.switch_vertex(*this).vid(*this);
        along.insert(va == vid ? vb : va);
    }
    for (const size_t fid : get_one_ring_fids_for_vertex(vid)) {
        if (m_face_extra[fid].label != 0) continue; // band or input: not the strip
        for (const size_t u : oriented_tri_vids(fid)) {
            if (u == vid) continue;
            const VertexExtra2d& ue = m_vertex_extra[u];
            if (ue.m_is_on_input || ue.m_is_on_region) return true;
            if (ue.m_is_on_offset && along.count(u) == 0) return true;
        }
    }
    return false;
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
    // fraction, in [1/4, 1/2], and p = -1 / log2(ratio) maps it back: 1/4 -> 2, 1/2 -> 1.
    // Anything degenerate falls back to 2.
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
            p = -1. / std::log2(ratio);
        }
    }
    return std::min(0.75 * len * std::pow(tube / sag, 1. / p), 0.5 * len);
}

size_t TopoOffsetTriMesh::refine_front_from_sag(
    const std::vector<EnergyCriterion::Refinable>& edges)
{
    // See EnergyCriterion. sag = L^2 / (8 rho) for a chord of length L on curvature radius rho,
    // and the construction rule allows L = 3/4 sqrt(8 eps delta rho); substituting rho gives the
    // target 3/4 L sqrt(tube / sag) -- the same rule with the curvature read off the mesh. The
    // 3/4 is the split pass's own slack (it splits at 4/3 of the target), so an edge is asked
    // to split exactly when its sag is over the tube. Only lowers; the standard gradation.
    const double l = std::max(m_params.l, 1e-300);
    // front_conv_rel, not the envelope: the accuracy the refinement serves, the same length
    // energy_criterion() measures sag against.
    const double tube = m_offset_params.front_conv_rel * m_offset_params.target_distance;
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / l);
    std::vector<size_t> changed;
    for (const EnergyCriterion::Refinable& r : edges) {
        if (!(r.sag > 0.) || !(r.len > 0.)) continue;
        // The cap is L/2, so the children land on their own target: 25% above the collapse band
        // (4/5 x L/2) and 25% below their own split threshold (4/3 x L/2), the fixed point of the
        // 4/3-4/5 hysteresis, with a symmetric margin and no boundary. Do not raise it to
        // 0.625 L, which puts the children exactly ON the collapse band's boundary and cycles
        // them split<->collapse; measured worse -- see git history of this file. A cap of some
        // kind is required: uncapped, the refinable set never empties.
        const double target = front_chord_target(r.a, r.b, r.len, r.sag, tube);
        const double sn = std::clamp(target / l, s_floor, m_offset_params.max_sizing_scalar);
        for (const size_t v : {r.a, r.b}) {
            double& sc = m_vertex_attribute[v].m_sizing_scalar;
            if (sn < sc) {
                sc = sn;
                changed.push_back(v);
            }
        }
    }
    if (!changed.empty()) gradation_smooth_sizing(m_offset_params.sizing_gradation, changed);
    return changed.size();
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

std::tuple<double, double> TopoOffsetTriMesh::optimization_quality_stats()
{
    // Phase A is TriWild, so its metric is TriWild's: element quality alone, in absolute AMIPS
    // against optimization_stop_metric() = stop_energy, with no Phi term in the stop test, the
    // stall test or the refinement ranking. The front is not unattended there -- m_offset_envelope
    // holds it -- and mixing Phi back in makes the two criteria fight. Delegated rather than
    // reimplemented, and the units are the reason: returning a normalized number here instead
    // starves refinement, since the filter is compared against the same absolute scale.
    if (m_phase != OptPhase::B) { // A, and Single, which runs TriWild's loop
        return wmtk::TriOptimizerMesh::optimization_quality_stats();
    }

    // The engine's "quality" is the offset's own criterion: (max, avg) Phi residual over the
    // band, normalized so 1.0 is the tolerance, maxed with AMIPS on the same scale.
    const DistanceSplit r = residual_split();

    // The runaway guard, before anything reports a number derived from Phi. A vertex outside the
    // support has a residual that saturates rather than growing, so the numbers below would
    // under-report it, and no smoothing move can bring it back. Checked here because the driver
    // calls this every iteration and residual_split() has already paid for the measurement.
    report_outside_support("Optimization iteration", r);

    const double tol = offset_residual_tolerance();
    double amips = 0.;
    for (const Tuple& f : get_faces()) {
        amips = std::max(amips, quality_rel(f.fid(*this)));
    }
    const double phi = r.max_reachable / tol;
    logger().info("\t[criteria] amips {:.4}x | phi {:.4}x", amips, phi);
    return {std::max(amips, phi), std::max(amips, r.avg_reachable / tol)};
}

double TopoOffsetTriMesh::face_criterion_rel(const size_t fid) const
{
    // The per-face form of optimization_quality_stats()'s max: the same two criteria, each over
    // its own target, restricted to what this face carries. >= 1 means the face fails at least
    // one of them, which is what makes it a candidate for refinement.
    double score = quality_rel(fid);

    const double tol = offset_residual_tolerance();
    for (int j = 0; j < 3; ++j) {
        const Tuple e = tuple_from_edge(fid, j);
        if (!edge_is_offset_surface_live(e)) continue;
        for (const size_t vid : {e.vid(*this), e.switch_vertex(*this).vid(*this)}) {
            if (!band_vertex_is_reachable(vid)) continue;
            score = std::max(score, band_vertex_residual(vid) / tol);
        }
        // Along the edge as well, so a face carrying a stretch of band too coarse to represent
        // the offset is refined -- the mechanism that keeps the band resolved at all.
        score = std::max(score, offset_edge_samples(e).max / tol);
    }
    return score;
}

size_t TopoOffsetTriMesh::refine_sizing_around_worst(const double max_metric)
{
    // TriWildMesh::refine_sizing_around_worst verbatim -- ranked by m_face_attribute[].m_quality,
    // filtered against stop_energy, seeding the same force-split edges. Phase A is TriWild, and
    // that includes how it responds to a stall.
    //
    // Phase A only, by construction: mesh_improvement() is this function's one caller, and the
    // driver only ever runs that as Phase A.
    //
    // Note the units: max_metric arrives from mesh_improvement() as whatever
    // optimization_quality_stats() returned, which in Phase A is absolute AMIPS, and the score it
    // is compared against is absolute too. Mixing units here is a standing hazard.
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
    gradation_smooth_sizing(m_params.stuck_refine_gradation, refined);

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
        m_offset_params.target_distance,
        std::abs(d - m_offset_params.target_distance));
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
    const char* fate =
        "Phase A: the shared smoother, AMIPS -- the offset term is NOT what moves it";
    if (!m_vertex_attribute[vid].m_is_rounded) {
        fate = "REFUSED by smooth_before: not rounded";
    } else if (phase_places_front() && ve.m_is_on_offset) {
        fate = "Phase B / single: the local root find on (Phi - c)^2";
    } else if (phase_places_front()) {
        fate = "Phase B: interior AMIPS, or refused if it carries a surface";
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
    // The band's OUTER surface, recomputed from the tags on every call for the same reason
    // edge_is_region_boundary_live() is: the operations that ask this run between one
    // label_offset_boundary() and the next. Same rule as band_vertex_mask() -- a band face
    // meeting a face that is neither band nor input complex.
    const std::optional<Tuple> opp = e.switch_face(*this);
    if (!opp) return false;
    const size_t fa = e.fid(*this), fb = opp->fid(*this);
    const bool a = face_is_offset_band(fa), b = face_is_offset_band(fb);
    if (a == b) return false;
    return !face_is_input_complex(a ? fb : fa);
}

void TopoOffsetTriMesh::check_no_vertex_on_both_surfaces(const char* when) const
{
    // A vertex on both surfaces is unsatisfiable: it sits at distance 0 from the input complex,
    // where Phi diverges, and is at the same time required to sit at target_distance from it, so
    // no placement, no smoothing and no refinement can fix it. The rest of the component steps
    // around such a vertex (refused in Phase B, dropped from the gradient metric, booked under
    // max_pinned), which is right for the measurement and wrong as the only response -- hence
    // this check.
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

std::shared_ptr<SampleEnvelope> TopoOffsetTriMesh::released_envelope() const
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
        const double eps =
            std::max(m_offset_params.offset_envelope_rel * m_offset_params.target_distance, 1e-12);
        m_released_envelope = std::make_shared<SampleEnvelope>(/*exact=*/true);
        m_released_envelope->init(verts, segs, eps);
    }
    m_released_tube_dirty.store(false, std::memory_order_release);
    return m_released_envelope;
}

void TopoOffsetTriMesh::rebuild_offset_envelope()
{
    // The released boundaries' ops-only tube: mark and rebuild NOW, at this consistent moment
    // (this function is only called between passes), so op passes that follow judge against
    // the current shape even when no sanity sweep runs to consume the dirty flag first.
    m_released_tube_dirty.store(true, std::memory_order_release);
    released_envelope();
    // First, and on every path out of here including the empty one: each entry is an
    // IntersectionEnvelope holding the tube this call is about to replace, so a survivor would
    // hold a simplex to where the front was one round ago -- a constraint tightening silently
    // every round. See m_offset_isect_cache.
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
        // Nothing to hold. Not an error: a run whose offset region never formed has other
        // problems, and they are reported where they happen.
        m_offset_envelope = nullptr;
        logger().warn("\t[offset envelope] no offset-boundary segments; the envelope is empty");
        return;
    }

    std::vector<Eigen::Vector2d> verts(vert_capacity());
    for (size_t i = 0; i < vert_capacity(); ++i) {
        verts[i] = m_vertex_attribute[i].m_posf;
    }

    // One Phi tolerance by default, so Phase A may move the front by exactly as much as the
    // convergence test is willing to ignore. Tighter and Phase A cannot improve the elements
    // straddling the front at all; looser and it can undo a Phi that Phase B had already brought
    // inside tolerance.
    //
    // A straight fraction of target_distance and nothing else: both are distances in model units,
    // so offset_envelope_rel is a pure percentage. Deliberately independent of the convergence
    // criterion -- chaining it to a criterion that is itself a fraction of a measured reference
    // would make the Phase A tube depend on how bad construction happened to be. And it is NOT
    // envelope_size_rel, a fraction of the bounding-box diagonal, which is what m_envelope (the
    // input-complex tube) is built from.
    const double eps =
        std::max(m_offset_params.offset_envelope_rel * m_offset_params.target_distance, 1e-12);

    m_offset_envelope = std::make_shared<SampleEnvelope>(/*exact=*/true); // see the tag envelopes
    m_offset_envelope->init(verts, segs, eps);
    logger().info(
        "\t[offset envelope] rebuilt: {} segments, {} (eps {:.6g} = "
        "offset_envelope_rel {:.4} x target_distance {:.6g})",
        segs.size(),
        m_offset_envelope->use_exact ? "EXACT" : "sampled",
        eps,
        m_offset_params.offset_envelope_rel,
        m_offset_params.target_distance);
}

void TopoOffsetTriMesh::append_frame_label(const size_t idx, const std::string& label) const
{
    // See write_smoothing_debug_output(). Truncated on the first frame of the run, appended to
    // afterwards; a run with no debug output never creates it.
    std::ofstream f(
        m_offset_params.output_path + "_frames.txt",
        idx == 0 ? std::ios::trunc : std::ios::app);
    if (f) f << fmt::format("{:05d}\t{}\n", idx, label);
}

void TopoOffsetTriMesh::optimize_offset_single_phase()
{
    // One phase. Phase A is already TriWild's mesh_improvement -- split / smooth / collapse /
    // smooth / swap / smooth -- so operations and smoothing interleave there already; the only
    // things Phase B adds are WHICH objective a front vertex is smoothed against and that it is
    // not caged in the offset tube while it moves. Give A's smoothing passes both
    // (OptPhase::Single) and the second phase has nothing left to do. The tube still holds the
    // front for the OPERATIONS (surface_envelope_for_edge, which does not read the phase) and is
    // rebuilt after every iteration, so it follows the front rather than capping it.
    const int rounds = std::max(1, m_offset_params.max_rounds);
    const int a_iters = std::max(1, m_offset_params.max_iterations);
    check_no_vertex_on_both_surfaces("construction");
    log_region_edge_mask_health("construction");
    audit_surface_containment("construction");
    needle_scan("after construction, before the single-phase loop");
    assign_band_regions();
    m_phase = OptPhase::B; // the reference is measured with the offset terms present
    m_front_gradient_reference = phase_b_front_gradient_linf();
    logger().info(
        "\tSINGLE PHASE: TriWild's loop with the front placed inside its "
        "smoothing passes | front energy-gradient reference {:.6g}, criterion {} at rel {}",
        m_front_gradient_reference,
        m_offset_params.front_conv_criterion,
        m_offset_params.front_conv_rel);
    // Its own key, max_rounds: one turn here is split | collapse | swap with a smoothing pass
    // after each, which is neither an A/B round nor a Phase A iteration, so borrowing either key
    // gives a stuck case a budget meant for something else. max_iterations still bounds the
    // finishing pass, which is an ordinary TriWild run.
    (void)rounds;
    const int budget = std::max(1, m_offset_params.max_rounds);
    // One turn is TriWild's operation groups, run here rather than through mesh_improvement() so
    // the tube can be rebuilt AFTER EVERY SMOOTHING PASS. Only smoothing moves the front -- an
    // accepted split, collapse or swap cannot take a front edge out of the tube -- so the tube
    // goes stale only across a smooth, and one rebuild per turn leaves collapse and swap judging
    // the front against a tube it has already left. local_operations() is the engine's own pass
    // driver; what mesh_improvement() adds and is left out here on purpose is its stall response,
    // which refines around the worst elements: a travelling front stretches cells by design.
    // The smoothing count is interleaved_smoothing_passes, the key the shared loop reads for the
    // same thing, so one key sets it for the pre-optimisation pass, the finishing pass and here.
    const int k = std::max(1, m_params.interleaved_smoothing_passes);
    const std::array<std::array<int, 4>, 3> groups = {
        {{{1, 0, 0, k}}, {{0, 1, 0, k}}, {{0, 0, 1, k}}}}; // split | collapse | swap, each + smooth
    partition_mesh_morton();
    // One turn of grace after a target is lowered: refine_front_from_sag() writes a sizing target
    // and the split pass that realizes it does not run until the NEXT turn, so a turn that
    // classifies every chord as at-floor may be looking at a mesh not yet refined to the targets
    // already written for it, and exiting there would end the run with the last write unspent.
    // Terminates for the same reason the rule itself does: the scalars only ever fall and stop at
    // the floor, so a turn that writes nothing always comes.
    size_t lowered_last_turn = 0;
    for (int it = 0; it < budget; ++it) {
        m_ab_round = it + 1;
        m_iterations_used = it + 1;
        m_phase = OptPhase::Single;
        for (const Tuple& v : get_vertices()) {
            const size_t vid = v.vid(*this);
            m_vertex_extra[vid].m_turn_start = m_vertex_attribute[vid].m_posf;
            m_vertex_extra[vid].m_turn_start_valid = true;
        }
        rebuild_offset_envelope();
        for (const auto& ops : groups) {
            stamp_plastic_rests(); // plastic: each group resists only its own increment
            local_operations(ops);
            rebuild_offset_envelope(); // the smoothing in this group moved the front
        }
        consolidate_mesh();
        assign_band_regions();
        const double amips = std::get<0>(optimization_quality_stats());
        const double bar = optimization_stop_metric();
        const EnergyCriterion ec = energy_criterion();
        const Vector2d wx = ec.worst_vid != static_cast<size_t>(-1)
                                ? m_vertex_attribute[ec.worst_vid].m_posf
                                : Vector2d::Zero();
        logger().info(
            "======== single-phase turn {} / {}: max AMIPS {:.4} (stop {:.4}) | front vertices "
            "max {:.4}x the bar (worst v{} at ({:.4}, {:.4})), edges max {:.4}x (reported) | {} "
            "vertices, {} edges | edges over the tube: {}, of which {} with both ends on the "
            "level set (worst {:.4}x at ({:.4}, {:.4})) | states: placed {} pressed {} travelling "
            "{} "
            "stuck {} | refinable edges {} (at the sizing floor {}) ========",
            it + 1,
            budget,
            amips,
            bar,
            ec.max_vertex,
            ec.worst_vid,
            wx.x(),
            wx.y(),
            ec.max_edge,
            ec.n_vertices,
            ec.n_edges,
            ec.n_edges_over,
            ec.n_edges_over_on_level,
            ec.max_edge_on_level,
            ec.worst_on_level_mid.x(),
            ec.worst_on_level_mid.y(),
            ec.n_placed,
            ec.n_pressed_on,
            ec.n_travelling,
            ec.n_stuck,
            ec.refinable.size(),
            ec.n_at_floor);
        if (m_offset_params.debug_output) {
            write_smoothing_debug_output(fmt::format("phase_{}S", it + 1));
        }
        // The resolution rule, every turn: see EnergyCriterion and refine_front_from_sag().
        const size_t lowered_prev = lowered_last_turn;
        lowered_last_turn = 0;
        if (!ec.refinable.empty()) {
            const size_t n = refine_front_from_sag(ec.refinable);
            lowered_last_turn = n;
            logger().info(
                "\t[resolution] turn {}: {} front edge(s) with both ends on the level set sag over "
                "the tube (worst {:.4}x at ({:.4}, {:.4})) -> target lowered at {} vertices",
                it + 1,
                ec.refinable.size(),
                ec.max_edge_on_level,
                ec.worst_on_level_mid.x(),
                ec.worst_on_level_mid.y(),
                n);
        }
        if (ec.n_stuck > 0) {
            const Vector2d sp = m_vertex_attribute[ec.worst_stuck_vid].m_posf;
            logger().info(
                "\t[stuck] turn {}: {} front vertex(es) stopped short of the level set touching "
                "nothing; worst v{} at ({:.4}, {:.4}), {:.4} x delta off",
                it + 1,
                ec.n_stuck,
                ec.worst_stuck_vid,
                sp.x(),
                sp.y(),
                ec.worst_stuck_rho / m_offset_params.target_distance);
        }
        // Termination: every front vertex placed or pressed, none travelling or stuck, and no
        // chord left to resolve -- then quality with the front frozen (below).
        if (ec.converged_single() && lowered_prev == 0) {
            m_energy_verdict = ec;
            m_converged = true;
            logger().info(
                "Single phase: the front is placed after {} iteration(s) (phi {:.4}x); max AMIPS "
                "{:.4} against stop {:.4}",
                it + 1,
                ec.max_vertex / ec.bar,
                amips,
                bar);
            if (amips >= bar) {
                logger().info(
                    "======== final pass, front frozen: max AMIPS {:.6g} >= stop_energy {} "
                    "========",
                    amips,
                    m_params.stop_energy);
                m_ab_round = it + 2;
                m_phase = OptPhase::A;
                // The tube first. Only smoothing moves the front -- an accepted operation cannot
                // leave the tube -- so one rebuild after the smoothing passes is enough, which
                // the loop above does at the top of every iteration. This pass is the exception:
                // it starts after the last iteration's smoothing, so without this it would judge
                // the front against a stale tube.
                rebuild_offset_envelope();
                m_freeze_front = true;
                // Pure TriWild: the pass exists to reach stop_energy, and a rest-shape term pulls
                // background vertices toward non-equilateral shapes. No plasticity here.
                const bool plastic_was = m_plastic_active;
                m_plastic_active = false;
                mesh_improvement(a_iters);
                m_plastic_active = plastic_was;
                m_freeze_front = false;
                assign_band_regions();
                const double final_amips = std::get<0>(optimization_quality_stats());
                logger().info(
                    "\t[final pass] max element quality {:.4} (stop {:.4}) -> {}",
                    final_amips,
                    optimization_stop_metric(),
                    final_amips < m_params.stop_energy ? "ok" : "STILL OVER");
                if (m_offset_params.debug_output) {
                    write_smoothing_debug_output(fmt::format("phase_{}A", it + 2));
                }
            }
            rebuild_offset_envelope();
            return;
        }
    }
    logger().warn("Single phase did not converge in {} turns (max_rounds)", budget);
    log_front_profile(energy_criterion().worst_vid);
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

    // deform_others: from here on, other input regions deform instead of being envelope-held.
    // After the labels (the held/released decision reads them), before the offset envelope and
    // the sizing seed, so the whole optimization sees one consistent world.
    if (m_offset_params.deform_others) {
        release_deformable_regions();
        // Plastic only where there is something to push. The medium is made plastic so a released
        // object can be carried out of the front's way; with nothing released it buys nothing and
        // costs real quality, since a plastic ambient has no shape preference and a pressed seam's
        // strip crushes instead of holding. An empty released set leaves the feature inert, which
        // is what a scene with no other objects should get.
        if (!m_deform_tags.empty()) {
            m_plastic_active = true;
            stamp_plastic_rests();
        }
    }

    // The offset envelope is born here, with the offset itself, and always exists from then on,
    // refreshed around wherever the last pass left the front. Its lifetime and the constraint's
    // applicability are separate questions -- the phase test lives in exactly one place,
    // containment_for() -- so nothing can silently read "no constraint" for "not applicable in
    // this phase".
    rebuild_offset_envelope();

    // The front as constructed must already be inside the potential's support, or nothing the
    // optimization does can move it. Checked before any operation runs so a construction defect
    // is reported as one.
    check_offset_within_support("Offset as constructed");

    // Spelled out so the line reproduces its own number. The reference is measured on the band
    // this function is still building, so this states the shape of the bound and the line that
    // follows states the value.
    logger().info(
        "\tOffset criterion: |grad (Phi - c)^2 . n| <= front_conv_rel {} x "
        "max|grad (Phi - c)^2 . n| over the band AS CONSTRUCTED, with n the unit normal from "
        "the offset surface's own normal (Voronoi-weighted at vertices, the edge's own inside "
        "an edge). Measured over every band vertex and {} sample(s) "
        "per band edge; the reference is reported next, before the loop starts.",
        m_offset_params.front_conv_rel,
        offset_residual_samples());

    // Seed the sizing field from the offset's current edge lengths, before any operation runs.
    // Must come after the offset edges are classified above, since it reads them.
    init_offset_sizing_field();

    // Unconditional, and it must stay that way: write_vtu() calls consolidate_mesh(), which
    // renumbers the mesh, which changes the order every subsequent pass enumerates operations in,
    // which changes the run. With the debug write below the only consolidate here, turning debug
    // output on silently produced a different numerical result.
    consolidate_mesh();
    if (m_offset_params.debug_output) {
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    // The shared engine's own loop, driven alternately rather than jointly -- see OptPhase. Each
    // Phase A is one mesh_improvement() with a TriWild criterion; each Phase B is smoothing to a
    // fixed point.
    //
    // The offset plugs into mesh_improvement() through the same virtuals simwild uses:
    //   - optimization_quality_stats(): per phase -- TriWild's own in A, the max of AMIPS and the
    //     offset residual in B;
    //   - optimization_stop_metric(): per phase, in the SAME units as the line above;
    //   - refine_sizing_around_worst(): TriWild's, fired only on a stall.
    // Placement is not among them: the front is placed by Phase B's own local solve.
    iter_cnt_split = 0;
    iter_cnt_split_born = 0;
    iter_cnt_recollapsed = 0;
    iter_cnt_recollapsed_same_pass = 0;
    iter_cnt_collapse = 0;
    iter_cnt_collapse_offset_removed = 0;
    iter_cnt_swap = 0;
    m_smooth_trace.reset();
    optimization_metrics.clear();
    op_counts.clear();
    churn_counts.clear();

    // Frame 0 is the mesh as constructed, before the optimization touches it: the shared driver
    // only writes a frame after each operation pass, so without this there is nothing to compare
    // the timeline against.
    if (m_params.debug_output) {
        m_debug_pass_name = "construction";
        write_smoothing_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
    }

    optimize_offset_single_phase();

    // Cumulative over the whole run, not per iteration: the engine loop has no per-iteration
    // hook, and the per-pass numbers it logs itself carry the history.
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
    // No push_back here: op_counts is a per-round series recorded inside the driver loop, so
    // appending the run totals would make the last entry mean something different from every
    // other one. The run total is the sum of the series.

    // Final metrics and the convergence verdict, one entry for the whole run. Convergence is
    // max_grad and nothing else -- the full placement-gradient norm at band vertices, the same
    // test every Phase B visit stopped on, and the same statement for any potential. The in-edge
    // samples, the Phi residual and the Euclidean error are all diagnostics.
    assign_band_regions();
    const auto [max_dist, avg_dist] = compute_distance_deviation();
    const DistanceSplit r = residual_split();
    const GradientSplit g = gradient_split();
    const double tol = offset_residual_tolerance();
    const double gtol = offset_gradient_tolerance();
    logger().info(
        "placement gradient (at band vertices): max {} (avg {}) vs tolerance {} "
        "[front_conv_rel {}] | in-edge diagnostic {} ({} edge samples) | {} "
        "reachable, {} pinned (max {}), {} skipped ({} unrounded, {} inverted ring)",
        g.max_reachable,
        g.avg_reachable,
        gtol,
        m_offset_params.front_conv_rel,
        g.max_in_edge,
        g.n_edge_samples,
        g.n_reachable,
        g.n_pinned,
        g.max_pinned,
        g.n_skipped_unrounded + g.n_skipped_inverted,
        g.n_skipped_unrounded,
        g.n_skipped_inverted);
    logger().info(
        "phi residual (diagnostic, absolute model units): max {} (avg {}) vs bar {} | at "
        "vertices {}, inside edges {} | {} samples, {} pinned vertices || euclid dist err: max {} "
        "| avg {}",
        r.max_reachable,
        r.avg_reachable,
        tol,
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

    {
        // Measured at convergence when the loop converged (see m_energy_verdict), else now.
        const EnergyCriterion ec = m_energy_verdict ? *m_energy_verdict : energy_criterion();
        m_converged = ec.converged_single();
        logger().log(
            m_converged ? spdlog::level::info : spdlog::level::warn,
            "{}{}: front vertices placed {} / pressed {} / travelling {} / stuck {} | "
            "chords to resolve {} (at the sizing floor {}) | accuracy front_conv_rel {} "
            "x target_distance = {:.4} | (vertex test, informative: max {:.4}x its bar)",
            m_converged ? "Converged" : "Optimization did not converge",
            m_energy_verdict ? " (measured at convergence, before the finishing pass)" : "",
            ec.n_placed,
            ec.n_pressed_on,
            ec.n_travelling,
            ec.n_stuck,
            ec.refinable.size(),
            ec.n_at_floor,
            m_offset_params.front_conv_rel,
            ec.tube,
            ec.max_vertex);
    }

    // Escalate to a hard failure if the caller asked for it, AFTER the warnings above so the log
    // still names which criterion missed before the throw.
    if (!m_converged && m_offset_params.throw_on_nonconvergence) {
        log_and_throw_error(
            "Optimization did not converge and throw_on_nonconvergence is set. Ran {} of {} "
            "iterations; see the warnings above for the criterion that failed.",
            optimization_metrics.size(),
            m_offset_params.max_iterations);
    }

    // The tracked surfaces are deliberately NOT re-derived here. From the moment they are tagged
    // the shared operations maintain those tags; the face labels they were derived from are
    // construction data the optimization does not propagate, so re-deriving now would read stale
    // labels and mislabel the result.
}

} // namespace wmtk::components::topological_offset
