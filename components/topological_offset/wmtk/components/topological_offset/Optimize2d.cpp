#include <wmtk/utils/AMIPS2D.h>
#include "TopoOffsetTriMesh.h"

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SizingField.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

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
    // THE EDGE'S OWN TRACKING RECORD, not a comparison of the incident faces' RAW TAG SETS.
    //
    // The tag comparison this replaced (2026-08-23) was the last functional reader of the input
    // tag sets left in the optimization phase, and it disagreed with the tracking in exactly the
    // place that matters. execute_offset() REPLACES the tags of every face the band grows
    // through, so a region boundary the band swallowed ends up with identical tag sets on both
    // sides -- the comparison then answers "not on a surface" for an edge that
    // init_surfaces_and_boundaries() classified, that carries m_is_surface_fs, and that is held
    // in its tags' envelopes. substructure_link_condition() was therefore evaluating the
    // collapse pass against a substructure missing those edges. Measured on two_circles: 81 of
    // 257 tracked region edges had an empty live tag difference by the time the optimization
    // started, which is the same divergence log_region_edge_mask_health() counts.
    //
    // m_is_surface_fs does not have that failure mode: it is assigned once from the INPUT
    // partition and then propagated by every operation -- the split copies the parent onto both
    // children and resets its cross edges (TriOptimizerMeshSplit.cpp:305-309), the collapse
    // merges and migrates (TriOptimizerMeshCollapse.cpp:227, :338), and the swap restores the
    // ring and resets the new diagonal (TriOptimizerMesh.cpp:336-341). The base's own
    // is_edge_on_surface() already asks exactly this question, so delegating also stops the two
    // from being able to disagree.
    //
    // The domain wall keeps its answer: init_surfaces_and_boundaries() tracks a wall edge like
    // any other region boundary, so the `!opp` short-circuit above is now belt-and-braces rather
    // than the only thing that classified it.
    //
    // Spelled out rather than delegating to is_edge_on_surface(vids) so the tuple lookup the
    // guard above already paid for is reused; the base additionally demands both endpoints'
    // m_is_on_surface, which is implied here -- every operation that sets m_is_surface_fs on an
    // edge sets that flag on both its ends.
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
    // ONCE, AT THE TOP OF THE OPTIMIZATION, and never again -- exactly the 3D flow. This used to
    // run at the start of every iteration and re-derive ALL THREE tracked classes plus the bbox
    // flags from the face labels, clearing them first. Two things were wrong with that. The
    // labels are CONSTRUCTION data the optimization does not propagate, so re-reading them after
    // an operation pass reads stale values; and the region boundaries and the wall are now
    // classified once from the INPUT partition in init_surfaces_and_boundaries(), so clearing
    // them here would throw away the only classification the per-tag envelopes are keyed on.
    //
    // What is left is what 3D does here: upgrade the OFFSET boundary to its own class. The
    // region/input edges keep the primary class 0, so they are envelope-checked by the shared
    // operations exactly as in triwild and simwild.

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
            // The band ran into the domain wall. There is no second face to compare against, so
            // this cannot be classified as offset boundary -- and it must not be, or the wall
            // segment would lose the per-tag containment that keeps the box a box. Counted so a
            // clipped band is visible here as well as in
            // warn_if_offset_reaches_domain_boundary().
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
            // The base's UNION flag, and the one the shared operations actually read.
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
    // TRIWILD PARITY: no offset-criterion capture here and no comparison in the `after` hook.
    // A second criterion on top of the envelope made Phase A stricter than TriWild in exactly
    // the phase documented as "Phase A is TriWild"; the envelope is the constraint and the
    // criterion belongs to Phase B. Same removal as 3D -- see Swap.cpp.
    if (!TriOptimizerMesh::swap_edge_before(t)) {
        return false;
    }

    // No special wall handling: a wall edge is m_is_surface_fs-tracked (the base refuses
    // tracked-surface edges), has no second face (the `!opp` test below refuses the flip
    // structurally), and its endpoints' mask differs from any interior apex pair (the junction
    // rule at the bottom refuses again). Three independent reasons it cannot flip, none of
    // them a bbox special case.

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

    // A flip across a JUNCTION would detach the new diagonal from one of the boundaries the old
    // edge lay on: refuse when the endpoints' masks differ from the apexes'. The 3D twin is the
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

    // The offset side. PHASE A ONLY -- Phase B is the pass that has to move the boundary, and a
    // tube around where it currently sits would cap how far it can travel. Null before the
    // first rebuild, which is the pre-pass, when there is no offset yet.
    const bool hold_offset = on_offset && m_phase == OptPhase::A && m_offset_envelope != nullptr;

    if (!hold_offset) return region; // may itself be null: nothing contains this simplex
    if (!region) return m_offset_envelope;

    // ON BOTH: the intersection, inside every tube it lies on. Memoized per region mask because
    // the second member is the same object for all of them; rebuild_offset_envelope() clears
    // the map, so an entry can never outlive the tube it was built around.
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

    // THE REAL MEMBERS, never envelope_for_mask()'s composite -- see the declaration for why
    // nearest_point() on an IntersectionEnvelope dereferences a null BVH.
    std::vector<const SampleEnvelope*> members;
    for (const auto& [tag, env] : m_tag_envelopes) {
        const auto it = m_tag_bit.find(tag);
        if (it != m_tag_bit.end() && (mask & (uint64_t(1) << it->second))) {
            members.push_back(env.get());
        }
    }
    if (members.empty()) return true; // every bit dangled: no tube was ever built for them

    // ALTERNATING PROJECTION, worst violation first. A single tube needs one round, because
    // nearest_point() lands x ON the curve and the tube contains its own curve; a junction needs
    // a few, and converges toward the point where its curves meet.
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

    // Did not settle in kMaxRounds. Two tubes that do not actually intersect near x would do
    // this, which is a construction question, not something to paper over by committing the last
    // iterate: the caller keeps the entry position instead.
    for (const SampleEnvelope* e : members) {
        if (e->is_outside(x)) return false;
    }
    return true;
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

    // START ON THE CURVE. The foot, not x itself: the vertex is inside a tube of half-width eps,
    // so it is generally a little off the polyline, and arclength is only defined on it. This is
    // also what makes the walk idempotent -- a vertex already placed by a previous pass returns
    // to the same foot.
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

    // MARCH. At each step, walk to the end of the current segment in `dir`; if the budget runs
    // out first, land inside it and stop. Otherwise step onto the shared vertex and pick the
    // continuation. A vertex with exactly two incident segments has one; anything else -- an
    // open end, or three curves meeting -- is where the walk stops, because there is no
    // unambiguous continuation and guessing one would slide the vertex onto a different curve.
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
    // TRIWILD PARITY: no offset-criterion acceptance for surface flips. The shared swap has
    // already checked both new segments against their envelopes (see swap_edge_before()); a
    // second criterion on top of that is what this used to be, and it is gone with the collapse
    // one -- see collapse_edge_after().
    ++iter_cnt_swap;
    return true;
}

bool TopoOffsetTriMesh::collapse_edge_after(const Tuple& t)
{
    if (!TriOptimizerMesh::collapse_edge_after(t)) {
        return false;
    }
    // TRIWILD PARITY: no offset-criterion acceptance test in the loop.
    //
    // This used to apply a non-degrading Phi gate to every collapse (refuse if the worst offset
    // face at the survivor got worse), captured per-edge in collapse_before_vertex(). That was
    // load-bearing when the offset boundary had NO envelope holding it. Phase A now holds it in
    // m_offset_envelope (rebuilt each round, eps = ab_offset_envelope_rel x target_distance
    // tolerance), so containment is enforced by the shared pass's surface_segment_is_outside() --
    // the same mechanism, with the same tolerance semantics, that TriWild's input gets. A
    // second, tighter criterion on top of the envelope made Phase A stricter than TriWild in
    // exactly the phase that is documented as "Phase A is TriWild". Phase B, which owns the Phi
    // criterion, re-places the boundary every round. Same removal as 3D.
    //
    // COARSENING keeps an ABSOLUTE bar. It runs after the loop has finished and trades elements
    // for nothing except the promise that the result is still good: a collapse that leaves any
    // offset face over tolerance afterwards is not a saving, it is a regression with fewer
    // elements. face_criterion_rel() is the max of both criteria (AMIPS and the offset
    // residual), each normalized so 1.0 is its own tolerance.
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
    return true;
}

bool TopoOffsetTriMesh::collapse_edge_before(const Tuple& t)
{
    // DIAGNOSTIC, off by default. See ab_no_collapse_after_first_round: it switches the collapse
    // half of the band's split/collapse stalemate off from round 2, to establish whether that
    // stalemate is what holds the residual up. It gives up everything coarsening provides and is
    // not a fix.
    if (m_ab_collapses_disabled) {
        return false;
    }

    if (!TriOptimizerMesh::collapse_edge_before(t)) {
        return false;
    }
    // Unconditionally, not just when both endpoints are already on a tracked simplex -- see
    // the declaration. substructure_link_condition() evaluates the condition for the mesh and
    // for every substructure, which is what a topological offset needs preserved.
    if (!substructure_link_condition(t)) {
        return false;
    }
    return true;
}

bool TopoOffsetTriMesh::collapse_before_vertex(const size_t v1_id, const size_t v2_id)
{
    // GENESIS BOOKKEEPING (diagnostic): the flattest face this collapse is about to reshape,
    // read back by record_flatness() in collapse_after_vertex().
    {
        double f = 1.;
        for (const size_t fid : get_one_ring_fids_for_vertex(tuple_from_vertex(v1_id))) {
            f = std::min(f, face_flatness(fid));
        }
        m_collapse_parent_flatness.local() = f;
    }

    const auto& VE = m_vertex_extra;

    // v1 is the vertex the collapse REMOVES (it merges into v2, which keeps its position).
    //
    // A vertex on ANY tracked boundary -- input complex, region boundary, domain wall alike --
    // may be removed, provided it merges onto a vertex of the same class (the per-class rules
    // below for the flags; the base's on_bbox_faces SUBSET rule for the wall,
    // TriOptimizerMeshCollapse.cpp, which also keeps corners), the result stays inside its
    // tags' envelopes (the shared collapse's containment check), and the substructure topology
    // survives (substructure_link_condition, applied unconditionally in collapse_edge_before).
    // That is TriWild's rule for its input surface, applied uniformly. The wall used to be
    // refused outright here -- "the box is not something to be coarsened" -- which was a
    // special case the envelope accounting makes redundant, contradicted the contract stated
    // in init_surfaces_and_boundaries(), and had no 3D counterpart.

    // THE INVARIANT: never both surfaces on one vertex. Construction cannot make such a vertex,
    // and a collapse is the only thing that could manufacture one -- it merges v1 into v2 and
    // collapse_after_vertex() ORs the flags, so an offset vertex merged into an input-complex
    // one (or the reverse) would come out carrying both. Such a vertex sits at distance 0 from
    // the complex and is asked to sit at target_distance from it at the same time; no placement
    // satisfies that, so the offset boundary through it can never converge.
    //
    // Refused here rather than repaired afterwards, and asserted independently by
    // check_no_vertex_on_both_surfaces() after construction and after every Phase A.
    {
        const bool input = VE[v1_id].m_is_on_input || VE[v2_id].m_is_on_input;
        const bool offset = VE[v1_id].m_is_on_offset || VE[v2_id].m_is_on_offset;
        if (input && offset) {
            return false;
        }
    }

    // THE OFFSET BOUNDARY IS ALWAYS LENGTH-LIMITED, whatever the pass says.
    //
    // Every other tracked surface here is held by its tags' envelopes, which bound how far it
    // can be decimated no matter what the length gate is doing. The offset boundary deliberately
    // has no envelope in Phase B -- it is the surface the optimization exists to MOVE -- so its
    // sizing field is the only thing bounding its resolution, and a pass that switches the
    // length gate off removes that too. mesh_improvement() runs exactly such a pass twice,
    // `it pre` and `it post`.
    //
    // Measured on topological_offset_2d_vertex_input: `it pre` alone took the mesh from 2619 to
    // 462 vertices and the band's max distance error from 0.125 to 0.25 -- exactly
    // target_distance, i.e. a band vertex had been collapsed onto the input complex.
    if (!m_collapse_limit_length && VE[v1_id].m_is_on_offset) {
        return false;
    }

    // The base only knows that both endpoints are on SOME tracked surface. A vertex may not
    // leave the particular surface it belongs to: an input-complex vertex carries input
    // geometry, an offset-boundary vertex carries the offset, and a region-boundary vertex
    // carries the outline of a tag region the output is built from. Each is checked separately
    // because a vertex can be on more than one, and satisfying the union is not enough.
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
    // NEEDLE TRIPWIRE (diagnostic). Runs after the collapse is committed, so what it sees is
    // real; the survivor's ring is every face the collapse reshaped. No parent quality to quote
    // -- collapse_quality_allowed() already carries that accounting.
    for (const size_t fid : get_one_ring_fids_for_vertex(tuple_from_vertex(v2_id))) {
        if (get_quality(fid) >= kNeedleQuality) report_needle("COLLAPSE", fid, -1.);
        record_flatness("COLLAPSE", m_collapse_parent_flatness.local(), fid);
    }

    if (m_vertex_extra.at(v1_id).m_is_on_offset) ++iter_cnt_collapse_offset_removed;
    // CHURN: v1 is the vertex being REMOVED. If a split created it, this collapse is undoing
    // that split. Same epoch means the collapse pass that immediately follows its own split
    // pass took it straight back out -- work the mesh never got to keep. A larger age means it
    // survived at least one further iteration before something decided against it.
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

    // The base calls this only once a collapse has actually gone through, so it is the 2D
    // equivalent of the 3D counter's home in collapse_edge_after().
    ++iter_cnt_collapse;
}

void TopoOffsetTriMesh::split_after_vertex(const size_t v_id)
{
    // THE NEW VERTEX'S CLASSIFICATION IS NOT SET HERE ANY MORE -- see split_adjust_position(),
    // which the base calls BEFORE its own containment check. This hook runs after it, and the
    // check dispatches on exactly those bits, so setting them here left the check reading a
    // recycled slot. Only the birth epoch is left, which nothing upstream reads.
    //
    // CHURN INSTRUMENTATION, read only by collapse_after_vertex(). Assigned rather than OR'd
    // because v_id may be a recycled slot carrying a dead vertex's bits.
    m_vertex_extra[v_id].m_born_epoch = m_op_epoch;
    if (m_op_epoch != 0) ++iter_cnt_split_born;

    // DEGENERACY ATTRIBUTION (diagnostic, see the header). Every face incident to the midpoint
    // was created by this split -- the base replaced the parents outright -- so a MAX_ENERGY
    // face here is one this split just manufactured. A split is never refused on quality, so
    // nothing upstream would have stopped it.
    const auto& sc = m_opt_split_cache.local();
    const double parent_q = sc.parent_q_max;
    for (const size_t fid : get_one_ring_fids_for_vertex(tuple_from_vertex(v_id))) {
        const double q = get_quality(fid);
        if (q >= MAX_ENERGY) ++m_deg_split_created;
        if (q >= kNeedleQuality) report_needle("SPLIT", fid, parent_q);
        record_flatness("SPLIT", sc.parent_flatness, fid);
    }

    // The children's region labels are NOT set here -- see split_adjust_position(), which the
    // base calls early enough for the split's own containment check to see them.
}

bool TopoOffsetTriMesh::split_adjust_position(const size_t v_id, const std::vector<Tuple>&)
{
    // Carry each parent's construction label onto the two children it became.
    //
    // A split is the only operation that creates faces, and a child is geometrically inside its
    // parent, so it belongs to whatever region the parent did -- there is no ambiguity to
    // resolve. What made this necessary rather than automatic is that the children land in
    // FRESH fid slots, whose m_face_extra defaults to label 0 (or, for a slot recycled from an
    // earlier collapse, to the previous occupant's label). Neither is the parent's.
    //
    // split_edge_before() records the parents' labels keyed by APEX -- the vertex opposite the
    // split edge, which both children of the same parent share and which no other parent has.
    //
    // Resolved here the way TriOptimizerMesh::split_edge_after resolves its OWN FaceAttributes
    // cache, which is keyed identically: walk the children of each new edge (v1,v_new) and
    // (v2,v_new) and take the vertex opposite THAT edge. Scanning a child's three vertices for
    // any key instead happens to give the same answer on a manifold mesh, but it is a search
    // rather than the map the base uses, and it silently does nothing when it finds no key.
    // Going through the same expression as the tags means the label cannot land anywhere the
    // tags did not.
    //
    // Writing before the base's own checks is safe: m_face_extra is registered with
    // m_face_attr_group, so a refused split rolls it back. It is also idempotent -- two of the
    // four children reuse their parent's fid, and get written the value already there.
    const auto& c = m_opt_split_cache.local();
    if (c.face_label.empty()) return true; // a marching-mode split; those set labels themselves

    // THE NEW VERTEX'S CLASSIFICATION, AND IT HAS TO BE HERE. The base's split_edge_after()
    // asks surface_segment_is_outside() about both new segments (TriOptimizerMeshSplit.cpp:297
    // and :300) and that reaches surface_envelope_for_edge(), which dispatches on the
    // ENDPOINTS' own bits: edge_mask() reads m_is_on_region and m_boundary_mask, the offset
    // branch reads m_is_on_offset. Those used to be written in split_after_vertex(), which the
    // base calls at :339 -- AFTER the check. So the check ran against whatever the recycled
    // vertex slot happened to hold, which for a fresh slot is all-false: both branches fell
    // through, the dispatch returned nullptr, and surface_segment_is_outside() answers FALSE
    // for a null envelope. The containment check on a split was therefore a NO-OP, silently,
    // and a split was free to put a tracked segment outside its tube. Measured on two_circles
    // (ESP, pre-optimized input): one offset edge left the Phase A tube in round 2 and the
    // sanity check reported it as "Edge [...] is outside!" -- the split that created it had
    // been asked and had said yes.
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
    // ... but m_is_on_region is the SPLIT EDGE'S OWN class, not an endpoint AND: a bare AND
    // over-claims on a chord whose two ends happen to share a region, which is what the mask
    // gate exists to prevent. The 3D twin reads is_edge_on_region() for the same reason.
    m_vertex_extra[v_id].m_is_on_region =
        e.m_is_surface_fs && e.m_surface_class != OFFSET_SURFACE_CLASS;
    // ASSIGNED, not OR'd -- v_id may be a recycled slot carrying a dead vertex's bits. The bits
    // are the ENDPOINTS' mask AND, captured in split_edge_before() (OptSplitCache2d::edge_bits);
    // the class gate right above is what keeps a chord's midpoint maskless. Zero when the new
    // vertex is on no region: the mask is inert there.
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

    // NEEDLE/SMOOTHING INSTRUMENTATION (diagnostic). Recorded for every visit; only visits whose
    // ring already holds a needle are counted, and smooth_after() reads this back.
    auto& pre = m_needle_pre.local();
    pre = {ring_max_quality(vid), m_vertex_attribute[vid].m_posf};
    if (pre.first >= kNeedleQuality) ++m_needle_smooth_offered;

    // THE BASE'S smooth_before MINUS ITS BOUNDING-BOX REFUSAL, which is why this does not call
    // it. The base rounds the vertex and then refuses outright any vertex with a non-empty
    // on_bbox_faces, i.e. it FREEZES the domain wall. Here the wall is a region boundary held in
    // ambient's tag envelope like every other one, so its vertices are smoothed and the
    // containment check decides whether the move survives -- exactly the contract the input
    // complex already gets. What keeps the wall a wall is that check, not immobility. Same
    // change as 3D; see TopoOffsetTetMesh::smooth_before() for the degeneracy it fixed.
    //
    // Rounding still has to happen, and its failure still refuses the move.
    const bool rounded_now = round(t);
    if (!m_vertex_attribute[vid].m_is_rounded && !rounded_now) {
        ++m_smooth_trace.before_unrounded;
        return false;
    }

    // PHASE B RUNS IN TWO ORDERED SUB-SWEEPS; SEE PhaseBSub.
    //
    // A pass places every offset-boundary vertex first, then relaxes the background under the
    // boundary those placements just defined. This admission is what splits them -- each
    // sub-sweep refuses the other's class outright.
    //
    // ENVELOPES ARE THE OTHER AXIS. Phase B runs with the offset envelope RELEASED and takes no
    // containment responsibility, so a vertex an envelope is supposed to hold has no one to hold
    // it here: an OFFSET vertex that is also envelope-held is a case this scheme does not handle
    // and it THROWS rather than being silently skipped; a BACKGROUND vertex that is
    // envelope-held is skipped, and left where Phase A put it.
    if (m_phase == OptPhase::B) {
        const bool is_offset = m_vertex_extra[vid].m_is_on_offset && m_offset_potential;
        // HELD MEANS AN ENVELOPE ACTUALLY HOLDS IT -- not that a flag says the vertex is on a
        // region. The two are not the same set: vertex_is_on_region() reads m_is_on_region,
        // which is deliberately over-broad (splits take it from the edge's persistent class,
        // collapses OR it onto survivors), while the envelope is dispatched from the boundary
        // MASK, seeded once from the INPUT partition and propagated exactly. Asking the
        // dispatch is also the only phrasing that cannot drift from it.
        //
        // Measured before the gate was rewritten: 5 offset vertices per pass on two_circles
        // carried the flag with mask 0x0 -- held by nothing, skipped anyway, never placed in
        // any pass of any round. (At the time the masks themselves also decayed through a
        // live-tag rederivation at the split sites; both are fixed, and the audit below keeps
        // both honest.)
        const bool enveloped = smoothing_containment_envelope(vid) != nullptr;

        if (m_phase_b_sub == PhaseBSub::Offset) {
            if (!is_offset) {
                ++m_smooth_trace.before_phase_b_not_offset;
                return false;
            }
            if (enveloped) {
                // PINNED, AND NOT PLACED AT ALL. This is the default as of 2026-08-23 and it is a
                // retreat, deliberately: an offset vertex that a region envelope also holds has
                // two requirements that are not simultaneously satisfiable -- stay within
                // envelope_size of the input region boundary, and sit on Phi = c, which is
                // target_distance away -- and NONE of the three placements written for it works.
                // See "OPEN PROBLEMS" in .claude/CLAUDE.md for the measurements; in short, on
                // topo_annots_groups (tag_0 & tag_2, delta 1.2, rel 1e-3) refusal froze four such
                // vertices in every pass of every round, projection-to-the-curve made the run
                // WORSE (130.6x vs 106.7x) because nearest_point lands at distance 0 when the
                // tube permits eps, and the tangential arclength solve froze the offset outright
                // on an anchoring bug that is understood but not fixed.
                //
                // So the vertex is left where Phase A put it AND booked unreachable -- see
                // band_vertex_is_reachable(), which is the half that matters. Skipping alone
                // would be the worst of both: the vertex never moves and still gates max_grad,
                // which is precisely the state that made the run above unconvergeable. Pinning
                // says the honest thing instead: this vertex's containment provably prevents it
                // reaching the level set, so it is reported and excluded rather than chased.
                //
                // THE PLACEMENT CALL IS KEPT, COMMENTED OUT, one line below. It is the thing to
                // restore first when the anchoring fix lands; leaving it in place keeps the two
                // halves of the decision next to each other rather than in a git log.
                ++m_smooth_trace.before_phase_b_enveloped_offset;
                return false;
                // return true; // <-- RESTORE THIS to place enveloped offset vertices again
                //                    (smooth_offset_vertex_backtracking's tangential branch)
            }
            return true;
        }

        // PhaseBSub::Background
        if (is_offset || m_vertex_attribute[vid].m_is_on_surface) {
            ++m_smooth_trace.before_phase_b_not_offset;
            return false;
        }
        if (enveloped) {
            // See the stencil in smooth_interior_vertex_phase_b().
            ++m_smooth_trace.before_phase_b_enveloped_background;
            return false;
        }
    }
    return true;
}

bool TopoOffsetTriMesh::smooth_after(const Tuple& t)
{
    const size_t vid = t.vid(*this);
    const auto& ve = m_vertex_extra[vid];

    // NEEDLE/SMOOTHING INSTRUMENTATION (diagnostic), the other half of smooth_before()'s record.
    {
        const auto& pre = m_needle_pre.local();
        if (pre.first >= kNeedleQuality) {
            ++m_needle_smooth_reached;
            const double after = ring_max_quality(vid);
            const double moved = (m_vertex_attribute[vid].m_posf - pre.second).norm();
            if (after < kNeedleQuality) ++m_needle_smooth_fixed;
            if (moved < 1e-12) ++m_needle_smooth_stationary;
            if (m_needle_smooth_reports.fetch_add(1) < 8) {
                logger().warn(
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

    // TWO PHASES; IN B, TWO SOLVES, AND NO AMIPS IN THE OFFSET PLACEMENT.
    //
    // Phase B places offset-only vertices with the offset potential alone: the 1-D minimization
    // of (Phi - c)^2 along grad Phi, backtracked into the element by bisection if the minimum
    // would invert the ring. Its interior vertices minimize their one-ring AMIPS to their own
    // minimum. smooth_before() has already refused everything else in this phase, so the branch
    // below is exhaustive.
    //
    // WHY THE OFFSET SOLVE TAKES NO AMIPS. Blending w_amips * AMIPS into an offset vertex's
    // objective leaves it resting a w_amips-proportional distance off the level set -- the
    // at-vertex wall. And AMIPS alone cannot be dropped from the shared solve either: the offset
    // energy's Hessian is 2*w*g*g^T, rank 1, with a 1-D nullspace along the level set's tangent,
    // so a 2-D minimize of it is ill-posed. The 1-D root find sidesteps both.
    if (m_phase == OptPhase::B) {
        const bool ok = ve.m_is_on_offset ? smooth_offset_vertex_backtracking(t)
                                          : smooth_interior_vertex_phase_b(t);
        if (ok && ve.m_is_on_offset) ++m_smooth_trace.offset_accepted;
        return ok;
    }

    // PHASE A is TriWild: the shared smoother, with the offset boundary held by
    // m_offset_envelope and carrying no offset term of its own.
    const double before = ve.m_is_on_offset ? band_vertex_residual(vid) : 0.;
    const bool ok = TriOptimizerMesh::smooth_after(t);
    if (!ve.m_is_on_offset) {
        return ok;
    }
    const double after = band_vertex_residual(vid);
    if (ok) ++m_smooth_trace.offset_accepted;

    // NOTE the residuals are read BEFORE the caller's rollback, so `after` is the position the
    // smoother proposed, not necessarily the one kept. That is deliberate -- it separates "the
    // solver could not find a better place" from "it found one and the accept checks refused
    // it" -- but it means the two numbers must always be read next to the accepted count.
    const auto nano = [](double x) { return static_cast<long long>(std::min(x, 1e9) * 1e9); };
    m_smooth_trace.res_before_nano += nano(before);
    m_smooth_trace.res_after_nano += nano(after);
    atomic_max(m_smooth_trace.res_max_before_nano, nano(before));
    atomic_max(m_smooth_trace.res_max_after_nano, nano(after));
    return ok;
}

Vector2d TopoOffsetTriMesh::offset_vertex_normal(const size_t vid) const
{
    // See the declaration: the single definition of an offset vertex's normal.
    //
    // Two implementations, selected by WMTK_OFFSET_NORMAL so one build can measure both. The
    // DEFAULT is the input-complex projection; `WMTK_OFFSET_NORMAL=surface` picks the other.
    // They agree wherever the offset is a clean outward front and diverge at the medial axis,
    // which is the only place worth comparing them.
    static const bool s_surface_normal = [] {
        const char* e = std::getenv("WMTK_OFFSET_NORMAL");
        return e && std::string(e) == "surface";
    }();

    if (s_surface_normal) {
        // ---- ALTERNATIVE: the offset surface's own normal ---------------------------------
        // Continuous across the medial axis, but tied to the current triangulation and zero for
        // a vertex with no live offset edge.
        const Vector2d n = offset_surface_normal(vid);
        const double len = n.norm();
        if (len > 0.) return Vector2d(n / len);
        return Vector2d::Zero();
    }

    // ---- DEFAULT: project to the input complex -------------------------------------------
    // The direction the offset grew along, taken from the geometry rather than from the mesh.
    // The BVH is the same structure Phi queries, so the foot point is the exact nearest point on
    // the complex as loaded. Flips discontinuously across the medial axis, where the nearest
    // feature changes -- which is exactly the regime this toggle exists to measure.
    if (m_input_complex_bvh) {
        const Vector2d x = m_vertex_attribute[vid].m_posf;
        const Vector3d foot = m_input_complex_bvh->nearest_point(x);
        const Vector2d d(x.x() - foot.x(), x.y() - foot.y());
        const double len = d.norm();
        if (len > 0.) return d / len;
    }
    return Vector2d::Zero();
}

Vector2d TopoOffsetTriMesh::offset_surface_normal(const size_t vid) const
{
    // See the declaration. Sum of unit edge normals weighted by half the edge length, which in
    // 2D is the Voronoi cell of `vid` along the offset polyline.
    const Vector2d x = m_vertex_attribute[vid].m_posf;
    Vector2d n = Vector2d::Zero();
    for (const Tuple& e : get_one_ring_edges_for_vertex(vid)) {
        if (!edge_is_offset_surface_live(e)) continue;
        const size_t a = e.vid(*this), b = e.switch_vertex(*this).vid(*this);
        const size_t other = (a == vid) ? b : a;
        if (other == vid) continue;
        const Vector2d tvec = m_vertex_attribute[other].m_posf - x;
        const double len = tvec.norm();
        if (!(len > 0.)) continue;
        Vector2d ne(-tvec.y() / len, tvec.x() / len); // quarter turn: a normal of this edge

        // ORIENT IT OUTWARD, away from the band. Without this the two edges at a vertex can
        // contribute opposite normals and cancel to nothing on a straight stretch, which is
        // precisely where the normal is best defined.
        const std::optional<Tuple> opp = e.switch_face(*this);
        const size_t fa = e.fid(*this);
        size_t band_fid = fa;
        if (!face_is_offset_band(fa)) {
            if (!opp) continue; // no band side to orient against
            band_fid = opp->fid(*this);
            if (!face_is_offset_band(band_fid)) continue;
        }
        const std::array<size_t, 3> tri = oriented_tri_vids(band_fid);
        Vector2d centroid = Vector2d::Zero();
        for (const size_t v : tri) centroid += m_vertex_attribute[v].m_posf;
        centroid /= 3.;
        const Vector2d mid = x + 0.5 * tvec;
        if (ne.dot(mid - centroid) < 0.) ne = -ne;

        n += 0.5 * len * ne;
    }
    const double nn = n.norm();
    return (nn > 0.) ? Vector2d(n / nn) : Vector2d::Zero();
}


bool TopoOffsetTriMesh::smooth_offset_vertex_backtracking(const Tuple& t)
{
    const size_t vid = t.vid(*this);
    const std::vector<size_t>& locs = get_one_ring_fids_for_vertex(t);

    // INSTRUMENTATION. Every exit below books a PlacementStop, so "placed" stops meaning "did
    // not throw" and starts meaning something. See the enum for why the four zero-gradient cases
    // are separated. `ex` is filled in as the entry quantities become known and is what the
    // per-reason exemplar is taken from.
    const bool reachable = band_vertex_is_reachable(vid);
    PlacementTrace::Exemplar ex;
    ex.vid = vid;
    const auto book = [&](const PlacementStop stop) {
        const Vector2d& p = m_vertex_attribute[vid].m_posf;
        ex.x = p.x();
        ex.y = p.y();
        m_placement_trace.record(stop, reachable, ex);
        // The one input to pin_interference_stalled_vertices() that only this function knows:
        // whether the vertex's own step found any descent. One write per vertex per pass.
        if (vid < m_placement_stalled.size()) {
            m_placement_stalled[vid] = (stop == PlacementStop::MidStationary) ? 1 : 0;
        }
    };

    if (locs.empty()) {
        book(PlacementStop::NoRing);
        return false;
    }

    // Same entry guard the shared smoother applies: a one-ring already inverted in floats has no
    // valid segment to search, and the exact predicate below would reject every candidate.
    for (const size_t fid : locs) {
        if (is_inverted_f(fid)) {
            ++m_smooth_rejects.already_inverted;
            ++m_phase_b_constrained; // cannot even attempt its minimum
            book(PlacementStop::PreInverted);
            return false;
        }
    }

    const Vector2d x_orig = m_vertex_attribute[vid].m_posf;

    // FIXED-STEP GRADIENT DESCENT ON E = (Phi - c)^2, STOPPED ON THE RUN'S OWN CONVERGENCE BAR.
    //
    // The step direction is -grad E / |grad E| with grad E = 2 (Phi - c) grad Phi, and the step
    // LENGTH is a fixed fraction of target_distance -- so every step moves the vertex the same
    // distance and the sequence is a walk down E rather than a Newton-like jump to its root.
    // This replaces a Levenberg-Marquardt step; the damped step solved for the minimum in one
    // move where a root existed, which is faster but couples the step size to |grad Phi| and so
    // to how close the vertex already is.
    //
    // THE STOPPING TEST IS THE CONVERGENCE CRITERION AT THIS VERTEX, exactly:
    //
    //     q(x) = || 2 (Phi(x) - c) grad Phi(x) ||  <=  offset_gradient_tolerance()
    //
    // the FULL norm of grad E against the run's own bar -- convergence_gradient_norm_rel x
    // m_gradient_reference, the same comparison gradient_split() makes when the run is judged.
    // One parameter, one bar, one quantity: the run has converged precisely when every one of
    // these visits terminates immediately, so the local solves cannot walk to a fixed point
    // the run then rejects, nor the run accept a surface a visit would still move. (The
    // REFERENCE the bar is a fraction of is normal-aligned -- max |grad E . n| over the
    // initial surface, see measure_gradient_reference() -- because "how misplaced did this
    // start" is a question about motion along the normal. The running test is the full norm.)
    //
    // This replaced two earlier stops. A per-visit RELATIVE test, q <= ab_vertex_grad_tol_rel *
    // q(x_entry), had no absolute floor and manufactured a limit cycle: a vertex 650x INSIDE
    // the run's bar still owed a further 100x reduction of its own entry value, could not
    // resolve it (the fixed step h changes |Phi - c| by h x |grad Phi| per move, ~100x more
    // than the target), and burned all 2000 iterations oscillating around the level set to end
    // exactly where it started -- 180 of 203 visits per pass on two_circles, ~9M wasted field
    // evaluations per run. Then a NORMAL-PROJECTED test, |grad E . n| against the bar: it
    // matched the criterion of the day at vertices, but that criterion also gated on
    // edge-interior samples no visit could move, so runs parked every vertex exactly AT the
    // bar and failed overall on a chord term a few percent above it. ab_vertex_grad_tol_rel
    // now governs only the interior AMIPS Newton, where the quantity is an energy gradient and
    // the run's Phi bar means nothing.
    //
    // ONE GAUSS-SEIDEL ITERATION PER VISIT, not a solve. The phase sweeps every eligible vertex
    // once per pass and repeats up to ab_phase_b_iterations times, so a vertex advances
    // alongside its neighbours instead of running to a fixed point they have not seen. Solving
    // each vertex to convergence is what let two approaching offset fronts each drive to the
    // same curve and crush the elements between them into zero-area triangles.
    //
    // THERE IS NO STEP-LENGTH CONSTANT ANY MORE. The step is a damped Newton step with an Armijo
    // line search, so its length comes from the local model; see the walk below for why the
    // hard-coded kStepFrac x delta existed and why it no longer has a reason to.

    // Exact orientation test on the rational position, exactly as the shared smoother's accept
    // test is. Leaves the vertex AT `p`; callers set the final position explicitly.
    const auto inverts = [&](const Vector2d& p) {
        set_vertex_position(vid, p);
        for (const size_t fid : locs) {
            if (is_inverted(fid)) return true;
        }
        return false;
    };

    // THE OBJECTIVE: AMIPS + AN OFFSET TERM, the offset term selected by WMTK_OFFSET_PLACEMENT.
    //
    //     F(x) = w_amips * sum_ring AMIPS(x) + (1 - w_amips) * <offset term>
    //
    //   DEFAULT (unset, or =quadratic)   the plain quadratic error (Phi - c)^2.
    //
    //   =custom          the NORMAL-PROJECTED GRADIENT of the quadratic error. EXPERIMENTAL --
    //                    off by default; it is here to be measured, not relied on. Last measured
    //                    on two_circles at delta 0.1 (offset_out_d0p1_ridge_*): it fixes nothing
    //                    on its own -- phase B declares a fixed point after ONE pass per round,
    //                    hands a barely-moved offset to phase A, and phase A runs away. It does
    //                    stay inside the support, which the unconstrained version did not.
    //
    //
    //       G(x) = ( (Phi(x) - c) * (grad Phi(x) . n_i) )^2
    //
    //     n_i is THIS vertex's normal (offset_vertex_normal), measured once at entry and held
    //     FIXED for the visit -- it is a per-vertex direction, not a field, so G's gradient
    //     carries no dn/dx term and is exact:
    //
    //       grad G = 2 s [ grad Phi (grad Phi . n) + (Phi - c) H n ],  s = (Phi - c)(grad Phi . n)
    //
    //     with H = hessian(x), which the potential supplies analytically. G vanishes on exactly
    //     the two places an offset vertex is allowed to rest:
    //
    //       Phi = c            the level set itself, where the offset belongs; and
    //       grad Phi . n = 0   no further progress along the normal -- the RIDGE. Where two
    //                          fronts approach, Phi in the corridor never falls to c (two
    //                          circles at delta = 0.1: Phi at the gap midpoint is 2c), so there
    //                          is no level set to reach and the front should settle at the
    //                          corridor's saddle, which is precisely this condition.
    //
    //     THE CONSTRAINT IS WHAT MAKES THOSE THE ONLY TWO. G has a third zero with no geometric
    //     meaning: outside dhat, Phi == 0 AND grad Phi == 0 identically, so G == 0 -- its global
    //     minimum, sitting in the flat region the potential does not reach. Worse, the level set
    //     is a zero of value 0 with no barrier past it, so a vertex could drift outward across
    //     it on AMIPS pressure alone and coast to that flat region. Measured, before the
    //     constraint: 20 offset vertices escaped to 4.74x target_distance and the run aborted.
    //     So the walk is confined to the CLOSED OFFSET REGION {Phi >= c}. Inside it every zero
    //     of G is one of the two above: Phi = c is its boundary, and an interior grad Phi.n = 0
    //     with Phi > c is a genuine local minimum of Phi along n, i.e. the ridge. A vertex that
    //     starts outside (construction can leave one there) is not frozen -- the floor is
    //     min(0, r_entry), so it may always improve, never worsen.
    //
    //     Where n_i is unavailable (zero vector -- no live offset edge for the surface
    //     normal, or a vertex sitting exactly on the complex for the projected one) the term
    //     falls back to the quadratic error and the visit is counted in m_placement_no_normal,
    //     so the fallback is never silent.
    //
    //   =descent         (Phi - c)^2 alone, w_amips = 0, stopped on the run's global bar --
    //                    the original pure descent, and the whole of this function's history below.
    //
    // WHY AMIPS IS BACK. The pure offset energy's Hessian is 2 w g g^T: rank 1, with a 1-D
    // nullspace along the level set's tangent, so a 2-D minimisation of it is ill-posed and the
    // solve only ever had a direction because the step was normalised. AMIPS supplies curvature
    // in exactly that nullspace. The price, which is why it was removed before, is that a
    // w_amips-weighted shape term leaves the vertex resting a w_amips-proportional distance OFF
    // the level set -- the at-vertex wall. w_amips = 1e-4 (OptimizerParameters' default, shared
    // with the smoother) is the same compromise the envelope smoothing already makes.
    //
    // The AMIPS stencil puts the MOVING vertex first in each cell, which is what
    // AMIPS2D_jacobian differentiates with respect to (see optimization/AMIPSEnergy.cpp).
    enum class OffTerm { Quadratic, GradProj, PureDescent };
    static const OffTerm s_off_term = [] {
        const char* e = std::getenv("WMTK_OFFSET_PLACEMENT");
        const std::string v = e ? std::string(e) : std::string();
        if (v.empty() || v == "quadratic") return OffTerm::Quadratic;
        if (v == "custom") return OffTerm::GradProj;
        if (v == "descent") return OffTerm::PureDescent;
        // NOT silent: a typo in an experiment knob otherwise reads as "the default was fine".
        logger().warn(
            "WMTK_OFFSET_PLACEMENT='{}' is not a known offset term (quadratic | custom | "
            "descent); using quadratic.",
            v);
        return OffTerm::Quadratic;
    }();
    const bool s_pure_descent = (s_off_term == OffTerm::PureDescent);
    const double w_amips = s_pure_descent ? 0. : m_params.w_amips;
    const double w_off = 1. - w_amips;

    // n_i, fixed for the whole visit -- see the objective note above. Measured BEFORE the entry
    // sample because grad_norm_at() reads it.
    const Vector2d n_entry = offset_vertex_normal(vid);
    const bool use_grad_proj = (s_off_term == OffTerm::GradProj) && (n_entry.norm() > 0.);
    if (s_off_term == OffTerm::GradProj && !use_grad_proj) ++m_placement_no_normal;

    std::vector<std::array<double, 6>> ring;
    if (!s_pure_descent) {
        ring.reserve(locs.size());
        for (const size_t fid : locs) {
            const auto vs = oriented_tri_vids(fid);
            int k = 0;
            while (k < 3 && vs[k] != vid) ++k;
            if (k == 3) continue; // not incident; cannot happen, but do not fabricate a cell
            const Vector2d& a = m_vertex_attribute[vs[(k + 1) % 3]].m_posf;
            const Vector2d& b = m_vertex_attribute[vs[(k + 2) % 3]].m_posf;
            ring.push_back({{0., 0., a.x(), a.y(), b.x(), b.y()}});
        }
    }

    // q at the CURRENT position: ||grad F||. The PARTS are kept as well as the product, because
    // q == 0 has three different causes and only r and |grad Phi| separately tell them apart.
    struct QSample
    {
        double q = -1.; ///< ||grad F||, or -1 where the field is not finite
        double r = 0.;
        Vector2d g = Vector2d::Zero(); ///< grad F
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero(); ///< grad^2 F, for the Newton step
    };
    // THE FIELD THIS VERTEX IS PLACED ON: its own region's, see m_region_potentials.
    const OffsetPotential2D& pot = potential_for(vid);
    const auto grad_norm_at = [&](const Vector2d& p) -> QSample {
        QSample sm;
        sm.r = pot.value(p) - pot.target_level();
        const Vector2d gphi = pot.gradient(p);
        if (!std::isfinite(sm.r) || !gphi.allFinite()) return sm; // q stays -1
        const Eigen::Matrix2d Hphi = pot.hessian(p);
        if (!Hphi.allFinite()) return sm;
        Vector2d gF;
        Eigen::Matrix2d HF;
        if (use_grad_proj) {
            // G = s^2 with s = (Phi - c)(grad Phi . n); see the objective note above.
            const double gdotn = gphi.dot(n_entry);
            const double sgn = sm.r * gdotn;
            const Vector2d ds = gdotn * gphi + sm.r * (Hphi * n_entry);
            gF = w_off * 2. * sgn * ds;
            // GAUSS-NEWTON on the least-squares form G = s^2: drop s * grad^2 s, which needs
            // the third derivative of Phi. Exact at a zero of s -- which is where this term is
            // trying to land -- and positive semi-definite everywhere, so it never contributes
            // an indefinite block for the shift below to undo.
            HF = w_off * 2. * ds * ds.transpose();
        } else {
            gF = w_off * 2. * sm.r * gphi;
            // grad^2 (Phi - c)^2 = 2 (grad Phi grad Phi^T + (Phi - c) grad^2 Phi). The first term
            // alone is rank 1 -- singular along the level set's tangent -- which is exactly why
            // this solve used to have no usable Newton step and carried a hard-coded length
            // instead. AMIPS below fills that nullspace.
            HF = w_off * 2. * (gphi * gphi.transpose() + sm.r * Hphi);
        }
        if (w_amips > 0.) {
            Vector2d gA = Vector2d::Zero();
            Eigen::Matrix2d HA = Eigen::Matrix2d::Zero();
            for (std::array<double, 6> cell : ring) {
                cell[0] = p.x();
                cell[1] = p.y();
                Vector2d j;
                Eigen::Matrix2d h;
                wmtk::AMIPS2D_jacobian(cell, j);
                wmtk::AMIPS2D_hessian(cell, h);
                // A degenerate cell has no usable AMIPS derivative.
                if (!j.allFinite() || !h.allFinite()) return sm;
                gA += j;
                HA += h;
            }
            gF += w_amips * gA;
            HF += w_amips * HA;
        }
        if (!gF.allFinite() || !HF.allFinite()) return sm;
        sm.g = gF;
        sm.q = gF.norm();
        sm.H = HF;
        return sm;
    };

    // The entry value, measured with the vertex where it started.
    set_vertex_position(vid, x_orig);
    const QSample entry = grad_norm_at(x_orig);
    const double q_entry = entry.q;

    // THE STOP IS RELATIVE TO THIS VISIT'S OWN ENTRY GRADIENT, one parameter:
    //     ||grad F(x)|| <= convergence_gradient_norm_rel * ||grad F(x_entry)||
    // In WMTK_OFFSET_PLACEMENT=descent mode it is the run's global bar instead, as before.
    //
    // KNOWN RISK, recorded so it is recognised if it returns: an entry-relative rule has no
    // absolute floor, and the last time this function used one it limit-cycled -- a vertex far
    // inside the run's bar still owed a further reduction of its own entry value, could not
    // resolve it at the FIXED step, and burned all 2000 iterations oscillating to end where it
    // started (180 of 203 visits per pass on two_circles). Two things are different now: F
    // carries AMIPS, so the fixed point is a real 2-D minimum rather than a point on a 1-D
    // valley floor; and the step is line-searched, so it cannot overshoot a minimum it can see.
    // The visit also takes only ONE step, so a limit cycle would show up as a pass count that
    // never converges rather than as wasted evaluations inside a visit.
    const double q_tol =
        s_pure_descent ? offset_gradient_tolerance()
                       : m_offset_params.convergence_gradient_norm_rel * std::max(q_entry, 0.);

    // THE WALK'S FEASIBLE SET: r = Phi - c may not fall below this. Zero for a vertex at or
    // inside the level set, so it is confined to {Phi >= c}; the entry value for one that starts
    // outside, so it can always come back but never go further out. Only the normal-projected
    // term needs it -- (Phi - c)^2 grows on both sides of the level set and confines itself --
    // so the other two modes are left unconstrained and keep their measured behaviour exactly.
    const double r_floor =
        use_grad_proj ? std::min(0., entry.r) : -std::numeric_limits<double>::infinity();

    // THE CONTAINMENT CONSTRAINT, for an offset vertex that a region envelope ALSO holds.
    //
    // ENFORCED BY PROJECTION AFTER THE STEP, not by refusing trial points during it
    // (2026-08-23). The step below is the SAME unconstrained Gauss-Seidel step on the SAME
    // objective every other offset vertex takes; project_into_containment() then restores
    // validity. See its declaration for the measurement that forced the change -- refusal froze
    // four junction vertices on topo_annots_groups for ten entire rounds.
    //
    // Read as a BOOLEAN here and nothing more. The projection walks the mask's real member
    // envelopes itself, because a junction's containment is an IntersectionEnvelope and
    // TagEnvelopes.hpp forbids asking a composite anything but is_outside().
    const bool has_containment = vertex_boundary_mask(vid) != 0;
    // THE CURVE THIS VERTEX SLIDES ALONG, and the incident REGION edges whose chords bound how
    // far it may slide. Resolved once, before the search, because both are fixed for the visit.
    const int64_t curve_tag = has_containment ? tangent_curve_tag(vid, x_orig) : int64_t(-1);
    std::vector<size_t> region_nbrs;
    if (curve_tag >= 0) {
        for (const Tuple& e : get_one_ring_edges_for_vertex(vid)) {
            const size_t eid = e.eid(*this);
            if (!edge_is_region(eid)) continue;
            const size_t a = e.vid(*this), b = e.switch_vertex(*this).vid(*this);
            region_nbrs.push_back(a == vid ? b : a);
        }
    }
    const bool tangential = curve_tag >= 0;
    // ALREADY OUTSIDE ON ENTRY. No longer disables anything -- the projection FIXES such a
    // vertex rather than having to route around it -- but still counted, because a vertex that
    // starts outside its own tube is a construction or Phase A defect and the invariant is zero.
    if (has_containment) {
        const std::shared_ptr<SampleEnvelope> containment = smoothing_containment_envelope(vid);
        if (containment && containment->is_outside(x_orig)) ++m_placement_env_entry_outside;
    }

    // n_entry is measured above, next to the objective it now feeds. Under the default term it
    // is part of F; under =quadratic and =descent it is a DIAGNOSTIC only -- the exemplar's
    // |cos(grad Phi, n)| column, which says how tangential an entry gradient was.
    ex.res = entry.r;
    ex.gnorm = entry.g.norm();
    ex.nnorm = n_entry.norm();
    ex.cosn = (ex.gnorm > 0. && ex.nnorm > 0.)
                  ? std::abs(entry.g.dot(n_entry)) / (ex.gnorm * ex.nnorm)
                  : 0.;

    // F itself, needed by the line search's sufficient-decrease test. Infinite where the field
    // or a cell is unusable, so a trial point there simply fails the test and gets halved away.
    const auto value_at = [&](const Vector2d& p) -> double {
        const double inf = std::numeric_limits<double>::infinity();
        const double r = pot.value(p) - pot.target_level();
        if (!std::isfinite(r)) return inf;
        double F;
        if (use_grad_proj) {
            const Vector2d gphi = pot.gradient(p);
            if (!gphi.allFinite()) return inf;
            const double sgn = r * gphi.dot(n_entry);
            F = w_off * sgn * sgn;
        } else {
            F = w_off * r * r;
        }
        for (std::array<double, 6> cell : ring) {
            cell[0] = p.x();
            cell[1] = p.y();
            const double a = wmtk::AMIPS2D_energy(cell);
            if (!std::isfinite(a)) return inf;
            F += w_amips * a;
        }
        return F;
    };

    // THE BAR PHASE A ENFORCES, APPLIED TO THE PLACEMENT STEP. Phase A ends every round with
    // max AMIPS <= stop_energy; the placement's only shape guard was the exact inversion test, so
    // where the field's level set is unreachable -- two fronts meeting, where Phi_A + Phi_B > c
    // across the whole gap -- the offset term pushed both fronts through the background strip
    // until inversion, and Phase A then remeshed the crushed strip into flat faces every round
    // (two_circles at target_distance 0.1: strip width 0.0009, 626 coincident vertex pairs, 92
    // folded edges, edge error cycling 143x -> 65x -> 52x -> 79x, never a joint fixed point).
    // A trial step may worsen the ring, but not past stop_energy, and not make a face already
    // over it worse: the step is SHRUNK, not refused, so the front presses up to the bar and
    // stops there -- the same state Phase A accepts, so the alternation can settle. TriWild's
    // own smoother never needed this: it descends on AMIPS alone and cannot raise it.
    const auto ring_max_at = [&](const Vector2d& p) -> double {
        double q = 0.;
        for (std::array<double, 6> cell : ring) {
            cell[0] = p.x();
            cell[1] = p.y();
            const double a = wmtk::AMIPS2D_energy(cell);
            if (!std::isfinite(a)) return std::numeric_limits<double>::infinity();
            q = std::max(q, a);
        }
        return q;
    };
    // ONE SWITCH, because it was asked whether the per-region fields made this redundant.
    // Measured 2026-08-24 with it OFF, fields per region: tangent cases (delta 0.1) and 0.04
    // still converge, but every case where the fronts actually push into each other fails --
    // smooth 0.15: strip crushed to 0.002, worst quality 76, folds of 164 deg, no convergence;
    // Euclidean 0.15: RUNAWAY. (smooth 0.02 also loses convergence, but only to one 6e-4-long
    // edge folded across the curve at (1.459, 0.954) -- 2.5% of delta, invisible at the
    // viewer's scale; a measured difference, not a visible kink.) The field fixes the
    // DIRECTION of the push, this bar is the STOP. Both are needed.
    constexpr bool kPlacementQualityBound = true;
    constexpr double kPlacementQualityBoundFactor =
        1.; ///< the bar is this x stop_energy; 2x and 10x measured worse, see below
    const double q_bar = kPlacementQualityBoundFactor * m_params.stop_energy;
    const double q_bound = kPlacementQualityBound ? std::max(q_bar, ring_max_at(x_orig))
                                                  : std::numeric_limits<double>::infinity();
    const auto over_bound = [&](const Vector2d& p) -> bool { return !(ring_max_at(p) <= q_bound); };
    bool bounded = false; ///< some valid trial step was refused by the bar

    Vector2d x_last_inside = x_orig; ///< THE LAST POSITION KNOWN TO KEEP THE ONE-RING VALID
    bool left_ring = false;
    int iters = 0; ///< accepted steps, for the trace
    PlacementStop stop = PlacementStop::Moved;
    if (q_entry > q_tol) {
        // ONE GAUSS-SEIDEL ITERATION: a DAMPED NEWTON STEP with an Armijo backtracking line
        // search. The step length comes out of the local quadratic model and the line search --
        // there is no dialled-in length, because a fixed length is not an iteration of anything.
        //
        // WHY THERE USED TO BE ONE. With the objective (Phi - c)^2 alone, grad^2 F = 2 grad Phi
        // grad Phi^T + 2 (Phi - c) grad^2 Phi, whose leading term is RANK 1: singular along the
        // level set's tangent, so Newton had no direction there and the code kept only the
        // normalised gradient plus an arbitrary kStepFrac x delta. AMIPS is back in F now and it
        // supplies curvature in exactly that nullspace, so the model is non-degenerate and the
        // hard-coded length has no reason to survive.
        //
        // The shift below makes that robust rather than assumed: it does not matter whether
        // AMIPS happens to fill the nullspace at this particular vertex.
        const QSample cur = entry;
        if (cur.q < 0.) {
            stop = PlacementStop::MidNonFinite;
        } else if (!(cur.q > 0.)) {
            stop = PlacementStop::MidStationary;
        } else {
            // DIRECTION. Shift the Hessian to positive definite -- 2x2, so the eigenvalues are
            // exact and cheap -- and solve H d = -g. With lam_min already positive this is the
            // plain Newton direction; where the model is flat or indefinite the shift is what
            // turns it into a descent direction, degrading smoothly to steepest descent as the
            // shift grows. Scale-aware floor: a fixed epsilon would mean different things on
            // meshes of different size.
            bool handled = false;
            if (tangential) {
                // ---- CODIMENSION-1 SOLVE, on the boundary curve itself ----
                //
                // The constraint is ELIMINATED, not enforced: the vertex is parameterized by
                // arclength along its tag's boundary polyline, so every point the search visits
                // lies ON the curve and its own containment holds by construction. What is left
                // to bound is the INCIDENT CHORDS -- the mesh edges from this vertex to its
                // region-boundary neighbours, which are straight and so leave the tube once the
                // vertex slides far enough past a corner. That bound is found by the same
                // backtracking as everything else, which is why no corner needs an angle
                // threshold: it comes out at roughly eps/sin(theta) for a turn of theta, and is
                // absent along a straight run.
                //
                // phi(s) = F(walk(s)); phi' = tau . g and phi'' = tau^T H tau, both already in
                // hand from the 2-D sample, so the Newton step is the 1-D one.
                handled = true;
                Vector2d tau;
                if (!curve_tangent(curve_tag, x_orig, Vector2d(-cur.g), tau)) {
                    stop = PlacementStop::EnvelopeBlocked;
                } else {
                    const double d1 = tau.dot(cur.g);
                    const double d2 = tau.dot(cur.H * tau);
                    double s_step =
                        (d2 > 0.)
                            ? (-d1 / d2)
                            : (-d1 / std::max(
                                         std::abs(d2),
                                         cur.q / std::max(m_offset_params.target_distance, 1e-16)));
                    if (!std::isfinite(s_step) || s_step == 0.) {
                        stop = PlacementStop::MidStationary;
                    } else {
                        const double F0 = value_at(x_orig);
                        constexpr double kArmijoC1 = 1e-4;
                        constexpr int kMaxHalvings = 40;
                        bool taken = false;
                        bool tan_ring = false;
                        bool tan_chord = false;
                        for (int k = 0; k < kMaxHalvings; ++k, s_step *= 0.5) {
                            Vector2d x_try;
                            if (!walk_along_curve(curve_tag, x_orig, s_step, x_try)) continue;
                            if (!x_try.allFinite()) continue;
                            if (inverts(x_try)) {
                                tan_ring = true;
                                continue;
                            }
                            if (over_bound(x_try)) {
                                bounded = true;
                                continue;
                            }
                            // THE CHORD BOUND, and the reason a corner needs no special case.
                            // surface_segment_is_outside() reads the vertex's stored position,
                            // so the trial has to be in place while it is asked; restored
                            // immediately either way. Phase B is single-visit-per-vertex, so no
                            // other thread is reading this slot.
                            bool chord_out = false;
                            for (const size_t nb : region_nbrs) {
                                const Vector2d save = m_vertex_attribute[vid].m_posf;
                                m_vertex_attribute[vid].m_posf = x_try;
                                const bool bad = surface_segment_is_outside(vid, nb);
                                m_vertex_attribute[vid].m_posf = save;
                                if (bad) {
                                    chord_out = true;
                                    break;
                                }
                            }
                            if (chord_out) {
                                tan_chord = true;
                                continue;
                            }
                            const QSample nxt = grad_norm_at(x_try);
                            if (nxt.r < r_floor) {
                                stop = PlacementStop::LeftOffset;
                                continue;
                            }
                            // Sufficient decrease in the REDUCED objective: the slope along the
                            // curve is d1 and the step is s_step of arclength.
                            if (!(value_at(x_try) <= F0 + kArmijoC1 * s_step * d1)) continue;
                            x_last_inside = x_try;
                            ++iters;
                            taken = true;
                            break;
                        }
                        if (taken) {
                            stop = PlacementStop::Moved;
                            ++m_placement_tangential;
                        } else if (bounded) {
                            stop = PlacementStop::QualityBound;
                        } else if (tan_ring) {
                            left_ring = true;
                            stop = PlacementStop::RingBlocked;
                        } else if (tan_chord) {
                            // Every admissible arclength put an incident region edge outside its
                            // tube. NOT a placement failure: it is the mesh saying it wants a
                            // vertex at the corner this one is trying to slide past, which is
                            // Phase A's job rather than this visit's.
                            stop = PlacementStop::ChordBlocked;
                        } else if (stop != PlacementStop::LeftOffset) {
                            stop = PlacementStop::MidStationary;
                        }
                    }
                }
            }
            if (!handled) {
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cur.H);
                const double lam_min = es.eigenvalues().minCoeff();
                const double lam_max = std::max(es.eigenvalues().maxCoeff(), 0.);
                const double floor_lam = std::max(
                    1e-8 * lam_max,
                    1e-8 * cur.q / std::max(m_offset_params.target_distance, 1e-16));
                const double shift = (lam_min >= floor_lam) ? 0. : (floor_lam - lam_min);
                const Eigen::Matrix2d Hpd = cur.H + shift * Eigen::Matrix2d::Identity();
                Vector2d d = Hpd.ldlt().solve(-cur.g);
                // Guard the solve rather than trust it: a direction that is not finite, or that
                // does not actually go downhill, falls back to steepest descent scaled by the model
                // so it still carries a length.
                if (!d.allFinite() || cur.g.dot(d) >= 0.) {
                    d = -cur.g /
                        std::max(lam_max, cur.q / std::max(m_offset_params.target_distance, 1e-16));
                }

                // ARMIJO BACKTRACKING, with the one-ring inversion, the feasible set and the
                // containment envelope as hard constraints on the trial point. Halving from the
                // full Newton step means the accepted length is the model's own unless something
                // refuses it.
                const double F0 = value_at(x_orig);
                const double slope = cur.g.dot(d); // < 0
                constexpr double kArmijoC1 = 1e-4;
                constexpr int kMaxHalvings = 40;
                double t = 1.;
                bool accepted = false;
                bool blocked_by_ring = false;
                for (int k = 0; k < kMaxHalvings; ++k, t *= 0.5) {
                    const Vector2d x_try = x_orig + t * d;
                    if (!x_try.allFinite()) continue;
                    if (inverts(x_try)) {
                        blocked_by_ring = true;
                        continue;
                    }
                    if (over_bound(x_try)) {
                        bounded = true;
                        continue;
                    }
                    const QSample nxt = grad_norm_at(x_try);
                    if (nxt.r < r_floor) {
                        stop = PlacementStop::LeftOffset;
                        continue; // a shorter step may still be feasible
                    }
                    if (!(value_at(x_try) <= F0 + kArmijoC1 * t * slope)) continue;
                    x_last_inside = x_try;
                    ++iters;
                    accepted = true;
                    break;
                }
                if (accepted) {
                    stop = PlacementStop::Moved;
                } else if (bounded) {
                    stop = PlacementStop::QualityBound;
                } else if (blocked_by_ring) {
                    // Every trial length inverted the ring: this vertex cannot move at all.
                    left_ring = true;
                    stop = PlacementStop::RingBlocked;
                } else if (stop != PlacementStop::LeftOffset) {
                    // No length gave sufficient decrease and nothing refused it either -- the model
                    // says downhill but the function does not follow, i.e. a numerically stationary
                    // point.
                    stop = PlacementStop::MidStationary;
                }
            } // !handled
        }
    } else {
        // NO STEP WAS EVER ATTEMPTED. Four different reasons; the first two are convergence.
        if (q_entry < 0.) {
            stop = PlacementStop::NonFinite;
        } else if (!(std::abs(entry.r) > 0.)) {
            stop = PlacementStop::OnLevelSet;
        } else if (q_entry > 0.) {
            // Off the level set, but the gradient is already at or under the run's own
            // convergence bar -- there is nothing left for this visit to chase. This is the
            // common case in late passes, and it is what the entry-relative rule turned into
            // 2000-step limit cycles.
            stop = PlacementStop::UnderBar;
        } else {
            // r != 0, so q == 0 means grad F vanished on its own: under the quadratic terms
            // q = 2 |r| |grad Phi|, leaving only grad Phi == 0 -- the pinch minimum. Under the
            // normal-projected term it also covers grad Phi . n == 0, a purely tangential
            // gradient. Either way a genuine stationary point with no descent direction.
            stop = PlacementStop::Stationary;
        }
    }
    ex.iters = iters;

    if (left_ring) {
        // The minimum this visit was walking toward lies outside what the one-ring admits --
        // the count the pass loop's backtrack-free exit watches.
        ++m_phase_b_constrained;
    }
    if (!x_last_inside.allFinite()) {
        set_vertex_position(vid, x_orig);
        ++m_smooth_rejects.inverted;
        ++m_phase_b_constrained;
        ex.disp = 0.;
        // x_last_inside is only ever x_orig or a position that already passed allFinite, so
        // reaching here means the vertex's OWN position was not finite on entry.
        book(PlacementStop::NonFinite);
        return false;
    }
    // Set it explicitly rather than relying on where the last inverts() probe left the vertex.
    set_vertex_position(vid, x_last_inside);

    ex.disp = (x_last_inside - x_orig).norm();
    if (stop == PlacementStop::Moved && iters == 1) ++m_placement_trace.moved_one_step;
    // PRESSED IS A STATE, NOT A STOP REASON. A vertex held against the bar may still take a
    // hair of a step within it and report Moved; what makes it pressed is that an incident face
    // sits AT stop_energy where it ends up, i.e. the mesh's quality bar, not the field, is what
    // stopped it. Only the stop_energy bar counts: a ring that was already worse than the bar on
    // entry is bounded by its own max instead, and that is not the seam.
    const bool pressed_now =
        (stop == PlacementStop::QualityBound) ||
        (kPlacementQualityBound && ring_max_at(x_last_inside) >= q_bar * (1. - 1e-9));
    if (pressed_now) ++m_phase_b_pressed;
    if (vid < m_placement_pressed.size()) m_placement_pressed[vid] = pressed_now ? 1 : 0;
    book(stop);

    // The one-ring's stored qualities are now stale; the split/collapse/swap passes read them.
    for (const size_t fid : locs) {
        m_face_attribute[fid].m_quality = get_quality(fid);
    }
    ++m_smooth_rejects.accepted;
    return true;
}

bool TopoOffsetTriMesh::smooth_interior_vertex_phase_b(const Tuple& t)
{
    // Pure one-ring AMIPS, solved to this vertex's own minimum; see the declaration. AMIPS alone
    // at unit weight -- Newton cancels any positive scale, so the value is irrelevant -- and no
    // envelope terms: an interior vertex carries no surface, so the shared smoother's pull and
    // containment branches are bypassed and this is exactly `solve()` plus the exact inversion
    // accept and the quality veto.
    optimization::SmoothVertexOptions opts;
    opts.w_amips = 1.0;
    opts.w_envelope = 0.0;
    opts.s_amips = 1.0;
    opts.s_envelope = 0.0;
    opts.two_stage = false;
    opts.quality_veto = m_params.smooth_quality_veto;

    auto& solver = m_phase_b_solver.local();
    if (!solver) {
        // The base solver's twin, with the stopping rule this phase is about: polysolve's
        // rel_grad_norm_tol is exactly gradNorm / initial_grad_norm, so the solve stops at
        // ab_vertex_grad_tol_rel of the visit's initial gradient, and the iteration budget is
        // deep enough that the tolerance is what actually fires.
        polysolve::json params = optimization::basic_nonlinear_solver_params;
        // ONE NEWTON STEP PER VISIT -- this is the "one Gauss-Seidel iteration" the phase is
        // built on. The tolerance is kept so a vertex already at its minimum costs nothing, but
        // with a budget of 1 it is the iteration count that decides, not the tolerance.
        params["max_iterations"] = 1;
        params["rel_grad_norm_tol"] = m_offset_params.ab_vertex_grad_tol_rel;
        solver = polysolve::nonlinear::Solver::create(
            params,
            optimization::basic_linear_solver_params,
            1,
            opt_logger());
    }
    return optimization::smooth_vertex_2d(*this, t, opts, solver, &m_smooth_rejects);

    // ---------------------------------------------------------------------------------------
    // STENCIL: the envelope-held background vertex, which smooth_before() currently refuses.
    // See the 3D twin for the options it needs (w_amips / w_envelope / smoothing_mode / the two
    // projection step counts) and for why it should run LAST in the pass. Not written yet
    // because the models under test have no such vertex, and a half-constrained version would be
    // worse than leaving them where Phase A put them.
    // ---------------------------------------------------------------------------------------
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
        // THE EDGE'S OWN STORED CLASS, which is ground truth here: this sweep runs between
        // operations, so no attribute slot is mid-write. It used to classify by `mask != 0`
        // alone, which is the endpoint-AND the dispatch itself over-claimed on -- so the audit
        // agreed with the bug it was written to find and filed offset edges under "region".
        const bool is_offset = edge_is_offset(eid);
        const uint64_t mask = is_offset ? uint64_t(0) : edge_mask(vids);
        if (is_offset)
            ++n_offset_class;
        else if (mask != 0)
            ++n_region_class;
        else
            ++n_other;

        // EXACTLY the dispatch the sanity check uses, so this cannot disagree with it.
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

        // How far outside, per REAL member. Never ask the composite -- walk the tags instead.
        //
        // SAMPLED ALONG THE SEGMENT, not just at the endpoints, because that is what
        // is_outside(edge) does and it is the whole distinction being drawn here: endpoints at
        // distance 0 with a large interior maximum is a CHORD spanning boundary the tube follows
        // around (a corner, a neck), not a boundary edge that has drifted. Endpoint distance is
        // kept separately so the two read side by side.
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
        // WHY it is tracked and why it claims that mask: the per-endpoint bits the AND is taken
        // over, and the edge's own stored class. A chord over-claim shows as two endpoints that
        // each legitimately carry the bit joined by an edge whose own class is not region.
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
    // TWO DIFFERENT COUNTS, one invariant and one expectation.
    //
    // The INVARIANT is on the stored masks: every tracked region edge must dispatch to an
    // envelope, i.e. edge_mask() of its endpoints -- the very expression
    // surface_envelope_for_edge() uses -- must be nonzero. The masks are seeded once from the
    // input partition and propagated (endpoint AND at splits, OR at collapses), so a zero here
    // is a propagation hole: the edge is tracked as a region boundary and contained by nothing.
    //
    // The LIVE bits going quiet is the EXPECTATION: execute_offset() replaces the tags of every
    // face the band grows through, so edge_boundary_bits() -- the CURRENT symmetric difference
    // -- is empty across any region edge the band swallowed. Counted as information, because it
    // is the reason the masks must be propagated rather than rederived, not a defect itself.
    int n_region = 0, n_unmasked = 0, n_live_dead = 0, n_wall = 0;
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
        ++n_unmasked;
        if (worst == size_t(-1)) worst = eid;
        // The first few, in full: what IS an uncontained region edge? Endpoint masks say which
        // end lost the chain; positions say where it sits (on the complex, at the front, ...).
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
        // WHERE the unmasked ones sit, which is what says who created them.
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
        "\t[envelope health @ {}] {} region-boundary edges tracked ({} on the wall) | {} with a "
        "ZERO stored mask (the invariant; must be 0) | {} with quiet LIVE bits (expected once the "
        "band retags the faces it grew through)",
        when,
        n_region,
        n_wall,
        n_unmasked,
        n_live_dead);

    // WHAT THE TAGS ACTUALLY ARE, band versus not. If band faces have been retagged, a region
    // boundary swallowed by the band loses its tag difference -- and its envelope with it.
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
            "contained by NOTHING: their endpoints' stored masks AND to zero, so "
            "surface_envelope_for_edge() has no envelope to hold them to. The masks are seeded "
            "at init and propagated, so this is a propagation hole, not a retag effect.",
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

void TopoOffsetTriMesh::audit_phase_b_offset_envelope_holds() const
{
    struct Ex
    {
        size_t vid = size_t(-1);
        double x = 0., y = 0.;
        bool flag = false, bbox = false, live_off = false;
        uint64_t mask = 0;
    };
    int n_offset = 0, n_held = 0, by_flag = 0, by_bbox = 0, live = 0, stale = 0, no_off_edge = 0;
    Ex ex_stale, ex_live;

    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!m_vertex_extra[vid].m_is_on_offset) continue;
        ++n_offset;
        const bool flag = m_vertex_extra[vid].m_is_on_region;
        const bool bbox = !m_vertex_attribute[vid].on_bbox_faces.empty();
        // THE INVARIANT IS ABOUT CONTAINMENT, so ask the dispatch, not the flags. A vertex
        // carrying m_is_on_region whose mask is 0 is held by nothing and is not a violation.
        if (!smoothing_containment_envelope(vid)) continue;
        ++n_held;
        by_flag += flag ? 1 : 0;
        by_bbox += bbox ? 1 : 0;

        // GROUND TRUTH, from the edges actually incident to this vertex right now.
        bool live_region = false, live_offset = false;
        for (const Tuple& e : get_one_ring_edges_for_vertex(vid)) {
            const size_t eid = e.eid(*this);
            if (edge_is_region(eid)) live_region = true;
            if (edge_is_offset_surface_live(e)) live_offset = true;
        }
        if (!live_offset) ++no_off_edge;
        Ex ex;
        ex.vid = vid;
        ex.x = m_vertex_attribute[vid].m_posf.x();
        ex.y = m_vertex_attribute[vid].m_posf.y();
        ex.flag = flag;
        ex.bbox = bbox;
        ex.live_off = live_offset;
        ex.mask = m_vertex_extra[vid].m_boundary_mask;
        if (live_region) {
            ++live;
            if (ex_live.vid == size_t(-1)) ex_live = ex;
        } else {
            ++stale;
            if (ex_stale.vid == size_t(-1)) ex_stale = ex;
        }
    }

    // NO LONGER AN INVARIANT. This used to warn on any envelope-held offset vertex, because
    // Phase B could not place one and a held vertex therefore never moved. It can now: those
    // vertices take the same Gauss-Seidel step under an added containment refusal (see
    // smooth_offset_vertex_backtracking), so a nonzero count here is the expected state of a
    // multi-tag model, not a defect. The breakdown below is still worth reading -- the
    // live-vs-stale region-edge split says whether the mask that dispatched the envelope
    // describes an edge the mesh still has.
    logger().info(
        "\t[phase B audit] {} offset-boundary vertices, {} of them held by an envelope in phase B "
        "(and therefore placed CONSTRAINED) | flags on those: {} m_is_on_region, {} on_bbox_faces "
        "| {} sit on a live region edge, {} do not, {} carry no live offset edge",
        n_offset,
        n_held,
        by_flag,
        by_bbox,
        live,
        stale,
        no_off_edge);
    log_region_edge_mask_health("phase B entry");
    const auto say = [&](const char* what, const Ex& e) {
        if (e.vid == size_t(-1)) return;
        logger().warn(
            "\t  [phase B audit] {}: v{} at ({:.6g}, {:.6g}) | m_is_on_region {} | on_bbox {} | "
            "live offset edge {} | boundary mask 0x{:x}",
            what,
            e.vid,
            e.x,
            e.y,
            e.flag,
            e.bbox,
            e.live_off,
            e.mask);
    };
    say("held WITH a live region edge", ex_live);
    say("held with NO live region edge -- STALE", ex_stale);

    // WHAT IS THE REGION EDGE UNDER THE EXEMPLAR? A class-0 edge whose two faces carry the SAME
    // tags is not a tag-region boundary at all: edge_boundary_bits() is their symmetric
    // difference, so its mask is 0 and envelope_for_mask() hands back nullptr. That is the
    // difference between "held by an envelope" and "carrying a flag that says it is".
    if (ex_live.vid != size_t(-1)) {
        const auto tag_str = [&](const size_t fid) {
            std::string out;
            for (const int64_t t : m_face_attribute[fid].tags) {
                out += (out.empty() ? "" : ",") + std::to_string(t);
            }
            return out.empty() ? std::string("-") : out;
        };
        for (const Tuple& e : get_one_ring_edges_for_vertex(ex_live.vid)) {
            const size_t eid = e.eid(*this);
            if (!m_edge_attribute[eid].m_is_surface_fs) continue;
            const std::optional<Tuple> opp = e.switch_face(*this);
            logger().warn(
                "\t    e{} class {} | live bits 0x{:x} | faces [{}] vs [{}] | label {} vs {}",
                eid,
                m_edge_attribute[eid].m_surface_class,
                edge_boundary_bits(e),
                tag_str(e.fid(*this)),
                opp ? tag_str(opp->fid(*this)) : std::string("WALL"),
                m_face_extra[e.fid(*this)].label,
                opp ? std::to_string(m_face_extra[opp->fid(*this)].label) : std::string("-"));
        }
    }
}

const char* TopoOffsetTriMesh::placement_stop_name(const PlacementStop s)
{
    switch (s) {
    case PlacementStop::Moved: return "moved";
    case PlacementStop::IterCap: return "hit the iteration cap";
    case PlacementStop::LeftOffset: return "reached the offset region's edge";
    case PlacementStop::EnvelopeBlocked: return "NO TANGENT ON ITS OWN BOUNDARY CURVE";
    case PlacementStop::ChordBlocked: return "SLIDE CUT OFF BY AN INCIDENT CHORD";
    case PlacementStop::QualityBound:
        return "PRESSED: EVERY STEP PUTS AN INCIDENT FACE OVER stop_energy";
    case PlacementStop::RingBlocked: return "RING BLOCKED, no first step";
    case PlacementStop::LeftRing: return "left the ring mid-walk";
    case PlacementStop::MidStationary: return "grad E vanished mid-walk";
    case PlacementStop::MidNonFinite: return "field non-finite mid-walk";
    case PlacementStop::PreInverted: return "ring already inverted on entry";
    case PlacementStop::NoRing: return "no one-ring";
    case PlacementStop::OnLevelSet: return "already on the level set";
    case PlacementStop::UnderBar: return "already under the run's bar";
    case PlacementStop::Stationary: return "stationary, grad Phi = 0";
    case PlacementStop::NonFinite: return "field non-finite on entry";
    default: return "?";
    }
}

void TopoOffsetTriMesh::log_placement_trace() const
{
    const auto& tr = m_placement_trace;
    int total = 0;
    for (size_t i = 0; i < PlacementTrace::N; ++i) total += tr.n[i].load();
    if (total == 0) return;

    // ORDERED SO THE ANSWER IS AT THE TOP. The exits that mean the vertex is where it wants
    // to be come first; everything after them is a vertex that did not get there, and the count
    // beside it is how many.
    static constexpr PlacementStop kOrder[] = {
        PlacementStop::Moved,
        PlacementStop::OnLevelSet,
        PlacementStop::UnderBar,
        // A CONVERGED STOP, not a failure: the walk reached the edge of {Phi >= c}, which under
        // the normal-projected term IS the level set. Grouped with the arrivals for that reason.
        PlacementStop::LeftOffset,
        PlacementStop::IterCap,
        PlacementStop::RingBlocked,
        PlacementStop::LeftRing,
        // A FAILURE, not an arrival -- unlike LeftOffset above. Reaching the edge of {Phi >= c}
        // is arriving at the level set; being stopped by a region tube is having nowhere to go.
        PlacementStop::EnvelopeBlocked,
        PlacementStop::ChordBlocked,
        PlacementStop::QualityBound,
        PlacementStop::Stationary,
        PlacementStop::MidStationary,
        PlacementStop::PreInverted,
        PlacementStop::NoRing,
        PlacementStop::NonFinite,
        PlacementStop::MidNonFinite,
    };
    // The list is hand-ordered, so it can silently fall behind the enum -- and did once, hiding
    // a whole stop reason while the printed counts quietly stopped summing to the visit count.
    static_assert(
        std::size(kOrder) == size_t(PlacementStop::COUNT),
        "kOrder must list every PlacementStop; add the new one and pick where it reads");
    std::string line;
    for (const PlacementStop st : kOrder) {
        const int c = tr.n[size_t(st)].load();
        if (c == 0) continue;
        line += fmt::format(
            "{}{} {} ({} reachable)",
            line.empty() ? "" : " | ",
            placement_stop_name(st),
            c,
            tr.n_reachable[size_t(st)].load());
    }
    logger().info(
        "\t[phase B] placement stops over {} offset visits: {} | of the {} that moved, {} took a "
        "single step",
        total,
        line,
        tr.n[size_t(PlacementStop::Moved)].load(),
        tr.moved_one_step.load());

    // THE EXEMPLARS ARE THE POINT. A count says how many; this says which, and carries the four
    // numbers that separate the causes -- so a suspect vertex can be found in the viewer instead
    // of inferred. Only the stops that mean the vertex is NOT placed get one.
    for (const PlacementStop st :
         {PlacementStop::RingBlocked,
          PlacementStop::LeftRing,
          PlacementStop::Stationary,
          PlacementStop::MidStationary,
          PlacementStop::IterCap,
          PlacementStop::PreInverted,
          PlacementStop::NonFinite,
          PlacementStop::MidNonFinite,
          PlacementStop::NoRing}) {
        const size_t i = size_t(st);
        if (tr.n[i].load() == 0) continue;
        const auto& w = tr.worst[i];
        if (w.vid == size_t(-1)) continue;
        logger().warn(
            "\t  worst [{}]: v{} at ({:.6g}, {:.6g}) | Phi - c = {:.4g} (tol {:.4g}) | "
            "|grad Phi| = {:.4g} | |cos(grad Phi, n)| = {:.3g} | |n| = {:.3g} | steps {} | "
            "moved {:.4g}",
            placement_stop_name(st),
            w.vid,
            w.x,
            w.y,
            w.res,
            offset_residual_tolerance(),
            w.gnorm,
            w.cosn,
            w.nnorm,
            w.iters,
            w.disp);
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
    // The envelope-held offset vertices, which this scheme does not place at all. WARNED, not
    // merely counted: they are left where Phase A put them, and the only reason that is not a
    // silent degradation is that band_vertex_is_reachable() also books them PINNED, so they stop
    // gating convergence and are reported separately in every band measure.
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
    // contain it. The projection pulls it back, so it is no longer a behavioural exception --
    // but it should not have been outside in the first place.
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

    // PHASE A'S UNITS. optimization_quality_stats() and optimization_stop_metric() both branch
    // on m_phase, and they have to agree: Phase A is absolute AMIPS against stop_energy, which
    // is what refine_sizing_around_worst() derives its filter from. Phase B's normalized pair
    // would also dereference m_offset_potential, which does not exist yet.
    m_phase = OptPhase::A;
    // THE SPLIT HOOK BRANCHES ON THIS. Midpoint is the construction path (simplicial embedding
    // and marching, which carry their own labels); the shared engine's splits must take the
    // Optimization path or they would be treated as marching splits.
    m_edge_split_mode = EdgeSplitMode::Optimization;

    // THE SIZING FIELD: target_distance ON THE INPUT-COMPLEX BOUNDARY, graded outward.
    //
    // That boundary is the curve Phi measures from and the curve the marching wraps the band
    // around, so it is the only place the mesh has to resolve at the offset's own scale. Asking
    // for delta THERE makes the band the marching builds one delta-scale cell thick, which is
    // what puts the constructed offset near delta from the complex -- and dhat is sized from
    // exactly that reach. Everything else is left to a few sweeps of one-ring averaging, so the
    // mesh is fine at the complex, coarse at the domain wall, with no jump in between.
    //
    // The BOUNDARY, not the complex: a filled complex (two_circles' disks are solid) has
    // interior vertices whose label is also 1, and resolving a disk's interior at delta would
    // multiply the mesh for geometry the offset never measures against. An edge is on that
    // boundary when exactly one of its incident faces carries the input-complex label -- the
    // same rule that produced m_phi_E. The interior is reached only by the averaging, at
    // whatever value diffusion carries in, like any other off-complex vertex.
    const double l_target = std::max(m_params.l, 1e-16);
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / l_target);
    const double s_input = std::clamp(
        m_offset_params.target_distance / l_target,
        s_floor,
        m_offset_params.max_sizing_scalar);
    // GRADED IN DISTANCE, NOT IN MESH RINGS.
    //
    // This used to seed the boundary vertices and hand them to gradation_smooth_sizing(), the
    // shared ring BFS: it walks the one-ring, multiplies the scalar by sizing_gradation each
    // ring, and stops once the cap reaches 1. From delta/l = 0.152 at grade 2 that is exactly
    // TWO rings -- and a ring is one EDGE of whatever mesh happens to be standing when it runs.
    // On a coarse input mesh those two rings are geometrically enormous. Measured on
    // two_circles: a ring is ~0.32 long, so the seed's footprint came out ~0.64 = 13 delta,
    // which reaches the centre of a disk of radius 1. The complex INTERIOR was never seeded and
    // was refined anyway, because the ramp is indexed by connectivity rather than by length.
    //
    // The continuous form of the same rule has no such dependence:
    //
    //     h(x) = target_distance + (grade - 1) * dist(x, input complex)
    //
    // Lipschitz with constant grade - 1 by construction -- which is the property the ring BFS
    // is trying to approximate -- equal to delta ON the complex and nowhere else, and reaching
    // the background target l at a fixed DISTANCE (l - delta) / (grade - 1) no matter how the
    // mesh is discretised. Symmetric in dist, so it grades INTO the complex on the same ramp it
    // grades out of it; the clamp caps it at l so nothing is ever asked to be coarser than the
    // background. m_input_complex_bvh is the same structure Phi queries, built by the driver
    // before execute_offset(), so the distance here is the exact one.
    // WHICH SIZING FIELD THIS PASS RUNS AGAINST -- see pre_optimize_sizing_from_edges.
    if (m_offset_params.pre_optimize_sizing_from_edges) {
        // "KEEP THE RESOLUTION YOU HAVE": every vertex takes the mean of its own incident edge
        // lengths, exactly the rule init_offset_sizing_field() uses on the offset surface.
        // target_distance never enters, so this pass improves element QUALITY without refining
        // anything toward delta -- and the constructed offset therefore lands wherever the input
        // mesh's own scale puts it, with dhat sized from that reach.
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
        // THE GRADATION: gradation_smooth_sizing(), the shared ring BFS, and nothing else.
        //
        // Five forms have been measured on two_circles and this is the only one that converges. Do
        // not re-litigate them:
        //  - Lipschitz ramp in distance, h = delta + (grade-1)*dist: first cell layer twice too
        //    coarse, band twice as thick, dhat 2.66 -> 6.64 delta, level sets nearly merge.
        //  - the same with a plateau to 2*delta: barely moved it (6.37 delta).
        //  - one-ring Jacobi averaging, seeds pinned: dhat fine (2.68) but 35% of edges violated
        //  the
        //    gradation cap right after seeding, and the run stalled at 1e50.
        //  - the tetwild component's init_sizing_field() distance BFS to R = 1.8*l: its min is
        //  taken
        //    against the SEED's scalar, so the ramp is dead code and the behaviour is a flat ball
        //    at
        //    the seed value. Fed delta/l it refines nearly the whole domain; the pre-pass never
        //    finished (165k vertices and climbing).
        //  - the triwild component's own recipe (grow_vertex_region + apply_sizing_refinement's
        //    MULTIPLICATIVE x stuck_refine_factor + gradation): relative, so from 1.0 it lands at
        //    scalar 0.5, nowhere near delta/l. The pre-pass barely refined (259 -> 305 vertices),
        //    the constructed offset landed at 6.16 delta, dhat came out 12.32 delta -- back in the
        //    regime where the level sets merge -- and the run died with 681 1e50 lines.
        //
        // The ring BFS's wart is real: its footprint is indexed by CONNECTIVITY, so it scales with
        // how coarse the input mesh happens to be rather than with a length. It stands anyway until
        // something beats it ON A RUN.
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
        wmtk::vector_unique(seeds);
        if (!seeds.empty()) {
            gradation_smooth_sizing(m_offset_params.sizing_gradation, seeds);
        }
        logger().info(
            "[pre-optimize] sizing seed: {} input-complex-boundary vertices at scalar {:.6g} "
            "(= target_distance {} / l {:.6g}), graded outward at {}x per ring",
            seeds.size(),
            s_input,
            m_offset_params.target_distance,
            l_target,
            m_offset_params.sizing_gradation);
    }

    // The SEEDED field, before a single operation runs -- the middle frame of the sizing story
    // (uniform as loaded, seeded here, whatever propagation makes of it by the end).
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

    mesh_improvement(std::max(1, m_offset_params.ab_phase_a_iterations));

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

    // RE-DERIVE THE CONSTRUCTION LABELS, because the optimization does not maintain them.
    //
    // split_after_vertex() propagates every per-vertex record a split has to carry -- the two
    // surface flags, m_is_on_region, the boundary mask, the birth epoch -- but NOT
    // VertexExtra2d::label, and neither collapse nor swap touches it either. That was harmless
    // for as long as optimization splits could only run AFTER marching_tris(), because nothing
    // reads the vertex label past that point: the band is tagged by then and the later stages
    // read m_face_extra[].label, which split_adjust_position() does carry.
    //
    // This pass breaks that assumption -- it is the first optimization that runs BEFORE the
    // marching -- and marching_tris() decides which edges to split from exactly this label
    // ((label == 0) != (label == 0) across an edge). Left stale, the crossing set comes out
    // wrong, the frontier is wrong, and the band it grows has HOLES: measured on two_circles,
    // 254 of 382 complex-boundary vertices ended up on the offset boundary as well as the input
    // complex, with ambient faces sitting directly against the complex, which is the
    // construction defect check_no_vertex_on_both_surfaces() then reports.
    //
    // Re-derived rather than propagated: the label is a function of the FACE TAGS
    // (expr->eval over the incident faces), the base does propagate tags through split and
    // collapse, and label_input_complex() is the authority on that function. Cleared first
    // because it only ever WRITES 1 -- it has no path back to 0, so a stale 1 on a vertex the
    // optimization moved out of the complex would survive a second call.
    for (const Tuple& v : get_vertices()) m_vertex_extra[v.vid(*this)].label = 0;
    for (const Tuple& e : get_edges()) m_edge_extra[e.eid(*this)].label = 0;
    for (const Tuple& f : get_faces()) m_face_extra[f.fid(*this)].label = 0;
    label_input_complex();

    // THE INPUT COMPLEX IS NOT RE-EXTRACTED. The driver builds m_input_complex_bvh -- and with
    // it m_phi_V/E/P, the arrays init_offset_potential() later hands to Phi -- once, before
    // execute_offset(), and that ONE extraction is used for the whole run.
    //
    // Re-extracting here was tried and it is what broke the run. init_input_complex_bvh()
    // collects every vertex in the CLOSURE OF THE LABEL-1 FACES, i.e. the input region's
    // interior, not just its boundary curve. On the input mesh that is 160 vertices; after this
    // pass has refined the region it is 2643, and those ~2400 interior vertices enter m_phi_V
    // and move Phi -- measured on two_circles, level c 0.379816 -> 0.384728 with a bit-identical
    // pre-pass mesh (2722 V / 5371 F / quality 8.132 either way). With the re-extraction the run
    // stalls at max energy 1e50; without it, it converges in 2 rounds. The function is written
    // to run once on the coarse input mesh and does not survive being called on a refined one.
    //
    // The cost of leaving it out is that Phi measures the polygon the mesh had ON LOAD while the
    // mesh carries the refined one -- a chord sagitta, 0.098^2/8 = 0.0012 = 2.4% of delta here.
    // Systematic, and it grows with the input's coarseness and curvature, so if it ever has to be
    // fixed the fix is to re-extract the BOUNDARY CURVE ONLY, not the closure.
    needle_scan("after the pre-pass");
}

void TopoOffsetTriMesh::init_offset_sizing_field()
{
    // Paper Sec. 5.3.3, Step 1: the sizing field "is defined on each edge of the offset mesh and
    // is INITIALIZED WITH THE CURRENT LENGTH OF EACH EDGE."
    //
    // This is not a detail. The base's m_sizing_scalar defaults to 1, i.e. a target length of
    // m_params.l everywhere -- and l is a fraction of the bounding box diagonal, which on any
    // reasonable configuration is far longer than the offset polyline's own edges (7.2x on the
    // dragon rectangle). Starting there makes every single offset edge a collapse candidate on
    // the first pass, and the offset is decimated before the metrics below ever get to speak.
    // The per-operation normal-deviation guard cannot save it either: each individual collapse
    // of a well-resolved curve barely moves the angle, so they pass one at a time.
    //
    // Starting from the current lengths says instead: keep the resolution you have, and change
    // it only where the sizing update below finds a reason to. The field is per-vertex here
    // rather than per-edge, so a vertex takes the mean of its incident offset-surface edges.
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
        // EVERY vertex is seeded, not just the offset boundary's. A vertex with no incident
        // offset edge falls back to its whole one-ring -- the same rule applied to whatever scale
        // it happens to sit at.
        //
        // Leaving the background at the base target -- a fraction of the bounding box, which on
        // any reasonable configuration is far coarser than the mesh the construction produced --
        // is what makes a bare collapse pass destructive. The gate is edge length against the
        // target at its endpoints, so a background target of l against actual edges a fraction of
        // that marks essentially every interior edge as collapsible. Seeding from the mesh as
        // built says "keep the resolution you have" everywhere.
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
        logger().warn(
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

        // WHY it scores MAX_ENERGY. is_inverted() is exact for the coordinates the vertices
        // actually carry; is_inverted_f() uses m_posf alone. A face that is inverted in float
        // but not exactly is a valid triangle whose double area underflowed -- refining it
        // produces two more of the same, which is the distinction this whole census exists for.
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

        // CAN the next pass even split it? The base's gate is length^2 > (l * sbar)^2 * 16/9,
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

    // Connected clusters among the MAX_ENERGY faces, by shared edge. Union-find over the set
    // only -- "seemingly random regions" is exactly the question of how many clumps there are.
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
    // Pure instrumentation: the base's rule is returned unchanged. See the header for why the
    // `q <= ring_max` clause is the one to watch.
    // 3D's admission rule, not the 2D base's. The base also admits any collapse whose result
    // scores under stop_energy -- a clause carried from the original TriWild
    // (EdgeCollapsing.cpp:189 before cb9b81a66c) that the original TetWild never had (its
    // EdgeCollapsing.cpp:146). It matters because mesh_improvement() OPENS with a collapse
    // sweep that has no length gate (local_operations({{0,1,0,0}}, false)), and the A/B loop
    // runs that sweep at the top of EVERY phase A: on a mesh whose faces all score 2-6, "under
    // 10 is fine" admits nearly everything, so the sweep demolished the mesh each round
    // regardless of the sizing field -- two_circles, target_distance 0.02: 1939 -> 476 and
    // 9510 -> 755 vertices -- and the split pass then rebuilt it 10-80x against a field halved
    // since, until the rebuild itself manufactured MAX_ENERGY faces (round 3). Requiring "no
    // worse than the ring's worst" keeps ~97% of the mesh through the sweep; with it (and
    // sizing_propagate_min off) the case converges. Ablated: necessary in every combination.
    const bool allowed = !m_vertex_attribute.at(v1).m_is_rounded || q <= ring_max;
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
    logger().warn(
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
    logger().warn(
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
    // Only the CREATIONS are logged: a flat child of a flat parent is the multiplication we
    // already understand, a flat child of a healthy parent is the event we are hunting.
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
        logger().warn(
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
    logger().warn(
        "[forensics] {} faces flatter than {:g} | gates: collapse 4/5*l = {:.6g}, split 4/3*l = "
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
    logger().warn(
        "[forensics] coincident vertices (closer than {:.3g}): {} pairs, {} of them NOT joined "
        "by an edge (no collapse can reach those). {}",
        eps,
        n_pairs,
        n_pairs_no_edge,
        first.empty() ? "none" : first);
    logger().warn(
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
    // Read from the TAGS, not from m_vertex_extra[].m_is_on_offset. The flag is only refreshed
    // by label_offset_boundary() once per iteration, while the shared operations maintain the
    // tags continuously; and the band is exactly where a face carrying an offset output tag
    // meets one that does not, which the tags say directly.
    // Only the OUTER surface of the band, the one that is supposed to sit at target_distance.
    // The band has two boundaries: this one, and the inner interface where it wraps the input
    // complex -- and that inner one is by construction at distance 0, so including it makes the
    // max error identically target_distance on every input and the test can never pass. 3D
    // draws the same line: it marks a face on-offset only when `label + other.label == 2`, i.e.
    // the offset region (2) meeting a face that is neither offset nor input complex (0).
    std::vector<bool> on_band(vert_capacity(), false);
    for (const Tuple& e : get_edges()) {
        const std::optional<Tuple> opp = e.switch_face(*this);
        if (!opp) {
            // A band edge on the DOMAIN BOUNDARY is still the band's outer surface, and its
            // vertices are still offset-boundary vertices that are supposed to sit at
            // target_distance. The rule below is a comparison between two incident faces and
            // there is only one here, but the missing side is outside the domain, which is
            // trivially neither band nor input complex -- so the test resolves, it just cannot
            // be written as a comparison.
            //
            // Skipping these is what let a clipped offset report a healthy error while half the
            // target distance was missing: where target_distance exceeds the clearance to the
            // bounding box the band is clipped, and exactly the clipped vertices -- the ones
            // that are frozen and cannot be fixed -- were the ones dropped from the measurement.
            // On the dragon rectangle at target_distance 0.1 the true error there reached 52% of
            // target while the report said 4.8%, and at target_distance_rel 1 the entire band
            // boundary lay on the box, leaving NO measured edges at all: max_dist_err 0.0 and
            // "converged" on the first iteration, having measured nothing.
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

// returns max_dist_err, avg_dist_err over the WHOLE band, pinned vertices included -- see
// distance_deviation_split() for why the loop uses a different number than the report does.
double TopoOffsetTriMesh::band_vertex_residual(const size_t vid) const
{
    // How far this vertex is from the level set Phi = c, as a LENGTH. The offset's own error,
    // as opposed to band_vertex_distance_error()'s Euclidean diagnostic.
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
    // The band's Phi residual. EVERY offset-boundary vertex and every edge sample counts toward
    // the driving max -- including pinned ones (on the domain wall), which used to be excluded.
    // A pinned vertex far from the level set is a real error in the offset the run returns, so
    // hiding it reported convergence for a boundary that was not at target distance. The
    // reachable/pinned split is kept as ATTRIBUTION: when the max comes from a pinned vertex,
    // the report says so, and the remedy is construction (domain size), not more optimization.
    // Same change as 3D.
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
            // THE RUNAWAY GUARD's measurement, taken here rather than in its own traversal:
            // this loop already visits exactly the vertices it cares about, and Phi is the
            // expensive part. report_outside_support() turns this into the error.
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

TopoOffsetTriMesh::GradientSplit TopoOffsetTriMesh::measure_gradient_reference()
{
    // THE SCALE THE CRITERION IS A FRACTION OF, measured once on the band as constructed and
    // never again -- a moving reference would be a moving bar.
    //
    // THE REFERENCE IS THE NORMAL-ALIGNED MAX AT VERTICES: max |2 (Phi - c) grad Phi . n| over
    // every initial offset-surface vertex, reachable and pinned alike, with n the surface's
    // own Voronoi-length-weighted normal. The normal component is what "misplaced" means -- it
    // is the part of the gradient that moves the surface off the level set -- so the reference
    // is a statement about how far out of place the offset STARTED (and a pinned vertex is out
    // of place too, hence both halves). What it scales is then the FULL norm at those same
    // vertices: the convergence criterion and every Phase B local stop compare
    // ||2 (Phi - c) grad Phi|| against convergence_gradient_norm_rel x this. One reference,
    // one parameter, one bar.
    const GradientSplit g = gradient_split();
    m_gradient_reference = g.max_normal_aligned;

    logger().info(
        "\tGradient reference (band as constructed): {:.6g} = max |2 (Phi - c) grad Phi . n| "
        "over {} offset vertices ({} reachable + {} pinned)",
        m_gradient_reference,
        g.n_reachable + g.n_pinned,
        g.n_reachable,
        g.n_pinned);
    logger().info(
        "\t  full-norm max: reachable {:.6g}, pinned {:.6g} | in-edge diagnostic {:.6g} "
        "({} samples)",
        g.max_reachable,
        g.max_pinned,
        g.max_in_edge,
        g.n_edge_samples);
    logger().info(
        "\t  => convergence bound {:.6g} = convergence_gradient_norm_rel {} x reference",
        offset_gradient_tolerance(),
        m_offset_params.convergence_gradient_norm_rel);

    if (!(m_gradient_reference > 0.)) {
        logger().warn(
            "Gradient reference measured as {} -- the convergence bound falls back to its 1e-16 "
            "floor, which no run will meet. The band as constructed is already stationary, or "
            "nothing on it was measurable.",
            m_gradient_reference);
    }
    return g;
}

TopoOffsetTriMesh::GradientSplit TopoOffsetTriMesh::gradient_split(
    const bool include_edge_samples) const
{
    // THE CONVERGENCE CRITERION: ||grad (Phi(x) - c)^2|| AT EVERY BAND VERTEX -- the full norm
    // of the gradient of the SAME objective Phase B's local solves minimize,
    // || 2 (Phi - c) grad Phi ||, measured at the variables the optimizer owns and compared
    // against offset_gradient_tolerance().
    //
    // IDENTICAL TO THE PHASE B LOCAL STOP BY CONSTRUCTION. Each visit terminates once its own
    // ||grad E|| is under the same bar, so "every visit finishes immediately" and "the run has
    // converged" are one statement; any drift between the two would measure a different fixed
    // point than the one the sweeps converge to.
    //
    // WEIGHT 1, deliberately: this is an ABSOLUTE bound in length units, so a tuning weight
    // would scale the bar.
    const std::vector<bool> on_band = band_vertex_mask();
    // One energy per region field, plus the union for a vertex with no region: a vertex is
    // measured against the field it is placed on. See potential_for().
    std::vector<std::unique_ptr<OffsetEnergy2D>> energies;
    for (const auto& rp : m_region_potentials)
        energies.push_back(std::make_unique<OffsetEnergy2D>(rp, 1.0));
    OffsetEnergy2D union_energy(m_offset_potential, 1.0);
    const auto energy_for = [&](const int region) -> OffsetEnergy2D& {
        return (region >= 0 && size_t(region) < energies.size()) ? *energies[size_t(region)]
                                                                 : union_energy;
    };

    // THE NORMAL-ALIGNED COMPANION, |grad E . n| with n the OFFSET SURFACE'S OWN normal (the
    // Voronoi-length-weighted vertex normal, offset_surface_normal()). NOT the deciding
    // measure: it is the quantity the REFERENCE is the max of -- measure_gradient_reference()
    // reads max_normal_aligned off the band as constructed, and every full-norm comparison is
    // against convergence_gradient_norm_rel x that. Only motion along n moves the surface off
    // the level set, so the reference is a statement about how MISPLACED the initial surface
    // was; the running tests are then the full norm, which a local solve can actually zero.
    // The same projection serves the in-edge chord diagnostic, onto the edge's own unit
    // normal (inside an edge the discrete surface IS the chord).
    //
    // A degenerate site (no live offset edge at a vertex, a zero-length edge) has no normal;
    // there the full norm stands in, which is the conservative reading since |g . n| <= |g|.
    const auto project = [](const Eigen::VectorXd& g, const Vector2d& n) -> double {
        return (n.squaredNorm() > 0.) ? std::abs(g.head<2>().dot(n)) : g.norm();
    };

    GradientSplit s;
    double sum_reachable = 0.;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid]) continue;
        if (!m_vertex_extra[vid].m_is_on_offset) continue;

        // SKIPPED, not measured: a vertex the smoother declines to place this pass has a
        // gradient that is not part of the fixed point. Counted so a run cannot report
        // convergence over a band it never fully measured.
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

        // PINNED VERTICES ARE REPORTED, NOT GATED -- and this is the one place the gradient
        // criterion deliberately parts company with residual_split(). A residual is a statement
        // about the BOUNDARY; a gradient is a statement about the ITERATION, and folding in a
        // vertex no sweep can move would make convergence unreachable by construction.
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

    // ... AND ALONG THE BAND'S EDGES, which since 2026-08-23 is the OTHER HALF OF THE CRITERION
    // rather than a diagnostic beside it.
    //
    // THE ARGUMENT FOR EXCLUDING THEM WAS WRONG, and it is worth stating why because it sounds
    // right. It ran: a sample is not a variable, so no placement can reduce it, so gating on it
    // would gate the run on a number the iteration cannot change. True of PLACEMENT and false of
    // the LOOP -- refinement changes it, that is precisely what
    // update_band_sizing_from_tolerance() exists to do, and Phase A then acts on the field it
    // writes. Excluding them made the sizing rule the only consumer of the chord term, so the
    // loop refined on a quantity it then refused to be judged by: measured on topo_annots_groups
    // (tag_0 & tag_2, delta 1.2, rel 1e-3), a run declared CONVERGED at max_grad 0.0338 against
    // a bar of 0.0339 while its chord term stood at 8.33 -- 246x the same bar. The offset was
    // visibly polygonal and the verdict said it was done.
    //
    // TWO CHANGES CAME WITH THE PROMOTION, both required for the comparison to mean anything:
    //  - THE SAME QUANTITY. The full norm ||2 (Phi - c) grad Phi||, as at the vertices. It used
    //    to be the NORMAL PROJECTION project(g, n_e), which is <= the full norm, so it was being
    //    compared against a bar calibrated for a different measure -- and the sizing rule was
    //    already using the full norm at the same sample points, so the two disagreed about what
    //    "this sample is out of tolerance" meant.
    //  - REACHABILITY. Only edges whose BOTH endpoints are reachable can gate. A chord to a
    //    PINNED vertex inherits that vertex's error, and no refinement fixes it: splitting the
    //    edge just puts a new vertex next to the one that cannot be placed. Gating on those
    //    would re-create exactly the unconvergeable-by-construction state pinning was introduced
    //    to remove. They are measured into max_in_edge_pinned and reported.
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

double TopoOffsetTriMesh::phase_b_band_gradient_linf()
{
    // One measurement, one definition -- the Phase B stop test and the convergence test read the
    // same function, since a phase that stops on one number while the run is judged by another
    // can sit at a fixed point of neither.
    //
    // VERTICES ONLY, AND THIS IS NOW A DELIBERATE ASYMMETRY with the convergence test, which
    // since 2026-08-23 gates on the edge-interior samples as well.
    //
    // Phase B moves vertices and does nothing else, so the chord term is not a quantity this
    // phase can reduce -- including it here would simply stop Phase B ever exiting early and
    // make it burn its whole pass budget on every round. The chord term is answered by
    // update_band_sizing_from_tolerance() and the Phase A that follows it, so it belongs to the
    // ROUND's test, not the phase's. "Phase B has finished its work" and "the run is converged"
    // are different questions now, and this is the first.
    return gradient_split(/*include_edge_samples=*/false).max_at_vertex;
}

TopoOffsetTriMesh::DistanceCriterion TopoOffsetTriMesh::distance_criterion(
    const bool include_edges) const
{
    // See the header. Reachability, rounding and the band mask follow gradient_split() exactly,
    // so the two criteria judge the same set of vertices and the same edge samples.
    DistanceCriterion s;
    const double delta = m_offset_params.target_distance;
    const double eps = std::max(m_offset_params.convergence_distance_rel, 1e-16) * delta;
    const double kPi = std::acos(-1.);
    const double cos_max = std::cos(m_offset_params.convergence_orientation_max_deg * kPi / 180.);
    const std::vector<bool> on_band = band_vertex_mask();
    // A vertex whose last placement stopped on QualityBound is held by the mesh's quality bar,
    // not by the field: its level set is unreachable by construction (two fronts meeting), so
    // it counts as placed, and an edge touching one is the seam -- not a resolution or an
    // orientation question, since the field's gradient is ~0 along the midline there.
    const auto pressed = [&](const size_t vtx) {
        return vtx < m_placement_pressed.size() && m_placement_pressed[vtx];
    };

    // First-order distance to the level set. -1 where the field gives no direction: outside the
    // support Phi is identically zero WITH a zero gradient, and a division there is not a
    // distance. Such points are counted, never silently skipped.
    const auto dist_at = [&](const OffsetPotential2D& P, const Vector2d& p) -> double {
        const double r = P.value(p) - P.target_level();
        const double gn = P.gradient(p).norm();
        if (!std::isfinite(r) || !std::isfinite(gn) || gn <= 1e-300) return -1.;
        return std::abs(r) / gn;
    };

    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid] || !m_vertex_extra[vid].m_is_on_offset) continue;
        if (!m_vertex_attribute[vid].m_is_rounded) continue;
        if (!band_vertex_is_reachable(vid)) continue;
        if (pressed(vid)) {
            ++s.n_pressed;
            continue;
        }
        ++s.n_vertices;
        const double d = dist_at(potential_for(vid), m_vertex_attribute[vid].m_posf);
        if (d < 0.) {
            ++s.n_outside_support;
            continue;
        }
        if (d > s.max_vertex_dist) {
            s.max_vertex_dist = d;
            s.worst_vid = vid;
        }
    }

    if (include_edges) {
        for (const Tuple& e : get_edges()) {
            if (!edge_is_offset_surface_live(e)) continue;
            const size_t va = e.vid(*this), vb = e.switch_vertex(*this).vid(*this);
            if (!band_vertex_is_reachable(va) || !band_vertex_is_reachable(vb)) continue;
            if (pressed(va) || pressed(vb)) {
                ++s.n_edges_pressed;
                continue;
            }
            ++s.n_edges;
            const OffsetPotential2D& P_e = potential_for_edge(va, vb);
            for_each_offset_edge_sample(e, [&](const Vector2d& q) {
                ++s.n_samples;
                const double d = dist_at(P_e, q);
                if (d < 0.) {
                    ++s.n_outside_support;
                    return;
                }
                s.max_edge_dist = std::max(s.max_edge_dist, d);
            });

            // ORIENTATION. The edge's outward normal is the one pointing away from the band face
            // (the centroid says which side that is -- no orientation convention to get wrong).
            // The field's outward direction is the way Phi changes on LEAVING the band, read off
            // the field itself by comparing it at the band centroid and at the midpoint, so the
            // test is right for the smooth potential (larger inside) and the Euclidean distance
            // (smaller inside) alike without knowing which one it has.
            const Vector2d a = m_vertex_attribute[va].m_posf, b = m_vertex_attribute[vb].m_posf;
            const Vector2d t = b - a;
            if (t.norm() <= 0.) continue;
            const std::optional<Tuple> opp = e.switch_face(*this);
            if (!opp) continue;
            const size_t fa = e.fid(*this), fb = opp->fid(*this);
            const size_t band_f = face_is_offset_band(fa) ? fa : fb;
            const auto vs = oriented_tri_vids(band_f);
            const Vector2d c =
                (m_vertex_attribute[vs[0]].m_posf + m_vertex_attribute[vs[1]].m_posf +
                 m_vertex_attribute[vs[2]].m_posf) /
                3.;
            const Vector2d m = 0.5 * (a + b);
            Vector2d n(t.y(), -t.x());
            n /= n.norm();
            if (n.dot(m - c) < 0.) n = -n;
            const Vector2d g = P_e.gradient(m);
            const double gn = g.norm();
            if (!std::isfinite(gn) || gn <= 1e-300) {
                ++s.n_outside_support;
                continue;
            }
            const double inside_minus_mid = P_e.value(c) - P_e.value(m);
            const Vector2d field_out = (inside_minus_mid >= 0. ? -1. : 1.) * (g / gn);
            const double cs = n.dot(field_out);
            if (cs < s.min_cos) {
                s.min_cos = cs;
                s.worst_edge_mid = m;
            }
            if (cs < 0.) ++s.n_folded;
        }
    }

    s.max_vertex_rel = s.max_vertex_dist / eps;
    s.max_edge_rel = s.max_edge_dist / eps;
    s.worst_angle_deg = std::acos(std::clamp(s.min_cos, -1., 1.)) * 180. / kPi;
    s.placed = s.n_outside_support == 0 && s.max_vertex_dist <= eps;
    s.resolved = !include_edges || s.max_edge_dist <= eps;
    s.oriented = !include_edges || s.min_cos >= cos_max;
    return s;
}

void TopoOffsetTriMesh::assign_band_regions()
{
    // See m_region_potentials. A flood fill over the band faces, seeded from every band face
    // that shares an edge with an input-complex face, with that face's region. A band face
    // reached with two different regions (the bands merged) and a vertex whose band faces
    // disagree read -2 and fall back to the union field -- reported, never silent.
    m_face_region.assign(tri_capacity(), -1);
    m_vertex_region.assign(vert_capacity(), -1);
    if (m_region_tags.size() <= 1 || m_region_potentials.empty()) return;
    const auto region_of_input_face = [&](const size_t fid) -> int {
        const CellTag& tags = m_face_attribute[fid].tags;
        for (size_t r = 0; r < m_region_tags.size(); ++r) {
            if (tags.count(m_region_tags[r])) return int(r);
        }
        return -1;
    };
    std::vector<size_t> queue;
    for (const Tuple& e : get_edges()) {
        const std::optional<Tuple> opp = e.switch_face(*this);
        if (!opp) continue;
        const size_t fa = e.fid(*this), fb = opp->fid(*this);
        for (const auto [band, input] :
             {std::pair<size_t, size_t>{fa, fb}, std::pair<size_t, size_t>{fb, fa}}) {
            if (!face_is_offset_band(band) || !face_is_input_complex(input)) continue;
            const int r = region_of_input_face(input);
            if (r < 0) continue;
            if (m_face_region[band] == -1) {
                m_face_region[band] = r;
                queue.push_back(band);
            } else if (m_face_region[band] >= 0 && m_face_region[band] != r) {
                m_face_region[band] = -2;
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
    std::vector<size_t> n_faces(m_region_tags.size(), 0);
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
    // PHASE A IS TRIWILD, so its metric is TriWild's: element quality alone, in ABSOLUTE AMIPS
    // against optimization_stop_metric() = stop_energy, with no Phi term in the stop test, the
    // stall test or the refinement ranking. The offset is not unattended there --
    // m_offset_envelope holds it -- and mixing Phi back in is what made the two criteria fight.
    //
    // DELEGATED, not reimplemented, and the units are the whole reason. See
    // optimization_stop_metric() for what returning a normalized number here instead did to the
    // 3D Phase A: filter 100 against a worst element of 97, no refinement, 20 bit-identical
    // iterations.
    if (m_phase == OptPhase::A) {
        return wmtk::TriOptimizerMesh::optimization_quality_stats();
    }

    // The engine's "quality" is the offset's own criterion: (max, avg) PHI RESIDUAL over the
    // band, normalized so 1.0 is the tolerance, maxed with AMIPS on the same scale.
    const DistanceSplit r = residual_split();

    // THE RUNAWAY GUARD, before anything reports a number derived from Phi. A vertex outside the
    // support has a residual that saturates rather than growing, so the numbers below would
    // under-report it, and no smoothing move can bring it back. Checked here because this is
    // what the driver calls every iteration, and because residual_split() has already paid for
    // the measurement.
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
        // ALONG the edge as well, so a face carrying a stretch of band too coarse to represent
        // the offset is refined -- which is the mechanism that keeps the band resolved at all.
        score = std::max(score, offset_edge_samples(e).max / tol);
    }
    return score;
}

size_t TopoOffsetTriMesh::refine_sizing_around_worst(const double max_metric)
{
    // TriWildMesh::refine_sizing_around_worst verbatim -- ranked by
    // m_face_attribute[].m_quality, filtered against stop_energy, seeding the same force-split
    // edges. Phase A is TriWild, and that includes how it responds to a stall; the field it
    // writes is the same field Phase B's update_band_sizing_from_tolerance() writes, so the
    // refinement each phase asks for accumulates rather than competing.
    //
    // PHASE A ONLY, by construction: mesh_improvement() is this function's one caller, and the
    // alternating driver only ever runs that as Phase A. A second branch used to live here for
    // a Phase B stall, scored by face_criterion_rel() in normalized units; it was unreachable
    // in the alternating driver and is deleted -- Phase B's refinement question belongs to
    // update_band_sizing_from_tolerance().
    //
    // NOTE THE UNITS: max_metric arrives from mesh_improvement() as whatever
    // optimization_quality_stats() returned, which in Phase A is ABSOLUTE AMIPS, and the score
    // it is compared against is absolute too. Mixing units is the bug that cost 3D twenty
    // iterations.
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
    // WHAT IS BLOCKING THE FIX, next to WHAT IS BROKEN. Same filter, so the two censuses cover
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
    // The band split, first: how much of the error is the optimizer's to fix. The loop and the
    // sizing field only ever see the reachable half, so a run whose reported max looks bad but
    // whose reachable max is fine is a construction problem, not an optimization one. Both
    // quantities are reported -- the Phi residual, which the loop converges on, and the
    // Euclidean distance, which says how far the smoothed offset ended up from the exact one.
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
    } else if (m_phase == OptPhase::B && ve.m_is_on_offset) {
        fate = "Phase B: the local root find on (Phi - c)^2";
    } else if (m_phase == OptPhase::B) {
        fate = "Phase B: interior AMIPS, or refused if it carries a surface";
    }
    logger().info("\t  smoothing fate: {}", fate);

    // Every incident edge, with the tag sets of the two faces across it. This is the ground
    // truth the classification is derived from: label is a 3-way coarsening of these sets, so
    // the pair of sets says exactly why an edge landed in the class it did, and whether a curve
    // that looks like it should continue through this vertex actually does.
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
    // label_offset_boundary() and the next. Same rule compute_distance_deviation() applies --
    // a band face meeting a face that is neither band nor input complex.
    const std::optional<Tuple> opp = e.switch_face(*this);
    if (!opp) return false;
    const size_t fa = e.fid(*this), fb = opp->fid(*this);
    const bool a = face_is_offset_band(fa), b = face_is_offset_band(fb);
    if (a == b) return false;
    return !face_is_input_complex(a ? fb : fa);
}

void TopoOffsetTriMesh::check_no_vertex_on_both_surfaces(const char* when) const
{
    // A VERTEX ON BOTH SURFACES IS UNSATISFIABLE, not merely awkward. It sits at distance 0 from
    // the input complex, where Phi diverges, and is simultaneously required to sit on the level
    // set at target_distance from it. No placement satisfies both, so no amount of smoothing or
    // refinement can fix it -- which is why the rest of the component quietly steps around it:
    // smooth_before() refuses it in Phase B, band_vertex_is_reachable() drops it from the
    // gradient metric, and residual_split() books it under max_pinned. Stepping around it is right
    // for the measurement and wrong as a response: a construction defect can otherwise sit in the
    // mesh contributing nothing but a boundary that cannot be placed, and never say so.
    //
    // ON THE DOMAIN BOUNDARY IS DIFFERENT and deliberately not checked here. Such a vertex is
    // constrained but not contradictory, and it is a legitimate outcome of growth meeting the
    // edge of the domain.
    //
    // Checked after every phase, not only at construction: collapse_after_vertex() ORs the flags
    // onto the surviving vertex, so a collapse that merges an offset vertex into an input one
    // CREATES this state out of two individually fine vertices.
    std::vector<size_t> both;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!m_vertex_extra[vid].m_is_on_offset || !m_vertex_extra[vid].m_is_on_input) {
            continue;
        }
        // THE GEOMETRY DECIDES, NOT THE FLAGS. m_is_on_input is over-broad: the split
        // propagates it onto new vertices and the collapse ORs it onto survivors, so a
        // vertex can carry it while sitting a full target_distance from the complex --
        // which is to say sitting exactly where the offset wants it. Erroring on the flag
        // pair alone failed topological_offset_3d and _3d_edge_input at CONSTRUCTION, on
        // 134 and 119 vertices whose real distances were 0.11 to 0.20 against a
        // target_distance of 0.2: the message asserted they were at distance 0 while
        // printing distances that plainly were not.
        //
        // Unsatisfiable is a GEOMETRIC fact. smoothing_position_is_allowed() holds an
        // input-complex vertex within envelope_size of the complex, and the offset asks it
        // to reach target_distance; those two demands contradict each other only when the
        // vertex really is on the complex.
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

void TopoOffsetTriMesh::rebuild_offset_envelope()
{
    // FIRST, AND ON EVERY PATH OUT OF HERE INCLUDING THE EMPTY ONE. Each entry is an
    // IntersectionEnvelope holding a shared_ptr to the tube this call is about to replace, so a
    // surviving entry would hold a simplex to where the offset boundary was one round ago -- a
    // constraint tightening silently with every round, against geometry Phase B has already
    // moved on from. See m_offset_isect_cache.
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

    // ONE PHI TOLERANCE by default, so Phase A may move the offset by exactly as much as the
    // convergence test is willing to ignore. Tighter and Phase A cannot improve the elements
    // straddling the boundary at all; looser and it can undo a Phi that Phase B had already
    // brought inside tolerance.
    //
    // A STRAIGHT FRACTION OF target_distance, and nothing else. eps is a distance in model
    // units, target_distance is one too, so ab_offset_envelope_rel is a pure percentage: 0.025
    // means "hold the boundary to within 2.5% of the offset distance while Phase A runs".
    //
    // IT NO LONGER TRACKS THE CONVERGENCE CRITERION. It used to be
    // ab_offset_envelope_rel x offset_residual_tolerance(), which chained the tube to whatever
    // the criterion permitted -- and once the criterion became a fraction of a MEASURED
    // reference, that chain made the Phase A tube depend on how bad construction happened to be.
    // A tube whose width is set by the input's initial error is not a tube anyone can reason
    // about, so the two are now independent: the criterion says when the run is done, this says
    // how far Phase A may move the boundary while getting there.
    //
    // It is NOT envelope_size_rel, which is a fraction of the bounding-box diagonal and is what
    // m_envelope (the input-complex tube) is built from.
    const double eps =
        std::max(m_offset_params.ab_offset_envelope_rel * m_offset_params.target_distance, 1e-12);

    m_offset_envelope = std::make_shared<SampleEnvelope>();
    m_offset_envelope->init(verts, segs, eps);
    logger().info(
        "\t[offset envelope] rebuilt: {} segments, {} (eps {:.6g} = "
        "ab_offset_envelope_rel {:.4} x target_distance {:.6g})",
        segs.size(),
        m_offset_envelope->use_exact ? "EXACT" : "sampled",
        eps,
        m_offset_params.ab_offset_envelope_rel,
        m_offset_params.target_distance);
}

size_t TopoOffsetTriMesh::pin_interference_stalled_vertices(const size_t pass)
{
    // See the declaration for the four conditions and why condition 4 is the load-bearing one.
    const double c = m_offset_potential->target_level();

    size_t n_candidates = 0, n_pinned = 0;
    size_t n_fail_outside = 0, n_fail_not_tangential = 0, n_fail_not_min = 0;
    double worst_excess = 0.; ///< max Phi/c over the pinned set
    size_t worst_vid = 0;
    double worst_cos = 0.;

    std::fill(m_interference_pinned.begin(), m_interference_pinned.end(), char(0));

    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!m_vertex_extra[vid].m_is_on_offset) continue;
        // EVERY offset vertex is a candidate: the geometry decides, not the solver. The stop
        // reason used to gate this and it never fired -- see the declaration.
        ++n_candidates;

        const Vector2d x = m_vertex_attribute[vid].m_posf;
        const double phi = potential_for(vid).value(x);
        if (!std::isfinite(phi) || !(phi > c)) { // 2. strictly inside the offset region
            ++n_fail_outside;
            continue;
        }
        const Vector2d n = offset_vertex_normal(vid);
        const Vector2d g = potential_for(vid).gradient(x);
        if (!(n.norm() > 0.) || !g.allFinite()) continue;

        const double gn = g.norm();
        const double cosn = (gn > 0.) ? std::abs(g.dot(n)) / gn : 0.;
        if (!(cosn <= kInterferenceTangentialRel)) { // 3. no progress available along n
            ++n_fail_not_tangential;
            continue;
        }
        const Eigen::Matrix2d H = potential_for(vid).hessian(x);
        if (!H.allFinite() || !(n.dot(H * n) > 0.)) { // 4. a GENUINE minimum of Phi along n
            ++n_fail_not_min;
            continue;
        }

        m_interference_pinned[vid] = 1;
        ++n_pinned;
        const double excess = (c > 0.) ? phi / c : 0.;
        if (excess > worst_excess) {
            worst_excess = excess;
            worst_vid = vid;
            worst_cos = cosn;
        }
    }

    if (n_candidates == 0) return 0;

    // LOUD, because this is the mechanism that lets a run converge by declaring part of the
    // surface impossible. Correct where the offset genuinely self-intersects; a mask over a real
    // failure where it does not. Never throws, by instruction -- the number is to be read.
    logger().warn(
        "\t[phase B] pass {}: INTERFERENCE-PINNED {} of {} offset vertices -- their level set "
        "does not exist (Phi > c, grad Phi tangential to n, Phi a minimum along n), so they are "
        "dropped from max_reachable | rejected: {} outside the offset region, {} gradient not "
        "tangential, {} not a minimum along n",
        pass,
        n_pinned,
        n_candidates,
        n_fail_outside,
        n_fail_not_tangential,
        n_fail_not_min);
    if (n_pinned > 0) {
        logger().warn(
            "\t  worst interference pin: v{} at ({:.6g}, {:.6g}) | Phi/c {:.6g} (the level set is "
            "{:.4g}x too deep to reach here) | |cos(grad Phi, n)| {:.4g}",
            worst_vid,
            m_vertex_attribute[worst_vid].m_posf.x(),
            m_vertex_attribute[worst_vid].m_posf.y(),
            worst_excess,
            worst_excess,
            worst_cos);
    }
    return n_pinned;
}

size_t TopoOffsetTriMesh::phase_b_smooth()
{
    // SMOOTHING ONLY, BY GAUSS-SEIDEL SWEEPS. No topology: Phase B's single job is to move the
    // offset boundary onto the level set, and the mesh it does that on is whatever Phase A left.
    // Each pass gives every eligible vertex exactly ONE local iteration and the phase runs
    // ab_phase_b_iterations of them, exiting early only on the run's convergence criterion.
    //
    // WHAT THIS COSTS THE SIZING UPDATE, and it must be read with the change:
    // update_band_sizing_from_tolerance() rests on "Phase B has just run to a fixed point, so an
    // offset edge still over tolerance is under-resolved rather than badly placed". A FIXED
    // BUDGET DOES NOT ESTABLISH THAT -- a pass count that simply ran out leaves badly-placed and
    // under-resolved indistinguishable, and the sizing field will refine around vertices that
    // merely had not finished moving. The per-pass residual and gradient lines below are what
    // tell the two apart: a run whose gradient is still falling at the last pass hit the budget,
    // not a fixed point, and its sizing update should be distrusted.
    std::vector<Vector2d> before(vert_capacity());
    assign_band_regions();

    // THE CRITERION IS THE GRADIENT, relative to its value at phase entry. Zero exactly at the
    // Gauss-Seidel fixed point, so unlike the displacement test it cannot read converged when
    // moves are blocked (a refused move has zero displacement and full gradient), and it keeps
    // going while sweeps still lower the energy. Entry-relative makes it scale-free across
    // rounds whose Phase A left very different amounts of work.
    audit_phase_b_offset_envelope_holds();

    // OPT-IN. Off by default: pinning changes which vertices the convergence criterion is
    // allowed to ignore, which is not something to turn on silently.
    static const bool s_interference_pin = [] {
        const char* e = std::getenv("WMTK_OFFSET_INTERFERENCE_PIN");
        return e && std::string(e) != "0" && std::string(e) != "off";
    }();
    if (s_interference_pin) {
        m_placement_stalled.assign(vert_capacity(), 0);
        m_interference_pinned.assign(vert_capacity(), 0);
        logger().info(
            "\t[phase B] interference pinning is ON (WMTK_OFFSET_INTERFERENCE_PIN): offset "
            "vertices whose step stalls against another front are dropped from max_reachable");
    }

    const double g_entry = phase_b_band_gradient_linf();
    // TWO BARS, WHICHEVER COMES FIRST. The entry-relative one asks "has this phase finished the
    // work it was handed"; the ABSOLUTE one is the run's own convergence bar, and once the band
    // is under it there is nothing left for another round to do.
    const double g_abs = offset_gradient_tolerance();
    logger().info(
        "\t[phase B] placement gradient at entry {:.6g}; the run's convergence bar is {:.6g}",
        g_entry,
        g_abs);
    if (g_entry <= 0.) {
        return 0; // already at the fixed point; nothing to smooth
    }

    // A FIXED GAUSS-SEIDEL BUDGET, and the run's own convergence criterion is the ONLY early
    // exit. The previous exits are deliberately gone: `constrained == 0` and the 10-pass
    // no-progress plateau both described a phase that SOLVED each vertex, where "nothing was
    // backtracked" meant every local minimum had been reached. Under one step per pass neither
    // means that -- a single short step is rarely blocked by its ring, so `constrained == 0`
    // fires on pass 1 and would end the phase having barely moved. They are kept as LOG lines
    // below, because they still say something about the configuration; they just no longer
    // decide anything.
    const size_t cap = size_t(std::max(1, m_offset_params.ab_phase_b_iterations));
    double g_prev = g_entry;
    int no_progress = 0;
    int constrained_prev = std::numeric_limits<int>::max();
    size_t pass = 0;
    double disp = 0.;
    for (; pass < cap; ++pass) {
        for (size_t i = 0; i < vert_capacity(); ++i) {
            before[i] = m_vertex_attribute[i].m_posf;
        }

        // TWO ORDERED SUB-SWEEPS. First place every offset-boundary vertex on the level set,
        // then let every free interior vertex relax against the mesh that placement just
        // distorted. Ordered rather than interleaved so the background always relaxes against
        // the offset's FINAL position for this pass.
        m_phase_b_constrained = 0;
        m_phase_b_pressed = 0;
        if (pass == 0) m_placement_pressed.assign(vert_capacity(), 0);
        m_placement_trace.reset(); // only the Offset sub-sweep books into it
        m_phase_b_sub = PhaseBSub::Offset;
        m_debug_pass_name = "B-offset";
        smooth_all_vertices(1);
        // Captured between the sweeps: only the offset sub-sweep can constrain a placement, and
        // smooth_all_vertices() resets the shared reject counters, so a before/after delta on
        // `accepted` would wrap.
        const int constrained = m_phase_b_constrained.load();
        const size_t placed = m_smooth_rejects.accepted.load();

        m_phase_b_sub = PhaseBSub::Background;
        m_debug_pass_name = "B-background";
        smooth_all_vertices(1);
        const size_t relaxed = m_smooth_rejects.accepted.load();

        disp = 0.;
        for (const Tuple& v : get_vertices()) {
            const size_t vid = v.vid(*this);
            disp = std::max(disp, (m_vertex_attribute[vid].m_posf - before[vid]).norm());
        }
        // Relative to the target edge length, so the test means the same thing at any scale.
        const double tol = m_offset_params.ab_smooth_tol * std::max(m_params.l, 1e-16);

        // THE RESIDUAL PER PASS, not just once at the end of the phase.
        //
        // update_band_sizing_from_tolerance() rests on "Phase B has just run to a fixed point,
        // so an offset edge still over tolerance is under-resolved rather than badly placed".
        // That is only true if this loop EXITS ON the gradient criterion below rather than the
        // pass cap. This line is what tells the two apart -- a residual still falling at the cap
        // means the cap is the binding constraint, a residual flat while the gradient is
        // converged means the boundary is genuinely stuck and refinement is the right answer.
        const DistanceSplit rp = residual_split();
        const double rtol = offset_residual_tolerance();
        logger().info(
            "\t[phase B] pass {}: max vertex displacement {:.6g} (tol {:.6g}) | residual {:.4}x "
            "(at-vertex {:.4}x, in-edge {:.4}x) | reachable {} pinned {}",
            pass + 1,
            disp,
            tol,
            rp.max_reachable / rtol,
            rp.max_at_vertex / rtol,
            rp.max_in_edge / rtol,
            rp.n_reachable,
            rp.n_pinned);
        // BEFORE the gradient is measured, so the pass's own reported number already excludes
        // vertices this pass found to be chasing a level set that is not there.
        if (s_interference_pin) pin_interference_stalled_vertices(pass + 1);
        const double g = phase_b_band_gradient_linf();
        logger().info(
            "\t[phase B] pass {}: placement gradient {:.6g} ({:.4g}x entry, {:.4g}x the "
            "convergence bar {:.6g})",
            pass + 1,
            g,
            g_entry > 0. ? g / g_entry : 0.,
            g / g_abs,
            g_abs);
        logger().info(
            "\t[phase B] pass {}: placed {} constrained {} pressed {} | background relaxed {}",
            pass + 1,
            placed,
            constrained,
            m_phase_b_pressed.load(),
            relaxed);
        // `placed` counts every visit that returned true, INCLUDING the ones that put the vertex
        // back exactly where it started. This is the breakdown that tells those apart.
        log_placement_trace();
        // ONE FRAME PER SMOOTHING PASS. write_smoothing_debug_output() renames it into the
        // single `step_<NNNNN>_r<round>B` timeline the phase A passes also land in, so the two
        // phases interleave in the order they actually ran. Opt-in through
        // DEBUG_output_per_pass -- see the override, which drops debug_ frames unless it is set.
        // No frame here: smooth_all_vertices() already wrote one after each of the two sweeps
        // above, and a third would be the background sweep's mesh again. (Comment above kept for
        // the naming it describes; the writes it refers to are the two sweeps'.)
        // THE NATURAL EXIT. Every offset vertex reached its unconstrained minimum inside its own
        // one-ring, so nothing had to be backtracked -- the fixed point this scheme is defined
        // to seek. Checked before the gradient bar because it is the stronger statement.
        if (constrained == 0) {
            logger().info(
                "\t[phase B] pass {}: no placement was constrained by its one-ring",
                pass + 1);
        }
        // The run's own convergence bar: once the band is under it there is nothing another
        // round could do with a better-placed boundary.
        // Under the "dist_and_orient" criterion the vertex half is judged in length units instead,
        // the same bar the loop will apply; the gradient is still logged above for comparison.
        bool under_bar = g <= g_abs;
        if (use_distance_criterion()) {
            const DistanceCriterion dc = distance_criterion(/*include_edges=*/false);
            under_bar = dc.placed;
            logger().info(
                "\t[phase B] pass {}: dist_and_orient, vertices: max {:.6g} = {:.4}x the bar "
                "({} outside support)",
                pass + 1,
                dc.max_vertex_dist,
                dc.max_vertex_rel,
                dc.n_outside_support);
        }
        if (under_bar) {
            ++pass;
            break;
        }
        // BOTH measures must stall. The gradient can sit flat while the constrained count is
        // still falling -- vertices are still being freed from their one-rings even though the
        // worst placement error has not moved -- and stopping on the gradient alone would cut
        // that short. A non-finite gradient is never progress: written as `g < g_prev` alone an
        // alternating inf -> finite sequence reads as a decrease every other pass, resets this
        // counter, and with an uncapped Phase B never terminates.
        const bool g_better = std::isfinite(g) && g < g_prev;
        const bool c_better = constrained < constrained_prev;
        no_progress = (g_better || c_better) ? 0 : no_progress + 1;
        g_prev = g;
        constrained_prev = std::min(constrained_prev, constrained);
        if (no_progress >= 10) {
            // REPORTED, NOT ACTED ON. It still names a stalled configuration, which is worth
            // seeing; but the phase now runs its fixed budget either way.
            logger().warn(
                "\t[phase B] neither the placement gradient nor the constrained count has "
                "improved for {} passes (gradient {:.6g}, {:.4g}x entry, {} still constrained)",
                no_progress,
                g,
                g_entry > 0. ? g / g_entry : 0.,
                constrained);
        }
    }
    return pass;
}

size_t TopoOffsetTriMesh::update_band_sizing_from_tolerance()
{
    // See the header for the rule. Three passes so each quantity is computed once: Phi's
    // gradient is the expensive part and the edge samples dominate it.
    const double gtol = offset_gradient_tolerance();
    const std::vector<bool> on_band = band_vertex_mask();
    std::vector<std::unique_ptr<OffsetEnergy2D>> energies;
    for (const auto& rp : m_region_potentials)
        energies.push_back(std::make_unique<OffsetEnergy2D>(rp, 1.0));
    OffsetEnergy2D union_energy(m_offset_potential, 1.0);
    const auto grad_at = [&](const int region, const Vector2d& p) {
        Eigen::VectorXd g(2);
        OffsetEnergy2D& en = (region >= 0 && size_t(region) < energies.size())
                                 ? *energies[size_t(region)]
                                 : union_energy;
        en.gradient(Eigen::VectorXd(p), g);
        return g.norm();
    };
    // Under the "dist_and_orient" criterion the same two questions are judged in length units, by
    // the
    // same bar the A/B loop gates on -- otherwise this rule would refine toward one criterion
    // and the loop would judge by another. See distance_criterion().
    const bool use_dist = use_distance_criterion();
    const double dist_bar =
        m_offset_params.convergence_distance_rel * m_offset_params.target_distance;
    const auto within_tol = [&](const int region, const Vector2d& p) -> bool {
        if (!use_dist) return grad_at(region, p) <= gtol;
        const OffsetPotential2D& P = potential_for_region(region);
        const double r = P.value(p) - P.target_level();
        const double gn = P.gradient(p).norm();
        if (!std::isfinite(r) || !std::isfinite(gn) || gn <= 1e-300) return false;
        return std::abs(r) / gn <= dist_bar;
    };

    // 1. Every band vertex against the criterion. Non-band vertices are left `true` so they
    //    never veto a neighbour's halving -- the rule is about the BOUNDARY one-ring.
    std::vector<char> in_tol(vert_capacity(), 1), is_band(vert_capacity(), 0);
    // Pressed vertices (see m_placement_pressed) are neither misplaced nor under-resolved: the
    // seam is where the level set does not exist, and halving there would refine it forever.
    const auto pressed = [&](const size_t vtx) {
        return vtx < m_placement_pressed.size() && m_placement_pressed[vtx];
    };
    size_t n_pressed_v = 0, n_pressed_e = 0;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid] || !m_vertex_extra[vid].m_is_on_offset) continue;
        if (!m_vertex_attribute[vid].m_is_rounded) continue; // not placeable, not this rule's
        is_band[vid] = 1;
        if (pressed(vid)) {
            in_tol[vid] = 1;
            ++n_pressed_v;
            continue;
        }
        in_tol[vid] = within_tol(vertex_region(vid), m_vertex_attribute[vid].m_posf) ? 1 : 0;
    }

    // 2. THE RULE, PER EDGE: if BOTH endpoints are in tolerance but ANY interior sample on the
    //    edge is not, BOTH endpoints are marked for halving.
    //
    //    THE MARK IS A BOOLEAN, WHICH IS WHAT STOPS HALVINGS FROM STACKING. A vertex between two
    //    such edges is marked twice and halved ONCE -- it gets 1/2, never 1/4. Refinement is a
    //    request for "finer here", not a tally of how many neighbours agree, and compounding it
    //    would refine a straight run of under-resolved boundary twice as hard as its ends for no
    //    geometric reason.
    //
    //    BOTH ENDPOINTS IN TOLERANCE is the whole precondition. If either is out, the edge's
    //    sample is over the bar because a VERTEX is misplaced, not because the chord is
    //    under-resolved -- refining there answers the wrong question, and Phase B still owns it.
    //    This is narrower than the rule it replaces, which additionally demanded the vertex's
    //    ENTIRE one-ring be in tolerance: that let one misplaced vertex veto refinement on every
    //    edge around it, including edges whose own two endpoints were both fine.
    std::vector<char> mark(vert_capacity(), 0);
    size_t n_bad_edges = 0, n_edges_seen = 0;
    for (const Tuple& e : get_edges()) {
        if (!edge_is_offset_surface_live(e)) continue;
        ++n_edges_seen;
        const std::array<size_t, 2> vs = {{e.vid(*this), e.switch_vertex(*this).vid(*this)}};
        if (pressed(vs[0]) || pressed(vs[1])) {
            ++n_pressed_e;
            continue;
        }
        if (!in_tol[vs[0]] || !in_tol[vs[1]]) continue;
        bool edge_bad = false;
        for_each_offset_edge_sample(e, [&](const Vector2d& q) {
            if (!edge_bad && !within_tol(edge_region(vs[0], vs[1]), q)) edge_bad = true;
        });
        if (!edge_bad) continue;
        ++n_bad_edges;
        mark[vs[0]] = 1;
        mark[vs[1]] = 1;
    }

    // 3. Apply. The floor is the BAND's, not stuck_refine_min_scalar.
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / m_params.l);
    std::vector<size_t> changed;
    size_t n_halved = 0, n_misplaced = 0, n_at_floor = 0;
    for (size_t vid = 0; vid < vert_capacity(); ++vid) {
        if (!is_band[vid]) continue;
        // MISPLACED, not under-resolved: leave it to Phase B. Counted so the two reasons a
        // vertex goes unrefined stay distinguishable in the log -- they are opposite diagnoses.
        if (!in_tol[vid]) {
            ++n_misplaced;
            continue;
        }
        if (!mark[vid]) continue; // in tolerance and every incident chord is too
        double& sc = m_vertex_attribute[vid].m_sizing_scalar;
        const double ns = std::max(s_floor, sc * 0.5);
        if (ns < sc) {
            sc = ns;
            ++n_halved;
            changed.push_back(vid);
        } else {
            ++n_at_floor; // wanted refining and cannot -- the floor is binding here
        }
    }
    if (!changed.empty()) gradation_smooth_sizing(m_offset_params.sizing_gradation, changed);
    const size_t n_band = size_t(std::count(is_band.begin(), is_band.end(), char(1)));
    logger().info(
        "\t[phase B] band sizing: {} of {} vertices halved (once each, never compounded), from "
        "{} of {} offset edges whose two endpoints are BOTH in tolerance while an interior "
        "sample is not | not refined: {} misplaced (vertex out of tolerance -- Phase B's, not "
        "this rule's), {} at the floor, {} nothing wrong | floor {:.6g}",
        n_halved,
        n_band,
        n_bad_edges,
        n_edges_seen,
        n_misplaced,
        n_at_floor,
        n_band - n_halved - n_misplaced - n_at_floor,
        s_floor);
    if (n_pressed_v > 0 || n_pressed_e > 0) {
        logger().info(
            "\t[phase B] band sizing: {} pressed vertices and {} edges touching one left alone "
            "(the seam is not a resolution question)",
            n_pressed_v,
            n_pressed_e);
    }
    return changed.size();
}

void TopoOffsetTriMesh::optimize_offset_alternating()
{
    const int rounds = std::max(1, m_offset_params.ab_max_rounds);
    const int a_iters = std::max(1, m_offset_params.ab_phase_a_iterations);

    // Before anything runs, so a construction defect is reported as one rather than surfacing
    // later as a residual that will not converge.
    check_no_vertex_on_both_surfaces("construction");
    log_region_edge_mask_health("construction");
    // BEFORE ANY OPERATION RUNS. An edge already outside here is a construction defect, which is
    // a different bug from one a split/collapse/swap created -- and the shared sanity check
    // cannot tell the two apart because it only runs inside the pass driver.
    audit_surface_containment("construction");

    // FIRST, because every tolerance below is a fraction of what this measures. Until it runs,
    // offset_gradient_tolerance() is at its 1e-16 floor and offset_residual_tolerance() with it,
    // so any ratio logged before this point would be meaningless.
    needle_scan("after construction, before the A/B loop");
    const GradientSplit construction_grad = measure_gradient_reference();

    // Where the mesh as constructed stands, before the loop touches it. A diagnostic: it is the
    // baseline every later round's residual is read against.
    const double construction_residual = residual_split().max_reachable;
    logger().info(
        "\tconstruction residual: {:.6g} ({:.4}x tolerance)",
        construction_residual,
        construction_residual / offset_residual_tolerance());
    logger().info(
        "\tconstruction placement gradient: {:.6g} ({:.4}x tolerance) | {} reachable, {} pinned, "
        "{} skipped",
        construction_grad.max_reachable,
        construction_grad.max_reachable / offset_gradient_tolerance(),
        construction_grad.n_reachable,
        construction_grad.n_pinned,
        construction_grad.n_skipped_unrounded + construction_grad.n_skipped_inverted);

    logger().info("[experiment switches] w_amips {:g} (Phase A only)", m_params.w_amips);

    // PER-ROUND OPERATION COUNTS. iter_cnt_* are cumulative from their reset at the top of
    // optimize_offset(), so each round's contribution is the delta against the previous round's
    // totals. Phase B performs no topological operations, so a round's counts are exactly what
    // its Phase A did.
    int prev_split = iter_cnt_split.load();
    int prev_collapse = iter_cnt_collapse.load();
    int prev_swap = iter_cnt_swap.load();
    int prev_born = iter_cnt_split_born.load();
    int prev_recol = iter_cnt_recollapsed.load();
    int prev_recol_same = iter_cnt_recollapsed_same_pass.load();

    for (int round = 0; round < rounds; ++round) {
        // Tags every debug frame this round writes; see write_smoothing_debug_output().
        m_ab_round = round + 1;
        // ---- PHASE A: TriWild, with the offset held inside its envelope ----
        logger().info("======== A/B round {} / {}: phase A ========", round + 1, rounds);
        audit_surface_containment(fmt::format("round {} phase A entry", round + 1));
        // CLEARED BEFORE ANY TOPOLOGY RUNS. The pins are per-pass evidence about specific
        // vertices; Phase A recycles vertex slots, so a flag left set here would land on an
        // unrelated vertex created by a split. Phase B recomputes them from scratch anyway.
        std::fill(m_interference_pinned.begin(), m_interference_pinned.end(), char(0));
        m_phase = OptPhase::A;
        // DIAGNOSTIC; see ab_no_collapse_after_first_round. Round 1 keeps its collapses because
        // the mesh as constructed genuinely needs them.
        m_ab_collapses_disabled = m_offset_params.ab_no_collapse_after_first_round && round > 0;
        if (m_ab_collapses_disabled) {
            logger().warn("\t[phase A] COLLAPSES DISABLED (ab_no_collapse_after_first_round)");
        }
        mesh_improvement(a_iters);
        assign_band_regions(); // Phase A changed the topology; the band-to-region map follows

        // ASK THE LOOP, in the loop's own units. optimization_quality_stats() reports ABSOLUTE
        // AMIPS against optimization_stop_metric() = stop_energy in Phase A, so a check written
        // in normalized units fails a Phase A that converged.
        const double amips = std::get<0>(optimization_quality_stats());
        const double bar = optimization_stop_metric();
        if (m_offset_params.debug_output) {
            // THE FRAME THE PHASE HANDS ON. write_smoothing_debug_output() renames it
            // `step_<NNNNN>_r<round>A_end`, so it sits in the one timeline at the right place
            // and is still identifiable as a phase boundary.
            write_smoothing_debug_output(fmt::format("phase_{}A", round + 1));
        }

        // PHASE A HAS TO CONVERGE. It is TriWild on a mesh TriWild can improve, with the offset
        // pinned to a tolerance-wide tube; if element quality is still above stop_energy when
        // the loop gives up, something is wrong that iterating further will not fix, and
        // continuing into Phase B would optimize the offset on a mesh that cannot carry it.
        if (amips > bar) {
            // Attribute the failure before throwing: the throw's own text offers two guesses
            // (envelope too tight / unfixable elements) and this is what tells them apart.
            log_refine_block_census(fmt::format("round {} phase A gave up", round + 1), bar);
            log_and_throw_error(
                "Phase A did not converge within {} iterations: max element quality {:.6} against "
                "stop_energy {}. The offset envelope may be too tight (ab_offset_envelope_rel "
                "{}), or the mesh has elements no operation can fix.",
                a_iters,
                amips,
                bar,
                m_offset_params.ab_offset_envelope_rel);
        }
        logger().info("\t[phase A] converged: max element quality {:.4} (stop {:.4})", amips, bar);
        // Phase A collapses can merge an offset vertex into an input one, and
        // collapse_after_vertex() ORs both flags onto the survivor.
        check_no_vertex_on_both_surfaces(fmt::format("round {} phase A", round + 1).c_str());

        // ---- PHASE B: smoothing only, and the sizing update ----
        logger().info("======== A/B round {} / {}: phase B ========", round + 1, rounds);
        m_phase = OptPhase::B;
        // NOT RELEASED. The pointer stays valid for the whole run; what makes the offset free to
        // move is the phase, which containment_for() tests in the one place the constraint is
        // assembled. Nulling it here was the old way of saying "not applicable", and it made the
        // pointer's lifetime carry a meaning that belongs to the phase -- so anything reading it
        // outside Phase A could not tell "no constraint applies now" from "none was ever built".
        // The refresh happens at the END of this phase, below, around wherever it leaves the
        // boundary.

        // RECLAIM THE SLOT POOL, or the next Phase A cannot split anything at all. The pool is
        // preallocated at preallocation_factor x the live count AT THE LAST CONSOLIDATE, and
        // slots consumed by operations are only returned by a consolidate -- not by the
        // collapses that removed the elements. A Phase A that meets stop_energy on its first
        // iteration exits without ever consolidating, and every later round inherits whatever
        // the pool was left in. See the 3D twin for the measurement.
        const size_t vcap_before = vert_capacity(), fcap_before = tri_capacity();
        consolidate_mesh();
        logger().info(
            "\t[phase B] consolidated: slot capacity #V {} -> {}, #F {} -> {}",
            vcap_before,
            vert_capacity(),
            fcap_before,
            tri_capacity());

        const size_t passes = phase_b_smooth();

        const DistanceSplit r = residual_split();
        report_outside_support("A/B round", r);
        // THE LOOP'S CONVERGENCE TEST IS THE GRADIENT. The Phi residual and the Euclidean
        // distance error are both still measured every round and both still logged, because they
        // answer questions this one cannot -- the residual carries the chord term that ranks the
        // sizing field, the Euclidean error says how far the smoothed offset ended up from the
        // exact one -- but neither gates the run. See offset_gradient_tolerance().
        const GradientSplit g = gradient_split();
        // BOTH HALVES. Phase B has just placed every vertex it can; whether the chords BETWEEN
        // those vertices still cut the level set is the other half of the same question, and it
        // is the half update_band_sizing_from_tolerance() -- called right below when this fails
        // -- exists to answer. Gating on only the vertex half meant the round refined on the
        // chord term and then declined to be judged by it.
        double phi = std::max(g.max_reachable, g.max_in_edge) / offset_gradient_tolerance();
        // Under the "dist_and_orient" criterion that geometric ratio gates instead. The gradient
        // line below is still logged every round, so one run shows both criteria side by side.
        if (use_distance_criterion()) {
            const DistanceCriterion dc = distance_criterion();
            phi = dc.ratio();
            logger().info(
                "\t[phase B] dist_and_orient: vertices {:.4}x, edge samples {:.4}x of the bar "
                "{:.6g} (= {} x target_distance) | worst edge angle {:.2f} deg (max {:.1f}), {} "
                "folded | {} points outside support | {} vertices, {} edges, {} samples | pressed: "
                "{} vertices and {} edges touching one, excluded -> {}",
                dc.max_vertex_rel,
                dc.max_edge_rel,
                m_offset_params.convergence_distance_rel * m_offset_params.target_distance,
                m_offset_params.convergence_distance_rel,
                dc.worst_angle_deg,
                m_offset_params.convergence_orientation_max_deg,
                dc.n_folded,
                dc.n_outside_support,
                dc.n_vertices,
                dc.n_edges,
                dc.n_samples,
                dc.n_pressed,
                dc.n_edges_pressed,
                dc.converged() ? "CONVERGED" : "not converged");
        }
        logger().info(
            "\t[phase B] {} smoothing passes, max placement gradient {:.6g} = {:.4}x tolerance "
            "(in-edge diagnostic {:.4}x) | phi residual {:.4}x its own bar | {} reachable, "
            "{} pinned (max {:.6g}), {} skipped ({} unrounded, {} inverted ring)"
            "{}",
            passes,
            g.max_reachable,
            phi,
            g.max_in_edge / offset_gradient_tolerance(),
            r.max_reachable / offset_residual_tolerance(),
            g.n_reachable,
            g.n_pinned,
            g.max_pinned,
            g.n_skipped_unrounded + g.n_skipped_inverted,
            g.n_skipped_unrounded,
            g.n_skipped_inverted,
            // Only ever nonzero under the default placement term, and the invariant is 0 --
            // see m_placement_no_normal. Absent from the line entirely when there is nothing
            // to report, so a clean run's log does not carry a permanent zero.
            m_placement_no_normal.load()
                ? fmt::format(
                      " | {} placements had NO NORMAL (fell back to (Phi-c)^2)",
                      m_placement_no_normal.load())
                : std::string());

        // BEFORE the convergence check below, so the round that converges is recorded like every
        // other one rather than dropped by the early return.
        {
            const int sp = iter_cnt_split.load();
            const int c = iter_cnt_collapse.load();
            const int w = iter_cnt_swap.load();
            op_counts.push_back({{sp - prev_split, c - prev_collapse, w - prev_swap}});
            logger().info(
                "\t[A/B round {}] operations: {} splits, {} collapses, {} swaps "
                "(cumulative {} / {} / {})",
                round + 1,
                sp - prev_split,
                c - prev_collapse,
                w - prev_swap,
                sp,
                c,
                w);
            prev_split = sp;
            prev_collapse = c;
            prev_swap = w;

            // CHURN, in the same per-round shape. "recollapsed" counts split-born vertices a
            // collapse removed; "same pass" is the subset the very next collapse pass took out.
            const int b = iter_cnt_split_born.load();
            const int rc = iter_cnt_recollapsed.load();
            const int rs = iter_cnt_recollapsed_same_pass.load();
            const int db = b - prev_born, drc = rc - prev_recol, drs = rs - prev_recol_same;
            churn_counts.push_back({{db, drc, drs}});
            logger().info(
                "\t[A/B round {}] split churn: {} of {} split-born vertices recollapsed "
                "({:.1f}%), {} of them in the collapse pass right after their own split "
                "({:.1f}% of born)",
                round + 1,
                drc,
                db,
                db > 0 ? 100.0 * drc / db : 0.0,
                drs,
                db > 0 ? 100.0 * drs / db : 0.0);
            prev_born = b;
            prev_recol = rc;
            prev_recol_same = rs;
        }

        if (m_offset_params.debug_output) {
            // See the phase A twin above: the same timeline, marked `_end`.
            write_smoothing_debug_output(fmt::format("phase_{}B", round + 1));
        }

        // THE OFFSET ENVELOPE IS REFRESHED HERE, at the end of every Phase B, around wherever
        // this phase left the boundary -- which is what lets the boundary keep travelling across
        // rounds. Each Phase A pins it near its current position, each Phase B moves it, and
        // this hands the next Phase A the new position to pin. Unconditional, so the invariant
        // "the tube always describes the CURRENT offset" holds on the converged path too, where
        // callers after the loop (the report, the final sanity sweep) would otherwise read a
        // tube one Phase B out of date.
        rebuild_offset_envelope();

        if (phi <= 1.0) {
            logger().info(
                "A/B converged after {} round(s): amips {:.4}x, phi {:.4}x",
                round + 1,
                amips,
                phi);
            return;
        }

        // Not converged: re-size the band per vertex against the convergence criterion, and let
        // the next Phase A rebuild the mesh at that resolution.
        //
        // THIS REPLACES refine_sizing_where_phi_is_stuck() RATHER THAN JOINING IT. Both only
        // ever lower a scalar, but they disagree on WHICH vertices: the old routine refined
        // around any segment over tolerance, including one whose own vertex is misplaced rather
        // than under-resolved. That routine is deleted, as it is in 3D.
        update_band_sizing_from_tolerance();
    }

    logger().warn(
        "A/B did not converge in {} rounds (ab_max_rounds); the offset residual is still above "
        "tolerance",
        rounds);
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

    // THE OFFSET ENVELOPE IS BORN HERE, with the offset itself, and from then on it always
    // exists -- refreshed at the end of every Phase B, around wherever that phase left the
    // boundary. It used to be built at the top of each Phase A and set back to null on entering
    // Phase B, which left the pointer null for the whole of construction and the whole of every
    // Phase B; anything that reached for it in those windows silently got "no constraint"
    // instead of "not applicable in this phase". The phase test now lives in exactly one place
    // (containment_for()), so the pointer's lifetime and the constraint's applicability are
    // separate questions, which is what they always were.
    rebuild_offset_envelope();

    // The offset boundary as CONSTRUCTED must already be inside the potential's support, or
    // nothing the optimization does can move it. Checked before any operation runs so that a
    // construction defect is reported as one.
    check_offset_within_support("Offset as constructed");

    // Spelled out so the line reproduces its own number. The reference itself is not known
    // yet -- measure_gradient_reference() runs inside optimize_offset_alternating(), on the band
    // this function is still building -- so this states the SHAPE of the bound and the
    // reference line that follows it states the value.
    logger().info(
        "\tOffset criterion: |grad (Phi - c)^2 . n| <= convergence_gradient_norm_rel {} x "
        "max|grad (Phi - c)^2 . n| over the band AS CONSTRUCTED, with n the unit normal from "
        "the offset surface's own normal (Voronoi-weighted at vertices, the edge's own inside "
        "an edge). Measured over every band vertex and {} sample(s) "
        "per band edge; the reference is reported next, before the loop starts.",
        m_offset_params.convergence_gradient_norm_rel,
        offset_residual_samples());

    // Seed the sizing field from the offset's current edge lengths, before any operation runs.
    // Must come after the offset edges are classified above, since it reads them.
    init_offset_sizing_field();

    // UNCONDITIONAL, and it must stay that way: write_vtu() calls consolidate_mesh(), which
    // renumbers the mesh, which changes the order every subsequent pass enumerates operations
    // in, which changes the run. With the debug write below being the only consolidate here,
    // turning DEBUG_output on silently produced a DIFFERENT numerical result -- measured on the
    // dragon rectangle as converged-in-7 versus not-converged-in-10, identical in every other
    // parameter.
    consolidate_mesh();
    if (m_offset_params.debug_output) {
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    // THE SHARED ENGINE'S OWN LOOP, driven ALTERNATELY rather than jointly -- see OptPhase. Each
    // Phase A is one mesh_improvement() with a TriWild criterion; each Phase B is smoothing to a
    // fixed point followed by a sizing update read off the convergence criterion itself.
    //
    // The offset plugs into mesh_improvement() through the same virtuals simwild uses:
    //   - optimization_quality_stats(): per phase -- TriWild's own in A, the max of AMIPS and the
    //     offset residual in B;
    //   - optimization_stop_metric(): per phase, in the SAME units as the line above;
    //   - refine_sizing_around_worst(): TriWild's, fired only on a stall.
    // Placement is not among them: the offset boundary is placed by Phase B's own local solve.
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

    // FRAME 0 IS THE MESH AS CONSTRUCTED, before the optimization touches it. The shared driver
    // only writes a frame after each operation pass, so without this the timeline starts at the
    // end of the first split and there is nothing to compare against.
    if (m_params.debug_output) {
        m_debug_pass_name = "construction";
        write_smoothing_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
    }

    optimize_offset_alternating();

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
    // NO push_back HERE. op_counts is a per-A/B-round series now, recorded inside the driver
    // loop; appending the run totals as a final element would make the last entry mean something
    // different from every other one. The run total is the sum of the series.

    // Final metrics and the convergence verdict. One entry, for the whole run.
    //
    // THREE MEASURES, ONE CRITERION. Convergence is max_grad and nothing else -- the full
    // placement-gradient norm at band vertices, the same test every Phase B visit stopped on,
    // and the same statement for any potential. The in-edge samples are a chord diagnostic,
    // the Phi residual is the criterion's own quantity in length units, and the Euclidean
    // error says how far the smoothed offset ended up from the exact one -- all diagnostics,
    // none a criterion.
    assign_band_regions();
    const auto [max_dist, avg_dist] = compute_distance_deviation();
    const DistanceSplit r = residual_split();
    const GradientSplit g = gradient_split();
    const double tol = offset_residual_tolerance();
    const double gtol = offset_gradient_tolerance();
    logger().info(
        "placement gradient (at band vertices): max {} (avg {}) vs tolerance {} "
        "[convergence_gradient_norm_rel {}] | in-edge diagnostic {} ({} edge samples) | {} "
        "reachable, {} pinned (max {}), {} skipped ({} unrounded, {} inverted ring)",
        g.max_reachable,
        g.avg_reachable,
        gtol,
        m_offset_params.convergence_gradient_norm_rel,
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

    // BOTH HALVES, ONE BAR. The vertex max and the edge-interior max are the same quantity
    // against the same tolerance; placement answers the first and refinement answers the second,
    // and a run is done only when both are met. See gradient_split() for why the edge half used
    // to be excluded and why that was wrong.
    const double crit = std::max(g.max_reachable, g.max_in_edge);
    m_converged = crit <= gtol;
    if (use_distance_criterion()) {
        const DistanceCriterion dc = distance_criterion();
        m_converged = dc.converged();
        logger().log(
            m_converged ? spdlog::level::info : spdlog::level::warn,
            "{} [dist_and_orient]: placed {} (max vertex distance {:.6g} = {:.4}x the bar, "
            "worst vertex {}), resolved {} (max edge-sample distance {:.6g} = {:.4}x), oriented {} "
            "(worst edge angle {:.2f} deg vs {:.1f}, {} folded, at ({:.6g}, {:.6g})), {} points "
            "outside support | pressed (excluded): {} vertices, {} edges | bar {:.6g} = {} x "
            "target_distance | gradient criterion for comparison: {:.6g} vs {:.6g} ({:.4}x)",
            m_converged ? "Converged" : "Optimization did not converge",
            dc.placed,
            dc.max_vertex_dist,
            dc.max_vertex_rel,
            dc.worst_vid,
            dc.resolved,
            dc.max_edge_dist,
            dc.max_edge_rel,
            dc.oriented,
            dc.worst_angle_deg,
            m_offset_params.convergence_orientation_max_deg,
            dc.n_folded,
            dc.worst_edge_mid.x(),
            dc.worst_edge_mid.y(),
            dc.n_outside_support,
            dc.n_pressed,
            dc.n_edges_pressed,
            m_offset_params.convergence_distance_rel * m_offset_params.target_distance,
            m_offset_params.convergence_distance_rel,
            crit,
            gtol,
            crit / gtol);
    }
    if (m_converged && !use_distance_criterion()) {
        logger().info(
            "Converged ([max placement gradient over band vertices AND edge-interior samples] "
            "{} <= {} [convergence_gradient_norm_rel x reference]); at vertices {}, in edges {}"
            " | pinned, not gating: vertices {}, in edges {}",
            crit,
            gtol,
            g.max_reachable,
            g.max_in_edge,
            g.max_pinned,
            g.max_in_edge_pinned);
        // A band with unmeasured vertices is not a fully measured band, and the criterion is a
        // max: saying so at the moment of the verdict is the only place it cannot be missed.
        if (const size_t skipped = g.n_skipped_unrounded + g.n_skipped_inverted; skipped > 0) {
            logger().warn(
                "Converged with {} band vertices NOT measured ({} unrounded, {} with an inverted "
                "ring). The smoother refuses to place these, so their gradient is not part of the "
                "fixed point -- but the verdict above is a max over the rest.",
                skipped,
                g.n_skipped_unrounded,
                g.n_skipped_inverted);
        }
        if (g.n_pinned > 0 && g.max_pinned > gtol) {
            logger().warn(
                "{} pinned band vertices are over the gradient tolerance (max {} > {}). These are "
                "on the domain boundary, where construction ran out of room, so no further round "
                "can lower this -- it is a construction defect, not a convergence one.",
                g.n_pinned,
                g.max_pinned,
                gtol);
        }
    }

    if (!m_converged && !use_distance_criterion()) {
        // WHICH HALF FAILED IS THE WHOLE DIAGNOSIS, and the two want opposite remedies: the
        // vertex term is placement (Phase B), the edge term is resolution (the sizing update and
        // the Phase A that acts on it). Both are printed against the one bar, and the failing
        // one is named explicitly so the reader does not have to compare them by eye.
        const char* which = (g.max_reachable > gtol && g.max_in_edge > gtol)
                                ? "BOTH halves"
                                : (g.max_reachable > gtol ? "the VERTEX half (placement)"
                                                          : "the EDGE half (resolution)");
        logger().warn(
            "Optimization did not converge: {} over the bar. max placement gradient {} > {} "
            "[convergence_gradient_norm_rel x reference] -- at vertices {} (worst vertex {}), "
            "in edges {} | pinned, not gating: vertices {}, in edges {}",
            which,
            crit,
            gtol,
            g.max_reachable,
            g.worst_vid,
            g.max_in_edge,
            g.max_pinned,
            g.max_in_edge_pinned);

        // The "blocked by topological preservation" warning that used to sit here is gone with
        // the growth pass. respect_all_topologies gated offset_tri_consistent_topology(), which
        // only grow_offset_conservative() ever called, so the flag now constrains nothing and
        // advising the user to change it would send them after a knob that no longer exists. If
        // topological blocking needs a diagnostic again, it has to be written against the
        // operations that actually refuse on topology now: the link conditions in
        // split/collapse/swap.
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

    // The tracked surfaces are deliberately NOT re-derived here. From the moment they are
    // tagged, it is the shared operations that maintain those tags -- the face LABELS they were
    // derived from are construction data the optimization does not propagate, so re-deriving now
    // would read stale labels and mislabel the result.
}

} // namespace wmtk::components::topological_offset
