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
    const std::optional<Tuple> opp = t.switch_face(*this);
    if (!opp) {
        return true; // domain boundary
    }
    // Compare TAGS, not labels. The shared split propagates FaceAttributes, so a face it
    // creates carries correct tags immediately; its label is only refreshed at the top of the
    // next iteration, so reading that here would evaluate the link condition of the collapse
    // pass against faces whose labels are still whatever occupied the slot before.
    const size_t f0 = t.fid(*this), f1 = opp->fid(*this);
    return m_face_attribute[f0].tags != m_face_attribute[f1].tags;
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

    // The base refuses tracked-surface edges, which covers the input complex and the region
    // boundaries, but a bbox edge is tracked separately and would slip through. Flipping a
    // tracked edge re-triangulates the curve it belongs to and so moves it; that is true of the
    // input complex whether or not its vertices may move, which is why unfreezing the complex
    // changes nothing here.
    if (edge_is_on_domain_boundary(t.eid(*this))) {
        return false;
    }

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
    // m_offset_envelope (rebuilt each round, eps = ab_offset_envelope_rel x the residual
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
    const auto& VE = m_vertex_extra;

    // v1 is the vertex the collapse REMOVES (it merges into v2, which keeps its position). The
    // domain boundary may not lose a vertex: the box is not something to be coarsened.
    //
    // An INPUT-COMPLEX vertex may be removed, provided it merges onto another input-complex
    // vertex (the per-class rule below) and the result stays inside its tags' envelopes (the
    // shared collapse's own containment check) and preserves the substructure topology
    // (substructure_link_condition, applied unconditionally in collapse_edge_before). That is
    // TriWild's rule for its input surface, and it is what lets the complex be coarsened where
    // it is over-resolved instead of pinning every face incident to it.
    if (vertex_is_on_domain_boundary(v1_id)) {
        return false;
    }

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

void TopoOffsetTriMesh::collapse_after_vertex(const size_t v1_id, const size_t v2_id)
{
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
    // The base has already set m_is_on_surface from the split edge's own attributes and copied
    // those attributes -- surface class included -- onto the two halves. All that is left is
    // to say which of the two surfaces the new vertex joined.
    const auto& e = split_cache.local().old_e_attrs;
    const auto& c = m_opt_split_cache.local();
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
    // ASSIGNED, not OR'd -- v_id may be a recycled slot carrying a dead vertex's bits -- and
    // taken from the PARENT EDGE rather than from its endpoints, for the reason spelled out on
    // edge_boundary_bits(). Zero when the new vertex is on no region: the mask is inert there.
    m_vertex_extra[v_id].m_boundary_mask =
        m_vertex_extra[v_id].m_is_on_region ? c.edge_bits : uint64_t(0);
    // CHURN INSTRUMENTATION, read only by collapse_after_vertex(). Assigned for the same reason.
    m_vertex_extra[v_id].m_born_epoch = m_op_epoch;
    if (m_op_epoch != 0) ++iter_cnt_split_born;

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
        const bool enveloped = vertex_is_on_region(vid);

        if (m_phase_b_sub == PhaseBSub::Offset) {
            if (!is_offset) {
                ++m_smooth_trace.before_phase_b_not_offset;
                return false;
            }
            if (enveloped) {
                // THE ONE DELIBERATE DIVERGENCE FROM 3D IN THIS PHASE, and it is forced by the
                // models. 3D throws here, on the grounds that a silently skipped offset vertex
                // would sit off the level set while the counters reported the surface placed --
                // and it can afford to, because no 3D model under test has such a vertex.
                //
                // 2D's own multi-tag fixtures do. topo_annots_groups is a boolean over four
                // overlapping tags, so the offset boundary genuinely coincides with region
                // boundaries, and the band reaches the domain wall on the dragon; throwing makes
                // both unrunnable. Skipping is the same answer this function already gives an
                // envelope-held BACKGROUND vertex, for the same reason: placing it needs the
                // envelope's pull and containment, which this scheme does not implement yet, and
                // a half-constrained placement would be worse than leaving it where Phase A put
                // it.
                //
                // IT IS NOT SILENT, which is the property the throw was protecting. A wall
                // vertex is booked PINNED by band_vertex_is_reachable() and reported as such; a
                // region-boundary one stays REACHABLE, so its gradient keeps gating and the run
                // cannot report convergence over it. The count below is logged every pass.
                ++m_smooth_trace.before_phase_b_enveloped_offset;
                return false;
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

bool TopoOffsetTriMesh::smooth_offset_vertex_backtracking(const Tuple& t)
{
    const size_t vid = t.vid(*this);
    const std::vector<size_t>& locs = get_one_ring_fids_for_vertex(t);
    if (locs.empty()) {
        return false;
    }

    // Same entry guard the shared smoother applies: a one-ring already inverted in floats has no
    // valid segment to search, and the exact predicate below would reject every candidate.
    for (const size_t fid : locs) {
        if (is_inverted_f(fid)) {
            ++m_smooth_rejects.already_inverted;
            ++m_phase_b_constrained; // cannot even attempt its minimum
            return false;
        }
    }

    const Vector2d x_orig = m_vertex_attribute[vid].m_posf;

    // MINIMISE E = (Phi - c)^2 ALONG grad Phi. NOT a 2-D minimization of E, and NOT a bare
    // Newton root find on Phi = c. See the 3D twin, which this is a transcription of:
    //
    //  - the Gauss-Newton Hessian of E is 2 g g^T with g = grad Phi, RANK ONE, so a 2-D solve
    //    fixes the normal direction and lets the tangential one drift;
    //  - the undamped Gauss-Newton step -(r/|g|^2) g is exact where a root exists and singular
    //    where one does not -- at a local minimum of E with r != 0, stationarity forces g -> 0,
    //    so the step divides by a quantity going to zero exactly as it approaches the answer.
    //    That is every pinch: two objects closer than 2 x target_distance have no level set
    //    between them and the boundary belongs on the ridge where the contributions cancel.
    //
    // Levenberg-Marquardt: solve (2 g g^T + lambda I) d = -2 r g, which along g is
    //     d = -( r / (|g|^2 + mu) ) g,     mu = lambda / 2
    // so |g| -> 0 sends the step to -(r/mu) g -> 0 instead of to infinity. mu carries units of
    // |grad Phi|^2, so it is held relative to the potential's own level-set slope. Shrink it on
    // a step that lowers E, grow it on one that does not.
    constexpr int kRootFindIters = 50;
    constexpr int kLmTries = 8; ///< damping increases per iteration before giving up
    constexpr double kLmInit = 1e-8; ///< mu/s^2 at entry: effectively undamped Gauss-Newton
    constexpr double kLmMin = 1e-12;
    constexpr double kLmMax = 1e8;
    const double vertex_tol_rel = m_offset_params.ab_vertex_grad_tol_rel;
    const double s_ref = m_offset_potential->level_set_slope();
    const double mu_scale = (s_ref > 0. && std::isfinite(s_ref)) ? s_ref * s_ref : 1.;
    double mu_rel = kLmInit;
    double e_grad_entry = -1.; // |grad E| at the visit's start; captured on the first iteration
    Vector2d x = x_orig;
    for (int it = 0; it < kRootFindIters; ++it) {
        const double r = m_offset_potential->value(x) - m_offset_potential->target_level();
        const Vector2d g = m_offset_potential->gradient(x);
        const double g2 = g.squaredNorm();
        if (!std::isfinite(r)) break;
        if (!(g2 > 0.)) {
            // grad E = 2 r g = 0 with g = 0: a STATIONARY POINT of E, which under the damped
            // step is a legitimate place to stop -- it is the pinch minimum.
            break;
        }
        const double e_grad = 2. * std::abs(r) * std::sqrt(g2);
        if (e_grad_entry < 0.) {
            e_grad_entry = e_grad;
            if (!(e_grad_entry > 0.)) break; // already at the minimum
        }
        if (e_grad <= vertex_tol_rel * e_grad_entry) {
            break;
        }
        // Damped step, with mu raised until E actually decreases. |r| is monotone in E, so the
        // acceptance test is on |r| directly.
        bool advanced = false;
        Vector2d step = Vector2d::Zero();
        for (int k = 0; k < kLmTries; ++k) {
            step = -(r / (g2 + mu_rel * mu_scale)) * g;
            if (!step.allFinite()) break;
            const Vector2d x_try = x + step;
            const double r_try =
                m_offset_potential->value(x_try) - m_offset_potential->target_level();
            if (std::isfinite(r_try) && std::abs(r_try) < std::abs(r)) {
                x = x_try;
                mu_rel = std::max(mu_rel * 0.1, kLmMin);
                advanced = true;
                break;
            }
            mu_rel = std::min(mu_rel * 10., kLmMax);
        }
        if (!advanced) {
            break; // no admissible damping lowers E from here
        }
        // A step this small cannot move the gradient test above; stop burning evaluations.
        if (step.norm() <= 1e-12 * std::max(m_offset_params.target_distance, 1e-16)) {
            break;
        }
    }
    const Vector2d x_new = x;
    if (!x_new.allFinite()) {
        set_vertex_position(vid, x_orig);
        ++m_smooth_rejects.inverted;
        ++m_phase_b_constrained; // no admissible motion toward its minimum
        return false;
    }

    // Place a candidate and report whether any incident face inverted. Exact, on the rational
    // position, exactly as the shared smoother's accept test is.
    const auto inverts = [&](const Vector2d& p) {
        set_vertex_position(vid, p);
        for (const size_t fid : locs) {
            if (is_inverted(fid)) return true;
        }
        return false;
    };

    if (inverts(x_new)) {
        // The minimum this visit solved for lies outside what the one-ring admits -- the count
        // the pass loop's backtrack-free exit watches.
        ++m_phase_b_constrained;
        // BISECT THE SEGMENT, keeping the invariant lo = valid, hi = invalid. s = 1 is known
        // invalid (just tested) and s = 0 is known valid (the entry guard), so this converges UP
        // to the constraint from below rather than capping at the midpoint. 30 halvings take s
        // below 1e-9 of the segment, so a refusal here means no admissible motion exists rather
        // than that the search gave up.
        constexpr int kBisectionSteps = 30;
        double lo = 0., hi = 1.;
        Vector2d best = x_orig;
        bool found = false;
        for (int j = 0; j < kBisectionSteps; ++j) {
            const double mid = 0.5 * (lo + hi);
            const Vector2d cand = x_orig + mid * (x_new - x_orig);
            if (inverts(cand)) {
                hi = mid;
            } else {
                lo = mid;
                best = cand;
                found = true;
            }
        }
        if (!found) {
            // Not even the smallest step is admissible: leave the vertex where it started.
            set_vertex_position(vid, x_orig);
            ++m_smooth_rejects.inverted;
            return false;
        }
        // A SAFETY MARGIN ON WHAT THE RING ALLOWS, and ONLY on this path. `lo` after 30 halvings
        // sits within ~1e-9 of the first inverting configuration -- valid by the orientation
        // predicate and numerically degenerate in every other sense, which is what feeds
        // near-zero-area faces to Phase A. A vertex whose minimum is reachable inside its ring
        // never enters this branch, so it still lands exactly on the level set; the margin is a
        // concession to the constraint and is owed only where the constraint actually bound.
        //
        // 0.8 is measured in 3D rather than tuned; see the 3D twin's sweep, and read it as
        // scatter rather than a trend. The valid set along the segment need not be a single
        // interval, so the retreated point is re-tested rather than assumed.
        constexpr double kBacktrackMargin = 0.8;
        const Vector2d retreated = x_orig + kBacktrackMargin * lo * (x_new - x_orig);
        // Either way set the position explicitly rather than relying on where inverts() left it.
        set_vertex_position(vid, inverts(retreated) ? best : retreated);
    }

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
        params["max_iterations"] = 50;
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
    // The envelope-held offset vertices are the ones this scheme cannot place; see
    // smooth_before(). Warned rather than merely counted, because their gradient still gates.
    if (s.before_phase_b_enveloped_offset.load() > 0) {
        logger().warn(
            "\t{} Phase B placements were SKIPPED: the vertex is on the offset boundary AND held "
            "by an envelope (a tag region boundary, or the domain wall). Placing one needs the "
            "envelope's pull and containment together with the offset term, which Phase B does "
            "not implement yet, so they are left where Phase A put them. Their gradient still "
            "counts toward convergence unless they are on the wall, where they are reported as "
            "pinned.",
            s.before_phase_b_enveloped_offset.load());
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
    return std::abs(m_input_complex_bvh.dist(VectorXd(p)) - m_offset_params.target_distance);
}

// returns max_dist_err, avg_dist_err over the WHOLE band, pinned vertices included -- see
// distance_deviation_split() for why the loop uses a different number than the report does.
double TopoOffsetTriMesh::band_vertex_residual(const size_t vid) const
{
    // How far this vertex is from the level set Phi = c, as a LENGTH. The offset's own error,
    // as opposed to band_vertex_distance_error()'s Euclidean diagnostic.
    return m_offset_potential->residual_length(m_vertex_attribute[vid].m_posf);
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
        const double r = m_offset_potential->residual_length(pa + t * (pb - pa));
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
        const double err = m_offset_potential->residual_length(p);
        s.max_reachable = std::max(s.max_reachable, err);
        s.max_at_vertex = std::max(s.max_at_vertex, err);
        sum_reachable += err;
        ++s.n_reachable;
        if (band_vertex_is_reachable(vid)) {
            // THE RUNAWAY GUARD's measurement, taken here rather than in its own traversal:
            // this loop already visits exactly the vertices it cares about, and Phi is the
            // expensive part. report_outside_support() turns this into the error.
            if (!m_offset_potential->within_support(p)) {
                ++s.n_outside_support;
                const double d = m_input_complex_bvh.dist(VectorXd(p));
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
    // THE CONVERGENCE CRITERION: |grad (Phi(x) - c)^2| over the offset boundary -- at every band
    // vertex, and at interior samples of every band edge.
    //
    // The gradient of the SAME objective Phase B's sweeps minimize for these vertices -- the
    // offset term, and nothing else. Any drift between the two would measure a different fixed
    // point than the one the sweeps converge to.
    //
    // WEIGHT 1, deliberately: this is an ABSOLUTE bound in length units, so a tuning weight
    // would scale the bar.
    const std::vector<bool> on_band = band_vertex_mask();
    OffsetEnergy2D offset_energy(m_offset_potential, 1.0);

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
        offset_energy.gradient(x, g);
        const double gn = g.norm();

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

    // ... AND ALONG THE BAND'S EDGES, on the same lattice the residual is sampled on.
    //
    // A VERTEX CRITERION IS NOT A BOUNDARY CRITERION. Every vertex can sit exactly on the level
    // set while the chords between them cut across it, and a vertex-only gradient reads that as
    // converged. E = (Phi(x) - c)^2 is a field, so evaluating it inside an edge is the same
    // computation as at an endpoint, not an approximation of one.
    //
    // The remedy differs, which is why the split is kept and reported. An at-vertex max is the
    // boundary in the wrong PLACE and wants smoothing; an in-edge max is a boundary too COARSE
    // to be in the right place and wants refinement, which is Phase A's job. Both are errors in
    // the returned offset, so both gate.
    if (include_edge_samples) {
        double sum_edges = 0.;
        for (const Tuple& e : get_edges()) {
            if (!edge_is_offset_surface_live(e)) continue;
            // An edge with a pinned endpoint is pinned: no placement of the vertices the
            // optimizer owns can carry its interior to the level set.
            bool pinned = false;
            for (const size_t vid : {e.vid(*this), e.switch_vertex(*this).vid(*this)}) {
                if (!band_vertex_is_reachable(vid)) {
                    pinned = true;
                    break;
                }
            }
            for_each_offset_edge_sample(e, [&](const Vector2d& q) {
                Eigen::VectorXd g(2);
                offset_energy.gradient(Eigen::VectorXd(q), g);
                const double gn = g.norm();
                if (pinned) {
                    s.max_pinned = std::max(s.max_pinned, gn);
                    return;
                }
                s.max_reachable = std::max(s.max_reachable, gn);
                s.max_in_edge = std::max(s.max_in_edge, gn);
                sum_edges += gn;
                ++s.n_edge_samples;
            });
        }
        sum_reachable += sum_edges;
        s.n_reachable += s.n_edge_samples;
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
    // VERTICES ONLY, and that is not a drift from the convergence test: it is the same function
    // restricted to the variables this phase owns. Phase B moves vertices and performs no
    // topological operation, so the in-edge term is CONSTANT under everything it can do, and
    // folding it in would leave the stop test pinned at a value the sweeps cannot lower. The
    // in-edge term is Phase A's to fix, through refinement.
    return gradient_split(/*include_edge_samples=*/false).max_at_vertex;
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
    // ONE SIZING FIELD, TWO REASONS TO REFINE IT.
    //
    // In PHASE A this is TriWildMesh::refine_sizing_around_worst verbatim -- ranked by
    // m_face_attribute[].m_quality, filtered against stop_energy, seeding the same force-split
    // edges. Phase A is TriWild, and that includes how it responds to a stall; the field it
    // writes is the same field Phase B reads and writes, so the refinement each phase asks for
    // accumulates rather than competing.
    //
    // NOTE THE UNITS: max_metric arrives from mesh_improvement() as whatever
    // optimization_quality_stats() returned, which in Phase A is ABSOLUTE AMIPS, and the score
    // it is compared against is absolute too. The Phase B branch below is entirely in normalized
    // units. Mixing the two is the bug that cost 3D twenty iterations.
    const int n_rings = std::max(0, m_params.stuck_refine_rings);

    if (m_phase == OptPhase::A) {
        // Clamped above exactly as TriWild does: without it a single degenerate face (quality
        // MAX_ENERGY) sets filter_energy astronomically high and select_worst_cells then picks
        // out only the degenerate faces, so refinement stops fixing the merely-bad ones.
        const double filter_energy =
            std::min(std::max(max_metric / 100., m_params.stop_energy), 100.);

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

    // PHASE B: the same shape, with face_criterion_rel() in place of the raw AMIPS energy and
    // 1.0 -- the normalized target -- in place of stop_energy. At TriWild's own default
    // stop_energy of 100 the two filter expressions are the same number.
    const double filter = std::min(std::max(max_metric / 100., 1.0), 100.);

    const auto worst = wmtk::utils::select_worst_cells(
        tri_capacity(),
        [this](size_t fid) { return tuple_from_tri(fid).is_valid(*this); },
        [this](size_t fid) { return face_criterion_rel(fid); },
        filter,
        m_params.stuck_refine_num_worst);

    if (worst.empty()) {
        return 0;
    }

    // Force-split: the longest edge of each selected face, split once next pass regardless of
    // the length gate, WITHOUT touching the sizing field. This is what unsticks a face whose
    // edges are already shorter than their target.
    m_force_split_edges.clear();
    if (m_params.stuck_refine_force_split) {
        for (const auto& [q, fid] : worst) {
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
    for (const auto& [_, fid] : worst) {
        for (const size_t v : oriented_tri_vids(fid)) {
            seeds.push_back(v);
        }
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
        "[stuck-refine] worst {} faces (worst {:.4}x target), refined {} of {} region vertices, "
        "filter {:.4}",
        worst.size(),
        worst.back().first,
        refined.size(),
        region.size(),
        filter);
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
    const Vector3d near3 = m_input_complex_bvh.nearest_point(VectorXd(p));
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
        m_offset_potential->value(p),
        m_offset_potential->target_level(),
        m_offset_potential->residual_length(p),
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
        if (m_input_complex_bvh.dist(VectorXd(m_vertex_attribute[vid].m_posf)) >
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
        const double d = m_input_complex_bvh.dist(VectorXd(m_vertex_attribute[vid].m_posf));
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
    std::vector<Eigen::Vector2i> segs;
    for (const Tuple& e : get_edges()) {
        if (!edge_is_offset_surface_live(e)) continue;
        segs.emplace_back(int(e.vid(*this)), int(e.switch_vertex(*this).vid(*this)));
    }
    if (segs.empty()) {
        // Nothing to hold. Not an error: a run whose offset region never formed has other
        // problems, and they are reported where they happen.
        m_offset_envelope = nullptr;
        logger().warn("\t[phase A] no offset-boundary segments; the offset envelope is empty");
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
    // A LENGTH, and this is where 2D could most easily go wrong: offset_residual_tolerance() is
    // (offset_gradient_rel / 2) x target_distance, a distance in model units, and SampleEnvelope's eps
    // is a distance in the same units -- the same pairing 3D uses. It is NOT the normalized
    // criterion, and it is not envelope_size_rel (a fraction of the bounding-box diagonal, which
    // is what m_envelope is built from).
    const double eps =
        std::max(m_offset_params.ab_offset_envelope_rel * offset_residual_tolerance(), 1e-12);

    m_offset_envelope = std::make_shared<SampleEnvelope>();
    m_offset_envelope->init(verts, segs, eps);
    logger().info(
        "\t[phase A] offset envelope rebuilt: {} segments, {} (eps {:.6g} = {:.4}x the Phi "
        "tolerance {:.6g})",
        segs.size(),
        m_offset_envelope->use_exact ? "EXACT" : "sampled",
        eps,
        m_offset_params.ab_offset_envelope_rel,
        offset_residual_tolerance());
}

size_t TopoOffsetTriMesh::phase_b_smooth()
{
    // SMOOTHING ONLY, TO A FIXED POINT. No topology: Phase B's single job is to move the offset
    // boundary onto the level set, and the mesh it does that on is whatever Phase A left.
    // Running to convergence rather than for a fixed count is what makes the sizing update
    // afterwards meaningful -- an edge still over tolerance once nothing moves is one smoothing
    // genuinely cannot place, which is a resolution problem and therefore the sizing field's
    // business.
    std::vector<Vector2d> before(vert_capacity());

    // THE CRITERION IS THE GRADIENT, relative to its value at phase entry. Zero exactly at the
    // Gauss-Seidel fixed point, so unlike the displacement test it cannot read converged when
    // moves are blocked (a refused move has zero displacement and full gradient), and it keeps
    // going while sweeps still lower the energy. Entry-relative makes it scale-free across
    // rounds whose Phase A left very different amounts of work.
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

    // Negative ab_smooth_max_passes means UNCAPPED: run until the gradient criterion fires. The
    // only other exit is the no-progress guard below -- a gradient that has stopped falling is a
    // blocked configuration, and looping on it forever helps nobody.
    const size_t cap = m_offset_params.ab_smooth_max_passes < 0
                           ? std::numeric_limits<size_t>::max()
                           : size_t(std::max(1, m_offset_params.ab_smooth_max_passes));
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
        m_phase_b_sub = PhaseBSub::Offset;
        smooth_all_vertices(1);
        // Captured between the sweeps: only the offset sub-sweep can constrain a placement, and
        // smooth_all_vertices() resets the shared reject counters, so a before/after delta on
        // `accepted` would wrap.
        const int constrained = m_phase_b_constrained.load();
        const size_t placed = m_smooth_rejects.accepted.load();

        m_phase_b_sub = PhaseBSub::Background;
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
            "\t[phase B] pass {}: placed {} constrained {} | background relaxed {}",
            pass + 1,
            placed,
            constrained,
            relaxed);
        // ONE FRAME PER SMOOTHING PASS, same debug_{N} series and same counter as the Phase A
        // passes in local_operations(), so a viewer globbing *debug_*.vtu gets both phases
        // interleaved in the order they actually ran. Opt-in through DEBUG_output_per_pass --
        // see write_smoothing_debug_output(), which drops debug_ frames unless it is set.
        if (m_params.debug_output) {
            write_smoothing_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
        }
        // THE NATURAL EXIT. Every offset vertex reached its unconstrained minimum inside its own
        // one-ring, so nothing had to be backtracked -- the fixed point this scheme is defined
        // to seek. Checked before the gradient bar because it is the stronger statement.
        if (constrained == 0) {
            logger().info(
                "\t[phase B] no placement was constrained by its one-ring this pass; the offset "
                "boundary is at its achievable fixed point");
            ++pass;
            break;
        }
        // The run's own convergence bar: once the band is under it there is nothing another
        // round could do with a better-placed boundary.
        if (g <= g_abs) {
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
            // THE PLATEAU IS THE ACHIEVABLE FIXED POINT: the fixed point sitting a hair above
            // the bar, not a configuration fighting the inversion guard. Nothing more is coming
            // from smoothing, so stop and let the sizing update answer the residual.
            logger().warn(
                "\t[phase B] neither the placement gradient nor the constrained count has "
                "improved for {} passes (gradient {:.6g}, {:.4g}x entry, {} still constrained); "
                "treating the plateau as the phase's fixed point and stopping",
                no_progress,
                g,
                g_entry > 0. ? g / g_entry : 0.,
                constrained);
            ++pass;
            break;
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
    OffsetEnergy2D energy(m_offset_potential, 1.0);
    const auto grad_at = [&](const Vector2d& p) {
        Eigen::VectorXd g(2);
        energy.gradient(Eigen::VectorXd(p), g);
        return g.norm();
    };

    // 1. Every band vertex against the criterion. Non-band vertices are left `true` so they
    //    never veto a neighbour's halving -- the rule is about the BOUNDARY one-ring.
    std::vector<char> in_tol(vert_capacity(), 1), is_band(vert_capacity(), 0);
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid] || !m_vertex_extra[vid].m_is_on_offset) continue;
        if (!m_vertex_attribute[vid].m_is_rounded) continue; // not placeable, not this rule's
        is_band[vid] = 1;
        in_tol[vid] = grad_at(m_vertex_attribute[vid].m_posf) <= gtol ? 1 : 0;
    }

    // 2. One sweep of the band's edges gives both the boundary one-ring and, per vertex,
    //    whether any incident edge carries an out-of-tolerance interior sample.
    std::vector<char> ring_in_tol(vert_capacity(), 1), has_bad_edge(vert_capacity(), 0);
    for (const Tuple& e : get_edges()) {
        if (!edge_is_offset_surface_live(e)) continue;
        const std::array<size_t, 2> vs = {{e.vid(*this), e.switch_vertex(*this).vid(*this)}};
        bool edge_bad = false;
        for_each_offset_edge_sample(e, [&](const Vector2d& q) {
            if (!edge_bad && grad_at(q) > gtol) edge_bad = true;
        });
        for (int i = 0; i < 2; ++i) {
            if (edge_bad) has_bad_edge[vs[i]] = 1;
            const int j = 1 - i;
            if (!in_tol[vs[j]]) ring_in_tol[vs[i]] = 0;
        }
    }

    // 3. Apply. The floor is the BAND's, not stuck_refine_min_scalar.
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / m_params.l);
    std::vector<size_t> changed;
    size_t n_halved = 0, n_misplaced = 0, n_ring_blocked = 0, n_at_floor = 0;
    for (size_t vid = 0; vid < vert_capacity(); ++vid) {
        if (!is_band[vid]) continue;
        // MISPLACED, not under-resolved: leave it to Phase B. Counted so the two reasons a
        // vertex goes unrefined stay distinguishable in the log -- they are opposite diagnoses.
        if (!in_tol[vid]) {
            ++n_misplaced;
            continue;
        }
        if (!has_bad_edge[vid]) continue; // nothing wrong here at all
        if (!ring_in_tol[vid]) {
            ++n_ring_blocked;
            continue;
        }
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
        "\t[phase B] band sizing: {} of {} halved | not refined: {} misplaced (vertex out of "
        "tolerance), {} ring-blocked (a neighbour is), {} at the floor, {} nothing wrong | "
        "floor {:.6g}",
        n_halved,
        n_band,
        n_misplaced,
        n_ring_blocked,
        n_at_floor,
        n_band - n_halved - n_misplaced - n_ring_blocked - n_at_floor,
        s_floor);
    return changed.size();
}

void TopoOffsetTriMesh::optimize_offset_alternating()
{
    const int rounds = std::max(1, m_offset_params.ab_max_rounds);
    const int a_iters = std::max(1, m_offset_params.ab_phase_a_iterations);

    // Before anything runs, so a construction defect is reported as one rather than surfacing
    // later as a residual that will not converge.
    check_no_vertex_on_both_surfaces("construction");

    // Where the mesh as constructed stands, before the loop touches it. A diagnostic: it is the
    // baseline every later round's residual is read against.
    const double construction_residual = residual_split().max_reachable;
    logger().info(
        "\tconstruction residual: {:.6g} ({:.4}x tolerance)",
        construction_residual,
        construction_residual / offset_residual_tolerance());
    const GradientSplit construction_grad = gradient_split();
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
        // ---- PHASE A: TriWild, with the offset held inside its envelope ----
        logger().info("======== A/B round {} / {}: phase A ========", round + 1, rounds);
        m_phase = OptPhase::A;
        // DIAGNOSTIC; see ab_no_collapse_after_first_round. Round 1 keeps its collapses because
        // the mesh as constructed genuinely needs them.
        m_ab_collapses_disabled = m_offset_params.ab_no_collapse_after_first_round && round > 0;
        if (m_ab_collapses_disabled) {
            logger().warn("\t[phase A] COLLAPSES DISABLED (ab_no_collapse_after_first_round)");
        }
        rebuild_offset_envelope();
        mesh_improvement(a_iters);

        // ASK THE LOOP, in the loop's own units. optimization_quality_stats() reports ABSOLUTE
        // AMIPS against optimization_stop_metric() = stop_energy in Phase A, so a check written
        // in normalized units fails a Phase A that converged.
        const double amips = std::get<0>(optimization_quality_stats());
        const double bar = optimization_stop_metric();
        if (m_offset_params.debug_output) {
            // NAMED BY PHASE, not by the debug counter: these two driver writes are the
            // per-phase timeline, and the viewer globs `*phase_*.vtu` to show exactly that.
            write_smoothing_debug_output(fmt::format("phase_{}A", round + 1));
        }

        // PHASE A HAS TO CONVERGE. It is TriWild on a mesh TriWild can improve, with the offset
        // pinned to a tolerance-wide tube; if element quality is still above stop_energy when
        // the loop gives up, something is wrong that iterating further will not fix, and
        // continuing into Phase B would optimize the offset on a mesh that cannot carry it.
        if (amips > bar) {
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
        // Released, so smoothing can actually move the boundary. Phase A rebuilds it next round
        // around wherever this leaves it.
        m_offset_envelope = nullptr;

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
        const double phi = g.max_reachable / offset_gradient_tolerance();
        logger().info(
            "\t[phase B] {} smoothing passes, max placement gradient {:.6g} = {:.4}x tolerance "
            "(at-vertex {:.4}x, in-edge {:.4}x) | phi residual {:.4}x its own bar | {} reachable, "
            "{} pinned (max {:.6g}), {} skipped ({} unrounded, {} inverted ring)",
            passes,
            g.max_reachable,
            phi,
            g.max_at_vertex / offset_gradient_tolerance(),
            g.max_in_edge / offset_gradient_tolerance(),
            r.max_reachable / offset_residual_tolerance(),
            g.n_reachable,
            g.n_pinned,
            g.max_pinned,
            g.n_skipped_unrounded + g.n_skipped_inverted,
            g.n_skipped_unrounded,
            g.n_skipped_inverted);

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
            // See the phase A twin above: the per-phase series the viewer globs.
            write_smoothing_debug_output(fmt::format("phase_{}B", round + 1));
        }

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

    // The offset boundary as CONSTRUCTED must already be inside the potential's support, or
    // nothing the optimization does can move it. Checked before any operation runs so that a
    // construction defect is reported as one.
    check_offset_within_support("Offset as constructed");

    // Spelled out including the slope^2 factor, so the line reproduces its own number. Without
    // it the criterion reads as rel x target_distance, which is only what it is on a distance
    // field.
    logger().info(
        "\tOffset criterion: |grad (Phi - c)^2| <= {} = offset_gradient_rel {} x target_distance "
        "{} x level-set slope^2 {:.6} -- i.e. residual <= {} ({} x target_distance) | diagnostic "
        "Phi residual bar {} (= half the gradient bound, in model units) | level c {:.6}, dhat "
        "{:.6}, {} interior samples per band edge",
        offset_gradient_tolerance(),
        m_offset_params.offset_gradient_rel,
        m_offset_params.target_distance,
        m_offset_potential->level_set_slope() * m_offset_potential->level_set_slope(),
        0.5 * m_offset_params.offset_gradient_rel * m_offset_params.target_distance,
        0.5 * m_offset_params.offset_gradient_rel,
        offset_residual_tolerance(),
        m_offset_potential->target_level(),
        m_offset_potential->dhat(),
        m_offset_params.offset_residual_samples);

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
    // THREE MEASURES, ONE CRITERION. Convergence is max_grad and nothing else -- the placement
    // gradient over the offset boundary, at band vertices AND at interior samples of band edges,
    // which is the same test for any potential. The Phi residual is the criterion's own quantity
    // in length units, and the Euclidean error says how far the smoothed offset ended up from
    // the exact one -- both diagnostics, neither a criterion.
    const auto [max_dist, avg_dist] = compute_distance_deviation();
    const DistanceSplit r = residual_split();
    const GradientSplit g = gradient_split();
    const double tol = offset_residual_tolerance();
    const double gtol = offset_gradient_tolerance();
    logger().info(
        "placement gradient: max {} (avg {}) vs tolerance {} = {} x target_distance | at "
        "vertices {}, inside edges {} ({} edge samples) | {} reachable, {} pinned (max {}), {} "
        "skipped ({} unrounded, {} inverted ring)",
        g.max_reachable,
        g.avg_reachable,
        gtol,
        m_offset_params.offset_gradient_rel,
        g.max_at_vertex,
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

    m_converged = g.max_reachable <= gtol;
    if (m_converged) {
        logger().info(
            "Converged ([max placement gradient] {} <= {} [offset_gradient_rel x "
            "target_distance]); worst term {} (at-vertex {}, in-edge {})",
            g.max_reachable,
            gtol,
            g.max_in_edge > g.max_at_vertex ? "in-edge" : "at-vertex",
            g.max_at_vertex,
            g.max_in_edge);
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

    if (!m_converged) {
        // WHICH TERM FAILED IS THE FIRST THING TO ACT ON, and naming a vertex when the max came
        // from an edge interior points at the wrong remedy: an in-edge max is a boundary too
        // coarse to represent the level set, which wants refinement, and there is no vertex to
        // blame for it. at-vertex is the boundary in the wrong place, which wants smoothing --
        // and only that case has a worst_vid.
        if (g.max_in_edge > g.max_at_vertex) {
            logger().warn(
                "Optimization did not converge ([max placement gradient] {} > {} "
                "[offset_gradient_rel x target_distance]); worst term is IN-EDGE ({} vs {} at "
                "vertices) -- the band is too coarse to represent the level set, which wants "
                "refinement rather than smoothing",
                g.max_reachable,
                gtol,
                g.max_in_edge,
                g.max_at_vertex);
        } else {
            logger().warn(
                "Optimization did not converge ([max placement gradient] {} > {} "
                "[offset_gradient_rel x target_distance]); worst term is AT-VERTEX, at vertex {} "
                "({} vs {} inside edges)",
                g.max_reachable,
                gtol,
                g.worst_vid,
                g.max_at_vertex,
                g.max_in_edge);
        }

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
