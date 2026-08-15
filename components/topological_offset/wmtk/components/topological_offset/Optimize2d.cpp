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
void atomic_max(std::atomic<int>& target, int value)
{
    int cur = target.load();
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
    // Face quality, which the shared operations read and keep up to date from here on.
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        m_face_attribute[fid].m_quality = get_quality(fid);
    }

    size_t n_both = 0, n_label_only = 0, n_tag_only = 0;
    for (const Tuple& e : get_edges()) {
        const size_t eid = e.eid(*this);
        m_edge_attribute[eid].reset();

        const std::optional<Tuple> opp = e.switch_face(*this);
        if (!opp) {
            // Only one incident face: this is the domain boundary. Tagging it bbox is what
            // keeps the box from collapsing inward; there is no side to compare against, so it
            // can never be an offset boundary.
            m_edge_attribute[eid].m_is_bbox_fs = 0;
            continue;
        }

        // The input complex's BOUNDARY is what carries input geometry, not its interior. The
        // edge label marks every edge inside the input region, and tracking all of them froze
        // the whole interior -- on a solid input that is most of the mesh, and no operation
        // could be accepted anywhere. An interior edge of the region carries no geometry, so
        // only the edges where input meets non-input are tracked, and they take precedence
        // over the offset class.
        const size_t fa = e.fid(*this), fb = opp->fid(*this);
        if (face_is_input_complex(fa) != face_is_input_complex(fb)) {
            m_edge_attribute[eid].m_is_surface_fs = true;
            m_edge_attribute[eid].m_surface_class = INPUT_SURFACE_CLASS;
            continue;
        }

        // The offset boundary IS the face-LABEL change across an edge. There is nothing else
        // that defines it -- it is not stored anywhere, it falls out of the labelling.
        //
        // A region-TAG change is tracked too, and has to be, but under its own class. The shared
        // 2D swap copies one incident face's tags onto both faces it creates, so an unguarded
        // flip across a tag boundary silently relabels a face and moves the region -- which is
        // what the output is built from. Marking the edge as tracked surface is what makes the
        // swap refuse it, and that is the whole reason these edges are tracked. They are NOT the
        // offset: another body's outline or an overlap seam can sit anywhere in the domain, and
        // treating it as offset boundary hands it to project_offset_vertex() and to the sizing
        // field, both of which then work to place it at target_distance from the input complex.
        const bool label_change = m_face_extra[fa].label != m_face_extra[fb].label;
        const bool tag_change = m_face_attribute[fa].tags != m_face_attribute[fb].tags;
        // Diagnostic: an edge that is BOTH is one where the offset boundary coincides with a
        // region boundary. The classification below has to pick one, and picking OFFSET hides
        // the region aspect from the envelope. Counting them says whether that situation is
        // built by construction or created by the optimization's own operations.
        n_both += (label_change && tag_change);
        n_label_only += (label_change && !tag_change);
        n_tag_only += (!label_change && tag_change);
        if (label_change) {
            m_edge_attribute[eid].m_is_surface_fs = true;
            m_edge_attribute[eid].m_surface_class = OFFSET_SURFACE_CLASS;
        } else if (tag_change) {
            m_edge_attribute[eid].m_is_surface_fs = true;
            m_edge_attribute[eid].m_surface_class = REGION_SURFACE_CLASS;
        }
    }
    logger().info(
        "\tedge classification: label+tag BOTH {}, label only {}, tag only {}",
        n_both,
        n_label_only,
        n_tag_only);

    // Vertices inherit from their incident edges. m_is_on_surface is the union, which is what
    // the shared operations protect; the two extra flags say which surface.
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        m_vertex_attribute[vid].m_is_on_surface = false;
        m_vertex_attribute[vid].on_bbox_faces.clear();
        m_vertex_extra[vid].m_is_on_input = false;
        m_vertex_extra[vid].m_is_on_offset = false;
        m_vertex_extra[vid].m_is_on_region = false;

        for (const Tuple& e : get_one_ring_edges_for_vertex(v)) {
            const size_t eid = e.eid(*this);
            if (m_edge_attribute[eid].m_is_bbox_fs >= 0 &&
                m_vertex_attribute[vid].on_bbox_faces.empty()) {
                m_vertex_attribute[vid].on_bbox_faces.push_back(0);
            }
            if (!m_edge_attribute[eid].m_is_surface_fs) continue;
            m_vertex_attribute[vid].m_is_on_surface = true;
            switch (m_edge_attribute[eid].m_surface_class) {
            case OFFSET_SURFACE_CLASS: m_vertex_extra[vid].m_is_on_offset = true; break;
            case REGION_SURFACE_CLASS: m_vertex_extra[vid].m_is_on_region = true; break;
            default: m_vertex_extra[vid].m_is_on_input = true; break;
            }
        }
    }

    size_t n_off = 0, n_in = 0, n_box = 0;
    size_t n_reg = 0;
    for (const Tuple& e : get_edges()) {
        const size_t eid = e.eid(*this);
        n_off += edge_is_offset(eid);
        n_in += edge_is_input(eid);
        n_reg += edge_is_region_boundary(eid);
        n_box += (m_edge_attribute[eid].m_is_bbox_fs >= 0);
    }
    logger().info(
        "\ttracked edges: {} offset boundary, {} input complex, {} other region boundary, {} bbox",
        n_off,
        n_in,
        n_reg,
        n_box);
}

bool TopoOffsetTriMesh::swap_edge_before(const Tuple& t)
{
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

    // Paper Sec. 5.3.3, Step 2: "The swap is not performed if the normal deviation before the
    // operation is below the user-defined maximum and would be above afterward." Unlike the
    // collapse rule, the paper states the swap rule in this asymmetric form itself, so it is
    // taken literally -- and the ratchet that made the asymmetry wrong for collapse cannot
    // happen here, because a swap removes no vertex and so cannot decimate anything.
    //
    // The quantity is captured over all four vertices of the two incident triangles, which is
    // every vertex whose incident edge set the flip changes.
    const size_t a = t.vid(*this);
    const size_t b = t.switch_vertex(*this).vid(*this);
    double nd = 0.;
    for (const size_t vid : {a, b, c, d}) {
        nd = std::max(nd, max_offset_normal_deviation_at_vertex(vid));
    }
    m_swap_nd_before.local() = nd;
    m_swap_verts.local() = {{a, b, c, d}};
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

void TopoOffsetTriMesh::init_region_boundary_envelope_from_input()
{
    // Pre-offset there is no band and no offset boundary, so every tag change IS a genuine
    // region boundary -- the exclusion that edge_is_region_boundary() exists to perform has
    // nothing to exclude yet.
    //
    // THE INPUT COMPLEX'S BOUNDARY IS INCLUDED, and this is the point of the whole envelope now.
    // It used to be skipped as "frozen": the complex could not move, so nothing could contain
    // it. It moves now, and this tube -- captured here, from the mesh as loaded, before
    // execute_offset() touches anything -- is what bounds how far. That makes it the exact
    // counterpart of TriWild's input envelope, at the same relative radius (envelope_size_rel
    // 1e-3 against TriWild's epsr 1e-3).
    std::vector<Eigen::Vector2i> segs;
    for (const Tuple& e : get_edges()) {
        const std::optional<Tuple> opp = e.switch_face(*this);
        if (!opp) continue; // domain boundary: frozen, never a region boundary
        const size_t fa = e.fid(*this), fb = opp->fid(*this);
        const bool input_boundary = face_is_input_complex(fa) != face_is_input_complex(fb);
        if (!input_boundary && m_face_attribute[fa].tags == m_face_attribute[fb].tags) continue;
        segs.emplace_back(e.vid(*this), e.switch_vertex(*this).vid(*this));
    }

    std::vector<Eigen::Vector2d> verts(vert_capacity());
    for (size_t i = 0; i < vert_capacity(); ++i) {
        verts[i] = m_vertex_attribute[i].m_posf;
    }

    m_envelope_eps = m_offset_params.envelope_size;
    m_envelope = std::make_shared<SampleEnvelope>();
    m_envelope->init(verts, segs, m_envelope_eps);
    logger().info(
        "\tRegion-boundary envelope (from input, pre-offset): {} segments, {} (eps {:.6})",
        segs.size(),
        m_envelope->use_exact ? "EXACT" : "sampled",
        m_envelope_eps);
}

void TopoOffsetTriMesh::init_region_boundary_envelope()
{
    // Already captured from the input mesh, before the band truncated the region curves.
    // Rebuilding now would re-anchor the tube to the truncated ones, which is the whole
    // thing init_region_boundary_envelope_from_input() exists to avoid.
    if (m_envelope) {
        logger().info("\tRegion-boundary envelope: keeping the one built from the input mesh.");
        return;
    }

    // REGION_SURFACE_CLASS edges only, and this is the whole point of the class existing.
    //
    // Building from raw tag differences instead sweeps in the offset boundary, because the band
    // carries the offset output tag, so its boundary IS a tag change. That anchors a tube around
    // wherever conservative growth happened to leave the offset and then requires the offset to
    // stay inside it -- an envelope around the one surface the optimization exists to move. On
    // the dragon rectangle that was 429 of the 1392 segments, and it capped every offset vertex
    // at envelope_size of travel from its initial position, which is what pinned max_dist_err.
    //
    // The INPUT COMPLEX is included, for the reason given in the _from_input variant: it is free
    // to move now, so it needs containing, and this envelope is what contains it. The domain
    // boundary stays out -- it is refused categorically, so it can never fail a containment
    // test.
    //
    // This path only runs with region_envelope_from_input off. It is strictly worse for the
    // input complex than the _from_input variant, because by now conservative growth may have
    // re-triangulated the mesh around it; the parameter defaults to true for that reason.
    std::vector<Eigen::Vector2i> segs;
    for (const Tuple& e : get_edges()) {
        const size_t eid = e.eid(*this);
        if (!edge_is_region_boundary(eid) && !edge_is_input(eid)) continue;
        segs.emplace_back(e.vid(*this), e.switch_vertex(*this).vid(*this));
    }

    std::vector<Eigen::Vector2d> verts(vert_capacity());
    for (size_t i = 0; i < vert_capacity(); ++i) {
        verts[i] = m_vertex_attribute[i].m_posf;
    }

    m_envelope_eps = m_offset_params.envelope_size;
    m_envelope = std::make_shared<SampleEnvelope>();
    m_envelope->init(verts, segs, m_envelope_eps);
    logger().info(
        "\tRegion-boundary envelope: {} segments, {} (eps {:.6})",
        segs.size(),
        m_envelope->use_exact ? "EXACT" : "sampled",
        m_envelope_eps);
}

bool TopoOffsetTriMesh::region_boundary_is_outside_envelope(const size_t vid) const
{
    if (!m_envelope) return false;
    const Vector2d p = m_vertex_attribute[vid].m_posf;
    for (const Tuple& e : get_one_ring_edges_for_vertex(tuple_from_vertex(vid))) {
        // Region-class and INPUT-class edges, matching what the envelope is built from. An
        // offset-class edge incident to this vertex is not contained by anything -- the distance
        // projection, the inversion test and the topology guards are what govern the offset
        // surface.
        //
        // LIVE, not the cached edge_is_region_boundary(): smoothing runs after split/collapse/
        // swap within the same iteration, so a neighbour edge those passes just created or
        // re-pointed carries whatever cached class its slot defaulted to, not one
        // label_offset_boundary() has had a chance to set. edge_is_input() reads the same
        // attribute the shared operations propagate across a split or collapse, so it needs no
        // live counterpart.
        if (!edge_is_region_boundary_live(e) && !edge_is_input(e.eid(*this))) continue;
        const size_t nb = (e.vid(*this) == vid) ? e.switch_vertex(*this).vid(*this) : e.vid(*this);
        if (m_envelope->is_outside(std::array<Vector2d, 2>{{p, m_vertex_attribute[nb].m_posf}})) {
            return true;
        }
    }
    return false;
}

bool TopoOffsetTriMesh::swap_edge_after(const Tuple& t)
{
    if (!TriOptimizerMesh::swap_edge_after(t)) {
        return false;
    }

    // The other half of the paper's swap rule: refuse only a flip that takes an offset patch
    // from good to bad. The vertices are the four the flip touched, remembered in
    // swap_edge_before() -- reading them off `t` here would only find the new edge's two.
    if (m_swap_nd_before.local() < m_offset_params.max_normal_deviation_deg) {
        double nd_after = 0.;
        for (const size_t vid : m_swap_verts.local()) {
            nd_after = std::max(nd_after, max_offset_normal_deviation_at_vertex(vid));
        }
        if (nd_after >= m_offset_params.max_normal_deviation_deg) {
            ++iter_cnt_swap_nd_reject;
            return false;
        }
    }
    ++iter_cnt_swap;
    return true;
}

bool TopoOffsetTriMesh::collapse_edge_before(const Tuple& t)
{
    if (!TriOptimizerMesh::collapse_edge_before(t)) {
        return false;
    }
    // Unconditionally, not just when both endpoints are already on a tracked simplex -- see
    // the declaration. substructure_link_condition() evaluates the condition for the mesh and
    // for every substructure, which is what a topological offset needs preserved.
    if (!substructure_link_condition(t)) {
        return false;
    }

    // Paper Sec. 5.3.3, Step 2: "a collapse is only performed if the user-defined maximum
    // normal deviation is not exceeded". Record how bad the offset surface already was around
    // both endpoints, so collapse_edge_after() can tell a collapse that DEGRADES a good patch
    // from one that merely fails to fix an already-bad one -- the same asymmetry the 3D
    // NormalDeviationAfterInvariant analogue applies. Without this guard nothing stops the
    // offset polyline being decimated: the length and quality tests are happy to collapse a
    // well-resolved curve into a few long chords.
    const size_t v1_id = t.vid(*this);
    const size_t v2_id = t.switch_vertex(*this).vid(*this);
    m_collapse_nd_before.local() = std::max(
        max_offset_normal_deviation_at_vertex(v1_id),
        max_offset_normal_deviation_at_vertex(v2_id));
    return true;
}

bool TopoOffsetTriMesh::collapse_edge_after(const Tuple& t)
{
    if (!TriOptimizerMesh::collapse_edge_after(t)) {
        return false;
    }
    // The paper's rule is flat: "a collapse is only performed if the user-defined maximum normal
    // deviation is not exceeded." The 3D branch softens it to "only block a collapse that makes
    // a GOOD patch bad", so that a patch already over sigma_max -- typically one sitting on a
    // genuine feature of the input, where no refinement will ever bring the field normals into
    // agreement -- does not freeze the mesh around it.
    //
    // Taken literally, that softening is a ratchet: the instant a stretch crosses sigma_max it
    // stops being protected at all, and the next collapses can take it to 90 degrees unopposed.
    // That is measurably what happened here -- the offset polyline went 429 -> 80 edges with
    // only 8 collapses refused. So the bar is the WORSE of the two readings: never exceed
    // sigma_max, and never make an already-over-sigma_max patch worse than it already is. Good
    // patches get the paper's rule exactly; bad ones are held where they are rather than
    // abandoned, and a feature can still be collapsed across as long as it does not degrade.
    const double bar =
        std::max(m_offset_params.max_normal_deviation_deg, m_collapse_nd_before.local());
    if (max_offset_normal_deviation_at_vertex(t.vid(*this)) > bar) {
        ++iter_cnt_collapse_nd_reject;
        return false;
    }
    return true;
}

bool TopoOffsetTriMesh::collapse_before_vertex(const size_t v1_id, const size_t v2_id)
{
    // v1 is the vertex the collapse REMOVES (it merges into v2, which keeps its position). The
    // domain boundary may not lose a vertex: the box is not something to be coarsened.
    //
    // An INPUT-COMPLEX vertex may now be removed, provided it merges onto another input-complex
    // vertex (the per-class rule below) and the result stays inside m_envelope (the shared
    // collapse's own containment check) and preserves the substructure topology
    // (substructure_link_condition, applied unconditionally in collapse_edge_before). That is
    // TriWild's rule for its input surface, and it is what lets the complex be coarsened where
    // it is over-resolved instead of pinning every face incident to it.
    if (vertex_is_on_domain_boundary(v1_id)) {
        return false;
    }

    // THE OFFSET BOUNDARY IS ALWAYS LENGTH-LIMITED, whatever the pass says.
    //
    // Every other tracked surface here is held by m_envelope, which bounds how far it can be
    // decimated no matter what the length gate is doing. The offset boundary deliberately has no
    // envelope -- it is the surface the optimization exists to MOVE -- so its sizing field is the
    // only thing bounding its resolution, and a pass that switches the length gate off removes
    // that too. mesh_improvement() runs exactly such a pass twice, `it pre` and `it post`, which
    // is right for TriWild and wrong here.
    //
    // Measured on topological_offset_2d_vertex_input: `it pre` alone took the mesh from 2619 to
    // 462 vertices and the band's max distance error from 0.125 to 0.25 -- exactly
    // target_distance, i.e. a band vertex had been collapsed onto the input complex. From there
    // the distance criterion can never be met, so the loop never stopped, the stall detector
    // fired every iteration, and the sizing ratcheted the mesh to 13.5k vertices with the max
    // AMIPS climbing through 1e6.
    //
    // This is the same rule init_offset_sizing_field() exists to serve, stated for the one pass
    // that would otherwise bypass it. The interior mesh still gets TriWild's unlimited warm-up.
    if (!m_collapse_limit_length && m_vertex_extra[v1_id].m_is_on_offset) {
        return false;
    }

    // The base only knows that both endpoints are on SOME tracked surface. A vertex may not
    // leave the particular surface it belongs to: an input-complex vertex carries input
    // geometry, an offset-boundary vertex carries the offset, and a region-boundary vertex
    // carries the outline of a tag region the output is built from. Each is checked separately
    // because a vertex can be on more than one, and satisfying the union is not enough.
    if (m_vertex_extra[v1_id].m_is_on_input && !m_vertex_extra[v2_id].m_is_on_input) {
        return false;
    }
    if (m_vertex_extra[v1_id].m_is_on_offset && !m_vertex_extra[v2_id].m_is_on_offset) {
        return false;
    }
    if (m_vertex_extra[v1_id].m_is_on_region && !m_vertex_extra[v2_id].m_is_on_region) {
        return false;
    }
    return true;
}

void TopoOffsetTriMesh::collapse_after_vertex(const size_t v1_id, const size_t v2_id)
{
    // The base ORs its own m_is_on_surface, which is the union of the two; these say which.
    m_vertex_extra[v2_id].m_is_on_input =
        m_vertex_extra.at(v1_id).m_is_on_input || m_vertex_extra.at(v2_id).m_is_on_input;
    m_vertex_extra[v2_id].m_is_on_offset =
        m_vertex_extra.at(v1_id).m_is_on_offset || m_vertex_extra.at(v2_id).m_is_on_offset;
    m_vertex_extra[v2_id].m_is_on_region =
        m_vertex_extra.at(v1_id).m_is_on_region || m_vertex_extra.at(v2_id).m_is_on_region;

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
    m_vertex_extra[v_id].m_is_on_input =
        e.m_is_surface_fs && e.m_surface_class == INPUT_SURFACE_CLASS;
    m_vertex_extra[v_id].m_is_on_offset =
        e.m_is_surface_fs && e.m_surface_class == OFFSET_SURFACE_CLASS;
    m_vertex_extra[v_id].m_is_on_region =
        e.m_is_surface_fs && e.m_surface_class == REGION_SURFACE_CLASS;

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
    // Read before the base call: the base folds "on the bounding box" and "could not be
    // rounded to doubles" into one false, and those mean completely different things.
    const bool on_bbox = !m_vertex_attribute[t.vid(*this)].on_bbox_faces.empty();
    // rounds the vertex and refuses the bounding box
    if (!TriOptimizerMesh::smooth_before(t)) {
        if (on_bbox) {
            ++m_smooth_trace.before_bbox;
        } else {
            ++m_smooth_trace.before_unrounded;
        }
        return false;
    }
    // The input complex is no longer refused here. It is smoothed like any other tracked
    // surface: the shared smoother projects it back onto m_envelope, which is built around the
    // input as loaded. See vertex_is_on_domain_boundary()'s comment for why freezing it was
    // costing more than it bought.
    return true;
}

bool TopoOffsetTriMesh::smooth_after(const Tuple& t)
{
    // A vertex ON THE INPUT COMPLEX takes the shared path first, whatever else it is on. Its job
    // is to represent the input, and the shared smoother is what keeps it there: it projects
    // onto m_envelope and vetoes any position that leaves it -- TriWild's mechanism, applied to
    // TriWild's kind of surface. Aiming such a vertex at target_distance instead would be asking
    // it to leave the very geometry the distance is measured from; where the offset boundary
    // runs into the complex, that vertex is one the offset cannot place, and the metric counts
    // it as pinned rather than pretending otherwise (see distance_deviation_split()).
    if (m_vertex_extra[t.vid(*this)].m_is_on_input) {
        ++m_smooth_trace.interior_attempted;
        return TriOptimizerMesh::smooth_after(t);
    }
    // Only a vertex on the OFFSET boundary is placed against the input complex's distance field,
    // because only that surface is supposed to sit at target_distance. A vertex on some other
    // tag region's boundary is not: it goes to the shared AMIPS smoother like any other
    // background vertex, and the envelope plus the inversion test are what keep its region
    // where it is.
    if (m_vertex_extra[t.vid(*this)].m_is_on_offset) {
        ++m_smooth_trace.offset_attempted;
        const bool ok = project_offset_vertex(t);
        if (ok) ++m_smooth_trace.offset_accepted;
        return ok;
    }
    if (m_vertex_extra[t.vid(*this)].m_is_on_region) {
        ++m_smooth_trace.region_attempted;
    }
    ++m_smooth_trace.interior_attempted;
    return TriOptimizerMesh::smooth_after(t);
}

void TopoOffsetTriMesh::log_smooth_trace() const
{
    const auto& s = m_smooth_trace;
    logger().info(
        "\tsmooth trace: attempted {} | before: bbox {}, unrounded {}, on-input {} | "
        "offset path: attempted {} -> accepted {}, no-neighbours {}, on-complex {}, "
        "inverted {}, envelope {} | interior path: attempted {} ({} of them on another region "
        "boundary) ({})",
        s.attempted.load(),
        s.before_bbox.load(),
        s.before_unrounded.load(),
        s.before_on_input.load(),
        s.offset_attempted.load(),
        s.offset_accepted.load(),
        s.offset_no_neighbours.load(),
        s.offset_on_complex.load(),
        s.offset_inverted.load(),
        s.offset_envelope.load(),
        s.interior_attempted.load(),
        s.region_attempted.load(),
        m_smooth_rejects.to_string());
    const int acc = std::max(1, s.offset_accepted.load());
    logger().info(
        "\tsmooth moves: accepted {} of which clamped {} (envelope {}, inverted {}, slid {}) | "
        "err over moved verts: avg {:.6} -> {:.6}, max {:.6} -> {:.6}",
        s.offset_accepted.load(),
        s.offset_clamped.load(),
        s.offset_clamp_env.load(),
        s.offset_clamp_inv.load(),
        s.offset_slid.load(),
        double(s.offset_err_before_nano.load()) / acc * 1e-9,
        double(s.offset_err_after_nano.load()) / acc * 1e-9,
        s.offset_err_max_before_nano.load() * 1e-9,
        s.offset_err_max_after_nano.load() * 1e-9);
}

bool TopoOffsetTriMesh::project_offset_vertex(const Tuple& t)
{
    const size_t vid = t.vid(*this);
    const Vector2d p0 = m_vertex_attribute[vid].m_posf;

    // Laplacian over the offset-boundary neighbours only. Pulling in input-complex or interior
    // neighbours would drag the boundary off its shape -- the 2D counterpart of restricting the
    // 3D Laplacian to offset-surface neighbours.
    Vector2d p_laplace = Vector2d::Zero();
    int n = 0;
    for (const Tuple& e : get_one_ring_edges_for_vertex(t)) {
        if (!edge_is_offset(e.eid(*this))) continue;
        const size_t nb = (e.vid(*this) == vid) ? e.switch_vertex(*this).vid(*this) : e.vid(*this);
        if (nb == vid) continue;
        p_laplace += m_vertex_attribute[nb].m_posf;
        ++n;
    }
    if (n == 0) {
        ++m_smooth_trace.offset_no_neighbours;
        return false;
    }
    p_laplace /= n;

    // Projection onto the target distance: walk from the nearest point on the input complex
    // outward, in the direction the vertex already lies, by exactly target_distance.
    const Vector3d near3 = m_input_complex_bvh.nearest_point(VectorXd(p0));
    const Vector2d nearest(near3[0], near3[1]);
    const Vector2d diff = p0 - nearest;
    const double dist = diff.norm();
    if (dist < 1e-12) {
        ++m_smooth_trace.offset_on_complex;
        return false; // sitting on the input complex; no direction to offset along
    }
    const Vector2d p_proj = nearest + m_offset_params.target_distance * (diff / dist);

    const double w = m_offset_params.smooth_quadrics_weight;
    const double u = m_offset_params.smooth_laplacian_weight;
    const Vector2d target = (1 - w - u) * p0 + w * p_proj + u * p_laplace;

    const std::vector<Tuple> ring = get_one_ring_tris_for_vertex(t);

    // THE SHAPE BAR. The shared smoother refuses outright any position that raises the worst
    // quality in the ring (optimization::smooth_vertex_2d's quality veto). This placement had no
    // such test: any position that was non-inverted and inside the envelope was taken, however
    // badly shaped it left the neighbourhood. Non-inverted is a very weak bar -- a triangle can
    // be arbitrarily close to collinear and still have the right orientation -- and measured on
    // topological_offset_2d one smoothing pass took the mesh's worst AMIPS from 21.7 to 305430.
    //
    // That was survivable only while nothing read element quality: the old loop stopped on
    // distance alone. Now AMIPS is one of the three criteria the loop stops on, so a placement
    // free to wreck shape and a stop condition that waits for good shape simply fight, and the
    // run burns every iteration it is given.
    //
    // The bar is the WORSE of where the ring already is and stop_energy: never make a good ring
    // bad, never make a bad ring worse. That is the same "worse-of" convention this file already
    // applies to normal deviation in collapse_edge_after(), and it matters for the same reason --
    // a flat veto against the ring's current max would freeze a vertex whose neighbourhood is
    // already over target, which is exactly where the distance error tends to be worst.
    //
    // Folded into `rejected` rather than checked at the end, so the binary search below scales
    // the move back to the largest fraction that satisfies all three constraints at once. A
    // vertex is therefore still moved as far toward its target distance as shape allows, instead
    // of being refused outright.
    double quality_bar = 0.;
    for (const Tuple& f : ring) {
        quality_bar = std::max(quality_bar, get_quality(f.fid(*this)));
    }
    quality_bar = std::max(quality_bar, m_params.stop_energy);

    // The offset boundary is a region boundary, so it is envelope-contained like every other
    // one. This placement never goes through the shared operations, so the containment test
    // that the shared split and collapse get for free has to be made here -- folded into the
    // same predicate as the inversion and shape tests, so the binary search below satisfies all
    // of them at once.
    const auto rejected = [&]() {
        for (const Tuple& f : ring) {
            if (is_inverted(f)) return true;
        }
        for (const Tuple& f : ring) {
            if (get_quality(f.fid(*this)) > quality_bar) return true;
        }
        return region_boundary_is_outside_envelope(vid);
    };

    // Distance error at an arbitrary position -- what the whole projection exists to minimise,
    // and therefore the only fair way to compare two candidate placements.
    const auto dist_err = [&](const Vector2d& p) {
        const Vector3d n3 = m_input_complex_bvh.nearest_point(VectorXd(p));
        return std::abs((p - Vector2d(n3[0], n3[1])).norm() - m_offset_params.target_distance);
    };

    // The largest fraction of `step` that is accepted, leaving the vertex at that position.
    // Returns -1 if even the zero step is refused, i.e. p0 itself is illegal.
    const auto search = [&](const Vector2d& step) -> double {
        set_vertex_position(vid, p0 + step);
        if (!rejected()) return 1.;
        double lo = 0., hi = 1.;
        for (int it = 0; it < 10; ++it) {
            const double mid = 0.5 * (lo + hi);
            set_vertex_position(vid, p0 + mid * step);
            if (rejected()) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        set_vertex_position(vid, p0 + lo * step);
        return rejected() ? -1. : lo;
    };

    // How far this vertex is off the target distance before and after the move. A move that is
    // "accepted" may still have been clamped to a small fraction of what was asked for, so the
    // acceptance count alone says nothing about whether the error actually came down.
    const auto record = [&](double frac) {
        const Vector2d p = m_vertex_attribute[vid].m_posf;
        const double e_before = std::abs(dist - m_offset_params.target_distance);
        const double e_after = dist_err(p);
        const auto nano = [](double x) { return static_cast<int>(x * 1e9); };
        m_smooth_trace.offset_err_before_nano += nano(e_before);
        m_smooth_trace.offset_err_after_nano += nano(e_after);
        atomic_max(m_smooth_trace.offset_err_max_before_nano, nano(e_before));
        atomic_max(m_smooth_trace.offset_err_max_after_nano, nano(e_after));
        if (frac < 0.99) {
            ++m_smooth_trace.offset_clamped;
            // Which constraint stopped the search: step just past the accepted fraction and see
            // which of the two predicates fires there.
            const double past = std::min(1.0, frac + (1. - frac) * 0.5 + 1e-9);
            set_vertex_position(vid, p0 + past * (target - p0));
            bool inv = false;
            for (const Tuple& f : ring) {
                if (is_inverted(f)) {
                    inv = true;
                    break;
                }
            }
            if (inv) {
                ++m_smooth_trace.offset_clamp_inv;
            } else {
                ++m_smooth_trace.offset_clamp_env;
            }
            set_vertex_position(vid, p); // restore the accepted position
        }
    };

    // Scale the blended move back toward the known-valid p0 until it is accepted.
    const Vector2d move = target - p0;
    const double frac = search(move);
    if (frac >= 0.) {
        // TANGENTIAL SLIDE. The search above can only SCALE `move`, and that is not enough for a
        // vertex sitting flush against the envelope wall: if p0 is on the wall and target is even
        // slightly outside it, every positive multiple of `move` is outside too, so the search
        // returns 0 and the vertex is frozen for the rest of the run -- even when most of the
        // motion it wants is ALONG the wall and perfectly legal.
        //
        // Measured on the dragon rectangle at target_distance_rel 5e-3: one triple point where a
        // region curve terminates on the offset held max_dist_err at 22% of target for every
        // iteration, refusing a step whose direction was only 41 degrees off its region edge --
        // 76% tangential -- while a probe along that tangent accepted twice the distance needed.
        //
        // So when the scaled search comes up short, solve the 1D problem on the region curve
        // instead -- see minimize_distance_along_tangent(), which also records why the projected
        // step this used to take was the wrong quantity. Guarded two ways: it runs only where a
        // region tangent is actually defined (see region_boundary_tangent()), and it is kept only
        // if it lowers the distance error below what the scaled search achieved, so it can never
        // make a vertex worse than before.
        Vector2d tang;
        if (frac < 0.99 && region_boundary_tangent(t, tang)) {
            const Vector2d p_scaled = m_vertex_attribute[vid].m_posf;
            const double e_scaled = dist_err(p_scaled);

            // How far along the curve one pass may travel: the shortest incident region edge,
            // but never less than the projected step, so the search can only ever see more of
            // the line than the old behaviour did. Sliding past a neighbour would fold the curve
            // and invert a triangle, which `rejected` catches anyway -- this just keeps the
            // bracket in the range where the tangent still describes the curve.
            double cap = std::abs(move.dot(tang));
            for (const Tuple& e : get_one_ring_edges_for_vertex(t)) {
                if (!edge_is_region_boundary_live(e)) continue;
                const size_t nb =
                    (e.vid(*this) == vid) ? e.switch_vertex(*this).vid(*this) : e.vid(*this);
                cap = std::max(cap, (m_vertex_attribute[nb].m_posf - p0).norm());
            }

            minimize_distance_along_tangent(vid, p0, tang, cap, rejected);
            if (dist_err(m_vertex_attribute[vid].m_posf) >= e_scaled) {
                set_vertex_position(vid, p_scaled); // no better; keep the scaled result
            } else {
                ++m_smooth_trace.offset_slid;
            }
        }
        // THE INCIDENT FACES' CACHED QUALITY MUST BE REWRITTEN, exactly as the shared smoother
        // does before it returns (optimization::smooth_vertex_2d). This placement moves a vertex
        // without going through any shared operation, so nothing else will.
        //
        // Leaving it stale is not merely a reporting problem. m_quality is what the collapse
        // compares its ring against (collapse_quality_allowed, CollapseInfoCache::max_energy and
        // changed_energies) and what the swap weighs, so every such decision taken next to the
        // offset boundary was being made on the quality the mesh had before the boundary last
        // moved. It went unnoticed because label_offset_boundary() recomputes every face's
        // quality once per iteration, which hid the staleness from anything that only looked
        // between iterations -- including the old convergence report. It stops being hidden the
        // moment AMIPS enters the loop's stop metric: measured on topological_offset_2d, the
        // per-pass log ended an iteration at 14.3x target while the recomputed value at the top
        // of the next was 3.2e5x.
        for (const Tuple& f : ring) {
            const size_t fid = f.fid(*this);
            m_face_attribute[fid].m_quality = get_quality(fid);
        }

        record(frac);
        return true;
    }
    // Attribute the failure: the search ran out and the vertex is still somewhere it may not
    // be. Which of the two it is decides what to do about it -- an inversion means the band is
    // locally too coarse to hold the projected position, an envelope failure means the
    // envelope is too tight.
    bool inv = false;
    for (const Tuple& f : ring) {
        if (is_inverted(f)) {
            inv = true;
            break;
        }
    }
    if (inv) {
        ++m_smooth_trace.offset_inverted;
    } else {
        ++m_smooth_trace.offset_envelope;
    }
    return false;
}

bool TopoOffsetTriMesh::region_boundary_tangent(const Tuple& t, Vector2d& tang) const
{
    const size_t vid = t.vid(*this);
    const Vector2d p0 = m_vertex_attribute[vid].m_posf;

    // LIVE, not the cached m_surface_class: this is called from inside the smoothing pass, after
    // split and collapse have created edges the once-per-iteration relabelling never saw. Same
    // reasoning as surface_envelope_for_edge(); querying the stale cache would report "not
    // region" on a fresh edge and silently disable the slide exactly where a collapse just
    // rearranged the curve.
    std::vector<Vector2d> nb;
    for (const Tuple& e : get_one_ring_edges_for_vertex(t)) {
        if (!edge_is_region_boundary_live(e)) continue;
        const size_t other =
            (e.vid(*this) == vid) ? e.switch_vertex(*this).vid(*this) : e.vid(*this);
        if (other == vid) continue;
        nb.push_back(m_vertex_attribute[other].m_posf);
        if (nb.size() > 2) return false; // a branch point: no single direction to slide along
    }

    Vector2d d;
    if (nb.size() == 2) {
        d = nb[1] - nb[0]; // curve passing through: the chord is its discrete tangent
    } else if (nb.size() == 1) {
        d = nb[0] - p0; // curve terminating here: its own last segment is the only direction
    } else {
        return false;
    }
    const double n = d.norm();
    if (n < 1e-14) return false; // degenerate segment; normalising would give garbage
    tang = d / n;
    return true;
}

double TopoOffsetTriMesh::minimize_distance_along_tangent(
    const size_t vid,
    const Vector2d& p0,
    const Vector2d& tang,
    const double cap,
    const std::function<bool()>& rejected)
{
    const auto err_at = [&](const double s) {
        const Vector2d p = p0 + s * tang;
        set_vertex_position(vid, p);
        if (rejected()) return std::numeric_limits<double>::infinity();
        const Vector3d n3 = m_input_complex_bvh.nearest_point(VectorXd(p));
        return std::abs((p - Vector2d(n3[0], n3[1])).norm() - m_offset_params.target_distance);
    };

    // How far the vertex may travel in one direction. Expanding geometrically rather than
    // bisecting down from `cap` matters: bisection from a rejected endpoint can only ever return
    // something smaller than what it started with, which is the very bias -- always shortening,
    // never lengthening -- that the projected step suffered from.
    const auto extent = [&](const double sign) {
        double good = 0., bad = cap;
        bool found_bad = false;
        for (double s = cap / 64.; s <= cap * 1.0000001; s *= 2.) {
            set_vertex_position(vid, p0 + (sign * s) * tang);
            if (rejected()) {
                bad = s;
                found_bad = true;
                break;
            }
            good = s;
        }
        if (!found_bad) return good; // the whole capped range is feasible
        for (int i = 0; i < 12; ++i) {
            const double mid = 0.5 * (good + bad);
            set_vertex_position(vid, p0 + (sign * mid) * tang);
            if (rejected()) {
                bad = mid;
            } else {
                good = mid;
            }
        }
        return good;
    };

    double a = -extent(-1.), b = extent(1.);
    if (b - a < 1e-15) {
        set_vertex_position(vid, p0);
        return 0.;
    }

    // Golden-section on |dist - target|. dist() is a smooth distance field away from the input
    // complex's features, so over a segment this short the objective is a V with one minimum.
    // An infeasible probe scores infinite, which walks the bracket back off it rather than
    // trusting a position the caller would refuse.
    constexpr double PHI = 0.6180339887498949;
    double c = b - PHI * (b - a), d = a + PHI * (b - a);
    double fc = err_at(c), fd = err_at(d);
    for (int i = 0; i < 24 && (b - a) > 1e-15; ++i) {
        if (fc <= fd) {
            b = d;
            d = c;
            fd = fc;
            c = b - PHI * (b - a);
            fc = err_at(c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + PHI * (b - a);
            fd = err_at(d);
        }
    }
    const double s_best = (fc <= fd) ? c : d;
    set_vertex_position(vid, p0 + s_best * tang);
    return s_best;
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
        if (n == 0) continue; // not on the offset surface: the background keeps the base target
        raw_sum += sum_len / n;
        ++n_seeded;
        m_vertex_attribute[vid].m_sizing_scalar =
            std::clamp((sum_len / n) / l, s_floor, m_offset_params.max_sizing_scalar);
    }
    logger().info(
        "\tOffset sizing seed: {} vertices, mean incident length {:.6} -> target {:.6} "
        "(base l {:.6}, l_min {:.6} = 2*{}*sin({} deg), scalar floor {:.6})",
        n_seeded,
        n_seeded > 0 ? raw_sum / n_seeded : 0.,
        std::max(n_seeded > 0 ? raw_sum / n_seeded : 0., m_offset_params.min_edge_length),
        l,
        m_offset_params.min_edge_length,
        m_offset_params.target_distance,
        m_offset_params.max_normal_deviation_deg,
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

void TopoOffsetTriMesh::optimization_iteration_begin()
{
    label_offset_boundary();

    // The three criteria, separately, once per iteration. optimization_quality_stats() returns
    // only their max, so without this the log says the loop is stuck without saying on what --
    // and the three behave completely differently: AMIPS is the mesh's own business, distance is
    // driven by the smoother, and normal deviation has a floor at every feature of the input.
    double max_q = 0.;
    for (const Tuple& f : get_faces()) {
        max_q = std::max(max_q, quality_rel(f.fid(*this)));
    }
    const DistanceSplit d = distance_deviation_split();
    const auto [max_nd, avg_nd] = compute_normal_deviation();
    const double ct = std::max(m_offset_params.convergence_target, 1e-16);
    const double cnd = m_offset_params.convergence_normal_deviation;
    logger().info(
        "[criteria] amips {:.4}x (max {:.6} / {:.6}) | dist {:.4}x (max {:.6} / {:.6}, {} pinned) "
        "| normal {} | #V {} #F {} (band {})",
        max_q,
        max_q * m_params.stop_energy,
        m_params.stop_energy,
        d.max_reachable / ct,
        d.max_reachable,
        m_offset_params.convergence_target,
        d.n_pinned,
        cnd > 0. ? fmt::format("{:.4}x (avg {:.6} / {:.6})", avg_nd / cnd, avg_nd, cnd)
                 : fmt::format("off (avg {:.6}, max {:.6})", avg_nd, max_nd),
        get_vertices().size(),
        get_faces().size(),
        d.n_reachable + d.n_pinned);
}

std::tuple<double, double> TopoOffsetTriMesh::optimization_quality_stats()
{
    // TriWild's own criterion, unchanged and first: the worst face relative to stop_energy.
    // quality_rel() is the base's, so this is exactly the number TriWild's loop stops on.
    double max_q = 0., sum_q = 0.;
    size_t n_faces = 0;
    for (const Tuple& f : get_faces()) {
        const double q = quality_rel(f.fid(*this));
        max_q = std::max(max_q, q);
        sum_q += q;
        ++n_faces;
    }
    double max_metric = max_q;
    double avg_metric = (n_faces > 0) ? sum_q / n_faces : 0.;

    // Distance, over the reachable band only. A pinned vertex cannot be placed at
    // target_distance by any operation, so leaving it in would hold the metric above the bar
    // forever: the loop would never stop and the stall detector would fire every iteration,
    // refining around vertices nothing can help.
    const double ct = std::max(m_offset_params.convergence_target, 1e-16);
    const DistanceSplit d = distance_deviation_split();
    max_metric = std::max(max_metric, d.max_reachable / ct);
    avg_metric = std::max(avg_metric, d.avg_reachable / ct);

    // Normal deviation, on the AVERAGE. The max has a floor at every sharp feature of the input
    // that no refinement can lower, which is why the convergence criterion asks for the average
    // and why this does too. <= 0 disables the criterion entirely.
    if (m_offset_params.convergence_normal_deviation > 0.) {
        const double cnd = m_offset_params.convergence_normal_deviation;
        const auto [max_nd, avg_nd] = compute_normal_deviation();
        (void)max_nd;
        max_metric = std::max(max_metric, avg_nd / cnd);
        avg_metric = std::max(avg_metric, avg_nd / cnd);
    }

    return {max_metric, avg_metric};
}

double TopoOffsetTriMesh::face_criterion_rel(const size_t fid) const
{
    // The per-face form of optimization_quality_stats()'s max: the same three criteria, each
    // over its own target, restricted to what this face carries. >= 1 means the face fails at
    // least one of them, which is what makes it a candidate for refinement.
    double score = quality_rel(fid);

    const double ct = std::max(m_offset_params.convergence_target, 1e-16);
    const double cnd = m_offset_params.convergence_normal_deviation;
    for (int j = 0; j < 3; ++j) {
        const Tuple e = tuple_from_edge(fid, j);
        if (!edge_is_offset_surface_live(e)) continue;
        if (cnd > 0.) {
            score = std::max(score, edge_normal_deviation(e) / cnd);
        }
        for (const size_t vid : {e.vid(*this), e.switch_vertex(*this).vid(*this)}) {
            if (!band_vertex_is_reachable(vid)) continue;
            score = std::max(score, band_vertex_distance_error(vid) / ct);
        }
    }
    return score;
}

size_t TopoOffsetTriMesh::refine_sizing_around_worst(const double max_metric)
{
    // TriWildMesh::refine_sizing_around_worst, with face_criterion_rel() in place of the raw
    // AMIPS energy and 1.0 -- the normalized target -- in place of stop_energy. At TriWild's own
    // default stop_energy of 100 the two filter expressions are the same number.
    //
    // The clamp is TriWild's and exists for the same reason: without it a single degenerate face
    // (quality MAX_ENERGY) sets the filter astronomically high and select_worst_cells then picks
    // out only the degenerate faces, so refinement stops seeing the merely-bad ones it exists to
    // fix.
    const int n_rings = std::max(0, m_params.stuck_refine_rings);
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
            m_force_split_edges.insert(wmtk::utils::longest_edge(
                oriented_tri_vids(fid),
                [this](size_t vid) -> const Vector2d& { return m_vertex_attribute[vid].m_posf; }));
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
    // The band split, first: how much of the distance error is the optimizer's to fix. The loop
    // and the sizing field only ever see the reachable half, so a run whose reported max looks
    // bad but whose reachable max is fine is a construction problem, not an optimization one.
    {
        const DistanceSplit d = distance_deviation_split();
        logger().info(
            "\tband split: {} reachable (max err {:.6}, avg {:.6}) | {} PINNED (max err {:.6})",
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
    int n_offset_e = 0, n_region_e = 0, n_input_e = 0, n_bbox_e = 0;
    for (const Tuple& e : get_one_ring_edges_for_vertex(tuple_from_vertex(vid))) {
        const size_t eid = e.eid(*this);
        n_offset_e += edge_is_offset_surface_live(e);
        n_region_e += edge_is_region_boundary_live(e);
        n_input_e += edge_is_input(eid);
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
    // NOT "envelope-blocked": this evaluates containment at the vertex's CURRENT position, which
    // for any vertex the optimization left in place is inside the tube by construction, so it
    // reads false even for a vertex every proposed move is refused for. Blocking is a property of
    // the position being PROPOSED, and there is none to test here. What is worth reporting is
    // whether the tangential slide is available if the scaled search does come up short.
    Vector2d tang;
    logger().info(
        "\t  flags: on_offset {} on_input {} on_region {} on_bbox {} rounded {} | incident edges: "
        "{} offset, {} region, {} input, {} bbox | outside-envelope-now {} | region-tangent {}",
        ve.m_is_on_offset,
        ve.m_is_on_input,
        ve.m_is_on_region,
        !m_vertex_attribute[vid].on_bbox_faces.empty(),
        m_vertex_attribute[vid].m_is_rounded,
        n_offset_e,
        n_region_e,
        n_input_e,
        n_bbox_e,
        region_boundary_is_outside_envelope(vid),
        region_boundary_tangent(tuple_from_vertex(vid), tang));
    // Which smoothing path it would take, and whether it is refused before reaching one.
    const char* fate = "project_offset_vertex (distance-driven)";
    if (!m_vertex_attribute[vid].on_bbox_faces.empty()) {
        fate = "REFUSED by smooth_before: on the bounding box";
    } else if (!m_vertex_attribute[vid].m_is_rounded) {
        fate = "REFUSED by smooth_before: not rounded";
    } else if (ve.m_is_on_input) {
        fate = "REFUSED by smooth_before: on the input complex (frozen)";
    } else if (!ve.m_is_on_offset) {
        fate = "shared AMIPS smoother -- distance error is NOT what moves it";
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
            switch (m_edge_attribute[eid].m_surface_class) {
            case OFFSET_SURFACE_CLASS: cls = "OFFSET"; break;
            case REGION_SURFACE_CLASS: cls = "REGION"; break;
            default: cls = "INPUT"; break;
            }
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

bool TopoOffsetTriMesh::offset_field_normal(const Vector2d& p, Vector2d& n) const
{
    // "The offset normal can be computed for any point in space by finding the projection point
    // on the offset and normalizing the vector from the point in space to its projection."
    // (paper, Appendix A). The offset is the level set of the distance to the input complex, so
    // its normal at p is the unit vector from the nearest point on the complex out to p.
    const Vector3d near3 = m_input_complex_bvh.nearest_point(VectorXd(p));
    const Vector2d diff = p - Vector2d(near3[0], near3[1]);
    const double d = diff.norm();
    if (d < 1e-12) return false; // on the complex: the field has no direction here
    n = diff / d;
    return true;
}

double TopoOffsetTriMesh::edge_normal_deviation(const Tuple& e) const
{
    // Paper Definition 5, verbatim in 2D: the maximum angle between the OFFSET normal at the
    // element's center and the OFFSET normal at other points within the element.
    //
    //     sigma(t) = max over p_i in t of angle( n(p_c), n(p_i) ),  p_i = 0.1*p_c + 0.9*p_v
    //
    // Both terms are field normals; the element's own geometric normal does not appear. This is
    // deliberate and it is what makes the quantity converge: n() is continuous away from the
    // input complex's features, so shrinking an element brings its samples closer together and
    // drives sigma to zero. A variant that compared the element's own normal against the field
    // (which is what the 3D branch does) measures something else -- misorientation, which a
    // finer element does not by itself fix -- and cannot be driven down by refinement, so the
    // sizing field below could never satisfy it.
    const size_t va = e.vid(*this), vb = e.switch_vertex(*this).vid(*this);
    const Vector2d p0 = m_vertex_attribute[va].m_posf;
    const Vector2d p1 = m_vertex_attribute[vb].m_posf;

    const Vector2d p_c = 0.5 * (p0 + p1); // "center" of a 1-simplex
    Vector2d n_c;
    if (!offset_field_normal(p_c, n_c)) return 0.;

    // p_i = 0.1*p_c + 0.9*p_v, the paper's sample positions, one per vertex. Two here rather
    // than 3D's three, because a 1-simplex has two vertices.
    constexpr double u = 0.1;
    const std::array<Vector2d, 2> samples = {{u * p_c + (1 - u) * p0, u * p_c + (1 - u) * p1}};

    double max_dev = 0.;
    for (const Vector2d& s : samples) {
        Vector2d n_i;
        if (!offset_field_normal(s, n_i)) continue;
        // A genuine angle between two field normals, both of which point outward from the
        // complex, so there is no orientation ambiguity to fold away here.
        const double c = std::clamp(n_c.dot(n_i), -1., 1.);
        max_dev = std::max(max_dev, (180. / M_PI) * std::acos(c));
    }
    return max_dev;
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

double TopoOffsetTriMesh::max_offset_normal_deviation_at_vertex(const size_t vid) const
{
    double max_nd = 0.;
    for (const Tuple& e : get_one_ring_edges_for_vertex(tuple_from_vertex(vid))) {
        if (!edge_is_offset_surface_live(e)) continue;
        max_nd = std::max(max_nd, edge_normal_deviation(e));
    }
    return max_nd;
}

// returns max_norm_dev, avg_norm_dev, both in degrees
std::pair<double, double> TopoOffsetTriMesh::compute_normal_deviation() const
{
    // The same band-outer-surface rule compute_distance_deviation() uses, and for the same
    // reason: the band's inner interface hugs the input complex, where the "ideal" normal is
    // meaningless, so measuring there would report an angle that means nothing.
    double max_dev = 0.0;
    double sum_dev = 0.0;
    int n_edges = 0;
    for (const Tuple& e : get_edges()) {
        if (!edge_is_offset_surface_live(e)) continue;
        const double dev = edge_normal_deviation(e);
        max_dev = std::max(max_dev, dev);
        sum_dev += dev;
        ++n_edges;
    }
    const double avg_dev = (n_edges > 0) ? sum_dev / n_edges : 0.0;
    return std::make_pair(max_dev, avg_dev);
}

void TopoOffsetTriMesh::optimize_offset(const std::filesystem::path& output_file)
{
    logger().info("Optimizing offset (2D)...");
    logger().info(
        "\ttarget_distance: {} | convergence_target: {}",
        m_offset_params.target_distance,
        m_offset_params.convergence_target);
    optimization_metrics.clear();
    op_counts.clear();

    logger().info("\tLabelling tracked surfaces...");
    label_offset_boundary();

    // Built once, from the region boundaries as constructed. Rebuilding it per iteration would
    // re-anchor it to wherever the boundaries had drifted to, which is no constraint at all.
    init_region_boundary_envelope();

    // Seed the sizing field from the offset's current edge lengths, before any operation runs.
    init_offset_sizing_field();

    // From here on every edge split is an optimization split, run by the shared engine. The
    // marching-triangles placement modes require one endpoint inside the offset and one
    // outside, which does not hold for an arbitrary long edge.
    m_edge_split_mode = EdgeSplitMode::Optimization;

    if (m_offset_params.debug_output) {
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    // THE SHARED ENGINE'S OWN LOOP, driving the offset's convergence criterion -- the same move
    // 3D made in 9aa78ded29, and for the same reasons. What this replaces was a fixed number of
    // split/collapse/swap/smooth rounds with a whole-band sizing sweep after each: no stall
    // detection, no `it pre`/`it post` passes, no consolidation between iterations, and a
    // refine_sizing_around_worst() override that nothing could ever call.
    //
    // The offset plugs into mesh_improvement() through three virtuals, exactly as simwild does:
    //   - optimization_quality_stats(): the max of AMIPS, distance and normal deviation, each
    //     over its own target;
    //   - optimization_stop_metric(): 1.0, since all three are normalized;
    //   - refine_sizing_around_worst(): TriWild's, fired only on a stall.
    // The engine consolidates every iteration and re-collects the operation queue, which is
    // also its own answer to slot-pool exhaustion: work dropped in one pass is retried in the
    // next.
    //
    // optimization_iteration_begin() re-derives the tracked surfaces each iteration. The shared
    // operations maintain the edge tags they are given, but a split creates edges the labelling
    // never classified, and the collapse's substructure link condition is only as good as the
    // substructure it is shown.
    iter_cnt_split = 0;
    iter_cnt_collapse = 0;
    iter_cnt_swap = 0;
    iter_cnt_collapse_nd_reject = 0;
    iter_cnt_swap_nd_reject = 0;
    m_smooth_trace.reset();

    mesh_improvement(m_offset_params.max_iterations);

    // Cumulative over the whole run, not per iteration as before: the engine loop has no
    // per-iteration hook for reporting, and the per-pass numbers it logs itself carry the
    // history. Matches 3D.
    log_smooth_trace();
    op_counts.push_back({{iter_cnt_split.load(), iter_cnt_collapse.load(), iter_cnt_swap.load()}});
    logger().info(
        "splits = {}  |  collapses = {} ({} refused by normal deviation)  |  swaps = {} ({} "
        "refused by normal deviation)",
        iter_cnt_split.load(),
        iter_cnt_collapse.load(),
        iter_cnt_collapse_nd_reject.load(),
        iter_cnt_swap.load(),
        iter_cnt_swap_nd_reject.load());

    bool converged = false;
    {
        // metrics
        const auto [max_dist, avg_dist] = compute_distance_deviation();
        const auto [max_norm, avg_norm] = compute_normal_deviation();
        logger().info(
            "max dist err: {} | avg dist err: {} | max normal dev: {} | avg normal dev: {}",
            max_dist,
            avg_dist,
            max_norm,
            avg_norm);
        optimization_metrics.push_back({{max_dist, avg_dist, max_norm, avg_norm}});
        log_worst_dist_vertex();

        // THE VERDICT IS ON THE REACHABLE BAND, which is what the optimization controls. A
        // pinned vertex -- one on the input complex, or on the domain boundary where growth ran
        // out of room -- cannot be placed at target_distance by any operation, so failing the
        // run on it would be reporting a construction defect as an optimization failure. It is
        // warned about separately, right below, and compute_distance_deviation() above still
        // reports the whole band so the defect is never hidden.
        const DistanceSplit d = distance_deviation_split();

        // Convergence: the worst-placed reachable offset vertex has reached the target error
        // band, and the offset's AVERAGE facing has reached the target normal deviation.
        //
        // The asymmetry -- max for distance, average for angle -- is the paper's (Sec. 5.3,
        // Termination: "the average sigma_max ... both the maximum and mean distance error"),
        // and it is forced by the geometry rather than chosen for convenience. Distance error
        // genuinely can go to zero everywhere, so the max is a fair bar. Normal deviation
        // cannot: at a reentrant corner of the input the distance field's normal is
        // discontinuous by the corner's own angle, at every scale, so max sigma has a floor set
        // by the input's sharpest feature that no refinement, no smaller target_distance and no
        // operation can lower. Measured on the dragon body, max sigma sat at exactly 83.66 deg
        // for two different target distances -- a fixed angle surviving a 2x change in delta is
        // the input's geometry, not something the optimizer left undone.
        //
        // max_norm is still computed and reported; it is a diagnostic, not a bar.
        // convergence_normal_deviation <= 0 disables the angular half entirely.
        const bool dist_ok = d.max_reachable <= m_offset_params.convergence_target;
        const bool norm_ok = m_offset_params.convergence_normal_deviation <= 0. ||
                             avg_norm <= m_offset_params.convergence_normal_deviation;
        converged = dist_ok && norm_ok;
        if (converged) {
            if (m_offset_params.convergence_normal_deviation > 0.) {
                logger().info(
                    "Converged ([max_dist] {} <= {} [convergence_target], [avg_normal_dev] {} <= "
                    "{} [convergence_normal_deviation])",
                    d.max_reachable,
                    m_offset_params.convergence_target,
                    avg_norm,
                    m_offset_params.convergence_normal_deviation);
            } else {
                logger().info(
                    "Converged ([max_dist] {} <= {} [convergence_target], normal deviation "
                    "criterion disabled)",
                    d.max_reachable,
                    m_offset_params.convergence_target);
            }
        }

        // A pinned vertex outside the target band is not something the optimizer can fix, so it
        // is reported here rather than failing the run -- but it IS a defect in the offset, so
        // it is never silent.
        if (d.max_pinned > m_offset_params.convergence_target) {
            logger().warn(
                "{} of {} band vertices are PINNED and {} of them sit outside the target band "
                "(worst error {} against {} [convergence_target]). A pinned vertex lies on the "
                "input complex, where its distance to the complex is 0 by definition, or on the "
                "domain boundary, where conservative growth ran out of room. No split, collapse "
                "or smoothing move can place it at target_distance, so this is the offset as "
                "CONSTRUCTED, not something the optimization left undone: the band still touches "
                "the input complex, or the box is too tight for target_distance.",
                d.n_pinned,
                d.n_pinned + d.n_reachable,
                d.n_pinned,
                d.max_pinned,
                m_offset_params.convergence_target);
        }
    }
    if (!converged && !optimization_metrics.empty()) {
        // Name the criterion that actually failed; with two of them, reporting only the distance
        // sends you looking at the wrong one.
        const double max_dist = distance_deviation_split().max_reachable;
        const double avg_norm = optimization_metrics.back()[3];
        if (max_dist > m_offset_params.convergence_target) {
            logger().warn(
                "Optimization did not converge ([max_dist] {} > {} [convergence_target])",
                max_dist,
                m_offset_params.convergence_target);
        }
        if (m_offset_params.convergence_normal_deviation > 0. &&
            avg_norm > m_offset_params.convergence_normal_deviation) {
            logger().warn(
                "Optimization did not converge ([avg_normal_dev] {} > {} "
                "[convergence_normal_deviation])",
                avg_norm,
                m_offset_params.convergence_normal_deviation);
        }

        // Is topological preservation holding the offset back, as opposed to a handful of
        // over-constrained vertices?
        //
        // The discriminator is the AVERAGE error relative to target_distance, not the max, and
        // not max/avg. Topological blocking is WIDESPREAD -- whole stretches of offset cannot
        // advance, so the average climbs toward the max. Every other failure mode seen on the
        // dragon rectangle is the opposite shape: one over-constrained vertex pins the max while
        // the rest of the offset lands correctly, leaving the average tiny.
        //
        //   run                                    max/avg   avg/delta   max/delta
        //   td=.02 env 1e-3  CONVERGED                  4.5      0.15%       0.69%
        //   td=.02 env 1e-4  CONVERGED                  6.0      0.15%       0.93%
        //   td=.02 env 1e-6  not converged             65.7      0.06%       3.78%
        //   td=.1  respect_all_topologies=false        25.7      0.19%       4.77%
        //   td=.1  respect_all_topologies=TRUE          2.4     35.62%      86.31%
        //   td=.1  respect_all_topologies=TRUE          3.0     28.62%      86.31%
        //
        // avg/delta separates the blocked runs from every other run by more than 150x, and any
        // threshold in [1%, 20%] splits them. max/avg would be actively wrong: it is LOWEST on
        // exactly the runs to be caught and higher on both converged ones, so a "max > 2*avg"
        // test fires on every run in the table.
        const double avg_dist = optimization_metrics.back()[1];
        if (m_offset_params.respect_all_topologies &&
            avg_dist > TOPOLOGY_BLOCK_AVG_FRAC * m_offset_params.target_distance) {
            logger().warn(
                "Offset growth appears to be BLOCKED BY TOPOLOGICAL PRESERVATION: "
                "respect_all_topologies is true and the AVERAGE distance error is {:.1f}% of "
                "target_distance ({} vs {}), against max {:.1f}% -- a max/avg ratio of only "
                "{:.1f}. Error that uniform means whole regions of the offset never reached the "
                "target distance, not that a few vertices are over-constrained; with "
                "respect_all_topologies every tag's topology is frozen after offset "
                "initialization, so the operations that would let the offset advance through a "
                "narrow region are refused and it stays where conservative growth left it. "
                "Set respect_all_topologies false if only the offset's own topology matters, or "
                "reduce target_distance so the offset does not need to pass through those "
                "regions.",
                100.0 * avg_dist / m_offset_params.target_distance,
                avg_dist,
                m_offset_params.target_distance,
                100.0 * max_dist / m_offset_params.target_distance,
                avg_dist > 0. ? max_dist / avg_dist : 0.);
        }
    }

    // Escalate to a hard failure if the caller asked for it. Deliberately AFTER the warnings
    // above, so the log still names which criterion missed and by how much before the throw --
    // the exception message alone cannot say whether it was distance or normal deviation, and on
    // a failing integration test that is the first thing anyone needs.
    //
    // Guarded on !converged alone rather than on the block above, which additionally requires
    // optimization_metrics to be non-empty: a run that produced no metrics at all has certainly
    // not converged, and silently returning success there is exactly the regression this exists
    // to catch.
    if (!converged && m_offset_params.throw_on_nonconvergence) {
        log_and_throw_error(
            "Optimization did not converge and throw_on_nonconvergence is set. Ran {} of {} "
            "iterations; see the warnings above for the criterion that failed.",
            m_iterations_used,
            m_offset_params.max_iterations);
    }

    // Deliberately NOT re-derived here. From the moment the tracked surfaces are tagged, it is
    // the shared operations that maintain those tags -- the face LABELS they were derived from
    // are construction data the optimization does not propagate, so re-deriving now would read
    // stale labels and mislabel the result.
}

} // namespace wmtk::components::topological_offset
