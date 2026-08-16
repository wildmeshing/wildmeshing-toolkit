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

        // A VERTEX THAT LIES ON THE INPUT COMPLEX IS ON THE INPUT COMPLEX, whatever its incident
        // edges say.
        //
        // The classification above derives m_is_on_input from edges where an input-complex FACE
        // meets a non-input one. That is the whole story for a 2-dimensional input, and it is
        // EMPTY BY CONSTRUCTION for a 0- or 1-dimensional one: an input made of isolated points
        // has no input faces, so no edge is ever INPUT_SURFACE_CLASS, and the three input points
        // of topological_offset_2d_vertex_input were classified as ordinary offset-boundary
        // vertices sitting at distance 0 from the complex -- exactly where Phi diverges. The
        // smoother was then handed an infinite energy at those vertices and the run's own
        // convergence metric read infinity.
        //
        // The geometric test is the definition, and unlike the construction-time vertex LABEL it
        // cannot go stale across a split or a collapse that recycles an attribute slot. Distance
        // zero is exact for a vertex that IS an input vertex (identical coordinates) and for one
        // lying on an input segment; a vertex that has merely drifted close is still reachable
        // and is deliberately not caught here.
        if (m_input_complex_bvh.dist(VectorXd(m_vertex_attribute[vid].m_posf)) <= 0.) {
            m_vertex_extra[vid].m_is_on_input = true;
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

bool TopoOffsetTriMesh::swap_edge_after(const Tuple& t)
{
    if (!TriOptimizerMesh::swap_edge_after(t)) {
        return false;
    }
    // A SWAP IS ACCEPTED BY THE SAME CRITERION THE SMOOTHING MINIMISES -- see collapse_edge_after()
    // for the argument, which is identical. A flip moves no vertex, but the diagonal it picks
    // decides whether the boundary follows the level set or cuts across it.
    if (m_offset_potential) {
        for (const size_t fid : get_one_ring_fids_for_vertex(t)) {
            if (face_criterion_rel(fid) > 1.0) {
                ++iter_cnt_swap_offset_reject;
                return false;
            }
        }
    }
    ++iter_cnt_swap;
    return true;
}

bool TopoOffsetTriMesh::collapse_edge_after(const Tuple& t)
{
    if (!TriOptimizerMesh::collapse_edge_after(t)) {
        return false;
    }
    // A COLLAPSE IS ACCEPTED BY THE SAME CRITERION THE SMOOTHING MINIMISES. See the declaration
    // for the argument. FLAT, not "no worse than before": a before/after bar reads the max over a
    // patch, so one already-bad face licenses coarsening its whole neighbourhood -- measured in
    // 3D, the worse-of bar still let the offset surface fall from 1172 faces to 496.
    if (m_offset_potential) {
        for (const size_t fid : get_one_ring_fids_for_vertex(t)) {
            if (face_criterion_rel(fid) > 1.0) {
                ++iter_cnt_collapse_offset_reject;
                return false;
            }
        }
    }
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
    // EVERY VERTEX TAKES THE SHARED PATH. What used to be a fork -- offset-boundary vertices to
    // a hand-rolled projection, everything else to the shared smoother -- is now a difference in
    // the OBJECTIVE, assembled by smoothing_extra_energy() and smoothing_envelope(). So the
    // offset boundary gets the shared line search and the exact inversion test that the
    // projection never had, and this hook is left with nothing to do but count. (The shared
    // quality veto is turned OFF for this application -- see SmoothVertexOptions::quality_veto.)
    const size_t vid = t.vid(*this);
    const auto& ve = m_vertex_extra[vid];
    if (ve.m_is_on_region) {
        ++m_smooth_trace.region_attempted;
    }
    if (!ve.m_is_on_offset || ve.m_is_on_input) {
        ++m_smooth_trace.interior_attempted;
        return TriOptimizerMesh::smooth_after(t);
    }

    ++m_smooth_trace.offset_attempted;
    const double before = band_vertex_residual(vid);
    const bool ok = TriOptimizerMesh::smooth_after(t);
    const double after = band_vertex_residual(vid);
    if (ok) ++m_smooth_trace.offset_accepted;

    // NOTE the residuals are read BEFORE the caller's rollback, so `after` is the position the
    // smoother proposed, not necessarily the one kept. That is deliberate -- it separates "the
    // solver could not find a better place" from "it found one and the accept checks refused
    // it" -- but it means the two numbers must always be read next to the accepted count.
    const auto nano = [](double x) {
        return static_cast<long long>(std::min(x, 1e9) * 1e9);
    };
    m_smooth_trace.res_before_nano += nano(before);
    m_smooth_trace.res_after_nano += nano(after);
    atomic_max(m_smooth_trace.res_max_before_nano, nano(before));
    atomic_max(m_smooth_trace.res_max_after_nano, nano(after));
    return ok;
}

void TopoOffsetTriMesh::log_smooth_trace() const
{
    const auto& s = m_smooth_trace;
    logger().info(
        "\tsmooth trace: attempted {} | before: bbox {}, unrounded {} | reached the smoother: "
        "{} on the offset boundary, {} elsewhere ({} of them on another region boundary) | ({})",
        s.attempted.load(),
        s.before_bbox.load(),
        s.before_unrounded.load(),
        s.offset_attempted.load(),
        s.interior_attempted.load(),
        s.region_attempted.load(),
        m_smooth_rejects.to_string());
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
    // distance_deviation_split(), over the Phi residual instead of the Euclidean error. Same
    // reachable/pinned rule, for the same reason: a vertex ON the input complex sits where Phi
    // diverges and no operation can place it on the level set.
    const std::vector<bool> on_band = band_vertex_mask();

    DistanceSplit s;
    double sum_reachable = 0.;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid]) continue;
        const double err = band_vertex_residual(vid);
        if (band_vertex_is_reachable(vid)) {
            s.max_reachable = std::max(s.max_reachable, err);
            sum_reachable += err;
            ++s.n_reachable;
        } else {
            s.max_pinned = std::max(s.max_pinned, err);
            ++s.n_pinned;
        }
    }
    // ... and the same measurement ALONG the band, which is what stops a boundary whose
    // vertices sit on the level set but whose edges cut across it from reading as converged.
    for (const Tuple& e : get_edges()) {
        if (!edge_is_offset_surface_live(e)) continue;
        const EdgeSamples es = offset_edge_samples(e);
        if (es.n == 0) continue; // no samples asked for, or an unreachable endpoint
        s.max_reachable = std::max(s.max_reachable, es.max);
        sum_reachable += es.sum;
        s.n_reachable += es.n;
    }

    s.avg_reachable = (s.n_reachable > 0) ? sum_reachable / s.n_reachable : 0.;
    return s;
}

void TopoOffsetTriMesh::check_offset_within_support(const char* when) const
{
    const std::vector<bool> on_band = band_vertex_mask();

    size_t n_out = 0, worst_vid = static_cast<size_t>(-1);
    double worst_d = 0.;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid] || !band_vertex_is_reachable(vid)) continue;
        if (m_offset_potential->within_support(m_vertex_attribute[vid].m_posf)) continue;
        ++n_out;
        const double d = m_input_complex_bvh.dist(VectorXd(m_vertex_attribute[vid].m_posf));
        if (d > worst_d) {
            worst_d = d;
            worst_vid = vid;
        }
    }
    if (n_out == 0) return;

    log_and_throw_error(
        "{}: {} offset-boundary vertices have left the smooth offset potential's support "
        "(dhat = {} = offset_dhat_factor x target_distance {}). The worst is vertex {} at "
        "Euclidean distance {} from the input complex, which is {:.2f}x target_distance. Out "
        "there Phi is identically zero WITH a zero gradient: the smoothing term gives those "
        "vertices no direction back, their residual saturates instead of growing, and the "
        "sizing field refines around vertices nothing can move. Raise offset_dhat_factor if "
        "the offset legitimately has to travel that far, or reduce target_distance.",
        when,
        n_out,
        m_offset_potential->dhat(),
        m_offset_params.target_distance,
        worst_vid,
        worst_d,
        worst_d / std::max(m_offset_params.target_distance, 1e-16));
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

TopoOffsetTriMesh::Criteria TopoOffsetTriMesh::optimization_criteria()
{
    Criteria c;
    for (const Tuple& f : get_faces()) {
        c.amips = std::max(c.amips, quality_rel(f.fid(*this)));
    }
    c.phi = residual_split().max_reachable / offset_residual_tolerance();
    return c;
}

void TopoOffsetTriMesh::optimization_iteration_begin()
{
    label_offset_boundary();

    // The three criteria, separately, once per iteration. optimization_quality_stats() returns
    // only their max, so without this the log says the loop is stuck without saying on what --
    // and the two behave completely differently: AMIPS is the mesh's own business, while the
    // offset residual is driven by the smoothing term.
    //
    // Captured as well as logged: this is the end of the previous iteration, which is the
    // interval optimization_stalled() has to measure over.
    const Criteria c = optimization_criteria();
    m_prev_criteria = c;

    // THE RUNAWAY GUARD, before anything reports a number derived from Phi. A vertex outside the
    // support has a residual that saturates rather than growing, so every line below would
    // under-report it, and no smoothing move can bring it back.
    check_offset_within_support("Optimization iteration");

    const DistanceSplit r = residual_split();
    const DistanceSplit d = distance_deviation_split();
    logger().info(
        "[criteria] amips {:.4}x (max {:.6} / {:.6}) | phi {:.4}x (max residual {:.6} / {:.6}, "
        "{} pinned) | euclid dist err {:.6} (avg {:.6}) | #V {} #F {} (band {} samples)",
        c.amips,
        c.amips * m_params.stop_energy,
        m_params.stop_energy,
        c.phi,
        r.max_reachable,
        offset_residual_tolerance(),
        r.n_pinned,
        d.max_reachable,
        d.avg_reachable,
        get_vertices().size(),
        get_faces().size(),
        r.n_reachable + r.n_pinned);
}

bool TopoOffsetTriMesh::optimization_stalled(double, double)
{
    const Criteria cur = optimization_criteria();
    const Criteria& prev = m_prev_criteria;

    // The base's inequality, per criterion. Both are normalized to a target of 1.0, so it is
    // the identical formula.
    const double eps = m_params.stuck_refine_stall_eps;
    const auto stuck = [eps](double p, double c) { return (p - c) <= eps * (c - 1.0); };

    struct Term
    {
        const char* name;
        double p, c;
    };
    const Term terms[2] = {
        {"amips", prev.amips, cur.amips},
        {"phi", prev.phi, cur.phi},
    };

    bool all_stuck = true;
    std::string why;
    for (const Term& t : terms) {
        if (t.c <= 1.0) continue; // already met: neither a reason to refine nor to hold off
        const bool s = stuck(t.p, t.c);
        all_stuck = all_stuck && s;
        why += fmt::format(
            "{}{} {:.4}->{:.4} {}",
            why.empty() ? "" : ", ",
            t.name,
            t.p,
            t.c,
            s ? "STUCK" : "moving");
    }
    if (why.empty()) return false; // nothing unmet; the driver's own guard should have caught it

    logger().info("[stall] {} => {}", why, all_stuck ? "refine" : "no refine");
    return all_stuck;
}

void TopoOffsetTriMesh::optimization_debug_checkpoint()
{
    consolidate_mesh();
    TriOptimizerMesh::optimization_debug_checkpoint();
}

std::tuple<double, double> TopoOffsetTriMesh::optimization_quality_stats()
{
    // The max of the two criteria, which is what the driver stops on. TriWild's own criterion
    // is the AMIPS term, via the base's quality_rel(), so this is exactly the number TriWild's
    // loop stops on with the offset residual maxed in beside it.
    //
    // The residual counts only the REACHABLE band: a pinned vertex sits on the input complex,
    // where Phi diverges and no operation can put it on the level set, so leaving it in would
    // hold the metric above the bar forever -- the loop would never stop and the stall detector
    // would fire every iteration, refining around vertices nothing can help.
    const double max_metric = optimization_criteria().max();

    // The companion average, on the same 1.0 scale so it stays readable next to the max.
    // Logged only; nothing reads it.
    double sum_q = 0.;
    size_t n_faces = 0;
    for (const Tuple& f : get_faces()) {
        sum_q += quality_rel(f.fid(*this));
        ++n_faces;
    }
    double avg_metric = (n_faces > 0) ? sum_q / n_faces : 0.;
    avg_metric = std::max(avg_metric, residual_split().avg_reachable / offset_residual_tolerance());

    return {max_metric, avg_metric};
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
    logger().info(
        "\t  flags: on_offset {} on_input {} on_region {} on_bbox {} rounded {} | incident edges: "
        "{} offset, {} region, {} input, {} bbox | phi {:.6} (level {:.6}), residual {:.6}, "
        "envelope {}",
        ve.m_is_on_offset,
        ve.m_is_on_input,
        ve.m_is_on_region,
        !m_vertex_attribute[vid].on_bbox_faces.empty(),
        m_vertex_attribute[vid].m_is_rounded,
        n_offset_e,
        n_region_e,
        n_input_e,
        n_bbox_e,
        m_offset_potential->value(p),
        m_offset_potential->target_level(),
        m_offset_potential->residual_length(p),
        smoothing_envelope(vid) ? "yes" : "none");
    // Which objective the smoother would give it, and whether it is refused before reaching one.
    const char* fate = "shared smoother, AMIPS only -- the offset term is NOT what moves it";
    if (!m_vertex_attribute[vid].on_bbox_faces.empty()) {
        fate = "REFUSED by smooth_before: on the bounding box";
    } else if (!m_vertex_attribute[vid].m_is_rounded) {
        fate = "REFUSED by smooth_before: not rounded";
    } else if (smoothing_extra_energy(vid)) {
        fate = "shared smoother, AMIPS + the offset term";
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

void TopoOffsetTriMesh::optimize_offset(const std::filesystem::path& output_file)
{
    logger().info("Optimizing offset (2D)...");
    logger().info(
        "\ttarget_distance: {} | phi residual tolerance: {} ({}x target_distance) | level c: {} "
        "| dhat: {}",
        m_offset_params.target_distance,
        offset_residual_tolerance(),
        m_offset_params.offset_residual_rel,
        m_offset_potential->target_level(),
        m_offset_potential->dhat());
    optimization_metrics.clear();
    op_counts.clear();

    logger().info("\tLabelling tracked surfaces...");
    label_offset_boundary();

    // The band as CONSTRUCTED must already be inside the potential's support, or nothing the
    // optimization does can move it. Checked before any operation runs so that a construction
    // defect is reported as one.
    check_offset_within_support("Offset as constructed");

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
    // The offset plugs into mesh_improvement() through the same virtuals simwild uses:
    //   - optimization_quality_stats(): the max of AMIPS and the offset residual, each over its
    //     own target;
    //   - optimization_stop_metric(): 1.0, since both are normalized;
    //   - refine_sizing_around_worst(): TriWild's, fired only on a stall.
    // Placement is no longer among them: the offset boundary is placed by the SHARED SMOOTHER,
    // through smoothing_extra_energy(), so it takes the same line search, inversion test and
    // quality veto as every other vertex. The engine consolidates every iteration and
    // re-collects the operation queue, which is also its own answer to slot-pool exhaustion:
    // work dropped in one pass is retried in the next.
    //
    // optimization_iteration_begin() re-derives the tracked surfaces each iteration. The shared
    // operations maintain the edge tags they are given, but a split creates edges the labelling
    // never classified, and the collapse's substructure link condition is only as good as the
    // substructure it is shown.
    iter_cnt_split = 0;
    iter_cnt_collapse = 0;
    iter_cnt_swap = 0;
    m_smooth_trace.reset();

    mesh_improvement(m_offset_params.max_iterations);

    // Cumulative over the whole run, not per iteration as before: the engine loop has no
    // per-iteration hook for reporting, and the per-pass numbers it logs itself carry the
    // history. Matches 3D.
    log_smooth_trace();
    op_counts.push_back({{iter_cnt_split.load(), iter_cnt_collapse.load(), iter_cnt_swap.load()}});
    logger().info(
        "splits = {} (offset-edge: {} offered -> {} accepted)  |  collapses = {} ({} refused by "
        "the offset criterion)  |  swaps = {} ({} refused by the offset criterion)",
        iter_cnt_split.load(),
        iter_cnt_split_offset_before.load(),
        iter_cnt_split_offset.load(),
        iter_cnt_collapse.load(),
        iter_cnt_collapse_offset_reject.load(),
        iter_cnt_swap.load(),
        iter_cnt_swap_offset_reject.load());

    bool converged = false;
    {
        // The quantity the loop converged on, and the Euclidean one beside it. The two agree
        // wherever a single primitive of the input is within reach and differ at reentrant
        // features, where Phi sums the contributions and the level set bulges outward -- so the
        // Euclidean number is the running measure of how far the smoothed offset ended up from
        // the exact one. See OffsetPotential, and tests/test_offset_potential.cpp for the
        // deviation measured on shapes whose exact offset is known.
        const DistanceSplit r = residual_split();
        const auto [max_dist, avg_dist] = compute_distance_deviation();
        logger().info(
            "max phi residual: {} | avg phi residual: {} || max euclidean dist err: {} | avg: {}",
            r.max_reachable,
            r.avg_reachable,
            max_dist,
            avg_dist);
        optimization_metrics.push_back({{max_dist, avg_dist, r.max_reachable, r.avg_reachable}});
        log_worst_dist_vertex();

        // THE VERDICT IS ON THE REACHABLE BAND, which is what the optimization controls. A
        // pinned vertex -- one on the input complex, or on the domain boundary where growth ran
        // out of room -- cannot be placed on the level set by any operation, so failing the run
        // on it would be reporting a construction defect as an optimization failure. It is
        // warned about separately, right below, and compute_distance_deviation() above still
        // reports the whole band so the defect is never hidden.
        converged = r.max_reachable <= offset_residual_tolerance();
        if (converged) {
            logger().info(
                "Converged ([max phi residual] {} <= {} [offset_residual_rel x "
                "target_distance]); euclidean distance error at that point: max {}, avg {}",
                r.max_reachable,
                offset_residual_tolerance(),
                max_dist,
                avg_dist);
        }

        // A pinned vertex outside the target band is not something the optimizer can fix, so it
        // is reported here rather than failing the run -- but it IS a defect in the offset, so
        // it is never silent.
        if (r.max_pinned > offset_residual_tolerance()) {
            logger().warn(
                "{} of {} band vertices are PINNED and {} of them sit outside the target band "
                "(worst residual {} against {}). A pinned vertex lies on the input complex, "
                "where Phi diverges and the level set is unreachable by definition, or on the "
                "domain boundary, where conservative growth ran out of room. No split, collapse "
                "or smoothing move can place it on the level set, so this is the offset as "
                "CONSTRUCTED, not something the optimization left undone: the band still touches "
                "the input complex, or the box is too tight for target_distance.",
                r.n_pinned,
                r.n_pinned + r.n_reachable,
                r.n_pinned,
                r.max_pinned,
                offset_residual_tolerance());
        }
    }
    if (!converged && !optimization_metrics.empty()) {
        const DistanceSplit r = residual_split();
        logger().warn(
            "Optimization did not converge ([max phi residual] {} > {} [offset_residual_rel x "
            "target_distance])",
            r.max_reachable,
            offset_residual_tolerance());

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
        // test fires on every run in the table. Measured on the EUCLIDEAN error, which is why
        // that is what is read here.
        const double max_dist = optimization_metrics.back()[0];
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
    // above, so the log still names by how much the criterion missed before the throw.
    if (!converged && m_offset_params.throw_on_nonconvergence) {
        log_and_throw_error(
            "Optimization did not converge and throw_on_nonconvergence is set. Ran {} of {} "
            "iterations; see the warnings above.",
            m_iterations_used,
            m_offset_params.max_iterations);
    }

    // Deliberately NOT re-derived here. From the moment the tracked surfaces are tagged, it is
    // the shared operations that maintain those tags -- the face LABELS they were derived from
    // are construction data the optimization does not propagate, so re-deriving now would read
    // stale labels and mislabel the result.
}

} // namespace wmtk::components::topological_offset
