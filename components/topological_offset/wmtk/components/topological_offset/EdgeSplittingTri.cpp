#include <set>
#include "TopoOffsetTriMesh.h"


namespace wmtk::components::topological_offset {

//// TriMesh splitting

bool TopoOffsetTriMesh::split_edge_before(const Tuple& t)
{
    if (m_edge_split_mode == EdgeSplitMode::Optimization && edge_is_offset_surface_live(t)) {
        ++iter_cnt_split_offset_before;
    }
    // Cleared for both modes: split_after_vertex() reads emptiness to tell which mode produced
    // the split, and the marching path sets its own labels. Entries left from a previous
    // optimization split would be stamped onto marching-created faces.
    m_opt_split_cache.local().face_label.clear();

    // The optimization phase runs wmtk::TriOptimizerMesh's split; everything below is the
    // marching-triangles machinery, which places the new vertex on the offset's distance field
    // and carries per-simplex labels the shared engine knows nothing about.
    if (m_edge_split_mode == EdgeSplitMode::Optimization) {
        // No edge class is refused here, the domain wall included: a wall edge is a tracked
        // region boundary like any other, so the envelopes hold it. Do not re-add a wall
        // refusal; measured worse -- see git history of this file.

        // The shared split propagates FaceAttributes but not the construction label, and
        // offset_is_manifold() is built from that label: without this the new faces default to
        // label 0 and the offset region develops holes. Keyed by the apex -- the vertex opposite
        // the split edge -- which names the parent unambiguously from a child.
        auto& c = m_opt_split_cache.local();
        c.v1_id = t.vid(*this);
        c.v2_id = t.switch_vertex(*this).vid(*this);
        // Captured here, while both endpoints are in hand, and propagated as the endpoints' AND
        // -- never recomputed from the incident faces, whose tags execute_offset() replaces as
        // the band grows. Boundary membership is a property of the input partition, not of the
        // current tags. split_after_vertex() gates these bits on the edge's own persistent
        // class, so a chord's midpoint never picks them up.
        c.edge_bits =
            m_vertex_extra[c.v1_id].m_boundary_mask & m_vertex_extra[c.v2_id].m_boundary_mask;
        const simplex::Edge edge(c.v1_id, c.v2_id);
        // parent_q_max is diagnostic: split_after_vertex() uses it to say whether a needle child
        // came from a parent that was already unscoreable, or from a healthy one.
        c.parent_q_max = -1.;
        c.parent_flatness = 1.;
        for (const size_t fid : get_incident_fids_for_edge(t)) {
            const size_t apex = simplex_from_face(fid).opposite_vertex(edge).id();
            c.face_label[apex] = m_face_extra[fid].label;
            c.parent_q_max = std::max(c.parent_q_max, get_quality(fid));
            c.parent_flatness = std::min(c.parent_flatness, face_flatness(fid));
        }
        return TriOptimizerMesh::split_edge_before(t);
    }
    return marching_split_edge_before(t);
}

bool TopoOffsetTriMesh::marching_split_edge_before(const Tuple& t)
{
    // load and clear cache
    auto& cache = edge_split_cache.local();
    cache.existing_eattr.clear();
    cache.opp_v_fattr.clear();

    size_t e_id = t.eid(*this);

    // new vertex
    cache.v1_id = t.vid(*this);
    cache.v2_id = t.switch_vertex(*this).vid(*this);
    Vector2d p1 = m_vertex_attribute[cache.v1_id].m_posf;
    Vector2d p2 = m_vertex_attribute[cache.v2_id].m_posf;
    Vector2d p_new;
    // The midpoint is the only construction placement: no target_distance enters construction at
    // all, and carrying the front out to the level set is entirely the optimization phase's job.
    if (m_edge_split_mode == EdgeSplitMode::Midpoint) {
        p_new = (p1 + p2) / 2.0;
    } else {
        log_and_throw_error("Invalid edge split mode.");
    }
    cache.new_v_pos = p_new;
    cache.new_v_extra = VertexExtra2d();
    cache.new_v_extra.label = m_edge_extra[e_id].label;
    // The flag is the edge's own class, not an AND of the endpoints: two vertices sharing a
    // region can be joined by a chord through the interior, and marching splits exactly such
    // chords. Behind that gate the bits are the endpoints' AND, propagated, never recomputed from
    // the incident faces, whose tags execute_offset() replaces as the band grows.
    cache.new_v_extra.m_is_on_region = edge_is_region(e_id);
    cache.new_v_extra.m_boundary_mask = cache.new_v_extra.m_is_on_region
                                            ? (m_vertex_extra[cache.v1_id].m_boundary_mask &
                                               m_vertex_extra[cache.v2_id].m_boundary_mask)
                                            : uint64_t(0);

    // split edge attribute
    cache.split_eattr = edge_snapshot(e_id);

    // per-opp vert attributes
    std::vector<size_t> opp_v_ids;
    opp_v_ids.push_back(t.switch_edge(*this).switch_vertex(*this).vid(*this));
    auto other = t.switch_face(*this);
    if (other) {
        opp_v_ids.push_back(other.value().switch_edge(*this).switch_vertex(*this).vid(*this));
    }
    for (const size_t opp_v_id : opp_v_ids) {
        Tuple ftup = tuple_from_simplex(simplex::Face(opp_v_id, cache.v1_id, cache.v2_id));
        size_t f_id = ftup.fid(*this);

        simplex::Edge e1(cache.v1_id, opp_v_id);
        size_t e1_id = tuple_from_edge(cache.v1_id, opp_v_id, f_id).eid(*this);
        simplex::Edge e2(cache.v2_id, opp_v_id);
        size_t e2_id = tuple_from_edge(cache.v2_id, opp_v_id, f_id).eid(*this);

        cache.existing_eattr[e1] = edge_snapshot(e1_id);
        cache.existing_eattr[e2] = edge_snapshot(e2_id);
        cache.opp_v_fattr[opp_v_id] = face_snapshot(f_id);
    }

    return true;
}


bool TopoOffsetTriMesh::split_edge_after(const Tuple& t)
{
    if (m_edge_split_mode == EdgeSplitMode::Optimization) {
        if (!TriOptimizerMesh::split_edge_after(t)) {
            return false;
        }
        // The labels of the faces this split created were carried from their parents by
        // split_after_vertex(), which the base calls above. Never re-derive them from the tags:
        // a band filled with a tag used elsewhere would relabel that other region as offset.
        ++iter_cnt_split;
        if (m_vertex_extra[t.vid(*this)].m_is_on_offset) ++iter_cnt_split_offset;
        return true;
    }
    return marching_split_edge_after(t);
}

bool TopoOffsetTriMesh::marching_split_edge_after(const Tuple& t)
{
    if (!TriMesh::split_edge_after(t)) {
        return false;
    } // why do we need this?

    auto& cache = edge_split_cache.local();
    size_t v_id = get_vertices().size() - 1;
    // std::vector<size_t> opp_vids;
    // for (const auto& pair : cache.opp_v_fattr) {
    //     opp_vids.push_back(pair.first);
    // }
    // size_t v_id = edge_split_get_new_vid(cache.v1_id, cache.v2_id, opp_vids);
    m_vertex_extra[v_id] = cache.new_v_extra;
    set_vertex_position(v_id, cache.new_v_pos);

    // split edge attributes
    for (const auto& pair : cache.opp_v_fattr) {
        size_t opp_v_id = pair.first;

        Tuple ftup_1 = tuple_from_simplex(simplex::Face(cache.v1_id, v_id, opp_v_id));
        size_t f1_id = ftup_1.fid(*this);
        restore_edge(tuple_from_edge(cache.v1_id, v_id, f1_id).eid(*this), cache.split_eattr);

        Tuple ftup_2 = tuple_from_simplex(simplex::Face(cache.v2_id, v_id, opp_v_id));
        size_t f2_id = ftup_2.fid(*this);
        restore_edge(tuple_from_edge(cache.v2_id, v_id, f2_id).eid(*this), cache.split_eattr);

        break;
    }

    // per existing edge attributes
    for (const auto& pair : cache.existing_eattr) {
        size_t e_id = edge_id_from_simplex(pair.first);
        restore_edge(e_id, pair.second);
    }

    // new edges and faces
    for (const auto& pair : cache.opp_v_fattr) {
        size_t opp_v_id = pair.first;
        const FaceSnapshot2d& f_attr = pair.second;

        size_t f1_id = tuple_from_simplex(simplex::Face(cache.v1_id, v_id, opp_v_id)).fid(*this);
        restore_face(f1_id, f_attr);
        size_t f2_id = tuple_from_simplex(simplex::Face(cache.v2_id, v_id, opp_v_id)).fid(*this);
        restore_face(f2_id, f_attr);
        size_t new_e_id = edge_id_from_simplex(simplex::Edge(opp_v_id, v_id));
        // Brand-new edge on a possibly recycled slot: reset both records before writing what is
        // meant. A slot freed by a dead region edge otherwise keeps its m_is_surface_fs and
        // class, and the cross edge is born a phantom region boundary -- tracked, contained by
        // nothing, and poisoning m_is_on_region on every vertex a later split of it creates.
        m_edge_attribute[new_e_id].reset();
        m_edge_extra[new_e_id] = EdgeExtra2d();
        m_edge_extra[new_e_id].label = f_attr.extra.label;
    }

    return true;
}


bool TopoOffsetTriMesh::split_face_before(const Tuple& t)
{
    auto& cache = face_split_cache.local();
    cache.existing_eattr.clear();

    // face id, retain attribute
    size_t f_id = t.fid(*this);
    cache.split_fattr = face_snapshot(f_id);

    // vertices (new vertex attributes too)
    cache.v1_id = t.vid(*this);
    cache.v2_id = t.switch_vertex(*this).vid(*this);
    cache.v3_id = t.switch_edge(*this).switch_vertex(*this).vid(*this);
    Vector2d p1 = m_vertex_attribute[cache.v1_id].m_posf;
    Vector2d p2 = m_vertex_attribute[cache.v2_id].m_posf;
    Vector2d p3 = m_vertex_attribute[cache.v3_id].m_posf;
    cache.new_v_pos = (p1 + p2 + p3) / 3;
    cache.new_v_extra = VertexExtra2d();
    cache.new_v_extra.label = cache.split_fattr.extra.label;
    // A face's centroid is interior by construction, so it is on no region boundary and no
    // boundary tube: both defaults are the answer.

    // existing edges
    simplex::Edge e1(cache.v1_id, cache.v2_id);
    simplex::Edge e2(cache.v2_id, cache.v3_id);
    simplex::Edge e3(cache.v1_id, cache.v3_id);
    cache.existing_eattr[e1] = edge_snapshot(edge_id_from_simplex(e1));
    cache.existing_eattr[e2] = edge_snapshot(edge_id_from_simplex(e2));
    cache.existing_eattr[e3] = edge_snapshot(edge_id_from_simplex(e3));

    return true;
}


bool TopoOffsetTriMesh::split_face_after(const Tuple& t)
{
    if (!TriMesh::split_face_after(t)) {
        return false;
    }

    auto& cache = face_split_cache.local();
    size_t v_id = get_vertices().size() - 1;
    m_vertex_extra[v_id] = cache.new_v_extra;
    set_vertex_position(v_id, cache.new_v_pos);

    // existing edges
    for (const auto& pair : cache.existing_eattr) {
        size_t e_id = edge_id_from_simplex(pair.first);
        restore_edge(e_id, pair.second);
    }

    // all internal edges and faces
    std::array<size_t, 3> vs = {{cache.v1_id, cache.v2_id, cache.v3_id}};
    for (int i = 0; i < 3; i++) {
        // edge -- a brand-new spoke on a possibly recycled slot; reset both records before
        // writing the label, same phantom-region hazard as in marching_split_edge_after().
        size_t e_id = edge_id_from_simplex(simplex::Edge(vs[i], v_id));
        m_edge_attribute[e_id].reset();
        m_edge_extra[e_id] = EdgeExtra2d();
        m_edge_extra[e_id].label = cache.split_fattr.extra.label;

        // face
        size_t f_id = tuple_from_simplex(simplex::Face(vs[i], vs[(i + 1) % 3], v_id)).fid(*this);
        restore_face(f_id, cache.split_fattr);
    }

    return true;
}

} // namespace wmtk::components::topological_offset
