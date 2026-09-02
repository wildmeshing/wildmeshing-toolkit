#include "TopoOffsetTetMesh.h"

#include <set>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/ParallelCollect.hpp>

namespace wmtk::components::topological_offset {


//// TetMesh splitting

bool TopoOffsetTetMesh::split_edge_before(const Tuple& t)
{
    // The optimization phase runs wmtk::TetOptimizerMesh's split; everything else here is the
    // marching-tets machinery, which places the new vertex on the offset's distance field and
    // carries per-simplex labels the shared engine knows nothing about.
    if (m_edge_split_mode == EdgeSplitMode::Optimization) {
        const bool dbg_on_offset = is_edge_on_offset(t);
        if (dbg_on_offset) ++iter_cnt_split_offset_before;
        // Nothing is frozen against splits: refining a surface is not moving it, so the midpoint
        // is checked against its tags' boundary envelopes like any other tracked geometry -- the
        // input complex and the domain wall alike, the wall being a region boundary like any
        // other. Do not re-add a wall-split refusal; measured worse -- see git history.
        //
        // Only the Optimization mode is guarded at all: the marching path below is how the offset
        // is constructed, and it has to be able to cut through anything.
        if (!TetOptimizerMesh::split_edge_before(t)) {
            if (dbg_on_offset) ++iter_cnt_split_offset_base_reject;
            return false;
        }
        return true;
    }
    return marching_split_edge_before(t);
}

bool TopoOffsetTetMesh::marching_split_edge_before(const Tuple& t)
{
    // load and reset cache
    auto& cache = edge_split_cache.local();
    cache.internal_e.clear();
    cache.external_e.clear();
    cache.link_e.clear();
    cache.split_f.clear();
    cache.internal_f.clear();
    cache.external_f.clear();
    cache.tets.clear();
    cache.changed_faces.clear();

    size_t e_id = t.eid(*this);

    const auto& VA = m_vertex_attribute;

    // vertices
    cache.v1_id = t.vid(*this);
    cache.v2_id = switch_vertex(t).vid(*this);

    cache.is_edge_on_region = is_edge_on_region(t);
    cache.is_edge_on_offset = is_edge_on_offset(t);

    Vector3d p1 = VA[cache.v1_id].m_posf;
    Vector3d p2 = VA[cache.v2_id].m_posf;
    Vector3d p_new;
    // The midpoint is the only construction placement: no target_distance enters construction at
    // all, and carrying the offset surface out to the level set is the optimization phase's job.
    if (m_edge_split_mode == EdgeSplitMode::Midpoint) {
        p_new = (p1 + p2) / 2.0;
    } else {
        log_and_throw_error("Invalid edge split mode.");
    }
    cache.new_v_pos = p_new;
    cache.new_v_extra = VertexExtra();
    cache.new_v_extra.label = m_edge_attribute[e_id].label;
    // On both boundaries only if the whole edge was: the midpoint's mask is the AND.
    cache.new_v_extra.m_boundary_mask =
        m_vertex_extra[cache.v1_id].m_boundary_mask & m_vertex_extra[cache.v2_id].m_boundary_mask;

    // split edge
    cache.split_e = m_edge_attribute[e_id];

    // link edge maps (and collect opp vert ids)
    std::set<size_t> opp_verts;
    auto tets = get_incident_tets_for_edge(t); // Tuples
    const simplex::Edge edge(cache.v1_id, cache.v2_id);
    for (const Tuple& t_inc : tets) {
        const simplex::Tet tet = simplex_from_tet(t_inc);
        const simplex::Edge opp = tet.opposite_edge(edge);
        opp_verts.insert(opp.vertices()[0]);
        opp_verts.insert(opp.vertices()[1]);

        // link edge attribute
        Tuple e1 = tuple_from_edge(opp.vertices());
        cache.link_e[opp] = m_edge_attribute[e1.eid(*this)];

        // face attributes
        FaceSnapshot new_f;
        new_f.extra.label = m_tet_attribute[t_inc.tid(*this)].label;
        cache.internal_f[opp] = new_f;

        auto [_1, global_fid1] =
            tuple_from_face({{opp.vertices()[0], opp.vertices()[1], cache.v1_id}});
        auto [_2, global_fid2] =
            tuple_from_face({{opp.vertices()[0], opp.vertices()[1], cache.v2_id}});
        cache.external_f[std::make_pair(opp, cache.v1_id)] = face_snapshot(global_fid1);
        cache.external_f[std::make_pair(opp, cache.v2_id)] = face_snapshot(global_fid2);

        // tet attribute
        cache.tets[opp] = m_tet_attribute[t_inc.tid(*this)];
    }

    // opp vertex maps
    for (const size_t opp_vid : opp_verts) {
        // edge maps
        auto [_1, global_fid1] = tuple_from_face({{opp_vid, cache.v1_id, cache.v2_id}});
        EdgeAttributes new_eattr;
        new_eattr.label = m_face_extra[global_fid1].label;
        cache.internal_e[opp_vid] = new_eattr;

        size_t glob_eid1 = tuple_from_edge({{cache.v1_id, opp_vid}}).eid(*this);
        size_t glob_eid2 = tuple_from_edge({{cache.v2_id, opp_vid}}).eid(*this);
        simplex::Edge e1(cache.v1_id, opp_vid);
        simplex::Edge e2(cache.v2_id, opp_vid);
        cache.external_e[e1] = m_edge_attribute[glob_eid1];
        cache.external_e[e2] = m_edge_attribute[glob_eid2];

        // face maps
        cache.split_f[opp_vid] = face_snapshot(global_fid1);
        // if any incident face is on the surface, the new vertex is on the surface as well
        if (face_is_region(global_fid1)) {
            cache.new_v_extra.m_is_on_region = true;
        }
    }

    return true;
}


bool TopoOffsetTetMesh::split_edge_after(const Tuple& t)
{
    if (m_edge_split_mode == EdgeSplitMode::Optimization) {
        if (!TetOptimizerMesh::split_edge_after(t)) {
            ++iter_cnt_split_offset_reject; // counts ALL refusals now; see the log line
            return false;
        }
        ++iter_cnt_split;
        // Read from the result, not from a cached flag: the new vertex is on the offset iff
        // split_after_cells() derived it so from the endpoints.
        if (m_vertex_extra[t.vid(*this)].m_is_on_offset) ++iter_cnt_split_offset;
        return true;
    }
    return marching_split_edge_after(t);
}

bool TopoOffsetTetMesh::marching_split_edge_after(const Tuple& t)
{
    if (!TetMesh::split_edge_after(t)) {
        return false;
    } // why do we need this?

    auto& cache = edge_split_cache.local();
    const size_t v_id = t.vid(*this); // new vertex
    const size_t v1_id = cache.v1_id;
    const size_t v2_id = cache.v2_id;

    const std::vector<Tuple> locs = get_one_ring_tets_for_vertex(t);

    /// check inversion & rounding
    set_vertex_position(
        v_id,
        (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2);

    for (const Tuple& tt : locs) {
        if (is_inverted(tt)) {
            return false;
        }
    }

    // vertex attribute
    m_vertex_extra[v_id] = cache.new_v_extra;
    set_vertex_position(v_id, cache.new_v_pos);
    m_vertex_extra[v_id].m_is_on_region = cache.is_edge_on_region;
    m_vertex_attribute[v_id].on_bbox_faces = wmtk::set_intersection(
        m_vertex_attribute[v1_id].on_bbox_faces,
        m_vertex_attribute[v2_id].on_bbox_faces);
    // on_bbox_faces is written after the base set m_is_on_surface, so a midpoint landing on the
    // domain wall can arrive here unflagged. Re-derive it once both halves of
    // vertex_is_on_region() are known: every containment check is gated on this flag.
    if (vertex_is_on_region(v_id)) {
        m_vertex_attribute[v_id].m_is_on_surface = true;
    }

    // split edges attribute
    size_t split_e1_id = tuple_from_edge({{v1_id, v_id}}).eid(*this);
    size_t split_e2_id = tuple_from_edge({{v2_id, v_id}}).eid(*this);
    m_edge_attribute[split_e1_id] = cache.split_e;
    m_edge_attribute[split_e2_id] = cache.split_e;

    // link edge maps
    for (const auto& pair : cache.link_e) { // for every link edge
        auto link_edge = pair.first;

        // tet attributes
        Tuple t1 = tuple_from_vids(link_edge.vertices()[0], link_edge.vertices()[1], v1_id, v_id);
        Tuple t2 = tuple_from_vids(link_edge.vertices()[0], link_edge.vertices()[1], v2_id, v_id);
        m_tet_attribute[t1.tid(*this)] = cache.tets[link_edge];
        m_tet_attribute[t2.tid(*this)] = cache.tets[link_edge];

        // face attributes
        auto [_1, glob_fid1] =
            tuple_from_face({{link_edge.vertices()[0], link_edge.vertices()[1], v_id}});
        restore_face(glob_fid1, cache.internal_f[link_edge]);

        auto [_2, glob_fid2] =
            tuple_from_face({{link_edge.vertices()[0], link_edge.vertices()[1], v1_id}});
        auto [_3, glob_fid3] =
            tuple_from_face({{link_edge.vertices()[0], link_edge.vertices()[1], v2_id}});
        restore_face(glob_fid2, cache.external_f[std::make_pair(link_edge, v1_id)]);
        restore_face(glob_fid3, cache.external_f[std::make_pair(link_edge, v2_id)]);

        // edge attributes
        size_t link_e_glob_id = tuple_from_edge(link_edge.vertices()).eid(*this);
        m_edge_attribute[link_e_glob_id] = pair.second;
    }

    // oppo vertex maps
    for (const auto& pair : cache.internal_e) { // for every oppo vertex
        size_t opp_vid = pair.first;

        // face attributes
        auto [_1, glob_fid1] = tuple_from_face({{opp_vid, v1_id, v_id}});
        auto [_2, glob_fid2] = tuple_from_face({{opp_vid, v2_id, v_id}});
        restore_face(glob_fid1, cache.split_f[opp_vid]);
        restore_face(glob_fid2, cache.split_f[opp_vid]);

        // edge attributes
        size_t glob_eid = tuple_from_edge({{v_id, opp_vid}}).eid(*this);
        size_t glob_eid1 = tuple_from_edge({{v1_id, opp_vid}}).eid(*this);
        size_t glob_eid2 = tuple_from_edge({{v2_id, opp_vid}}).eid(*this);
        m_edge_attribute[glob_eid] = cache.internal_e[opp_vid];
        m_edge_attribute[glob_eid1] = cache.external_e[simplex::Edge(v1_id, opp_vid)];
        m_edge_attribute[glob_eid2] = cache.external_e[simplex::Edge(v2_id, opp_vid)];
    }

    return true;
}


bool TopoOffsetTetMesh::split_face_before(const Tuple& t)
{
    // load and reset cache
    auto& cache = face_split_cache.local();
    cache.existing_e.clear();
    cache.existing_f.clear();
    cache.tets.clear();

    // get split face tags (used a bunch later)
    size_t split_f_id = t.fid(*this);
    cache.splitf_label = m_face_extra[split_f_id].label;

    // new vertex
    cache.v1_id = t.vid(*this);
    cache.v2_id = switch_vertex(t).vid(*this);
    cache.v3_id = switch_vertex(switch_edge(t)).vid(*this);

    // get 1 or 2 vertex(es) opposite to face
    std::vector<size_t> tet_ids;
    tet_ids.push_back(t.tid(*this));
    auto other_tet = switch_tetrahedron(t); // is std::optional<Tuple> object
    if (other_tet) {
        tet_ids.push_back(other_tet.value().tid(*this));
    }
    std::vector<size_t> oppo_vids;
    for (const size_t tet_id : tet_ids) {
        auto tet_vids = oriented_tet_vids(tet_id);
        for (const size_t tet_vid : tet_vids) {
            if ((tet_vid != cache.v1_id) && (tet_vid != cache.v2_id) && (tet_vid != cache.v3_id)) {
                oppo_vids.push_back(tet_vid);
            }
        }
    }
    assert(oppo_vids.size() == tet_ids.size());

    // cache tet attributes
    for (int i = 0; i < tet_ids.size(); i++) {
        cache.tets[oppo_vids[i]] = m_tet_attribute[tet_ids[i]];
    }

    // existing edge maps on split face
    size_t glob_eid1 = tuple_from_edge({{cache.v1_id, cache.v2_id}}).eid(*this);
    cache.existing_e[simplex::Edge(cache.v1_id, cache.v2_id)] = m_edge_attribute[glob_eid1];
    size_t glob_eid2 = tuple_from_edge({{cache.v2_id, cache.v3_id}}).eid(*this);
    cache.existing_e[simplex::Edge(cache.v2_id, cache.v3_id)] = m_edge_attribute[glob_eid2];
    size_t glob_eid3 = tuple_from_edge({{cache.v1_id, cache.v3_id}}).eid(*this);
    cache.existing_e[simplex::Edge(cache.v1_id, cache.v3_id)] = m_edge_attribute[glob_eid3];

    // existing (unmodified) edge and face maps per-tet
    for (const size_t oppo_vid : oppo_vids) {
        // edges
        size_t glob_opp_eid_1 = tuple_from_edge({{cache.v1_id, oppo_vid}}).eid(*this);
        cache.existing_e[simplex::Edge(cache.v1_id, oppo_vid)] = m_edge_attribute[glob_opp_eid_1];
        size_t glob_opp_eid_2 = tuple_from_edge({{cache.v2_id, oppo_vid}}).eid(*this);
        cache.existing_e[simplex::Edge(cache.v2_id, oppo_vid)] = m_edge_attribute[glob_opp_eid_2];
        size_t glob_opp_eid_3 = tuple_from_edge({{cache.v3_id, oppo_vid}}).eid(*this);
        cache.existing_e[simplex::Edge(cache.v3_id, oppo_vid)] = m_edge_attribute[glob_opp_eid_3];

        // faces
        auto [_1, glob_fid1] = tuple_from_face({{cache.v1_id, cache.v2_id, oppo_vid}});
        cache.existing_f[simplex::Face(cache.v1_id, cache.v2_id, oppo_vid)] =
            face_snapshot(glob_fid1);
        auto [_2, glob_fid2] = tuple_from_face({{cache.v2_id, cache.v3_id, oppo_vid}});
        cache.existing_f[simplex::Face(cache.v2_id, cache.v3_id, oppo_vid)] =
            face_snapshot(glob_fid2);
        auto [_3, glob_fid3] = tuple_from_face({{cache.v1_id, cache.v3_id, oppo_vid}});
        cache.existing_f[simplex::Face(cache.v1_id, cache.v3_id, oppo_vid)] =
            face_snapshot(glob_fid3);
    }

    return true;
}


bool TopoOffsetTetMesh::split_face_after(const Tuple& t)
{
    if (!TetMesh::split_face_after(t)) {
        return false;
    } // why do we need this?

    auto& cache = face_split_cache.local();
    // size_t v_id = t.vid(*this);
    size_t v_id = vertex_size() - 1;
    size_t v1_id = cache.v1_id;
    size_t v2_id = cache.v2_id;
    size_t v3_id = cache.v3_id;
    std::array<size_t, 3> splitf_vids = {{v1_id, v2_id, v3_id}};

    // new_vertex
    set_vertex_position(
        v_id,
        (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf +
         m_vertex_attribute[v3_id].m_posf) /
            3);
    m_vertex_extra[v_id].label = cache.splitf_label;
    // Interior to the split face, so on exactly the boundaries the whole face is on: the AND of
    // its corners. Assigned, not OR'd -- the slot may be recycled. No surface flags are derived
    // here and none need to be: split_face() runs only from simplicial_embedding(), during
    // construction, so the surfaces this vertex could be on do not exist yet.
    m_vertex_extra[v_id].m_boundary_mask = m_vertex_extra[v1_id].m_boundary_mask &
                                           m_vertex_extra[v2_id].m_boundary_mask &
                                           m_vertex_extra[v3_id].m_boundary_mask;

    // new edges/faces on split face
    EdgeAttributes splitf_eattr;
    splitf_eattr.label = cache.splitf_label;
    FaceSnapshot splitf_fattr;
    splitf_fattr.extra.label = cache.splitf_label;
    for (int i = 0; i < 3; i++) {
        size_t curr_v1_id = splitf_vids[i];
        size_t curr_v2_id = splitf_vids[(i + 1) % 3];

        size_t glob_eid = tuple_from_edge({{curr_v1_id, v_id}}).eid(*this); // new edge
        m_edge_attribute[glob_eid] = splitf_eattr;

        auto [_, glob_fid] = tuple_from_face({{curr_v1_id, curr_v2_id, v_id}}); // new face
        restore_face(glob_fid, splitf_fattr);
    }

    // existing edges on split face
    size_t glob_eid1 = tuple_from_edge({{v1_id, v2_id}}).eid(*this);
    size_t glob_eid2 = tuple_from_edge({{v2_id, v3_id}}).eid(*this);
    size_t glob_eid3 = tuple_from_edge({{v1_id, v3_id}}).eid(*this);
    m_edge_attribute[glob_eid1] = cache.existing_e[simplex::Edge(v1_id, v2_id)];
    m_edge_attribute[glob_eid2] = cache.existing_e[simplex::Edge(v2_id, v3_id)];
    m_edge_attribute[glob_eid3] = cache.existing_e[simplex::Edge(v1_id, v3_id)];

    // per oppo-vert
    for (const auto& pair : cache.tets) {
        size_t opp_vid = pair.first;

        // new edge
        size_t glob_newe_id = tuple_from_edge({{v_id, opp_vid}}).eid(*this);
        m_edge_attribute[glob_newe_id].label = pair.second.label;

        // every pair of existing split face verts (every split face edge)
        for (int i = 0; i < 3; i++) {
            size_t curr_v1_id = splitf_vids[i];
            size_t curr_v2_id = splitf_vids[(i + 1) % 3];

            // new tet
            Tuple tet = tuple_from_vids(v_id, opp_vid, curr_v1_id, curr_v2_id);
            m_tet_attribute[tet.tid(*this)] = pair.second;

            // new face
            auto [_, glob_fid] = tuple_from_face({{v_id, curr_v1_id, opp_vid}});
            m_face_extra[glob_fid].label = pair.second.label;

            // existing face
            auto [_2, glob_fid2] = tuple_from_face({{opp_vid, curr_v1_id, curr_v2_id}});
            restore_face(
                glob_fid2,
                cache.existing_f[simplex::Face(opp_vid, curr_v1_id, curr_v2_id)]);

            // existing edge
            size_t glob_eid = tuple_from_edge({{curr_v1_id, opp_vid}}).eid(*this);
            m_edge_attribute[glob_eid] = cache.existing_e[simplex::Edge(curr_v1_id, opp_vid)];
        }
    }

    return true;
}


bool TopoOffsetTetMesh::split_tet_before(const Tuple& t)
{
    auto& cache = tet_split_cache.local();
    cache.existing_e.clear();
    cache.existing_f.clear();

    // vertices
    auto vs = oriented_tet_vids(t);
    for (int i = 0; i < 4; i++) { // deep copy just to be safe
        cache.v_ids[i] = vs[i];
    }

    // cache retained edge attributes
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 4; j++) {
            size_t glob_eid = tuple_from_edge({{cache.v_ids[i], cache.v_ids[j]}}).eid(*this);
            cache.existing_e[simplex::Edge(cache.v_ids[i], cache.v_ids[j])] =
                m_edge_attribute[glob_eid];
        }
    }

    // cache retained face attributes
    for (int i = 0; i < 4; i++) {
        size_t v1 = cache.v_ids[i];
        size_t v2 = cache.v_ids[(i + 1) % 4];
        size_t v3 = cache.v_ids[(i + 2) % 4];
        auto [_, glob_fid] = tuple_from_face({{v1, v2, v3}});
        cache.existing_f[simplex::Face(v1, v2, v3)] = face_snapshot(glob_fid);
    }

    // tet attribute
    cache.tet = m_tet_attribute[t.tid(*this)];

    return true;
}


bool TopoOffsetTetMesh::split_tet_after(const Tuple& t)
{
    if (!TetMesh::split_tet_after(t)) {
        return false;
    } // why do we need this?

    auto& cache = tet_split_cache.local();
    int tet_label = cache.tet.label;
    size_t v_id = vertex_size() - 1;

    // new vertex
    set_vertex_position(
        v_id,
        (m_vertex_attribute[cache.v_ids[0]].m_posf + m_vertex_attribute[cache.v_ids[1]].m_posf +
         m_vertex_attribute[cache.v_ids[2]].m_posf + m_vertex_attribute[cache.v_ids[3]].m_posf) /
            4);
    m_vertex_extra[v_id].label = tet_label;
    // Strictly interior to a tet: on no boundary at all. Assigned -- the slot may be recycled.
    // As in split_face_after(), split_tet() runs only during construction, so there are no
    // surface flags to derive; nothing in the optimization creates a tet-interior vertex.
    m_vertex_extra[v_id].m_boundary_mask = 0;

    // iterate over new tets (retained faces, new tets, new edge (opposite tet) )
    for (int i = 0; i < 4; i++) {
        size_t v1 = cache.v_ids[i];
        size_t v2 = cache.v_ids[(i + 1) % 4];
        size_t v3 = cache.v_ids[(i + 2) % 4];

        // new edge (doesn't matter which vertex, will iterate through all 4)
        size_t glob_new_eid = tuple_from_edge({{v_id, v1}}).eid(*this);
        m_edge_attribute[glob_new_eid].label = tet_label;

        // retained face
        auto [_, glob_fid] = tuple_from_face({{v1, v2, v3}});
        restore_face(glob_fid, cache.existing_f[simplex::Face(v1, v2, v3)]);

        // new tet
        size_t t_id = tuple_from_vids(v1, v2, v3, v_id).tid(*this);
        m_tet_attribute[t_id] = cache.tet; // TO VERIFY: this creates deep copy?
    }

    // existing edges and new faces
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 4; j++) {
            size_t v1 = cache.v_ids[i];
            size_t v2 = cache.v_ids[j];

            // existing edge
            size_t glob_eid = tuple_from_edge({{v1, v2}}).eid(*this);
            m_edge_attribute[glob_eid] = cache.existing_e[simplex::Edge(v1, v2)];

            // new face
            auto [_, glob_fid] = tuple_from_face({{v1, v2, v_id}});
            m_face_extra[glob_fid].label = tet_label;
        }
    }

    return true;
}


/**
 * The shared split places the new vertex, keeps quality and shared attributes, and checks envelope
 * containment. These three hooks add what only the offset knows: which region tag each child tet
 * inherits, and which tracked surfaces the new vertex joined.
 */
bool TopoOffsetTetMesh::split_before_cells(const Tuple& edge, const std::vector<Tuple>& parents)
{
    auto& cache = m_opt_split_cache.local();
    cache.tets.clear();
    cache.is_edge_on_region = is_edge_on_region(edge);
    cache.is_edge_on_offset = is_edge_on_offset(edge);

    // Key each parent by the edge OPPOSITE the one being split: that edge survives the split
    // and is shared by exactly the two children of this parent, so it names them afterwards.
    const simplex::Edge e(edge.vid(*this), edge.switch_vertex(*this).vid(*this));
    for (const Tuple& tt : parents) {
        cache.tets[simplex_from_tet(tt).opposite_edge(e)] = m_tet_attribute.at(tt.tid(*this));
    }
    return true;
}

bool TopoOffsetTetMesh::split_after_cells(
    const size_t v1_id,
    const size_t v2_id,
    const size_t v_id,
    const std::vector<Tuple>&)
{
    // The new vertex's offset membership is derived from its endpoints, never from the cache: a
    // vertex placed on an edge lies on whichever tracked surfaces both endpoints lie on. 3D marks
    // m_is_on_offset once in optimize_offset() and has no fallback, unlike 2D which re-derives
    // the whole front from the face labels every iteration.
    m_vertex_extra[v_id].m_is_on_offset =
        m_vertex_extra[v1_id].m_is_on_offset && m_vertex_extra[v2_id].m_is_on_offset;
    // The input complex, the same way: a midpoint is on it only if the whole edge was. Written
    // here because this is the hook that has the endpoints, and because the AND keeps the
    // never-both invariant true across a split -- an edge running from the complex to the offset
    // surface produces a midpoint on neither, which is what it geometrically is.
    m_vertex_extra[v_id].m_is_on_input_complex =
        m_vertex_extra[v1_id].m_is_on_input_complex && m_vertex_extra[v2_id].m_is_on_input_complex;
    // Churn instrumentation, read only by collapse_after_vertex(). Assigned, never OR'd: v_id may
    // be a recycled slot whose previous occupant was born long ago. See m_born_epoch.
    m_vertex_extra[v_id].m_born_epoch = m_op_epoch;
    if (m_op_epoch != 0) ++iter_cnt_split_born;
    if (m_vertex_extra[v1_id].m_is_on_offset && m_vertex_extra[v2_id].m_is_on_offset) {
        ++iter_cnt_split_offset_endpoints;
    }
    // The boundary mask follows the same AND rule: the midpoint is on a tag boundary only if the
    // whole edge was. Assigned, not OR'd -- v_id may be a recycled slot carrying a dead vertex's
    // bits. Runs before the shared split's containment check, which reads the mask through
    // face_mask() on the two child triangles.
    m_vertex_extra[v_id].m_boundary_mask =
        m_vertex_extra[v1_id].m_boundary_mask & m_vertex_extra[v2_id].m_boundary_mask;

    const auto& cache = m_opt_split_cache.local();
    for (const size_t v_end : {v1_id, v2_id}) {
        const simplex::Edge half(v_end, v_id);
        for (const Tuple& tt : get_incident_tets_for_edge(v_end, v_id)) {
            const auto it = cache.tets.find(simplex_from_tet(tt).opposite_edge(half));
            if (it == cache.tets.end()) {
                return false; // no parent to inherit from; refuse rather than mislabel a tet
            }
            // The quality is written by the shared split just after this returns.
            m_tet_attribute[tt.tid(*this)] = it->second;
        }
    }
    return true;
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

void TopoOffsetTetMesh::split_after_vertex(const size_t v_id, const bool is_edge_open_boundary)
{
    const auto& cache = m_opt_split_cache.local();
    // The base has already set m_is_on_surface, which is the union; this says which. The offset
    // half is never rewritten here -- split_after_cells() derived it from the two endpoints, and
    // that is the authority.
    m_vertex_extra[v_id].m_is_on_region = cache.is_edge_on_region;
    if (is_edge_open_boundary) {
        m_vertex_attribute[v_id].m_order = 2;
    }
}

} // namespace wmtk::components::topological_offset
