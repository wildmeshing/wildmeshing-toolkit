#include <wmtk/TriMeshOperation.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/link_condition.h>
using namespace wmtk;


auto TriMeshOperation::vertex_connectivity(TriMesh& m)
    -> wmtk::AttributeCollection<VertexConnectivity>&
{
    return m.m_vertex_connectivity;
}
auto TriMeshOperation::tri_connectivity(TriMesh& m)
    -> wmtk::AttributeCollection<TriangleConnectivity>&
{
    return m.m_tri_connectivity;
}
auto TriMeshOperation::vertex_connectivity(const TriMesh& m)
    -> const wmtk::AttributeCollection<VertexConnectivity>&
{
    return m.m_vertex_connectivity;
}
auto TriMeshOperation::tri_connectivity(const TriMesh& m)
    -> const wmtk::AttributeCollection<TriangleConnectivity>&
{
    return m.m_tri_connectivity;
}
size_t TriMeshOperation::get_next_empty_slot_t(TriMesh& m)
{
    return m.get_next_empty_slot_t();
}
size_t TriMeshOperation::get_next_empty_slot_v(TriMesh& m)
{
    return m.get_next_empty_slot_v();
}


auto TriMeshOperation::operator()(TriMesh& m, const Tuple& t) -> ExecuteReturnData
{
    ExecuteReturnData retdata;
    retdata.success = false;

    m.start_protected_connectivity();
    m.start_protected_attributes();
    if (before(m, t)) {
        retdata = execute(m, t);

        if (retdata.success) {
            if (!(after(m, retdata) && invariants(m, retdata))) {
                retdata.success = false;
            }
        }
    }

    if (retdata.success == false) {
        m.rollback_protected_connectivity();
        m.rollback_protected_attributes();
    }
    m.release_protected_connectivity();
    m.release_protected_attributes();

    return retdata;
}

bool TriMeshOperation::invariants(TriMesh& m, ExecuteReturnData& ret_data)
{
    return m.invariants(ret_data.new_tris);
}

void TriMeshOperation::set_vertex_size(TriMesh& m, size_t v_cnt)
{
    m.current_vert_size = v_cnt;
    auto& vertex_con = vertex_connectivity(m);
    vertex_con.m_attributes.grow_to_at_least(v_cnt);
    vertex_con.shrink_to_fit();
    m.resize_mutex(m.vert_capacity());
}
void TriMeshOperation::set_tri_size(TriMesh& m, size_t t_cnt)
{
    m.current_tri_size = t_cnt;

    auto& tri_con = tri_connectivity(m);
    tri_con.m_attributes.grow_to_at_least(t_cnt);
    tri_con.shrink_to_fit();
}

auto TriMeshSplitEdgeOperation::execute(TriMesh& m, const Tuple& t) -> ExecuteReturnData
{
    ExecuteReturnData ret_data;
    std::vector<Tuple>& new_tris = ret_data.new_tris;
    Tuple& return_tuple = ret_data.tuple;

    auto& vertex_connectivity = this->vertex_connectivity(m);
    auto& tri_connectivity = this->tri_connectivity(m);

    // get local eid for return tuple construction
    auto eid = t.local_eid(m);

    // get the vids
    size_t vid1 = t.vid(m);
    size_t vid2 = m.switch_vertex(t).vid(m);
    size_t fid1 = t.fid(m);
    size_t fid1_vid3 = ((t.switch_vertex(m)).switch_edge(m)).switch_vertex(m).vid(m);

    for (auto vid : tri_connectivity[fid1].m_indices) {
        if ((vid != vid1) && (vid != vid2)) {
            assert(fid1_vid3 == vid);
            break;
        }
    }
    std::optional<size_t> fid2;
    if (t.switch_face(m).has_value()) fid2 = (t.switch_face(m).value()).fid(m);
    std::optional<size_t> fid2_vid3;
    if (fid2.has_value()) {
        for (auto vid : tri_connectivity[fid2.value()].m_indices) {
            if ((vid != vid1) && (vid != vid2)) {
                fid2_vid3 = vid;
                break;
            }
        }
    }

    size_t new_vid = get_next_empty_slot_v(m);
    size_t new_fid1 = get_next_empty_slot_t(m);
    std::optional<size_t> new_fid2;
    if (fid2.has_value()) new_fid2 = get_next_empty_slot_t(m);

    // first work on the vids
    // the old triangles are connected to the vertex of t
    // fid1_vid3
    {
        auto& conn_tris = vertex_connectivity[fid1_vid3].m_conn_tris;
        conn_tris.push_back(new_fid1);
        std::sort(conn_tris.begin(), conn_tris.end());
    }
    // vid2
    vector_erase(vertex_connectivity[vid2].m_conn_tris, fid1);
    vertex_connectivity[vid2].m_conn_tris.push_back(new_fid1);
    if (fid2.has_value()) {
        vector_erase(vertex_connectivity[vid2].m_conn_tris, fid2.value());
        vertex_connectivity[vid2].m_conn_tris.push_back(new_fid2.value());
    }
    std::sort(
        vertex_connectivity[vid2].m_conn_tris.begin(),
        vertex_connectivity[vid2].m_conn_tris.end());
    // fid2_vid3
    if (fid2_vid3.has_value()) {
        vertex_connectivity[fid2_vid3.value()].m_conn_tris.push_back(new_fid2.value());
        std::sort(
            vertex_connectivity[fid2_vid3.value()].m_conn_tris.begin(),
            vertex_connectivity[fid2_vid3.value()].m_conn_tris.end());
    }

    // new_vid
    vertex_connectivity[new_vid].m_is_removed = false;
    vertex_connectivity[new_vid].m_conn_tris.push_back(fid1);
    vertex_connectivity[new_vid].m_conn_tris.push_back(new_fid1);
    if (fid2.has_value()) {
        vertex_connectivity[new_vid].m_conn_tris.push_back(fid2.value());
        vertex_connectivity[new_vid].m_conn_tris.push_back(new_fid2.value());
    }
    std::sort(
        vertex_connectivity[new_vid].m_conn_tris.begin(),
        vertex_connectivity[new_vid].m_conn_tris.end());


    // now the triangles
    // need to update the hash
    // fid1 fid2 update m_indices and hash
    size_t j = tri_connectivity[fid1].find(vid2);
    tri_connectivity[fid1].m_indices[j] = new_vid;
    tri_connectivity[fid1].hash++;
    size_t i = tri_connectivity[fid1].find(vid1);
    size_t k = tri_connectivity[fid1].find(fid1_vid3);
    // new_fid1 m_indices in same order
    tri_connectivity[new_fid1].m_indices[i] = new_vid;
    tri_connectivity[new_fid1].m_indices[j] = vid2;
    tri_connectivity[new_fid1].m_indices[k] = fid1_vid3;
    tri_connectivity[new_fid1].hash++;
    tri_connectivity[new_fid1].m_is_removed = false;
    if (fid2.has_value()) {
        j = tri_connectivity[fid2.value()].find(vid2);
        tri_connectivity[fid2.value()].m_indices[j] = new_vid;
        tri_connectivity[fid2.value()].hash++;
        i = tri_connectivity[fid2.value()].find(vid1);
        k = tri_connectivity[fid2.value()].find(fid2_vid3.value());
        // new_fid1 m_indices in same order
        tri_connectivity[new_fid2.value()].m_indices[i] = new_vid;
        tri_connectivity[new_fid2.value()].m_indices[j] = vid2;
        tri_connectivity[new_fid2.value()].m_indices[k] = fid2_vid3.value();
        tri_connectivity[new_fid2.value()].hash++;
        tri_connectivity[new_fid2.value()].m_is_removed = false;
    }
    // make the new tuple
    size_t new_fid = std::min(fid1, new_fid1);
    if (new_fid2.has_value()) new_fid = std::min(new_fid, new_fid2.value());
    int l = tri_connectivity[new_fid].find(new_vid);
    auto new_vertex = Tuple(new_vid, (l + 2) % 3, new_fid, m);
    return_tuple = Tuple(vid1, eid, fid1, m);
    assert(new_vertex.is_valid(m));
    assert(return_tuple.is_valid(m));

    new_tris = m.get_one_ring_tris_for_vertex(new_vertex);
    ret_data.success = true;
    return ret_data;
}
bool TriMeshSplitEdgeOperation::before(TriMesh& m, const Tuple& t)
{
    return true;
}
bool TriMeshSplitEdgeOperation::after(TriMesh& m, ExecuteReturnData& ret_data)
{
    return true;
}
std::string TriMeshSplitEdgeOperation::name() const
{
    return "edge_split";
}

auto TriMeshSplitEdgeOperation::new_vertex(TriMesh& m, const Tuple& t) const -> Tuple
{
    return t.switch_vertex(m);
}

auto TriMeshSplitEdgeOperation::original_endpoints(TriMesh& m, const Tuple& t) const
    -> std::array<Tuple, 2>
{
    // diamond below encodes vertices with lower alpha
    // edges with num
    // faces with upper alpha
    // (only encodes simplices adjacent to vertex c
    //   a
    //  A1B
    // b2c3d
    //  C4D
    //   e
    //
    // initially e4D
    // switch_vertex -> c4D
    // switch_edge -> c3D
    // switch_face -> c3B
    // switch_edge -> c1B
    // switch_vertex -> a1B
    auto face_opt = t.switch_vertex(m).switch_edge(m).switch_face(m);

    assert(face_opt);

    return {{t, face_opt->switch_edge(m).switch_vertex(m)}};
}


auto TriMeshSwapEdgeOperation::execute(TriMesh& m, const Tuple& t) -> ExecuteReturnData
{
    ExecuteReturnData ret_data;
    std::vector<Tuple>& new_tris = ret_data.new_tris;
    Tuple& return_tuple = ret_data.tuple;

    auto& vertex_connectivity = this->vertex_connectivity(m);
    auto& tri_connectivity = this->tri_connectivity(m);

    // get the vids
    size_t vid1 = t.vid(m);
    size_t vid2 = t.switch_vertex(m).vid(m);

    auto tmp_tuple_opt = m.switch_face(t);
    if (!tmp_tuple_opt.has_value()) {
        return ret_data;
    }
    Tuple tmp_tuple = tmp_tuple_opt.value();
    assert(tmp_tuple.is_valid(m));

    tmp_tuple = tmp_tuple.switch_edge(m);
    size_t vid3 = tmp_tuple.switch_vertex(m).vid(m);
    const Tuple tmp_tuple2 = t.switch_edge(m);
    assert(tmp_tuple2.is_valid(m));
    size_t vid4 = tmp_tuple2.switch_vertex(m).vid(m);
    // check if the triangles intersection is the one adjcent to the edge
    size_t test_fid1 = t.fid(m);
    auto other_face_opt = m.switch_face(t);
    if (!other_face_opt.has_value()) {
        return ret_data; // can't sawp on boundary edge
    }
    assert(other_face_opt.has_value());
    const size_t test_fid2 = other_face_opt.value().fid(m);

    // first work on triangles, there are only 2
    int j = tri_connectivity[test_fid1].find(vid2);
    tri_connectivity[test_fid1].m_indices[j] = vid3;
    tri_connectivity[test_fid1].hash++;

    j = tri_connectivity[test_fid2].find(vid1);
    tri_connectivity[test_fid2].m_indices[j] = vid4;
    tri_connectivity[test_fid2].hash++;

    // then work on the vertices
    vector_erase(vertex_connectivity[vid1].m_conn_tris, test_fid2);
    vector_erase(vertex_connectivity[vid2].m_conn_tris, test_fid1);
    vertex_connectivity[vid3].m_conn_tris.push_back(test_fid1);
    vector_unique(vertex_connectivity[vid3].m_conn_tris);

    vertex_connectivity[vid4].m_conn_tris.push_back(test_fid2);
    vector_unique(vertex_connectivity[vid4].m_conn_tris);
    // change the tuple to the new edge tuple
    return_tuple = m.init_from_edge(vid4, vid3, test_fid2);

    assert(return_tuple.switch_vertex(m).vid(m) != vid1);
    assert(return_tuple.switch_vertex(m).vid(m) != vid2);
    assert(return_tuple.is_valid(m));
    auto new_other_face_opt = return_tuple.switch_face(m);
    if (new_other_face_opt) {
        new_tris = {return_tuple, new_other_face_opt.value()};
    } else {
        return ret_data;
    }

    ret_data.success = true;
    return ret_data;
}
bool TriMeshSwapEdgeOperation::before(TriMesh& mesh, const Tuple& t)
{
    auto other_face_opt = t.switch_face(mesh);
    if (!other_face_opt) {
        return false;
    }
    size_t v4 = ((other_face_opt.value()).switch_edge(mesh)).switch_vertex(mesh).vid(mesh);
    size_t v3 = ((t.switch_edge(mesh)).switch_vertex(mesh)).vid(mesh);
    if (!set_intersection(
             vertex_connectivity(mesh)[v4].m_conn_tris,
             vertex_connectivity(mesh)[v3].m_conn_tris)
             .empty()) {
        return false;
    }
    return true;
}
bool TriMeshSwapEdgeOperation::after(TriMesh& mesh, ExecuteReturnData& ret_data)
{
    return true;
}
std::string TriMeshSwapEdgeOperation::name() const
{
    return "edge_swap";
}


auto TriMeshSmoothVertexOperation::execute(TriMesh& m, const Tuple& t) -> ExecuteReturnData
{
    // always succeed and return the Tuple for the (vertex) that we pointed at
    return {t, m.get_one_ring_tris_for_vertex(t), true};
}
bool TriMeshSmoothVertexOperation::before(TriMesh& m, const Tuple& t)
{
    return true;
}
bool TriMeshSmoothVertexOperation::after(TriMesh& m, ExecuteReturnData& ret_data)
{
    return true;
}
std::string TriMeshSmoothVertexOperation::name() const
{
    return "vertex_smooth";
}

// bool TriMeshSmoothVertexOperation::invariants(TriMesh& m, ExecuteReturnData& ret_data)
// {
//     // todo: mtao: figure out how to incorporate this properly
//     //  our execute should have tuple set to the input tuple (vertex)
//     // return TriMesh::invariants(m.get_one_ring_tris_for_vertex(ret_data.tuple));
//     return m.invariants();
// }


auto TriMeshEdgeCollapseOperation::execute(TriMesh& m, const Tuple& loc0) -> ExecuteReturnData
{
    ExecuteReturnData ret_data;
    std::vector<Tuple>& new_tris = ret_data.new_tris;
    Tuple& return_t = ret_data.tuple;

    auto& vertex_connectivity = this->vertex_connectivity(m);
    auto& tri_connectivity = this->tri_connectivity(m);
    // get fid for the return tuple
    // take the face that shares the same vertex the loc0 tuple is pointing to
    // or if that face doesn't exit
    // take the face that shares the same vertex of loc0
    size_t new_fid;
    {
        std::optional<Tuple> new_tup_opt;
        new_tup_opt = loc0.switch_vertex(m).switch_edge(m).switch_face(m);
        if (!new_tup_opt.has_value()) {
            new_tup_opt = loc0.switch_edge(m).switch_face(m);
            assert(new_tup_opt.has_value());
        }
        new_fid = new_tup_opt.value().fid(m);
    }

    // get the vids
    size_t vid1 = loc0.vid(m);
    size_t vid2 = m.switch_vertex(loc0).vid(m);


    // get the fids
    auto n1_fids = vertex_connectivity[vid1].m_conn_tris;

    auto n2_fids = vertex_connectivity[vid2].m_conn_tris;

    // get the fids that will be modified
    auto n12_intersect_fids = set_intersection(n1_fids, n2_fids);
    // check if the triangles intersection is the one adjcent to the edge
    size_t test_fid1 = loc0.fid(m);
    TriMesh::Tuple loc1 = m.switch_face(loc0).value_or(loc0);
    size_t test_fid2 = loc1.fid(m);
    //"faces at the edge is not correct"
    assert(
        vector_contains(n12_intersect_fids, test_fid1) &&
        vector_contains(n12_intersect_fids, test_fid2));
    // now mark the vertices as removed so the assertion for tuple validity in switch operations
    // won't fail
    vertex_connectivity[vid1].m_is_removed = true;
    vertex_connectivity[vid2].m_is_removed = true;
    for (size_t fid : n12_intersect_fids) {
        tri_connectivity[fid].m_is_removed = true;
    }

    std::vector<size_t> n12_union_fids;
    std::set_union(
        n1_fids.begin(),
        n1_fids.end(),
        n2_fids.begin(),
        n2_fids.end(),
        std::back_inserter(n12_union_fids));

    // record the fids that will be modified/erased for roll back on failure
    vector_unique(n12_union_fids);
    std::vector<std::pair<size_t, TriangleConnectivity>> old_tris(n12_union_fids.size());

    for (const size_t fid : n12_union_fids) {
        tri_connectivity[fid].hash++;
    }
    // modify the triangles
    // the m_conn_tris needs to be sorted
    size_t new_vid = get_next_empty_slot_v(m);
    for (size_t fid : n1_fids) {
        if (tri_connectivity[fid].m_is_removed)
            continue;
        else {
            int j = tri_connectivity[fid].find(vid1);
            tri_connectivity[fid].m_indices[j] = new_vid;
        }
    }
    for (size_t fid : n2_fids) {
        if (tri_connectivity[fid].m_is_removed)
            continue;
        else {
            int j = tri_connectivity[fid].find(vid2);
            tri_connectivity[fid].m_indices[j] = new_vid;
        }
    }

    // now work on vids
    // add in the new vertex

    for (size_t fid : n12_union_fids) {
        if (tri_connectivity[fid].m_is_removed)
            continue;
        else
            vertex_connectivity[new_vid].m_conn_tris.push_back(fid);
    }
    vertex_connectivity[new_vid].m_is_removed = false;
    // This is sorting too, and it is important to sort
    vector_unique(vertex_connectivity[new_vid].m_conn_tris);

    // remove the erased fids from the vertices' (the one of the triangles that is not the end
    // points of the edge) connectivity list
    std::vector<std::pair<size_t, size_t>> same_edge_vid_fid;
    for (size_t fid : n12_intersect_fids) {
        auto f_vids = tri_connectivity[fid].m_indices;
        for (size_t f_vid : f_vids) {
            if (f_vid != vid1 && f_vid != vid2) {
                same_edge_vid_fid.emplace_back(f_vid, fid);
                assert(vector_contains(vertex_connectivity[f_vid].m_conn_tris, fid));
                vector_erase(vertex_connectivity[f_vid].m_conn_tris, fid);
            }
        }
    }

    // ? ? tuples changes. this needs to be done before post check since checked are done on tuples
    // update the old tuple version number
    // create an edge tuple for each changed edge
    // call back check will be done on this vector of tuples

    assert(vertex_connectivity[new_vid].m_conn_tris.size() != 0);

    const size_t gfid = vertex_connectivity[new_vid].m_conn_tris[0];
    int j = tri_connectivity[gfid].find(new_vid);
    auto new_t = Tuple(new_vid, (j + 2) % 3, gfid, m);
    int j_ret = tri_connectivity[new_fid].find(new_vid);
    return_t = Tuple(new_vid, (j_ret + 2) % 3, new_fid, m);
    assert(new_t.is_valid(m));

    new_tris = m.get_one_ring_tris_for_vertex(new_t);

    ret_data.success = true;
    return ret_data;
}

bool TriMeshEdgeCollapseOperation::check_link_condition(const TriMesh& mesh, const Tuple& edge)
{
    return utils::link_condition(mesh, edge);
}


bool TriMeshEdgeCollapseOperation::before(TriMesh& m, const Tuple& t)
{
    return check_link_condition(m, t);
}

bool TriMeshEdgeCollapseOperation::after(TriMesh& m, ExecuteReturnData& ret_data)
{
    ret_data.success &= true;
    return ret_data;
}

std::string TriMeshEdgeCollapseOperation::name() const
{
    return "edge_collapse";
}

auto TriMeshConsolidateOperation::execute(TriMesh& m, const Tuple& t) -> ExecuteReturnData
{
    auto& vertex_con = vertex_connectivity(m);
    auto& tri_con = tri_connectivity(m);

    auto v_cnt = 0;
    std::vector<size_t> map_v_ids(m.vert_capacity(), -1);
    for (auto i = 0; i < m.vert_capacity(); i++) {
        if (vertex_con[i].m_is_removed) continue;
        map_v_ids[i] = v_cnt;
        v_cnt++;
    }
    auto t_cnt = 0;
    std::vector<size_t> map_t_ids(m.tri_capacity(), -1);
    for (auto i = 0; i < m.tri_capacity(); i++) {
        if (tri_con[i].m_is_removed) continue;
        map_t_ids[i] = t_cnt;
        t_cnt++;
    }
    v_cnt = 0;
    for (auto i = 0; i < m.vert_capacity(); i++) {
        if (vertex_con[i].m_is_removed) continue;
        if (v_cnt != i) {
            assert(v_cnt < i);
            vertex_con[v_cnt] = vertex_con[i];
            if (m.p_vertex_attrs) m.p_vertex_attrs->move(i, v_cnt);
        }
        for (size_t& t_id : vertex_con[v_cnt].m_conn_tris) {
            t_id = map_t_ids[t_id];
        }
        v_cnt++;
    }
    t_cnt = 0;
    for (int i = 0; i < m.tri_capacity(); i++) {
        if (tri_con[i].m_is_removed) continue;

        if (t_cnt != i) {
            assert(t_cnt < i);
            tri_con[t_cnt] = tri_con[i];
            tri_con[t_cnt].hash = 0;
            if (m.p_face_attrs) {
                m.p_face_attrs->move(i, t_cnt);
            }

            for (auto j = 0; j < 3; j++) {
                if (m.p_edge_attrs) {
                    m.p_edge_attrs->move(i * 3 + j, t_cnt * 3 + j);
                }
            }
        }
        for (size_t& v_id : tri_con[t_cnt].m_indices) {
            v_id = map_v_ids[v_id];
        }
        t_cnt++;
    }

    set_vertex_size(m, v_cnt);
    set_tri_size(m, t_cnt);

    // Resize user class attributes
    if (m.p_vertex_attrs) m.p_vertex_attrs->grow_to_at_least(m.vert_capacity());
    if (m.p_edge_attrs) m.p_edge_attrs->grow_to_at_least(m.tri_capacity() * 3);
    if (m.p_face_attrs) m.p_face_attrs->grow_to_at_least(m.tri_capacity());

    assert(m.check_edge_manifold());
    assert(m.check_mesh_connectivity_validity());
    ExecuteReturnData ret;
    ret.success = true;
    return ret;
}
bool TriMeshConsolidateOperation::before(TriMesh& m, const Tuple& t)
{
    return true;
}
bool TriMeshConsolidateOperation::after(TriMesh& m, ExecuteReturnData& ret_data)
{
    ret_data.success &= true;
    return ret_data;
}
std::string TriMeshConsolidateOperation::name() const
{
    return "consolidate";
}
