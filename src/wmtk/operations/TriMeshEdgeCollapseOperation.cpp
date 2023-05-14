#include <wmtk/operations/TriMeshEdgeCollapseOperation.h>
namespace wmtk {

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

    assign(new_t);
    new_tris = modified_tuples(m);

    ret_data.success = true;
    return ret_data;
}

auto TriMeshEdgeCollapseOperation::modified_tuples(const TriMesh& m) -> std::vector<Tuple>
{
    const auto& new_tup_opt = get_return_tuple_opt();
    assert(new_tup_opt.has_value());
    return m.get_one_ring_tris_for_vertex(new_tup_opt.value());
}
namespace {
constexpr static size_t dummy = std::numeric_limits<size_t>::max();
}

auto TriMeshEdgeCollapseOperation::links_of_vertex(const TriMesh& mesh, const Tuple& vertex)
    -> LinksOfVertex
{
    size_t vid = vertex.vid(mesh);
    auto vid_ring = mesh.get_one_ring_edges_for_vertex(vertex);

    LinksOfVertex ret;
    std::vector<size_t>& lk_vid = ret.vertex;
    std::vector<std::pair<size_t, size_t>>& lk_e_vid = ret.edge;

    for (const auto& e_vid : vid_ring) {
        if (!e_vid.switch_face(mesh).has_value()) {
            lk_vid.push_back(dummy);
            lk_e_vid.emplace_back(e_vid.vid(mesh), dummy);
        }
        lk_vid.push_back(e_vid.vid(mesh));
    }
    std::vector<Tuple> vid_tris = mesh.get_one_ring_tris_for_vertex(vertex);
    for (const auto& v_tri_t : vid_tris) {
        const size_t fid = v_tri_t.fid(mesh);
        const auto& tri_con = tri_connectivity(mesh)[fid];
        const auto& indices = tri_con.m_indices;
        auto l = tri_con.find(vid);
        assert(l != -1);
        auto i0 = indices[(l + 1) % 3], i1 = indices[(l + 2) % 3];
        lk_e_vid.emplace_back(std::min(i0, i1), std::max(i0, i1));
    }
    vector_unique(lk_vid);
    std::sort(lk_e_vid.begin(), lk_e_vid.end());
    return ret;
}
std::vector<size_t> TriMeshEdgeCollapseOperation::edge_link_of_edge(
    const TriMesh& mesh,
    const Tuple& edge)
{
    auto get_opposing_vertex_vid = [&mesh](const Tuple& t) {
        return t.switch_edge(mesh).switch_vertex(mesh).vid(mesh);
    };
    std::vector<size_t> lk_edge;
    lk_edge.push_back(get_opposing_vertex_vid(edge));
    const std::optional<Tuple> other_face_opt = edge.switch_face(mesh);
    if (!other_face_opt.has_value()) {
        lk_edge.push_back(dummy);
    } else {
        lk_edge.push_back(get_opposing_vertex_vid(other_face_opt.value()));
    }
    vector_sort(lk_edge);
    return lk_edge;
}

bool TriMeshEdgeCollapseOperation::check_link_condition(const TriMesh& mesh, const Tuple& edge)
{
    assert(edge.is_valid(mesh));
    // the edge initially points at the first of two vertex links we are computing
    const LinksOfVertex v1 = links_of_vertex(mesh, edge);
    const LinksOfVertex v2 = links_of_vertex(mesh, edge.switch_vertex(mesh));

    // compute vertex link condition
    auto lk_vid12 = set_intersection(v1.vertex, v2.vertex);
    bool v_link = lk_vid12 == edge_link_of_edge(mesh, edge);

    // check edge link condition
    // in 2d edge link for an edge is always empty

    std::vector<std::pair<size_t, size_t>> res;
    const auto& lk_e_vid1 = v1.edge;
    const auto& lk_e_vid2 = v2.edge;
    std::set_intersection(
        lk_e_vid1.begin(),
        lk_e_vid1.end(),
        lk_e_vid2.begin(),
        lk_e_vid2.end(),
        std::back_inserter(res));
    const bool e_link = res.empty();
    return v_link && e_link;
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
} // namespace wmtk
