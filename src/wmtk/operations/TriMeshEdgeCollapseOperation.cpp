#include <wmtk/operations/TriMeshEdgeCollapseOperation.h>
namespace wmtk {

bool TriMeshEdgeCollapseOperation::execute(TriMesh& m, const Tuple& loc0)
{
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
            // print_tup(m, new_tup_opt.value(), "TECO::execute: Using backup new_tup_opt ");
            assert(new_tup_opt.has_value());
        } else {
            // print_tup(m, new_tup_opt.value(), "TECO::execute: Using default new_tup_opt ");
        }
        new_fid = new_tup_opt.value().fid(m);
    }

    // get the vids
    size_t vid1 = loc0.vid(m);
    size_t vid2 = m.switch_vertex(loc0).vid(m);


    // get the fids
    const auto& n1_fids = vertex_connectivity[vid1].m_conn_tris;

    const auto& n2_fids = vertex_connectivity[vid2].m_conn_tris;

    // get the fids that will be modified
    auto n12_intersect_fids = fids_containing_edge(m, loc0);
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

    for (const size_t fid : n12_union_fids) {
        tri_connectivity[fid].hash++;
    }
    // modify the triangles
    // the m_conn_tris needs to be sorted
    const size_t new_vid = get_next_empty_slot_v(m);

    auto update_fid_vids = [&](const std::vector<size_t> fids, const size_t old_vid) {
        for (size_t fid : fids) {
            auto& tri_con = tri_connectivity[fid];
            if (tri_con.m_is_removed) {
                continue;
            }
            for (size_t& id : tri_con.m_indices) {
                if (id == old_vid) {
                    id = new_vid;
                }
            }
        }
    };

    update_fid_vids(n1_fids, vid1);
    update_fid_vids(n2_fids, vid2);

    // now work on vids
    // add in the new vertex

    auto& new_vertex_conn = vertex_connectivity[new_vid];
    for (size_t fid : n12_union_fids) {
        if (tri_connectivity[fid].m_is_removed) {
            continue;
        } else {
            new_vertex_conn.m_conn_tris.push_back(fid);
        }
    }
    new_vertex_conn.m_is_removed = false;
    // This is sorting too, and it is important to sort
    vector_unique(new_vertex_conn.m_conn_tris);

    // remove the erased fids from the vertices' (the one of the triangles that is not the end
    // points of the edge) connectivity list
    for (size_t fid : n12_intersect_fids) {
        for (size_t f_vid : tri_connectivity[fid].m_indices) {
            if (f_vid != vid1 && f_vid != vid2) {
                auto& tri_cons = vertex_connectivity[f_vid].m_conn_tris;
                assert(vector_contains(tri_cons, fid));
                vector_erase(tri_cons, fid);
            }
        }
    }

    // ? ? tuples changes. this needs to be done before post check since checked are done on tuples
    // update the old tuple version number
    // create an edge tuple for each changed edge
    // call back check will be done on this vector of tuples

    assert(!new_vertex_conn.m_conn_tris.empty());

    const size_t gfid = new_vertex_conn.m_conn_tris[0];
    int j = tri_connectivity[gfid].find(new_vid);
#if !defined(NDEBUG)
    auto new_t = Tuple(new_vid, (j + 2) % 3, gfid, m);
    assert(new_t.is_valid(m));
#endif


    int j_ret = tri_connectivity[new_fid].find(new_vid);
    Tuple return_t = Tuple(new_vid, (j_ret + 2) % 3, new_fid, m);

    set_return_tuple(return_t);


#if !defined(NDEBUG)
    for (const auto& tri : modified_triangles(m)) {
        for (const size_t index : m.oriented_tri_vids(tri)) {
            assert(
                std::find(
                    vertex_connectivity[index].m_conn_tris.begin(),
                    vertex_connectivity[index].m_conn_tris.end(),
                    tri.fid(m)) != vertex_connectivity[index].m_conn_tris.end());
        }
    }
#endif

    return true;
}

auto TriMeshEdgeCollapseOperation::modified_triangles(const TriMesh& m) const -> std::vector<Tuple>
{
    const auto& new_tup_opt = get_return_tuple_opt();

    assert(new_tup_opt.has_value());
    const Tuple& new_tup = new_tup_opt.value();
    return m.get_one_ring_tris_for_vertex(new_tup);
}

auto TriMeshEdgeCollapseOperation::new_vertex(const TriMesh& m) const -> std::optional<Tuple>
{
    return get_return_tuple_opt();
}


bool TriMeshEdgeCollapseOperation::before(TriMesh& m, const Tuple& t)
{
    return check_link_condition(m, t);
}

bool TriMeshEdgeCollapseOperation::after(TriMesh& m)
{
    return true;
}

std::string TriMeshEdgeCollapseOperation::name() const
{
    return "edge_collapse";
}

std::vector<size_t> TriMeshEdgeCollapseOperation::fids_containing_edge(
    const TriMesh& m,
    const Tuple& edge) const
{
    const auto faces = m.tris_bounded_by_edge(edge);
    std::vector<size_t> fids;
    std::transform(faces.begin(), faces.end(), std::back_inserter(fids), [&](const Tuple& t) {
        return t.fid(m);
    });
    return fids;
}
} // namespace wmtk
