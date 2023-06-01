
#include <wmtk/operations/TriMeshEdgeSwapOperation.h>
namespace wmtk {

bool TriMeshEdgeSwapOperation::execute(TriMesh& m, const Tuple& t)
{
    auto& vertex_connectivity = this->vertex_connectivity(m);
    auto& tri_connectivity = this->tri_connectivity(m);

    // get the vids
    size_t vid1 = t.vid(m);
    size_t vid2 = t.switch_vertex(m).vid(m);

    auto tmp_tuple_opt = m.switch_face(t);
    if (!tmp_tuple_opt.has_value()) {
        return false;
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
        return false; // can't sawp on boundary edge
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
    set_return_tuple(m.init_from_edge(vid4, vid3, test_fid2));
    // we just assigned so we are confident that the value exists
#if !defined(NDEBUG)
    const Tuple return_tuple = get_return_tuple_opt().value();

    assert(return_tuple.switch_vertex(m).vid(m) != vid1);
    assert(return_tuple.switch_vertex(m).vid(m) != vid2);
    assert(return_tuple.is_valid(m));
#endif

    return true;
}

auto TriMeshEdgeSwapOperation::modified_triangles(const TriMesh& m) const -> std::vector<Tuple>
{
    const std::optional<Tuple>& new_tuple_opt = get_return_tuple_opt();
    if (!new_tuple_opt.has_value()) {
        return {};
    }
    const Tuple& new_tuple = new_tuple_opt.value();
    assert(new_tuple.is_valid(m));
    auto new_other_face_opt = new_tuple.switch_face(m);
    assert(new_other_face_opt);
    return {new_tuple, new_other_face_opt.value()};
}
bool TriMeshEdgeSwapOperation::before(TriMesh& mesh, const Tuple& t)
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
bool TriMeshEdgeSwapOperation::after(TriMesh& mesh)
{
    return true;
}
std::string TriMeshEdgeSwapOperation::name() const
{
    return "edge_swap";
}

} // namespace wmtk
