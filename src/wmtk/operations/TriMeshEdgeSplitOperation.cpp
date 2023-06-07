
#include <wmtk/operations/TriMeshEdgeSplitOperation.h>

namespace wmtk {
bool TriMeshEdgeSplitOperation::execute(TriMesh& m, const Tuple& t)
{
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
    const Tuple return_tuple = Tuple(vid1, eid, fid1, m);
    assert(return_tuple.is_valid(m));

#if !defined(NDEBUG)
    auto new_vertex = Tuple(new_vid, (l + 2) % 3, new_fid, m);
    assert(new_vertex.is_valid(m));
    // assert(new_vertex == this->new_vertex(m));
#endif

    set_return_tuple(return_tuple);
    return true;
}

bool TriMeshEdgeSplitOperation::before(TriMesh& m, const Tuple& t)
{
    return true;
}
bool TriMeshEdgeSplitOperation::after(TriMesh& m)
{
    return true;
}
std::string TriMeshEdgeSplitOperation::name() const
{
    return "edge_split";
}

auto TriMeshEdgeSplitOperation::new_vertex(const TriMesh& m) const -> Tuple
{
    assert(bool(*this));
    const std::optional<Tuple>& new_tup = get_return_tuple_opt();
    assert(new_tup.has_value());
    return new_tup.value().switch_vertex(m);
}

auto TriMeshEdgeSplitOperation::original_endpoints(TriMesh& m, const Tuple& t) const
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
auto TriMeshEdgeSplitOperation::modified_triangles(const TriMesh& m) const -> std::vector<Tuple>
{
    if (!bool(*this)) {
        return {};
    }

    const Tuple new_v = new_vertex(m);

    return m.get_one_ring_tris_for_vertex(new_v);
}
} // namespace wmtk
