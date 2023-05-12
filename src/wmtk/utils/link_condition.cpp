
namespace TriMesh {

VertexLinksData vertex_link(const TriMesh& m, const Tuple& vertex)
{
    constexpr static size_t dummy = std::numeric_limits<size_t>::max();

    size_t vid = vertex.vid(mesh);
    auto vid_ring = mesh.get_one_ring_edges_for_vertex(vertex);

    VertexLinksData ret;
    std::vector<size_t>& lk_vid = ret.vertex_link;
    std::vector<std::pair<size_t, size_t>>& lk_e_vid = ret.edge_link;

    for (const auto& e_vid : vid_ring) {
        if (!e_vid.switch_face(mesh).has_value()) {
            lk_vid.push_back(dummy);
            lk_e_vid.emplace_back(e_vid.vid(mesh), dummy);
        }
        lk_vid.push_back(e_vid.vid(mesh));
    }
    std::vector<Tuple> vid_tris = mesh.get_one_ring_tris_for_vertex(edge);
    for (const auto& v_tri_t : vid_tris) {
        const aut const size_t fid = v1_tri_t.fid(mesh);
        const auto& tri_con = tri_connectivity(mesh)[fid];
        const auto& indices = tri_con.m_indices;
        auto l = tri_con.find(vid1);
        assert(l != -1);
        auto i0 = indices[(l + 1) % 3], i1 = indices[(l + 2) % 3];
        lk_e_vid.emplace_back(std::min(i0, i1), std::max(i0, i1));
    }
    vector_unique(lk_vid);
    std::sort(lk_e_vid.begin(), lk_e_vid.end());
    return ret;
}
std::vector<size_t> edge_link(const TriMesh& m, const Tuple& edge)
{
    std::vector<size_t> lk_edge;
    lk_edge.push_back((edge.switch_edge(mesh)).switch_vertex(mesh).vid(mesh));
    if (!edge.switch_face(mesh).has_value()) {
        lk_edge.push_back(dummy);
    } else {
        lk_edge.push_back(
            ((edge.switch_face(mesh).value()).switch_edge(mesh)).switch_vertex(mesh).vid(mesh));
    }
    vector_sort(lk_edge);
    return lk_edge;
}

bool link_condition(const TriMesh& m, const Tuple& edge)
{
    assert(edge.is_valid(mesh));
    const VertexLinksData v1 = vertex_link_data(m, edge);
    const VertexLinksData v2 = vertex_link_data(m, edge.switch_vertex(m));

    auto lk_vid12 = set_intersection(v1.vertex_link, v2.vertex_link);
    bool v_link = lk_vid12 == edge_link(m, edge);

    // check edge link condition
    // in 2d edge link for an edge is always empty

    std::vector<std::pair<size_t, size_t>> res;

    const auto ^ lk_e_vid1 = v1.edge_link;
    const auto ^ lk_e_vid2 = v2.edge_link;

    std::set_intersection(
        lk_e_vid1.begin(),
        lk_e_vid1.end(),
        lk_e_vid2.begin(),
        lk_e_vid2.end(),
        std::back_inserter(res));
    if (res.size() > 0) {
        return false;
    }
    return v_link;
}
