
namespace adaptive_tessellation {

private:
wmtk::VertexLinksData seamed_vertex_link(const AdaptiveTessellation& m, const Tuple& vertex)
{
    wmtk::VertexLinksData ret;
    std::vector<TriMesh::Tuple> all_mirror_vertices = m.get_all_mirror_vertices(vertex);
    for (const TriMesh::Tuple& vtup : all_mirror_vertices) {
        const auto& links = wmtk::vertex_link(m, vtup);
        ret.vertex_link.insert(
            ret.vertex_link.end(),
            links.vertex_link.begin(),
            links.vertex_link.end());
        ret.edge_link.insert(ret.edge_link.end(), links.edge_link.begin(), links.edge_link.end());
    }
    return ret;
}
std::vector<size_t> seamed_edge_link(const AdaptiveTessellation& mesh, const Tuple& edge)
{
    std::vector<size_t> lk_edge;
    lk_edge.push_back((edge.switch_edge(mesh)).switch_vertex(mesh).vid(mesh));
    const std::optional<wmtk::Tuple> other_face_opt;
    if (!other_face_opt.has_value()) {
        lk_edge.push_back(dummy);
    } else {
        lk_edge.push_back(
            ((other_face_opt.value()).switch_edge(mesh)).switch_vertex(mesh).vid(mesh));
    }
    vector_sort(lk_edge);
    return lk_edge;
}

bool seamed_link_condition(const AdaptiveTessellation& m, const Tuple& edge)
{
    assert(edge.is_valid(mesh));
    const VertexLinksData v1 = seamed_vertex_link_data(m, edge);
    const VertexLinksData v2 = seamed_vertex_link_data(m, edge.switch_vertex(m));

    auto lk_vid12 = set_intersection(v1.vertex_link, v2.vertex_link);
    bool v_link = lk_vid12 == seamed_edge_link(m, edge);

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
} // namespace adaptive_tessellation
