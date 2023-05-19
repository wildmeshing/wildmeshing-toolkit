#include <wmtk/operations/TriMeshEdgeCollapseOperation.h>
namespace wmtk {


auto TriMeshEdgeCollapseOperation::links_of_vertex(const TriMesh& mesh, const Tuple& vertex)
    -> LinksOfVertex
{
    size_t vid = vertex.vid(mesh);
    const std::vector<Tuple> vid_ring = mesh.get_one_ring_edges_for_vertex(vertex);

    LinksOfVertex ret;
    std::vector<size_t>& lk_vid = ret.vertex;
    std::vector<std::array<size_t, 2>>& lk_e_vid = ret.edge;

    // if i have a boundary vertex then
    for (const auto& e_vid : vid_ring) {
        const size_t vid = e_vid.vid(mesh);
        if (mesh.is_boundary_edge(e_vid)) {
            ret.infinite_vertex = true;
            ret.infinite_edge.emplace_back(vid);
        }
        lk_vid.push_back(vid);
    }
    std::vector<Tuple> vid_tris = mesh.get_one_ring_tris_for_vertex(vertex);
    for (const auto& v_tri_t : vid_tris) {
        const size_t fid = v_tri_t.fid(mesh);
        const auto& tri_con = tri_connectivity(mesh)[fid];
        const auto& indices = tri_con.m_indices;
        int l = tri_con.find(vid);
        assert(l != -1);
        size_t i0 = indices[(l + 1) % 3];
        size_t i1 = indices[(l + 2) % 3];
        auto& vids = lk_e_vid.emplace_back(std::array<size_t, 2>{{i0, i1}});
        std::sort(vids.begin(), vids.end());
    }
    vector_unique(lk_vid);
    vector_unique(lk_e_vid);
    vector_unique(ret.infinite_edge);
    return ret;
}
std::tuple<std::vector<size_t>, bool> TriMeshEdgeCollapseOperation::edge_link_of_edge_vids(
    const TriMesh& mesh,
    const Tuple& edge)
{
    auto get_opposing_vertex_vid = [&mesh](const Tuple& edge) -> size_t {
        return edge.switch_edge(mesh).switch_vertex(mesh).vid(mesh);
    };
    std::vector<size_t> lk_edge;
    lk_edge.push_back(get_opposing_vertex_vid(edge));
    const std::optional<Tuple> other_face_opt = edge.switch_face(mesh);
    bool has_infinite = false;
    if (!other_face_opt.has_value()) {
        has_infinite = true;
    } else {
        lk_edge.push_back(get_opposing_vertex_vid(other_face_opt.value()));
    }
    vector_sort(lk_edge);
    return {lk_edge, has_infinite};
}

bool TriMeshEdgeCollapseOperation::check_link_condition(const TriMesh& mesh, const Tuple& edge)
{
    assert(edge.is_valid(mesh));
    // the edge initially points at the first of two vertex links we are computing
    const LinksOfVertex v1 = links_of_vertex(mesh, edge);
    const LinksOfVertex v2 = links_of_vertex(mesh, edge.switch_vertex(mesh));

    // compute vertex link condition
    auto lk_vid12 = set_intersection(v1.vertex, v2.vertex);
    const bool lk_vid12_infinite = v1.infinite_vertex && v2.infinite_vertex;

    const auto [edge_link, edge_link_has_infinite] = edge_link_of_edge_vids(mesh, edge);

    // over finite vertices v_link
    const bool v_link_fin =  lk_vid12 == edge_link;
    // over infinite vertices v_link
    const bool v_link_inf = lk_vid12_infinite == edge_link_has_infinite;
    const bool v_link = v_link_fin && v_link_inf;

    // check edge link condition
    // in 2d edge link for an edge is always empty

    std::vector<std::array<size_t, 2>> res;
    const auto& lk_e_vid1 = v1.edge;
    const auto& lk_e_vid2 = v2.edge;
    std::set_intersection(
        lk_e_vid1.begin(),
        lk_e_vid1.end(),
        lk_e_vid2.begin(),
        lk_e_vid2.end(),
        std::back_inserter(res));

    
    const auto& lk_e_vid1_inf = v1.infinite_edge;
    const auto& lk_e_vid2_inf = v2.infinite_edge;
    std::vector<size_t> res_inf;
    std::set_intersection(
        lk_e_vid1_inf.begin(),
        lk_e_vid1_inf.end(),
        lk_e_vid2_inf.begin(),
        lk_e_vid2_inf.end(),
        std::back_inserter(res_inf));
    const bool e_link = res.empty() && res_inf.empty();
    return v_link && e_link;
}
} // namespace wmtk
