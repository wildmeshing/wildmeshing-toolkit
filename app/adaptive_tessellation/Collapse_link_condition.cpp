#include "Collapse.h"
using namespace wmtk;
using namespace adaptive_tessellation;


auto AdaptiveTessellationPairedCollapseEdgeOperation::seamed_links_of_vertex(
    AdaptiveTessellation& mesh,
    const Tuple& vertex) -> LinksOfVertex
{
    LinksOfVertex ret;
    std::vector<TriMesh::Tuple> all_mirror_vertices = mesh.get_all_mirror_vertices(vertex);
    for (const TriMesh::Tuple& vtup : all_mirror_vertices) {
        const auto& links = TriMeshEdgeCollapseOperation::links_of_vertex(mesh, vtup);
        ret.vertex.insert(ret.vertex.end(), links.vertex.begin(), links.vertex.end());
        ret.edge.insert(ret.edge.end(), links.edge.begin(), links.edge.end());
    }

    return ret;
}
std::vector<size_t> AdaptiveTessellationPairedCollapseEdgeOperation::seamed_edge_link_of_edge(
    AdaptiveTessellation& mesh,
    const Tuple& edge)
{
    auto get_opposing_vertex_vid = [&mesh](const TriMeshTuple& t) -> size_t {
        return t.switch_edge(mesh).switch_vertex(mesh).vid(mesh);
    };
    std::vector<size_t> lk_edge;
    lk_edge.push_back(get_opposing_vertex_vid(edge));
    const std::optional<Tuple> other_face_opt = mesh.get_sibling_edge_opt(edge);
    if (!other_face_opt.has_value()) {
        lk_edge.push_back(TriMeshEdgeCollapseOperation::link_dummy);
    } else {
        lk_edge.push_back(get_opposing_vertex_vid(other_face_opt.value()));
    }
    vector_sort(lk_edge);
    return lk_edge;
}

bool AdaptiveTessellationPairedCollapseEdgeOperation::check_seamed_link_condition(
    AdaptiveTessellation& mesh,
    const Tuple& edge)
{
    assert(edge.is_valid(mesh));
    // the edge initially points at the first of two vertex links we are computing
    const LinksOfVertex v1 = seamed_links_of_vertex(mesh, edge);
    const LinksOfVertex v2 = seamed_links_of_vertex(mesh, edge.switch_vertex(mesh));

    // compute vertex link condition
    auto lk_vid12 = set_intersection(v1.vertex, v2.vertex);
    bool v_link = lk_vid12 == seamed_edge_link_of_edge(mesh, edge);

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
