#include <wmtk/utils/TupleUtils.hpp>
#include "Collapse.h"
using namespace wmtk;
using namespace adaptive_tessellation;

namespace {

template <typename A, typename B>
void tuples_to_vids(const TriMesh& m, const A& a, B& b)
{
    std::transform(a.begin(), a.end(), b.begin(), [&](const TriMeshTuple& t) -> size_t {
        return t.vid(m);
    });
}
template <typename A, typename B>
void vids_to_tuples(const TriMesh& m, const A& a, B& b)
{
    std::transform(a.begin(), a.end(), b.begin(), [&](const size_t vid) -> TriMeshTuple {
        return m.tuple_from_vertex(vid);
    });
}

std::vector<size_t> tuples_to_vids(const TriMesh& m, const std::vector<TriMeshTuple>& tuples)
{
    std::vector<size_t> r;
    r.resize(tuples.size());
    tuples_to_vids(m, tuples, r);
    return r;
}
std::vector<TriMeshTuple> vids_to_tuples(const TriMesh& m, const std::vector<size_t>& vids)
{
    std::vector<TriMeshTuple> r;
    r.resize(vids.size());
    vids_to_tuples(m, vids, r);
    return r;
}
template <size_t D>
std::array<size_t, D> tuples_to_vids(const TriMesh& m, const std::array<TriMeshTuple, D>& tuples)
{
    std::array<size_t, D> r;
    tuples_to_vids(m, tuples, r);
    return r;
}
template <size_t D>
std::array<TriMeshTuple, D> vids_to_tuples(const TriMesh& m, const std::array<size_t, D>& vids)
{
    std::array<TriMeshTuple, D> r;
    vids_to_tuples(m, vids, r);
    return r;
}
std::array<size_t, 2> edge_tuple_to_vids(const TriMesh& m, const TriMeshTuple& edge)
{
    const size_t v0 = edge.vid(m);
    const size_t v1 = edge.switch_vertex(m).vid(m);
    std::array<size_t, 2> r{{v0, v1}};
    std::sort(r.begin(), r.end());
    return r;
}


std::vector<std::array<size_t, 2>> mirror_edge_vids(
    const AdaptiveTessellation& m,
    const std::array<size_t, 2>& e)
{
    std::vector<std::array<size_t, 2>> ret;
    ret.emplace_back(e);
    const auto edge_tup_opt = m.tuple_from_edge_vids_opt(e[0], e[1]);
    assert(edge_tup_opt.has_value()); // this edge came from somewhere

    const TriMeshTuple& edge_tup = edge_tup_opt.value();
    const std::vector<TriMeshTuple> tris = m.tris_bounded_by_edge(edge_tup);

    for (const TriMeshTuple& t : tris) {
        auto mirror_edge_opt = m.get_mirror_edge_opt(t);
        if (mirror_edge_opt.has_value()) {
            const TriMeshTuple& mirror_edge = mirror_edge_opt.value();
            ret.emplace_back(edge_tuple_to_vids(m, mirror_edge));
        }
    }

    vector_sort(ret);
    return ret;
}
std::vector<std::array<size_t, 2>> mirror_edge_vids(
    const AdaptiveTessellation& m,
    const std::vector<std::array<size_t, 2>>& edges)
{
    std::vector<std::array<size_t, 2>> ret = edges;

    for (const auto& e : edges) {
        set_union_inplace(ret, mirror_edge_vids(m, e));
    }
    vector_unique(ret);
    return ret;
}

std::vector<size_t> mirror_vids(const AdaptiveTessellation& m, size_t vid)
{
    auto r = m.get_all_mirror_vids(m.tuple_from_vertex(vid));
    vector_sort(r);
    return r;
}


std::vector<size_t> mirror_vids(const AdaptiveTessellation& m, const std::vector<size_t>& vids)
{
    std::vector<size_t> ret = vids;

    for (const size_t vid : vids) {
        set_union_inplace(ret, mirror_vids(m, vid));
    }
    vector_unique(ret);
    return ret;
}
} // namespace

auto AdaptiveTessellationPairedCollapseEdgeOperation::seamed_links_of_vertex(
    AdaptiveTessellation& mesh,
    const TriMeshTuple& vertex) -> LinksOfVertex
{
    LinksOfVertex ret;
    std::vector<TriMesh::Tuple> all_mirror_vertices;
    if (mesh.is_seam_vertex(vertex)) {
        all_mirror_vertices = mesh.get_all_mirror_vertices(vertex);
    } else {
        all_mirror_vertices.emplace_back(vertex);
    }
    for (const TriMesh::Tuple& vtup : all_mirror_vertices) {
        auto links = TriMeshEdgeCollapseOperation::links_of_vertex(mesh, vtup);

        ret.infinite_vertex |= links.infinite_vertex;

        vector_sort(links.vertex);
        vector_sort(links.edge);
        vector_sort(links.infinite_edge);
        set_union_inplace(ret.vertex, links.vertex);
        set_union_inplace(ret.edge, links.edge);
        set_union_inplace(ret.infinite_edge, links.infinite_edge);
    }


    ret.edge = mirror_edge_vids(mesh, ret.edge);
    ret.vertex = mirror_vids(mesh, ret.vertex);
    ret.infinite_edge = mirror_vids(mesh, ret.infinite_edge);
    vector_sort(ret.vertex);
    vector_sort(ret.edge);
    vector_sort(ret.infinite_edge);

    return ret;
}
std::tuple<std::vector<size_t>, bool>
AdaptiveTessellationPairedCollapseEdgeOperation::seamed_edge_link_of_edge(
    AdaptiveTessellation& mesh,
    const TriMeshTuple& edge)
{
    auto get_opposing_vertex_vid = [&mesh](const TriMeshTuple& t) -> size_t {
        return t.switch_edge(mesh).switch_vertex(mesh).vid(mesh);
    };
    std::vector<size_t> lk_edge;
    lk_edge.push_back(get_opposing_vertex_vid(edge));
    const std::optional<Tuple> other_face_opt = mesh.get_sibling_edge_opt(edge);
    bool has_infinite = false;
    if (!other_face_opt.has_value()) {
        has_infinite = true;
    } else {
        lk_edge.push_back(get_opposing_vertex_vid(other_face_opt.value()));
    }
    vector_sort(lk_edge);
    return {lk_edge, has_infinite};
}

auto AdaptiveTessellationPairedCollapseEdgeOperation::accumulate_mirror_vertices(
    const AdaptiveTessellation& m,
    const std::vector<Tuple>& vertices) const -> std::vector<Tuple>
{
    std::vector<Tuple> ret = vertices;

    for (const TriMeshTuple& v : vertices) {
        if (m.is_seam_vertex(v)) {
            auto tups = m.get_all_mirror_vertices(v);
            ret.insert(ret.end(), tups.begin(), tups.end());
        }
    }

    wmtk::unique_vertex_tuples(m, ret);
    return ret;
}
bool AdaptiveTessellationPairedCollapseEdgeOperation::check_seamed_link_condition(
    AdaptiveTessellation& mesh,
    const TriMeshTuple& edge)
{
    assert(edge.is_valid(mesh));
    // the edge initially points at the first of two vertex links we are computing
    const LinksOfVertex v1 = seamed_links_of_vertex(mesh, edge);
    const LinksOfVertex v2 = seamed_links_of_vertex(mesh, edge.switch_vertex(mesh));

    // compute vertex link condition
    auto lk_vid12 = set_intersection(v1.vertex, v2.vertex);
    const bool lk_vid12_infinite = v1.infinite_vertex && v2.infinite_vertex;

    const auto [edge_link, edge_link_has_infinite] = seamed_edge_link_of_edge(mesh, edge);
    bool v_link = lk_vid12 == edge_link && edge_link_has_infinite &&
                  lk_vid12_infinite == edge_link_has_infinite;

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
