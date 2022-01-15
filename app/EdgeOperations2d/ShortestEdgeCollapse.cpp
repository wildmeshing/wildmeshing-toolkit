#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "EdgeOperations2d.h"
using namespace wmtk;
using namespace Edge2d;
std::vector<TriMesh::Tuple> Edge2d::EdgeOperations2d::new_edges_after_collapse_split(
    const TriMesh::Tuple& t) const
{
    std::vector<TriMesh::Tuple> new_edges;
    std::vector<size_t> one_ring_fid;

    for (auto e : get_one_ring_edges_for_vertex(t)) {
        // centripedal edge
        new_edges.push_back(e);
        // petal edge
        if (!wmtk::vector_contains(one_ring_fid, e.fid())) {
            one_ring_fid.emplace_back(e.fid());
            new_edges.push_back(e.switch_edge(*this));
            if (!is_boundary_edge(e) &&
                !wmtk::vector_contains(one_ring_fid, (e.switch_face(*this).value()).fid())) {
                one_ring_fid.emplace_back((e.switch_face(*this).value()).fid());
                new_edges.push_back((e.switch_face(*this).value()).switch_edge(*this));
            }
        }
    }
    return new_edges;
}

bool Edge2d::EdgeOperations2d::collapse_shortest(int target_vertex_count)
{
    std::vector<TriMesh::Tuple> edges = get_edges();
    std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_s> ec_queue;
    double shortest = std::numeric_limits<double>::max();
    for (auto& loc : edges) {
        TriMesh::Tuple v2 = loc.switch_vertex(*this);
        double length =
            (m_vertex_positions[loc.vid()] - m_vertex_positions[v2.vid()]).squaredNorm();
        if (length < shortest) shortest = length;
        ec_queue.push(ElementInQueue(loc, length));
    }
    int cnt = 0;
    while (!ec_queue.empty()) {
        auto loc = ec_queue.top().edge;
        double weight = ec_queue.top().weight;
        ec_queue.pop();
        if (!loc.is_valid(*this)) continue;

        TriMesh::Tuple new_vert;

        if (!TriMesh::collapse_edge(loc, new_vert)) continue;
        cnt++;
        if (cnt % 100 == 0) wmtk::logger().trace(" 100 more collpased");

        target_vertex_count--;
        if (target_vertex_count <= 0) break;
        // push new edges to the queue
        auto new_edges = new_edges_after_collapse_split(new_vert);
        for (auto new_e : new_edges)
            ec_queue.push(ElementInQueue(
                new_e,
                (m_vertex_positions[new_e.vid()] -
                 m_vertex_positions[new_e.switch_vertex(*this).vid()])
                    .squaredNorm()));
    }
    return true;
}