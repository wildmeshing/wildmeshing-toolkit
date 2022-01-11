#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "EdgeOperations2d.h"

using namespace Edge2d;

bool Edge2d::EdgeOperations2d::collapse_before(const Tuple& t)
{
    if (!TriMesh::collapse_before(t)) return false;
    collapse_cache.v1p = m_vertex_positions[t.vid()];
    collapse_cache.v2p = m_vertex_positions[t.switch_vertex(*this).vid()];
    return true;
}

bool Edge2d::EdgeOperations2d::collapse_after(const Tuple& t)
{
    m_vertex_positions[t.vid()] = (collapse_cache.v1p + collapse_cache.v2p) / 2.0;
    return true;
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

        assert(check_mesh_connectivity_validity());

        if (!TriMesh::collapse_edge(loc, new_vert)) continue;
        cnt++;
        if (cnt % 100 == 0) wmtk::logger().trace(" 100 more collpased");

        target_vertex_count--;
        if (target_vertex_count <= 0) break;

        size_t new_vid = new_vert.vid();
        std::vector<TriMesh::Tuple> one_ring_edges = get_one_ring_edges_for_vertex(new_vert);

        for (TriMesh::Tuple edge : one_ring_edges) {
            size_t vid = edge.vid();
            double length = (m_vertex_positions[new_vid] - m_vertex_positions[vid]).squaredNorm();
            ec_queue.push(ElementInQueue(edge, length));
        }
    }


    return true;
}