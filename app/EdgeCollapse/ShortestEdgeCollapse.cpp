#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "EdgeCollapse.h"

using namespace Edge2d;
// delete the old two vert position, add the new at the middle of the old two points
void Edge2d::EdgeCollapse::update_position(size_t v1, size_t v2, Tuple& new_vert)
{
    size_t new_vid = new_vert.get_vid();
    Eigen::Vector3d new_position = (m_vertex_positions[v1] + m_vertex_positions[v2]) / 2;
    if (new_vid < m_vertex_positions.size()) m_vertex_positions[new_vid] = new_position;
    m_vertex_positions.emplace_back(new_position);
}


bool Edge2d::EdgeCollapse::collapse_shortest()
{
    std::vector<TriMesh::Tuple> edges = get_edges();
    std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_s> ec_queue;
    double shortest = std::numeric_limits<double>::max();
    for (auto& loc : edges) {
        TriMesh::Tuple v2 = loc.switch_vertex(*this);
        double length =
            (m_vertex_positions[loc.get_vid()] - m_vertex_positions[v2.get_vid()]).squaredNorm();
        if (length < shortest) shortest = length;
        ec_queue.push(ElementInQueue(loc, length));
    }
    while (!ec_queue.empty()) {
        auto loc = ec_queue.top().edge;
        double weight = ec_queue.top().weight;
        ec_queue.pop();
        // check if the edge tuple is valid
        if (!loc.is_valid(*this)) continue;

        TriMesh::Tuple new_vert;
        size_t v1 = loc.get_vid();
        TriMesh::Tuple v2_tuple = loc.switch_vertex(*this);
        size_t v2 = v2_tuple.get_vid();
        if (!TriMesh::collapse_edge(loc, new_vert)) continue;
        update_position(v1, v2, new_vert);
        size_t new_vid = new_vert.get_vid();
        std::vector<TriMesh::Tuple> one_ring_edges = get_one_ring_edges_for_vertex(new_vert);
        for (TriMesh::Tuple edge : one_ring_edges) {
            TriMesh::Tuple tmp_tuple = switch_vertex(new_vert);
            size_t vid = tmp_tuple.get_vid();
            double length = (m_vertex_positions[new_vid] - m_vertex_positions[vid]).squaredNorm();
            ec_queue.push(ElementInQueue(edge, length));
        }
        return true;
    }
    return false;
}