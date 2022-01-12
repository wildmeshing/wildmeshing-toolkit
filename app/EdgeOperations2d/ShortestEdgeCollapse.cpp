#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "EdgeOperations2d.h"

using namespace Edge2d;


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


        std::vector<TriMesh::Tuple> one_ring_edges = get_one_ring_edges_for_vertex(new_vert);
        std::vector<size_t> one_ring_fid;
        one_ring_fid.resize(one_ring_edges.size());
        size_t new_vid = new_vert.vid();
        for (TriMesh::Tuple edge : one_ring_edges) {
            // radial edge
            double length =
                (m_vertex_positions[new_vid] - m_vertex_positions[edge.vid()]).squaredNorm();
            ec_queue.push(ElementInQueue(edge, length));
            // petal edge
            if (!wmtk::vector_contains(one_ring_fid, edge.fid())) {
                one_ring_fid.emplace_back(edge.switch_edge(*this).fid());
                length = (m_vertex_positions[(edge.switch_edge(*this)).switch_vertex(*this).vid()] -
                          m_vertex_positions[edge.vid()])
                             .squaredNorm();
                ec_queue.push(ElementInQueue(edge.switch_edge(*this), length));
            }
            if (edge.switch_face(*this).has_value()) {
                if (!wmtk::vector_contains(one_ring_fid, (edge.switch_face(*this).value()).fid())) {
                    one_ring_fid.emplace_back((edge.switch_face(*this).value()).fid());
                    length =
                        (m_vertex_positions[((edge.switch_face(*this).value()).switch_edge(*this))
                                                .switch_vertex(*this)
                                                .vid()] -
                         m_vertex_positions[edge.vid()])
                            .squaredNorm();
                    ec_queue.push(ElementInQueue(
                        ((edge.switch_face(*this).value()).switch_edge(*this)),
                        length));
                }
            }
        }
    }
    return true;
}