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
    if (new_vid < m_vertex_positions.size())
        m_vertex_positions[new_vid] = new_position;
    else
        m_vertex_positions.emplace_back(new_position);
}


bool Edge2d::EdgeCollapse::collapse_shortest(int target_vertex_count)
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
    int cnt = 0;
    while (!ec_queue.empty()) {
        auto loc = ec_queue.top().edge;
        double weight = ec_queue.top().weight;
        ec_queue.pop();
        // check if the edge tuple is valid
        // std::cout << "the candidate " << loc.get_vid() << " fid is " << loc.get_fid() <<
        // std::endl;
        if (!loc.is_valid(*this)) continue;

        Eigen::Vector3d v1p = m_vertex_positions[loc.get_vid()];
        auto tmp_tuple = loc.switch_vertex(*this);
        Eigen::Vector3d v2p = m_vertex_positions[tmp_tuple.get_vid()];

        // std::cout << "actually candidate is between  " << v1 << " " << v2 << std::endl;

        TriMesh::Tuple new_vert;

        check_mesh_connectivity_validity();

        if (!TriMesh::collapse_edge(loc, new_vert)) continue;
        cnt++;
        if (cnt % 100 == 0) std::cout << " 100 more collpased" << std::endl;


        // std::cout << "collapsed and got " << new_vert.get_vid() << " " << new_vert.get_fid()
        //   << std::endl;

        // update_position(v1, v2, new_vert);
        m_vertex_positions[new_vert.get_vid()] = (v1p + v2p) / 2.0;
        // wmtk::vector_print(m_vertex_positions);

        assert(check_mesh_connectivity_validity());

        target_vertex_count--;
        if (target_vertex_count <= 0) break;

        size_t new_vid = new_vert.get_vid();
        std::vector<TriMesh::Tuple> one_ring_edges = get_one_ring_edges_for_vertex(new_vert);

        for (TriMesh::Tuple edge : one_ring_edges) {
            size_t vid = edge.get_vid();
            // std::cout << " the one ring edge for " << new_vert.get_vid() << " include " << vid
            //           << std::endl;
            double length = (m_vertex_positions[new_vid] - m_vertex_positions[vid]).squaredNorm();
            ec_queue.push(ElementInQueue(edge, length));
        }
    }


    return true;
}