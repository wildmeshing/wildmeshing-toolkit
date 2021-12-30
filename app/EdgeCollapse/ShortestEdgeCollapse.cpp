#include <EdgeCollapse.h>
#include <wmtk/TriMesh.h>
#include <wmtk/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <climits>

// delete the old two vert position, add the new at the middle of the old two points
void EdgeCollapse::update_position(size_t v1, size_t v2, Tuple& new_vert)
{
    size_t new_vid = new_vert.get_vid();
    Eigen::Vector3d new_position = (m_vertex_positions[v1] + m_vertex_positions[v2]) / 2;
    if (new_vid < m_vertex_positions.size()) m_vertex_connectivity[new_vid] = new_position;
    m_vertex_position.emplace_back(new_position);
}

// add in the new edges
void EdgeCollapse::update_priorityq(
    std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_s> ec_queue,
    Tuple& new_vert)
{
    size_t new_vid = new_vert.get_vid();
    std::vector<Tuple> one_ring_edges = get_one_ring_edges_for_vertex(new_vert);
    for (Tuple edge : one_ring_edges) {
        Tuple tmp_tuple = switch_vertex(new_vert);
        size_t vid = tmp_tuple.get_vid();
        double length = (m_vertex_positions[new_vid] - m_vertex_positions[vid]).squaredNorm();
        ec_queue.push(ElementInQueue(edge, length));
    }
}


bool EdgeCollapse::collapse_shortest()
{
    std::vector<Tuple> edges = get_edges();
    std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_s> ec_queue;
    double shortest = DBL_MAX;
    for (auto& loc : edges) {
        Tuple v2 = loc.switch_vertex(*this);
        double length =
            (m_vertex_positions[loc.vid()] - m_vertex_positions[v2.vid()]).squaredNorm();
        if (lenght < shortest) shortest = length;
        ec_queue.push(ElementInQueue(loc, length));
    }
    if (!ec_queue.empty()) {
        auto loc = ec_queue.top().edge;
        double weight = ec_queue.top().weight;
        ec_queue.pop();
        // check if the edge tuple is valid
        if (!loc.is_valid()) return false;

        Tuple new_vert;
        size_t v1 = loc.get_vid();
        Tuple v2_tuple = loc.switch_vertex(*this);
        size_t v2 = v2_tuple.get_vid();
        TriMesh::collapse_edge(loc, new_vert);
        update_position(v1, v2, new_vert);
        update_priorityq(ec_queue, new_vert);
        return true;
    }
    return false;
}