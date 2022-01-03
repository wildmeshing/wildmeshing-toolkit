#include <wmtk/ConcurrentTriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ParallelEdgeCollapse.h"
#include <tbb/task_group.h>
#include <tbb/parallel_for.h>
#include <tbb/concurrent_priority_queue.h>
#include <tbb/task_scheduler_init.h>

#define NUM_THREADS 1

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
    std::vector<ConcurrentTriMesh::Tuple> edges = get_edges();
    tbb::concurrent_priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_s> ec_queue;
    double shortest = std::numeric_limits<double>::max();
    for (auto& loc : edges) {
        ConcurrentTriMesh::Tuple v2 = loc.switch_vertex(*this);
        double length =
            (m_vertex_positions[loc.get_vid()] - m_vertex_positions[v2.get_vid()]).squaredNorm();
        if (length < shortest) shortest = length;
        ec_queue.push(ElementInQueue(loc, length));
    }

    tbb::task_scheduler_init init(NUM_THREADS);
    tbb::parallel_for(tbb::blocked_range<size_t>(0,NUM_THREADS), parallel_collapse_shortest(ec_queue));

    return true;
}