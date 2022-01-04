#pragma once
#include <wmtk/ConcurrentTriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ParallelEdgeCollapse.h"
#include <tbb/task_group.h>
#include <tbb/parallel_for.h>
#include <tbb/concurrent_priority_queue.h>
#include <tbb/task_group.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_arena.h>


using namespace Edge2d;

#define NUM_THREADS 1
// delete the old two vert position, add the new at the middle of the old two points
void Edge2d::ParallelEdgeCollapse::update_position(size_t v1, size_t v2, Tuple& new_vert)
{
    size_t new_vid = new_vert.get_vid();
    Eigen::Vector3d new_position = (m_vertex_positions[v1] + m_vertex_positions[v2]) / 2;
    if (new_vid < m_vertex_positions.size()) m_vertex_positions[new_vid] = new_position;
    m_vertex_positions.emplace_back(new_position);
}

void Edge2d::ParallelEdgeCollapse::collapse_shortest_stuff(ElementInQueue &eiq, tbb::concurrent_priority_queue<ElementInQueue, cmp_s> &ec_queue){
    auto loc = eiq.edge;
    double weight = eiq.weight;
    // check if the edge tuple is valid
    if (!loc.is_valid(*this)) return;
    //set lock here
    size_t v1 = loc.get_vid();
    ConcurrentTriMesh::Tuple v2_tuple = loc.switch_vertex(*this);
    size_t v2 = v2_tuple.get_vid();
    ConcurrentTriMesh::Tuple new_vert;
    if (!ConcurrentTriMesh::collapse_edge(loc, new_vert)) return;
    update_position(v1, v2, new_vert);
    size_t new_vid = new_vert.get_vid();
    std::vector<ConcurrentTriMesh::Tuple> one_ring_edges = get_one_ring_edges_for_vertex(new_vert);
    for (ConcurrentTriMesh::Tuple edge : one_ring_edges) {
        ConcurrentTriMesh::Tuple tmp_tuple = switch_vertex(new_vert);
        size_t vid = tmp_tuple.get_vid();
        double length = (m_vertex_positions[new_vid] - m_vertex_positions[vid]).squaredNorm();
        ec_queue.push(ElementInQueue(edge, length));
    }
}


bool Edge2d::ParallelEdgeCollapse::collapse_shortest()
{
    std::vector<ConcurrentTriMesh::Tuple> edges = get_edges();
    tbb::concurrent_priority_queue<ElementInQueue, cmp_s> ec_queue;
    double shortest = std::numeric_limits<double>::max();
    for (auto& loc : edges) {
        ConcurrentTriMesh::Tuple v2 = loc.switch_vertex(*this);
        double length =
            (m_vertex_positions[loc.get_vid()] - m_vertex_positions[v2.get_vid()]).squaredNorm();
        if (length < shortest) shortest = length;
        ec_queue.push(ElementInQueue(loc, length));
    }

    tbb::task_arena arena(NUM_THREADS);
    tbb::task_group tg;

    arena.execute([&ec_queue, &tg, this]{
        ElementInQueue eiq;
        while (ec_queue.try_pop(eiq)) {
            tg.run([&]{collapse_shortest_stuff(eiq, ec_queue);});
        }
    });
    

    // parallel_collapse_shortest pcs(ec_queue, this->m_vertex_positions);
    // tbb::parallel_reduce(tbb::blocked_range<size_t>(0,NUM_THREADS), pcs);

    return true;
}