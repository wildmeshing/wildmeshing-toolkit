#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_queue.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_arena.h>
#include <tbb/task_group.h>
#include <wmtk/ConcurrentTriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <wmtk/utils/Logger.hpp>
#include "ParallelEdgeCollapse.h"


using namespace Edge2d;

#define NUM_THREADS 1

class Runner;

void collapse_edge_task(ElementInQueue eiq, Runner& r);

class Runner
{
private:
    tbb::concurrent_priority_queue<ElementInQueue, cmp_s> ec_queue;
    tbb::task_group tg;
    ParallelEdgeCollapse& m;

public:
    Runner(ParallelEdgeCollapse& mesh)
        : m(mesh)
    {}

    void initialize()
    {
        std::vector<wmtk::ConcurrentTriMesh::Tuple> edges = m.get_edges();
        double shortest = std::numeric_limits<double>::max();
        for (auto& loc : edges) {
            wmtk::ConcurrentTriMesh::Tuple v2 = loc.switch_vertex(m);
            double length =
                (m.m_vertex_positions[loc.get_vid()] - m.m_vertex_positions[v2.get_vid()])
                    .squaredNorm();
            if (length < shortest) shortest = length;
            ec_queue.push(ElementInQueue(loc, length));
        }

        for (int i = 0, n = ec_queue.size(); i < n; ++i) {
            tg.run([&]() { this->runNext(); });
        }
    }

    void add(ElementInQueue& eiq)
    {
        ec_queue.push(eiq);
        tg.run([&]() { this->runNext(); });
    }

    void runNext()
    {
        ElementInQueue eiq;
        bool ok = ec_queue.try_pop(eiq);
        assert(ok);
        collapse_edge_task(eiq);
    }

    void tgwait() { tg.wait(); }

    void collapse_edge_task(ElementInQueue& eiq)
    {
        auto loc = eiq.edge;
        double weight = eiq.weight;
        // check if the edge tuple is valid
        if (!loc.is_valid(m)) return;
        // set lock here
        size_t v1 = loc.get_vid();
        wmtk::ConcurrentTriMesh::Tuple v2_tuple = loc.switch_vertex(m);
        size_t v2 = v2_tuple.get_vid();
        wmtk::ConcurrentTriMesh::Tuple new_vert;
        if (!m.collapse_edge(loc, new_vert)) return;
        m.update_position(v1, v2, new_vert);
        size_t new_vid = new_vert.get_vid();
        std::vector<wmtk::ConcurrentTriMesh::Tuple> one_ring_edges =
            m.get_one_ring_edges_for_vertex(new_vert);
        for (wmtk::ConcurrentTriMesh::Tuple edge : one_ring_edges) {
            wmtk::ConcurrentTriMesh::Tuple tmp_tuple = m.switch_vertex(new_vert);
            size_t vid = tmp_tuple.get_vid();
            double length =
                (m.m_vertex_positions[new_vid] - m.m_vertex_positions[vid]).squaredNorm();
            ElementInQueue new_eiq(edge, length);
            this->add(eiq);
        }
    }
};


// delete the old two vert position, add the new at the middle of the old two points
void Edge2d::ParallelEdgeCollapse::update_position(size_t v1, size_t v2, Tuple& new_vert)
{
    size_t new_vid = new_vert.get_vid();
    Eigen::Vector3d new_position = (m_vertex_positions[v1] + m_vertex_positions[v2]) / 2;
    if (new_vid < m_vertex_positions.size()) m_vertex_positions[new_vid] = new_position;
    m_vertex_positions.emplace_back(new_position);
}

int global_cnt = 0;

void Edge2d::ParallelEdgeCollapse::collapse_shortest_stuff(
    ElementInQueue& eiq,
    tbb::concurrent_priority_queue<ElementInQueue, cmp_s>& ec_queue)
{
    auto loc = eiq.edge;
    double weight = eiq.weight;
    // check if the edge tuple is valid
    if (!loc.is_valid(*this)) return;
    // set lock here
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
    // task-based version

    tbb::task_arena arena(NUM_THREADS);
    arena.execute([this]() {
        Runner r(*this);
        r.initialize();
        r.tgwait();
    });

    //  thread simulate version

    // std::vector<ConcurrentTriMesh::Tuple> edges = get_edges();
    // tbb::concurrent_priority_queue<ElementInQueue, cmp_s> ec_queue;
    // double shortest = std::numeric_limits<double>::max();
    // for (auto& loc : edges) {
    //     ConcurrentTriMesh::Tuple v2 = loc.switch_vertex(*this);
    //     double length =
    //         (m_vertex_positions[loc.get_vid()] - m_vertex_positions[v2.get_vid()]).squaredNorm();
    //     if (length < shortest) shortest = length;
    //     ec_queue.push(ElementInQueue(loc, length));
    // }

    // tbb::task_arena arena(NUM_THREADS);
    // tbb::task_group tg;

    // arena.execute([&ec_queue, &tg, this]() {
    //     for (int i = 0; i < NUM_THREADS; ++i) {
    //         tg.run([&] {
    //             ElementInQueue eiq;
    //             while (ec_queue.try_pop(eiq)) {
    //                 // std::cout << global_cnt++ << std::endl;
    //                 collapse_shortest_stuff(eiq, ec_queue);
    //             }
    //         });
    //     }

    //     tg.wait();
    // });

    return true;
}