#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_queue.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/spin_mutex.h>
#include <tbb/task_arena.h>
#include <tbb/task_group.h>
#include <wmtk/ConcurrentTriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <wmtk/utils/Logger.hpp>
#include "ParallelEdgeCollapse.h"


using namespace Edge2d;

#define NUM_THREADS 4

class Runner;

void collapse_edge_task(ElementInQueue eiq, Runner& r);

class Runner
{
private:
    tbb::concurrent_priority_queue<ElementInQueue, cmp_s> ec_queue;
    tbb::task_group tg;
    ParallelEdgeCollapse& m;
    int target_vertex_count;
    int cnt = 0;
    tbb::spin_mutex rw_lock;

public:
    Runner(ParallelEdgeCollapse& mesh, int tvc)
        : m(mesh)
        , target_vertex_count(tvc)
    {}

    void initialize()
    {
        std::vector<wmtk::ConcurrentTriMesh::Tuple> edges = m.get_edges();
        double shortest = std::numeric_limits<double>::max();
        for (auto& loc : edges) {
            wmtk::ConcurrentTriMesh::Tuple v2 = loc.switch_vertex(m);
            double length =
                (m.m_vertex_positions[loc.vid()] - m.m_vertex_positions[v2.vid()]).squaredNorm();
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
        rw_lock.lock();
        // std::cout << target_vertex_count << std::endl;
        if (target_vertex_count <= 0) {
            rw_lock.unlock();
            return;
        }
        rw_lock.unlock();

        // std::cout << "in task" << std::endl;
        // std::cout << (m.n_vertices() == m.n_mutex()) << std::endl;


        auto loc = eiq.edge;
        double weight = eiq.weight;

        // auto eid = loc;
        // std::cout << "collpase edge_v_id: " << eid.vid() << " " << eid.switch_vertex(m).vid()
        //           << std::endl;
        // check if the edge tuple is valid
        if (!loc.is_valid(m)) return;

        wmtk::ConcurrentTriMesh::Tuple new_vert;
        // assert(m.check_mesh_connectivity_validity());

        std::vector<size_t> mutex_release_stack;
        // std::cout << "before set lock" << std::endl;
        if (!m.try_set_edge_mutex_two_ring(loc, mutex_release_stack)) {
            eiq.times_skipped++;
            if (eiq.times_skipped > 10) return;
            this->add(eiq);
            return;
        }
        // std::cout << "locked: " << mutex_release_stack.size() << std::endl;
        // std::cout << "vids: ";
        // for (int i = 0; i < mutex_release_stack.size(); i++) {
        //     std::cout << mutex_release_stack[i] << " ";
        // }
        // std::cout << std::endl;

        auto loc_copy = loc;
        auto v1 = loc_copy.vid();
        auto v2 = loc.switch_vertex(m).vid();

        m.add_new_vertex_mutex();

        if (!m.collapse_edge(loc, new_vert)) {
            int num_released = m.release_vertex_mutex_in_stack(mutex_release_stack);
            // std::cout << "released: " << num_released << std::endl;
            return;
        }

        rw_lock.lock();
        cnt++;
        if (cnt % 100 == 0) std::cout << " 100 more collpased" << std::endl;
        target_vertex_count--;
        rw_lock.unlock();

        // m.add_new_vertex_mutex();

        size_t new_vid = new_vert.vid();
        // update position
        m.m_vertex_positions[new_vid] = (m.m_vertex_positions[v1] + m.m_vertex_positions[v2]) / 2.0;

        std::vector<wmtk::ConcurrentTriMesh::Tuple> one_ring_edges =
            m.get_one_ring_edges_for_vertex(new_vert);

        for (wmtk::ConcurrentTriMesh::Tuple edge : one_ring_edges) {
            size_t vid = edge.vid();
            double length =
                (m.m_vertex_positions[new_vid] - m.m_vertex_positions[vid]).squaredNorm();
            ElementInQueue new_eiq(edge, length);
            this->add(new_eiq);
        }

        int num_released = m.release_vertex_mutex_in_stack(mutex_release_stack);
        // std::cout << "released: " << num_released << std::endl;


        // // set lock here
        // std::vector<size_t> mutex_release_stack;
        // // std::cout << "before set lock" << std::endl;
        // if (!m.try_set_edge_mutex_two_ring(loc, mutex_release_stack)) {
        //     this->add(eiq);
        //     return;
        // }
        // std::cout << "locked: " << mutex_release_stack.size() << std::endl;
        // std::cout << "vids: ";
        // for (int i = 0; i < mutex_release_stack.size(); i++) {
        //     std::cout << mutex_release_stack[i] << " ";
        // }
        // std::cout << std::endl;
        // size_t v1 = loc.vid();
        // wmtk::ConcurrentTriMesh::Tuple v2_tuple = loc.switch_vertex(m);
        // size_t v2 = v2_tuple.vid();
        // wmtk::ConcurrentTriMesh::Tuple new_vert;
        // if (!m.collapse_edge(loc, new_vert)) {
        //     int num_released = m.release_vertex_mutex_in_stack(mutex_release_stack);
        //     std::cout << "released: " << num_released << std::endl;
        //     return;
        // }
        // m.update_position(v1, v2, new_vert);
        // size_t new_vid = new_vert.vid();
        // m.add_new_vertex_mutex();
        // std::vector<wmtk::ConcurrentTriMesh::Tuple> one_ring_edges =
        //     m.get_one_ring_edges_for_vertex(new_vert);
        // for (wmtk::ConcurrentTriMesh::Tuple edge : one_ring_edges) {
        //     wmtk::ConcurrentTriMesh::Tuple tmp_tuple = m.switch_vertex(new_vert);
        //     size_t vid = tmp_tuple.vid();
        //     double length =
        //         (m.m_vertex_positions[new_vid] - m.m_vertex_positions[vid]).squaredNorm();
        //     ElementInQueue new_eiq(edge, length);
        //     this->add(new_eiq);
        // }
        // int num_released = m.release_vertex_mutex_in_stack(mutex_release_stack);
        // std::cout << "released: " << num_released << std::endl;
    }
};


// delete the old two vert position, add the new at the middle of the old two points
void Edge2d::ParallelEdgeCollapse::update_position(size_t v1, size_t v2, Tuple& new_vert)
{
    size_t new_vid = new_vert.vid();
    Eigen::Vector3d new_position = (m_vertex_positions[v1] + m_vertex_positions[v2]) / 2;
    if (new_vid < m_vertex_positions.size())
        m_vertex_positions[new_vid] = new_position;
    else
        m_vertex_positions.emplace_back(new_position);
}

bool Edge2d::ParallelEdgeCollapse::collapse_before(const Tuple& t)
{
    if (check_link_condition(t) && check_manifold(t)) {
        collapse_cache.v1p = m_vertex_positions[t.vid()];
        collapse_cache.v2p = m_vertex_positions[t.switch_vertex(*this).vid()];
        return true;
    }
    return false;
}

bool Edge2d::ParallelEdgeCollapse::collapse_after(const Tuple& t)
{
    // TODO: for parallel check?
    // if (check_mesh_connectivity_validity() && t.is_valid(*this)) {
    if (t.is_valid(*this)) {
        // m_vertex_positions[t.vid()] = (collapse_cache.v1p + collapse_cache.v2p) / 2.0;
        return true;
    }
    return false;
}

bool Edge2d::ParallelEdgeCollapse::collapse_shortest(int target_vertex_count)
{
    tbb::task_arena arena(NUM_THREADS);
    arena.execute([this, target_vertex_count]() {
        Runner r(*this, target_vertex_count);
        r.initialize();
        r.tgwait();
    });

    return true;
}

// int global_cnt = 0;

// void Edge2d::ParallelEdgeCollapse::collapse_shortest_stuff(
//     ElementInQueue& eiq,
//     tbb::concurrent_priority_queue<ElementInQueue, cmp_s>& ec_queue)
// {
//     auto loc = eiq.edge;
//     double weight = eiq.weight;
//     // check if the edge tuple is valid
//     if (!loc.is_valid(*this)) return;
//     // set lock here
//     std::vector<size_t> mutex_release_stack;
//     try_set_edge_mutex_two_ring(loc, mutex_release_stack);
//     if (!try_set_edge_mutex_two_ring(loc, mutex_release_stack)) {
//         ec_queue.push(eiq);
//         return;
//     }
//     size_t v1 = loc.vid();
//     ConcurrentTriMesh::Tuple v2_tuple = loc.switch_vertex(*this);
//     size_t v2 = v2_tuple.vid();
//     ConcurrentTriMesh::Tuple new_vert;
//     if (!ConcurrentTriMesh::collapse_edge(loc, new_vert)) return;
//     update_position(v1, v2, new_vert);
//     size_t new_vid = new_vert.vid();
//     std::vector<ConcurrentTriMesh::Tuple> one_ring_edges =
//     get_one_ring_edges_for_vertex(new_vert); for (ConcurrentTriMesh::Tuple edge : one_ring_edges)
//     {
//         ConcurrentTriMesh::Tuple tmp_tuple = switch_vertex(new_vert);
//         size_t vid = tmp_tuple.vid();
//         double length = (m_vertex_positions[new_vid] - m_vertex_positions[vid]).squaredNorm();
//         ec_queue.push(ElementInQueue(edge, length));
//     }
//     release_vertex_mutex_in_stack(mutex_release_stack);
// }


// bool Edge2d::ParallelEdgeCollapse::collapse_shortest()
// {
//     // task-based version

//     tbb::task_arena arena(NUM_THREADS);
//     arena.execute([this]() {
//         Runner r(*this);
//         r.initialize();
//         r.tgwait();
//     });

//     //  thread simulate version

//     // std::vector<ConcurrentTriMesh::Tuple> edges = get_edges();
//     // tbb::concurrent_priority_queue<ElementInQueue, cmp_s> ec_queue;
//     // double shortest = std::numeric_limits<double>::max();
//     // for (auto& loc : edges) {
//     //     ConcurrentTriMesh::Tuple v2 = loc.switch_vertex(*this);
//     //     double length =
//     //         (m_vertex_positions[loc.get_vid()] - m_vertex_positions[v2.get_vid()]).squaredNorm();
//     //     if (length < shortest) shortest = length;
//     //     ec_queue.push(ElementInQueue(loc, length));
//     // }

//     // tbb::task_arena arena(NUM_THREADS);
//     // tbb::task_group tg;

//     // arena.execute([&ec_queue, &tg, this]() {
//     //     for (int i = 0; i < NUM_THREADS; ++i) {
//     //         tg.run([&] {
//     //             ElementInQueue eiq;
//     //             while (ec_queue.try_pop(eiq)) {
//     //                 // std::cout << global_cnt++ << std::endl;
//     //                 collapse_shortest_stuff(eiq, ec_queue);
//     //             }
//     //         });
//     //     }

//     //     tg.wait();
//     // });

//     return true;
// }