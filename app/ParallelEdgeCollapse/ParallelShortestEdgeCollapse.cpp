#include <igl/Timer.h>
#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_queue.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/spin_mutex.h>
#include <tbb/task_arena.h>
#include <tbb/task_group.h>
#include <unistd.h>
#include <wmtk/ConcurrentTriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <wmtk/utils/Logger.hpp>
#include "ParallelEdgeCollapse.h"
#include <atomic>


using namespace Edge2d;

bool Edge2d::ParallelEdgeCollapse::collapse_before(const Tuple& t)
{
    if (check_link_condition(t)) {
        collapse_cache.v1p = m_vertex_positions[t.vid()];
        collapse_cache.v2p = m_vertex_positions[t.switch_vertex(*this).vid()];
        return true;
    }
    return false;
}

bool Edge2d::ParallelEdgeCollapse::collapse_after(const Tuple& t)
{
    return true;
}


// TODO ConcurrentTriMesh::Tuple -> TriMesh::Tuple
void Edge2d::ParallelEdgeCollapse::collapse_shortest_stuff(
    tbb::concurrent_priority_queue<ElementInQueue, cmp_s>& ec_queue,
    std::atomic_int &target_vertex_count,
    int task_id)
{
    int cnt = 0;
    int suc_cnt = 0;
    ElementInQueue eiq;
    while (ec_queue.try_pop(eiq)) {
        cnt++;
        if (target_vertex_count <= 0) {
            break;
        }

        auto loc = eiq.edge;
        double weight = eiq.weight;
        int loc_pid = m_vertex_partition_id[loc.vid()];

        if (!loc.is_valid(*this)) {
            continue;
        }

        wmtk::TriMesh::Tuple new_vert;

        std::vector<size_t> mutex_release_stack;
        if (!try_set_edge_mutex_two_ring(loc, mutex_release_stack)) {
            eiq.times_skipped++;
            if (eiq.times_skipped > 10) {
                continue;
            }
            ec_queue.push(eiq);
            continue;
        }


        auto loc_copy = loc;
        auto v1 = loc_copy.vid();
        auto v2 = loc.switch_vertex(*this).vid();

        if (!collapse_edge(loc, new_vert)) {
            int num_released = release_vertex_mutex_in_stack(mutex_release_stack);
            continue;
        }

        suc_cnt++;

        // rw_lock.lock();
        target_vertex_count--;
        // rw_lock.unlock();

        size_t new_vid = new_vert.vid();
        // update position
        //TODO move to paralleledgecollaspe::collpase_after
        m_vertex_positions[new_vid] = (m_vertex_positions[v1] + m_vertex_positions[v2]) / 2.0;
        m_vertex_partition_id[new_vid] = loc_pid;


        std::vector<wmtk::ConcurrentTriMesh::Tuple> one_ring_edges =
            get_one_ring_edges_for_vertex(new_vert);

        for (wmtk::ConcurrentTriMesh::Tuple edge : one_ring_edges) {
            size_t vid = edge.vid();
            double length = (m_vertex_positions[new_vid] - m_vertex_positions[vid]).squaredNorm();
            ElementInQueue new_eiq(edge, length);
            ec_queue.push(new_eiq);
        }
        int num_released = release_vertex_mutex_in_stack(mutex_release_stack);
    }
}

bool Edge2d::ParallelEdgeCollapse::collapse_shortest(int target_vertex_count)
{
    std::vector<TriMesh::Tuple> edges = get_edges();
    std::vector<tbb::concurrent_priority_queue<ElementInQueue, cmp_s>> ec_queues(NUM_THREADS);
    double shortest = std::numeric_limits<double>::max();
    for (auto& loc : edges) {
        TriMesh::Tuple v2 = loc.switch_vertex(*this);
        double length =
            (m_vertex_positions[loc.vid()] - m_vertex_positions[v2.vid()]).squaredNorm();
        if (length < shortest) shortest = length;
        ec_queues[m_vertex_partition_id[loc.vid()]].push(ElementInQueue(loc, length));
    }

    std::atomic_int tvc(target_vertex_count);

    tbb::task_arena arena(NUM_THREADS);
    tbb::task_group tg;

    arena.execute([this, &tvc, &ec_queues, &tg]() {
        for (int i = 0; i < this->NUM_THREADS; i++) {
            // std::cout<<i<<std::endl;
            int j = i;
            tg.run([&ec_queues, &tvc, j, this] { collapse_shortest_stuff(ec_queues[j], tvc, j); });
        }
    });

    arena.execute([&] { tg.wait(); });

    return true;
}
