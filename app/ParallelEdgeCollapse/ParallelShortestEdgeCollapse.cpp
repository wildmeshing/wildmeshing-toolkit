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
#include <atomic>
#include <wmtk/utils/Logger.hpp>
#include "ParallelEdgeCollapse.h"


using namespace Edge2d;

bool Edge2d::ParallelEdgeCollapse::collapse_before(const Tuple& t)
{
    // set locks and re-push
    auto loc = eiq_cache.local().edge;
    if (!try_set_edge_mutex_two_ring(loc, mutex_release_stack_cache.local())) {
        eiq_cache.local().times_skipped++;
        if (eiq_cache.local().times_skipped > retry_limit) {
            return false;
        }
        ec_queues[task_id_cache.local()].push(eiq_cache.local());
        return false;
    }

    if (check_link_condition(t)) {
        collapse_cache.local().v1p = m_vertex_positions[t.vid()];
        collapse_cache.local().v2p = m_vertex_positions[t.switch_vertex(*this).vid()];
        collapse_cache.local().v1pid = m_vertex_partition_id[t.vid()];
        return true;
    }
    return false;
}

bool Edge2d::ParallelEdgeCollapse::collapse_after(const Tuple& t)
{
    size_t new_vid = t.vid();
    m_vertex_positions[new_vid] = (collapse_cache.local().v1p + collapse_cache.local().v2p) / 2.0;
    m_vertex_partition_id[new_vid] = collapse_cache.local().v1pid;
    return true;
}


void Edge2d::ParallelEdgeCollapse::collapse_shortest_stuff(
    tbb::concurrent_priority_queue<ElementInQueue, cmp_s>& ec_queue,
    std::atomic_int& target_vertex_count,
    int task_id)
{
    int cnt = 0;
    int suc_cnt = 0;
    // ElementInQueue eiq;
    while (ec_queue.try_pop(eiq_cache.local())) {
        cnt++;
        if (target_vertex_count <= 0) {
            break;
        }

        auto loc = eiq_cache.local().edge;
        double weight = eiq_cache.local().weight;
        int loc_pid = m_vertex_partition_id[loc.vid()];

        if (!loc.is_valid(*this)) {
            continue;
        }

        wmtk::TriMesh::Tuple new_vert;

        if (!collapse_edge(loc, new_vert)) {
            int num_released = release_vertex_mutex_in_stack(mutex_release_stack_cache.local());
            continue;
        }

        suc_cnt++;
        target_vertex_count--;

        size_t new_vid = new_vert.vid();
        std::vector<wmtk::TriMesh::Tuple> one_ring_edges = get_one_ring_edges_for_vertex(new_vert);

        for (wmtk::TriMesh::Tuple edge : one_ring_edges) {
            size_t vid = edge.vid();
            double length = (m_vertex_positions[new_vid] - m_vertex_positions[vid]).squaredNorm();
            ElementInQueue new_eiq(edge, length);
            ec_queue.push(new_eiq);
        }
        int num_released = release_vertex_mutex_in_stack(mutex_release_stack_cache.local());
    }
}

bool Edge2d::ParallelEdgeCollapse::collapse_shortest(int target_vertex_count)
{
    partition_mesh();
    std::vector<TriMesh::Tuple> edges = get_edges();
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

    arena.execute([this, &tvc, &tg]() {
        for (int i = 0; i < this->NUM_THREADS; i++) {
            int j = i;
            tg.run([&tvc, j, this] { collapse_shortest_stuff(this->ec_queues[j], tvc, j); });
        }
    });

    arena.execute([&] { tg.wait(); });

    return true;
}
