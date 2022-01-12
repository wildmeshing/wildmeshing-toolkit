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


using namespace Edge2d;

// #define NUM_THREADS 4
// #define STATISTICS

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
    int invalid_cnt = 0;
    tbb::spin_mutex rw_lock;
    tbb::spin_mutex conflict_lock;
    tbb::spin_mutex total_lock;
    tbb::spin_mutex success_lock;
    tbb::spin_mutex invalid_lock;
    tbb::spin_mutex fail_collapse_lock;
    tbb::spin_mutex retry_lock;
    tbb::spin_mutex skipped_lock;

public:
    int conflict_cnt = 0;
    int total_cnt = 0;
    int fail_collapse_cnt = 0;
    int retry_cnt = 0;
    int skipped_cnt = 0;

    Runner(ParallelEdgeCollapse& mesh, int tvc)
        : m(mesh)
        , target_vertex_count(tvc)
    {}

    void print_status()
    {
        std::cout << "total_cnt: " << total_cnt << std::endl;
        std::cout << "conflict_cnt: " << conflict_cnt << std::endl;
        std::cout << "retry_cnt: " << retry_cnt << std::endl;
        std::cout << "skippped_cnt: " << skipped_cnt << std::endl;
        std::cout << "success_cnt: " << cnt << std::endl;
        std::cout << "invalid_cnt: " << invalid_cnt << std::endl;
        std::cout << "fail_collapse_cnt: " << fail_collapse_cnt << std::endl;
    }

    void sleep_test()
    {
        usleep(10000);
        return;
    }

    void initialize()
    {
        // for (int i=0;i<target_vertex_count;i++){
        //     tg.run([&]() { sleep_test(); });
        // }
        // return;
        // std::cout<<"x"<<std::endl;

        // initialization timer
#ifdef STATISTICS
        igl::Timer timer;
        double time;
        timer.start();
#endif
        std::vector<wmtk::ConcurrentTriMesh::Tuple> edges = m.get_edges();
        double shortest = std::numeric_limits<double>::max();
        for (auto& loc : edges) {
            wmtk::ConcurrentTriMesh::Tuple v2 = loc.switch_vertex(m);
            double length =
                (m.m_vertex_positions[loc.vid()] - m.m_vertex_positions[v2.vid()]).squaredNorm();
            if (length < shortest) shortest = length;
            ec_queue.push(ElementInQueue(loc, length));
        }

        // std::cout<<ec_queue.size()<<std::endl;

        int num_tasks;
        if (target_vertex_count > ec_queue.size()) {
            num_tasks = ec_queue.size();
        } else {
            num_tasks = target_vertex_count;
        }

        for (int i = 0; i < num_tasks; ++i) {
            tg.run([&]() { this->runNext(); });
        }
#ifdef STATISTICS
        time = timer.getElapsedTimeInMilliSec();
        std::cout << time << std::endl;
#endif
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

#ifdef STATISTICS
        total_lock.lock();
        total_cnt++;
        total_lock.unlock();
#endif

        // std::cout << "in task" << std::endl;
        // std::cout << (m.n_vertices() == m.n_mutex()) << std::endl;


        auto loc = eiq.edge;
        double weight = eiq.weight;

        // auto eid = loc;
        // std::cout << "collpase edge_v_id: " << eid.vid() << " " << eid.switch_vertex(m).vid()
        //           << std::endl;
        // check if the edge tuple is valid
        if (!loc.is_valid(m)) {
#ifdef STATISTICS
            invalid_lock.lock();
            invalid_cnt++;
            invalid_lock.unlock();
#endif
            return;
        }

        wmtk::ConcurrentTriMesh::Tuple new_vert;
        // assert(m.check_mesh_connectivity_validity());

        std::vector<size_t> mutex_release_stack;
        // std::cout << "before set lock" << std::endl;
        if (!m.try_set_edge_mutex_two_ring(loc, mutex_release_stack)) {
            eiq.times_skipped++;
#ifdef STATISTICS
            conflict_lock.lock();
            conflict_cnt++;
            conflict_lock.unlock();
#endif
            if (eiq.times_skipped > 10) {
#ifdef STATISTICS
                skipped_lock.lock();
                skipped_cnt++;
                skipped_lock.unlock();
#endif
                return;
            }

#ifdef STATISTICS
            retry_lock.lock();
            retry_cnt++;
            retry_lock.unlock();
#endif
            this->add(eiq);
            return;
        }
        // std::cout << "locked: " << mutex_release_stack.size() << std::endl;
        // std::cout << "vids: ";
        // for (int i = 0; i < mutex_release_stack.size(); i++) {
        //     std::cout << mutex_release_stack[i] << " ";
        // }
        // std::cout << std::endl;

        // usleep(10000);

        auto loc_copy = loc;
        auto v1 = loc_copy.vid();
        auto v2 = loc.switch_vertex(m).vid();

        m.add_new_vertex_mutex();

        if (!m.collapse_edge(loc, new_vert)) {
            int num_released = m.release_vertex_mutex_in_stack(mutex_release_stack);
            // std::cout << "released: " << num_released << std::endl;
#ifdef STATISTICS
            fail_collapse_lock.lock();
            fail_collapse_cnt++;
            fail_collapse_lock.unlock();
#endif
            return;
        }

        rw_lock.lock();
        cnt++;
        // if (cnt % 100 == 0) std::cout << " 100 more collpased" << std::endl;
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

void Edge2d::ParallelEdgeCollapse::collapse_shortest_stuff(
    tbb::concurrent_priority_queue<ElementInQueue, cmp_s>& ec_queue,
    int& target_vertex_count,
    int task_id)
{
    // std::cout<<task_id<<std::endl;
    int cnt = 0;
    int suc_cnt = 0;
    ElementInQueue eiq;
    while (ec_queue.try_pop(eiq)) {
        cnt++;
        // std::cout<<task_id<<std::endl;
        rw_lock.lock();
        if (target_vertex_count <= 0) {
            rw_lock.unlock();
            break;
        }
        rw_lock.unlock();

        auto loc = eiq.edge;
        double weight = eiq.weight;
        int loc_pid = m_vertex_partition_id[loc.vid()];

        if (!loc.is_valid(*this)) {
            continue;
        }

        wmtk::ConcurrentTriMesh::Tuple new_vert;
        // assert(m.check_mesh_connectivity_validity());

        std::vector<size_t> mutex_release_stack;
        if (!try_set_edge_mutex_two_ring(loc, mutex_release_stack)) {
            eiq.times_skipped++;
            if (eiq.times_skipped > 10) {
                continue;
            }
            ec_queue.push(eiq);
            continue;
        }

        // usleep(1000);

        auto loc_copy = loc;
        auto v1 = loc_copy.vid();
        auto v2 = loc.switch_vertex(*this).vid();

        add_new_vertex_mutex();

        if (!collapse_edge(loc, new_vert)) {
            int num_released = release_vertex_mutex_in_stack(mutex_release_stack);
            continue;
        }

        suc_cnt++;

        rw_lock.lock();
        target_vertex_count--;
        rw_lock.unlock();

        size_t new_vid = new_vert.vid();
        // update position
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
    std::cout << task_id << ": " << suc_cnt << std::endl;
}

bool Edge2d::ParallelEdgeCollapse::collapse_shortest(int target_vertex_count)
{
    // task-based version
    //     tbb::task_arena arena(NUM_THREADS);
    //     arena.execute([this, target_vertex_count]() {
    //         Runner r(*this, target_vertex_count);
    //         r.initialize();
    //         r.tgwait();
    // #ifdef STATISTICS
    //         r.print_status();
    // #endif
    //     });


    // thread-based version
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

    int tvc = target_vertex_count;

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


    // tbb::task_group tg;
    //     for(int i=0;i<NUM_THREADS;i++){
    //         // std::cout<<i<<std::endl;
    //         int j = i;
    //         tg.run([&] {
    //             collapse_shortest_stuff(ec_queue, tvc, j);
    //         });
    //     }
    // tg.wait();


    return true;
}
