#pragma once

#include "TetMesh.h"
#include "oneapi/tbb/concurrent_priority_queue.h"
#include "wmtk/TriMesh.h"

#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_queue.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/spin_mutex.h>
#include <tbb/task_arena.h>
#include <tbb/task_group.h>

#include <atomic>
#include <cassert>
#include <cstddef>
#include <queue>
#include <type_traits>
// in utils
namespace wmtk::utils {
inline std::vector<TetMesh::Tuple> one_edge_ring_tuple(const TetMesh& m, const TetMesh::Tuple& t)
{
    return {};
}
} // namespace wmtk::utils

namespace wmtk {
enum class ExecutionPolicy { kSeq, kUnSeq, kPartition, kColor, kMax };

template <class AppMesh, ExecutionPolicy policy = ExecutionPolicy::kSeq>
struct ExecutePass
{
    using Tuple = typename AppMesh::Tuple;
    using Op = std::string;

    // A dictionary that registers names with operations.
    std::map<Op, std::function<Tuple(AppMesh&, const Tuple&)>> edit_operation_maps;

    // Priority function (default to edge length)
    std::function<double(const AppMesh&, const Tuple&)> priority;

    // Renew Neighboring Tuples
    // Right now, use pre-implemented functions to get one edge ring.
     // TODO: Ideally, this depend on both operation and priority criterion.
    std::function<std::vector<Tuple>(const AppMesh&, const Tuple&)>
        renew_neighbor_tuples = utils::one_edge_ring_tuple;

    // lock vertices: should depend on the operation
    // returns a range of vertices that we need to acquire and lock.
    // trivial in serial version
    std::function<std::vector<size_t>(const AppMesh&, const Tuple&)> lock_vertices =
        [](const AppMesh&, const Tuple&) {
            return std::vector<size_t>();
        };
    
    // Stopping Criterion based on the whole mesh
    // For efficiency, not every time is checked.
    // In serial, this may go over all the elements. For parallel, this involves synchronization.
    // So there is a checking frequency.
    std::function<bool(const AppMesh&)> stopping_criterion = [](const AppMesh&) {
        return false; // non-stop, process everything
    };
    size_t stopping_criterion_checking_frequency = 100;

    // Should Process drops some Tuple from being processed.
    // For example, if the energy is out-dated.
    // This is in addition to calling tuple valid.
    std::function<bool(const AppMesh&, const std::tuple<double, Op, Tuple>& t)> should_process =
        [](const AppMesh& m, const std::tuple<double, Op, Tuple>& t) {
            // always do. 
            assert(t.is_valid(m));
            return true;
        };


    size_t num_threads = 1;

    ExecutePass()
    {
        if constexpr (std::is_base_of<wmtk::TriMesh, AppMesh>::value) {
            edit_operation_maps = {
                {"edge_collapse",
                 [](AppMesh& m, Tuple& t) {
                     std::vector<Tuple> ret;
                     m.collapse_edge(t, ret);
                     return ret.front();
                 }},
                {"edge_swap",
                 [](AppMesh& m, Tuple& t) {
                     Tuple ret;
                     m.swap_edge(t, ret);
                     return ret;
                 }},
                {"edge_split",
                 [](AppMesh& m, Tuple& t) {
                     std::vector<Tuple> ret;
                     m.collapse_edge(t, ret);
                     return ret.front();
                 }},
                {"face_swap",
                 [](AppMesh& m, Tuple& t) {
                     Tuple ret;
                     m.swap_face(t, ret);
                     return ret;
                 }},
                {"vertex_smooth", [](AppMesh& m, Tuple& t) {
                     Tuple ret;
                     m.smooth_vertex(t, ret);
                     return ret;
                 }}};
        }
        if constexpr (std::is_base_of<wmtk::TetMesh, AppMesh>::value) {
            edit_operation_maps = {
                {"edge_collapse",
                 [](AppMesh& m, Tuple& t) {
                     std::vector<Tuple> ret;
                     m.collapse_edge(t, ret);
                     return ret.front();
                 }},
                {"edge_swap",
                 [](AppMesh& m, Tuple& t) {
                     Tuple ret;
                     m.swap_edge(t, ret);
                     return ret;
                 }},
                {"edge_split",
                 [](AppMesh& m, Tuple& t) {
                     std::vector<Tuple> ret;
                     m.collapse_edge(t, ret);
                     return ret.front();
                 }},
                {"edge_swap",
                 [](AppMesh& m, Tuple& t) {
                     Tuple ret;
                     m.swap_edge(t, ret);
                     return ret;
                 }},
                {"vertex_smooth", [](AppMesh& m, Tuple& t) {
                     Tuple ret;
                     m.smooth_vertex(t, ret);
                     return ret;
                 }}};
        }
    };

private:
    void operation_cleanup(AppMesh&, const std::vector<size_t>&)
    { //
      // release mutex, but this should be implemented in TetMesh class.
    }
    class ResourceManger
    {
    };

public:
    bool operator()(AppMesh& m, std::vector<std::pair<Op, Tuple>> operation_tuples)
    {
        static_assert(policy == ExecutionPolicy::kSeq, "start with serial version");
        auto cnt_update = std::atomic<int>(0);
        auto stop = std::atomic<bool>(false);
        auto run_single_queue = [&](auto& Q) {
            auto ele_in_queue = std::tuple<double, Op, Tuple>();
            while (Q.try_pop(ele_in_queue)) {
                auto& [weight, op, tup] = ele_in_queue;
                if (!tup.is_valid(m)) continue;
                if (!should_process(ele_in_queue))
                    continue; // this can encode, in qslim, recompute(energy) == weight.
                std::vector<Tuple> renewed_tuples;
                {
                    auto locked_vid =
                        lock_vertices(m, tup); // Note that returning `Tuples` would be invalid.
                    auto newtup = edit_operation_maps[op](m, tup);
                    renewed_tuples = renew_neighbor_tuples(newtup);
                    operation_cleanup(m, locked_vid); // Maybe use RAII
                }
                cnt_update++;
                for (auto& e : renewed_tuples) Q.emplace(priority(e), op, e);
            }

            if (stop.load(std::memory_order_acquire)) return;
            if (cnt_update > stopping_criterion_checking_frequency) {
                if (stopping_criterion(m)) {
                    stop.store(true);
                    return;
                }
                cnt_update.store(0, std::memory_order_release);
            }
        };

        auto queue =
            std::vector<tbb::concurrent_priority_queue<std::tuple<double, Op, Tuple>>>(num_threads);
        for (auto& [op, e] : operation_tuples) {
            queue[0].emplace(priority(e), op, e);
        }
        run_single_queue(queue[0]);
        if constexpr (policy != ExecutionPolicy::kSeq) {
        // Comment out parallel: work on serial first.
        tbb::task_arena arena(num_threads);
        tbb::task_group tg;
        arena.execute([&queue, &run_single_queue, &tg]() {
            for (auto& q: queue) {
                tg.run([&run_single_queue, &q] { run_single_queue(q); });
            }
        });
        arena.execute([&] { tg.wait(); });
        }
        return true;
    }
};

} // namespace wmtk


// Example client code as follows
// void MyMesh::edge_collapsing()
// {
//     auto compute_length = [](const Tuple&) -> double {};
//     auto renew =
//         wmtk::utils::renew_all_edges; // return all the edges in one (or two?) ring neighbor
//     auto op_tup = std::vector<std::pair<Op, Tuple>>();
//     for (auto& e : get_edges()) op_tup.emplace_back(Op::kCollapse, e);
//     execute_pass(ExecutionPolicy::kPartition, op_tup, compute_length, renew);
// }

// void MyMesh::adaptive_edge_split()
// {
//     auto compute_length_ratio = [](const Tuple& e) -> double { return length(e) / desirable(e) };
//     auto renew =
//         wmtk::utils::renew_all_edges; // return all the edges in one (or two?) ring neighbor
//     auto op_tup = std::vector<std::pair<Op, Tuple>>();
//     for (auto& e : get_edges()) {
//         if (within_selected_region(e)) op_tup.emplace_back(Op::COLLAPSE, e);
//     }
//     execute_pass(ExecutionPolicy::kPartition, op_tup, compute_length_ratio, renew);
// }

// void MyMesh::mixed_swap_pass()
// {
//     auto compute_quality = [](const Tuple&) -> double {};
//     auto renew =
//         wmtk::utils::renew_all_edges; // return all the edges in one (or two?) ring neighbor
//     auto op_tup = std::vector<std::pair<Op, Tuple>>();
//     for (auto& e : get_edges()) op_tup.emplace_back(Op::kEdgeSwap, e);
//     for (auto& e : get_faces()) op_tup.emplace_back(Op::kFaceSwap, e);
//     execute_pass(ExecutionPolicy::kPartition, op_tup, compute_quality, renew);
// }
