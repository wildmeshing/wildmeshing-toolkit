#pragma once

#include "wmtk/TetMesh.h"
#include "wmtk/TriMesh.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_queue.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/spin_mutex.h>
#include <tbb/task_arena.h>
#include <tbb/task_group.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <Tracy.hpp>

#include <atomic>
#include <cassert>
#include <cstddef>
#include <queue>
#include <stdexcept>
#include <type_traits>

namespace wmtk {
enum class ExecutionPolicy { kSeq, kUnSeq, kPartition, kColor, kMax };

using Op = std::string;

template <class AppMesh, ExecutionPolicy policy = ExecutionPolicy::kSeq>
struct ExecutePass
{
    using Tuple = typename AppMesh::Tuple;
    // A dictionary that registers names with operations.
    std::map<
        Op, // strings
        std::function<std::optional<std::vector<Tuple>>(AppMesh&, const Tuple&)>>
        edit_operation_maps;

    // Priority function (default to edge length)
    std::function<double(const AppMesh&, Op op, const Tuple&)> priority = [](auto&, auto, auto&) {
        return 0.;
    };

    // Renew Neighboring Tuples
    // Right now, use pre-implemented functions to get one edge ring.
    // TODO: Ideally, this depend on both operation and priority criterion.
    std::function<std::vector<std::pair<Op, Tuple>>(const AppMesh&, Op, const std::vector<Tuple>&)>
        renew_neighbor_tuples =
            [](auto&, auto, auto&) -> std::vector<std::pair<Op, Tuple>> { return {}; };

    // lock vertices: should depend on the operation
    // returns a range of vertices that we need to acquire and lock.
    // trivial in serial version
    std::function<std::optional<std::vector<size_t>>(AppMesh&, const Tuple&)> lock_vertices =
        [](const AppMesh&, const Tuple&) { return std::vector<size_t>(); };

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
            assert(std::get<2>(t).is_valid(m));
            return true;
        };


    size_t num_threads = 1;

    ExecutePass()
    {
        if constexpr (std::is_base_of<wmtk::TetMesh, AppMesh>::value) {
            edit_operation_maps = {
                {"edge_collapse",
                 [](AppMesh& m, const Tuple& t) -> std::optional<std::vector<Tuple>> {
                     std::vector<Tuple> ret;
                     if (m.collapse_edge(t, ret))
                         return ret;
                     else
                         return {};
                 }},
                {"edge_swap",
                 [](AppMesh& m, const Tuple& t) -> std::optional<std::vector<Tuple>> {
                     std::vector<Tuple> ret;
                     if (m.swap_edge(t, ret))
                         return ret;
                     else
                         return {};
                 }},
                {"edge_split",
                 [](AppMesh& m, const Tuple& t) -> std::optional<std::vector<Tuple>> {
                     std::vector<Tuple> ret;
                     if (m.split_edge(t, ret))
                         return ret;
                     else
                         return {};
                 }},
                {"face_swap",
                 [](AppMesh& m, const Tuple& t) -> std::optional<std::vector<Tuple>> {
                     std::vector<Tuple> ret;
                     if (m.swap_face(t, ret))
                         return ret;
                     else
                         return {};
                 }},
                {"vertex_smooth",
                 [](AppMesh& m, const Tuple& t) -> std::optional<std::vector<Tuple>> {
                     if (m.smooth_vertex(t))
                         return std::vector<Tuple>{};
                     else
                         return {};
                 }}};
        }
        if constexpr (std::is_base_of<wmtk::TriMesh, AppMesh>::value) {
            edit_operation_maps = {
                {"edge_collapse",
                 [](AppMesh& m, const Tuple& t) -> std::optional<std::vector<Tuple>> {
                     std::vector<Tuple> ret;
                     if (m.collapse_edge(t, ret))
                         return ret;
                     else
                         return {};
                 }},
                {"edge_swap",
                 [](AppMesh& m, const Tuple& t) -> std::optional<std::vector<Tuple>> {
                     std::vector<Tuple> ret;
                     if (m.swap_edge(t, ret))
                         return ret;
                     else
                         return {};
                 }},
                {"edge_split",
                 [](AppMesh& m, const Tuple& t) -> std::optional<std::vector<Tuple>> {
                     std::vector<Tuple> ret;
                     if (m.split_edge(t, ret))
                         return ret;
                     else
                         return {};
                 }},
                {"vertex_smooth",
                 [](AppMesh& m, const Tuple& t) -> std::optional<std::vector<Tuple>> {
                     if (m.smooth_vertex(t))
                         return std::vector<Tuple>{};
                     else
                         return {};
                 }}};
        }
    };

private:
    void operation_cleanup(AppMesh& m, std::optional<std::vector<size_t>>& stack)
    { //
        // class ResourceManger
        // what about RAII mesh edit locking?
        // release mutex, but this should be implemented in TetMesh class.
        if constexpr (policy == ExecutionPolicy::kSeq)
            return;
        else {
            if (stack) m.release_vertex_mutex_in_stack(stack.value());
        }
    }

    size_t get_partition_id(const AppMesh& m, const Tuple&e)
    {
        if constexpr(policy == ExecutionPolicy::kSeq) return 0;
        if constexpr(std::is_base_of<wmtk::TetMesh, AppMesh>::value)
            return m.m_vertex_partition_id[e.vid(m)];
        else if constexpr(std::is_base_of<wmtk::TriMesh, AppMesh>::value) // TODO: make same interface.
            return m.m_vertex_partition_id[e.vid()];
        return 0;
    }

public:
    bool operator()(AppMesh& m, const std::vector<std::pair<Op, Tuple>>& operation_tuples)
    {
        auto cnt_update = std::atomic<int>(0);
        auto stop = std::atomic<bool>(false);
        auto run_single_queue = [&](auto& Q) {
            using Elem = std::tuple<double, Op, Tuple>;
            auto ele_in_queue = Elem();
            while (Q.try_pop(ele_in_queue)) {
                auto& [weight, op, tup] = ele_in_queue;
                if (!tup.is_valid(m)) continue;
                if (!should_process(m, ele_in_queue))
                    continue; // this can encode, in qslim, recompute(energy) == weight.
                std::vector<Elem> renewed_elements;
                {
                    auto locked_vid =
                        lock_vertices(m, tup); // Note that returning `Tuples` would be invalid.
                    if (!locked_vid) {
                        Q.emplace(ele_in_queue);
                        continue;
                    }
                    if (tup.is_valid(m)) {
                        auto newtup = edit_operation_maps[op](m, tup);
                        std::vector<std::pair<Op, Tuple>> renewed_tuples;
                        if (newtup) renewed_tuples = renew_neighbor_tuples(m, op, newtup.value());
                        for (auto& [o, e] : renewed_tuples) {
                            renewed_elements.emplace_back(priority(m, o, e), o, e);
                        }
                        cnt_update++;
                    }
                    operation_cleanup(m, locked_vid); // Maybe use RAII
                }
                for (auto& e : renewed_elements) Q.emplace(e);

                if (stop.load(std::memory_order_acquire)) return;
                if (cnt_update > stopping_criterion_checking_frequency) {
                    if (stopping_criterion(m)) {
                        stop.store(true);
                        return;
                    }
                    cnt_update.store(0, std::memory_order_release);
                }
                FrameMark;
            }
        };

        auto queues =
            std::vector<tbb::concurrent_priority_queue<std::tuple<double, Op, Tuple>>>(num_threads);
        if constexpr (policy == ExecutionPolicy::kSeq) {
            for (auto& [op, e] : operation_tuples) {
                queues[0].emplace(priority(m, op, e), op, e);
            }
            run_single_queue(queues[0]);
        } else {
            for (auto& [op, e] : operation_tuples) {
                //
                queues[get_partition_id(m,e)].emplace(priority(m, op, e), op, e);
            }
            // Comment out parallel: work on serial first.
            tbb::task_arena arena(num_threads);
            tbb::task_group tg;
            arena.execute([&queues, &run_single_queue, &tg]() {
                for (auto& q : queues) {
                    tg.run([&run_single_queue, &q] { run_single_queue(q); });
                }
            });
            arena.execute([&] { tg.wait(); });
        }
        return true;
    }
};
} // namespace wmtk
