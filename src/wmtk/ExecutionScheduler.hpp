#pragma once

#include "wmtk/TetMesh.h"
#include "wmtk/TriMesh.h"
#include "wmtk/TriMeshOperation.h"
#include "wmtk/utils/Logger.hpp"

// clang-format off
#include <functional>
#include <limits>
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

#include <tracy/Tracy.hpp>

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
    using OperatorFunc = std::function<std::optional<std::vector<Tuple>>(AppMesh&, const Tuple&)>;
    constexpr static bool IsTetMesh = std::is_base_of_v<wmtk::TetMesh, AppMesh>;
    // assume other than tetmesh we just have TriMesh
    using OperationType = std::conditional_t<IsTetMesh, OperatorFunc, TriMeshOperation>;
    /**
     * @brief A dictionary that registers names with operations.
     *
     */
    std::map<
        Op, // strings
        OperatorFunc>
        edit_operation_maps;
    std::map<
        Op, // strings
        std::shared_ptr<OperationType>>
        new_edit_operation_maps;
    /**
     * @brief Priority function (default to edge length)
     *
     */
    std::function<double(const AppMesh&, Op op, const Tuple&)> priority = [](auto&, auto, auto&) {
        return 0.;
    };
    /**
     * @brief check on wheather new operations should be added to the prioirity queue
     *
     */
    std::function<bool(double)> should_renew = [](auto) { return true; };
    /**
     * @brief renew neighboring Tuples after each operation depends on the operation
     *
     */
    std::function<std::vector<std::pair<Op, Tuple>>(const AppMesh&, Op, const std::vector<Tuple>&)>
        renew_neighbor_tuples =
            [](auto&, auto, auto&) -> std::vector<std::pair<Op, Tuple>> { return {}; };
    /**
     * @brief lock the vertices concerned depends on the operation
     *
     */
    std::function<bool(AppMesh&, const Tuple&, int task_id)> lock_vertices =
        [](const AppMesh&, const Tuple&, int task_id) { return true; };
    /**
     * @brief Stopping Criterion based on the whole mesh
        For efficiency, not every time is checked.
        In serial, this may go over all the elements. For parallel, this involves synchronization.
        So there is a checking frequency.
     *
     */
    std::function<bool(const AppMesh&)> stopping_criterion = [](const AppMesh&) {
        return true; // non-stop, process everything
    };
    /**
     * @brief checking frequency to decide whether to stop execution given the stopping criterion
     *
     */
    size_t stopping_criterion_checking_frequency = std::numeric_limits<size_t>::max();
    /**
     * @brief Should Process drops some Tuple from being processed.
         For example, if the energy is out-dated.
         This is in addition to calling tuple valid.
     *
     */
    std::function<bool(const AppMesh&, const std::tuple<double, Op, Tuple>& t)>
        is_weight_up_to_date = [](const AppMesh& m, const std::tuple<double, Op, Tuple>& t) {
            // always do.
            assert(std::get<2>(t).is_valid(m));
            return true;
        };
    /**
     * @brief
     *used to collect operations that are not finished and used for later re-execution
     */
    std::function<void(const AppMesh&, Op, const Tuple& t)> on_fail = [](auto&, auto, auto&) {};


    size_t num_threads = 1;

    /**
     * To Avoid mutual locking, retry limit is set, and then put in a serial queue in the end.
     *
     */
    size_t max_retry_limit = 10;

    /*
     * Add an operation to the set of operation types
     */
    template <typename OpType>
    void add_operation(std::shared_ptr<OpType> op, const std::string& name)
    {
        new_edit_operation_maps[name] = op;
    }
    /*
     * Add an operation using its default name
     */
    template <typename OpType>
    void add_operation(std::shared_ptr<OpType> op)
    {
        add_operation(op, op->name());
    }
    // convenience function for when we want to default construct the op but use a custom name
    template <typename OpType>
    void add_operation(const std::string& name)
    {
        auto op = std::make_shared<OpType>();
        add_operation(op, name);
    }
    // convenience function for when we want to default construct the op an use default name
    template <typename OpType>
    void add_operation()
    {
        auto op = std::make_shared<OpType>();
        add_operation(op);
    }
    /**
     * @brief Construct a new Execute Pass object. It contains the name-to-operation map and the
     *functions that define the rules for operations
     *@note the constructor is differentiated by the type of mesh, namingly wmtk::TetMesh or
     *wmtk::TriMesh
     */
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
                {"edge_swap_44",
                 [](AppMesh& m, const Tuple& t) -> std::optional<std::vector<Tuple>> {
                     std::vector<Tuple> ret;
                     if (m.swap_edge_44(t, ret))
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
            // keeping this trimesh behavior even though it really should be deprecated
            // add_operation<wmtk::TriMeshEdgeCollapseOperation>();
            // add_operation<wmtk::TriMeshSwapEdgeOperation>();
            // add_operation<wmtk::TriMeshSplitEdgeOperation>();
            // add_operation<wmtk::TriMeshSmoothVertexOperation>();
            // add_operation<wmtk::TriMeshConsolidateOperation>();
        }
    }

#if !defined(__cpp_concepts)
    template <bool Tet = IsTetMesh>
#endif
    ExecutePass(const std::map<Op, OperatorFunc>& customized_ops)
#if defined(__cpp_concepts)
        requires(std::is_base_of_v<wmtk::TetMesh, AppMesh>)
#endif
        : ExecutePass()
    {
#if !defined(__cpp_concepts)
        static_assert(Tet);
#endif
        if (!customized_ops.empty()) {
            edit_operation_maps.insert(customized_ops.begin(), customized_ops.end());
        }
    }
#if !defined(__cpp_concepts)
    template <bool Tet = IsTetMesh>
#endif
    ExecutePass(const AppMesh& m)
#if defined(__cpp_concepts)
        requires(std::is_base_of_v<wmtk::TriMesh, AppMesh>)
#endif
    {
#if !defined(__cpp_concepts)
        static_assert(!Tet);
#endif
        new_edit_operation_maps = m.get_operations();
    }


private:
    void operation_cleanup(AppMesh& m)
    { //
        // class ResourceManger
        // what about RAII mesh edit locking?
        // release mutex, but this should be implemented in TetMesh class.
        if constexpr (policy == ExecutionPolicy::kSeq)
            return;
        else {
            m.release_vertex_mutex_in_stack();
        }
    }

    size_t get_partition_id(const AppMesh& m, const Tuple& e)
    {
        if constexpr (policy == ExecutionPolicy::kSeq) return 0;
        if constexpr (std::is_base_of<wmtk::TetMesh, AppMesh>::value)
            return m.get_partition_id(e);
        else if constexpr (std::is_base_of<wmtk::TriMesh, AppMesh>::value) // TODO: make same
                                                                           // interface.
            return m.vertex_attrs[e.vid(m)].partition_id; // TODO: this is temporary.
        return 0;
    }

public:
    /**
     * @brief Executes the operations for an application when the lambda function is invoked. The
     * rules that are customizly defined for applications are applied.
     *
     * @param m
     * @param operation_tuples a vector of pairs of operation's name and the Tuple to be operated on
     * @returns true if finished successfully
     */
    bool operator()(AppMesh& m, const std::vector<std::pair<Op, Tuple>>& operation_tuples)
    {
        auto cnt_update = std::atomic<int>(0);
        auto cnt_success = std::atomic<int>(0);
        auto cnt_fail = std::atomic<int>(0);
        auto stop = std::atomic<bool>(false);
        using Elem = std::tuple<double, Op, Tuple, size_t>;
        auto queues = std::vector<tbb::concurrent_priority_queue<Elem>>(num_threads);
        auto final_queue = tbb::concurrent_priority_queue<Elem>();

        auto run_single_queue = [&](auto& Q, int task_id) {
            auto ele_in_queue = Elem();
            while ([&]() { return Q.try_pop(ele_in_queue); }()) {
                auto& [weight, op, tup, retry] = ele_in_queue;
                if (!tup.is_valid(m)) continue;
                std::vector<Elem> renewed_elements;
                {
                    auto locked_vid = lock_vertices(
                        m,
                        tup,
                        task_id); // Note that returning `Tuples` would be invalid.
                    if (!locked_vid) {
                        retry++;
                        if (retry < max_retry_limit) {
                            Q.emplace(ele_in_queue);
                        } else {
                            retry = 0;
                            final_queue.emplace(ele_in_queue);
                        }
                        continue;
                    }
                    if (tup.is_valid(m)) {
                        if (!is_weight_up_to_date(
                                m,
                                std::tuple<double, Op, Tuple>(
                                    std::get<0>(ele_in_queue),
                                    std::get<1>(ele_in_queue),
                                    std::get<2>(ele_in_queue)))) {
                            operation_cleanup(m);
                            continue;
                        } // this can encode, in qslim, recompute(energy) == weight.
                        std::vector<std::pair<Op, Tuple>> renewed_tuples;
                        if constexpr (std::is_base_of<wmtk::TriMesh, AppMesh>::value) {
                            auto& my_op = (*new_edit_operation_maps[op]);
                            const bool success = my_op(m, tup);
                            if (success) {
                                // TODO: renew_neighbor_tuples should allow for triangles, edges, or
                                // vertices
                                renewed_tuples =
                                    renew_neighbor_tuples(m, op, my_op.modified_triangles(m));
                                cnt_success++;
                                cnt_update++;
                            } else {
                                on_fail(m, op, tup);
                                cnt_fail++;
                            }
                        } else {
                            auto newtup = edit_operation_maps[op](m, tup);
                            if (newtup) {
                                renewed_tuples = renew_neighbor_tuples(m, op, newtup.value());
                                cnt_success++;
                                cnt_update++;
                            } else {
                                on_fail(m, op, tup);
                                cnt_fail++;
                            }
                        }

                        for (auto& [o, e] : renewed_tuples) {
                            auto val = priority(m, o, e);
                            if (should_renew(val)) renewed_elements.emplace_back(val, o, e, 0);
                        }
                    }
                    operation_cleanup(m); // Maybe use RAII
                }
                for (auto& e : renewed_elements) {
                    Q.emplace(e);
                }

                if (stop.load(std::memory_order_acquire)) return;
                if (cnt_success > stopping_criterion_checking_frequency) {
                    if (stopping_criterion(m)) {
                        stop.store(true);
                        return;
                    }
                    cnt_update.store(0, std::memory_order_release);
                }
                FrameMark;
            }
        };

        if constexpr (policy == ExecutionPolicy::kSeq) {
            for (auto& [op, e] : operation_tuples) {
                if (!e.is_valid(m)) continue;
                final_queue.emplace(priority(m, op, e), op, e, 0);
            }
            run_single_queue(final_queue, 0);
        } else {
            for (auto& [op, e] : operation_tuples) {
                if (!e.is_valid(m)) continue;
                queues[get_partition_id(m, e)].emplace(priority(m, op, e), op, e, 0);
            }
            // Comment out parallel: work on serial first.
            tbb::task_arena arena(num_threads);
            tbb::task_group tg;
            arena.execute([&queues, &run_single_queue, &tg]() {
                for (int task_id = 0; task_id < queues.size(); task_id++) {
                    tg.run([&run_single_queue, &queues, task_id] {
                        run_single_queue(queues[task_id], task_id);
                    });
                }
                tg.wait();
            });
            logger().debug("Parallel Complete, remains element {}", final_queue.size());
            run_single_queue(final_queue, 0);
        }

        logger().info("cnt_success {} cnt_fail {}", cnt_success, cnt_fail);
        return true;
    }
};
} // namespace wmtk
