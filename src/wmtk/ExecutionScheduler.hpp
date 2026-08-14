#pragma once

#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/threading/concurrent_priority_queue.hpp>
#include <wmtk/threading/serial_priority_queue.hpp>
#include <wmtk/threading/task_group.hpp>
#include <wmtk/utils/Logger.hpp>

// clang-format off
#include <functional>
#include <limits>
#include <wmtk/utils/DisableWarnings.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <atomic>
#include <cassert>
#include <cstddef>
#include <queue>
#include <stdexcept>
#include <type_traits>

namespace wmtk {
enum class ExecutionPolicy { kSeq, kUnSeq, kPartition, kColor, kMax };

using Op = std::string;

template <class AppMesh>
struct ExecutePass
{
    using Tuple = typename AppMesh::Tuple;
    /**
     * @brief A dictionary that registers names with operations.
     *
     */
    std::map<
        Op, // strings
        std::function<std::optional<std::vector<Tuple>>(AppMesh&, const Tuple&)>>
        edit_operation_maps;
    /**
     * @brief Priority function (default to edge length)
     *
     */
    std::function<double(const AppMesh&, Op op, const Tuple&)> priority =
        [](const AppMesh&, Op, const Tuple&) { return 0.; };
    /**
     * @brief check on wheather new operations should be added to the priority queue
     *
     */
    std::function<bool(double)> should_renew = [](double) { return true; };
    /**
     * @brief renew neighboring Tuples after each operation depends on the operation
     *
     */
    std::function<std::vector<std::pair<Op, Tuple>>(const AppMesh&, Op, const std::vector<Tuple>&)>
        renew_neighbor_tuples =
            [](const AppMesh&, Op, const std::vector<Tuple>&) -> std::vector<std::pair<Op, Tuple>> {
        return {};
    };
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
        return false; // non-stop, process everything
    };
    /**
     * @brief Cumulative successful operations before `stopping_criterion` is first consulted.
     *
     * Despite the name this is a threshold, not a period: the count it is tested against is
     * never reset, so once the pass has had this many successes the criterion is consulted after
     * every subsequent operation. Both current users rely on exactly that -- they set the
     * criterion to `return true` and the threshold to the number of collapses needed to reach a
     * target vertex count, making this a decimation counter that stops the pass on its first
     * check. It is not a "check every N operations" knob, and writing a genuinely periodic
     * criterion against it would evaluate that criterion on every operation forever after.
     *
     * Left at the default, the criterion is never consulted at all, which is the case for every
     * tetwild/triwild/simwild pass.
     *
     * (There used to be a `cnt_update` member here that was incremented per success and reset
     * inside the check, as if the threshold were a period. Nothing ever read it -- the reset was
     * on a branch the always-true criteria above never reach -- so it was one more contended
     * atomic on the hot path buying nothing, and it is gone.)
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
     * @brief used to collect operations that are not finished and used for later re-execution
     */
    std::function<void(const AppMesh&, Op, const Tuple& t)> on_fail =
        [](const AppMesh&, Op, const Tuple& t) {};

    ExecutionPolicy policy;

    int num_threads = 1;

    /**
     * To Avoid mutual locking, retry limit is set, and then put in a serial queue in the end.
     *
     */
    size_t max_retry_limit = 10;
    /**
     * @brief Construct a new Execute Pass object. It contains the name-to-operation map and the
     *functions that define the rules for operations
     *@note the constructor is differentiated by the type of mesh, namingly wmtk::TetMesh or
     *wmtk::TriMesh
     */
    ExecutePass(const ExecutionPolicy& policy_ = ExecutionPolicy::kSeq)
        : policy(policy_)
    {
        if constexpr (std::is_base_of<TetMesh, AppMesh>::value) {
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
                {"edge_swap_56",
                 [](AppMesh& m, const Tuple& t) -> std::optional<std::vector<Tuple>> {
                     std::vector<Tuple> ret;
                     if (m.swap_edge_56(t, ret))
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
                 }},
                {"face_split",
                 [](AppMesh& m, const Tuple& t) -> std::optional<std::vector<Tuple>> {
                     std::vector<Tuple> ret;
                     if (m.split_face(t, ret))
                         return ret;
                     else
                         return {};
                 }},
                {"tet_split", [](AppMesh& m, const Tuple& t) -> std::optional<std::vector<Tuple>> {
                     std::vector<Tuple> ret;
                     if (m.split_tet(t, ret))
                         return ret;
                     else
                         return {};
                 }}};
        }
        if constexpr (std::is_base_of<TriMesh, AppMesh>::value) {
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
                 }},
                {"face_split", [](AppMesh& m, const Tuple& t) -> std::optional<std::vector<Tuple>> {
                     std::vector<Tuple> ret;
                     if (m.split_face(t, ret))
                         return ret;
                     else
                         return {};
                 }}};
        }
    };

    ExecutePass(ExecutePass&) = delete;

private:
    void operation_cleanup(AppMesh& m)
    { //
        // class ResourceManger
        // what about RAII mesh edit locking?
        // release mutex, but this should be implemented in TetMesh class.
        if (policy == ExecutionPolicy::kSeq)
            return;
        else {
            m.release_vertex_mutex_in_stack();
        }
    }

    size_t get_partition_id(const AppMesh& m, const Tuple& e)
    {
        if (policy == ExecutionPolicy::kSeq) {
            return 0;
        }
        return m.get_partition_id(e);
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
        // The queue holds an operation's INDEX rather than its name. edit_operation_maps is a
        // std::map, so iterating it yields the names in lexicographic order and an index is
        // exactly a name's rank among them -- comparing indices is therefore identical to
        // comparing the strings, which is what keeps the queue order, and so the output,
        // bit-for-bit unchanged. What it buys is that a queue element is now trivially
        // copyable: a heap sift moves 8 bytes instead of a std::string, and a tie on the
        // priority is an integer compare instead of a string compare. It also turns the
        // per-operation dispatch from a string-keyed std::map lookup into an index.
        using OpId = uint32_t;
        using Elem = std::tuple<double, OpId, Tuple, size_t>; // priority, op index, tuple, #retries
        // Each task owns its queue outright -- it is seeded before any thread starts, and the
        // task both pops from it and pushes its renewed operations back into it -- so those need
        // no lock. `final_queue` is the one that genuinely crosses threads: tasks push retry
        // overflow into it while running, and it is drained after the barrier. Both are
        // std::priority_queue with the same comparator underneath, so pop order is unchanged.
        using LocalQueue = wmtk::threading::serial_priority_queue<Elem>;
        using SharedQueue = wmtk::threading::concurrent_priority_queue<Elem>;

        std::vector<const Op*> op_name;
        std::vector<std::function<std::optional<std::vector<Tuple>>(AppMesh&, const Tuple&)>*>
            op_fn;
        std::map<Op, OpId> op_id;
        op_name.reserve(edit_operation_maps.size());
        op_fn.reserve(edit_operation_maps.size());
        for (auto& kv : edit_operation_maps) {
            op_id.emplace(kv.first, OpId(op_name.size()));
            op_name.push_back(&kv.first);
            op_fn.push_back(&kv.second);
        }
        // An operation with no entry here was previously default-constructed into the map by
        // operator[] and then called, which throws bad_function_call -- so it cannot occur in
        // any working configuration. Say so plainly rather than ordering it arbitrarily.
        const auto id_of = [&op_id](const Op& name) {
            const auto it = op_id.find(name);
            if (it == op_id.end()) {
                log_and_throw_error("No operation registered under the name '{}'.", name);
            }
            return it->second;
        };

        std::atomic<bool> stop(false);
        cnt_success = 0;
        cnt_fail = 0;

        // Whether anything actually watches the success count *while the pass runs*. When no
        // stopping criterion is configured -- the case for every tetwild/triwild/simwild pass --
        // nobody does, and the counters can be accumulated per task and folded in at the end
        // instead of hammering one shared cache line from every thread on every operation.
        const bool track_live_success =
            stopping_criterion_checking_frequency != std::numeric_limits<size_t>::max();
        std::atomic<size_t> live_success(0);

        std::vector<LocalQueue> queues(num_threads);
        SharedQueue final_queue;

        auto run_single_queue = [&](auto& Q, int task_id) {
            // Per-task tallies folded into the shared counters once, on the way out. The guard
            // is RAII rather than a line at the bottom because the loop below has early
            // returns.
            struct CountFlusher
            {
                std::atomic_int& success_total;
                std::atomic_int& fail_total;
                int success = 0;
                int fail = 0;
                ~CountFlusher()
                {
                    success_total.fetch_add(success, std::memory_order_relaxed);
                    fail_total.fetch_add(fail, std::memory_order_relaxed);
                }
            } counts{cnt_success, cnt_fail};

            Elem ele_in_queue;
            while ([&]() { return Q.try_pop(ele_in_queue); }()) {
                auto& [weight, op, tup, retry] = ele_in_queue;
                if (!tup.is_valid(m)) {
                    continue;
                }

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
                        const Op& op_str = *op_name[op];
                        if (!is_weight_up_to_date(
                                m,
                                std::tuple<double, Op, Tuple>(weight, op_str, tup))) {
                            operation_cleanup(m);
                            continue;
                        } // this can encode, in qslim, recompute(energy) == weight.
                        auto newtup = (*op_fn[op])(m, tup);
                        std::vector<std::pair<Op, Tuple>> renewed_tuples;
                        if (newtup) {
                            renewed_tuples = renew_neighbor_tuples(m, op_str, newtup.value());
                            counts.success++;
                            if (track_live_success) {
                                live_success.fetch_add(1, std::memory_order_relaxed);
                            }
                        } else {
                            on_fail(m, op_str, tup);
                            counts.fail++;
                        }
                        for (const auto& [o, e] : renewed_tuples) {
                            auto val = priority(m, o, e);
                            if (should_renew(val)) {
                                renewed_elements.emplace_back(val, id_of(o), e, 0);
                            }
                        }
                    }
                    operation_cleanup(m); // Maybe use RAII
                }
                for (auto& e : renewed_elements) {
                    Q.emplace(e);
                }

                if (stop.load(std::memory_order_acquire)) {
                    return;
                }
                if (track_live_success && live_success.load(std::memory_order_relaxed) >
                                              stopping_criterion_checking_frequency) {
                    if (stopping_criterion(m)) {
                        stop.store(true);
                        return;
                    }
                }
            }
        };

        if (policy == ExecutionPolicy::kSeq) {
            for (const auto& [op, e] : operation_tuples) {
                if (!e.is_valid(m)) {
                    continue;
                }
                final_queue.emplace(priority(m, op, e), id_of(op), e, 0);
            }
            run_single_queue(final_queue, 0);
        } else {
            for (const auto& [op, e] : operation_tuples) {
                if (!e.is_valid(m)) {
                    continue;
                }
                queues[get_partition_id(m, e)].emplace(priority(m, op, e), id_of(op), e, 0);
            }
            // Comment out parallel: work on serial first.
            wmtk::threading::task_group tg;
            for (int task_id = 0; task_id < queues.size(); task_id++) {
                tg.run([&run_single_queue, &queues, task_id] {
                    run_single_queue(queues[task_id], task_id);
                });
            }
            tg.wait();
            logger().debug("Parallel Complete, remains element {}", final_queue.size());
            run_single_queue(final_queue, 0);
        }

        logger().info(
            "executed: {} | success / fail: {} / {}",
            (int)cnt_success + (int)cnt_fail,
            (int)cnt_success,
            (int)cnt_fail);
        return true;
    }

    int get_cnt_success() const { return cnt_success; }
    int get_cnt_fail() const { return cnt_fail; }

private:
    // Totals for the whole pass. Written once per task, at the end -- see CountFlusher.
    std::atomic_int cnt_success = 0;
    std::atomic_int cnt_fail = 0;
};
} // namespace wmtk
