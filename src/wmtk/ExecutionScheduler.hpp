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

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <queue>
#include <stdexcept>
#include <thread>
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
     * @brief Attempts an operation gets at claiming its ring before it is handed to the serial
     * queue drained after the barrier.
     *
     * 10 was swept and left alone. Measured on 128k-tet Thingi10K 103197 at 16 threads, three
     * reps each, in µs per attempted operation:
     *
     *   immediate requeue (before the second-chance deferral below existed):
     *       1: +17.0%   2: +7.2%   3: +6.2%   5: +4.6%   10: best   20: +6.2%
     *   with the deferral:
     *       2: 2.170    3: 1.969    10: 1.974          (baseline at 10 was 2.527)
     *
     * So under the old immediate-requeue behaviour the value mattered a lot -- a low limit sent
     * everything it stopped retrying to the serial queue, and that tail (23% of scheduler time
     * at 10, 39% at 1) cost more than the spinning it avoided. With the deferral the retries are
     * nearly free, the overflow pressure disappears, and 3 and 10 become indistinguishable.
     * Left at 10 because nothing argues for moving it.
     *
     * Below about 3 it still hurts: at 2 the overflow rate climbs enough to be visible again.
     */
    size_t max_retry_limit = 10;

    /**
     * @brief Operations to run before handing deferred operations back to the queue, or 0 to
     * wait until the queue empties.
     *
     * Bounding this is what makes the second-chance list pay. A per-thread queue holds thousands
     * of operations, so draining only at queue-empty means a deferred operation waits that long,
     * by which time the mesh around it has moved, `is_weight_up_to_date` rejects it, and the work
     * has to be rediscovered. That inflates the attempt count by roughly a fifth and eats most of
     * the per-operation saving.
     *
     * Measured at 16 threads, three reps, optimization wall time against the pre-deferral
     * baseline, with attempts in brackets:
     *
     *                    103197 (base 23.0M)      101881 (base 48.8M)
     *      window 0       -2.2%  [28.5M]          -20.4%  [45.7M]
     *      window 32     -16.3%  [23.7M]          -21.6%  [43.7M]
     *      window 128    -17.9%  [23.1M]          -26.4%  [41.9M]
     *      window 512    -17.0%  [23.7M]          -25.5%  [42.7M]
     *
     * 128 is at or near best on both and keeps the attempt count closest to the baseline's. The
     * result is not sharp between 32 and 512; it is sharp against 0.
     */
    size_t deferral_window = 128;
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

        // Contention accounting. Everything here is either a per-task local folded in once or a
        // write to the task's own slot, so it adds nothing to the inner loop. It answers the
        // two questions the pass could not previously be asked: how often ring acquisition
        // loses a race, and how much of a "parallel" pass is really the serial drain.
        m_stats = PassStats{};
        std::atomic<size_t> lock_failures(0);
        std::atomic<size_t> overflowed(0);
        std::vector<double> task_seconds(queues.size(), 0.);

        // Per-task tallies folded into the shared counters once, on the way out. The guard is
        // RAII rather than a line at the bottom because run_single_queue has early returns.
        //
        // DECLARED HERE, NOT INSIDE THE LAMBDA, to work around an Apple clang 17 codegen bug.
        // A local class declared inside a GENERIC lambda (one with an `auto` parameter, so its
        // operator() is a template) gets its destructor emitted as an undefined reference and
        // never defined, at every optimization level including -O0. The link then fails with
        //     Undefined symbols: ... ::'lambda'(auto&, int)::operator()<...>
        //                            ::CountFlusher::~CountFlusher()
        // referenced from every translation unit that instantiates a pass -- 26 references
        // across 6 archives, defined nowhere. Homebrew clang 22 compiles the same code
        // correctly, which is why CI does not see this.
        //
        // Hoisting the class one scope out is enough: it is still local to operator(), which is
        // itself a member of a class template, and that instantiates fine. Nothing else changes
        // -- `counts` is still constructed per task inside the lambda, so the RAII flush and its
        // ordering are identical.
        struct CountFlusher
        {
            std::atomic_int& success_total;
            std::atomic_int& fail_total;
            std::atomic<size_t>& lock_failure_total;
            std::atomic<size_t>& overflow_total;
            int success = 0;
            int fail = 0;
            size_t lock_failure = 0;
            size_t overflow = 0;
            ~CountFlusher()
            {
                success_total.fetch_add(success, std::memory_order_relaxed);
                fail_total.fetch_add(fail, std::memory_order_relaxed);
                lock_failure_total.fetch_add(lock_failure, std::memory_order_relaxed);
                overflow_total.fetch_add(overflow, std::memory_order_relaxed);
            }
        };

        auto run_single_queue = [&](auto& Q, int task_id) {
            CountFlusher counts{cnt_success, cnt_fail, lock_failures, overflowed};

            Elem ele_in_queue;
            // Operations that lost a race for their ring wait here rather than going straight
            // back into the live queue. Pushing them back immediately is a spin: the element was
            // just popped as the queue's maximum, `retry` is the last tie-break key in Elem, so
            // the requeued copy compares strictly greater than what was popped and nothing else
            // was added -- it comes right back off the top and is retried against a conflict that
            // has had no time to clear. Deferring lets every other operation this task owns run
            // first, which is both useful work and the delay the conflict needs.
            std::vector<Elem> second_chance;
            // Operations popped since the deferred list was last given back to the queue.
            // Waiting for the queue to drain completely can mean thousands of operations, by
            // which time the mesh around a deferred operation has moved and its work has to be
            // rediscovered. A bounded window gives the conflict time to clear without letting
            // the operation go stale. 0 = only when the queue empties.
            size_t since_refill = 0;
            const auto refill = [&] {
                for (auto& e : second_chance) {
                    Q.emplace(std::move(e));
                }
                second_chance.clear();
                since_refill = 0;
            };
            for (;;) {
                if (!second_chance.empty() && deferral_window > 0 &&
                    since_refill >= deferral_window) {
                    refill();
                }
                if (!Q.try_pop(ele_in_queue)) {
                    if (second_chance.empty()) {
                        break;
                    }
                    // Queue exhausted: the deferred operations have now had everything else
                    // run ahead of them, so give them another go. `retry` still increments on
                    // each attempt and still overflows to final_queue at max_retry_limit, so
                    // this terminates after at most that many rounds.
                    refill();
                    std::this_thread::yield();
                    continue;
                }
                ++since_refill;
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
                        counts.lock_failure++;
                        retry++;
                        if (retry < max_retry_limit) {
                            second_chance.push_back(ele_in_queue);
                        } else {
                            retry = 0;
                            counts.overflow++;
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
            using clock = std::chrono::steady_clock;
            const auto t_parallel = clock::now();
            wmtk::threading::task_group tg;
            for (int task_id = 0; task_id < queues.size(); task_id++) {
                tg.run([&run_single_queue, &queues, &task_seconds, task_id] {
                    const auto t0 = clock::now();
                    run_single_queue(queues[task_id], task_id);
                    // Each task writes only its own slot.
                    task_seconds[task_id] =
                        std::chrono::duration<double>(clock::now() - t0).count();
                });
            }
            tg.wait();
            m_stats.parallel_seconds =
                std::chrono::duration<double>(clock::now() - t_parallel).count();
            m_stats.final_queue_size = final_queue.size();

            logger().debug("Parallel Complete, remains element {}", final_queue.size());

            const auto t_tail = clock::now();
            run_single_queue(final_queue, 0);
            m_stats.serial_tail_seconds =
                std::chrono::duration<double>(clock::now() - t_tail).count();
        }

        m_stats.lock_failures = lock_failures.load(std::memory_order_relaxed);
        m_stats.overflowed = overflowed.load(std::memory_order_relaxed);
        if (!task_seconds.empty()) {
            const auto mm = std::minmax_element(task_seconds.begin(), task_seconds.end());
            m_stats.idlest_task_seconds = *mm.first;
            m_stats.busiest_task_seconds = *mm.second;
        }

        logger().info(
            "executed: {} | success / fail: {} / {}",
            (int)cnt_success + (int)cnt_fail,
            (int)cnt_success,
            (int)cnt_fail);
        log_contention();
        return true;
    }

    int get_cnt_success() const { return cnt_success; }
    int get_cnt_fail() const { return cnt_fail; }

    /**
     * @brief What the last pass cost in contention, as opposed to in work.
     *
     * Populated by every `operator()` call, so under run_localized_to_convergence it describes
     * the most recent round only. Zeroed at the start of each pass.
     */
    struct PassStats
    {
        /// Operations that could not claim their ring and were requeued. Counts *attempts*, so
        /// one stubborn operation can contribute up to max_retry_limit.
        size_t lock_failures = 0;
        /// Operations that exhausted max_retry_limit and were pushed to the post-barrier queue.
        size_t overflowed = 0;
        /// Size of that queue once every task had finished.
        size_t final_queue_size = 0;
        /// Wall time inside the parallel region, and in the serial drain that follows it. The
        /// pass is billed as "parallel" in the driver's log line, but it is the sum of these.
        double parallel_seconds = 0.;
        double serial_tail_seconds = 0.;
        /// Busy time of the longest- and shortest-running task. A wide gap means the partition
        /// split the work unevenly, and since tasks never steal, the tail is one thread.
        double busiest_task_seconds = 0.;
        double idlest_task_seconds = 0.;
    };
    const PassStats& stats() const { return m_stats; }

private:
    /// Debug-level because it is per pass and there are many passes per iteration. Enable with
    /// the logger at debug to see whether contention is worth acting on.
    void log_contention() const
    {
        if (policy == ExecutionPolicy::kSeq || !logger().should_log(spdlog::level::debug)) {
            return;
        }
        const int executed = (int)cnt_success + (int)cnt_fail;
        const double total = m_stats.parallel_seconds + m_stats.serial_tail_seconds;
        logger().debug(
            "  contention: {} ring-acquisition failures over {} executed ops ({:.2f} per op); "
            "{} overflowed to the serial queue ({} queued at the barrier)",
            m_stats.lock_failures,
            executed,
            executed > 0 ? double(m_stats.lock_failures) / executed : 0.,
            m_stats.overflowed,
            m_stats.final_queue_size);
        logger().debug(
            "  time: {:.4}s parallel + {:.4}s serial tail ({:.1f}% of the pass); busiest task "
            "{:.4}s, idlest {:.4}s",
            m_stats.parallel_seconds,
            m_stats.serial_tail_seconds,
            total > 0. ? 100. * m_stats.serial_tail_seconds / total : 0.,
            m_stats.busiest_task_seconds,
            m_stats.idlest_task_seconds);
    }

    // Totals for the whole pass. Written once per task, at the end -- see CountFlusher.
    std::atomic_int cnt_success = 0;
    std::atomic_int cnt_fail = 0;
    PassStats m_stats;
};
} // namespace wmtk
