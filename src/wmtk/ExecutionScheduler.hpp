#pragma once

#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/threading/concurrent_priority_queue.hpp>
#include <wmtk/threading/serial_priority_queue.hpp>
#include <wmtk/threading/spin_mutex.hpp>
#include <wmtk/threading/task_group.hpp>
#include <wmtk/utils/Logger.hpp>

// clang-format off
#include <deque>
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
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <type_traits>
#include <unordered_map>

namespace wmtk {
enum class ExecutionPolicy { kSeq, kUnSeq, kPartition, kColor, kMax };

using Op = std::string;

/**
 * @brief A pass's candidate operations, compact and consumed as they are queued.
 *
 * A pass lists every candidate before it runs -- two per edge for a collapse, three for the
 * combined edge swap -- and on the first pass after construction that list is the size of the
 * whole mesh. As `std::vector<std::pair<Op, Tuple>>` each entry carried a std::string (72 bytes
 * an entry on a tet mesh), and the list sat next to the priority queue built from it until the
 * queue was complete.
 *
 * Here an entry is the Tuple and a 32-bit index into the handful of operation names the list
 * has seen, and the entries live in a deque that ExecutePass pops as it seeds its queues, so
 * the list shrinks while the queue grows instead of the two coexisting. The interface the passes
 * use to build it -- `emplace_back(name, tuple)` -- is unchanged.
 */
template <class Tuple>
class OpList
{
public:
    void emplace_back(const Op& op, const Tuple& t) { m_items.emplace_back(intern(op), t); }
    size_t size() const { return m_items.size(); }
    bool empty() const { return m_items.empty(); }
    void clear() { m_items.clear(); }

    /// Hand every entry to `f(name, tuple)`, front to back, releasing them as it goes.
    template <class F>
    void consume(F&& f)
    {
        while (!m_items.empty()) {
            const auto& [id, t] = m_items.front();
            f(m_names[id], t);
            m_items.pop_front();
        }
        m_items.shrink_to_fit();
    }

    /// Move `other`'s entries to the end of this list, releasing them from `other` as it goes.
    void append(OpList&& other)
    {
        if (m_items.empty() && m_names.empty()) {
            *this = std::move(other);
            return;
        }
        std::vector<uint32_t> remap(other.m_names.size());
        for (size_t i = 0; i < remap.size(); ++i) remap[i] = intern(other.m_names[i]);
        while (!other.m_items.empty()) {
            m_items.emplace_back(remap[other.m_items.front().first], other.m_items.front().second);
            other.m_items.pop_front();
        }
    }

private:
    uint32_t intern(const Op& op)
    {
        // A list sees one to three distinct names; a scan is cheaper than any map.
        for (size_t i = 0; i < m_names.size(); ++i) {
            if (m_names[i] == op) return static_cast<uint32_t>(i);
        }
        m_names.push_back(op);
        return static_cast<uint32_t>(m_names.size() - 1);
    }

    std::vector<Op> m_names;
    std::deque<std::pair<uint32_t, Tuple>> m_items;
};

/// An edge's ExecutePass::queue_key: its two vertex ids, smaller first, packed into 64 bits.
/// Never 0, the "not tracked" key: the ids of an edge differ, so the larger one is at least 1.
inline uint64_t edge_queue_key(size_t a, size_t b)
{
    if (a > b) std::swap(a, b);
    if (b >> 32) {
        log_and_throw_error("edge_queue_key: vertex id {} does not fit in 32 bits", b);
    }
    return (uint64_t(a) << 32) | uint64_t(b);
}

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

    /**
     * @brief Optional order between queued operations, used by the split passes.
     *
     * `queue_key` names what an operation is about -- for a split, its edge (edge_queue_key) --
     * or returns 0 for "not tracked". A tracked operation is counted from the moment it is
     * queued until it is tried, or dropped as invalid or out of date. Being set aside after a
     * lost lock race does not end it. is_queued() reads that count, for any thread.
     *
     * `must_wait` is asked after is_weight_up_to_date, with the operation's ring locked. True
     * sets the operation aside exactly like a lost lock race: it is retried later in the pass
     * and, after max_retry_limit attempts, handed to the serial queue drained after the barrier.
     * A serial run -- kSeq, or that serial queue -- pops in priority order, so whatever an
     * operation could wait on has been tried before it. There a `must_wait` that still says true
     * is a defect: the operation fails (on_fail) and is counted in wait_defects() rather than
     * being set aside forever.
     *
     * Both empty, the default: nothing is counted and nothing waits.
     */
    std::function<uint64_t(const AppMesh&, const Op&, const Tuple&)> queue_key;
    std::function<bool(const AppMesh&, const Op&, const Tuple&)> must_wait;
    bool is_queued(uint64_t key) const { return m_queued.contains(key); }
    /// Over every call on this executor: operations set aside by must_wait, and must_wait
    /// refusals in a serial run (defects, see above).
    size_t waits() const { return m_waits.load(); }
    size_t wait_defects() const { return m_wait_defects.load(); }

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
        return execute(m, [&](const Push& push) {
            for (const auto& [op, e] : operation_tuples) push(op, e);
        });
    }

    /**
     * @brief As above, but takes the operation list over and frees it as soon as the queues are
     * seeded.
     *
     * The queues hold their own copy of every operation, so the list is dead weight from then
     * on -- and it is at its longest on the first pass after construction, which is also when
     * the mesh is at its largest. Releasing it there is what keeps a pass from holding every
     * candidate twice at the run's peak.
     */
    bool operator()(AppMesh& m, std::vector<std::pair<Op, Tuple>>&& operation_tuples)
    {
        return execute(m, [&](const Push& push) {
            for (const auto& [op, e] : operation_tuples) push(op, e);
            // Swapping with an empty vector, not clear(), so the storage is returned.
            std::vector<std::pair<Op, Tuple>>().swap(operation_tuples);
        });
    }

    /**
     * @brief As above for an OpList, which is released entry by entry while the queues are
     * seeded, so the list and the queue never both hold every candidate.
     */
    bool operator()(AppMesh& m, OpList<Tuple>&& operation_tuples)
    {
        return execute(m, [&](const Push& push) { operation_tuples.consume(push); });
    }

private:
    /// Queues one candidate; what a seeder calls for each entry of its list.
    using Push = std::function<void(const Op&, const Tuple&)>;

    bool execute(AppMesh& m, const std::function<void(const Push&)>& seed)
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
        // priority, op index, #retries, tuple, queue_key. A struct rather than a std::tuple so the
        // two 32-bit fields share a word: std::tuple lays its members out in reverse and pads each
        // one to the Tuple's alignment, which costs 8 bytes per queued operation. The ordering
        // is the one the std::tuple had -- (priority, op, tuple, retry, key), lexicographic -- so
        // the pop order is unchanged. The key compares last, after the tuple, and two elements
        // with the same tuple have the same key.
        // What a parallel task may still take from the storage this round.
        struct SlotBudget
        {
            size_t cells = 0;
            size_t verts = 0;
            bool stopped = false; // ended its round for want of slots, queue not drained
            bool progressed = false; // ran at least one operation this round
        };
        struct Elem
        {
            double weight = 0;
            OpId op = 0;
            uint32_t retry = 0;
            Tuple tup;
            uint64_t key = 0; // queue_key, 0 when untracked

            Elem() = default;
            Elem(double w, OpId o, const Tuple& t, uint32_t r, uint64_t k)
                : weight(w)
                , op(o)
                , retry(r)
                , tup(t)
                , key(k)
            {}

            // A member, not a hidden friend: a local class cannot define friend functions.
            bool operator<(const Elem& b) const
            {
                return std::tie(weight, op, tup, retry, key) <
                       std::tie(b.weight, b.op, b.tup, b.retry, b.key);
            }
        };
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

        // queue_key bookkeeping (see must_wait). `track` when an element is queued, `done` when
        // it is tried or dropped; both no-ops for the untracked key 0.
        m_queued.clear();
        std::atomic<size_t> untracked_done(0);
        const auto key_of = [&](const Op& op, const Tuple& e) -> uint64_t {
            return queue_key ? queue_key(m, op, e) : 0;
        };
        const auto track = [&](const uint64_t key) {
            if (key != 0) m_queued.add(key);
        };
        const auto done = [&](const uint64_t key) {
            if (key != 0 && !m_queued.remove(key)) {
                untracked_done.fetch_add(1, std::memory_order_relaxed);
            }
        };

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

        // `serial`: no other thread is running operations on the mesh, and Q is popped by this
        // task alone and in priority order -- the serial policy, and the serial drain after a
        // parallel pass. The storage may then grow between operations (see
        // TetMesh::reserve_free_slots), never inside a parallel task; and a must_wait that says
        // true is a defect there.
        //
        // `budget`: the slots a parallel task may still take this round (see the round loop
        // below); null when the task may grow the storage instead.
        auto run_single_queue = [&](auto& Q, int task_id, const bool serial, SlotBudget* budget) {
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
                auto& [weight, op, retry, tup, key] = ele_in_queue;
                if (!tup.is_valid(m)) {
                    done(key);
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
                            done(key);
                            operation_cleanup(m);
                            continue;
                        } // this can encode, in qslim, recompute(energy) == weight.
                        if (must_wait && must_wait(m, op_str, tup)) {
                            if (serial) {
                                m_wait_defects.fetch_add(1, std::memory_order_relaxed);
                                done(key);
                                on_fail(m, op_str, tup);
                                counts.fail++;
                                operation_cleanup(m);
                                continue;
                            }
                            // Set aside exactly like a lost lock race, above.
                            operation_cleanup(m);
                            m_waits.fetch_add(1, std::memory_order_relaxed);
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
                        if (serial) {
                            m.reserve_free_slots(m.cell_slot_bound(tup), 1);
                        } else if (budget != nullptr) {
                            // The ring is locked, so the stars the bound reads are stable.
                            if (m.cell_slot_bound(tup) > budget->cells || budget->verts < 1) {
                                // Out of this round's slots: hand the operation back untouched,
                                // with everything deferred, and end the task's round.
                                operation_cleanup(m);
                                Q.emplace(ele_in_queue);
                                refill();
                                budget->stopped = true;
                                return;
                            }
                        }
                        const auto usage_before = AppMesh::slot_usage_of_this_thread();
                        auto newtup = (*op_fn[op])(m, tup);
                        done(key);
                        if (budget != nullptr) {
                            const auto usage = AppMesh::slot_usage_of_this_thread();
                            budget->cells -= usage.cells - usage_before.cells;
                            budget->verts -= usage.verts - usage_before.verts;
                            budget->progressed = true;
                        }
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
                        // Tracked here, with the ring still locked, although the elements are
                        // queued only after it is released: in between they count as queued.
                        for (const auto& [o, e] : renewed_tuples) {
                            auto val = priority(m, o, e);
                            if (should_renew(val)) {
                                const uint64_t k = key_of(o, e);
                                track(k);
                                renewed_elements.emplace_back(val, id_of(o), e, 0, k);
                            }
                        }
                    } else {
                        done(key);
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
            seed([&](const Op& op, const Tuple& e) {
                if (!e.is_valid(m)) {
                    return;
                }
                const uint64_t k = key_of(op, e);
                track(k);
                final_queue.emplace(priority(m, op, e), id_of(op), e, 0, k);
            });
            run_single_queue(final_queue, 0, /*serial=*/true, nullptr);
        } else {
            seed([&](const Op& op, const Tuple& e) {
                if (!e.is_valid(m)) {
                    return;
                }
                const uint64_t k = key_of(op, e);
                track(k);
                queues[get_partition_id(m, e)].emplace(priority(m, op, e), id_of(op), e, 0, k);
            });
            // Comment out parallel: work on serial first.
            using clock = std::chrono::steady_clock;
            const auto t_parallel = clock::now();
            // Rounds. The storage cannot grow while the tasks run, so each round starts at a
            // serial point that grows it by a headroom split into per-task budgets; a task runs
            // an operation only if its budget covers the operation's slot bound, and is charged
            // what the operation actually took. A task that cannot afford its next operation
            // hands it back and ends its round. No operation is ever refused for want of slots,
            // and the storage only ever needs to be a quarter larger than the mesh.
            //
            // A task that ends a round without having run anything had a single operation
            // larger than its share; its share doubles until it fits, so every round but the
            // last makes progress and the loop terminates.
            const size_t n_tasks = queues.size();
            std::vector<SlotBudget> budgets(n_tasks);
            std::vector<unsigned> boost(n_tasks, 0);
            wmtk::threading::task_group tg;
            for (size_t round = 0;; ++round) {
                // A quarter of the mesh per round, or whatever the storage already has free if
                // that is more -- a mesh that still preallocates never runs more rounds than
                // its headroom forces.
                const size_t cell_room =
                    std::max(m.cell_capacity() / 4, m.cell_storage_capacity() - m.cell_capacity());
                const size_t vert_room =
                    std::max(m.vert_capacity() / 4, m.vert_storage_capacity() - m.vert_capacity());
                const size_t cell_share = std::max<size_t>(4096, cell_room / n_tasks);
                const size_t vert_share = std::max<size_t>(1024, vert_room / n_tasks);
                size_t cells = 0, verts = 0;
                for (size_t t = 0; t < n_tasks; ++t) {
                    budgets[t] = SlotBudget{cell_share << boost[t], vert_share << boost[t]};
                    cells += budgets[t].cells;
                    verts += budgets[t].verts;
                }
                m.reserve_free_slots(cells, verts);
                for (int task_id = 0; task_id < n_tasks; task_id++) {
                    tg.run([&run_single_queue, &queues, &task_seconds, &budgets, task_id] {
                        const auto t0 = clock::now();
                        run_single_queue(
                            queues[task_id],
                            task_id,
                            /*serial=*/false,
                            &budgets[task_id]);
                        // Each task writes only its own slot.
                        task_seconds[task_id] +=
                            std::chrono::duration<double>(clock::now() - t0).count();
                    });
                }
                tg.wait();
                bool again = false;
                for (size_t t = 0; t < n_tasks; ++t) {
                    if (!budgets[t].stopped) continue;
                    again = true;
                    if (!budgets[t].progressed) boost[t] = std::min(boost[t] + 1, 24u);
                }
                if (!again) break;
                m_stats.rounds = round + 2;
            }
            m_stats.parallel_seconds =
                std::chrono::duration<double>(clock::now() - t_parallel).count();
            m_stats.final_queue_size = final_queue.size();

            logger().debug("Parallel Complete, remains element {}", final_queue.size());

            const auto t_tail = clock::now();
            run_single_queue(final_queue, 0, /*serial=*/true, nullptr);
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
        if (m_stats.rounds > 1) {
            logger().info(
                "  parallel region ran in {} rounds (storage grown between them)",
                m_stats.rounds);
        }
        log_contention();
        // Every tracked element queued in this call was tried or dropped, unless the stopping
        // criterion ended the call early. Anything else is a bookkeeping defect, and is_queued()
        // may have answered wrongly during the call.
        if (queue_key && !stop.load()) {
            const size_t left = m_queued.size();
            const size_t untracked = untracked_done.load();
            if (left > 0 || untracked > 0) {
                logger().warn(
                    "queue_key bookkeeping: {} keys still counted as queued after the pass, {} "
                    "finished elements were never counted",
                    left,
                    untracked);
            }
        }
        m_queued.clear();
        return true;
    }

public:
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
        /// Rounds the parallel region took: more than one when the tasks ran through their
        /// slot budgets and the storage had to grow between rounds.
        size_t rounds = 1;
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

    /// The count behind is_queued(): per key, the queued elements not yet tried or dropped.
    /// Striped, so threads queueing and finishing elements of different keys rarely meet on a
    /// lock; a single mutex (threading::concurrent_map) would be taken by every thread for
    /// every queued split.
    class QueuedCount
    {
        static constexpr size_t n_stripes = 1024;
        struct alignas(64) Stripe
        {
            threading::spin_mutex mutex;
            std::unordered_map<uint64_t, int> count;
        };
        std::unique_ptr<Stripe[]> m_stripes = std::make_unique<Stripe[]>(n_stripes);
        Stripe& stripe(const uint64_t key) const
        {
            return m_stripes[std::hash<uint64_t>{}(key) % n_stripes];
        }

    public:
        void add(const uint64_t key)
        {
            Stripe& s = stripe(key);
            std::lock_guard<threading::spin_mutex> lock(s.mutex);
            ++s.count[key];
        }
        /// False if `key` was not counted.
        bool remove(const uint64_t key)
        {
            Stripe& s = stripe(key);
            std::lock_guard<threading::spin_mutex> lock(s.mutex);
            const auto it = s.count.find(key);
            if (it == s.count.end()) return false;
            if (--it->second == 0) s.count.erase(it);
            return true;
        }
        bool contains(const uint64_t key) const
        {
            Stripe& s = stripe(key);
            std::lock_guard<threading::spin_mutex> lock(s.mutex);
            return s.count.find(key) != s.count.end();
        }
        /// Keys still counted. Only while no thread is queueing or finishing elements.
        size_t size() const
        {
            size_t n = 0;
            for (size_t i = 0; i < n_stripes; ++i) n += m_stripes[i].count.size();
            return n;
        }
        /// Only while no thread is queueing or finishing elements.
        void clear()
        {
            for (size_t i = 0; i < n_stripes; ++i) m_stripes[i].count.clear();
        }
    };
    QueuedCount m_queued;
    std::atomic<size_t> m_waits = 0;
    std::atomic<size_t> m_wait_defects = 0;
};
} // namespace wmtk
