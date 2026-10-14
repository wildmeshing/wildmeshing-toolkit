#pragma once

#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/threading/collector.hpp>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <deque>
#include <mutex>
#include <utility>
#include <vector>

namespace wmtk {

/**
 * Run `executor` on `ops` until a pass produces no successful operation, but between
 * passes only re-attempt a failed operation if one of its incident vertices was
 * modified during the pass in which it failed (a per-vertex "dirty epoch").
 *
 * This replaces the brute-force "re-try every failed operation every pass" loop. That
 * loop re-runs the expensive geometric pre-checks (envelope BVH queries, inversion /
 * quality tests over the one-ring) on every failure each round, even for operations
 * whose neighborhood cannot have changed. Because those pre-checks are deterministic,
 * a failure whose region did not change re-fails identically -- so skipping it loses no
 * operation, it only removes wasted work. On pathological meshes (e.g. ~10^8 collapse
 * candidates) that turns each retry round from O(#failures * pre-check) into
 * O(#failures) hash-free integer comparisons.
 *
 * Thread-safety: `renew_neighbor_tuples` (stamps epochs) and `on_fail` (records the
 * failure) are invoked by the scheduler while the operation still holds its two-ring /
 * one-ring lock, so concurrent operations write disjoint vertices; every stamp in a
 * given round writes the same value (`round`), so overlapping-value races are impossible.
 * The between-pass filter reads the epochs single-threaded, after the parallel barrier.
 *
 * The set of modified vertices is taken from the tuples returned by the driver's own
 * `renew_neighbor_tuples` -- i.e. exactly the region the driver already considers
 * "affected" and re-enqueues within a pass -- so this stays consistent with the existing
 * intra-pass renewal logic.
 *
 * `max_passes` caps the loop; 0 means "until convergence". A cap is worth setting when a
 * retry is expensive relative to what it finds. The dirty-epoch filter only asks whether a
 * failure's neighbourhood MOVED, not whether it moved in a direction that helps, so after a
 * productive first pass it re-offers most of the mesh. For a pass whose failures are cheap
 * (pre-checks only) that is a good trade. For one whose every failure runs a full smoothing
 * composite and rolls it back, it is not: measured on tetwild's octocat, passes 2+ of the
 * coarsening loop cost 38.4s to find 27 collapses after pass 1 found 5110 in 135.8s.
 */
template <class Mesh>
size_t run_localized_to_convergence(
    Mesh& m,
    ExecutePass<Mesh>& executor,
    OpList<typename Mesh::Tuple> ops,
    size_t max_passes = 0)
{
    using Tuple = typename Mesh::Tuple;

    // vertex_epoch[v] = the last round in which v was in some successful operation's
    // modified region. Capacity is fixed within a phase (storage is preallocated), and
    // any vertex created during the phase (splits) has an id below this capacity.
    std::vector<uint64_t> vertex_epoch(m.vert_capacity(), 0);
    uint64_t round = 0;
    // A failure is recorded by its operation's rank among the registered names rather than by
    // the name itself: a pass can fail on most of its candidates, and a std::string per entry
    // is 32 of the 72 bytes. edit_operation_maps is a std::map, so its keys are already sorted
    // and the rank is a binary search; it is read-only while the pass runs.
    std::vector<Op> op_names;
    op_names.reserve(executor.edit_operation_maps.size());
    for (const auto& kv : executor.edit_operation_maps) {
        op_names.push_back(kv.first);
    }
    // A deque rather than a vector-backed collector: most candidates of a pass can fail, and a
    // vector growing by doubling to that size holds its old and new buffer at once.
    std::mutex failures_mutex;
    std::deque<std::pair<uint32_t, Tuple>> failures;

    auto edge_epoch = [&vertex_epoch](const Mesh& m_, const Tuple& t) -> uint64_t {
        const size_t a = t.vid(m_);
        const size_t b = t.switch_vertex(m_).vid(m_);
        const uint64_t ea = a < vertex_epoch.size() ? vertex_epoch[a] : 0;
        const uint64_t eb = b < vertex_epoch.size() ? vertex_epoch[b] : 0;
        return std::max(ea, eb);
    };

    // Wrap the driver-provided renewal: keep its behavior (re-enqueue affected tuples
    // within the pass) and additionally stamp those tuples' vertices with the current
    // round so the between-pass filter can find the failures adjacent to them.
    auto driver_renew = executor.renew_neighbor_tuples;
    executor.renew_neighbor_tuples =
        [&, driver_renew](const Mesh& m_, Op op, const std::vector<Tuple>& newts) {
            auto tups = driver_renew(m_, op, newts);
            for (const auto& [_, t] : tups) {
                const size_t a = t.vid(m_);
                const size_t b = t.switch_vertex(m_).vid(m_);
                if (a < vertex_epoch.size()) {
                    // this is thread-safe because each vertex is only ever modified by one
                    // operation at a time (the two-ring lock)
                    vertex_epoch[a] = round;
                }
                if (b < vertex_epoch.size()) {
                    vertex_epoch[b] = round;
                }
            }
            return tups;
        };
    executor.on_fail = [&failures, &failures_mutex, &op_names](const Mesh&, Op op, const Tuple& t) {
        const auto it = std::lower_bound(op_names.begin(), op_names.end(), op);
        assert(it != op_names.end() && *it == op);
        std::lock_guard<std::mutex> lock(failures_mutex);
        failures.emplace_back(static_cast<uint32_t>(it - op_names.begin()), t);
    };

    size_t total_success = 0;
    do {
        ++round;
        failures.clear();
        // Handed over, so the executor frees the list once it has queued it.
        executor(m, std::move(ops));
        total_success += static_cast<size_t>(executor.get_cnt_success());
        ops.clear();
        for (const auto& pr : failures) {
            const Tuple& t = pr.second;
            if (!t.is_valid(m)) {
                continue;
            }
            // retry only if this failure's neighborhood was modified during this round
            if (edge_epoch(m, t) == round) {
                ops.emplace_back(op_names[pr.first], t);
            }
        }
    } while (executor.get_cnt_success() > 0 && !ops.empty() &&
             (max_passes == 0 || round < max_passes));
    return total_success;
}

} // namespace wmtk
