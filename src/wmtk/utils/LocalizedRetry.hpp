#pragma once

#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/threading/collector.hpp>

#include <cstdint>
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
 */
template <class Mesh>
size_t run_localized_to_convergence(
    Mesh& m,
    ExecutePass<Mesh>& executor,
    std::vector<std::pair<Op, typename Mesh::Tuple>> ops)
{
    using Tuple = typename Mesh::Tuple;

    // vertex_epoch[v] = the last round in which v was in some successful operation's
    // modified region. Capacity is fixed within a phase (storage is preallocated), and
    // any vertex created during the phase (splits) has an id below this capacity.
    std::vector<uint64_t> vertex_epoch(m.vert_capacity(), 0);
    uint64_t round = 0;
    threading::collector<std::pair<Op, Tuple>> failures;

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
    executor.on_fail = [&failures](const Mesh&, Op op, const Tuple& t) {
        failures.emplace_back(op, t);
    };

    size_t total_success = 0;
    do {
        ++round;
        failures.clear();
        executor(m, ops);
        total_success += static_cast<size_t>(executor.get_cnt_success());
        ops.clear();
        for (const auto& pr : failures) {
            const Tuple& t = pr.second;
            if (!t.is_valid(m)) {
                continue;
            }
            // retry only if this failure's neighborhood was modified during this round
            if (edge_epoch(m, t) == round) {
                ops.emplace_back(pr);
            }
        }
    } while (executor.get_cnt_success() > 0 && !ops.empty());
    return total_success;
}

} // namespace wmtk
