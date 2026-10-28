#pragma once

namespace wmtk {

/**
 * @brief Whether mesh operations on this thread are dry runs.
 *
 * A dry run executes everything an operation does before its first change to the mesh -- the
 * application's *_before hook, the valence and boundary tests, choosing the best swap
 * configuration -- and then returns true instead of making the change. It never writes the mesh,
 * so any number of threads may dry-run operations against the same unchanging mesh without a
 * lock. This is what ExecutePass's screening phase is made of (see
 * ExecutePass::screen_before_commit): on the challenging tetwild models ~98% of attempts are
 * refused before their first change.
 *
 * A dry run that passes means "worth attempting for real", nothing more: the real attempt repeats
 * every check (the mesh may have changed in between) and may still be refused by the *_after
 * hook.
 *
 * Honoured by every operation ExecutePass registers, in TetMesh and TriMesh alike. What that asks
 * of an application: its *_before hooks run concurrently with each other during a dry run, so
 * they may write thread-local caches and atomic counters but nothing else of the mesh's. A hook
 * that does write (TetOptimizerMesh rounds the vertex in smooth_before, and claims high-valence
 * vertices in split_edge_before) skips that write when operation_dry_run() is set, answering as
 * if it had succeeded where it cannot tell without writing.
 *
 * A counter in a hook sees the dry run and then, for a candidate that passes it, the real attempt
 * too. A refusal is counted once, since a refused dry run ends the candidate; a counter of how far
 * candidates got either skips the dry runs (as topological_offset's flip funnel does) or is
 * corrected for them (as TetOptimizerMesh's swap stages are, see SwapStage::screened).
 */
inline bool& operation_dry_run()
{
    static thread_local bool flag = false;
    return flag;
}

/// Turns operation_dry_run() on for the calling thread for the lifetime of the object, and then
/// back to what it was -- so scopes nest.
struct OperationDryRunScope
{
    OperationDryRunScope()
        : m_previous(operation_dry_run())
    {
        operation_dry_run() = true;
    }
    ~OperationDryRunScope() { operation_dry_run() = m_previous; }
    OperationDryRunScope(const OperationDryRunScope&) = delete;
    OperationDryRunScope& operator=(const OperationDryRunScope&) = delete;

private:
    bool m_previous;
};

} // namespace wmtk
