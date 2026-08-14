#pragma once

#include <wmtk/TetOptimizerMesh.h>
#include <wmtk/TriOptimizerMesh.h>
#include <wmtk/ExecutionScheduler.hpp>

#include <functional>
#include <string>

namespace wmtk {

/**
 * @brief Which simplex an operation seeds its lock from, before it may run in parallel.
 *
 * The RADIUS is a separate argument: see `default_ring` for what each kind claims when the
 * caller does not say, and the five-argument `run_pass` below for when it should.
 */
enum class PassLock {
    EdgeRing, ///< topology-changing operations queued on edges
    FaceRing, ///< face swaps (3D only)
    VertexRing, ///< smoothing, which moves a single vertex and changes no connectivity
};

/**
 * @brief The radius each lock kind claims when the caller does not specify one.
 *
 * Two rings for a topology change (it rewrites the one-ring and reads the ring beyond), one
 * for smoothing (it moves one vertex and reads its one-ring). An operation that does more
 * than that -- one that also re-smooths a k-ring, say -- has to ask for more.
 */
constexpr int default_ring(PassLock lock)
{
    return lock == PassLock::VertexRing ? 1 : 2;
}

/**
 * @brief Run one optimization pass, serial or parallel according to the mesh's NUM_THREADS.
 *
 * Every driver in the three applications ended in the same shell: branch on `NUM_THREADS > 0`,
 * build an `ExecutePass` with `kPartition` or `kSeq`, install the same lock function, run the
 * driver-specific setup, and log how long it took with "parallel" or "serial" in the message.
 * The only things that actually varied were the lock (edge two-ring for everything except
 * smoothing), the log label, and the setup itself -- which is what @p body is.
 *
 * `ExecutePass` takes its policy as a *constructor argument*, so both branches build the same
 * type and @p body can be an ordinary `std::function` rather than a template.
 *
 * @p body receives the executor to configure and the mesh to run it on, and is responsible for
 * actually executing (`executor(m, ops)` or `run_localized_to_convergence(m, executor, ops)`) --
 * drivers differ in which, and in what they do with the result.
 *
 * @p label is used as "<label> time parallel: ...". Pass an empty label to log nothing.
 *
 * @note The mesh is taken as the optimizer base, so the callbacks installed on the executor see
 * the base type too. A lambda that needs an application-specific member must capture `this`
 * rather than reach through the executor's mesh argument.
 *
 * @p ring is the radius of the vertex ball to claim, in edges; the overload without it uses
 * `default_ring(lock)`.
 */
void run_pass(
    TriOptimizerMesh& m,
    PassLock lock,
    int ring,
    const std::string& label,
    const std::function<void(ExecutePass<TriOptimizerMesh>&, TriOptimizerMesh&)>& body);

void run_pass(
    TetOptimizerMesh& m,
    PassLock lock,
    int ring,
    const std::string& label,
    const std::function<void(ExecutePass<TetOptimizerMesh>&, TetOptimizerMesh&)>& body);

inline void run_pass(
    TriOptimizerMesh& m,
    PassLock lock,
    const std::string& label,
    const std::function<void(ExecutePass<TriOptimizerMesh>&, TriOptimizerMesh&)>& body)
{
    run_pass(m, lock, default_ring(lock), label, body);
}

inline void run_pass(
    TetOptimizerMesh& m,
    PassLock lock,
    const std::string& label,
    const std::function<void(ExecutePass<TetOptimizerMesh>&, TetOptimizerMesh&)>& body)
{
    run_pass(m, lock, default_ring(lock), label, body);
}

} // namespace wmtk
