#pragma once

#include <wmtk/TetOptimizerMesh.h>
#include <wmtk/TriOptimizerMesh.h>
#include <wmtk/ExecutionScheduler.hpp>

#include <functional>
#include <string>

namespace wmtk {

/**
 * @brief Which mutex an operation has to claim before it may run in parallel.
 */
enum class PassLock {
    EdgeTwoRing, ///< topology-changing operations queued on edges
    FaceTwoRing, ///< face swaps (3D only)
    VertexOneRing, ///< smoothing, which moves a single vertex and changes no connectivity
};

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
 */
void run_pass(
    TriOptimizerMesh& m,
    PassLock lock,
    const std::string& label,
    const std::function<void(ExecutePass<TriOptimizerMesh>&, TriOptimizerMesh&)>& body);

void run_pass(
    TetOptimizerMesh& m,
    PassLock lock,
    const std::string& label,
    const std::function<void(ExecutePass<TetOptimizerMesh>&, TetOptimizerMesh&)>& body);

} // namespace wmtk
