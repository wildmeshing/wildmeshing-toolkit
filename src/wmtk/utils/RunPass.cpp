#include <wmtk/utils/RunPass.hpp>

#include <wmtk/utils/Logger.hpp>

#include <igl/Timer.h>

namespace wmtk {
namespace {

template <class Mesh>
std::function<bool(Mesh&, const typename Mesh::Tuple&, int)> make_locker(PassLock lock)
{
    using Tuple = typename Mesh::Tuple;
    switch (lock) {
    case PassLock::VertexOneRing:
        return [](Mesh& m, const Tuple& e, int task_id) {
            return m.try_set_vertex_mutex_one_ring(e, task_id);
        };
    case PassLock::FaceTwoRing:
        if constexpr (std::is_base_of<TetMesh, Mesh>::value) {
            return [](Mesh& m, const Tuple& e, int task_id) {
                return m.try_set_face_mutex_two_ring(e, task_id);
            };
        } else {
            log_and_throw_error("PassLock::FaceTwoRing is only meaningful on a tet mesh");
        }
    case PassLock::EdgeTwoRing: break;
    }
    return [](Mesh& m, const Tuple& e, int task_id) {
        return m.try_set_edge_mutex_two_ring(e, task_id);
    };
}

template <class Mesh>
void run_pass_impl(
    Mesh& m,
    PassLock lock,
    const std::string& label,
    const std::function<void(ExecutePass<Mesh>&, Mesh&)>& body)
{
    igl::Timer timer;
    timer.start();

    const bool parallel = m.NUM_THREADS > 0;

    ExecutePass<Mesh> executor(parallel ? ExecutionPolicy::kPartition : ExecutionPolicy::kSeq);
    executor.num_threads = m.NUM_THREADS;
    if (parallel) {
        // Serial leaves `lock_vertices` at its default (always succeeds): there is nothing to
        // lock against, and claiming the ring would only add work.
        executor.lock_vertices = make_locker<Mesh>(lock);
    }

    body(executor, m);

    if (!label.empty()) {
        logger().info(
            "{} time {}: {:.4}s",
            label,
            parallel ? "parallel" : "serial",
            timer.getElapsedTimeInSec());
    }
}

} // namespace

void run_pass(
    TriOptimizerMesh& m,
    PassLock lock,
    const std::string& label,
    const std::function<void(ExecutePass<TriOptimizerMesh>&, TriOptimizerMesh&)>& body)
{
    run_pass_impl(m, lock, label, body);
}

void run_pass(
    TetOptimizerMesh& m,
    PassLock lock,
    const std::string& label,
    const std::function<void(ExecutePass<TetOptimizerMesh>&, TetOptimizerMesh&)>& body)
{
    run_pass_impl(m, lock, label, body);
}

} // namespace wmtk
