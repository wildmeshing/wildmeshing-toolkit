#include <wmtk/utils/RunPass.hpp>

#include <wmtk/utils/Logger.hpp>

#include <igl/Timer.h>

namespace wmtk {
namespace {

template <class Mesh>
std::function<bool(Mesh&, const typename Mesh::Tuple&, int)> make_locker(PassLock lock, int ring)
{
    using Tuple = typename Mesh::Tuple;

    // At the default radius, go through the named helper rather than the generic ball. The
    // two are the same set in 2D -- TriMesh's helpers delegate -- but deliberately are NOT in
    // 3D, where the hand-written form under-claims and the honest ball costs +80% wall clock
    // on the challenging tetwild set. See the note above TetMesh::try_set_vertex_mutex_two_ring.
    const bool at_default = ring == default_ring(lock);

    switch (lock) {
    case PassLock::VertexRing:
        if (at_default) {
            return [](Mesh& m, const Tuple& e, int task_id) {
                return m.try_set_vertex_mutex_one_ring(e, task_id);
            };
        }
        return [ring](Mesh& m, const Tuple& e, int task_id) {
            return m.try_set_vertex_mutex_n_ring(e, task_id, ring);
        };
    case PassLock::FaceRing:
        if constexpr (std::is_base_of<TetMesh, Mesh>::value) {
            // The face variant seeds from three vertices and only exists in 3D.
            if (!at_default) {
                log_and_throw_error("PassLock::FaceRing supports only its default radius");
            }
            return [](Mesh& m, const Tuple& e, int task_id) {
                return m.try_set_face_mutex_two_ring(e, task_id);
            };
        } else {
            log_and_throw_error("PassLock::FaceRing is only meaningful on a tet mesh");
        }
    case PassLock::EdgeRing: break;
    }
    if (at_default) {
        return [](Mesh& m, const Tuple& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
    }
    return [ring](Mesh& m, const Tuple& e, int task_id) {
        return m.try_set_edge_mutex_n_ring(e, task_id, ring);
    };
}

template <class Mesh>
void run_pass_impl(
    Mesh& m,
    PassLock lock,
    int ring,
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
        executor.lock_vertices = make_locker<Mesh>(lock, ring);
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
    int ring,
    const std::string& label,
    const std::function<void(ExecutePass<TriOptimizerMesh>&, TriOptimizerMesh&)>& body)
{
    run_pass_impl(m, lock, ring, label, body);
}

void run_pass(
    TetOptimizerMesh& m,
    PassLock lock,
    int ring,
    const std::string& label,
    const std::function<void(ExecutePass<TetOptimizerMesh>&, TetOptimizerMesh&)>& body)
{
    run_pass_impl(m, lock, ring, label, body);
}

} // namespace wmtk
