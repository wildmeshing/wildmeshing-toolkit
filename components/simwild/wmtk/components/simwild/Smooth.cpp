
#include "SimWildMesh.h"

#include <wmtk/optimization/SmoothVertex.hpp>
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <igl/edges.h>
#include <wmtk/utils/AMIPS.h>
#include <array>
#include <unordered_set>
#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/DirichletEnergy.hpp>
#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/EnvelopeEnergy.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>


#include <limits>
#include <optional>

namespace wmtk::components::simwild {

bool SimWildMesh::smooth_before(const Tuple& t)
{
    const bool r = round(t);

    const size_t vid = t.vid(*this);

    if (!m_vertex_attribute[vid].on_bbox_faces.empty()) return false;

    if (m_vertex_attribute[vid].m_is_rounded) return true;
    // try to round.
    // Note: no need to roll back.
    return r;
}

bool SimWildMesh::smooth_after(const Tuple& t)
{
    // The body lives in wmtk::optimization::smooth_vertex_3d, shared with tetwild.
    optimization::SmoothVertexOptions opts;
    opts.w_amips = m_params.w_amips;
    opts.w_envelope = m_params.w_envelope;
    opts.s_amips = m_s_amips;
    opts.s_envelope = m_s_envelope;
    opts.two_stage = true;

    return optimization::smooth_vertex_3d(*this, t, opts, m_solver.local(), &m_smooth_rejects);
}

std::shared_ptr<SampleEnvelope> SimWildMesh::smoothing_energy_envelope(const size_t vid) const
{
    // Order 2 means the vertex sits on a surface boundary or a non-manifold edge, so it is
    // pulled toward the feature-edge envelope rather than the original surface.
    const std::shared_ptr<SampleEnvelope> env =
        m_vertex_attribute[vid].m_order == 2 ? m_order_2_edge_envelope : m_envelope_orig;
    if (!env) {
        log_and_throw_error(
            "Envelope was not initialized. Vertex was of order {}",
            m_vertex_attribute[vid].m_order);
    }
    return env;
}

std::shared_ptr<SampleEnvelope> SimWildMesh::smoothing_containment_envelope(const size_t) const
{
    // The working envelope, which is not m_envelope_orig: the pull target and the
    // containment test are deliberately different objects here.
    return m_envelope;
}

void SimWildMesh::smooth_all_vertices(const size_t n_iters = 1)
{
    for (size_t i = 0; i < n_iters; ++i) {
        igl::Timer timer;
        double time;
        timer.start();
        m_smooth_rejects.reset();
        auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
        if (m_params.skip_good_regions) {
            // Only smooth vertices incident to an "active" (non-good) tet -- smoothing
            // a vertex surrounded by good tets does nothing. Smooth is the only op
            // gated this way: it never changes topology/sizing, so skipping good
            // regions cannot starve the optimizer or bloat the element count.
            for (const size_t v : active_vertices()) {
                collect_all_ops.emplace_back("vertex_smooth", tuple_from_vertex(v));
            }
        } else {
            for (auto& loc : get_vertices()) {
                collect_all_ops.emplace_back("vertex_smooth", loc);
            }
        }
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing prepare time: {:.4}s", time);
        wmtk::logger().debug("Num verts {}", collect_all_ops.size());
        if (NUM_THREADS > 0) {
            timer.start();
            auto executor = wmtk::ExecutePass<SimWildMesh>(wmtk::ExecutionPolicy::kPartition);
            executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
                return m.try_set_vertex_mutex_one_ring(e, task_id);
            };
            executor.num_threads = NUM_THREADS;
            executor(*this, collect_all_ops);
            time = timer.getElapsedTime();
            wmtk::logger().info("vertex smoothing operation time parallel: {:.4}s", time);
        } else {
            timer.start();
            auto executor = wmtk::ExecutePass<SimWildMesh>(wmtk::ExecutionPolicy::kSeq);
            executor(*this, collect_all_ops);
            time = timer.getElapsedTime();
            wmtk::logger().info("vertex smoothing operation time serial: {:.4}s", time);
        }
        logger().info("\tsmooth: {}", m_smooth_rejects.to_string());
        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
        }
    }
}


} // namespace wmtk::components::simwild