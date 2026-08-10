
#include "TriWildMesh.h"

#include <cstdlib>

#include <wmtk/optimization/SmoothVertex.hpp>
#include "wmtk/ExecutionScheduler.hpp"

#include <igl/Timer.h>
#include <array>
#include <limits>
#include <optional>
#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/EnvelopeEnergy.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::triwild {

bool TriWildMesh::smooth_before(const Tuple& t)
{
    // try to round.
    const bool r = round(t);

    const size_t vid = t.vid(*this);

    if (!m_vertex_attribute[vid].on_bbox_faces.empty()) {
        return false;
    }

    if (m_vertex_attribute[vid].m_is_rounded) {
        return true;
    }
    // Note: no need to roll back.
    return r;
}


bool TriWildMesh::smooth_after(const Tuple& t)
{
    // The body lives in wmtk::optimization::smooth_vertex_2d, shared with simwild's 2D mesh.
    optimization::SmoothVertexOptions opts;
    opts.w_amips = m_params.w_amips;
    opts.w_envelope = m_params.w_envelope;
    opts.s_amips = m_s_amips;
    opts.s_envelope = m_s_envelope;
    opts.two_stage = true;
    opts.quality_veto_on_surface = false;
    // DEBUG: WMTK_SURF_VETO=1 refuses a surface-vertex move that worsens the worst incident
    // element, the way interior vertices are already treated.
    if (const char* e = std::getenv("WMTK_SURF_VETO")) opts.quality_veto_on_surface = std::atoi(e) != 0;
    // DEBUG: WMTK_GLOBAL_CAP=0 turns the narrow guard off.
    const char* gc = std::getenv("WMTK_GLOBAL_CAP");
    if (!gc || std::atoi(gc) != 0) opts.global_max_quality = m_smooth_global_max_quality;

    return optimization::smooth_vertex_2d(*this, t, opts, m_solver.local(), &m_smooth_rejects);
}

Vector2d TriWildMesh::smoothing_position(const size_t vid) const
{
    return m_vertex_attribute[vid].m_posf;
}

void TriWildMesh::set_smoothing_position(const size_t vid, const Vector2d& p)
{
    m_vertex_attribute[vid].m_posf = p;
    m_vertex_attribute[vid].m_pos = to_rational(p);
}

std::shared_ptr<SampleEnvelope> TriWildMesh::smoothing_energy_envelope(const size_t) const
{
    return m_envelope;
}

std::shared_ptr<SampleEnvelope> TriWildMesh::smoothing_containment_envelope(const size_t) const
{
    // triwild keeps a single envelope, so the pull target and the containment test coincide.
    return m_envelope;
}

void TriWildMesh::smooth_all_vertices(const size_t n_iters)
{
    for (size_t i = 0; i < n_iters; ++i) {
        // log_total_surface_energy();
        igl::Timer timer;
        timer.start();
        m_smooth_rejects.reset();
        m_smooth_global_max_quality = -1.;
        for (int i_ = 0; i_ < tri_capacity(); ++i_) {
            if (!tuple_from_tri(i_).is_valid(*this)) continue;
            m_smooth_global_max_quality =
                std::max(m_smooth_global_max_quality, m_face_attribute[i_].m_quality);
        }
        std::vector<std::pair<std::string, Tuple>> collect_all_ops;
        if (m_params.skip_good_regions) {
            // Only smooth vertices incident to an "active" (non-good) face -- smoothing a
            // vertex surrounded by good faces does nothing. Smooth is the only op gated
            // this way: it never changes topology/sizing, so skipping good regions cannot
            // starve the optimizer or bloat the element count.
            for (const size_t v : active_vertices()) {
                collect_all_ops.emplace_back("vertex_smooth", tuple_from_vertex(v));
            }
        } else {
            for (const Tuple& t : get_vertices()) {
                collect_all_ops.emplace_back("vertex_smooth", t);
            }
        }
        logger().info("vertex smoothing prepare time: {:.4}s", timer.getElapsedTimeInSec());
        logger().info("#V = {}", collect_all_ops.size());
        if (NUM_THREADS > 0) {
            timer.start();
            ExecutePass<TriWildMesh> executor(ExecutionPolicy::kPartition);
            executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
                return m.try_set_vertex_mutex_one_ring(e, task_id);
            };
            executor.num_threads = NUM_THREADS;
            executor(*this, collect_all_ops);
            logger().info("vertex smoothing time parallel: {:.4}s", timer.getElapsedTimeInSec());
        } else {
            timer.start();
            ExecutePass<TriWildMesh> executor(ExecutionPolicy::kSeq);
            executor(*this, collect_all_ops);
            logger().info("vertex smoothing time serial: {:.4}s", timer.getElapsedTimeInSec());
        }
        logger().info("\tsmooth: {}", m_smooth_rejects.to_string());
        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
        }
    }
}


} // namespace wmtk::components::triwild