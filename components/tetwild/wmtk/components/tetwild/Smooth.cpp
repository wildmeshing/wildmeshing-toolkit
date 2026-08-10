
#include "TetWildMesh.h"

#include <cstdlib>
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <wmtk/utils/AMIPS.h>
#include <array>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>


#include <limits>
#include <optional>

namespace wmtk::components::tetwild {

bool TetWildMesh::smooth_before(const Tuple& t)
{
    const bool r = round(t);

    const size_t vid = t.vid(*this);

    if (!m_vertex_attribute[vid].on_bbox_faces.empty()) return false;

    if (m_vertex_attribute[vid].m_is_rounded) return true;
    // try to round.
    // Note: no need to roll back.
    return r;
}


bool TetWildMesh::smooth_after(const Tuple& t)
{
    // The body lives in wmtk::optimization::smooth_vertex_3d, shared with simwild.
    //
    // This replaces the previous scheme, which optimized AMIPS alone and then snapped the
    // vertex onto the envelope with nearest_point. That snap moved a drifted vertex the
    // whole way in one jump, which almost always worsened an incident tet, so the operation
    // was vetoed and the rollback discarded the projection -- a vertex near the envelope
    // wall could never come back. The envelope is now a term in the objective instead, so
    // the pull is gradual and the line search refuses steps that leave the envelope.
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

    return optimization::smooth_vertex_3d(*this, t, opts, m_solver.local(), &m_smooth_rejects);
}

std::shared_ptr<SampleEnvelope> TetWildMesh::smoothing_energy_envelope(const size_t vid) const
{
    // Order 2 means the vertex is on a surface boundary or a non-manifold edge. This is
    // broader than the old m_is_on_open_boundary flag, which covered only open boundaries.
    if (get_order_of_vertex(vid) >= 2 && m_order2_envelope && m_order2_envelope->initialized()) {
        return m_order2_envelope;
    }
    return m_envelope;
}

std::shared_ptr<SampleEnvelope> TetWildMesh::smoothing_containment_envelope(const size_t) const
{
    // tetwild keeps one surface envelope, so pull target and containment test coincide --
    // unlike simwild, which has a separate working envelope.
    return m_envelope;
}

void TetWildMesh::smooth_all_vertices()
{
    // the order is randomized in every iteration but deterministic when executed sequentially
    static int rnd_seed = 0;
    srand(rnd_seed++);

    igl::Timer timer;
    double time;
    timer.start();
    m_smooth_rejects.reset();
    m_smooth_global_max_quality = -1.;
    for (int i_ = 0; i_ < tet_capacity(); ++i_) {
        if (!tuple_from_tet(i_).is_valid(*this)) continue;
        m_smooth_global_max_quality =
            std::max(m_smooth_global_max_quality, m_tet_attribute[i_].m_quality);
    }
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
        auto executor = wmtk::ExecutePass<TetWildMesh>(wmtk::ExecutionPolicy::kPartition);
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_vertex_mutex_one_ring(e, task_id);
        };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time parallel: {:.4}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<TetWildMesh>(wmtk::ExecutionPolicy::kSeq);
        // executor.priority = [&](auto& m, auto op, auto& t) -> double { return rand(); };
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time serial: {:.4}s", time);
    }
    wmtk::logger().info("\tsmooth: {}", m_smooth_rejects.to_string());
}

} // namespace wmtk::components::tetwild