
#include "TriWildMesh.h"

#include <cstdlib>

#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/utils/RunPass.hpp>

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
        wmtk::run_pass(
            *this,
            wmtk::PassLock::VertexOneRing,
            "vertex smoothing",
            [&](auto& executor, auto& m) { executor(m, collect_all_ops); });
        logger().info("\tsmooth: {}", m_smooth_rejects.to_string());
        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
        }
    }
}


} // namespace wmtk::components::triwild