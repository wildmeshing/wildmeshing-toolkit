#include <wmtk/TriOptimizerMesh.h>

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/RunPass.hpp>
#include <wmtk/utils/SizingField.hpp>

#include <igl/Timer.h>
#include <spdlog/fmt/bundled/format.h>

namespace wmtk {

bool TriOptimizerMesh::smooth_before(const Tuple& t)
{
    // Smoothing writes a double position. An exact vertex that cannot first be rounded must
    // keep its rational position, because that is the only position preserving its one-ring.
    const bool rounded = round(t);
    const size_t vid = t.vid(*this);

    if (!m_vertex_attribute[vid].on_bbox_faces.empty()) {
        return false;
    }
    return m_vertex_attribute[vid].m_is_rounded || rounded;
}

bool TriOptimizerMesh::smooth_after(const Tuple& t)
{
    optimization::SmoothVertexOptions opts;
    opts.w_amips = m_params.w_amips;
    opts.w_envelope = m_params.w_envelope;
    opts.s_amips = m_s_amips;
    opts.s_envelope = m_s_envelope;
    // Off: the balanced warm-up's line search accepts tangential slides at any configured
    // w_amips (its objective contains no small weight), producing a weight-independent
    // surface-deviation floor (~4e-5 on triwild20k_202090, flat from w=1e-3 to 1e-12) that
    // vanishes with the warm-up off, at max energy unchanged to four digits and +1.8%
    // iterations on the 53 registered configs. Tangential mobility never needed the warm-up:
    // both the tangential gradient and Hessian scale with w_amips, so Newton cancels the
    // weight and vertices slide identically at any fitting strength.
    opts.two_stage = false;
    opts.smoothing_mode = m_params.smoothing_mode == "exact"
                              ? optimization::SmoothVertexOptions::SmoothingMode::Exact
                              : optimization::SmoothVertexOptions::SmoothingMode::Projected;
    opts.project_line_search_steps = m_params.project_line_search_steps;
    opts.project_line_search_nested_steps = m_params.project_line_search_nested_steps;
    opts.reject_backoff_steps = m_params.smooth_reject_backoff_steps;

    // Deliberately retain TriWild's default quality_veto_on_surface=true. A homogeneous
    // SimWild mesh must accept and reject the same candidate positions as TriWild.
    return optimization::smooth_vertex_2d(*this, t, opts, m_solver.local(), &m_smooth_rejects);
}

Vector2d TriOptimizerMesh::smoothing_position(const size_t vid) const
{
    return m_vertex_attribute[vid].m_posf;
}

void TriOptimizerMesh::set_smoothing_position(const size_t vid, const Vector2d& p)
{
    m_vertex_attribute[vid].m_posf = p;
    m_vertex_attribute[vid].m_pos = to_rational(p);
}

void TriOptimizerMesh::smooth_all_vertices(const size_t n_iters)
{
    for (size_t i = 0; i < n_iters; ++i) {
        igl::Timer timer;
        timer.start();
        m_smooth_rejects.reset();

        std::vector<std::pair<std::string, Tuple>> collect_all_ops;
        if (m_params.skip_good_regions) {
            for (const size_t vid : active_vertices()) {
                collect_all_ops.emplace_back("vertex_smooth", tuple_from_vertex(vid));
            }
        } else {
            for (const Tuple& t : get_vertices()) {
                collect_all_ops.emplace_back("vertex_smooth", t);
            }
        }

        logger().info("vertex smoothing prepare time: {:.4}s", timer.getElapsedTimeInSec());
        logger().info("#V = {}", collect_all_ops.size());
        run_pass(*this, PassLock::VertexRing, "vertex smoothing", [&](auto& executor, auto& mesh) {
            executor(mesh, collect_all_ops);
        });
        logger().info("\tsmooth: {}", m_smooth_rejects.to_string());

        optimization_debug_checkpoint();
    }
}

std::vector<size_t> TriOptimizerMesh::active_vertices() const
{
    return utils::active_vertices(
        vert_capacity(),
        tri_capacity(),
        [this](size_t fid) { return tuple_from_tri(fid).is_valid(*this); },
        [this](size_t fid) { return m_face_attribute[fid].m_quality; },
        [this](size_t fid) { return oriented_tri_vids(fid); },
        active_quality_threshold(),
        [this](size_t vid) { return m_vertex_attribute[vid].m_is_on_surface; });
}

} // namespace wmtk
