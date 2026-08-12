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
    opts.two_stage = true;
    opts.spring_pull = m_params.spring_pull;

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
        run_pass(
            *this,
            PassLock::VertexOneRing,
            "vertex smoothing",
            [&](auto& executor, auto& mesh) { executor(mesh, collect_all_ops); });
        logger().info("\tsmooth: {}", m_smooth_rejects.to_string());

        if (m_params.debug_output) {
            write_smoothing_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
        }
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
