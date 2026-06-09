
#include "TriWildMesh.h"
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
    // Newton iterations are encapsulated here.
    logger().trace("Newton iteration for vertex smoothing.");
    const size_t vid = t.vid(*this);

    const auto& VA = m_vertex_attribute;

    const auto& locs = get_one_ring_fids_for_vertex(t);
    assert(locs.size() > 0);

    double max_quality = 0.;
    for (const size_t fid : locs) {
        max_quality = std::max(max_quality, m_face_attribute[fid].m_quality);
        if (is_inverted_f(fid)) {
            // Neighbors that are not rounded could cause a face to be inverted in floats
            return false;
        }
    }

    auto& solver = m_solver.local();
    if (!solver) {
        solver = optimization::create_basic_solver();
    }

    std::vector<std::array<double, 6>> assembles = get_amips_assembles(t);

    const Vector2d old_pos = VA[vid].m_posf;

    auto amips_energy = get_amips_energy(t);
    std::shared_ptr<polysolve::nonlinear::Problem> total_energy = amips_energy;

    auto solve = [&]() {
        VectorXd x = VA[vid].m_posf;
        try {
            solver->minimize(*total_energy, x);
        } catch (const std::exception&) {
            // polysolve might throw errors that we want to ignore (e.g., line search failed)
        }
        m_vertex_attribute[vid].m_posf = x;
        m_vertex_attribute[vid].m_pos = to_rational(Vector2d(x));
    };

    const auto surf_assembles = get_surface_assembles(t);
    if (VA[vid].m_is_on_surface) {
        assert(!surf_assembles.empty());

        auto energy_sum = std::make_shared<optimization::EnergySum>();

        auto envelope_energy = get_envelope_energy(t);
        // do one solve without weights for amips and envelope
        if (m_params.w_amips > 0) {
            energy_sum->add_energy(amips_energy, 1. / m_params.w_amips);
        }
        if (m_params.w_envelope > 0) {
            energy_sum->add_energy(envelope_energy, 1. / m_params.w_envelope);
        }
        total_energy = energy_sum;
        solve();

        // second solve (with proper weights)
        energy_sum = std::make_shared<optimization::EnergySum>();

        if (m_params.w_amips > 0) {
            energy_sum->add_energy(amips_energy);
        }
        if (m_params.w_envelope > 0) {
            energy_sum->add_energy(envelope_energy);
        }
        total_energy = energy_sum;
        solve();
    } else {
        // not on the surface
        solve();
    }

    logger().trace("old pos {} -> new pos {}", old_pos.transpose(), VA[vid].m_posf.transpose());

    // check surface containment
    if (VA[vid].m_is_on_surface) {
        for (size_t i = 1; i < surf_assembles.size(); ++i) {
            std::array<Eigen::Vector2d, 2> edge;
            edge[0] = VA[vid].m_posf;
            edge[1] = surf_assembles[i];
            if (m_envelope->is_outside(edge)) {
                return false;
            }
        }
    }

    // quality (only check if not on surface)
    auto max_after_quality = 0.;
    for (const size_t fid : locs) {
        if (is_inverted(fid)) {
            return false;
        }
        const double q = get_quality(fid);
        m_face_attribute[fid].m_quality = q;
        max_after_quality = std::max(max_after_quality, q);
    }
    if (!VA[vid].m_is_on_surface) {
        if (max_after_quality > max_quality) {
            return false;
        }
    }

    return true;
}

void TriWildMesh::smooth_all_vertices(const size_t n_iters)
{
    for (size_t i = 0; i < n_iters; ++i) {
        // log_total_surface_energy();
        igl::Timer timer;
        timer.start();
        std::vector<std::pair<std::string, Tuple>> collect_all_ops;
        for (const Tuple& t : get_vertices()) {
            collect_all_ops.emplace_back("vertex_smooth", t);
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
        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
        }
    }
}

std::vector<Vector2d> TriWildMesh::get_surface_assembles(const Tuple& t) const
{
    const auto& VA = m_vertex_attribute;
    const size_t vid = t.vid(*this);

    std::vector<Vector2d> surface_pts;

    if (!VA[vid].m_is_on_surface) {
        return surface_pts;
    }

    const auto es = get_surface_edges_for_vertex(vid);

    surface_pts.resize(es.size() + 1);
    surface_pts[0] = VA[vid].m_posf;

    for (size_t i = 0; i < es.edges().size(); ++i) {
        const auto& vs = es.edges()[i].vertices();
        size_t neighbor_id = vs[0] != vid ? vs[0] : vs[1];
        surface_pts[i + 1] = VA[neighbor_id].m_posf;
    }

    return surface_pts;
}

std::shared_ptr<polysolve::nonlinear::Problem> TriWildMesh::get_envelope_energy(
    const Tuple& t) const
{
    const double w = m_s_envelope * m_params.w_envelope;

    auto envelope_energy = std::make_shared<optimization::EnvelopeEnergy2D>(m_envelope, w);
    return envelope_energy;
}

std::vector<std::array<double, 6>> TriWildMesh::get_amips_assembles(const Tuple& t) const
{
    const size_t vid = t.vid(*this);
    const auto& locs = get_one_ring_fids_for_vertex(t);

    const auto& VA = m_vertex_attribute;

    std::vector<std::array<double, 6>> assembles;
    assembles.reserve(locs.size());

    for (const size_t fid : locs) {
        if (is_inverted(fid)) {
            log_and_throw_error("Inverted face in amips assemble!");
        }
        std::array<size_t, 3> local_verts = oriented_tri_vids(fid);
        {
            size_t v_loc = 0;
            for (size_t i = 0; i < 3; ++i) {
                if (local_verts[i] == vid) {
                    v_loc = i;
                    break;
                }
            }
            std::array<size_t, 3> buf = local_verts;
            local_verts[0] = buf[v_loc];
            local_verts[1] = buf[(v_loc + 1) % 3];
            local_verts[2] = buf[(v_loc + 2) % 3];
        }

        std::array<double, 6> T;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 2; j++) {
                T[i * 2 + j] = VA[local_verts[i]].m_posf[j];
            }
        }
        assembles.push_back(T);
    }

    return assembles;
}

std::shared_ptr<polysolve::nonlinear::Problem> TriWildMesh::get_amips_energy(const Tuple& t) const
{
    const double w = m_params.w_amips > 0 ? m_s_amips * m_params.w_amips : 1;

    const auto assembles = get_amips_assembles(t);
    auto amips_energy = std::make_shared<optimization::AMIPSEnergy2D>(assembles, w);
    assert(amips_energy->initial_position() == m_vertex_attribute.at(t.vid(*this)).m_posf);
    return amips_energy;
}

} // namespace wmtk::components::triwild