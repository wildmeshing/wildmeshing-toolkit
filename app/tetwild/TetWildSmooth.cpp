
#include "TetWild.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <wmtk/utils/AMIPS.h>
#include <array>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>

#include <igl/point_simplex_squared_distance.h>

#include <limits>
#include <optional>
bool tetwild::TetWild::smooth_before(const Tuple& t)
{
    if (m_vertex_attribute[t.vid(*this)].m_is_rounded) return true;
    // try to round.
    return round(t); // Note: no need to roll back.
}

auto try_project(
    const Eigen::Vector3d& point,
    const std::vector<std::array<double, 12>>& assembled_neighbor) -> std::optional<Eigen::Vector3d>
{
    auto min_dist = std::numeric_limits<double>::infinity();
    Eigen::Vector3d closest_point = Eigen::Vector3d::Zero();
    for (const auto& tri : assembled_neighbor) {
        auto V = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(tri.data());
        Eigen::Vector3d project;
        auto dist2 = -1;
        igl::point_simplex_squared_distance<3>(
            point,
            V,
            Eigen::RowVector3i(0, 1, 2),
            0,
            dist2,
            project);
        // TODO: libigl might not be robust how to verify this?
        if (dist2 < min_dist) {
            min_dist = dist2;
            closest_point = project;
        }
    }
    return closest_point;
}

bool tetwild::TetWild::smooth_after(const Tuple& t)
{
    // Newton iterations are encapsulated here.
    // TODO: bbox/surface tags.
    // TODO: envelope check.
    wmtk::logger().trace("Newton iteration for vertex smoothing.");
    auto vid = t.vid(*this);

    auto locs = get_one_ring_tets_for_vertex(t);
    auto max_quality = 0.;
    for (auto& tet : locs) {
        max_quality = std::max(max_quality, get_quality(tet));
    }

    assert(locs.size() > 0);
    std::vector<std::array<double, 12>> assembles(locs.size());
    auto loc_id = 0;

    for (auto& loc : locs) {
        auto& T = assembles[loc_id];
        auto t_id = loc.tid(*this);

        assert(!is_inverted(loc));
        auto local_tuples = oriented_tet_vertices(loc);
        std::array<size_t, 4> local_verts;
        for (auto i = 0; i < 4; i++) {
            local_verts[i] = local_tuples[i].vid(*this);
        }

        local_verts = wmtk::orient_preserve_tet_reorder(local_verts, vid);

        for (auto i = 0; i < 4; i++) {
            for (auto j = 0; j < 3; j++) {
                T[i * 3 + j] = m_vertex_attribute[local_verts[i]].m_posf[j];
            }
        }
        loc_id++;
    }

    auto old_pos = m_vertex_attribute[vid].m_posf;
    auto old_asssembles = assembles;
    m_vertex_attribute[vid].m_posf = wmtk::newton_method_from_stack(
        assembles,
        wmtk::AMIPS_energy,
        wmtk::AMIPS_jacobian,
        wmtk::AMIPS_hessian);
    wmtk::logger().trace(
        "old pos {} -> new pos {}",
        old_pos.transpose(),
        m_vertex_attribute[vid].m_posf.transpose());

    if (m_vertex_attribute[vid].m_is_on_surface) {
        auto project = try_project(m_vertex_attribute[vid].m_posf, old_asssembles);
        if (project) {
            m_vertex_attribute[vid].m_posf = project.value();
        }
    }

    m_vertex_attribute[vid].m_pos = tetwild::to_rational(m_vertex_attribute[vid].m_posf);

    auto max_after_quality = 0.;
    for (auto& loc : locs) {
        auto t_id = loc.tid(*this);
        m_tet_attribute[t_id].m_quality = get_quality(loc);
        max_after_quality = std::max(max_after_quality, m_tet_attribute[t_id].m_quality);
    }
    if (max_after_quality > max_quality) return false;
    return true;
}

void tetwild::TetWild::smooth_all_vertices()
{
    auto executor = wmtk::ExecutePass<tetwild::TetWild>();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_vertices()) {
        collect_all_ops.emplace_back("vertex_smooth", loc);
    }
    wmtk::logger().info("Num verts {}", collect_all_ops.size());
    if (NUM_THREADS > 1) {
        auto executor = wmtk::ExecutePass<TetWild, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [&](auto& m, const auto& e) -> std::optional<std::vector<size_t>> {
            auto stack = std::vector<size_t>();
            if (!m.try_set_vertex_mutex_two_ring(e, stack)) return {};
            return stack;
        };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    } else {
        auto executor = wmtk::ExecutePass<TetWild, wmtk::ExecutionPolicy::kSeq>();
        executor(*this, collect_all_ops);
    }
}
