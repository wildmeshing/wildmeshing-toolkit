
#include "TetWild.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <wmtk/utils/AMIPS.h>
#include <array>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <igl/Timer.h>



#include <limits>
#include <optional>
bool tetwild::TetWild::smooth_before(const Tuple& t)
{
    if (!m_vertex_attribute[t.vid(*this)].on_bbox_faces.empty()) return false;
    if (m_vertex_attribute[t.vid(*this)].m_is_rounded) return true;
    // try to round.
    return round(t); // Note: no need to roll back.
}



bool tetwild::TetWild::smooth_after(const Tuple& t)
{
    // Newton iterations are encapsulated here.
    wmtk::logger().trace("Newton iteration for vertex smoothing.");
    auto vid = t.vid(*this);

    auto locs = get_one_ring_tets_for_vertex(t);
    auto max_quality = 0.;
    for (auto& tet : locs) {
        max_quality = std::max(max_quality, m_tet_attribute[tet.tid(*this)].m_quality);
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
        auto project = wmtk::try_project(m_vertex_attribute[vid].m_posf, old_asssembles);
        if (project) {
            m_vertex_attribute[vid].m_posf = project.value();

            auto max_after_quality = 0.;
            for (auto& loc : locs) {
                auto t_id = loc.tid(*this);
                m_tet_attribute[t_id].m_quality = get_quality(loc);
                max_after_quality = std::max(max_after_quality, m_tet_attribute[t_id].m_quality);
            }
            if (max_after_quality > max_quality) return false;
        }
        for (auto& t : locs) {
            for (auto j = 0; j < 4; j++) {
                auto f_t = tuple_from_face(t.tid(*this), j);
                auto fid = f_t.fid(*this);
                if (m_face_attribute[fid].m_is_surface_fs) {
                    auto vs = get_face_vertices(f_t);
                    if (m_envelope.is_outside(
                            {{m_vertex_attribute[vs[0].vid(*this)].m_posf,
                              m_vertex_attribute[vs[1].vid(*this)].m_posf,
                              m_vertex_attribute[vs[2].vid(*this)].m_posf}}))
                        return false;
                }
            }
        }
    }

    for (auto& loc : locs) {
        auto t_id = loc.tid(*this);
        m_tet_attribute[t_id].m_quality = get_quality(loc);
    }

    m_vertex_attribute[vid].m_pos = tetwild::to_rational(m_vertex_attribute[vid].m_posf);


    return true;
}

void tetwild::TetWild::smooth_all_vertices()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto executor = wmtk::ExecutePass<tetwild::TetWild>();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_vertices()) {
        collect_all_ops.emplace_back("vertex_smooth", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("vertex smoothing prepare time: {}s", time);
    wmtk::logger().debug("Num verts {}", collect_all_ops.size());
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<TetWild, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_vertex_mutex_one_ring(e, task_id);
        };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<TetWild, wmtk::ExecutionPolicy::kSeq>();
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time serial: {}s", time);
    }
}
