
#include "TetRemeshingMesh.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <wmtk/utils/AMIPS.h>
#include <array>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>


#include <limits>
#include <optional>

namespace wmtk::components::tet_remeshing {

bool TetRemeshingMesh::smooth_before(const Tuple& t)
{
    const size_t vid = t.vid(*this);

    if (!m_vertex_attribute[vid].on_bbox_faces.empty()) {
        return false;
    }

    // update if vertex is on boundary
    if (!is_vertex_on_boundary(vid)) {
        m_vertex_attribute[vid].m_is_on_open_boundary = false;
    }

    return true;
}


bool TetRemeshingMesh::smooth_after(const Tuple& t)
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
    int loc_id = 0;

    for (const Tuple& loc : locs) {
        auto& T = assembles[loc_id];
        auto t_id = loc.tid(*this);

        assert(!is_inverted(loc));
        std::array<size_t, 4> local_verts = oriented_tet_vids(t_id);
        local_verts = wmtk::orient_preserve_tet_reorder(local_verts, vid);

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                T[i * 3 + j] = m_vertex_attribute[local_verts[i]].m_posf[j];
            }
        }
        loc_id++;
    }

    const Vector3d old_pos = m_vertex_attribute[vid].m_posf;
    const auto old_asssembles = assembles;

    // std::vector<std::array<double, 9>> boundary_assemble;
    //// boundary must be collected before smoothing!!!
    // if (m_vertex_attribute[vid].m_is_on_open_boundary) {
    //    // debug code
    //    // std::cout << "in smoothing open boundary" << std::endl;
    //
    //    std::set<size_t> unique_eid;
    //    for (const Tuple& t : locs) {
    //        for (int j = 0; j < 6; j++) {
    //            auto e_t = tuple_from_edge(t.tid(*this), j);
    //            size_t v1_id = e_t.vid(*this);
    //            size_t v2_id = e_t.switch_vertex(*this).vid(*this);
    //            if (v1_id != vid && v2_id != vid) {
    //                // edge does not contain point of interest
    //                continue;
    //            }
    //
    //
    //            auto eid = e_t.eid(*this);
    //            auto [it, suc] = unique_eid.emplace(eid);
    //            if (!suc) continue;
    //
    //            if (is_open_boundary_edge(e_t)) {
    //                if (v2_id == vid) {
    //                    std::swap(v1_id, v2_id);
    //                }
    //                std::array<double, 9> coords = {
    //                    {m_vertex_attribute[v1_id].m_posf[0],
    //                     m_vertex_attribute[v1_id].m_posf[1],
    //                     m_vertex_attribute[v1_id].m_posf[2],
    //                     m_vertex_attribute[v2_id].m_posf[0],
    //                     m_vertex_attribute[v2_id].m_posf[1],
    //                     m_vertex_attribute[v2_id].m_posf[2],
    //                     m_vertex_attribute[v1_id].m_posf[0],
    //                     m_vertex_attribute[v1_id].m_posf[1],
    //                     m_vertex_attribute[v1_id].m_posf[2]}}; // {v1, v2, v1}
    //                boundary_assemble.emplace_back(coords);
    //            }
    //        }
    //    }
    //}

    m_vertex_attribute[vid].m_posf = wmtk::newton_method_from_stack(
        assembles,
        wmtk::AMIPS_energy,
        wmtk::AMIPS_jacobian,
        wmtk::AMIPS_hessian);

    wmtk::logger().trace(
        "old pos {} -> new pos {}",
        old_pos.transpose(),
        m_vertex_attribute[vid].m_posf.transpose());

    // project to open boundary
    if (m_vertex_attribute[vid].m_is_on_open_boundary) {
        auto project = Eigen::Vector3d();

        if (m_open_boundary_envelope.initialized()) {
            // project to envelope
            m_open_boundary_envelope.nearest_point(m_vertex_attribute[vid].m_posf, project);
        } else {
            // project to neighborhood
            log_and_throw_error("Deprecated code that should not be used.");
            // project = wmtk::try_project(m_vertex_attribute[vid].m_posf, boundary_assemble);
        }

        m_vertex_attribute[vid].m_posf = project;
        // m_vertex_attribute[vid].m_posf = 0.5 * (m_vertex_attribute[vid].m_posf + project); // Project only half way
    }

    // TODO directly store an array of Vector3d here
    std::vector<std::array<double, 9>> surface_assemble;
    if (m_vertex_attribute[vid].m_is_on_surface) {
        std::set<size_t> unique_fid;
        for (const Tuple& t : locs) {
            for (auto j = 0; j < 4; j++) {
                auto f_t = tuple_from_face(t.tid(*this), j);
                auto fid = f_t.fid(*this);
                auto [it, suc] = unique_fid.emplace(fid);
                if (!suc) continue;
                if (m_face_attribute[fid].m_is_surface_fs) {
                    auto vs_id = get_face_vids(f_t);
                    for (int k : {1, 2}) {
                        if (vs_id[k] == vid) {
                            std::swap(vs_id[k], vs_id[0]);
                        }
                    }
                    if (vs_id[0] != vid) {
                        continue; // does not contain point of interest
                    }
                    std::array<double, 9> coords;
                    for (int k = 0; k < 3; k++) {
                        for (int kk = 0; kk < 3; kk++) {
                            coords[k * 3 + kk] = m_vertex_attribute[vs_id[k]].m_posf[kk];
                        }
                    }
                    surface_assemble.emplace_back(coords);
                }
            }
        }
        if (!m_vertex_attribute[vid].m_is_on_open_boundary) {
            Vector3d project;
            if (m_envelope->initialized()) {
                m_envelope->nearest_point(m_vertex_attribute[vid].m_posf, project);
            } else {
                log_and_throw_error("Deprecated code that should not be used.");
                project = wmtk::try_project(m_vertex_attribute[vid].m_posf, surface_assemble);
            }

            m_vertex_attribute[vid].m_posf = project;
        }

        for (auto& n : surface_assemble) {
            for (int kk = 0; kk < 3; kk++) {
                n[kk] = m_vertex_attribute[vid].m_posf[kk];
            }
        }
    }

    //// update position in boundary_assemble
    // for (auto& n : boundary_assemble) {
    //     n[0] = m_vertex_attribute[vid].m_posf[0];
    //     n[1] = m_vertex_attribute[vid].m_posf[1];
    //     n[2] = m_vertex_attribute[vid].m_posf[2];
    //     n[6] = m_vertex_attribute[vid].m_posf[0];
    //     n[7] = m_vertex_attribute[vid].m_posf[1];
    //     n[8] = m_vertex_attribute[vid].m_posf[2];
    // }
    //
    //// check boundary containment
    // for (const auto& n : boundary_assemble) {
    //     std::array<Eigen::Vector3d, 3> tri;
    //     for (int k = 0; k < 3; k++) {
    //         for (int kk = 0; kk < 3; kk++) {
    //             tri[k][kk] = n[k * 3 + kk];
    //         }
    //     }
    //     if (m_open_boundary_envelope.is_outside(tri)) {
    //         return false;
    //     }
    // }

    // check surface containment
    for (const auto& n : surface_assemble) {
        std::array<Eigen::Vector3d, 3> tri;
        for (int k = 0; k < 3; k++) {
            for (int kk = 0; kk < 3; kk++) {
                tri[k][kk] = n[k * 3 + kk];
            }
        }
        if (m_envelope->is_outside(tri)) {
            return false;
        }
    }

    // quality
    auto max_after_quality = 0.;
    for (const Tuple& loc : locs) {
        if (is_inverted(loc)) {
            return false;
        }
        auto t_id = loc.tid(*this);
        m_tet_attribute[t_id].m_quality = get_quality(loc);
        max_after_quality = std::max(max_after_quality, m_tet_attribute[t_id].m_quality);
    }
    if (std::cbrt(max_after_quality) > m_params.stop_energy && max_after_quality > max_quality) {
        return false;
    }

    return true;
}

void TetRemeshingMesh::pull_towards_smooth_surface()
{
    for (const Tuple& t : get_vertices()) {
        const size_t vid = t.vid(*this);
        if (!m_vertex_attribute[vid].m_is_on_surface) {
            continue;
        }
        if (m_vertex_attribute[vid].m_is_frozen) {
            continue;
        }

        SimpleBVH::BVH& bvh =
            m_vertex_attribute[vid].m_is_on_feature_edge ? *m_smooth_edges : *m_smooth_surface;

        Vector3d& p = m_vertex_attribute[vid].m_posf;

        Vector3d p_target;
        double sq_dist;
        bvh.nearest_facet(p, p_target, sq_dist);

        const Vector3d p_old = p;
        p = p_target;

        const auto locs = get_one_ring_tets_for_vertex(t);

        bool success = true;
        for (size_t i = 0; i < 10; ++i) {
            for (const Tuple& loc : locs) {
                if (is_inverted(loc)) {
                    success = false;
                    break;
                }
            }

            if (success) {
                break;
            }

            p = 0.5 * (p_old + p);
        }

        if (!success) {
            p = p_old;
        }
    }
}

void TetRemeshingMesh::smooth_all_vertices()
{
    // the order is randomized in every iteration but deterministic when executed sequentially
    static int rnd_seed = 0;
    srand(rnd_seed++);

    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_vertices()) {
        collect_all_ops.emplace_back("vertex_smooth", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("vertex smoothing prepare time: {:.4}s", time);
    wmtk::logger().debug("Num verts {}", collect_all_ops.size());
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<TetRemeshingMesh, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_vertex_mutex_one_ring(e, task_id);
        };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time parallel: {:.4}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<TetRemeshingMesh, wmtk::ExecutionPolicy::kSeq>();
        // executor.priority = [&](auto& m, auto op, auto& t) -> double { return rand(); };
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time serial: {:.4}s", time);
    }
}

} // namespace wmtk::components::tet_remeshing