
#include "ImageSimulationMesh.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <wmtk/utils/AMIPS.h>
#include <array>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>


#include <limits>
#include <optional>

namespace wmtk::components::image_simulation {

bool ImageSimulationMesh::smooth_before(const Tuple& t)
{
    if (!m_vertex_attribute[t.vid(*this)].on_bbox_faces.empty()) {
        return false;
    }

    if (m_vertex_attribute[t.vid(*this)].m_is_rounded) {
        return true;
    }
    // try to round.
    // Note: no need to roll back.
    return round(t);
}


bool ImageSimulationMesh::smooth_after(const Tuple& t)
{
    // Newton iterations are encapsulated here.
    wmtk::logger().trace("Newton iteration for vertex smoothing.");
    const size_t vid = t.vid(*this);

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


    if (m_vertex_attribute[vid].m_is_on_open_boundary) {
        // debug code
        // std::cout << "in smoothing open boundary" << std::endl;

        std::vector<std::array<double, 9>> neighbor_assemble;
        std::set<size_t> unique_eid;
        for (auto& t : locs) {
            for (auto j = 0; j < 6; j++) {
                auto e_t = tuple_from_edge(t.tid(*this), j);
                auto eid = e_t.eid(*this);
                auto [it, suc] = unique_eid.emplace(eid);
                if (!suc) continue;
                if (is_open_boundary_edge(e_t)) {
                    size_t v1_id = e_t.vid(*this);
                    size_t v2_id = e_t.switch_vertex(*this).vid(*this);
                    if (v2_id == vid) {
                        size_t tmp = v1_id;
                        v1_id = v2_id;
                        v2_id = tmp;
                    }
                    if (v1_id != vid) continue; // does not contain point of interest
                    std::array<double, 9> coords = {
                        {m_vertex_attribute[v1_id].m_posf[0],
                         m_vertex_attribute[v1_id].m_posf[1],
                         m_vertex_attribute[v1_id].m_posf[2],
                         m_vertex_attribute[v2_id].m_posf[0],
                         m_vertex_attribute[v2_id].m_posf[1],
                         m_vertex_attribute[v2_id].m_posf[2],
                         m_vertex_attribute[v1_id].m_posf[0],
                         m_vertex_attribute[v1_id].m_posf[1],
                         m_vertex_attribute[v1_id].m_posf[2]}}; // {v1, v2, v1}
                    neighbor_assemble.emplace_back(coords);
                }
            }
        }

        {
            auto project = Eigen::Vector3d();

            if (boundaries_tree.initialized()) {
                // project to envelope
                // std::cout << "in smoothing open boundary" << std::endl;
                boundaries_tree.nearest_point(m_vertex_attribute[vid].m_posf, project);
            } else {
                // project to neighborhood
                project = wmtk::try_project(m_vertex_attribute[vid].m_posf, neighbor_assemble);
            }

            m_vertex_attribute[vid].m_posf = project;
        }

        for (auto& n : neighbor_assemble) {
            n[0] = m_vertex_attribute[vid].m_posf[0];
            n[1] = m_vertex_attribute[vid].m_posf[1];
            n[2] = m_vertex_attribute[vid].m_posf[2];
            n[6] = m_vertex_attribute[vid].m_posf[0];
            n[7] = m_vertex_attribute[vid].m_posf[1];
            n[8] = m_vertex_attribute[vid].m_posf[2];
        }
        for (auto& n : neighbor_assemble) {
            auto boundaries = std::array<Eigen::Vector3d, 3>();
            for (auto k = 0; k < 3; k++) {
                for (auto kk = 0; kk < 3; kk++) boundaries[k][kk] = n[k * 3 + kk];
            }
            bool is_out = m_open_boundary_envelope.is_outside(boundaries);
            if (is_out) return false;
        }
    } else if (m_vertex_attribute[vid].m_is_on_surface) {
        std::vector<std::array<double, 9>> neighbor_assemble; // incident surface triangles
        std::set<size_t> unique_fid;
        for (const Tuple& t : locs) {
            for (int j = 0; j < 4; j++) {
                const Tuple f_t = tuple_from_face(t.tid(*this), j);
                const size_t fid = f_t.fid(*this);
                if (!m_face_attribute[fid].m_is_surface_fs) {
                    continue;
                }
                const auto [it, suc] = unique_fid.emplace(fid);
                if (!suc) {
                    continue;
                }
                const auto vs = get_face_vertices(f_t);
                auto vs_id = std::array<size_t, 3>();
                for (int k = 0; k < 3; k++) {
                    vs_id[k] = vs[k].vid(*this);
                }
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
                neighbor_assemble.emplace_back(coords);
            }
        }
        // auto project = wmtk::try_project(m_vertex_attribute[vid].m_posf, neighbor_assemble);
        {
            Vector3d project;

            if (triangles_tree->initialized()) {
                triangles_tree->nearest_point(m_vertex_attribute[vid].m_posf, project);
            } else {
                // Does that even make sense? The vertex is part of these triangles so this
                // projection doesn't do anything.
                project = wmtk::try_project(m_vertex_attribute[vid].m_posf, neighbor_assemble);
            }

            m_vertex_attribute[vid].m_posf = project;
        }

        for (auto& n : neighbor_assemble) {
            for (auto kk = 0; kk < 3; kk++) {
                n[kk] = m_vertex_attribute[vid].m_posf[kk];
            }
        }
        for (auto& n : neighbor_assemble) {
            auto tris = std::array<Eigen::Vector3d, 3>();
            for (auto k = 0; k < 3; k++) {
                for (auto kk = 0; kk < 3; kk++) {
                    tris[k][kk] = n[k * 3 + kk];
                }
            }
            bool is_out = m_envelope->is_outside(tris);
            if (is_out) {
                return false;
            }
        }
    }

    // quality
    auto max_after_quality = 0.;
    for (auto& loc : locs) {
        if (is_inverted(loc)) {
            return false;
        }
        auto t_id = loc.tid(*this);
        m_tet_attribute[t_id].m_quality = get_quality(loc);
        max_after_quality = std::max(max_after_quality, m_tet_attribute[t_id].m_quality);
    }
    if (max_after_quality > max_quality) {
        return false;
    }


    m_vertex_attribute[vid].m_pos = to_rational(m_vertex_attribute[vid].m_posf);


    return true;
}

void ImageSimulationMesh::smooth_all_vertices()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_vertices()) {
        collect_all_ops.emplace_back("vertex_smooth", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("vertex smoothing prepare time: {}s", time);
    wmtk::logger().debug("Num verts {}", collect_all_ops.size());
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<ImageSimulationMesh, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_vertex_mutex_one_ring(e, task_id);
        };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<ImageSimulationMesh, wmtk::ExecutionPolicy::kSeq>();
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time serial: {}s", time);
    }
}

void ImageSimulationMesh::smooth_input(const int n_iterations)
{
    const auto vertices = get_vertices();

    for (int i = 0; i < n_iterations; ++i) {
        for (const Tuple& t : vertices) {
            const size_t vid = t.vid(*this);
            if (!m_vertex_attribute[vid].on_bbox_faces.empty()) {
                continue;
            }
            if (!m_vertex_attribute[vid].m_is_rounded) {
                continue;
            }

            const auto locs = get_one_ring_tets_for_vertex(t);

            if (m_vertex_attribute[vid].m_is_on_surface) {
                // on surface --> try laplacian smoothing

                std::vector<std::array<Vector3d, 3>>
                    neighbor_assemble; // incident surface triangles
                std::set<size_t> unique_fid;
                for (const Tuple& t : locs) {
                    for (int j = 0; j < 4; j++) {
                        const Tuple f_t = tuple_from_face(t.tid(*this), j);
                        const size_t fid = f_t.fid(*this);
                        if (!m_face_attribute[fid].m_is_surface_fs) {
                            continue;
                        }
                        const auto [it, suc] = unique_fid.emplace(fid);
                        if (!suc) {
                            continue;
                        }
                        const auto vs = get_face_vertices(f_t);
                        auto vs_id = std::array<size_t, 3>();
                        for (int k = 0; k < 3; k++) {
                            vs_id[k] = vs[k].vid(*this);
                        }
                        for (int k : {1, 2}) {
                            if (vs_id[k] == vid) {
                                std::swap(vs_id[k], vs_id[0]);
                            }
                        }
                        if (vs_id[0] != vid) {
                            continue; // does not contain point of interest
                        }
                        std::array<Vector3d, 3> coords;
                        for (int k = 0; k < 3; k++) {
                            coords[k] = m_vertex_attribute[vs_id[k]].m_posf;
                        }
                        neighbor_assemble.emplace_back(coords);
                    }
                }

                // TODO smooth
                Vector3d p_lap = Vector3d::Zero();
                for (const auto& n : neighbor_assemble) {
                    p_lap += n[1] + n[2];
                }
                p_lap /= (neighbor_assemble.size() * 2);

                const Vector3d p_old = m_vertex_attribute[vid].m_posf;

                bool success = true;
                const int n_bisections = 10;
                for (int j = 0; j < n_bisections; ++j) {
                    // try to find a valid position
                    double u = 1.0 / (1 << j);
                    Vector3d p = u * p_lap + (1.0 - u) * p_old;

                    // update position of vid in neighbor_assamble
                    for (auto& n : neighbor_assemble) {
                        n[0] = p;
                    }
                    success = true;
                    for (const auto& n : neighbor_assemble) {
                        if (m_envelope->is_outside(n)) {
                            success = false;
                            break;
                        }
                    }

                    if (!success) {
                        continue;
                    }

                    // check for tet validity
                    m_vertex_attribute[vid].m_posf = p;
                    for (const Tuple& loc : locs) {
                        if (is_inverted(loc)) {
                            success = false;
                            break;
                        }
                    }

                    if (success) {
                        m_vertex_attribute[vid].m_pos = to_rational(p);
                        break;
                    } else {
                        m_vertex_attribute[vid].m_posf = p_old;
                    }
                }

                for (const Tuple& loc : locs) {
                    if (is_inverted(loc)) {
                        log_and_throw_error("Input surface laplacian smooth caused inversion");
                    }
                }

            } else {
                // interior --> amips optimization

                auto max_quality = 0.;
                for (const Tuple& tet : locs) {
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

                // quality
                double max_after_quality = 0.;
                for (const Tuple& loc : locs) {
                    if (is_inverted(loc)) {
                        log_and_throw_error("Input smoothing caused inversion");
                    }
                    const size_t t_id = loc.tid(*this);
                    m_tet_attribute[t_id].m_quality = get_quality(loc);
                    max_after_quality =
                        std::max(max_after_quality, m_tet_attribute[t_id].m_quality);
                }
                // if (max_after_quality > max_quality) {
                //     log_and_throw_error("Input smoothing reduced tet quality");
                // }

                m_vertex_attribute[vid].m_pos = to_rational(m_vertex_attribute[vid].m_posf);
            }
        }
        //
    }
    //
}

} // namespace wmtk::components::image_simulation