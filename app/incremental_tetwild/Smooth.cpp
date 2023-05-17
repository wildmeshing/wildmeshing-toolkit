
#include "IncrementalTetWild.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <wmtk/utils/AMIPS.h>
#include <array>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>


#include <limits>
#include <optional>
bool tetwild::TetWild::smooth_before(const Tuple& t)
{
    if (!m_vertex_attribute[t.vid(*this)].on_bbox_faces.empty()) return false;

    // for geometry preservation
    if (m_params.preserve_geometry) {
        if (m_vertex_attribute[t.vid(*this)].m_is_on_surface)
            if (m_vertex_attribute[t.vid(*this)].is_freezed ||
                m_vertex_attribute[t.vid(*this)].in_edge_param.size() > 1)
                return false;
    }

    if (m_vertex_attribute[t.vid(*this)].m_is_rounded) return true;
    // try to round.
    // Note: no need to roll back.
    return round(t);
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
    /*
    uv = ...
    auto param = [&o, &x, &y](const Vector2d &uv)
    {
        return o + uv[0]*x + uv[1]*y;
    }

    //verify me with FD
    auto jac = [&x, &y](const std::array<double, 12>& T, Eigen::Vector2d& result)
    {
        Matrix<double, 3, 2> jac_param;
        jac_param.col(0) = x;
        jac_param.col(1) = y;

        Eigen::Vector3d jac_amips;
        wmtk::AMIPS_jacobian(T, jac_amips);

        // not 100 sure
        result = jac_param * jac_amips;
    }
    //FD gradient
    du = uv; du[0] += eps;
    duT = T;
    for (auto& t : duT)
        for (auto j = 0; j < 3; j++) {
            t[j] = param(du)[j]; // only filling the front point x,y,z.
        }
    double dufd = (wmtk::AMIPS_energy(duT) - wmtk::AMIPS_energy(T))/eps;

    dv = uv; dv[1] += eps;
    dvT = T;
    for (auto& t : dvT)
        for (auto j = 0; j < 3; j++) {
            t[j] = param(dv)[j]; // only filling the front point x,y,z.
        }
    double dvfd = (wmtk::AMIPS_energy(dvT) - wmtk::AMIPS_energy(T))/eps;

    Vector2d J;
    jac(T, J)
    J[0] == dufd; J[1] == dvfd;
    //end of FD


    //verify me with FD
    auto hessian = [&x, &y](const std::array<double, 12>& T, Eigen::Matrix2d& result)
    {
        Matrix<double, 3, 2> jac_param;
        jac_param.col(0) = x;
        jac_param.col(1) = y;

        Eigen::Vector3d hessian_amips;
        wmtk::AMIPS_hessian(T, hessian_amips);

        // not 100 sure
        result = jac_param * hessian_amips * jac_param.transpose();
    }
    //FD hessian
    du = uv; du[0] += eps;
    duT = T;
    for (auto& t : duT)
        for (auto j = 0; j < 3; j++) {
            t[j] = param(du)[j]; // only filling the front point x,y,z.
        }
        Vector2d jacdu, jacn;
        jac(duT, jacdu);
        jac(T, jacn);
    Vector2d dufd = (jacdu- jacn)/eps;

    dv = uv; dv[1] += eps;
    dvT = T;
    for (auto& t : dvT)
        for (auto j = 0; j < 3; j++) {
            t[j] = param(dv)[j]; // only filling the front point x,y,z.
        }
    Vector2d jacdv, jacn;
        jac(dvT, jacdv);
        jac(T, jacn);
    Vector2d dvfd = (jacdv- jacn)/eps;

    Matrix2d J;
    hessian(T, J)
    J.col(0) == dufd; J.col(1) == dvfd;
    //end of FD




    wmtk::newton_method_from_stack(
        uv,
        assembles,
        param,
        wmtk::AMIPS_energy,
        jac,
        hessian);
    */

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

        // auto project = wmtk::try_project(m_vertex_attribute[vid].m_posf, neighbor_assemble);
        auto project = Eigen::Vector3d();

        if (m_params.preserve_geometry) {
            if (m_vertex_attribute[vid].in_edge_param.size() > 0) {
                // project to param edge
                int edge_param_id = m_vertex_attribute[vid].in_edge_param[0];
                const auto& edge_param = edge_params[edge_param_id];

                double t =
                    (m_vertex_attribute[vid].m_posf - edge_param.origin).dot(edge_param.direction);
                project = edge_param.origin + t * edge_param.direction;
            } else {
                // project to the parametrization plane
                // std::cout << "boundary vertex" << std::endl;
                // std::cout << "face_nearly_param_type " << vid << ": "
                //           << m_vertex_attribute[vid].face_nearly_param_type.size() << std::endl;
                int collection_nearly_id = m_vertex_attribute[vid].face_nearly_param_type[0];
                // std::cout << "ollection_nearly_id " << vid << ": " << collection_nearly_id <<
                // std::endl;

                assert(
                    collection_nearly_id <
                    triangle_collections_from_input_surface.nearly_coplanar_collections.size());
                const auto& collection = triangle_collections_from_input_surface
                                             .nearly_coplanar_collections[collection_nearly_id];
                double dist =
                    (m_vertex_attribute[vid].m_posf - collection.a_pos_f).dot(collection.normal_f);
                project = m_vertex_attribute[vid].m_posf - dist * collection.normal_f;
            }


        } else {
            if (boundaries_tree.initialized()) {
                // project to envelope
                // std::cout << "in smoothing open boundary" << std::endl;
                boundaries_tree.nearest_point(m_vertex_attribute[vid].m_posf, project);

            } else
                // project to neighborhood
                project = wmtk::try_project(m_vertex_attribute[vid].m_posf, neighbor_assemble);
        }


        m_vertex_attribute[vid].m_posf = project;
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
        std::vector<std::array<double, 9>> neighbor_assemble;
        std::set<size_t> unique_fid;
        for (auto& t : locs) {
            for (auto j = 0; j < 4; j++) {
                auto f_t = tuple_from_face(t.tid(*this), j);
                auto fid = f_t.fid(*this);
                auto [it, suc] = unique_fid.emplace(fid);
                if (!suc) continue;
                if (m_face_attribute[fid].m_is_surface_fs) {
                    auto vs = get_face_vertices(f_t);
                    auto vs_id = std::array<size_t, 3>();
                    for (auto k = 0; k < 3; k++) vs_id[k] = vs[k].vid(*this);
                    for (auto k : {1, 2})
                        if (vs_id[k] == vid) {
                            std::swap(vs_id[k], vs_id[0]);
                        };
                    if (vs_id[0] != vid) continue; // does not contain point of interest
                    std::array<double, 9> coords;
                    for (auto k = 0; k < 3; k++)
                        for (auto kk = 0; kk < 3; kk++)
                            coords[k * 3 + kk] = m_vertex_attribute[vs_id[k]].m_posf[kk];
                    neighbor_assemble.emplace_back(coords);
                }
            }
        }
        // auto project = wmtk::try_project(m_vertex_attribute[vid].m_posf, neighbor_assemble);
        auto project = Eigen::Vector3d();
        if (m_params.preserve_geometry) {
            if (m_vertex_attribute[vid].in_edge_param.size() > 0) {
                // project to param edge
                int edge_param_id = m_vertex_attribute[vid].in_edge_param[0];
                const auto& edge_param = edge_params[edge_param_id];

                double t =
                    (m_vertex_attribute[vid].m_posf - edge_param.origin).dot(edge_param.direction);
                project = edge_param.origin + t * edge_param.direction;
            } else {
                // project to the parametrization plane
                // std::cout << "surface vertex" << std::endl;
                // std::cout << "face_nearly_param_type " << vid << ": "
                //           << m_vertex_attribute[vid].face_nearly_param_type.size() <<
                //           std::endl;
                int collection_nearly_id = m_vertex_attribute[vid].face_nearly_param_type[0];
                // std::cout << "ollection_nearly_id " << vid << ": " << collection_nearly_id <<
                // std::endl;

                // std::cout << collection_nearly_id << std::endl;
                assert(
                    collection_nearly_id <
                    triangle_collections_from_input_surface.nearly_coplanar_collections.size());
                const auto& collection = triangle_collections_from_input_surface
                                             .nearly_coplanar_collections[collection_nearly_id];
                double dist =
                    (m_vertex_attribute[vid].m_posf - collection.a_pos_f).dot(collection.normal_f);
                project = m_vertex_attribute[vid].m_posf - dist * collection.normal_f;
            }

        } else {
            if (triangles_tree.initialized())
                triangles_tree.nearest_point(m_vertex_attribute[vid].m_posf, project);
            else
                project = wmtk::try_project(m_vertex_attribute[vid].m_posf, neighbor_assemble);
        }

        m_vertex_attribute[vid].m_posf = project;
        for (auto& n : neighbor_assemble) {
            for (auto kk = 0; kk < 3; kk++) n[kk] = m_vertex_attribute[vid].m_posf[kk];
        }
        for (auto& n : neighbor_assemble) {
            auto tris = std::array<Eigen::Vector3d, 3>();
            for (auto k = 0; k < 3; k++) {
                for (auto kk = 0; kk < 3; kk++) tris[k][kk] = n[k * 3 + kk];
            }
            bool is_out = m_envelope.is_outside(tris);
            if (is_out) return false;
        }
    }

    // quality
    auto max_after_quality = 0.;
    for (auto& loc : locs) {
        if (is_inverted(loc)) return false;
        auto t_id = loc.tid(*this);
        m_tet_attribute[t_id].m_quality = get_quality(loc);
        max_after_quality = std::max(max_after_quality, m_tet_attribute[t_id].m_quality);
    }
    if (max_after_quality > max_quality) return false;


    m_vertex_attribute[vid].m_pos = tetwild::to_rational(m_vertex_attribute[vid].m_posf);


    return true;
}

void tetwild::TetWild::smooth_all_vertices()
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
