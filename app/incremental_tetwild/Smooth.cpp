
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

    auto old_pos = m_vertex_attribute[vid].m_posf;
    auto old_asssembles = assembles;

    // bool preserve_geo =
    //     m_params.preserve_geometry && (m_vertex_attribute[vid].face_nearly_param_type.size() >
    //     0);
    bool preserve_geo = m_params.preserve_geometry;

    if (true) {
        if (preserve_geo && m_vertex_attribute[vid].in_edge_param.size() > 0) {
            // use edge param
            Vector3d o, d;
            size_t edge_param_id = m_vertex_attribute[vid].in_edge_param[0];
            const auto& edge_param = edge_params[edge_param_id];
            d = edge_param.direction;
            o = edge_param.origin;
            double t = (old_pos - o).dot(d);
            // param function
            auto param = [&o, &d](double t) { return o + t * d; };
            // jacobian with chainrule
            auto jac = [&d](const std::array<double, 12>& T) {
                Vector3d jac_param;
                jac_param = d;

                Vector3d jac_amips;
                wmtk::AMIPS_jacobian(T, jac_amips);

                double result = jac_amips.dot(jac_param);
                return result;
            };
            // hessian with chainrule
            auto hessian = [&d](const std::array<double, 12>& T) {
                Vector3d jac_param;
                jac_param = d;

                Eigen::Matrix3d hessian_amips;
                wmtk::AMIPS_hessian(T, hessian_amips);

                double result = jac_param.transpose() * hessian_amips * jac_param;
                return result;
            };
            // newton method 1d
            double new_t = wmtk::newton_method_from_stack(
                t,
                assembles,
                param,
                wmtk::AMIPS_energy,
                jac,
                hessian);

            m_vertex_attribute[vid].m_posf = param(t);
        } else if (preserve_geo && m_vertex_attribute[vid].face_nearly_param_type.size() > 0) {
            // use face param

            Vector3d o, x, y;
            size_t collection_id = m_vertex_attribute[vid].face_nearly_param_type[0];
            const auto& collection =
                triangle_collections_from_input_surface.nearly_coplanar_collections[collection_id];
            o = collection.a_pos_f;
            x = collection.param_u_f;
            y = collection.param_v_f;
            Vector2d uv((old_pos - o).dot(x), (old_pos - o).dot(y));
            // param function
            auto param = [&o, &x, &y](const Vector2d& uv) { return o + uv[0] * x + uv[1] * y; };
            // jacobian with chainrule
            auto jac = [&x, &y](const std::array<double, 12>& T, Eigen::Vector2d& result) {
                Eigen::Matrix<double, 3, 2> jac_param;
                jac_param.col(0) = x;
                jac_param.col(1) = y;

                Vector3d jac_amips;
                wmtk::AMIPS_jacobian(T, jac_amips);

                result = jac_amips.transpose() * jac_param;
            };
            // hessian with chainrule
            auto hessian = [&x, &y](const std::array<double, 12>& T, Eigen::Matrix2d& result) {
                Eigen::Matrix<double, 3, 2> jac_param;
                jac_param.col(0) = x;
                jac_param.col(1) = y;

                Eigen::Matrix3d hessian_amips;
                wmtk::AMIPS_hessian(T, hessian_amips);

                result = jac_param.transpose() * hessian_amips * jac_param;
            };
            // newton method 2d
            Vector2d new_uv = wmtk::newton_method_from_stack(
                uv,
                assembles,
                param,
                wmtk::AMIPS_energy,
                jac,
                hessian);

            m_vertex_attribute[vid].m_posf = param(new_uv);
        }
    } else {
        m_vertex_attribute[vid].m_posf = wmtk::newton_method_from_stack(
            assembles,
            wmtk::AMIPS_energy,
            wmtk::AMIPS_jacobian,
            wmtk::AMIPS_hessian);
    }
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
        if (!preserve_geo) {
            // if not preserve geo then project, else no project
            auto project = Eigen::Vector3d();


            if (boundaries_tree.initialized()) {
                // project to envelope
                // std::cout << "in smoothing open boundary" << std::endl;
                boundaries_tree.nearest_point(m_vertex_attribute[vid].m_posf, project);

            } else
                // project to neighborhood
                project = wmtk::try_project(m_vertex_attribute[vid].m_posf, neighbor_assemble);


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
        if (!preserve_geo) {
            auto project = Eigen::Vector3d();

            if (triangles_tree.initialized())
                triangles_tree.nearest_point(m_vertex_attribute[vid].m_posf, project);
            else
                project = wmtk::try_project(m_vertex_attribute[vid].m_posf, neighbor_assemble);


            m_vertex_attribute[vid].m_posf = project;
        }

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
