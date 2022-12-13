
#include "TriWild.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>
#include <array>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>


#include <limits>
#include <optional>

bool triwild::TriWild::smooth_before(const Tuple& t)
{
    if (m_bnd_freeze && vertex_attrs[t.vid(*this)].fixed) return false;
    return true;
}

bool triwild::TriWild::smooth_after(const Tuple& t)
{
    // Newton iterations are encapsulated here.
    auto vid = t.vid(*this);
    auto locs = get_one_ring_tris_for_vertex(t);
    assert(locs.size() > 0);

    // infomation needed for newton's method
    wmtk::NewtonMethodInfo nminfo;
    nminfo.curve_id = vertex_attrs[t.vid(*this)].curve_id;
    nminfo.target_length = this->m_target_l;
    nminfo.neighbors.resize(locs.size(), 4);

    auto is_inverted_coordinates = [this, &vid](auto& A, auto& B) {
        auto res = igl::predicates::orient2d(A, B, this->vertex_attrs[vid].pos);
        if (res != igl::predicates::Orientation::POSITIVE)
            return true;
        else
            return false;
    };

    for (auto i = 0; i < locs.size(); i++) {
        auto tri = locs[i];
        assert(!is_inverted(tri));
        auto local_tuples = oriented_tri_vertices(tri);
        for (auto j = 0; j < 3; j++) {
            if (local_tuples[j].vid(*this) == vid) {
                auto v2 = vertex_attrs[local_tuples[(j + 1) % 3].vid(*this)].pos;
                auto v3 = vertex_attrs[local_tuples[(j + 2) % 3].vid(*this)].pos;
                nminfo.neighbors.row(i) << v2(0), v2(1), v3(0), v3(1);
                assert(!is_inverted_coordinates(v2, v3));
                // sanity check, no inversion should be heres
            }
        }
    }
    assert(locs.size() == nminfo.neighbors.rows());

    // use a general root finding method that defaults to newton but if not changeing the position,
    // try gradient descent
    auto old_pos = vertex_attrs[vid].pos;
    auto old_t = vertex_attrs[vid].t;

    wmtk::DofVector dofx;
    if (is_boundary_vertex(t)) {
        dofx.resize(1);
        dofx[0] = vertex_attrs[t.vid(*this)].t; // t
        wmtk::logger().info("////// boundary vertex dofx {} ", dofx);
    } else {
        dofx.resize(2);
        dofx = vertex_attrs[t.vid(*this)].pos; // uv;
        wmtk::logger().info("////// non boundary vertex dofx {}", dofx);
    }

    wmtk::newton_method_with_fallback(*m_energy, m_boundary, nminfo, dofx);

    // check boundary and project
    // this should be outdated since now every boundary vertex will be on boundary (but good to have
    // as an assert)
    // add assert!!!!
    if (is_boundary_vertex(t)) {
        vertex_attrs[vid].t = dofx(0);
        vertex_attrs[vid].pos = m_boundary.t_to_uv(nminfo.curve_id, dofx(0));
        wmtk::logger().info("after smooth position {}", vertex_attrs[vid].pos);
    } else
        vertex_attrs[vid].pos = dofx;

    // check invariants
    if (!invariants(locs)) {
        vertex_attrs[vid].pos = old_pos;
        vertex_attrs[vid].t = old_t;
        return false;
    }
    wmtk::logger().info("!!! success !!!!");

    assert(invariants(locs));
    return true;
}

void triwild::TriWild::smooth_all_vertices()
{
    assert(m_energy != nullptr);
    wmtk::logger().info("=======smooth==========");
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
        auto executor = wmtk::ExecutePass<TriWild, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_vertex_mutex_one_ring(e, task_id);
        };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<TriWild, wmtk::ExecutionPolicy::kSeq>();
        bool nochange = 1;
        int itr = 0;
        do {
            nochange = 1;
            std::vector<Eigen::Vector2d> old_pos(vert_capacity());
            for (auto& v : get_vertices()) {
                old_pos[v.vid(*this)] = vertex_attrs[v.vid(*this)].pos;
            }
            executor(*this, collect_all_ops);
            // write_obj("smooth" + std::to_string(itr) + ".obj");
            std::vector<Tuple> verts = get_vertices();
            for (int i = 0; i < verts.size() && nochange; i++) {
                auto vid = verts[i].vid(*this);
                nochange &= ((old_pos[vid] - vertex_attrs[vid].pos).norm() < 1e-2);
            }
            itr++;
        } while (!nochange && itr < 1);
        wmtk::logger().info(itr);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time serial: {}s", time);
    }
}