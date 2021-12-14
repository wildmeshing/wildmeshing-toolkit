//
// Created by Yixin Hu on 11/3/21.
//

#include "TetWild.h"

#include <MshSaver.h>
#include <Logger.hpp>

#include <wmtk/AMIPS.h>


#include <igl/predicates/predicates.h>
#include <spdlog/fmt/ostr.h>

bool tetwild::TetWild::is_inverted(const Tuple& loc)
{
    std::array<Vector3f, 4> ps;
    auto its = loc.oriented_tet_vertices(*this);
    for (int j = 0; j < 4; j++) {
        ps[j] = m_vertex_attribute[its[j].vid()].m_posf;
    }

    //
    igl::predicates::exactinit();
    auto res = igl::predicates::orient3d(ps[0], ps[1], ps[2], ps[3]);
    Scalar result;
    if (res == igl::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == igl::predicates::Orientation::NEGATIVE)
        result = -1;
    else
        result = 0;

    if (result <= 0) return true;
    return false;
}

double tetwild::TetWild::get_quality(const Tuple& loc)
{
    std::array<Vector3f, 4> ps;
    auto its = loc.oriented_tet_vertices(*this);
    for (int j = 0; j < 4; j++) {
        ps[j] = m_vertex_attribute[its[j].vid()].m_posf;
    }

    std::array<double, 12> T;
    for (int j = 0; j < 3; j++) {
        T[0 * 3 + j] = ps[0][j];
        T[1 * 3 + j] = ps[1][j];
        T[2 * 3 + j] = ps[2][j];
        T[3 * 3 + j] = ps[3][j];
    }

    double energy = wmtk::AMIPS_energy(T);
    if (std::isinf(energy) || std::isnan(energy) || energy < 3 - 1e-3) return MAX_ENERGY;
    return energy;
}


bool tetwild::TetWild::vertex_invariant(const Tuple& t)
{
    int v_id = t.vid();

    // check rounded

    // check surface
    if (m_vertex_attribute[v_id].m_is_on_surface) {
    }

    return true;
}

bool tetwild::TetWild::tetrahedron_invariant(const Tuple& t)
{
    // check inversion

    return true;
}

void tetwild::TetWild::smooth_all_vertices()
{
    auto tuples = get_vertices();
    logger().debug("tuples");
    auto cnt_suc = 0;
    for (auto& t : tuples) { // TODO: threads
        if (smooth_vertex(t)) cnt_suc++;
    }
    logger().debug("Smoothing Success Count {}", cnt_suc);
}

bool tetwild::TetWild::smooth_before(const Tuple& t)
{
    return true;
}

bool tetwild::TetWild::smooth_after(const Tuple& t)
{
    // Newton iterations are encapsulated here.
    // TODO: bbox/surface tags.
    // TODO: envelope check.
    logger().trace("Newton iteration for vertex smoothing.");
    using vec = Vector3f;
    auto vid = t.vid();

    auto locs = t.get_conn_tets(*this);
    assert(locs.size() > 0);
    std::vector<std::array<double, 12>> assembles(locs.size());
    auto loc_id = 0;
    for (auto& loc : locs) {
        auto& T = assembles[loc_id];
        auto t_id = loc.tid();

        auto local_tuples = loc.oriented_tet_vertices(*this);
        std::array<size_t, 4> local_verts;
        for (auto i = 0; i < 4; i++) {
            local_verts[i] = local_tuples[i].vid();
        }
        auto vl_id = [local_verts, vid, t_id]() {
            for (auto i = 0; i < 4; i++) {
                if (local_verts[i] == vid) return i;
            }
            assert(false);
            return -1;
        }();

        // flatten and reorder v_id to be the first.
        //
        switch (vl_id) { // ABCD
        case 1: // BA-DC
            local_verts = {local_verts[1], local_verts[0], local_verts[3], local_verts[2]};
            break;
        case 2: // (CAB)D
            local_verts = {local_verts[2], local_verts[0], local_verts[1], local_verts[3]};
            break;
        case 3: // 3102
            local_verts = {local_verts[3], local_verts[1], local_verts[0], local_verts[2]};
            break;
        case 0:
        default: break;
        }
        for (auto i = 0; i < 4; i++) {
            for (auto j = 0; j < 3; j++) {
                T[i * 3 + j] = m_vertex_attribute[local_verts[i]].m_posf[j];
            }
        }
        loc_id++;
    }

    // Compute New Coordinate.
    auto newton_direction = [&assembles](auto& pos) -> vec {
        auto total_energy = 0.;
        vec total_jac = vec::Zero();
        Matrix3 total_hess = Matrix3::Zero();

        // E = \sum_i E_i(x)
        // J = \sum_i J_i(x)
        // H = \sum_i H_i(x)
        auto local_id = 0;
        for (auto& T : assembles) {
            for (auto j = 0; j < 3; j++) {
                T[j] = pos[j]; // only filling the front point.
            }
            auto jac = decltype(total_jac)();
            auto hess = decltype(total_hess)();
            total_energy += wmtk::AMIPS_energy(T);
            wmtk::AMIPS_jacobian(T, jac);
            wmtk::AMIPS_hessian(T, hess);
            total_jac += jac;
            total_hess += hess;
            assert(!std::isnan(total_energy));
        }
        vec x = total_hess.ldlt().solve(total_jac);
        logger().trace("energy {}", total_energy);
        if (total_jac.isApprox(total_hess * x)) // a hacky PSD trick. TODO: change this.
            return -x;
        else {
            logger().trace("gradient descent instead.");
            return -total_jac;
        }
    };
    auto compute_energy = [&assembles](const vec& pos) -> double {
        auto total_energy = 0.;
        for (auto& T : assembles) {
            for (auto j = 0; j < 3; j++) {
                T[j] = pos[j]; // only filling the front point x,y,z.
            }
            total_energy += wmtk::AMIPS_energy(T);
        }
        return total_energy;
    };
    auto linesearch = [&compute_energy](const vec& pos, const vec& dir, const int& max_iter) {
        auto lr = 0.8;
        auto old_energy = compute_energy(pos);
        logger().trace("dir {}", dir);
        for (auto iter = 1; iter <= max_iter; iter++) {
            vec newpos = pos + std::pow(lr, iter) * dir;
            logger().trace("pos {}, dir {}, [{}]", pos, dir, std::pow(lr, iter));
            auto new_energy = compute_energy(newpos);
            logger().trace("iter {}, {}, [{}]", iter, new_energy, newpos);
            if (new_energy < old_energy) return newpos; // TODO: armijo conditions.
        }
        return pos;
    };
    auto compute_new_valid_pos = [&linesearch, &newton_direction](const vec& old_pos) {
        auto current_pos = old_pos;
        auto line_search_iters = 12;
        auto newton_iters = 10;
        for (auto iter = 0; iter < newton_iters; iter++) {
            auto dir = newton_direction(current_pos);
            auto newpos = linesearch(current_pos, dir, line_search_iters);
            if ((newpos - current_pos).norm() < 1e-9) // barely moves
            {
                break;
            }
            current_pos = newpos;
        }
        return current_pos;
    };

    auto old_pos = m_vertex_attribute[vid].m_posf;
    m_vertex_attribute[vid].m_posf = compute_new_valid_pos(old_pos);
    logger().trace(
        "old pos {} -> new pos {}",
        old_pos.transpose(),
        m_vertex_attribute[vid].m_posf.transpose());
    // note: duplicate code snippets.
    for (auto& loc : locs) {
        if (is_inverted(loc)) {
            m_vertex_attribute[vid].m_posf = old_pos;
            return false;
        }
    }

    for (auto& loc : locs) {
        auto t_id = loc.tid();
        m_tet_attribute[t_id].m_qualities = get_quality(loc);
    }
    return true;
}

void tetwild::TetWild::output_mesh(std::string file)
{
    PyMesh::MshSaver mSaver(file, true);

    Eigen::VectorXd V_flat(3 * m_vertex_attribute.size());
    for (int i = 0; i < m_vertex_attribute.size(); i++) {
        for (int j = 0; j < 3; j++) V_flat(3 * i + j) = m_vertex_attribute[i].m_posf[j];
    }

    Eigen::VectorXi T_flat(4 * n_tets());
    for (int i = 0; i < n_tets(); i++) {
        Tuple loc = tuple_from_tet(i);
        auto vs = oriented_tet_vertices(loc);
        for (int j = 0; j < 4; j++) {
            T_flat(4 * i + j) = vs[j].vid();
        }
    }

    mSaver.save_mesh(V_flat, T_flat, 3, mSaver.TET);
}
