#include "TriQualityUtils.hpp"
#include <igl/predicates/predicates.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <array>
#include "AMIPS2D_autodiff.h"
#include "Logger.hpp"

std::array<size_t, 3> wmtk::orient_preserve_tri_reorder(
    const std::array<size_t, 3>& conn,
    size_t v0)
{
    auto id_in_array = [](const auto& arr, auto a) {
        for (auto i = 0; i < arr.size(); i++)
            if (arr[i] == a) return i;
        return -1;
    };
    auto vl_id = id_in_array(conn, v0);
    assert(vl_id != -1);
    auto reorder = std::array<std::array<size_t, 3>, 3>{{{{0, 1, 2}}, {{1, 2, 0}}, {{2, 0, 1}}}};
    auto newconn = conn;
    for (auto j = 0; j < 3; j++) newconn[j] = conn[reorder[vl_id][j]];
    return newconn;
}

// TODO: These three functions should not be in global namespace
auto newton_direction_2d = [](auto& compute_energy,
                              auto& compute_jacobian,
                              auto& compute_hessian,
                              auto& assembles,
                              const Eigen::Vector2d& pos) -> Eigen::Vector2d {
    auto total_energy = 0.;
    Eigen::Vector2d total_jac = Eigen::Vector2d::Zero();
    Eigen::Matrix2d total_hess = Eigen::Matrix2d::Zero();

    // E = \sum_i E_i(x)
    // J = \sum_i J_i(x)
    // H = \sum_i H_i(x)
    auto local_id = 0;
    for (auto& T : assembles) {
        for (auto j = 0; j < 2; j++) {
            T[j] = pos[j]; // only filling the front point.
        }
        auto jac = decltype(total_jac)();
        auto hess = decltype(total_hess)();
        total_energy += compute_energy(T);
        compute_jacobian(T, jac);
        compute_hessian(T, hess);
        total_jac += jac;
        total_hess += hess;
        assert(!std::isnan(total_energy));
    }
    Eigen::Vector2d x = total_hess.ldlt().solve(total_jac);

    if (total_jac.isApprox(total_hess * x)) // a hacky PSD trick. TODO: change this.
        return -x;
    else {
        wmtk::logger().info("gradient descent instead.");
        return -total_jac;
    }
};

auto gradient_direction_2d = [](auto& compute_energy,
                                auto& compute_jacobian,
                                auto& assembles,
                                const Eigen::Vector2d& pos) -> Eigen::Vector2d {
    Eigen::Vector2d total_jac = Eigen::Vector2d::Zero();

    for (auto& T : assembles) {
        for (auto j = 0; j < 2; j++) {
            T[j] = pos[j]; // only filling the front point.
        }
        auto jac = decltype(total_jac)();
        compute_jacobian(T, jac);
        total_jac += jac;
    }
    return total_jac;
};

auto linesearch_2d = [](auto& is_inverted,
                        auto&& energy_from_point,
                        const wmtk::DofVector& dofx,
                        const wmtk::DofVector& dir,
                        const int& max_iter) {
    wmtk::DofVector search_dir;
    if (dofx.size() == 1) {
        search_dir.resize(1);
        search_dir(0) = dir(0);
    } else
        search_dir = dir;
    auto lr = 0.5;

    auto old_energy = energy_from_point(dofx);
    if (old_energy == std::numeric_limits<double>::infinity()) return dofx;

    auto new_energy = std::numeric_limits<double>::infinity();
    wmtk::DofVector new_dofx = dofx;
    do {
        new_dofx = dofx + lr * search_dir;
        if (!is_inverted(new_dofx)) {
            auto new_energy = energy_from_point(new_dofx);
            if (new_energy < old_energy) return new_dofx;
        }
        lr /= 2.;
    } while (lr > std::numeric_limits<double>::denorm_min());
    return dofx;
};

Eigen::Vector2d wmtk::newton_method_from_stack_2d(
    std::vector<std::array<double, 6>>& assembles,
    std::function<double(const std::array<double, 6>&)> compute_energy,
    std::function<void(const std::array<double, 6>&, Eigen::Vector2d&)> compute_jacobian,
    std::function<void(const std::array<double, 6>&, Eigen::Matrix2d&)> compute_hessian)
{
    assert(!assembles.empty());
    auto& T0 = assembles.front();
    Eigen::Vector2d old_pos(T0[0], T0[1]);

    auto energy_from_point = [&assembles, &compute_energy](const Eigen::Vector2d& pos) -> double {
        auto total_energy = 0.;
        for (auto& T : assembles) {
            for (auto j = 0; j < 2; j++) {
                T[j] = pos[j]; // only filling the front point x,y,z.
            }
            total_energy += compute_energy(T);
        }
        return total_energy;
    };
    auto is_inverted = [&T0](const Eigen::Vector2d& newpos) {
        Eigen::Vector2d a, b, c;
        a << newpos(0), newpos(1);
        b << T0[2], T0[3];
        c << T0[4], T0[5];
        auto res = igl::predicates::orient2d(a, b, c);
        return (res != igl::predicates::Orientation::POSITIVE);
    };

    auto compute_new_valid_pos = [&is_inverted,
                                  &energy_from_point,
                                  &assembles,
                                  &compute_energy,
                                  &compute_jacobian,
                                  &compute_hessian](const Eigen::Vector2d& pos) {
        auto current_pos = pos;
        auto line_search_iters = 12;
        auto newton_iters = 10;

        auto dir = newton_direction_2d(
            compute_energy,
            compute_jacobian,
            compute_hessian,
            assembles,
            current_pos);
        auto newpos =
            linesearch_2d(is_inverted, energy_from_point, current_pos, dir, line_search_iters);

        current_pos = newpos;

        return current_pos;
    };
    return compute_new_valid_pos(old_pos);
}

Eigen::Vector2d wmtk::gradient_descent_from_stack_2d(
    std::vector<std::array<double, 6>>& assembles,
    std::function<double(const std::array<double, 6>&)> compute_energy,
    std::function<void(const std::array<double, 6>&, Eigen::Vector2d&)> compute_jacobian)
{
    assert(!assembles.empty());
    auto& T0 = assembles.front();
    Eigen::Vector2d old_pos(T0[0], T0[1]);

    auto is_inverted = [&T0](const Eigen::Vector2d& newpos) {
        Eigen::Vector2d a, b, c;
        a << newpos(0), newpos(1);
        b << T0[2], T0[3];
        c << T0[4], T0[5];
        auto res = igl::predicates::orient2d(a, b, c);
        return (res != igl::predicates::Orientation::POSITIVE);
    };

    auto energy_from_point = [&assembles, &compute_energy](const Eigen::Vector2d& pos) -> double {
        auto total_energy = 0.;
        for (auto& T : assembles) {
            for (auto j = 0; j < 2; j++) {
                T[j] = pos[j]; // only filling the front point x,y,z.
            }
            total_energy += compute_energy(T);
        }
        return total_energy;
    };

    auto compute_new_valid_pos = [&is_inverted,
                                  &energy_from_point,
                                  &assembles,
                                  &compute_energy,
                                  &compute_jacobian](const Eigen::Vector2d& pos) {
        auto current_pos = pos;
        auto line_search_iters = 12;
        auto newton_iters = 10;
        for (auto iter = 0; iter < newton_iters; iter++) {
            Eigen::Vector2d dir =
                -gradient_direction_2d(compute_energy, compute_jacobian, assembles, current_pos);
            dir.normalize(); // HACK: TODO: should use flip_avoid_line_search.
            auto newpos =
                linesearch_2d(is_inverted, energy_from_point, current_pos, dir, line_search_iters);
            if ((newpos - current_pos).norm() < 1e-9) // barely moves
            {
                break;
            }
            current_pos = newpos;
        }
        return current_pos;
    };
    return compute_new_valid_pos(old_pos);
}

Eigen::Vector2d wmtk::try_project(
    const Eigen::Vector2d& point,
    const std::vector<std::array<double, 4>>& assembled_neighbor)
{
    auto min_dist = std::numeric_limits<double>::infinity();
    Eigen::Vector2d closest_point = Eigen::Vector2d::Zero();
    // just 2 edges touching the point
    for (const auto& edge : assembled_neighbor) {
        auto V = Eigen::Map<const Eigen::Matrix<double, 2, 2, Eigen::RowMajor>>(edge.data());
        Eigen::Vector2d project;
        auto dist2 = -1.;
        igl::point_simplex_squared_distance<2>(
            point,
            V,
            Eigen::RowVector2i(0, 1),
            0,
            dist2,
            project);
        // Note: libigl might not be robust, but this can be rejected with envelope.
        if (dist2 < min_dist) {
            min_dist = dist2;
            closest_point = project;
        }
    }
    return closest_point;
}

///////////////////////////////////////////////////
///////////////////////////////////////////////////

auto newton_direction_2d_per_vert = [](auto& i,
                                       auto& compute_energy,
                                       auto& compute_jacobian,
                                       auto& compute_hessian,
                                       auto& assembles,
                                       const Eigen::Vector2d& pos) -> Eigen::Vector2d {
    auto total_energy = 0.;
    Eigen::Vector2d total_jac = Eigen::Vector2d::Zero();
    Eigen::Matrix2d total_hess = Eigen::Matrix2d::Zero();

    // E = \sum_i E_i(x)
    // J = \sum_i J_i(x)
    // H = \sum_i H_i(x)
    auto local_id = 0;
    for (auto& T : assembles) {
        T[i * 2] = pos[0];
        T[i * 2 + 1] = pos[1];
        auto jac = decltype(total_jac)();
        auto hess = decltype(total_hess)();
        total_energy += compute_energy(T, i);
        compute_jacobian(T, jac, i);
        compute_hessian(T, hess, i);
        total_jac += jac;
        total_hess += hess;
        assert(!std::isnan(total_energy));
    }
    Eigen::Vector2d x = total_hess.ldlt().solve(total_jac);

    if (total_jac.isApprox(total_hess * x, 1e-6)) // a hacky PSD trick. TODO: change this.
        return -x;
    else {
        wmtk::logger().info("gradient descent instead.");
        return -total_jac;
    }
};

auto gradient_direction_2d_per_vert = [](auto& i,
                                         auto& compute_energy,
                                         auto& compute_jacobian,
                                         auto& assembles,
                                         const Eigen::Vector2d& pos) -> Eigen::Vector2d {
    Eigen::Vector2d total_jac = Eigen::Vector2d::Zero();

    for (auto& T : assembles) {
        T[i * 2] = pos[0];
        T[i * 2 + 1] = pos[1];

        auto jac = decltype(total_jac)();
        compute_jacobian(T, jac, i);
        total_jac += jac;
    }
    return -total_jac;
};

Eigen::Vector2d wmtk::gradient_descent_from_stack_2d_per_vert(
    std::vector<std::array<double, 6>>& assembles,
    int i,
    std::function<double(const std::array<double, 6>&, int&)> compute_energy,
    std::function<void(const std::array<double, 6>&, Eigen::Vector2d&, int&)> compute_jacobian)
{
    assert(!assembles.empty());
    auto& T0 = assembles.front();
    Eigen::Vector2d old_pos(T0[i * 2], T0[i * 2 + 1]);

    auto energy_from_point =
        [&assembles, &i, &compute_energy](const Eigen::Vector2d& pos) -> double {
        auto total_energy = 0.;
        for (auto& T : assembles) {
            T[i * 2] = pos[0]; // only filling the current vert x,y.
            T[i * 2 + 1] = pos[1];
            // wmtk::logger().info("T in energy computation {} ", T);
            // wmtk::logger().info("T energy {} ", compute_energy(T));
            total_energy += compute_energy(T, i);
        }
        return total_energy;
    };

    auto is_inverted = [&T0, &i](const Eigen::Vector2d& newpos) {
        Eigen::Vector2d a, b, c;
        a << newpos(0), newpos(1);
        b << T0[((i + 1) % 3) * 2], T0[((i + 1) % 3) * 2 + 1];
        c << T0[((i + 2) % 3) * 2], T0[((i + 2) % 3) * 2 + 1];
        auto res = igl::predicates::orient2d(a, b, c);
        return (res != igl::predicates::Orientation::POSITIVE);
    };

    auto compute_new_valid_pos =
        [&i, &energy_from_point, &is_inverted, &assembles, &compute_energy, &compute_jacobian](
            const Eigen::Vector2d& pos) {
            auto current_pos = pos;
            auto line_search_iters = 20;
            // one newton's iteration over 3 points of the triangle
            auto dir = gradient_direction_2d_per_vert(
                i,
                compute_energy,
                compute_jacobian,
                assembles,
                current_pos);
            auto newpos =
                linesearch_2d(is_inverted, energy_from_point, current_pos, dir, line_search_iters);
            current_pos = newpos;

            return current_pos;
        };
    return compute_new_valid_pos(old_pos);
}

/**
 * @brief this is method written for debugging energy computation purpose
 *
 * @param assembles
 * @param i the vertex index in a triangle, corresponding positions V_i(x,y) in the stack is T[i*2],
 * T[i*2 +1]
 * @param compute_energy
 * @param compute_jacobian
 * @param compute_hessian
 * @return Eigen::Vector2d
 */
Eigen::Vector2d wmtk::newton_method_from_stack_2d_per_vert(
    std::vector<std::array<double, 6>>& assembles,
    int i,
    std::function<double(const std::array<double, 6>&, int&)> compute_energy,
    std::function<void(const std::array<double, 6>&, Eigen::Vector2d&, int&)> compute_jacobian,
    std::function<void(const std::array<double, 6>&, Eigen::Matrix2d&, int&)> compute_hessian)
{
    assert(!assembles.empty());
    auto& T0 = assembles.front();
    Eigen::Vector2d old_pos(T0[i * 2], T0[i * 2 + 1]);

    auto energy_from_point =
        [&assembles, &i, &compute_energy](const Eigen::Vector2d& pos) -> double {
        auto total_energy = 0.;
        for (auto& T : assembles) {
            T[i * 2] = pos[0]; // only filling the current vert x,y.
            T[i * 2 + 1] = pos[1];
            // wmtk::logger().info("T in energy computation {} ", T);
            // wmtk::logger().info("T energy {} ", compute_energy(T));
            total_energy += compute_energy(T, i);
        }
        return total_energy;
    };

    auto is_inverted = [&T0, &i](const Eigen::Vector2d& newpos) {
        Eigen::Vector2d a, b, c;
        a << newpos(0), newpos(1);
        b << T0[((i + 1) % 3) * 2], T0[((i + 1) % 3) * 2 + 1];
        c << T0[((i + 2) % 3) * 2], T0[((i + 2) % 3) * 2 + 1];
        auto res = igl::predicates::orient2d(a, b, c);
        return (res != igl::predicates::Orientation::POSITIVE);
    };

    auto compute_new_valid_pos = [&i,
                                  &energy_from_point,
                                  &is_inverted,
                                  &assembles,
                                  &compute_energy,
                                  &compute_jacobian,
                                  &compute_hessian](const Eigen::Vector2d& pos) {
        auto current_pos = pos;
        auto line_search_iters = 20;
        // one newton's iteration over 3 points of the triangle
        auto dir = newton_direction_2d_per_vert(
            i,
            compute_energy,
            compute_jacobian,
            compute_hessian,
            assembles,
            current_pos);
        auto newpos =
            linesearch_2d(is_inverted, energy_from_point, current_pos, dir, line_search_iters);
        current_pos = newpos;

        return current_pos;
    };
    return compute_new_valid_pos(old_pos);
}

std::array<double, 6> wmtk::smooth_over_one_triangle(
    std::array<double, 6>& triangle,
    std::function<double(const std::array<double, 6>&, int&)> compute_energy,
    std::function<void(const std::array<double, 6>&, Eigen::Vector2d&, int&)> compute_jacobian,
    std::function<void(const std::array<double, 6>&, Eigen::Matrix2d&, int&)> compute_hessian)
{ // run until no changes in the triangle position
    std::array<double, 6> input_triangle;
    input_triangle = triangle;
    auto norm = [](const std::array<double, 6>& a, const std::array<double, 6>& b) {
        double ret;
        for (int i = 0; i < 6; i++) {
            ret += std::pow(a[i] - b[i], 2);
        }
        return std::sqrt(ret);
    };
    int itr = 0;
    std::array<double, 6> old_triangle;
    do {
        old_triangle = triangle;
        std::vector<std::array<double, 6>> assembles;
        assembles.emplace_back(triangle);
        // smooth 3 vertices in 1 iter
        for (int i = 0; i < 3; i++) {
            assembles[0] = triangle;
            auto new_pos = wmtk::newton_method_from_stack_2d_per_vert(
                assembles,
                i,
                compute_energy,
                compute_jacobian,
                compute_hessian);
            triangle[i * 2] = new_pos[0];
            triangle[i * 2 + 1] = new_pos[1];
        }
        itr++;
    } while (norm(old_triangle, triangle) > 1e-5);
    if (norm(input_triangle, triangle) < 1e-5) {
        wmtk::logger().info("XXXXXXXX using gradient descent xxxxxxxx");
        do {
            old_triangle = triangle;
            std::vector<std::array<double, 6>> assembles;
            assembles.emplace_back(triangle);
            // smooth 3 vertices in 1 iter
            for (int i = 0; i < 3; i++) {
                assembles[0] = triangle;
                auto new_pos = wmtk::gradient_descent_from_stack_2d_per_vert(
                    assembles,
                    i,
                    compute_energy,
                    compute_jacobian);
                triangle[i * 2] = new_pos[0];
                triangle[i * 2 + 1] = new_pos[1];
            }
            itr++;
        } while (norm(old_triangle, triangle) > 1e-5);
    }
    return triangle;
}

/**
 * @brief assembles is a vector of array.
 * For each array, the first 6 doubles are the positions (x,y) for 3 vertices of the triangle.
 * the last one is the local vertex index of the smoothing vertex
 * e.g.: std::array<double, 7> a = {x1,y1,x2,y2,x3,y3,idx}
 * where x*, y* are double, and idx should be cast to size_t to be used.
 * @note The specific assembles design is to bypass passing in mesh m and avoid doing navigations/vid-quiries in this function
 */
wmtk::DofVector wmtk::newton_direction_2d_with_index(
    const wmtk::Energy& energy_def,
    State& state,
    const wmtk::NewtonMethodInfo& nminfo,
    const wmtk::Boundary& boundary_mapping,
    const wmtk::DofVector& dofx)
{
    auto total_energy = 0.;
    Eigen::Vector2d total_jac = Eigen::Vector2d::Zero();
    Eigen::Matrix2d total_hess = Eigen::Matrix2d::Zero();

    for (auto i = 0; i < nminfo.neighbors.rows(); i++) {
        // set State
        // pass the state energy
        state.two_opposite_vertices = nminfo.neighbors.row(i);
        state.dofx = dofx;
        state.scaling = nminfo.target_length;
        wmtk::DofsToPositions dofs_to_pos(boundary_mapping, nminfo.curve_id);
        energy_def.eval(state, dofs_to_pos);
        total_energy += state.value;
        total_jac += state.gradient;
        total_hess += state.hessian;
        assert(!std::isnan(total_energy));
    }
    Eigen::Vector2d x = total_hess.ldlt().solve(total_jac);
    if (total_jac.isApprox(total_hess * x, 1e-6)) // a hacky PSD trick. TODO: change this.
        return -x;
    else {
        wmtk::logger().info("gradient descent instead.");
        return -total_jac;
    }
};
wmtk::DofVector wmtk::gradient_descent_direction_2d_with_index(
    const wmtk::Energy& energy_def,
    State& state,
    const wmtk::NewtonMethodInfo& nminfo,
    const wmtk::Boundary& boundary_mapping,
    const wmtk::DofVector& dofx)
{
    wmtk::logger().info("######### using gradient descent #########");

    auto total_energy = 0.;
    Eigen::Vector2d total_jac = Eigen::Vector2d::Zero();

    for (auto i = 0; i < nminfo.neighbors.rows(); i++) {
        // set State
        // pass the state energy
        state.two_opposite_vertices = nminfo.neighbors.row(i);
        state.dofx = dofx;
        state.scaling = nminfo.target_length;
        wmtk::DofsToPositions dofs_to_pos(boundary_mapping, nminfo.curve_id);
        energy_def.eval(state, dofs_to_pos);

        total_energy += state.value;
        total_jac += state.gradient;
        assert(!std::isnan(total_energy));
    }
    return -total_jac;
}

void wmtk::newton_method_with_fallback(
    const wmtk::Energy& energy_def,
    const wmtk::Boundary& boundary_mapping,
    const NewtonMethodInfo& nminfo,
    DofVector& dofx,
    State& state)
{
    DofVector old_dofx = dofx;

    // this is the same for both boundary vertex and interior veretx since energy is handled by
    // energy function automatically
    auto energy_from_point =
        [&energy_def, &nminfo, &boundary_mapping, &state](const DofVector& dofx) -> double {
        auto total_energy = 0.;
        for (auto i = 0; i < nminfo.neighbors.rows(); i++) {
            // set State
            // pass the state energy
            state.two_opposite_vertices = nminfo.neighbors.row(i);
            state.dofx = dofx;
            state.scaling = nminfo.target_length;
            assert(boundary_mapping.m_arclengths.size() > 0);
            assert(boundary_mapping.m_boundaries.size() > 0);
            DofsToPositions dofs_to_pos(boundary_mapping, nminfo.curve_id);
            energy_def.eval(state, dofs_to_pos);
            total_energy += state.value;
        }
        return total_energy;
    };

    // check every triangle in the assembles with the new position whether any trinagle is flipped
    // this needs to accomodate boundary and interior vertex differently
    // if it is boundary vertex, newpos is t but needs to be converted here
    auto is_inverted = [&boundary_mapping, &nminfo](const DofVector& dofx) {
        for (auto i = 0; i < nminfo.neighbors.rows(); i++) {
            Eigen::Vector2d A, B, C;

            if (dofx.size() == 1) {
                A = boundary_mapping.t_to_uv(nminfo.curve_id, dofx(0));
            } else
                A = dofx;
            B << nminfo.neighbors(i, 0), nminfo.neighbors(i, 1);
            C << nminfo.neighbors(i, 2), nminfo.neighbors(i, 3);
            auto res = igl::predicates::orient2d(A, B, C);
            if (res != igl::predicates::Orientation::POSITIVE) return true;
        }
        return false;
    };

    // the position as input is 2d coordinates. need to convert to 1d before pass in
    auto compute_new_valid_pos =
        [&is_inverted, &energy_from_point, &energy_def, &boundary_mapping, &nminfo, &state](
            const DofVector& dofx,
            bool NEWTON) {
            auto current_dofx = dofx;
            auto line_search_iters = 12;
            DofVector dir; // size() == 1 if it's boundary, ow, size() == 2
            if (NEWTON) {
                dir = newton_direction_2d_with_index(
                    energy_def,
                    state,
                    nminfo,
                    boundary_mapping,
                    current_dofx);
            } else {
                dir = gradient_descent_direction_2d_with_index(
                    energy_def,
                    state,
                    nminfo,
                    boundary_mapping,
                    current_dofx);
            }

            auto new_dofx =
                linesearch_2d(is_inverted, energy_from_point, current_dofx, dir, line_search_iters);

            current_dofx = new_dofx;

            return current_dofx;
        };

    auto new_dofx = compute_new_valid_pos(old_dofx, 1);
    // check is the new position is same as the old. If yes switch to gradient descent
    if ((new_dofx - old_dofx).squaredNorm() < std::numeric_limits<double>::denorm_min())
        new_dofx = compute_new_valid_pos(old_dofx, 0);
    dofx = new_dofx;
}
