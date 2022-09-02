#include "TriQualityUtils.hpp"
#include "Logger.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>

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
    auto reorder = std::array<std::array<size_t, 3>, 3>{
        {{{0, 1, 2}}, {{1, 2, 0}}, {{2, 0, 1}}}};
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
    wmtk::logger().trace("energy {}", total_energy);
    if (total_jac.isApprox(total_hess * x)) // a hacky PSD trick. TODO: change this.
        return -x;
    else {
        wmtk::logger().trace("gradient descent instead.");
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

auto linesearch_2d = [](auto&& energy_from_point,
                     const Eigen::Vector2d& pos,
                     const Eigen::Vector2d& dir,
                     const int& max_iter) {
    auto lr = 0.5;
    auto old_energy = energy_from_point(pos);
    wmtk::logger().trace("old energy {} dir {}", old_energy, dir.transpose());
    for (auto iter = 1; iter <= max_iter; iter++) {
        Eigen::Vector2d newpos = pos + std::pow(lr, iter) * dir;
        wmtk::logger().trace("pos {}, dir {}, [{}]", pos.transpose(), dir.transpose(), std::pow(lr, iter));
        auto new_energy = energy_from_point(newpos);
        wmtk::logger().trace("iter {}, E= {}, [{}]", iter, new_energy, newpos.transpose());
        if (new_energy < old_energy) return newpos; // TODO: armijo conditions.
    }
    return pos;
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

    auto compute_new_valid_pos =
        [&energy_from_point, &assembles, &compute_energy, &compute_jacobian, &compute_hessian](
            const Eigen::Vector2d& pos) {
            auto current_pos = pos;
            auto line_search_iters = 12;
            auto newton_iters = 10;
            for (auto iter = 0; iter < newton_iters; iter++) {
                auto dir = newton_direction_2d(
                    compute_energy,
                    compute_jacobian,
                    compute_hessian,
                    assembles,
                    current_pos);
                auto newpos = linesearch_2d(energy_from_point, current_pos, dir, line_search_iters);
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

Eigen::Vector2d wmtk::gradient_descent_from_stack_2d(
    std::vector<std::array<double, 6>>& assembles,
    std::function<double(const std::array<double, 6>&)> compute_energy,
    std::function<void(const std::array<double, 6>&, Eigen::Vector2d&)> compute_jacobian)
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

    auto compute_new_valid_pos = [&energy_from_point,
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
            auto newpos = linesearch_2d(energy_from_point, current_pos, dir, line_search_iters);
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
