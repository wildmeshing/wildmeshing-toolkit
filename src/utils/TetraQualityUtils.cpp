#include "TetraQualityUtils.hpp"

#include "AMIPS.h"
#include "Logger.hpp"

#include <array>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <spdlog/fmt/ostr.h>

std::array<size_t, 4> wmtk::orient_preserve_tet_reorder(const std::array<size_t, 4>& conn, size_t v0)
{
    auto id_in_array = [](const auto& arr, auto a) {
        for (auto i = 0; i < arr.size(); i++)
            if (arr[i] == a) return i;
        return -1;
    };
    auto vl_id = id_in_array(conn, v0);
    assert(vl_id != -1);
    auto reorder = std::array<std::array<size_t, 4>, 4>{
        {{0, 1, 2, 3}, {1, 0, 3, 2}, {2, 0, 1, 3}, {3, 1, 0, 2}}};
    auto newconn = conn;
    for (auto j = 0; j < 4; j++) newconn[j] = conn[reorder[vl_id][j]];
    return newconn;
};

Eigen::Vector3d wmtk::newton_direction_from_stack(std::vector<std::array<double, 12>>& assembles)
{
    assert(!assembles.empty());
    auto& T = assembles.front();
    Eigen::Vector3d old_pos(T[0], T[1], T[2]);
    auto newton_direction = [&assembles](auto& pos) -> Eigen::Vector3d {
        auto total_energy = 0.;
        Eigen::Vector3d total_jac = Eigen::Vector3d::Zero();
        Eigen::Matrix3d total_hess = Eigen::Matrix3d::Zero();

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
        Eigen::Vector3d x = total_hess.ldlt().solve(total_jac);
        logger().trace("energy {}", total_energy);
        if (total_jac.isApprox(total_hess * x)) // a hacky PSD trick. TODO: change this.
            return -x;
        else {
            logger().trace("gradient descent instead.");
            return -total_jac;
        }
    };
    auto compute_energy = [&assembles](const Eigen::Vector3d& pos) -> double {
        auto total_energy = 0.;
        for (auto& T : assembles) {
            for (auto j = 0; j < 3; j++) {
                T[j] = pos[j]; // only filling the front point x,y,z.
            }
            total_energy += wmtk::AMIPS_energy(T);
        }
        return total_energy;
    };
    auto linesearch = [&compute_energy](const Eigen::Vector3d& pos, const Eigen::Vector3d& dir, const int& max_iter) {
        auto lr = 0.8;
        auto old_energy = compute_energy(pos);
        logger().trace("dir {}", dir);
        for (auto iter = 1; iter <= max_iter; iter++) {
            Eigen::Vector3d newpos = pos + std::pow(lr, iter) * dir;
            logger().trace("pos {}, dir {}, [{}]", pos, dir, std::pow(lr, iter));
            auto new_energy = compute_energy(newpos);
            logger().trace("iter {}, {}, [{}]", iter, new_energy, newpos);
            if (new_energy < old_energy) return newpos; // TODO: armijo conditions.
        }
        return pos;
    };
    auto compute_new_valid_pos = [&linesearch, &newton_direction](const Eigen::Vector3d& old_pos) {
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
    return compute_new_valid_pos(old_pos);
};
