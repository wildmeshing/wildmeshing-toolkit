#include "TetraQualityUtils.hpp"

#include "Logger.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <igl/point_simplex_squared_distance.h>

#include <array>

std::array<size_t, 4> wmtk::orient_preserve_tet_reorder(
    const std::array<size_t, 4>& conn,
    size_t v0)
{
    auto id_in_array = [](const auto& arr, auto a) {
        for (auto i = 0; i < arr.size(); i++)
            if (arr[i] == a) return i;
        return -1;
    };
    auto vl_id = id_in_array(conn, v0);
    assert(vl_id != -1);
    auto reorder = std::array<std::array<size_t, 4>, 4>{
        {{{0, 1, 2, 3}}, {{1, 0, 3, 2}}, {{2, 0, 1, 3}}, {{3, 1, 0, 2}}}};
    auto newconn = conn;
    for (auto j = 0; j < 4; j++) newconn[j] = conn[reorder[vl_id][j]];
    return newconn;
}

auto newton_direction_1d = [](auto& compute_energy,
                              auto& compute_jacobian,
                              auto& compute_hessian,
                              auto& assembles,
                              const Eigen::Vector3d& pos) -> double {
    auto total_energy = 0.;
    double total_jac = 0;
    double total_hess = 0;


    // E = \sum_i E_i(x)
    // J = \sum_i J_i(x)
    // H = \sum_i H_i(x)
    auto local_id = 0;
    for (auto& T : assembles) {
        for (auto j = 0; j < 3; j++) {
            T[j] = pos[j]; // only filling the front point.
        }

        total_energy += compute_energy(T);
        total_jac += compute_jacobian(T);
        total_hess += compute_hessian(T);
        assert(!std::isnan(total_energy));
    }
    wmtk::logger().trace("energy {}", total_energy);
    if (std::abs(total_hess) > 1e-10) // a hacky PSD trick. TODO: change this.
        return -total_jac / total_hess;
    else {
        wmtk::logger().trace("gradient descent instead.");
        return -total_jac;
    }
};

// TODO: These three functions should not be in global namespace
template <int dim>
auto newton_direction = [](auto& compute_energy,
                           auto& compute_jacobian,
                           auto& compute_hessian,
                           auto& assembles,
                           const Eigen::Vector3d& pos) -> Eigen::Matrix<double, dim, 1> {
    auto total_energy = 0.;
    Eigen::Matrix<double, dim, 1> total_jac = Eigen::Matrix<double, dim, 1>::Zero();
    Eigen::Matrix<double, dim, dim> total_hess = Eigen::Matrix<double, dim, dim>::Zero();


    // E = \sum_i E_i(x)
    // J = \sum_i J_i(x)
    // H = \sum_i H_i(x)
    auto local_id = 0;
    for (auto& T : assembles) {
        for (int j = 0; j < 3; j++) {
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
    Eigen::Matrix<double, dim, 1> x = total_hess.ldlt().solve(total_jac);
    wmtk::logger().trace("energy {}", total_energy);
    if (total_jac.isApprox(total_hess * x, 1e-5)) // a hacky PSD trick. TODO: change this.
        return -x;
    else {
        wmtk::logger().trace("gradient descent instead.");
        return -total_jac;
    }
};


auto gradient_direction = [](auto& compute_energy,
                             auto& compute_jacobian,
                             auto& assembles,
                             const Eigen::Vector3d& pos) -> Eigen::Vector3d {
    Eigen::Vector3d total_jac = Eigen::Vector3d::Zero();

    for (auto& T : assembles) {
        for (auto j = 0; j < 3; j++) {
            T[j] = pos[j]; // only filling the front point.
        }
        auto jac = decltype(total_jac)();
        compute_jacobian(T, jac);
        total_jac += jac;
    }
    return total_jac;
};

auto linesearch_1d =
    [](auto&& energy_from_point, const double pos, const double dir, const int& max_iter) {
        auto lr = 0.5;
        auto old_energy = energy_from_point(pos);
        wmtk::logger().trace("old energy {} dir {}", old_energy, dir);
        for (auto iter = 1; iter <= max_iter; iter++) {
            const double newpos = pos + std::pow(lr, iter) * dir;
            wmtk::logger().trace("pos {}, dir {}, [{}]", pos, dir, std::pow(lr, iter));
            auto new_energy = energy_from_point(newpos);
            wmtk::logger().trace("iter {}, E= {}, [{}]", iter, new_energy, newpos);
            if (new_energy < old_energy) return newpos; // TODO: armijo conditions.
        }
        return pos;
    };

template <int dim>
auto linesearch = [](auto&& energy_from_point,
                     const Eigen::Matrix<double, dim, 1>& pos,
                     const Eigen::Matrix<double, dim, 1>& dir,
                     const int& max_iter) {
    const double lr = 0.5;
    const double old_energy = energy_from_point(pos);
    double alpha = 1;
    wmtk::logger().trace("old energy {} dir {}", old_energy, dir.transpose());
    for (int iter = 1; iter <= max_iter; iter++) {
        // const Eigen::Matrix<double, dim, 1> newpos = pos + std::pow(lr, iter) * dir;
        const Eigen::Matrix<double, dim, 1> newpos = pos + alpha * dir;
        // wmtk::logger()
        //     .trace("pos {}, dir {}, [{}]", pos.transpose(), dir.transpose(), std::pow(lr, iter));
        const double new_energy = energy_from_point(newpos);
        // wmtk::logger().trace("iter {}, E= {}, [{}]", iter, new_energy, newpos.transpose());
        if (new_energy < old_energy) {
            return newpos; // TODO: armijo conditions.
        }
        alpha *= lr;
    }
    return pos;
};

double wmtk::newton_method_from_stack(
    const double t,
    std::vector<std::array<double, 12>>& assembles,
    std::function<Eigen::Vector3d(const double t)> param,
    std::function<double(const std::array<double, 12>&)> compute_energy,
    std::function<double(const std::array<double, 12>&)> compute_jacobian,
    std::function<double(const std::array<double, 12>&)> compute_hessian)
{
    assert(!assembles.empty());
    double old_pos = t;

    auto energy_from_uv = [&assembles, &param, &compute_energy](const double t) -> double {
        auto total_energy = 0.;
        const Eigen::Vector3d pos = param(t);

        for (auto& T : assembles) {
            for (auto j = 0; j < 3; j++) {
                T[j] = pos[j]; // only filling the front point x,y,z.
            }
            total_energy += compute_energy(T);
        }
        return total_energy;
    };

    auto compute_new_valid_pos =
        [&energy_from_uv, &assembles, &param, &compute_energy, &compute_jacobian, &compute_hessian](
            const double t) {
            double current_pos = t;
            auto line_search_iters = 12;
            auto newton_iters = 10;
            for (auto iter = 0; iter < newton_iters; iter++) {
                auto dir = newton_direction_1d(
                    compute_energy,
                    compute_jacobian,
                    compute_hessian,
                    assembles,
                    param(current_pos));
                auto newpos = linesearch_1d(energy_from_uv, current_pos, dir, line_search_iters);
                if (std::abs(newpos - current_pos) < 1e-9) // barely moves
                {
                    break;
                }
                current_pos = newpos;
            }
            return current_pos;
        };
    return compute_new_valid_pos(old_pos);
}

Eigen::Vector2d wmtk::newton_method_from_stack(
    const Eigen::Vector2d& uv,
    std::vector<std::array<double, 12>>& assembles,
    std::function<Eigen::Vector3d(const Eigen::Vector2d& uv)> param,
    std::function<double(const std::array<double, 12>&)> compute_energy,
    std::function<void(const std::array<double, 12>&, Eigen::Vector2d&)> compute_jacobian,
    std::function<void(const std::array<double, 12>&, Eigen::Matrix2d&)> compute_hessian)
{
    assert(!assembles.empty());
    Eigen::Vector2d old_pos = uv;

    auto energy_from_uv =
        [&assembles, &param, &compute_energy](const Eigen::Vector2d& uvpos) -> double {
        auto total_energy = 0.;
        const Eigen::Vector3d pos = param(uvpos);

        for (auto& T : assembles) {
            for (auto j = 0; j < 3; j++) {
                T[j] = pos[j]; // only filling the front point x,y,z.
            }
            total_energy += compute_energy(T);
        }
        return total_energy;
    };

    auto compute_new_valid_pos =
        [&energy_from_uv, &assembles, &param, &compute_energy, &compute_jacobian, &compute_hessian](
            const Eigen::Vector2d& uvpos) {
            auto current_pos = uvpos;
            auto line_search_iters = 12;
            auto newton_iters = 10;
            for (auto iter = 0; iter < newton_iters; iter++) {
                auto dir = newton_direction<2>(
                    compute_energy,
                    compute_jacobian,
                    compute_hessian,
                    assembles,
                    param(current_pos));
                auto newpos = linesearch<2>(energy_from_uv, current_pos, dir, line_search_iters);
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

Eigen::Vector3d wmtk::newton_method_from_stack(
    std::vector<std::array<double, 12>>& assembles,
    std::function<double(const std::array<double, 12>&)> compute_energy,
    std::function<void(const std::array<double, 12>&, Eigen::Vector3d&)> compute_jacobian,
    std::function<void(const std::array<double, 12>&, Eigen::Matrix3d&)> compute_hessian)
{
    assert(!assembles.empty());
    auto& T0 = assembles.front();
    Eigen::Vector3d old_pos(T0[0], T0[1], T0[2]);

    auto energy_from_point = [&assembles, &compute_energy](const Eigen::Vector3d& pos) -> double {
        double total_energy = 0.;
        for (auto& T : assembles) {
            for (int j = 0; j < 3; j++) {
                T[j] = pos[j]; // only filling the front point x,y,z.
            }
            total_energy += compute_energy(T);
        }
        return total_energy;
    };

    auto compute_new_valid_pos =
        [&energy_from_point, &assembles, &compute_energy, &compute_jacobian, &compute_hessian](
            const Eigen::Vector3d& pos) {
            auto current_pos = pos;
            auto line_search_iters = 12;
            auto newton_iters = 10;
            for (int iter = 0; iter < newton_iters; iter++) {
                const Eigen::Vector3d dir = newton_direction<3>(
                    compute_energy,
                    compute_jacobian,
                    compute_hessian,
                    assembles,
                    current_pos);
                const Eigen::Vector3d newpos =
                    linesearch<3>(energy_from_point, current_pos, dir, line_search_iters);
                //if ((newpos - current_pos).norm() < 1e-9) // barely moves
                //{
                //    break;
                //}
                if (std::abs(energy_from_point(newpos) - energy_from_point(current_pos)) <
                    1e-5) // energy didn't change much
                {
                    break;
                }
                current_pos = newpos;
            }
            return current_pos;
        };
    return compute_new_valid_pos(old_pos);
}

Eigen::Vector3d wmtk::gradient_descent_from_stack(
    std::vector<std::array<double, 12>>& assembles,
    std::function<double(const std::array<double, 12>&)> compute_energy,
    std::function<void(const std::array<double, 12>&, Eigen::Vector3d&)> compute_jacobian)
{
    assert(!assembles.empty());
    auto& T0 = assembles.front();
    Eigen::Vector3d old_pos(T0[0], T0[1], T0[2]);

    auto energy_from_point = [&assembles, &compute_energy](const Eigen::Vector3d& pos) -> double {
        auto total_energy = 0.;
        for (auto& T : assembles) {
            for (auto j = 0; j < 3; j++) {
                T[j] = pos[j]; // only filling the front point x,y,z.
            }
            total_energy += compute_energy(T);
        }
        return total_energy;
    };

    auto compute_new_valid_pos = [&energy_from_point,
                                  &assembles,
                                  &compute_energy,
                                  &compute_jacobian](const Eigen::Vector3d& pos) {
        auto current_pos = pos;
        auto line_search_iters = 12;
        auto newton_iters = 10;
        for (auto iter = 0; iter < newton_iters; iter++) {
            Eigen::Vector3d dir =
                -gradient_direction(compute_energy, compute_jacobian, assembles, current_pos);
            dir.normalize(); // HACK: TODO: should use flip_avoid_line_search.
            auto newpos = linesearch<3>(energy_from_point, current_pos, dir, line_search_iters);
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

Eigen::Vector3d wmtk::try_project(
    const Eigen::Vector3d& point,
    const std::vector<std::array<double, 9>>& assembled_neighbor)
{
    auto min_dist = std::numeric_limits<double>::infinity();
    Eigen::Vector3d closest_point = Eigen::Vector3d::Zero();
    for (const auto& tri : assembled_neighbor) {
        auto V = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(tri.data());
        Eigen::Vector3d project;
        auto dist2 = -1.;
        igl::point_simplex_squared_distance<3>(
            point,
            V,
            Eigen::RowVector3i(0, 1, 2),
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