#pragma once

#include <igl/point_simplex_squared_distance.h>
#include <igl/predicates/predicates.h>
#include <Eigen/Core>
#include <optional>
#include "Energy2d.h"

namespace wmtk {
// store information for Newton's emthod with fallbacks
struct NewtonMethodInfo
{
    Eigen::MatrixXd neighbors; /// N x 4 matrix (2 vtx per triangle)
    Eigen::VectorXi facet_ids; /// N x 1 vector of facet ids
    double target_length;
    int curve_id = 0;
};

// template get 3d tri area
template <typename T>
T triangle_3d_area(
    const Eigen::Matrix<T, 3, 1>& a,
    const Eigen::Matrix<T, 3, 1>& b,
    const Eigen::Matrix<T, 3, 1>& c)
{
    const T n0 = (a[1] - b[1]) * (a[2] - c[2]) - (a[1] - c[1]) * (a[2] - b[2]);
    const T n1 = -(a[0] - b[0]) * (a[2] - c[2]) + (a[0] - c[0]) * (a[2] - b[2]);
    const T n2 = (a[0] - b[0]) * (a[1] - c[1]) - (a[0] - c[0]) * (a[1] - b[1]);

    return sqrt(n0 * n0 + n1 * n1 + n2 * n2) * static_cast<T>(0.5);
}

// template get 3d tri area
template <typename T>
T triangle_2d_area(
    const Eigen::Matrix<T, 2, 1>& A,
    const Eigen::Matrix<T, 2, 1>& B,
    const Eigen::Matrix<T, 2, 1>& C)
{
    Eigen::Matrix<T, 2, 1> B_A = B - A;
    Eigen::Matrix<T, 2, 1> C_A = C - A;
    T area = static_cast<T>(0.5) * abs(B_A.x() * C_A.y() - B_A.y() * C_A.x());
    return area;
}

// use triangle is std vector that has the 2d coordinate of ABC ordered correctly to the orientation
// return true if is degenerate (collinear/ flipped)
// flase o.w.
inline bool is_degenerate_2d_oriented_triangle_array(const std::array<float, 6>& triangle)
{
    Eigen::Vector2d A = Eigen::Vector2d(triangle[0], triangle[1]);
    Eigen::Vector2d B = Eigen::Vector2d(triangle[2], triangle[3]);
    Eigen::Vector2d C = Eigen::Vector2d(triangle[4], triangle[5]);
    auto res = igl::predicates::orient2d(A, B, C);
    if (res != igl::predicates::Orientation::POSITIVE)
        return true;
    else {
        if (triangle_2d_area(A, B, C) < 1e-10) return true;
        return false;
    }
}

using DofVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 2, 1>;
/**
 * Newton method or gradient descent.
 *
 * @param stack with flattened 4x3 vertices positions, with the mover vertex always on the front.
 * This is the same convention with AMIPS_energy
 * @return Descend direction as computed from Newton method, (or gradient descent if Newton fails).
 */
Eigen::Vector2d newton_method_from_stack_2d(
    std::vector<std::array<double, 6>>& stack,
    std::function<double(const std::array<double, 6>&)> energy,
    std::function<void(const std::array<double, 6>&, Eigen::Vector2d&)> jacobian,
    std::function<void(const std::array<double, 6>&, Eigen::Matrix2d&)> hessian);

Eigen::Vector2d newton_method_from_stack_2d_per_vert(
    std::vector<std::array<double, 6>>& stack,
    int i,
    std::function<double(const std::array<double, 6>&, int&)> energy,
    std::function<void(const std::array<double, 6>&, Eigen::Vector2d&, int&)> jacobian,
    std::function<void(const std::array<double, 6>&, Eigen::Matrix2d&, int&)> hessian);
std::array<double, 6> smooth_over_one_triangle(
    std::array<double, 6>& triangle,
    std::function<double(const std::array<double, 6>&, int&)> energy,
    std::function<void(const std::array<double, 6>&, Eigen::Vector2d&, int&)> jacobian,
    std::function<void(const std::array<double, 6>&, Eigen::Matrix2d&, int&)> hessian);

Eigen::Vector2d gradient_descent_from_stack_2d(
    std::vector<std::array<double, 6>>& stack,
    std::function<double(const std::array<double, 6>&)> energy,
    std::function<void(const std::array<double, 6>&, Eigen::Vector2d&)> jacobian);

Eigen::Vector2d gradient_descent_from_stack_2d_per_vert(
    std::vector<std::array<double, 6>>& assembles,
    int i,
    std::function<double(const std::array<double, 6>&, int&)> compute_energy,
    std::function<void(const std::array<double, 6>&, Eigen::Vector2d&, int&)> compute_jacobian);
/**
 * Reorders indices in a triangle such that v0 is on the front. Using the triangle symmetry to
 * preserve orientation. Assumes v0 is in the triangle.
 * @param conn
 * @param v0
 * @return std::array<size_t, 3>
 */
std::array<size_t, 3> orient_preserve_tri_reorder(const std::array<size_t, 3>& tri, size_t v0);

Eigen::Vector2d try_project(
    const Eigen::Vector2d& point,
    const std::vector<std::array<double, 4>>& assembled_neighbor);

wmtk::DofVector newton_direction_2d_with_index(
    const wmtk::Energy& energy_def,
    State& state,
    const wmtk::NewtonMethodInfo& nminfo,
    const wmtk::Boundary& boundary_mapping,
    const wmtk::DofVector& dofx);

wmtk::DofVector gradient_descent_direction_2d_with_index(
    const wmtk::Energy& energy_def,
    State& state,
    const wmtk::NewtonMethodInfo& nminfo,
    const wmtk::Boundary& boundary_mapping,
    const wmtk::DofVector& dofx);

void newton_method_with_fallback(
    const wmtk::Energy& energy_def,
    const wmtk::Boundary& boundary_mapping,
    const NewtonMethodInfo& nminfo,
    DofVector& dofx,
    State& state);
} // namespace wmtk
