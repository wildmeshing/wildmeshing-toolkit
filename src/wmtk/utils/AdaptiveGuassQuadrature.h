#pragma once
#include <Eigen/Dense>
#include <type_traits>

inline double gauss_quadrature(
    std::function<Eigen::Vector3d(const double&, const double&)> f,
    Eigen::Vector2d a,
    Eigen::Vector2d b,
    int degree)
{
    if (degree == 2) {
        double x1 = -sqrt(3) / 3, x2 = sqrt(3) / 3;
        double w1 = 1, w2 = 1;
        Eigen::Vector3d fxa = f(a(0), a(1));
        Eigen::Vector2d x;
        x(0) = (a(0) + b(0)) / 2 + (b(0) - a(0)) * x1 / 2;
        x(1) = (a(1) + b(1)) / 2 + (b(1) - a(1)) * x1 / 2;
        Eigen::Vector3d fx1 = f(x(0), x(1));

        x(0) = (a(0) + b(0)) / 2 + (b(0) - a(0)) * x2 / 2;
        x(1) = (a(1) + b(1)) / 2 + (b(1) - a(1)) * x2 / 2;
        Eigen::Vector3d fx2 = f(x(0), x(1));

        return ((b - a).norm() / 2) * (w1 * (fx1 - fxa).norm() + w2 * (fx2 - fxa).norm());
    } else if (degree == 3) {
        double x1 = -sqrt(3 / 5), x2 = 0, x3 = sqrt(3 / 5);
        double w1 = 5 / 9, w2 = 8 / 9, w3 = 5 / 9;
        Eigen::Vector3d fxa = f(a(0), a(1));
        Eigen::Vector2d x;
        x(0) = (a(0) + b(0)) / 2 + (b(0) - a(0)) * x1 / 2;
        x(1) = (a(1) + b(1)) / 2 + (b(1) - a(1)) * x1 / 2;
        Eigen::Vector3d fx1 = f(x(0), x(1));
        x(0) = (a(0) + b(0)) / 2 + (b(0) - a(0)) * x2 / 2;
        x(1) = (a(1) + b(1)) / 2 + (b(1) - a(1)) * x2 / 2;
        Eigen::Vector3d fx2 = f(x(0), x(1));
        x(0) = (a(0) + b(0)) / 2 + (b(0) - a(0)) * x3 / 2;
        x(1) = (a(1) + b(1)) / 2 + (b(1) - a(1)) * x3 / 2;
        Eigen::Vector3d fx3 = f(x(0), x(1));

        return ((b - a).norm() / 2) *
               (w1 * (fx1 - fxa).norm() + w2 * (fx2 - fxa).norm() + w3 * (fx3 - fxa).norm());
    }
}

inline double adaptive_gauss_quadrature(
    std::function<Eigen::Vector3d(const double&, const double&)> f,
    Eigen::Vector2d a,
    Eigen::Vector2d b,
    double tol)
{
    auto mid = (a + b) / 2.;
    auto I_2 = gauss_quadrature(f, a, b, 2);
    auto I_3 = gauss_quadrature(f, a, b, 3);
    auto I_left = gauss_quadrature(f, a, mid, 3);
    auto I_right = gauss_quadrature(f, mid, b, 3);
    auto I = I_left + I_right;
    if (fabs(I - I_2) < I * tol) {
        return I;
    } else {
        return adaptive_gauss_quadrature(f, a, mid, tol / 2.) +
               adaptive_gauss_quadrature(f, mid, b, tol / 2.);
    }
}