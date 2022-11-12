#pragma once
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <Eigen/LU>
#include <iostream>
#include "Logger.hpp"
#include "autodiff.h"

namespace wmtk {
class Energy
{
public:
    typedef DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> DScalar;
    using Scalar = typename DScalar::Scalar;
    /**
     * @brief unit area equilateral triangle
     *
     */
    std::array<double, 6> target =
        {0, 0, 2 * 1 / sqrt(sqrt(3)), 0, 1 / sqrt(sqrt(3)), sqrt(sqrt(3))};
    double scaling = 1.;

public:
    Energy() = default;

    virtual ~Energy() = default;

    virtual DScalar energy_function(std::array<double, 6>& T1, std::array<double, 6>& T2, int idx);

    // Computes the value of a function.
    double Value(std::array<double, 6>& input, int idx)
    {
        std::array<double, 6> target_scaled;
        for (auto i = 0; i < 6; i++) {
            target_scaled[i] = target[i] * scaling;
        }
        return this->energy_function(target_scaled, input, idx).getValue();
    }

    // Computes the gradient of a function.
    Eigen::Vector2d Gradient(std::array<double, 6>& input, int idx)
    {
        std::array<double, 6> target_scaled;
        for (auto i = 0; i < 6; i++) {
            target_scaled[i] = target[i] * scaling;
        }
        return this->energy_function(target_scaled, input, idx).getGradient();
    }
    // Computes the Hessian of a function.
    Eigen::Matrix2d Hessian(std::array<double, 6>& input, int idx)
    {
        std::array<double, 6> target_scaled;
        for (auto i = 0; i < 6; i++) {
            target_scaled[i] = target[i] * scaling;
        }
        return this->energy_function(target_scaled, input, idx).getHessian();
    }
};
class AMIPS : public wmtk::Energy
{
public:
    DScalar energy_function(std::array<double, 6>& T1, std::array<double, 6>& T2, int i) override;
};
class SymDi : public wmtk::Energy
{
public:
    DScalar energy_function(std::array<double, 6>& T1, std::array<double, 6>& T2, int i) override;
};
} // namespace wmtk