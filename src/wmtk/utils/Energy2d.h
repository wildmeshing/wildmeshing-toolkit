#pragma once
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <Eigen/LU>
#include <iostream>
#include "Logger.hpp"
#include "autodiff.h"

namespace wmtk {
struct State
{
    int idx = 0;
    double scaling = 1.;
    double value;
    Eigen::Vector2d gradient;
    Eigen::Matrix2d hessian;
    std::array<double, 6> target_triangle = {0., 0., 1., 0., 1. / 2., sqrt(3) / 2.};
    std::array<double, 6> input_triangle;
};

class Energy
{
public:
    typedef DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> DScalar;
    using Scalar = typename DScalar::Scalar;

public:
    Energy() = default;

    virtual ~Energy() = default;

    virtual void eval(State& state) const = 0;
};
class AMIPS : public wmtk::Energy
{
public:
    void eval(State& state) const override;
};
class SymDi : public wmtk::Energy
{
public:
    void eval(State& state) const override;
};
} // namespace wmtk