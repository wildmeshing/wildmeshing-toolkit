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
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;
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
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;
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
class TwoAndAHalf : public wmtk::Energy
{
public:
    TwoAndAHalf(std::function<DScalar(const DScalar&, const DScalar&)> displacement_func)
        : m_displacement(std::move(displacement_func))
    {}

public:
    std::function<DScalar(const DScalar&, const DScalar&)> m_displacement;

public:
    void eval(State& state) const override;
    DScalar displacement(const DScalar& x, const DScalar& y) const { return m_displacement(x, y); };
};
class EdgeLengthEnergy : public wmtk::Energy
{
public:
    EdgeLengthEnergy(std::function<Eigen::Vector3d(const double&, const double&)> displacement_func)
        : m_displacement(std::move(displacement_func))
    {}

public:
    std::function<Eigen::Vector3d(const double&, const double&)> m_displacement;

public:
    void eval(State& state) const override;
    Eigen::Vector3d displacement(const double& x, const double& y) const { return m_displacement(x, y); };
};
} // namespace wmtk