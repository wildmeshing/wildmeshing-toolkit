#pragma once
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <Eigen/LU>
#include <iostream>
#include "BoundaryParametrization.h"
#include "Displacement.h"
#include "Image.h"
#include "Logger.hpp"
#include "autodiff.h"

namespace wmtk {
using DofVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 2, 1>;
struct State
{
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;
    int idx = 0;
    double scaling = 1.;
    double value;
    Eigen::Vector2d gradient;
    Eigen::Matrix2d hessian;
    wmtk::DofVector dofx;
    Eigen::MatrixXd two_opposite_vertices;
    std::array<double, 6> target_triangle = {0., 0., 1., 0., 1. / 2., sqrt(3) / 2.};
    std::array<double, 6> input_triangle;
};
struct DofsToPositions
{
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;

protected:
    const Boundary& m_boundary_mapping;
    const int& m_curve_id = -1;

public:
    DofsToPositions(const wmtk::Boundary& b, const int& curve_id)
        : m_boundary_mapping(b)
        , m_curve_id(curve_id)
    {}

    std::pair<DScalar, DScalar> eval(const DofVector& dofx) const
    {
        if (dofx.size() == 2) {
            DiffScalarBase::setVariableCount(2);
            DScalar x1(0, dofx(0));
            DScalar y1(1, dofx(1));
            return std::pair<DScalar, DScalar>(x1, y1);
        }
        assert(m_boundary_mapping.m_arclengths.size() != 0);
        auto arclength = m_boundary_mapping.m_arclengths[m_curve_id];
        double t_value = std::fmod(dofx(0), arclength.back());
        while (t_value < 0) t_value += arclength.back();
        assert(t_value <= arclength.back());
        assert(t_value >= 0);
        DScalar t(0, t_value);

        auto it = std::prev(std::upper_bound(arclength.begin(), arclength.end(), t.getValue()));
        assert(*it >= 0);
        auto a = std::distance(arclength.begin(), it);
        assert(a >= 0);
        assert((a + 1) < arclength.size());

        auto r = t - *it;

        const auto& boundary = m_boundary_mapping.m_boundaries[m_curve_id];
        assert(a < boundary.size());
        Eigen::Vector2d A = boundary[a];
        Eigen::Vector2d B = boundary[(a + 1) % boundary.size()];

        auto n = (B - A) / (arclength[a + 1] - arclength[a]);

        assert(std::pow((n.squaredNorm() - 1), 2) < 1e-8);
        Eigen::Matrix<DScalar, 2, 1> tmpA;
        tmpA << r * n(0), r * n(1);

        Eigen::Matrix<DScalar, 2, 1> V;
        V << A(0) + tmpA(0), A(1) + tmpA(1);

        return {V(0), V(1)};
    }
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
    virtual void eval(State& state, DofsToPositions& dofstopositions) const = 0;
};
class AMIPS : public wmtk::Energy
{
public:
    void eval([[maybe_unused]] State& state) const override{};
    void eval(State& state, DofsToPositions& dofstopositions) const override;
};
class SymDi : public wmtk::Energy
{
public:
    void eval([[maybe_unused]] State& state) const override{};
    void eval(State& state, DofsToPositions& dofstopositions) const override;
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
    void eval([[maybe_unused]] State& state, [[maybe_unused]] DofsToPositions& dofstopositions)
        const override{};

    DScalar displacement(const DScalar& x, const DScalar& y) const { return m_displacement(x, y); };
};
class EdgeLengthEnergy : public wmtk::Energy
{
public:
    // m_displacement needs to be defined with 2 arguments of DScalar, and return a DScalar
    EdgeLengthEnergy(std::function<DScalar(const DScalar&, const DScalar&)> displacement_func)
        : m_displacement(std::move(displacement_func))
    {}

public:
    std::function<DScalar(const DScalar&, const DScalar&)> m_displacement;

public:
    void eval([[maybe_unused]] State& state) const override{};
    void eval(State& state, DofsToPositions& x) const override;
    // a wrapper function of m_displacement that takes 2 doubles and cast into DScalar, and
    // returns a Vector3d
    double displacement(const double& x, const double& y) const
    {
        double z = m_displacement(DScalar(x), DScalar(y)).getValue();
        return z;
    };
};
class AccuracyEnergy : public wmtk::Energy
{
public:
    AccuracyEnergy(std::shared_ptr<Displacement> displ)
        : m_displ(displ)
    {}

public:
    std::shared_ptr<Displacement> m_displ; // Initiated using the Displacement class

public:
    void eval([[maybe_unused]] State& state) const override{};
    void eval(State& state, DofsToPositions& x) const override;
};
} // namespace wmtk