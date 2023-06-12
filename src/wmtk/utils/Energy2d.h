#pragma once
#include <igl/predicates/predicates.h>
#include <wmtk/image/Displacement.h>
#include <wmtk/image/Image.h>
#include <wmtk/image/Quadric.h>
#include <wmtk/image/TextureIntegral.h>
#include <wmtk/utils/autodiff.h>
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <iostream>
#include <wmtk/utils/Logger.hpp>
#include "BoundaryParametrization.h"
namespace wmtk {
using DofVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 2, 1>;
struct State
{
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;

    double value; // total energy value
    Eigen::Vector2d gradient;
    Eigen::Matrix2d hessian;
    wmtk::DofVector dofx;
    Eigen::MatrixXd two_opposite_vertices;
    int idx = 0; // facet index
    double scaling = 1.; // target edge length

    ////// ==== archived ===== (keep for compilation)

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
        lagrange::enable_fpe();
        if (dofx.size() == 2) {
            DiffScalarBase::setVariableCount(2);
            DScalar x1(0, dofx(0));
            DScalar y1(1, dofx(1));
            return std::pair<DScalar, DScalar>(x1, y1);
        }

        auto s = m_boundary_mapping.t_to_segment(m_curve_id, dofx(0));

        DScalar t(0, dofx(0));
        Eigen::Matrix<DScalar, 2, 1> V;
        V(0) = s.A(0) + (s.B(0) - s.A(0)) * (t - s.t0) / s.tlen;
        V(1) = s.A(1) + (s.B(1) - s.A(1)) * (t - s.t0) / s.tlen;

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
    EdgeLengthEnergy(std::shared_ptr<Displacement> displ)
        : m_displ(displ)
    {}

public:
    std::shared_ptr<Displacement> m_displ; // Initiated using the Displacement class

public:
    void eval([[maybe_unused]] State& state) const override{};
    void eval(State& state, DofsToPositions& x) const override;
};
class EdgeAccuracyEnergy : public wmtk::Energy
{
public:
    EdgeAccuracyEnergy(std::shared_ptr<Displacement> displ)
        : m_displ(displ)
    {}

public:
    std::shared_ptr<Displacement> m_displ; // Initiated using the Displacement class

public:
    void eval([[maybe_unused]] State& state) const override{};
    void eval(State& state, DofsToPositions& x) const override;
};

class AreaAccuracyEnergy : public wmtk::Energy
{
public:
    AreaAccuracyEnergy(std::shared_ptr<Displacement> displ, const TextureIntegral& texture_integral)
        : m_displ(displ)
        , m_texture_integral(texture_integral)
    {}

public:
    std::shared_ptr<Displacement> m_displ; // Initiated using the Displacement class
    const TextureIntegral& m_texture_integral;

public:
    void eval([[maybe_unused]] State& state) const override{};
    void eval(State& state, DofsToPositions& x) const override;
};

class QuadricEnergy : public wmtk::Energy
{
protected:
    std::shared_ptr<Displacement> m_displ;
    std::vector<wmtk::Quadric<double>> m_facet_quadrics;

public:
    QuadricEnergy(std::shared_ptr<Displacement> displ)
        : m_displ(std::move(displ))
    {}

    std::vector<wmtk::Quadric<double>>& facet_quadrics() { return m_facet_quadrics; }

public:
    void eval([[maybe_unused]] State& state) const override{};
    void eval(State& state, DofsToPositions& x) const override;
};

// use finite difference to calculate gradient and hessian
// use fixed tangent plane
class AMIPS3D : public wmtk::Energy
{
public:
    AMIPS3D(std::shared_ptr<Displacement> displ)
        : m_displ(displ)
    {}

public:
    std::shared_ptr<Displacement> m_displ; // Initiated using the Displacement class

public:
    void eval([[maybe_unused]] State& state) const override{};
    void eval(State& state, DofsToPositions& x) const override;
};

} // namespace wmtk
