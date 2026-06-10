#include "AMIPSEnergy.hpp"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/orient.hpp>

namespace wmtk::optimization {

AMIPSEnergy2D::AMIPSEnergy2D(const std::vector<std::array<double, 6>>& cells, const double weight)
    : m_cells(cells)
    , m_weight(weight)
{}

AMIPSEnergy2D::TVector AMIPSEnergy2D::initial_position() const
{
    Vector2d tmp(m_cells[0][0], m_cells[0][1]);
    return tmp;
}

double AMIPSEnergy2D::value(const TVector& x)
{
    assert(x.size() == 2);

    double res = 0;
    for (size_t i = 0; i < m_cells.size(); ++i) {
        auto c = m_cells[i];
        c[0] = x[0];
        c[1] = x[1];
        res += wmtk::AMIPS2D_energy(c);
    }
    return m_weight * res;
}

void AMIPSEnergy2D::gradient(const TVector& x, TVector& gradv)
{
    assert(x.size() == 2);
    gradv.resize(2);
    gradv.setZero();

    Vector2d tmp;
    for (size_t i = 0; i < m_cells.size(); ++i) {
        auto c = m_cells[i];
        c[0] = x[0];
        c[1] = x[1];
        wmtk::AMIPS2D_jacobian(c, tmp);
        gradv += tmp;
    }

    gradv *= m_weight;
}

void AMIPSEnergy2D::hessian(const TVector& x, MatrixXd& hessian)
{
    assert(x.size() == 2);
    hessian.resize(2, 2);
    hessian.setZero();

    Matrix2d tmp;
    for (size_t i = 0; i < m_cells.size(); ++i) {
        auto c = m_cells[i];
        c[0] = x[0];
        c[1] = x[1];
        wmtk::AMIPS2D_hessian(c, tmp);
        hessian += tmp;
    }

    hessian *= m_weight;
}

void AMIPSEnergy2D::solution_changed(const TVector& new_x) {}

bool AMIPSEnergy2D::is_step_valid(const TVector& x0, const TVector& x1)
{
    const Vector2d p0 = x1;
    for (const auto& c : m_cells) {
        const Vector2d p1(c[2], c[3]);
        const Vector2d p2(c[4], c[5]);

        if (!utils::orient2d(p0, p1, p2)) {
            return false;
        }
    }
    return true;
}

AMIPSEnergy3D::AMIPSEnergy3D(const std::vector<std::array<double, 12>>& cells, const double weight)
    : m_cells(cells)
    , m_weight(weight)
{}

AMIPSEnergy3D::TVector AMIPSEnergy3D::initial_position() const
{
    Vector3d tmp(m_cells[0][0], m_cells[0][1], m_cells[0][2]);
    return tmp;
}

double AMIPSEnergy3D::value(const TVector& x)
{
    assert(x.size() == 3);

    double res = 0;
    for (size_t i = 0; i < m_cells.size(); ++i) {
        auto c = m_cells[i];
        c[0] = x[0];
        c[1] = x[1];
        c[2] = x[2];
        res += wmtk::AMIPS_energy(c);
    }
    return m_weight * res;
}

void AMIPSEnergy3D::gradient(const TVector& x, TVector& gradv)
{
    assert(x.size() == 3);
    gradv.resize(3);
    gradv.setZero();

    Vector3d tmp;
    for (size_t i = 0; i < m_cells.size(); ++i) {
        auto c = m_cells[i];
        c[0] = x[0];
        c[1] = x[1];
        c[2] = x[2];
        wmtk::AMIPS_jacobian(c, tmp);
        gradv += tmp;
    }

    gradv *= m_weight;
}

void AMIPSEnergy3D::hessian(const TVector& x, MatrixXd& hessian)
{
    assert(x.size() == 3);
    hessian.resize(3, 3);
    hessian.setZero();

    Matrix3d tmp;
    for (size_t i = 0; i < m_cells.size(); ++i) {
        auto c = m_cells[i];
        c[0] = x[0];
        c[1] = x[1];
        c[2] = x[2];
        wmtk::AMIPS_hessian(c, tmp);
        hessian += tmp;
    }

    hessian *= m_weight;
}

void AMIPSEnergy3D::solution_changed(const TVector& new_x) {}

bool AMIPSEnergy3D::is_step_valid(const TVector& x0, const TVector& x1)
{
    const Vector3d p0 = x1;
    for (const auto& c : m_cells) {
        const Vector3d p1(c[3], c[4], c[5]);
        const Vector3d p2(c[6], c[7], c[8]);
        const Vector3d p3(c[9], c[10], c[11]);

        if (!utils::orient3d(p0, p1, p2, p3)) {
            return false;
        }
    }
    return true;
}

} // namespace wmtk::optimization