#include "AMIPSEnergy.hpp"

#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/orient.hpp>

namespace wmtk::optimization {

double triangle_area(const Vector2d& p0, const Vector2d& p1, const Vector2d& p2)
{
    const double a = (p1 - p0).norm();
    const double b = (p2 - p1).norm();
    const double c = (p0 - p2).norm();

    const double s = (a + b + c) * 0.5;
    const double area = std::sqrt(
        std::clamp(s * (s - a) * (s - b) * (s - c), 0.0, std::numeric_limits<double>::max()));

    return area;
}

AMIPSEnergy2D::AMIPSEnergy2D(std::vector<std::array<double, 6>>& cells, bool area_weighted)
    : m_cells(cells)
{
    if (area_weighted) {
        m_weights.reserve(cells.size());
        for (const auto& c : m_cells) {
            const Vector2d p0(c[0], c[1]);
            const Vector2d p1(c[2], c[3]);
            const Vector2d p2(c[4], c[5]);
            const double a = triangle_area(p0, p1, p2);
            m_weights.push_back(a);
        }
    } else {
        m_weights = std::vector<double>(cells.size(), 1.);
    }
}

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
        res += m_weights[i] * wmtk::AMIPS2D_energy(c);
    }
    return res;
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
        gradv += m_weights[i] * tmp;
    }
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
        hessian += m_weights[i] * tmp;
    }
}

void AMIPSEnergy2D::solution_changed(const TVector& new_x) {}

bool AMIPSEnergy2D::is_step_valid(const TVector& x0, const TVector& x1)
{
    const Vector2d p0 = x1;
    for (const auto& c : m_cells) {
        const Vector2d p1(c[2], c[3]);
        const Vector2d p2(c[4], c[5]);

        if (utils::orient2d(p0, p1, p2) <= 0) {
            return false;
        }
    }
    return true;
}

} // namespace wmtk::optimization