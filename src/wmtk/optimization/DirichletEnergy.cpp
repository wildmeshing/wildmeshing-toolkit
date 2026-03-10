#include "DirichletEnergy.hpp"

namespace wmtk::optimization {

DirichletEnergy2D::DirichletEnergy2D(std::vector<std::array<double, 4>>& cells)
    : m_cells(cells)
{}

DirichletEnergy2D::TVector DirichletEnergy2D::initial_position() const
{
    Vector2d tmp(m_cells[0][0], m_cells[0][1]);
    return tmp;
}

double DirichletEnergy2D::value(const TVector& x)
{
    assert(x.size() == 2);

    double res = 0;
    for (const auto& c : m_cells) {
        Vector2d y(c[2], c[3]);
        res += (x - y).squaredNorm();
    }
    res *= 0.5;
    return res;
}

void DirichletEnergy2D::gradient(const TVector& x, TVector& gradv)
{
    assert(x.size() == 2);

    Vector2d tmp = 2 * x;
    for (const auto& c : m_cells) {
        Vector2d y(c[2], c[3]);
        tmp -= y;
    }

    gradv = tmp;
}

void DirichletEnergy2D::hessian(const TVector& x, MatrixXd& hessian)
{
    hessian = 2 * Matrix2d::Identity();
}

void DirichletEnergy2D::solution_changed(const TVector& new_x) {}

} // namespace wmtk::optimization