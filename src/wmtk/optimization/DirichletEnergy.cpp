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


SmoothingEnergy2D::SmoothingEnergy2D(
    const std::array<Vector2d, 3>& pts,
    const double& M,
    const Vector3d& L_w)
    : m_pts(pts)
    , m_M(M)
    , m_M_inv(1. / M)
    , m_L_w_row(L_w)
{
    m_LTML_row = m_M_inv * m_L_w_row[0] * m_L_w_row;
}

SmoothingEnergy2D::TVector SmoothingEnergy2D::initial_position() const
{
    return m_pts[0];
}

double SmoothingEnergy2D::value(const TVector& x)
{
    assert(x.size() == 2);
    double energy = 0;
    for (size_t i = 0; i < 2; ++i) {
        Vector3d v(x[i], m_pts[1][i], m_pts[2][i]);
        double Lwv = m_L_w_row.dot(v);
        energy += m_M_inv * Lwv * Lwv;
    }

    return energy;
}

void SmoothingEnergy2D::gradient(const TVector& x, TVector& gradv)
{
    assert(x.size() == 2);
    gradv.resize(2);

    for (size_t i = 0; i < 2; ++i) {
        Vector3d v(x[i], m_pts[1][i], m_pts[2][i]);
        gradv[i] = 2 * m_LTML_row.dot(v);
    }
}

void SmoothingEnergy2D::hessian(const TVector& x, MatrixXd& hessian)
{
    assert(x.size() == 2);
    hessian = Matrix2d::Identity() * 2 * m_LTML_row[0];
}

void SmoothingEnergy2D::local_mass_and_stiffness(
    const std::array<Vector2d, 3>& pts,
    double& M,
    Vector3d& L_w)
{
    const double e1 = (pts[1] - pts[0]).norm();
    const double e2 = (pts[2] - pts[0]).norm();

    M = 0.5 * (e1 + e2);

    L_w[0] = -(1 / e1 + 1 / e2);
    L_w[1] = 1 / e1;
    L_w[2] = 1 / e2;
}

void SmoothingEnergy2D::uniform_mass_and_stiffness(
    const std::array<Vector2d, 3>& pts,
    double& M,
    Vector3d& L_w)
{
    M = 2;
    L_w[0] = -2;
    L_w[1] = 1;
    L_w[2] = 1;
}

} // namespace wmtk::optimization