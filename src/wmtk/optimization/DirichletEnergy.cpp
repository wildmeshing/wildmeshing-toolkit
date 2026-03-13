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


SmoothingEnergy2D::SmoothingEnergy2D(std::vector<std::array<double, 4>> m_cells)
{
    assert(m_cells.size() == 2);

    const Vector2d p0(m_cells[0][2], m_cells[0][3]);
    const Vector2d p1(m_cells[0][0], m_cells[0][1]); // this vertex is optimized
    const Vector2d p2(m_cells[1][2], m_cells[1][3]);

    m_x.resize(3, 2);
    m_x.row(0) = p0;
    m_x.row(1) = p1;
    m_x.row(2) = p2;

    // uniform laplacian
    m_M = 2;
    m_M_inv = 1 / m_M;
    m_L_w[0] = 1;
    m_L_w[1] = -2;
    m_L_w[2] = 1;

    m_LTML_row1 = m_M_inv * m_L_w[1] * m_L_w;
}

SmoothingEnergy2D::TVector SmoothingEnergy2D::initial_position() const
{
    return m_x.row(1);
}

double SmoothingEnergy2D::value(const TVector& x)
{
    assert(x.size() == 2);
    double energy = 0;
    for (size_t i = 0; i < 2; ++i) {
        Vector3d v(m_x(0, i), x[i], m_x(2, i));
        // energy += v.transpose() * m_LTML * v;
        double Lwv = m_L_w.dot(v);
        energy += m_M_inv * Lwv * Lwv;
    }

    return energy;
}

void SmoothingEnergy2D::gradient(const TVector& x, TVector& gradv)
{
    assert(x.size() == 2);
    gradv.resize(2);

    for (size_t i = 0; i < 2; ++i) {
        Vector3d v(m_x(0, i), x[i], m_x(2, i));
        gradv[i] = 2 * m_LTML_row1.dot(v);
    }
}

void SmoothingEnergy2D::hessian(const TVector& x, MatrixXd& hessian)
{
    assert(x.size() == 2);
    hessian = Matrix2d::Identity() * 2 * m_LTML_row1[1];
}

void SmoothingEnergy2D::add_mass_and_stiffness_matrix(const double& M, const Vector3d& L_w)
{
    m_M = M;
    m_M_inv = 1 / M;
    m_L_w = L_w;
    m_LTML_row1 = m_M_inv * m_L_w[1] * m_L_w;
}

void SmoothingEnergy2D::compute_local_mass_and_stiffness(
    std::vector<std::array<double, 4>> m_cells,
    double& M,
    Vector3d& L_w)
{
    assert(m_cells.size() == 2);

    const Vector2d p0(m_cells[0][2], m_cells[0][3]);
    const Vector2d p1(m_cells[0][0], m_cells[0][1]); // this vertex is optimized
    const Vector2d p2(m_cells[1][2], m_cells[1][3]);

    const double e0 = (p1 - p0).norm();
    const double e2 = (p2 - p1).norm();

    M = 0.5 * (e0 + e2);

    L_w[0] = 1 / e0;
    L_w[1] = -(1 / e0 + 1 / e2);
    L_w[2] = 1 / e2;
}

} // namespace wmtk::optimization