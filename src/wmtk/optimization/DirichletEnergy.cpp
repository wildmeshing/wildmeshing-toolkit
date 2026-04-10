#include "DirichletEnergy.hpp"

#include <igl/cotmatrix.h>
#include <igl/massmatrix.h>

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


BiharmonicEnergy2D::BiharmonicEnergy2D(
    const std::array<Vector2d, 3>& pts,
    const double& M,
    const Vector3d& L_w,
    const double weight)
    : m_pts(pts)
    , m_M(M)
    , m_M_inv(1. / M)
    , m_L_w_row(L_w)
    , m_weight(weight)
{
    m_LTML_row = m_M_inv * m_L_w_row[0] * m_L_w_row;
}

BiharmonicEnergy2D::TVector BiharmonicEnergy2D::initial_position() const
{
    return m_pts[0];
}

double BiharmonicEnergy2D::value(const TVector& x)
{
    assert(x.size() == 2);
    double energy = 0;
    for (size_t i = 0; i < 2; ++i) {
        Vector3d v(x[i], m_pts[1][i], m_pts[2][i]);
        double Lwv = m_L_w_row.dot(v);
        energy += m_M_inv * Lwv * Lwv;
    }

    return m_weight * energy;
}

void BiharmonicEnergy2D::gradient(const TVector& x, TVector& gradv)
{
    assert(x.size() == 2);
    gradv.resize(2);

    for (size_t i = 0; i < 2; ++i) {
        Vector3d v(x[i], m_pts[1][i], m_pts[2][i]);
        gradv[i] = 2 * m_LTML_row.dot(v);
    }

    gradv *= m_weight;
}

void BiharmonicEnergy2D::hessian(const TVector& x, MatrixXd& hessian)
{
    assert(x.size() == 2);
    hessian = m_weight * Matrix2d::Identity() * 2 * m_LTML_row[0];
}

void BiharmonicEnergy2D::local_mass_and_stiffness(
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

void BiharmonicEnergy2D::uniform_mass_and_stiffness(
    const std::array<Vector2d, 3>& pts,
    double& M,
    Vector3d& L_w)
{
    M = 2;
    L_w[0] = -2;
    L_w[1] = 1;
    L_w[2] = 1;
}


BiharmonicEnergy3D::BiharmonicEnergy3D(
    const MatrixXd& pts,
    const double& M,
    const VectorXd& L_w,
    const double weight)
    : m_pts(pts)
    , m_M(M)
    , m_M_inv(1. / M)
    , m_L_w_row(L_w)
    , m_weight(weight)
{
    assert(pts.cols() == 3);
    // assert(pts.rows() == L_w.size());
    if (pts.rows() != L_w.size()) {
        log_and_throw_error(
            "Number of points and entries in L_w are different.\npts = {}\nL_w = {}",
            pts,
            L_w);
    }

    m_LTML_row = m_M_inv * m_L_w_row[0] * m_L_w_row;
}

BiharmonicEnergy3D::TVector BiharmonicEnergy3D::initial_position() const
{
    return m_pts.row(0);
}

double BiharmonicEnergy3D::value(const TVector& x)
{
    assert(x.size() == 3);
    double energy = 0;
    for (size_t i = 0; i < 3; ++i) {
        // Vector3d v(x[i], m_pts[1][i], m_pts[2][i]);
        VectorXd v = m_pts.col(i);
        v[0] = x[i];
        double Lwv = m_L_w_row.dot(v);
        energy += m_M_inv * Lwv * Lwv;
    }

    return m_weight * energy;
}

void BiharmonicEnergy3D::gradient(const TVector& x, TVector& gradv)
{
    assert(x.size() == 3);
    gradv.resize(3);

    for (size_t i = 0; i < 3; ++i) {
        // Vector3d v(x[i], m_pts[1][i], m_pts[2][i]);
        VectorXd v = m_pts.col(i);
        v[0] = x[i];
        gradv[i] = 2 * m_LTML_row.dot(v);
    }

    gradv *= m_weight;
}

void BiharmonicEnergy3D::hessian(const TVector& x, MatrixXd& hessian)
{
    assert(x.size() == 3);
    hessian = m_weight * Matrix3d::Identity() * 2 * m_LTML_row[0];
}

void BiharmonicEnergy3D::global_mass_and_stiffness(
    const MatrixXd& pts,
    const MatrixXi& tris,
    Eigen::SparseMatrix<double>& M,
    Eigen::SparseMatrix<double>& L_w)
{
    igl::massmatrix(pts, tris, igl::MASSMATRIX_TYPE_DEFAULT, M);
    igl::cotmatrix(pts, tris, L_w);
}

void BiharmonicEnergy3D::extract_local_mass_and_stiffness(
    const size_t vid,
    const Eigen::SparseMatrix<double>& M_glob,
    const Eigen::SparseMatrix<double>& L_w_glob,
    double& M_loc,
    VectorXd& L_w_loc)
{
    assert(L_w_glob.rows() == L_w_glob.cols());
    assert(vid < L_w_glob.rows());

    M_loc = M_glob.coeff(vid, vid);

    /**
     * The stiffness matrix is symmetric. It doesn't matter if we extract the row or column for vid.
     */

    const int* outer = L_w_glob.outerIndexPtr();
    int n_non_zeros = outer[vid + 1] - outer[vid];

    L_w_loc.resize(n_non_zeros);

    size_t adj_counter = 1;
    for (Eigen::SparseMatrix<double>::InnerIterator it(L_w_glob, vid); it; ++it) {
        if (it.row() == vid) {
            L_w_loc[0] = it.value();
        } else {
            L_w_loc[adj_counter++] = it.value();
        }
    }

    assert(adj_counter == n_non_zeros);
}

void BiharmonicEnergy3D::adjacency_from_stiffness(
    const size_t vid,
    const Eigen::SparseMatrix<double>& L_w_glob,
    std::vector<size_t>& adj)
{
    adj.clear();
    adj.reserve(6);

    for (Eigen::SparseMatrix<double>::InnerIterator it(L_w_glob, vid); it; ++it) {
        if (it.row() != vid) {
            adj.push_back(it.row());
        }
    }
}

void BiharmonicEnergy3D::uniform_mass_and_stiffness(const MatrixXd& pts, double& M, VectorXd& L_w)
{
    M = pts.rows() - 1;

    L_w.resize(pts.rows(), 1);
    L_w[0] = -M;
    for (size_t i = 1; i < pts.rows(); ++i) {
        L_w[i] = 1;
    }
}

} // namespace wmtk::optimization