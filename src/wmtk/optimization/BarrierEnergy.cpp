#include "BarrierEnergy.hpp"


namespace wmtk::optimization {

BarrierEnergy2D::BarrierEnergy2D(
    const MatrixXd& V,
    const MatrixXi& E,
    const size_t vid,
    const double dhat,
    const double weight)
    : m_collision_mesh(V, E)
    , m_V(V)
#ifndef WMTK_SMOOTH_BARRIER
    , m_B(dhat)
#endif
    , m_x0(V.row(vid))
    , m_vid(vid)
    , m_weight(weight)
#ifdef WMTK_SMOOTH_BARRIER
    , m_params(dhat, 1.0, 0.0, 1.0, 0.0, 1)
    , m_smooth_B(m_params)
#endif
{
    assert(V.cols() == 2);
    assert(E.cols() == 2);

#ifdef WMTK_SMOOTH_BARRIER
    m_smooth_collisions.build(m_collision_mesh, m_V, m_params);
#else
    m_collisions.build(m_collision_mesh, m_V, m_B.dhat());
#endif
}

BarrierEnergy2D::TVector BarrierEnergy2D::initial_position() const
{
    return m_x0;
}

void BarrierEnergy2D::replace_vid(const size_t vid)
{
    //// reset m_V
    // m_V.row(m_vid) = m_x0;

    // set new vid
    m_vid = vid;
    m_x0 = m_V.row(m_vid);

#ifdef WMTK_SMOOTH_BARRIER
    m_smooth_collisions.build(m_collision_mesh, m_V, m_params);
#else
    m_collisions.build(m_collision_mesh, m_V, m_B.dhat());
#endif
}


double BarrierEnergy2D::value(const TVector& x)
{
    assert(x.size() == 2);
    update_collisions(x);
#ifdef WMTK_SMOOTH_BARRIER
    double potential = m_smooth_B(m_smooth_collisions, m_collision_mesh, m_V);
#else
    double potential = m_B(m_collisions, m_collision_mesh, m_V);
#endif
    return m_weight * potential;
}

void BarrierEnergy2D::gradient(const TVector& x, TVector& gradv)
{
    assert(x.size() == 2);
    update_collisions(x);
#ifdef WMTK_SMOOTH_BARRIER
    VectorXd grad = m_smooth_B.gradient(m_smooth_collisions, m_collision_mesh, m_V);
#else
    VectorXd grad = m_B.gradient(m_collisions, m_collision_mesh, m_V);
#endif
    gradv = m_weight * grad.segment<2>(m_vid * 2);
}

void BarrierEnergy2D::hessian(const TVector& x, MatrixXd& hessian)
{
    assert(x.size() == 2);
    update_collisions(x);
#ifdef WMTK_SMOOTH_BARRIER
    Eigen::SparseMatrix<double> hess =
        m_smooth_B.hessian(m_smooth_collisions, m_collision_mesh, m_V);
#else
    Eigen::SparseMatrix<double> hess = m_B.hessian(m_collisions, m_collision_mesh, m_V);
#endif

    Matrix2d v_hess;
    v_hess.setZero();
    v_hess(0, 0) = hess.coeff(2 * m_vid, 2 * m_vid);
    v_hess(1, 1) = hess.coeff(2 * m_vid + 1, 2 * m_vid + 1);
    hessian = m_weight * v_hess;
}

void BarrierEnergy2D::update_collisions(const TVector& x)
{
    assert(m_V.cols() == x.size());
    const VectorXd& curr = m_V.row(m_vid);
    if (curr == x) {
        return;
    }

    m_V.row(m_vid) = x;
#ifdef WMTK_SMOOTH_BARRIER
    m_smooth_collisions.build(m_collision_mesh, m_V, m_params);
#else
    m_collisions.build(m_collision_mesh, m_V, m_B.dhat());
#endif
}

} // namespace wmtk::optimization