#include "EnvelopeEnergy.hpp"

namespace wmtk::optimization {

namespace {

/**
 * @brief Gauss-Newton hessian of `weight * d(r)^2`, d being the distance to the input:
 * `2 * weight * n n^T` with n the unit normal.
 *
 * Rank one, not isotropic, and that is the whole point. Moving ALONG the input does not
 * change the distance, so the curvature in that direction is zero. An isotropic
 * `2 * weight * I` applies the full normal stiffness tangentially as well, and a surface
 * vertex's AMIPS term is weighted 1e-4 against this one -- so that spurious stiffness
 * divides the tangential Newton step by orders of magnitude. The vertex is then pinned
 * where it stands instead of being free to slide along the curve or surface it sits on,
 * which is what leaves a sliver whose vertices are all on the input unable to relax.
 *
 * On the input itself the direction is unavailable (r == nearest) and the envelope also
 * contributes no gradient there, so the block is dropped and the step is governed by the
 * quality term alone -- exactly the free tangential motion that is wanted. The normal
 * stiffness reappears as soon as the vertex has moved off, and the line search's
 * is_step_valid still refuses any step that leaves the envelope.
 */
template <class Vec, class Mat>
Mat gauss_newton_hessian(const Vec& r, const Vec& nearest, const double weight)
{
    const Vec d = r - nearest;
    const double len = d.norm();
    if (len <= 1e-12) {
        return Mat::Zero();
    }
    const Vec u = d / len;
    return 2.0 * weight * (u * u.transpose());
}

} // namespace

EnvelopeEnergy2D::EnvelopeEnergy2D(
    const std::shared_ptr<SampleEnvelope>& envelope,
    const double weight)
    : m_envelope(envelope)
    , m_weight(weight)
{
    assert(m_envelope);
}

double EnvelopeEnergy2D::value(const TVector& x)
{
    assert(x.size() == 2);
    Vector2d r(x);
    return m_weight * m_envelope->squared_distance(r);
}

void EnvelopeEnergy2D::gradient(const TVector& x, TVector& gradv)
{
    assert(x.size() == 2);
    Vector2d r(x);
    Vector2d n;
    m_envelope->nearest_point(r, n);
    // The derivative of value(), which is m_weight * |r - n|^2. The factor of 2 was missing,
    // so value, gradient and hessian each described a differently scaled energy and the line
    // search tested Armijo against half the true directional derivative.
    gradv = 2 * m_weight * (r - n);
}

void EnvelopeEnergy2D::hessian(const TVector& x, MatrixXd& hessian)
{
    Vector2d r(x);
    Vector2d n;
    m_envelope->nearest_point(r, n);
    hessian = gauss_newton_hessian<Vector2d, Matrix2d>(r, n, m_weight);
}

void EnvelopeEnergy2D::solution_changed(const TVector& new_x) {}


SpringEnergy2D::SpringEnergy2D(const Vector2d& target, const double weight)
    : m_target(target)
    , m_weight(weight)
{}

double SpringEnergy2D::value(const TVector& x)
{
    assert(x.size() == 2);
    return m_weight * (Vector2d(x) - m_target).squaredNorm();
}

void SpringEnergy2D::gradient(const TVector& x, TVector& gradv)
{
    assert(x.size() == 2);
    gradv = 2 * m_weight * (Vector2d(x) - m_target);
}

void SpringEnergy2D::hessian(const TVector& x, MatrixXd& hessian)
{
    // Exact, not Gauss-Newton: with the target fixed the energy is a plain quadratic. The
    // isotropy is the whole difference from EnvelopeEnergy2D -- it is what forbids sliding.
    hessian = 2 * m_weight * Matrix2d::Identity();
}


EnvelopeEnergy3D::EnvelopeEnergy3D(
    const std::shared_ptr<SampleEnvelope>& envelope,
    const double weight)
    : m_envelope(envelope)
    , m_weight(weight)
{
    assert(m_envelope);
}

double EnvelopeEnergy3D::value(const TVector& x)
{
    assert(x.size() == 3);
    Vector3d r(x);
    return m_weight * m_envelope->squared_distance(r);
}

void EnvelopeEnergy3D::gradient(const TVector& x, TVector& gradv)
{
    assert(x.size() == 3);
    Vector3d r(x);
    Vector3d n;
    m_envelope->nearest_point(r, n);
    // The derivative of value(), which is m_weight * |r - n|^2. The factor of 2 was missing,
    // so value, gradient and hessian each described a differently scaled energy and the line
    // search tested Armijo against half the true directional derivative.
    gradv = 2 * m_weight * (r - n);
}

void EnvelopeEnergy3D::hessian(const TVector& x, MatrixXd& hessian)
{
    Vector3d r(x);
    Vector3d n;
    m_envelope->nearest_point(r, n);
    hessian = gauss_newton_hessian<Vector3d, Matrix3d>(r, n, m_weight);
}

void EnvelopeEnergy3D::solution_changed(const TVector& new_x) {}

SpringEnergy3D::SpringEnergy3D(const Vector3d& target, const double weight)
    : m_target(target)
    , m_weight(weight)
{}

double SpringEnergy3D::value(const TVector& x)
{
    assert(x.size() == 3);
    return m_weight * (Vector3d(x) - m_target).squaredNorm();
}

void SpringEnergy3D::gradient(const TVector& x, TVector& gradv)
{
    assert(x.size() == 3);
    gradv = 2 * m_weight * (Vector3d(x) - m_target);
}

void SpringEnergy3D::hessian(const TVector& x, MatrixXd& hessian)
{
    // Exact, not Gauss-Newton: with the target fixed the energy is a plain quadratic.
    hessian = 2 * m_weight * Matrix3d::Identity();
}

} // namespace wmtk::optimization