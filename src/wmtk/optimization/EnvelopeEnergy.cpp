#include "EnvelopeEnergy.hpp"

namespace wmtk::optimization {

EnvelopeEnergy2D::EnvelopeEnergy2D(
    const std::shared_ptr<SampleEnvelope>& envelope,
    const double weight,
    bool check_step_validity)
    : m_envelope(envelope)
    , m_weight(weight)
    , m_check_step_validity(check_step_validity)
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
    gradv = m_weight * (r - n);
}

void EnvelopeEnergy2D::hessian(const TVector& x, MatrixXd& hessian)
{
    hessian = m_weight * 2 * Matrix2d::Identity();
}

void EnvelopeEnergy2D::solution_changed(const TVector& new_x) {}

bool EnvelopeEnergy2D::is_step_valid(const TVector& x0, const TVector& x1)
{
    if (!m_check_step_validity) {
        return true;
    }

    Vector2d r(x1);
    if (m_envelope->is_outside(r)) {
        return false;
    }

    return true;
}

} // namespace wmtk::optimization