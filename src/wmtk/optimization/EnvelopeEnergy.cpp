#include "EnvelopeEnergy.hpp"

namespace wmtk::optimization {

EnvelopeEnergy2D::EnvelopeEnergy2D(
    const std::shared_ptr<SampleEnvelope>& envelope,
    const std::array<Vector2d, 3>& pts)
    : m_envelope(envelope)
    , m_pts(pts)
{
    assert(m_envelope);
}

double EnvelopeEnergy2D::value(const TVector& x)
{
    assert(x.size() == 2);
    Vector2d r(x);
    return m_envelope->squared_distance(r);
}

void EnvelopeEnergy2D::gradient(const TVector& x, TVector& gradv)
{
    assert(x.size() == 2);
    Vector2d r(x);
    Vector2d n;
    m_envelope->nearest_point(r, n);
    gradv = r - n;
}

void EnvelopeEnergy2D::hessian(const TVector& x, MatrixXd& hessian)
{
    hessian = 2 * Matrix2d::Identity();
}

void EnvelopeEnergy2D::solution_changed(const TVector& new_x) {}

bool EnvelopeEnergy2D::is_step_valid(const TVector& x0, const TVector& x1)
{
    // Vector2d r(x1);
    // if (m_envelope->is_outside(r)) {
    //     return false;
    // }

    //  for (size_t i = 0; i < 2; ++i) {
    //      std::array<Eigen::Vector2d, 2> edge;
    //      edge[0] = r;
    //      edge[1] = m_pts[i + 1];
    //      if (m_envelope->is_outside(edge)) {
    //          return false;
    //      }
    //  }

    return true;
}

} // namespace wmtk::optimization