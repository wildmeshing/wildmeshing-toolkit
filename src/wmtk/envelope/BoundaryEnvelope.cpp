#include "BoundaryEnvelope.hpp"

namespace wmtk {

void BoundaryEnvelope::build(
    const std::vector<Eigen::Vector3d>& V,
    const std::vector<Eigen::Vector2i>& E,
    double eps,
    bool use_exact)
{
    // The flag has to be set before init(): an edge envelope builds its exact structure only
    // when it is.
    m_envelope.use_exact = use_exact;
    m_envelope.init(V, E, eps);
    m_initialized = true;
}

} // namespace wmtk
