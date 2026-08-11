#include "SimWildMesh.h"

namespace wmtk::components::simwild {

std::shared_ptr<SampleEnvelope> SimWildMesh::smoothing_energy_envelope(const size_t vid) const
{
    // Order 2 means the vertex sits on a surface boundary or a non-manifold edge, so it is
    // pulled toward the feature-edge envelope rather than the original surface.
    const std::shared_ptr<SampleEnvelope> env =
        m_vertex_attribute[vid].m_order == 2 ? m_order_2_edge_envelope : m_envelope_orig;
    if (!env) {
        log_and_throw_error(
            "Envelope was not initialized. Vertex was of order {}",
            m_vertex_attribute[vid].m_order);
    }
    return env;
}

} // namespace wmtk::components::simwild
