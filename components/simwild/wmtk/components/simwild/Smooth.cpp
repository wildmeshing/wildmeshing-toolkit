#include "SimWildMesh.h"

namespace wmtk::components::simwild {

std::shared_ptr<SampleEnvelope> SimWildMesh::smoothing_energy_envelope(const size_t vid) const
{
    // Match TetWild: order >= 2 is a surface boundary, non-manifold edge, or a junction of
    // such edges, and is pulled toward the feature-edge envelope when one exists.
    if (get_order_of_vertex(vid) >= 2 && m_order_2_edge_envelope &&
        m_order_2_edge_envelope->initialized()) {
        return m_order_2_edge_envelope;
    }
    return m_envelope;
}

} // namespace wmtk::components::simwild
