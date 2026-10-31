#include "SimWildMesh.h"

namespace wmtk::components::simwild {

std::shared_ptr<Envelope> SimWildMesh::smoothing_energy_envelope(const size_t vid) const
{
    // A vertex on a tagged curve is pulled toward its tube, the feature tube first, as tetwild
    // pulls (tagged_edge_pull_envelope()); the containment that holds it there reads the same
    // tags.
    if (auto tube = tagged_edge_pull_envelope(vid)) {
        return tube;
    }
    if (m_input_envelopes.empty()) return m_envelope;
    // Toward the tags of the vertex's own faces, one real envelope at a time -- the member of
    // their union the vertex is farthest from, as tetwild pulls a vertex toward its inputs. The
    // containment check holds every face to its own envelope. A vertex on no tagged face keeps
    // m_envelope's pull.
    InputSet set = 0;
    const auto& FA = std::as_const(m_face_attribute);
    // Named, not iterated as a temporary: faces() returns a reference into the collection.
    const simplex::SimplexCollection surf = get_surface_faces_for_vertex(vid);
    for (const simplex::Face& f : surf.faces()) {
        set |= FA[std::get<1>(tuple_from_face(f.vertices()))].input_set();
    }
    if (set == 0) return m_envelope;
    return m_input_envelopes.pull(set, m_vertex_attribute[vid].m_posf);
}

} // namespace wmtk::components::simwild
