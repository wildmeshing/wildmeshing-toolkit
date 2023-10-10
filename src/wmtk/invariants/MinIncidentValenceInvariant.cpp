#include "MinIncidentValenceInvariant.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/SimplicialComplex.hpp>

wmtk::MinIncidentValenceInvariant::MinIncidentValenceInvariant(const Mesh& m, long min_valence)
    : MeshInvariant(m)
    , m_min_valence(min_valence)
{}

bool wmtk::MinIncidentValenceInvariant::before(const Tuple& t) const
{
    const Tuple v0 = t;
    const Tuple v1 = mesh().switch_vertex(t);
    long val0 = static_cast<long>(SimplicialComplex::vertex_one_ring(mesh(), v0).size());
    long val1 = static_cast<long>(SimplicialComplex::vertex_one_ring(mesh(), v1).size());
    if (mesh().is_boundary_vertex(v0)) {
        ++val0;
    }
    if (mesh().is_boundary_vertex(v1)) {
        ++val1;
    }

    return val0 >= m_min_valence && val1 >= m_min_valence;
}
