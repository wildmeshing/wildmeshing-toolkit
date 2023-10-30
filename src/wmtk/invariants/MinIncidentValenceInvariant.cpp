#include "MinIncidentValenceInvariant.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/link.hpp>

namespace wmtk::invariants {


MinIncidentValenceInvariant::MinIncidentValenceInvariant(const Mesh& m, long min_valence)
    : MeshInvariant(m)
    , m_min_valence(min_valence)
{}

bool MinIncidentValenceInvariant::before(const Tuple& t) const
{
    return is_greater_min_valence(t);
}

bool MinIncidentValenceInvariant::after(PrimitiveType type, const std::vector<Tuple>& t) const
{
    if (type == PrimitiveType::Edge) {
        for (const Tuple& e : t) {
            if (!is_greater_min_valence(e)) {
                return false;
            }
        }
    }

    return true;
}

bool MinIncidentValenceInvariant::is_greater_min_valence(const Tuple& t) const
{
    using namespace simplex;

    const std::vector<Tuple> vs =
        faces_single_dimension(mesh(), Simplex::face(t), PrimitiveType::Vertex);

    const Simplex v0 = Simplex::vertex(vs[0]);
    const Simplex v1 = Simplex::vertex(vs[1]);
    const long val0 =
        static_cast<long>(link(mesh(), v0).simplex_vector(PrimitiveType::Vertex).size());
    const long val1 =
        static_cast<long>(link(mesh(), v1).simplex_vector(PrimitiveType::Vertex).size());

    return val0 >= m_min_valence && val1 >= m_min_valence;
}

} // namespace wmtk::invariants