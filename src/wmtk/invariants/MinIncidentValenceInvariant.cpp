#include "MinIncidentValenceInvariant.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/link.hpp>

namespace wmtk::invariants {


MinIncidentValenceInvariant::MinIncidentValenceInvariant(const Mesh& m, int64_t min_valence)
    : Invariant(m, true, false, false)
    , m_min_valence(min_valence)
{}

bool MinIncidentValenceInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == PrimitiveType::Edge);
    return is_greater_min_valence(t.tuple());
}

bool MinIncidentValenceInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    for (const Tuple& e : top_dimension_tuples_after) {
        const std::vector<Tuple> e_edges = simplex::faces_single_dimension_tuples(
            mesh(),
            simplex::Simplex(mesh(), mesh().top_simplex_type(), e),
            PrimitiveType::Edge);
        for (const Tuple& edge : e_edges) {
            if (!is_greater_min_valence(edge)) {
                return false;
            }
        }
    }


    return true;
}

bool MinIncidentValenceInvariant::is_greater_min_valence(const Tuple& t) const
{
    using namespace simplex;

    const std::vector<Tuple> vs = faces_single_dimension_tuples(
        mesh(),
        simplex::Simplex::face(mesh(), t),
        PrimitiveType::Vertex);

    const simplex::Simplex v0 = simplex::Simplex::vertex(mesh(), vs[0]);
    const simplex::Simplex v1 = simplex::Simplex::vertex(mesh(), vs[1]);
    const int64_t val0 =
        static_cast<int64_t>(link(mesh(), v0).simplex_vector(PrimitiveType::Vertex).size());
    const int64_t val1 =
        static_cast<int64_t>(link(mesh(), v1).simplex_vector(PrimitiveType::Vertex).size());

    return val0 >= m_min_valence && val1 >= m_min_valence;
}

} // namespace wmtk::invariants
