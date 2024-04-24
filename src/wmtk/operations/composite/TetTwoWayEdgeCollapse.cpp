#include "TetTwoWayEdgeCollapse.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::operations::composite {

TetTwoWayEdgeCollapse::TetTwoWayEdgeCollapse(Mesh& m)
    : Operation(m)
    , m_collapse(m)
{}

std::vector<simplex::Simplex> TetTwoWayEdgeCollapse::execute(const simplex::Simplex& simplex)
{
    assert(simplex.primitive_type() == PrimitiveType::Edge);
    const auto collapse_1_simplices = m_collapse(simplex);
    if (!collapse_1_simplices.empty()) {
        return collapse_1_simplices;
    }

    const auto collapse_2_simplices = m_collapse(
        simplex::Simplex::edge(mesh().switch_tuple(simplex.tuple(), PrimitiveType::Vertex)));
    return collapse_2_simplices;
}

std::vector<simplex::Simplex> TetTwoWayEdgeCollapse::unmodified_primitives(
    const simplex::Simplex& simplex) const
{
    return {simplex};
}


} // namespace wmtk::operations::composite