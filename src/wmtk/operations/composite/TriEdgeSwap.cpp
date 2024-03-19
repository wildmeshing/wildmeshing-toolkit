#include "TriEdgeSwap.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::operations::composite {

TriEdgeSwap::TriEdgeSwap(Mesh& m)
    : Operation(m)
    , m_split(m)
    , m_collapse(m)
{
    operation_name = "TriEdgeSwap";
}


std::vector<simplex::Simplex> TriEdgeSwap::execute(const simplex::Simplex& simplex)
{
    // TODO: set it not to record for now
    m_split.set_not_record();
    m_collapse.set_not_record();

    // input
    //    / \.
    //   /   \ .
    //  /  f  \ .
    // X--->---
    //  \     /
    //   \   /
    //    \ /
    const auto split_simplicies = m_split(simplex);
    if (split_simplicies.empty()) return {};
    assert(split_simplicies.size() == 1);

    // after split
    //    /|\ .
    //   / | \ .
    //  /  | f\ .
    //  ---X-->
    //  \  |  /
    //   \ | /
    //    \|/

    // switch also face to keep edge orientation
    const Tuple collapse_input_tuple =
        mesh().switch_tuples(split_simplicies.front().tuple(), {PrimitiveType::Edge, PrimitiveType::Triangle});
    // switch edge - switch face
    //    /|\ .
    //   / ^ \ .
    //  /f |  \ .
    //  ---X---
    //  \  |  /
    //   \ | /
    //    \|/
    const auto collapse_simplicies =
        m_collapse(simplex::Simplex(m_collapse.primitive_type(), collapse_input_tuple));
    if (collapse_simplicies.empty()) return {};
    assert(collapse_simplicies.size() == 1);

    // collapse output
    //     X
    //    /|\ .
    //   < | \ .
    //  /  |  \ .
    // | f |   |
    //  \  |  /
    //   \ | /
    //    \|/
    // adjust return tuple to be the swapped edge in the same orientation as the input
    const Tuple output_tuple =
        mesh().switch_tuples(collapse_simplicies.front().tuple(), {PrimitiveType::Edge, PrimitiveType::Vertex});

    return {simplex::Simplex::edge(output_tuple)};
}


std::vector<simplex::Simplex> TriEdgeSwap::unmodified_primitives(
    const simplex::Simplex& simplex) const
{
    return {simplex};
}

} // namespace wmtk::operations::composite
