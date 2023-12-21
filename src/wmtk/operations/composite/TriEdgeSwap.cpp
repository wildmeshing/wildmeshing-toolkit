#include "TriEdgeSwap.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::operations::composite {

TriEdgeSwap::TriEdgeSwap(Mesh& m)
    : EdgeSplit(m)
    , EdgeCollapse(m)
{}


std::vector<Simplex> TriEdgeSwap::execute(const Simplex& simplex)
{
    auto& mesh = EdgeSplit::mesh();
    // input
    //    / \ .
    //   /   \ .
    //  /  f  \ .
    // X--->---
    //  \     /
    //   \   /
    //    \ /
    const auto split_simplicies = EdgeSplit::execute(simplex);
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
        mesh.switch_face(mesh.switch_edge(split_simplicies.front().tuple()));
    // switch edge - switch face
    //    /|\ .
    //   / ^ \ .
    //  /f |  \ .
    //  ---X---
    //  \  |  /
    //   \ | /
    //    \|/
    const auto collapse_simplicies =
        EdgeCollapse::execute(Simplex(EdgeCollapse::primitive_type(), collapse_input_tuple));
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
        mesh.switch_vertex(mesh.switch_edge(collapse_simplicies.front().tuple()));

    return {simplex::Simplex::edge(output_tuple)};
}


std::vector<Simplex> TriEdgeSwap::unmodified_primitives(const Simplex& simplex) const
{
    return {simplex};
}

} // namespace wmtk::operations::composite
