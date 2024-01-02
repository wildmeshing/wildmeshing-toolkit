#include "TriFaceSplit.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::operations::composite {

TriFaceSplit::TriFaceSplit(Mesh& m)
    : Operation(m)
    , m_split(m)
    , m_collapse(m)
{}

std::vector<simplex::Simplex> TriFaceSplit::execute(const simplex::Simplex& simplex)
{
    // input
    //     p1
    //    / \ .
    //   / f \ .
    //  /     \ .
    // p0-->--p2
    //  \     /
    //   \   /
    //    \ /

    const auto split_simplicies = m_split(simplex::Simplex::edge(simplex.tuple()));
    if (split_simplicies.empty()) return {};
    assert(split_simplicies.size() == 1);
    const Tuple split_ret = split_simplicies.front().tuple();


    // after split
    //    /|\ .
    //   / | \ .
    //  /  | f\ .
    //  ---X-->
    //  \  |  /
    //   \ | /
    //    \|/

    // switch edge - switch face
    //    /|\ .
    //   / v \ .
    //  /f |  \ .
    //  ---X---
    //  \  |  /
    //   \ | /
    //    \|/
    const Tuple second_split_input_tuple = mesh().switch_vertex(mesh().switch_edge(split_ret));
    const auto second_split_simplicies = m_split(simplex::Simplex::edge(second_split_input_tuple));
    if (second_split_simplicies.empty()) return {};
    assert(second_split_simplicies.size() == 1);
    const Tuple second_split_ret = second_split_simplicies.front().tuple();

    // after split
    //
    //     /|\ .
    //    / | \ .
    //   /  X  \ .
    //  /  /|\  \ .
    // /__/_v_\__\ .
    //  \   |   /
    //   \  |  /
    //    \ | /
    //     \|/

    // collapse the split ret
    //     /|\ .
    //    / | \ .
    //   / /|\ \ .
    //  / / | \ \ .
    //  |/__X__\>
    //  \   |   /
    //   \  |  /
    //    \ | /
    //     \|/

    const Tuple coll_input_tuple = mesh().switch_edge(mesh().switch_vertex(second_split_ret));
    const auto collapse_simplicies = m_collapse(simplex::Simplex::edge(coll_input_tuple));
    if (collapse_simplicies.empty()) return {};
    assert(collapse_simplicies.size() == 1);
    const Tuple coll_ret = collapse_simplicies.front().tuple();

    // collapse output
    //     /| \ .
    //    / |  \ .
    //   /  *   \ .
    //  / /   \  \ .
    // / /  f   > \ .
    // |/_ _ _ _ \|
    //  \        /
    //   \      /
    //    \    /
    //     \  /
    // return new vertex's tuple
    const Tuple output_tuple = mesh().switch_edge(mesh().switch_vertex(coll_ret));

    return {simplex::Simplex::vertex(output_tuple)};
}

std::vector<simplex::Simplex> TriFaceSplit::unmodified_primitives(
    const simplex::Simplex& simplex) const
{
    return {simplex};
}

} // namespace wmtk::operations::composite
