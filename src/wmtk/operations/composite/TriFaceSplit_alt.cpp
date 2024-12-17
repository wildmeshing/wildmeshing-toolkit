#include "TriFaceSplit_alt.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::operations::composite {

TriFaceSplit_alt::TriFaceSplit_alt(Mesh& m)
    : Operation(m)
    , m_split_1(m)
    , m_split_2(m)
    , m_collapse(m)
{}

std::vector<simplex::Simplex> TriFaceSplit_alt::execute(const simplex::Simplex& simplex)
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

    const auto split_simplicies = m_split_1(simplex::Simplex::edge(mesh(), simplex.tuple()));
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
    const Tuple second_split_input_tuple =
        mesh().switch_tuples(split_ret, {PrimitiveType::Edge, PrimitiveType::Vertex});
    const auto second_split_simplicies =
        m_split_2(simplex::Simplex::edge(mesh(), second_split_input_tuple));
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

    const Tuple col1_input_tuple =
        mesh().switch_tuples(second_split_ret, {PrimitiveType::Vertex, PrimitiveType::Edge});
    const auto collapse_simplicies = m_collapse(simplex::Simplex::edge(mesh(), col1_input_tuple));
    if (collapse_simplicies.empty()) return {};
    assert(collapse_simplicies.size() == 1);
    const Tuple col1_ret = collapse_simplicies.front().tuple();

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
    const Tuple output_tuple =
        mesh().switch_tuples(col1_ret, {PrimitiveType::Vertex, PrimitiveType::Edge});

    return {simplex::Simplex::vertex(mesh(), output_tuple)};
}

std::vector<simplex::Simplex> TriFaceSplit_alt::unmodified_primitives(
    const simplex::Simplex& simplex) const
{
    return {simplex};
}

} // namespace wmtk::operations::composite
