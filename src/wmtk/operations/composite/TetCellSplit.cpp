#include "TetCellSplit.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::operations::composite {


TetCellSplit::TetCellSplit(Mesh& m)
    : Operation(m)
    , m_split(m)
    , m_collapse(m)
{}

std::vector<simplex::Simplex> TetCellSplit::execute(const simplex::Simplex& simplex)
{
    const auto first_split_simplicies = m_split(simplex::Simplex::edge(mesh(), simplex.tuple()));
    if (first_split_simplicies.empty()) return {};
    assert(first_split_simplicies.size() == 1);
    const Tuple first_split_ret = first_split_simplicies.front().tuple();

    const Tuple second_split_input = mesh().switch_tuples(first_split_ret, {PrimitiveType::Edge});
    const auto second_split_simplicies =
        m_split(simplex::Simplex::edge(mesh(), second_split_input));
    if (second_split_simplicies.empty()) return {};
    assert(second_split_simplicies.size() == 1);
    const Tuple second_split_ret = second_split_simplicies.front().tuple();

    const Tuple third_split_input = mesh().switch_tuples(
        second_split_ret,
        {PrimitiveType::Triangle, PrimitiveType::Edge, PrimitiveType::Vertex});
    const auto third_split_simplicies = m_split(simplex::Simplex::edge(mesh(), third_split_input));
    if (third_split_simplicies.empty()) return {};
    assert(third_split_simplicies.size() == 1);
    const Tuple third_split_ret = third_split_simplicies.front().tuple();

    const Tuple first_collapse_input = mesh().switch_tuples(
        third_split_ret,
        {PrimitiveType::Triangle,
         PrimitiveType::Tetrahedron,
         PrimitiveType::Triangle,
         PrimitiveType::Tetrahedron,
         PrimitiveType::Vertex,
         PrimitiveType::Edge,
         PrimitiveType::Triangle});
    const auto first_collapse_simplicies =
        m_collapse(simplex::Simplex::edge(mesh(), first_collapse_input));
    if (first_collapse_simplicies.empty()) return {};
    assert(first_collapse_simplicies.size() == 1);
    const Tuple first_collapse_ret = first_collapse_simplicies.front().tuple();

    const Tuple second_collapse_input = first_collapse_ret;
    const auto second_collapse_simplicies =
        m_collapse(simplex::Simplex::edge(mesh(), second_collapse_input));
    if (second_collapse_simplicies.empty()) return {};
    assert(second_collapse_simplicies.size() == 1);
    const Tuple second_collapse_ret = second_collapse_simplicies.front().tuple();

    const Tuple output_tuple = mesh().switch_tuples(
        second_collapse_ret,
        {PrimitiveType::Tetrahedron, PrimitiveType::Triangle});

    return {simplex::Simplex::vertex(mesh(), output_tuple)};
}


std::vector<simplex::Simplex> TetCellSplit::unmodified_primitives(
    const simplex::Simplex& simplex) const
{
    return {simplex};
}

} // namespace wmtk::operations::composite
