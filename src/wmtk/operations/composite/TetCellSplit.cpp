#include "TetCellSplit.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::operations::composite {


TetCellSplit::TetCellSplit(Mesh& m)
    : Operation(m)
    , m_split(m)
    , m_collapse(m)
{}

std::vector<Simplex> TetCellSplit::execute(const Simplex& simplex)
{
    const auto first_split_simplicies = m_split(Simplex::edge(simplex.tuple()));
    if (first_split_simplicies.empty()) return {};
    assert(first_split_simplicies.size() == 1);
    const Tuple first_split_ret = first_split_simplicies.front().tuple();

    const Tuple second_split_input = mesh().switch_edge(mesh().switch_vertex(first_split_ret));
    const auto second_split_simplicies = m_split(Simplex::edge(second_split_input));
    if (second_split_simplicies.empty()) return {};
    assert(second_split_simplicies.size() == 1);
    const Tuple second_split_ret = second_split_simplicies.front().tuple();

    const Tuple third_split_input =
        mesh().switch_edge(mesh().switch_vertex(mesh().switch_face(second_split_ret)));
    const auto third_split_simplicies = m_split(Simplex::edge(third_split_input));
    if (third_split_simplicies.empty()) return {};
    assert(third_split_simplicies.size() == 1);
    const Tuple third_split_ret = third_split_simplicies.front().tuple();


    const Tuple first_collapse_input = mesh().switch_face(mesh().switch_edge(third_split_ret));
    const auto first_collapse_simplicies = m_collapse(Simplex::edge(first_collapse_input));
    if (first_collapse_simplicies.empty()) return {};
    assert(first_collapse_simplicies.size() == 1);
    const Tuple first_collapse_ret = first_collapse_simplicies.front().tuple();

    const Tuple second_collapse_input = first_collapse_ret;
    const auto second_collapse_simplicies = m_collapse(Simplex::edge(second_collapse_input));
    if (second_collapse_simplicies.empty()) return {};
    assert(second_collapse_simplicies.size() == 1);
    const Tuple second_collapse_ret = second_collapse_simplicies.front().tuple();

    const Tuple output_tuple = mesh().switch_face(mesh().switch_tetrahedron(second_collapse_ret));

    return {Simplex::vertex(output_tuple)};
}


std::vector<Simplex> TetCellSplit::unmodified_primitives(const Simplex& simplex) const
{
    return {simplex};
}

} // namespace wmtk::operations::composite
