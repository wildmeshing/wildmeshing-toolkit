#include "TetEdgeSwap.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::operations::composite {
TetEdgeSwap::TetEdgeSwap(Mesh& m, long collapse_index)
    : Operation(m)
    , m_split(m)
    , m_collapse(m)
    , m_collapse_index(collapse_index)
{}

std::vector<Simplex> TetEdgeSwap::execute(const Simplex& simplex)
{
    const auto split_simplicies = m_split(simplex);
    if (split_simplicies.empty()) return {};
    assert(split_simplicies.size() == 1);

    // aftert split
    const Tuple& split_ret = split_simplicies.front().tuple();
    assert(!mesh().is_boundary(split_ret));
    auto iter_tuple = split_ret;
    std::vector<Tuple> candidate_edge_tuples;
    while (true) {
        candidate_edge_tuples.push_back(mesh().switch_edge(iter_tuple));
        iter_tuple = mesh().switch_tetrahedron(mesh().switch_face(iter_tuple));
        if (iter_tuple == split_ret) {
            break;
        }
    }

    assert(m_collapse_index < candidate_edge_tuples.size());

    const auto collapse_simplicies =
        m_collapse(Simplex(m_collapse.primitive_type(), candidate_edge_tuples[m_collapse_index]));
    if (collapse_simplicies.empty()) return {};
    assert(collapse_simplicies.size() == 1);

    const Tuple output_tuple = mesh().switch_edge(collapse_simplicies.front().tuple());

    // TODO: get all new edges and return them
    return {simplex::Simplex::edge(output_tuple)};
}

std::vector<Simplex> TetEdgeSwap::unmodified_primitives(const Simplex& simplex) const
{
    return {simplex};
}

} // namespace wmtk::operations::composite