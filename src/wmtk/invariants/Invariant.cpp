#include "Invariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/primitive_range.hpp>

namespace wmtk {

Invariant::Invariant(const Mesh& mesh)
    : m_mesh(mesh)
{}
Invariant::~Invariant() = default;
bool Invariant::before(const simplex::Simplex& t) const
{
    return true;
}
bool Invariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    return true;
}

const Mesh& Invariant::mesh() const
{
    return m_mesh;
}
bool Invariant::directly_modified_after(
    const std::vector<simplex::Simplex>& simplices_before,
    const std::vector<simplex::Simplex>& simplices_after) const
{
    const std::vector<Tuple> tuples_after = get_top_dimension_cofaces(simplices_after);
    const std::vector<Tuple> tuples_before =
        m_mesh.parent_scope([&]() { return get_top_dimension_cofaces(simplices_before); });


    if (!after(tuples_before, tuples_after)) {
        return false;
    }

    return true;
}

const std::vector<Tuple> invariants::Invariant::get_top_dimension_cofaces(
    const std::vector<simplex::Simplex>& simplices) const
{
    simplex::SimplexCollection all_simplices(mesh());

    for (const simplex::Simplex& s : simplices) {
        all_simplices.add(simplex::top_dimension_cofaces(mesh(), s, false));
    }
    all_simplices.sort_and_clean();

    const std::vector<Tuple> all_tuples =
        all_simplices.simplex_vector_tuples(m_mesh.top_simplex_type());

    assert(all_tuples.size() == all_simplices.simplex_vector().size());

    return all_tuples;
}
} // namespace wmtk
