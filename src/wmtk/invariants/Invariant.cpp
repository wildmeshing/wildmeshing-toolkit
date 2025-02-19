#include "Invariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/primitive_range.hpp>

namespace wmtk::invariants {


Invariant::Invariant(const Mesh& mesh)
    : Invariant(mesh, true, true, true)
{}
Invariant::Invariant(
    const Mesh& mesh,
    bool use_before,
    bool use_old_state_in_after,
    bool use_new_state_in_after)
    : m_mesh(mesh)
    , m_use_before(use_before)
    , m_use_old_state_in_after(use_old_state_in_after)
    , m_use_new_state_in_after(use_new_state_in_after)
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
    if (use_after()) {
        const std::vector<Tuple> tuples_after = use_new_state_in_after()
                                                    ? get_top_dimension_cofaces(simplices_after)
                                                    : std::vector<Tuple>{};
        const std::vector<Tuple> tuples_before =
            use_old_state_in_after()
                ? m_mesh.parent_scope([&]() { return get_top_dimension_cofaces(simplices_before); })
                : std::vector<Tuple>{};


        if (!after(tuples_before, tuples_after)) {
            return false;
        }
    }

    return true;
}
bool Invariant::is_collection() const
{
    return false;
}
bool Invariant::use_old_state_in_after() const
{
    return m_use_old_state_in_after;
}
bool Invariant::use_new_state_in_after() const
{
    return m_use_new_state_in_after;
}
bool Invariant::use_after() const
{
    return use_old_state_in_after() || use_new_state_in_after();
}
bool Invariant::use_before() const
{
    return m_use_before;
}
const std::vector<Tuple> Invariant::get_top_dimension_cofaces(
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
} // namespace wmtk::invariants
