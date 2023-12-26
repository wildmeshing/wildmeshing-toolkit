#include "Operation.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::operations {


Operation::Operation(Mesh& mesh)
    : m_mesh(mesh)
    , m_invariants(mesh)
{}

Operation::~Operation() = default;

std::shared_ptr<operations::NewAttributeStrategy> Operation::get_strategy(
    attribute::MeshAttributeHandleVariant& attribute)
{
    for (auto& s : m_new_attr_strategies) {
        if (s->matches_attribute(attribute)) return s;
    }

    throw std::runtime_error("unable to find attribute");
}

void Operation::set_strategy(
    attribute::MeshAttributeHandleVariant& attribute,
    const std::shared_ptr<operations::NewAttributeStrategy>& other)
{
    for (size_t i = 0; i < m_new_attr_strategies.size(); ++i) {
        if (m_new_attr_strategies[i]->matches_attribute(attribute)) {
            m_new_attr_strategies[i] = other;
            m_new_attr_strategies[i]->update_handle_mesh(mesh());
        }
    }

    throw std::runtime_error("unable to find attribute");
}

std::vector<Simplex> Operation::operator()(const Simplex& simplex)
{
    auto scope = mesh().create_scope();
    assert(simplex.primitive_type() == primitive_type());

    if (before(simplex)) {
        auto unmods = unmodified_primitives(simplex);
        auto mods = execute(simplex);
        if (!mods.empty()) { // success should be marked here
            if (after(unmods, mods)) {
                return mods; // scope destructor is called
            }
        }
    }
    scope.mark_failed();
    return {}; // scope destructor is called
}

bool Operation::before(const Simplex& simplex) const
{
    ConstAccessor<long> accessor = hash_accessor();

    if (!mesh().is_valid(simplex.tuple(), accessor)) {
        return false;
    }

    // map simplex to the invariant mesh
    const Mesh& invariant_mesh = m_invariants.mesh();

    // TODO check if this is correct
    const std::vector<Simplex> invariant_simplices = m_mesh.map(invariant_mesh, simplex);

    for (const Simplex& s : invariant_simplices) {
        if (!m_invariants.before(s)) {
            return false;
        }
    }

    return true;
}

bool Operation::after(const std::vector<Simplex>& unmods, const std::vector<Simplex>& mods) const
{
    return m_invariants.directly_modified_after(unmods, mods);
}

void Operation::update_cell_hashes(const std::vector<Tuple>& cells)
{
    Accessor<long> accessor = hash_accessor();

    mesh().update_cell_hashes(cells, accessor);
}

Tuple Operation::resurrect_tuple(const Tuple& tuple) const
{
    return mesh().resurrect_tuple(tuple, hash_accessor());
}

Accessor<long> Operation::hash_accessor()
{
    return m_mesh.get_cell_hash_accessor();
}

ConstAccessor<long> Operation::hash_accessor() const
{
    return m_mesh.get_const_cell_hash_accessor();
}

} // namespace wmtk::operations
