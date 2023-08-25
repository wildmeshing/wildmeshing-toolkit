#include "Operation.hpp"
#include <wmtk/Accessor.hpp>
#include <wmtk/Mesh.hpp>
#include "Operation.hpp"

namespace wmtk::operations {
Operation::Operation(Mesh& m)
    : m_mesh(m)
{}
Operation::~Operation() = default;

bool Operation::operator<(const Operation& o) const
{
    return priority() < o.priority();
}
bool Operation::operator==(const Operation& o) const
{
    return priority() == o.priority();
}

bool Operation::operator()()
{
    auto scope = m_mesh.create_scope();

    if (before()) {
        if (execute()) { // success should be marked here
            if (after()) {
                return true; // scope destructor is called
            }
        }
    }
    scope.mark_failed();
    return false; // scope destructor is called
}

bool Operation::before() const
{
    // TODO: process invariants


    return true;
}
bool Operation::after() const
{
    // TODO: process invariants


    return true;
}

void Operation::update_cell_hash(
    const std::vector<Tuple>& cells,
    std::vector<Tuple>& updated_tuples)
{
    Accessor<long> hash_accessor = m_mesh.get_cell_hash_accessor();
    m_mesh.update_cell_hashes(cells, hash_accessor);

    for (Tuple& t : updated_tuples) {
        t.m_hash = hash_accessor.scalar_attribute(t);
    }
}


} // namespace wmtk::operations
