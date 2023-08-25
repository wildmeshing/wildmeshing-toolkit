#include "Operation.hpp"
#include <wmtk/Accessor.hpp>
#include <wmtk/Mesh.hpp>
#include "Operation.hpp"

namespace wmtk::operations {
Operation::Operation(Mesh& m)
    : m_mesh(m)
    , m_hash_accessor(m_mesh.get_cell_hash_accessor())
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

void Operation::update_cell_hash(const std::vector<Tuple>& cells)
{
    m_mesh.update_cell_hashes(cells, m_hash_accessor);
}

Tuple Operation::resurrect_tuple(const Tuple& tuple)
{
    return m_mesh.resurrect_tuple(tuple, m_hash_accessor);
}


} // namespace wmtk::operations
