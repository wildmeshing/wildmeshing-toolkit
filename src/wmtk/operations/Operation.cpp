#include "Operation.hpp"
#include <wmtk/Accessor.hpp>
#include <wmtk/Mesh.hpp>
#include "Operation.hpp"

namespace wmtk::operations {
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
    auto scope = base_mesh().create_scope();

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

void Operation::update_cell_hashes(const std::vector<Tuple>& cells)
{
    base_mesh().update_cell_hashes(cells, hash_accessor());
}

Tuple Operation::resurrect_tuple(const Tuple& tuple) const
{
    return base_mesh().resurrect_tuple(tuple, hash_accessor());
}

Accessor<long> Operation::get_hash_accessor(Mesh& m)
{
    return m.get_cell_hash_accessor();
}
const Accessor<long>& Operation::hash_accessor() const
{
    return const_cast<Operation*>(this)->hash_accessor();
}

} // namespace wmtk::operations
