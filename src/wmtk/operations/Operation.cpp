#include "Operation.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk {
Operation::Operation(Mesh& m)
    : m_mesh(m)
{}
Operation::~Operation() = default;


bool Operation::operator()()
{
    auto scope = m_mesh.create_scope();

    if (before()) {
        if (execute()) { // success should be marked here
            if (after()) {
                return true;
            }
        }
    }
    scope.mark_failed();
    return false;
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


} // namespace wmtk

