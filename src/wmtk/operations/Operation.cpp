#include "Operation.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk {
Operation::Operation(Mesh& m)
    : m_mesh(m)
{}
Operation::~Operation() = default;


bool Operation::operator()()
{
    AttributeScopeHandle scope = m_mesh.create_scope();

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

std::vector<double> Operation::priority() const
{
    return {0};
}

bool Operation::after() const
{
    // TODO: default implement the invariants
    /*
    for (const auto& invariant : invariants) {
        if (!invariant(m_mesh, modified_triangles())) {
            return false;
        }
    }
    */
    return true;
}
} // namespace wmtk

