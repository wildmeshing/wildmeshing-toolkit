#include "Operation.hpp"


namespace wmtk {
Operation::Operation(Mesh& m)
    : m_mesh(m)
{}
Operation::~Operation() = default;


bool Operation::operator()()
{
    auto scope = m.create_scope();

    if (before()) {
        if (execute()) { // success should be marked here
            if (after()) {
                return true;
            }
        }
    }
    scope->clear();
    return false;
}

std::vector<double> Operation::priority() const
{
    return {0};
}

} // namespace wmtk
