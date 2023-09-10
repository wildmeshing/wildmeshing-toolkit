#include "Invariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk {
Invariant::~Invariant() = default;
bool Invariant::before(const Tuple& t) const
{
    return true;
}
bool Invariant::after(PrimitiveType type, const std::vector<Tuple>& t) const
{
    return true;
}
} // namespace wmtk
