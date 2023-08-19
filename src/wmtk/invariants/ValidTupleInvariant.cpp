#include "ValidTupleInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk {
bool ValidTupleInvariant::before(const Tuple& t) const
{
    return mesh().is_valid(t) && mesh().is_outdated(t);
}
} // namespace wmtk
