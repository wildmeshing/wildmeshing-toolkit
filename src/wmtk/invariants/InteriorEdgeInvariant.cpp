#include "InteriorEdgeInvariant.hpp"

namespace wmtk {
bool InteriorEdgeInvariant::before(const Tuple& t) override
{
    return mesh().is_boundary(t);
}
} // namespace wmtk
