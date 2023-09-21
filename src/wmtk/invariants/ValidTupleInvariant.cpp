#include "ValidTupleInvariant.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/Mesh.hpp>

namespace wmtk {
bool ValidTupleInvariant::before(const Tuple& t) const
{
    const bool result = mesh().is_valid_slow(t);
    return result;
}
} // namespace wmtk
