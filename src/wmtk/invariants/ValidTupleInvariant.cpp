#include "ValidTupleInvariant.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/Mesh.hpp>

namespace wmtk {
bool ValidTupleInvariant::before(const Tuple& t) const
{
    const bool result = mesh().is_valid(t) && !mesh().is_outdated(t);
    spdlog::info("invar:checking valid tuple!: {}", result);
    return result;
}
} // namespace wmtk
