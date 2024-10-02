
#include "DEBUG_GlobalTuple.hpp"
#include <spdlog/spdlog.h> // just to access its internal fmt

namespace wmtk::tests {

DEBUG_GlobalTuple::operator std::string() const
{
    return fmt::format(
        "(gid {}:lids{},{},{}:hash{})",
        cid(),
        vid(),
        eid(),
        fid(),
        hash());
}

} // namespace wmtk::tests
