#include "DEBUG_Tuple.hpp"
#include <spdlog/spdlog.h> // just to access its internal fmt

namespace wmtk::tests {

DEBUG_Tuple::operator std::string() const
{
    return fmt::format(
        "(gid {}:lids{},{},{})",
        global_cid(),
        local_vid(),
        local_eid(),
        local_fid());
}

} // namespace wmtk::tests
