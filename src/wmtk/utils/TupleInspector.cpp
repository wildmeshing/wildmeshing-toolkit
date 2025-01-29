#include "TupleInspector.hpp"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::utils {
std::string TupleInspector::as_string(const Tuple& t)
{
    return fmt::format(
        "(gid {}:lids[v{},e{},f{}])",
        t.global_cid(),
        t.local_vid(),
        t.local_eid(),
        t.local_fid());
}
} // namespace wmtk::utils
