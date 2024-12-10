#include "TupleInspector.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::utils {
static int64_t TupleInspector::global_id(const Mesh& m, const Tuple& t, PrimitiveType pt)
{
    return m.id(t, pt);
}
std::string TupleInspector::as_string(const Tuple& t)
{
    return fmt::format(
        "(gid {}:lids[v{},e{},f{}])",
        global_cid(t),
        local_vid(t),
        local_eid(t),
        local_fid(t));
}
} // namespace wmtk::utils
