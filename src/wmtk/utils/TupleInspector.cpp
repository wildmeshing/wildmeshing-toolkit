#include "TupleInspector.hpp"
#include <spdlog/spdlog.h>
namespace wmtk::utils {
    std::string TupleInspector::as_string(const Tuple& t) {
    return fmt::format(
        "(gid {}:lids{},{},{}:hash{})",
        global_cid(t),
        local_vid(t),
        local_eid(t),
        local_fid(t),
        hash(t));
    }
}
