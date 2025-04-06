#include "Tuple.hpp"

#include <fmt/format.h>

namespace wmtk {

std::string Tuple::as_string() const
{
    return fmt::format(
        "(gid {} : lids[v{},e{},f{}])",
        global_cid(),
        local_vid(),
        local_eid(),
        local_fid());
}

Tuple::operator std::string() const
{
    return as_string();
}

std::ostream& operator<<(std::ostream& os, const Tuple& t)
{
    os << t.as_string();
    return os;
}

} // namespace wmtk