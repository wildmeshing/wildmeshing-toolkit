
#include "Tuple.hpp"
#include <cstddef>
#include <iostream>
#include <optional>
#include <string>
#include <tuple>

namespace wmtk {
Tuple::Tuple(
    int64_t local_vid,
    int64_t local_eid,
    int64_t local_fid,
    int64_t global_cid,
    int64_t hash)
    : m_local_vid(local_vid)
    , m_local_eid(local_eid)
    , m_local_fid(local_fid)
    , m_global_cid(global_cid)
    , m_hash(hash)
{}

//         v2
//       /    \ .
//  e1  /      \  e0
//     v0 - - - v1
//         e2

Tuple::Tuple() = default;
Tuple::Tuple(const Tuple& other) = default;
Tuple::Tuple(Tuple&& other) = default;
Tuple& Tuple::operator=(const Tuple& other) = default;
Tuple& Tuple::operator=(Tuple&& other) = default;

bool Tuple::operator!=(const wmtk::Tuple& t) const
{
    return !(*this == t);
}
bool Tuple::operator==(const wmtk::Tuple& t) const
{
    return std::tie(m_local_vid, m_local_eid, m_local_fid, m_global_cid, m_hash) ==
           std::tie(t.m_local_vid, t.m_local_eid, t.m_local_fid, t.m_global_cid, t.m_hash);
}

bool Tuple::operator<(const wmtk::Tuple& t) const
{
    return std::tie(m_local_vid, m_local_eid, m_local_fid, m_global_cid, m_hash) <
           std::tie(t.m_local_vid, t.m_local_eid, t.m_local_fid, t.m_global_cid, t.m_hash);
}
bool Tuple::same_ids(const wmtk::Tuple& t) const
{
    return std::tie(m_local_vid, m_local_eid, m_local_fid, m_global_cid) ==
           std::tie(t.m_local_vid, t.m_local_eid, t.m_local_fid, t.m_global_cid);
}

bool Tuple::is_null() const
{
    return m_local_vid == -1 && m_local_eid == -1 && m_local_fid == -1 && m_global_cid == -1 &&
           m_hash == -1;
}

Tuple Tuple::with_updated_hash(int64_t new_hash) const
{
    return Tuple(m_local_vid, m_local_eid, m_local_fid, m_global_cid, new_hash);
}

} // namespace wmtk
