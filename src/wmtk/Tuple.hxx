#include "Tuple.hpp"

#include <cassert>

namespace wmtk {

//         v2
//       /    \ .
//  e1  /      \  e0
//     v0 - - - v1
//         e2

inline Tuple::Tuple(int8_t local_vid, int8_t local_eid, int8_t local_fid, int64_t global_cid)
    : m_global_cid(global_cid)
    , m_local_vid(local_vid)
    , m_local_eid(local_eid)
    , m_local_fid(local_fid)
{}

inline bool Tuple::operator!=(const wmtk::Tuple& t) const
{
    return !(*this == t);
}
inline bool Tuple::operator==(const wmtk::Tuple& t) const
{
    return std::tie(m_local_vid, m_local_eid, m_local_fid, m_global_cid) ==
           std::tie(t.m_local_vid, t.m_local_eid, t.m_local_fid, t.m_global_cid);
}

inline bool Tuple::operator<(const wmtk::Tuple& t) const
{
    return std::tie(m_local_vid, m_local_eid, m_local_fid, m_global_cid) <
           std::tie(t.m_local_vid, t.m_local_eid, t.m_local_fid, t.m_global_cid);
}
inline bool Tuple::same_ids(const wmtk::Tuple& t) const
{
    return std::tie(m_local_vid, m_local_eid, m_local_fid, m_global_cid) ==
           std::tie(t.m_local_vid, t.m_local_eid, t.m_local_fid, t.m_global_cid);
}

inline bool Tuple::is_null() const
{
    return m_global_cid == -1;
}


inline int64_t Tuple::global_cid() const
{
    return m_global_cid;
}

inline int8_t Tuple::local_vid() const
{
    return m_local_vid;
}

inline int8_t Tuple::local_eid() const
{
    return m_local_eid;
}

inline int8_t Tuple::local_fid() const
{
    return m_local_fid;
}

inline int8_t Tuple::local_id(const PrimitiveType pt) const
{
    switch (pt) {
    case PrimitiveType::Vertex: return local_vid();
    case PrimitiveType::Edge: return local_eid();
    case PrimitiveType::Triangle: return local_fid();
    case PrimitiveType::Tetrahedron:
    default: assert(false);
    }
    return -1;
}

} // namespace wmtk
