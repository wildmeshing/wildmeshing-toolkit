#pragma once
#include <string>
#include <wmtk/Tuple.hpp>


namespace wmtk::utils {
// NOTE: this is just for helping with autogen accessing tuple intenrals. DO NOT USE ELSEWHERE
class TupleInspector
{
public:
    static int64_t local_vid(const Tuple& t) { return t.m_local_vid; }
    static int64_t local_eid(const Tuple& t) { return t.m_local_eid; }
    static int64_t local_fid(const Tuple& t) { return t.m_local_fid; }

    static int64_t global_cid(const Tuple& t) { return t.m_global_cid; }
    static int64_t hash(const Tuple& t) { return t.m_hash; }
    static std::string as_string(const Tuple& t);
};
} // namespace wmtk::utils
