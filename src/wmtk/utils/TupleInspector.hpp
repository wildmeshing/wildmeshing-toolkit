#pragma once
#include <wmtk/Tuple.hpp>
#include <string>


namespace wmtk::utils {
// NOTE: this is just for helping with autogen accessing tuple intenrals. DO NOT USE ELSEWHERE
struct TupleInspector
{
    static long local_vid(const Tuple& t) { return t.m_local_vid; }
    static long local_eid(const Tuple& t) { return t.m_local_eid; }
    static long local_fid(const Tuple& t) { return t.m_local_fid; }

    static long global_cid(const Tuple& t) { return t.m_global_cid; }
    static long hash(const Tuple& t) { return t.m_hash; }
    static std::string as_string(const Tuple& t);
};
} // namespace wmtk::autogen::utils
