#pragma once
#include <wmtk/Tuple.hpp>


namespace wmtk::autogen::utils {
// NOTE: this is just for helping with autogen accessing tuple intenrals. DO NOT USE ELSEWHERE
struct TupleInspector
{
    static long local_vid(const Tuple& t) { return t.m_local_vid; }
    static long local_eid(const Tuple& t) { return t.m_local_eid; }
    static long local_fid(const Tuple& t) { return t.m_local_fid; }

    static long global_cid(const Tuple& t) { return t.m_global_cid; }
    static long hash(const Tuple& t) { return t.m_hash; }
};
} // namespace wmtk::autogen::utils
