#pragma once
#include <cassert>
#include <string>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/Tuple.hpp>


namespace wmtk::utils {
// NOTE: this is just for helping with autogen accessing tuple intenrals. DO NOT USE ELSEWHERE
class TupleInspector
{
public:
    static int8_t local_vid(const Tuple& t) { return t.m_local_vid; }
    static int8_t local_eid(const Tuple& t) { return t.m_local_eid; }
    static int8_t local_fid(const Tuple& t) { return t.m_local_fid; }
    static int8_t local_id(const Tuple& t, const PrimitiveType pt)
    {
        switch (pt) {
        case PrimitiveType::Vertex: return local_vid(t);
        case PrimitiveType::Edge: return local_eid(t);
        case PrimitiveType::Triangle: return local_fid(t);
        case PrimitiveType::Tetrahedron:
        default: assert(false);
        }
        return -1;
    }

    static int8_t local_id(const PrimitiveType pt, const Tuple& t)
    {
        switch (pt) {
        case PrimitiveType::Triangle: return local_fid(t);
        case PrimitiveType::Edge: return local_eid(t);
        case PrimitiveType::Vertex: return local_vid(t);
        case PrimitiveType::Tetrahedron: assert(false);
        default: return -1;
        }
    }

    static int64_t global_cid(const Tuple& t) { return t.m_global_cid; }
    static int64_t global_id(const Mesh& m, const Tuple& t, PrimitiveType pt)
    {
        return m.id(pt, t);
    }
    static std::string as_string(const Tuple& t);
};
} // namespace wmtk::utils
