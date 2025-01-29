#pragma once
#include <cassert>
#include <string>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/Tuple.hpp>


namespace wmtk::utils {
// NOTE: this is just for helping with autogen accessing tuple internals. DO NOT USE ELSEWHERE
class TupleInspector
{
public:
    static int8_t local_id(const Tuple& t, const PrimitiveType pt)
    {
        switch (pt) {
        case PrimitiveType::Vertex: return t.local_vid();
        case PrimitiveType::Edge: return t.local_eid();
        case PrimitiveType::Triangle: return t.local_fid();
        case PrimitiveType::Tetrahedron:
        default: assert(false);
        }
        return -1;
    }

    static int8_t local_id(const PrimitiveType pt, const Tuple& t)
    {
        switch (pt) {
        case PrimitiveType::Vertex: return t.local_vid();
        case PrimitiveType::Edge: return t.local_eid();
        case PrimitiveType::Triangle: return t.local_fid();
        case PrimitiveType::Tetrahedron:
        default: assert(false);
        }
        return -1;
    }

    static std::string as_string(const Tuple& t);
};
} // namespace wmtk::utils
