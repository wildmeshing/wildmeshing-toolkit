#pragma once
#include <cassert>
#include <stdexcept>
#include <wmtk/utils/TupleInspector.hpp>

namespace wmtk::autogen::edge_mesh {
inline Tuple local_switch_tuple(const Tuple& tuple, PrimitiveType pt)
{
    using namespace utils;
    const int64_t global_cid = TupleInspector::global_cid(tuple);
    const int64_t hash = TupleInspector::hash(tuple);
    switch (pt) {
    case PrimitiveType::Vertex:
        return Tuple(
            1 - TupleInspector::local_vid(tuple),
            TupleInspector::local_eid(tuple),
            TupleInspector::local_fid(tuple),
            global_cid,
            hash);

    case PrimitiveType::Edge:
    case PrimitiveType::Triangle:
    case PrimitiveType::Tetrahedron:
    default: assert(false); // "Tuple switch: Invalid primitive type"
    }
    return Tuple();
}
} // namespace wmtk::autogen::edge_mesh
