#pragma once
#include <cassert>
#include <wmtk/Tuple.hpp>
#include "autogenerated_tables.hpp"
#if !defined(_NDEBUG)
#include "is_ccw.hpp"
#endif

namespace wmtk::autogen::edge_mesh {

inline Tuple get_tuple_from_simplex_local_vertex_id(int8_t local_id, int64_t global_id = 0)
{
    assert(local_id >= 0);
    assert(local_id < 2);
    return Tuple(local_id, -1, -1, global_id);
}
inline Tuple
get_tuple_from_simplex_local_id(PrimitiveType pt, int8_t local_id, int64_t global_fid = 0)
{
    switch (pt) {
    case PrimitiveType::Vertex: return get_tuple_from_simplex_local_vertex_id(local_id, global_fid);
    case PrimitiveType::Edge:
    case PrimitiveType::Triangle:
    default:
    case PrimitiveType::Tetrahedron: assert(false); return {};
    }
}
} // namespace wmtk::autogen::edge_mesh
