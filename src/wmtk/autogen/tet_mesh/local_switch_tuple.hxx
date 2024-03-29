#pragma once
#include <cassert>
#include <stdexcept>
#include <wmtk/utils/TupleInspector.hpp>
#include "autogenerated_tables.hpp"
#include "local_id_table_offset.hpp"


namespace wmtk::autogen::tet_mesh {
inline Tuple local_switch_tuple(const Tuple& tuple, PrimitiveType pt)
{
    using namespace utils;
    const int64_t offset = local_id_table_offset(tuple);


    const int64_t global_cid = TupleInspector::global_cid(tuple);
    const int64_t hash = TupleInspector::hash(tuple);
    switch (pt) {
    case PrimitiveType::Vertex:
        return Tuple(
            auto_3d_table_vertex[offset][0],
            auto_3d_table_vertex[offset][1],
            auto_3d_table_vertex[offset][2],
            global_cid,
            hash);

    case PrimitiveType::Edge:
        return Tuple(
            auto_3d_table_edge[offset][0],
            auto_3d_table_edge[offset][1],
            auto_3d_table_edge[offset][2],
            global_cid,
            hash);
    case PrimitiveType::Triangle:
        return Tuple(
            auto_3d_table_face[offset][0],
            auto_3d_table_face[offset][1],
            auto_3d_table_face[offset][2],
            global_cid,
            hash);

    case PrimitiveType::Tetrahedron:
    default: assert(false); // "Tuple switch: Invalid primitive type"
    }
    return Tuple();
}
} // namespace wmtk::autogen::tet_mesh
