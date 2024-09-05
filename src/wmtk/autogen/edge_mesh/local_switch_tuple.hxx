#pragma once
#include <cassert>
#include <stdexcept>
#include <wmtk/utils/TupleInspector.hpp>
#include "autogenerated_tables.hpp"
#include "local_id_table_offset.hpp"
#include "tuple_from_valid_index.hpp"
#include "valid_index_from_tuple.hpp"

namespace wmtk::autogen::edge_mesh {
inline Tuple local_switch_tuple(const Tuple& tuple, PrimitiveType pt)
{
    using namespace utils;
    const int64_t global_cid = TupleInspector::global_cid(tuple);
    switch (pt) {
    case PrimitiveType::Vertex:
        return Tuple(
            1 - TupleInspector::local_vid(tuple),
            TupleInspector::local_eid(tuple),
            TupleInspector::local_fid(tuple),
            global_cid);

    case PrimitiveType::Edge:
    case PrimitiveType::Triangle:
    case PrimitiveType::Tetrahedron:
    default: assert(false); // "Tuple switch: Invalid primitive type"
    }
    return Tuple();
}

inline Tuple local_switch_tuple(const Tuple& t, int8_t valid_tuple_index)
{
    int8_t input_index = valid_index_from_tuple(t);
    const int64_t global_cid = wmtk::utils::TupleInspector::global_cid(t);
    const int8_t product_result = auto_valid_switch_product_table[input_index][valid_tuple_index];
    return tuple_from_valid_index(global_cid, product_result);
}

namespace internal {
inline int8_t switch_primitive_to_valid_tuple_index(wmtk::PrimitiveType pt)
{
    return auto_valid_tuple_switch_indices[get_primitive_type_id(pt)];
}
inline int8_t identity_valid_tuple_index()
{
    return switch_primitive_to_valid_tuple_index(wmtk::PrimitiveType::Edge);
}
} // namespace internal
} // namespace wmtk::autogen::edge_mesh
