#pragma once
// DO NOT MODIFY, autogenerated from the /scripts directory
#include "autogenerated_tables.hpp"
#include "local_id_table_offset.hpp"
#include "simplex_from_valid_index.hpp"
namespace wmtk::autogen::tet_mesh {
inline int8_t simplex_index_from_valid_index(wmtk::PrimitiveType type, int8_t valid_index)
{
    return auto_valid_tuples[valid_index][get_primitive_type_id(type)];
}
