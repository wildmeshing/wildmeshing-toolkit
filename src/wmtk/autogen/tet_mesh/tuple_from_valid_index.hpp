#pragma once
#include <wmtk/Tuple.hpp>

namespace wmtk::autogen::tet_mesh {
Tuple tuple_from_valid_index(int64_t global_cid, int8_t valid_tuple_index);
Tuple tuple_from_valid_index(int64_t global_cid, int8_t valid_tuple_index, int64_t hash);

}

#include "tuple_from_valid_index.hxx"
