#pragma once
#include <wmtk/Tuple.hpp>


namespace wmtk::autogen::tri_mesh {
// computes the offset of a tuple's local ids in the tables
long local_id_table_offset(const Tuple& t);

} // namespace wmtk::autogen::tri_mesh

