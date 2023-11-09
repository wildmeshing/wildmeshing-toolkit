#pragma once
#include <array>
#include <wmtk/Tuple.hpp>


namespace wmtk::autogen::tri_mesh {
// computes the offset of a tuple's local ids in the tables
long local_id_table_offset(const Tuple& t);

// returns a lvid/leid associated iwth a particular tuple offset
std::array<long, 2> lvid_leid_from_table_offset(long table_offset);

} // namespace wmtk::autogen::tri_mesh

