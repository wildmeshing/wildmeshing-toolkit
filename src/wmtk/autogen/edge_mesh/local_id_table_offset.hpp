#pragma once
#include <array>
#include <wmtk/Tuple.hpp>


namespace wmtk::autogen::edge_mesh {
// computes the offset of a tuple's local ids in the tables
int64_t local_id_table_offset(const Tuple& t);

// returns a lvid/leid associated iwth a particular tuple offset
std::array<int64_t, 1> lvid_from_table_offset(int64_t table_offset);

} // namespace wmtk::autogen::edge_mesh

#include "local_id_table_offset.hxx"
