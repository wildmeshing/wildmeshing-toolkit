#pragma once

#include "autogenerated_tables.hpp"
namespace wmtk::autogen::tri_mesh {
// computes the offset of a tuple's local ids in the tables
inline int64_t local_id_table_offset(const Tuple& tuple)
{
    return tuple.local_vid() * 3 + tuple.local_eid();
}

inline std::array<int64_t, 2> lvid_leid_from_table_offset(int64_t table_offset)
{
    std::array<int64_t, 2> r;
    auto& [lvid, leid] = r;

    lvid = table_offset / 3;
    leid = table_offset % 3;
    return r;
}

} // namespace wmtk::autogen::tri_mesh
