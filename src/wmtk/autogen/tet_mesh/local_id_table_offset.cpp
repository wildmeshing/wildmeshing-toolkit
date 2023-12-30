#include "local_id_table_offset.hpp"
#include <wmtk/utils/TupleInspector.hpp>
#include "autogenerated_tables.hpp"
namespace wmtk::autogen::tet_mesh {
// computes the offset of a tuple's local ids in the tables
int64_t local_id_table_offset(const Tuple& tuple)
{
    using namespace utils;
    int64_t value = TupleInspector::local_vid(tuple) * 6 * 4 +
                    TupleInspector::local_eid(tuple) * 4 + TupleInspector::local_fid(tuple);


    // value = (TupleInspector::local_vid(tuple) * 6 + TupleInspector::local_eid(tuple)) * 4 +
    //        TupleInspector::local_fid(tuple);
    return value;
}

std::array<int64_t, 3> lvid_leid_lfid_from_table_offset(int64_t table_offset)
{
    std::array<int64_t, 3> r;
    auto& [lvid, leid, lfid] = r;
    lfid = table_offset % 4;

    int64_t ve_offset = table_offset / 4;
    leid = ve_offset % 6;
    lvid = ve_offset / 6;
    return r;
}

} // namespace wmtk::autogen::tet_mesh
