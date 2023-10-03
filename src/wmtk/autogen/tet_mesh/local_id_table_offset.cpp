#include "local_id_table_offset.hpp"
#include <wmtk/utils/TupleInspector.hpp>
#include "autogenerated_tables.hpp"
namespace wmtk::autogen::tet_mesh {
// computes the offset of a tuple's local ids in the tables
long local_id_table_offset(const Tuple& tuple)
{
    using namespace utils;
    return TupleInspector::local_vid(tuple) * 6 * 4 + TupleInspector::local_eid(tuple) * 4 +
           TupleInspector::local_fid(tuple);
}

} // namespace wmtk::autogen::tet_mesh
