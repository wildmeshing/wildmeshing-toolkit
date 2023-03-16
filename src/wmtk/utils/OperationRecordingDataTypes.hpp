#pragma once
#include <wmtk/TriMesh.h>
#include <wmtk/utils/AttributeCollectionRecorder.h>
#include <wmtk/utils/Hdf5Utils.h>
#include <ostream>
#include <string_view>
#include <wmtk/utils/Logger.hpp>

namespace wmtk {


// The actions of an operator are defined by a sequence of commands and how those commands affect
// attributes. They are tehrefore serialized through 2 + N tables of data, where N is the number of
// attributes 1...N) each table containing all attribute updates for a single attribute, organized
// implicitly by attribute and which command
//        each table is named by the attribute name
// N+1) A table of index ranges into the attribute update table, organized by which attribute and
// (implicitly) which command N+2) A ordered table of commands + references to ranges of attribute
// update groups

// stores the update data for a single operation as well as a range into the table of per-attribute
// updates
//
struct TriMeshOperationData
{
    // TODO: operation_name could be mapped to an enum at some point
    char name[20];
    TriMeshTupleData input_tuple;
    TriMeshTupleData output_tuple;
    AttributeCollectionRange update_range;
    size_t vertex_size = 0;
    size_t triangle_size = 0;

    static HighFive::CompoundType datatype();
};


} // namespace wmtk
