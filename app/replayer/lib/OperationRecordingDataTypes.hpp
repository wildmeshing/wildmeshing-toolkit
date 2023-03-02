#pragma once
#include <wmtk/TriMesh.h>
#include <wmtk/utils/AttributeRecorder.h>
#include <wmtk/utils/Hdf5Utils.h>
#include <wmtk/utils/OperationLogger.h>
#include <ostream>
#include <string_view>
#include <wmtk/utils/Logger.hpp>

namespace wmtk {

// Helper for adding values to a dataset holding a 1d vector
template <typename T, typename Allocator>
std::array<size_t, 2> append_values_to_1d_dataset(
    HighFive::DataSet& dataset,
    const std::vector<T, Allocator>& data);

// Helper for adding a single value to a dataset holding a 1d vector
template <typename T>
size_t append_value_to_1d_dataset(HighFive::DataSet& dataset, const T& value);

// The actions of an operator are defined by a sequence of commands and how those commands affect
// attributes. They are tehrefore serialized through 2 + N tables of data, where N is the number of
// attributes 1...N) each table containing all attribute updates for a single attribute, organized
// implicitly by attribute and which command
//        each table is named by the attribute name
// N+1) A table of index ranges into the attribute update table, organized by which attribute and
// (implicitly) which command N+2) A ordered table of commands + references to ranges of attribute
// update groups
//
// The attribute update tables use columns [index, old_value, new_value], where
// the index is the index update in some pertinent AttributeCollection and
// the latter two columns are customized for each attribute type.
//
//
// The Index ranges use columns [attribute_name, change_range_begin,change_range_end]
// If we let Data[attribute_name] point to the attribute update table 'attribute_name'
// Then in pythonic-slice notation we see that the updates to 'attribute_name' are stored by
// Data[attribute_name][change_range_begin:change_range_end] .
// This intermediate table is to allow us to track updates of different attributes/types.
//
//
// Finally, the table of commands currently is particular to TriMesh and
// stores [command_name, triangle_id, local_edge_id, vertex_id, update_range_begin,
// update_range_end] where the first 4 entries define a TriMesh command and the latter two specify
// ranges in the previous attribute indexing table to help specify which attributes and which types
// exist.
//


// For any new Attribute type we want to record we have to register the type
// using HighFive's HIGHFIVE_REGISTER_TYPE and then declare its attribute type.
// For example, if we want to record a AttributeCollection<Vec>
// Where Vec is
// struct Vec { double x,y; };
// Then we must declare:
//
// HighFive::CompoundType vec_datatype() {
//      return {
//              {"x", create_datatype<double>()},
//              {"y", create_datatype<double>()}
//      };
// }
//
// and then outside of any namespace's scope we declare the following two lines
// HIGHFIVE_REGISTER_TYPE(Vec, vec_datatype)
// WMTK_HDF5_REGISTER_ATTRIBUTE_TYPE(Vec)
#define WMTK_HDF5_REGISTER_ATTRIBUTE_TYPE(type)              \
    HIGHFIVE_REGISTER_TYPE(                                  \
        wmtk::AttributeCollectionRecorder<type>::UpdateData, \
        wmtk::AttributeCollectionRecorder<type>::datatype)   \
    HIGHFIVE_REGISTER_TYPE(                                  \
        wmtk::AttributeUpdateData<type>, \
        wmtk::AttributeUpdateData<type>::datatype)

#define WMTK_HDF5_DECLARE_ATTRIBUTE_TYPE(type)               \
    HIGHFIVE_REGISTER_TYPE(                                  \
        wmtk::AttributeCollectionRecorder<type>::UpdateData, \
        wmtk::AttributeCollectionRecorder<type>::datatype)

template <typename T>
struct AttributeUpdateData
{
    size_t index;
    T old_value;
    T new_value;
    static HighFive::CompoundType datatype();
};


// Indicates which attribute was changed and the range of updates that are associated with its
// updates in the per-attribute update table
struct AttributeChanges
{
    AttributeChanges() = default;
    AttributeChanges(AttributeChanges&&) = default;
    AttributeChanges(AttributeChanges const&) = default;
    AttributeChanges& operator=(AttributeChanges const&) = default;
    AttributeChanges& operator=(AttributeChanges&&) = default;

    AttributeChanges(const std::string_view& view, size_t begin, size_t end, size_t size);
    char name[20];

    size_t attribute_size = 0;
    size_t change_range_begin = 0;
    size_t change_range_end = 0;

    static HighFive::CompoundType datatype();
};

inline AttributeChanges::AttributeChanges(
    const std::string_view& view,
    size_t begin,
    size_t end,
    size_t size)
    : attribute_size(size)
    , change_range_begin(begin)
    , change_range_end(end)
{
    strncpy(
        name,
        view.data(),
        sizeof(name) / sizeof(char)); // yes sizeof(char)==1, maybe chartype changes someday?
}

struct TriMeshTupleData
{
    TriMeshTupleData() = default;
    TriMeshTupleData(TriMeshTupleData&&) = default;
    TriMeshTupleData(const TriMeshTupleData&) = default;
    TriMeshTupleData& operator=(TriMeshTupleData&&) = default;
    TriMeshTupleData& operator=(const TriMeshTupleData&) = default;
    TriMeshTupleData(const TriMesh::Tuple& tuple);
    size_t triangle_id = 0;
    size_t local_edge_id = 0;
    size_t vertex_id = 0;
    static HighFive::CompoundType datatype();
};

// stores the update data for a single operation as well as a range into the table of per-attribute
// updates
//
struct TriMeshOperationData
{
    // TODO: operation_name could be mapped to an enum at some point
    char name[20];
    TriMeshTupleData input_tuple;
    TriMeshTupleData output_tuple;
    size_t update_range_begin = 0;
    size_t update_range_end = 0;
    size_t vertex_size = 0;
    size_t triangle_size = 0;

    static HighFive::CompoundType datatype();
};

template <typename T>
HighFive::CompoundType AttributeUpdateData<T>::datatype()
{
    return HighFive::CompoundType{
        {"index", HighFive::create_datatype<size_t>()},
        {"old_value", HighFive::create_datatype<T>()},
        {"new_value", HighFive::create_datatype<T>()}};
}


template <typename T, typename Allocator>
std::array<size_t, 2> append_values_to_1d_dataset(
    HighFive::DataSet& dataset,
    const std::vector<T, Allocator>& data)
{
    // compute the new dataset size
    std::vector<size_t> first_position = dataset.getDimensions();
    // double check that the dataset is 1-dimensional
    assert(first_position.size() == 1);
    std::vector<size_t> new_size{first_position[0] + data.size()};
    std::vector<size_t> count{data.size()};

    // resize the datset to the right size
    dataset.resize(new_size);

    dataset.select(first_position, count).write(data);
    return std::array<size_t, 2>{{first_position[0], new_size[0]}};
}

template <typename T>
size_t append_value_to_1d_dataset(HighFive::DataSet& dataset, const T& value)
{
    // compute the new dataset size
    std::vector<size_t> first_position = dataset.getDimensions();
    // double check that the dataset is 1-dimensional
    assert(first_position.size() == 1);
    std::vector<size_t> new_size{first_position[0] + 1};
    std::vector<size_t> count{1};

    // resize the datset to the right size
    dataset.resize(new_size);

    dataset.select(first_position, count).write_raw(&value);
    return new_size[0];
}
} // namespace wmtk
