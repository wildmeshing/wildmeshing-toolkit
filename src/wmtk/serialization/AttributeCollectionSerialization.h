#pragma once
#include <wmtk/serialization/Hdf5Utils.hpp>


// Attribute Change tables:
// Given an attribute named ATTR
//
// ATTR_value_changes
// stores changes to an attribute
// ============
// Type: AttributeCollectionValueChange<T>
// size_t index;
// T old_value;
// T new_value;
//
// ATTR_updates
// AttributeCollectionUpdate
// Stores the changes associated with a single "operation"
// Assumes ATTR_value_changes stores values consecutively
// ============
// size_t old_size;
// size_t new_size;
// AttributeCollectionRange range; <-- points into ATTR_value_changes
//

//
// The attribute update tables use columns [index, old_value, new_value], where
// the index is the index update in some pertinent AttributeCollection and
// the latter two columns are customized for each attribute type.
//
//
// The Index ranges use columns [attribute_name, range_begin,range_end]
// If we let Data[attribute_name] point to the attribute update table 'attribute_name'
// Then in pythonic-slice notation we see that the updates to 'attribute_name' are stored by
// Data[attribute_name][range_begin:range_end] .
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
#define WMTK_HDF5_REGISTER_ATTRIBUTE_TYPE(type)     \
    HIGHFIVE_REGISTER_TYPE(                         \
        wmtk::AttributeCollectionValueChange<type>, \
        wmtk::AttributeCollectionValueChange<type>::datatype)

#define WMTK_HDF5_DECLARE_ATTRIBUTE_TYPE(type) \
    WMTK_HIGHFIVE_DECLARE_TYPE(wmtk::AttributeCollectionValueChange<type>)

namespace wmtk {

template <typename T>
struct AttributeCollectionValueChange;
struct AttributeCollectionUpdate;

class AttributeCollectionRecorder;
class AttributeCollectionReplayer;

class AbstractAttributeCollection;
template <typename T>
class AttributeCollection;

struct AttributeCollectionRange
{
    size_t begin = 0;
    size_t end = 0;
    static HighFive::CompoundType datatype();
};

// internal Interface for adding an attribute to a logged hdf5 file
// this is intended as an internal interface because Recording should only append while Replaying
// should allow for forward and backward playback. Please look at AttributeCollectionRecorder and
// AttributeCollectionReplayer for these functionalities. the HDF5 interactions are, as much as
// possible, shoved into the AttributeCollectionSerializationBase class with implementation in the
// related cpp file whereas the type-specific details are defined with
// AttributeCollectionSerialization<T> and implemeneted at the bottom of this file.
//
class AttributeCollectionSerializationBase
{
protected:
    friend class AttributeCollectionRecorder;
    friend class AttributeCollectionReplayer;
    // the file being serialized to, the name of the attribute, and information on how the data
    // should be serialized
    AttributeCollectionSerializationBase(
        HighFive::File& file,
        const std::string& name,
        const HighFive::DataType& data_type);
    AttributeCollectionSerializationBase(
        const std::string& name,
        HighFive::DataSet&& value_changes_ds,
        HighFive::DataSet&& updates_ds);

public:
    virtual ~AttributeCollectionSerializationBase();
    static HighFive::CompoundType record_datatype();
    static HighFive::DataSetCreateProps create_properties();
    static HighFive::DataSetAccessProps access_properties();


    // the number of updates serialized
    size_t changes_size() const;
    // the number of updates serialized
    size_t updates_size() const;
    const std::string& name() const;

    AttributeCollectionUpdate update(size_t index) const;

protected:
    virtual AttributeCollectionUpdate record_value_changes() = 0;
    virtual AttributeCollectionUpdate record_entire_state() = 0;
    // returns the index of the recorded changes in the m_updates_dataset
    size_t record();
    size_t record_initial_state();

    // load a particular set of attribute changes from a particular dataset
    virtual void apply_update(const AttributeCollectionUpdate& update) = 0;

    // undoes a particular change to an attribute
    virtual void unapply_update(const AttributeCollectionUpdate& update) = 0;


    void load(size_t index);
    void unload(size_t index);

    virtual AbstractAttributeCollection& abstract_attribute_collection() = 0;


    std::string m_name;
    HighFive::DataSet m_value_changes_dataset;
    HighFive::DataSet m_updates_dataset;
};

template <typename T>
struct AttributeCollectionValueChange
{
    size_t index;
    T old_value;
    T new_value;
    static HighFive::CompoundType datatype();
};

// Class capable of recording updates to a single attribute
template <typename T>
class AttributeCollectionSerialization : public AttributeCollectionSerializationBase
{
public:
    using UpdateData = AttributeCollectionValueChange<T>;
    static HighFive::CompoundType datatype();
    AttributeCollectionSerialization(
        HighFive::File& file,
        const std::string& name,
        AttributeCollection<T>& attr_);

    AbstractAttributeCollection& abstract_attribute_collection() override
    {
        return this->attribute_collection;
    }

    AttributeCollectionUpdate record_value_changes() override;
    AttributeCollectionUpdate record_entire_state() override;
    // load a particular set of attribute changes from a particular dataset. loading backwards
    void apply_update(const AttributeCollectionUpdate& update) override;

    // undoes a particular change to an attribute
    void unapply_update(const AttributeCollectionUpdate& update) override;
    using AttributeCollectionSerializationBase::record;

private:
    std::vector<AttributeCollectionValueChange<T>> get_value_changes(
        const AttributeCollectionUpdate& update) const;

private:
    AttributeCollection<T>& attribute_collection;
};


// Indicates which attribute was changed and the range of updates that are associated with its
// updates in the per-attribute update table
struct AttributeCollectionUpdate
{
    AttributeCollectionUpdate() = default;
    AttributeCollectionUpdate(AttributeCollectionUpdate&&) = default;
    AttributeCollectionUpdate(AttributeCollectionUpdate const&) = default;
    AttributeCollectionUpdate& operator=(AttributeCollectionUpdate const&) = default;
    AttributeCollectionUpdate& operator=(AttributeCollectionUpdate&&) = default;

    AttributeCollectionUpdate(size_t begin, size_t end, size_t old_size, size_t size);

    size_t old_size = 0;
    size_t new_size = 0;
    AttributeCollectionRange range;

    static HighFive::CompoundType datatype();
};

// deduction guide
template <typename T>
AttributeCollectionSerialization(
    HighFive::File& file,
    const std::string& name,
    const AttributeCollection<T>&) -> AttributeCollectionSerialization<T>;

template <typename T>
AttributeCollectionSerialization<T>::AttributeCollectionSerialization(
    HighFive::File& file,
    const std::string& name,
    AttributeCollection<T>& attr_)
    : AttributeCollectionSerializationBase(file, name, datatype())
    , attribute_collection(attr_)
{}

template <typename T>
HighFive::CompoundType AttributeCollectionSerialization<T>::datatype()
{
    return HighFive::CompoundType{
        {"index", HighFive::create_datatype<size_t>()},
        {"old_value", HighFive::create_datatype<T>()},
        {"new_value", HighFive::create_datatype<T>()}};
}


template <typename T>
AttributeCollectionUpdate AttributeCollectionSerialization<T>::record_value_changes()
{
    assert(attribute_collection.is_in_protect());
    const std::map<size_t, T>& rollback_list = attribute_collection.m_rollback_list.local();
    const size_t old_size = attribute_collection.m_rollback_size.local().value();
    const auto& attributes = attribute_collection.m_attributes;
    // const tbb::concurrent_vector<T>& attributes = attribute_collection.m_attributes;

    std::vector<UpdateData> data;
    size_t start, end;
    if (rollback_list.size() > 0) {
        data.reserve(rollback_list.size());
        std::transform(
            rollback_list.begin(),
            rollback_list.end(),
            std::back_inserter(data),
            [&attributes](const std::pair<const size_t, T>& pr) -> UpdateData {
                const auto& [index, old_value] = pr;
                if (index < attributes.size()) {
                    const T& new_value = attributes[index];
                    return UpdateData{index, old_value, new_value};
                } else {
                    return UpdateData{index, old_value, {}};
                }
            });

        auto [s, e] = utils::append_values_to_dataset(m_value_changes_dataset, data);
        start = s;
        end = e;
    } else {
        start = 0;
        end = 0;
    }
    return AttributeCollectionUpdate(start, end, old_size, attribute_collection.size());
    // return std::array<size_t, 3>{{start, end, attribute_collection.m_rollback_size.local(),
    // attribute_collection.size()}};
}

template <typename T>
AttributeCollectionUpdate AttributeCollectionSerialization<T>::record_entire_state()
{
    const std::map<size_t, T>& rollback_list = attribute_collection.m_rollback_list.local();
    const size_t old_size = attribute_collection.m_rollback_size.local().value_or(attribute_collection.size());
    const auto& attributes = attribute_collection.m_attributes;
    // const tbb::concurrent_vector<T>& attributes = attribute_collection.m_attributes;


    std::vector<UpdateData> data;
    data.reserve(attribute_collection.size());
    auto& attr_data = attribute_collection.m_attributes;
    for (size_t index = 0; index < attr_data.size(); ++index) {
        const T& new_value = attr_data[index];
        data.emplace_back(UpdateData{index, T{}, new_value});
    }

    auto [start, end] = utils::append_values_to_dataset(m_value_changes_dataset, data);
    return AttributeCollectionUpdate(start, end, old_size, attribute_collection.size());
    // return std::array<size_t, 3>{{start, end, attribute_collection.m_rollback_size.local(),
    // attribute_collection.size()}};
}

template <typename T>
void AttributeCollectionSerialization<T>::apply_update(const AttributeCollectionUpdate& update)
{
    // resize the AC to the new size
    attribute_collection.resize(update.new_size);


    // write data
    for (const AttributeCollectionValueChange<T>& upd : get_value_changes(update)) {
        if (upd.index < attribute_collection.size()) {
            attribute_collection[upd.index] = upd.new_value;
        }
    }
}
template <typename T>
void AttributeCollectionSerialization<T>::unapply_update(const AttributeCollectionUpdate& update)
{
    attribute_collection.resize(update.old_size);

    for (const AttributeCollectionValueChange<T>& upd : get_value_changes(update)) {
        attribute_collection[upd.index] = upd.old_value;
    }
}
template <typename T>
std::vector<AttributeCollectionValueChange<T>>
AttributeCollectionSerialization<T>::get_value_changes(
    const AttributeCollectionUpdate& update) const
{
    // the update data
    std::vector<AttributeCollectionValueChange<T>> value_changes;

    // compute hte parts of hte update data we want to read
    std::vector<size_t> start, size;
    start.emplace_back(update.range.begin);
    size.emplace_back(update.range.end - update.range.begin);
    if (size[0] == 0) {
        return value_changes;
    }

    // read the data
    m_value_changes_dataset.select(start, size).read(value_changes);
    return value_changes;
}


template <typename T>
HighFive::CompoundType AttributeCollectionValueChange<T>::datatype()
{
    return HighFive::CompoundType{
        {"index", HighFive::create_datatype<size_t>()},
        {"old_value", HighFive::create_datatype<T>()},
        {"new_value", HighFive::create_datatype<T>()}};
}
} // namespace wmtk


WMTK_HDF5_DECLARE_ATTRIBUTE_TYPE(wmtk::AttributeCollectionRange)
