#include <wmtk/serialization/AttributeCollectionSerialization.h>
#include <wmtk/utils/Logger.hpp>

HIGHFIVE_REGISTER_TYPE(wmtk::AttributeCollectionRange, wmtk::AttributeCollectionRange::datatype);
using namespace wmtk;
HIGHFIVE_REGISTER_TYPE(wmtk::AttributeCollectionUpdate, wmtk::AttributeCollectionUpdate::datatype);


HighFive::CompoundType AttributeCollectionUpdate::datatype()
{
    return HighFive::CompoundType{
        {"old_size", HighFive::create_datatype<size_t>()},
        {"new_size", HighFive::create_datatype<size_t>()},
        {"range", HighFive::create_datatype<AttributeCollectionRange>()}};
}

HighFive::CompoundType AttributeCollectionRange::datatype()
{
    return HighFive::CompoundType{
        {"begin", HighFive::create_datatype<size_t>()},
        {"end", HighFive::create_datatype<size_t>()}};
}

AttributeCollectionSerializationBase::AttributeCollectionSerializationBase(
    HighFive::File& file,
    const std::string& name,
    const HighFive::DataType& data_type)
    : AttributeCollectionSerializationBase(
          name,
          utils::create_extendable_dataset(file, name + "_value_changes", data_type),
          utils::create_extendable_dataset(
              file,
              name + "_update",
              AttributeCollectionUpdate::datatype()))
{}

const std::string& AttributeCollectionSerializationBase::name() const
{
    return m_name;
}

AttributeCollectionSerializationBase::AttributeCollectionSerializationBase(
    const std::string& name_,
    HighFive::DataSet&& value_changes_ds,
    HighFive::DataSet&& updates_ds)
    : m_name(name_)
    , m_value_changes_dataset(value_changes_ds)
    , m_updates_dataset(updates_ds)
{}

AttributeCollectionSerializationBase::~AttributeCollectionSerializationBase() = default;

HighFive::DataSetAccessProps AttributeCollectionSerializationBase::access_properties()
{
    HighFive::DataSetAccessProps props;
    return props;
}
HighFive::DataSetCreateProps AttributeCollectionSerializationBase::create_properties()
{
    HighFive::DataSetCreateProps props;
    props.add(HighFive::Chunking(std::vector<hsize_t>{2}));
    return props;
}

size_t AttributeCollectionSerializationBase::record()
{
    AttributeCollectionUpdate update = record_value_changes();
    return utils::append_value_to_dataset(m_updates_dataset, update) - 1;
}

size_t AttributeCollectionSerializationBase::record_initial_state()
{
    AttributeCollectionUpdate update = record_entire_state();
    return utils::append_value_to_dataset(m_updates_dataset, update) - 1;
}

// the number of updates serialized
size_t AttributeCollectionSerializationBase::changes_size() const
{
    return m_value_changes_dataset.getElementCount();
}
// the number of updates serialized
size_t AttributeCollectionSerializationBase::updates_size() const
{
    return m_updates_dataset.getElementCount();
}

AttributeCollectionUpdate AttributeCollectionSerializationBase::update(size_t index) const
{
    assert(index < updates_size());
    AttributeCollectionUpdate ret;
    std::vector<AttributeCollectionUpdate> retvec;
    m_updates_dataset.select({index}, {1}).read(retvec);
    return retvec[0];
}

AttributeCollectionUpdate::AttributeCollectionUpdate(
    size_t begin,
    size_t end,
    size_t old_size_,
    size_t new_size_)
    : old_size(old_size_)
    , new_size(new_size_)
    , range{begin, end}
{}
void AttributeCollectionSerializationBase::load(size_t index)
{
    AttributeCollectionUpdate upd = update(index);
    apply_update(upd);
}
void AttributeCollectionSerializationBase::unload(size_t index)
{
    AttributeCollectionUpdate upd = update(index);
    unapply_update(upd);
}


// void read_attribute_changes()
//{
//     std::vector<AttributeCollectionUpdate> attr_changes;
//     std::vector<size_t> attr_start, attr_size;
//     attr_start.emplace_back(tri_op.update_range_begin);
//     attr_size.emplace_back(tri_op.update_range_end - tri_op.update_range_begin);
//
//     logger.attribute_changes_dataset.select(attr_start, attr_size).read(attr_changes);
// }
