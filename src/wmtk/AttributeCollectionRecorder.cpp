#include <wmtk/AttributeCollectionRecorder.h>
#include <wmtk/AttributeCollectionSerialization.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/utils/Hdf5Utils.hpp>

using namespace wmtk;
AttributeCollectionRecorder::AttributeCollectionRecorder(
    std::unique_ptr<AttributeCollectionSerializationBase>&& serialization)
    : m_serialization(std::move(serialization))
{
    AbstractAttributeCollection& aac = m_serialization->abstract_attribute_collection();

    aac.recorder_ptrs.emplace_back(this);
}


AttributeCollectionRecorder::~AttributeCollectionRecorder()
{
    AbstractAttributeCollection& aac = m_serialization->abstract_attribute_collection();

    aac.recorder_ptrs.remove(this);
}


size_t AttributeCollectionRecorder::record()
{
    return m_serialization->record();
}

// load a particular set of attribute changes from a particular dataset
void AttributeCollectionRecorder::load(
    const AttributeCollectionUpdate& update,
    const HighFive::DataSet& data_set)
{
    m_serialization->load(update, data_set);
}

// undoes a particular change to an attribute
void AttributeCollectionRecorder::unload(
    const AttributeCollectionUpdate& update,
    const HighFive::DataSet& data_set)
{
    m_serialization->load(update, data_set);
}

size_t AttributeCollectionRecorder::updates_size() const
{
    return m_serialization->updates_size();
}
size_t AttributeCollectionRecorder::changes_size() const
{
    return m_serialization->changes_size();
}
AttributeCollectionUpdate AttributeCollectionRecorder::update(size_t index) const
{
    return m_serialization->update(index);
}
