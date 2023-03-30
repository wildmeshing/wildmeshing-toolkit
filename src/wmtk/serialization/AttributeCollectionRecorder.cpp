#include <wmtk/serialization/AttributeCollectionRecorder.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/serialization/AttributeCollectionSerialization.h>
#include <wmtk/serialization/Hdf5Utils.hpp>

using namespace wmtk;
AttributeCollectionRecorder::AttributeCollectionRecorder(
    std::unique_ptr<AttributeCollectionSerializationBase>&& serialization)
    : m_serialization(std::move(serialization))
{
    AbstractAttributeCollection& aac = m_serialization->abstract_attribute_collection();

    aac.add_recorder(this);
}

AttributeCollectionRecorder::AttributeCollectionRecorder() = default;

AttributeCollectionRecorder::AttributeCollectionRecorder(AttributeCollectionRecorder&&) = default;

AttributeCollectionRecorder& AttributeCollectionRecorder::operator=(AttributeCollectionRecorder&&) =
    default;

AttributeCollectionRecorder::~AttributeCollectionRecorder()
{
    AbstractAttributeCollection& aac = m_serialization->abstract_attribute_collection();

    aac.remove_recorder(this);
}


size_t AttributeCollectionRecorder::record()
{
    return m_serialization->record();
}

size_t AttributeCollectionRecorder::record_initial_state()
{
    return m_serialization->record_initial_state();
}

const std::string& AttributeCollectionRecorder::name() const
{
    return m_serialization->name();
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
    bool AttributeCollectionRecorder::valid() const {
        return bool(m_serialization);
    }
