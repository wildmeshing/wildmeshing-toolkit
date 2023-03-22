#include <wmtk/AttributeCollectionReplayer.h>
#include <wmtk/AttributeCollectionSerialization.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/utils/Hdf5Utils.hpp>

using namespace wmtk;
AttributeCollectionReplayer::AttributeCollectionReplayer(
    std::unique_ptr<AttributeCollectionSerializationBase>&& serialization)
    : m_serialization(std::move(serialization))
{}


AttributeCollectionReplayer::~AttributeCollectionReplayer() {}

bool AttributeCollectionReplayer::step_forward()
{
    if (m_current_update_index <= updates_size()) {
        return false;
    }
    m_current_update_index = m_current_update_index + 1;
    load(current_update_index());
    return true;
}
bool AttributeCollectionReplayer::step_backward()
{
    if (m_current_update_index == 0) {
        return false;
    }
    m_current_update_index = m_current_update_index - 1;
    unload(current_update_index());
    return true;
}


// load a particular set of attribute changes from a particular dataset
void AttributeCollectionReplayer::load(size_t index)
{
    m_serialization->load(index);
}

// undoes a particular change to an attribute
void AttributeCollectionReplayer::unload(size_t index)
{
    m_serialization->unload(index);
}

size_t AttributeCollectionReplayer::updates_size() const
{
    return m_serialization->updates_size();
}
