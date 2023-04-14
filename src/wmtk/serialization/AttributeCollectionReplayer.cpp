#include <wmtk/serialization/AttributeCollectionReplayer.h>
#include <wmtk/serialization/AttributeCollectionSerialization.h>
#include <wmtk/serialization/Hdf5Utils.hpp>

using namespace wmtk;
AttributeCollectionReplayer::AttributeCollectionReplayer(
    std::unique_ptr<AttributeCollectionSerializationBase>&& serialization)
    : m_serialization(std::move(serialization))
{
    reset_to_initial_state();
}


AttributeCollectionReplayer::~AttributeCollectionReplayer() {}

bool AttributeCollectionReplayer::step_forward()
{
    if (m_current_update_index >= updates_size()) {
        return false;
    }
    load(current_update_index());
    m_current_update_index = m_current_update_index + 1;
    return true;
}
bool AttributeCollectionReplayer::step_backward()
{
    // clamp in case we went past by accident. this is HACKY and needs to be figured out properly
    m_current_update_index = std::min(m_current_update_index, updates_size()-1);
    if (m_current_update_index == 0) {
        return false;
    }
    unload(current_update_index());
    m_current_update_index = m_current_update_index - 1;
    return true;
}
bool AttributeCollectionReplayer::reset_to_initial_state()
{
    // assumes that the first data stored is the initial data state
    // return run_to_step(0);
    if (updates_size() == 0) {
        return false;
    }
    m_current_update_index = 1;
    m_serialization->load(0);
    return true;
}
bool AttributeCollectionReplayer::run_to_step(size_t index)
{
    // could check for running forward / backward but using two while loops should suffice
    while (current_update_index() < index) {
        if (!step_forward()) {
            return false;
        }
    }
    while (current_update_index() > index) {
        if (!step_backward()) {
            return false;
        }
    }
    return true;
}
bool AttributeCollectionReplayer::run_to_end()
{
    return run_to_step(updates_size() );
}

bool AttributeCollectionReplayer::valid_current_update_index() const
{
    return current_update_index() < updates_size();
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
