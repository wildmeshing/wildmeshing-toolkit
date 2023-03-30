#pragma once
#include <wmtk/serialization/AttributeCollectionSerialization.h>


namespace wmtk {


// Interface for adding an attribute to a logged hdf5 file
class AttributeCollectionReplayer
{
public:
    template <typename T>
    AttributeCollectionReplayer(
        HighFive::File& file,
        const std::string& name,
        AttributeCollection<T>& attr_);

    ~AttributeCollectionReplayer();


    // sets the held attribute collection's state to the initial recorded state
    bool reset_to_initial_state();
    // runs forwards or backwards to a target update index
    bool run_to_step(size_t index);
    bool run_to_end();

    // advances one set of updates
    bool step_forward();
    // undoes one set of updates
    bool step_backward();

    bool valid_current_update_index() const;

    // the number of updates serialized
    size_t updates_size() const;
    // indexe over updates
    size_t current_update_index() const { return m_current_update_index; }

protected:
    // the file being serialized to, the name of the attribute, and information on how the data
    // should be serialized
    AttributeCollectionReplayer(
        std::unique_ptr<AttributeCollectionSerializationBase>&& serialization);
    // friend class OperationSerialization;


protected:
    std::unique_ptr<AttributeCollectionSerializationBase> m_serialization;
    size_t m_current_update_index = 0;

private:
    // load a particular set of attribute changes from a particular dataset
    void load(size_t update_index);

    // undoes a particular change to an attribute
    void unload(size_t update_index);
};


// Class capable of recording updates to a single attribute
template <typename T>
AttributeCollectionReplayer::AttributeCollectionReplayer(
    HighFive::File& file,
    const std::string& name,
    AttributeCollection<T>& attr_)
    : AttributeCollectionReplayer(
          std::make_unique<AttributeCollectionSerialization<T>>(file, name, attr_))
{}

} // namespace wmtk
