#pragma once
#include <wmtk/AttributeCollectionSerialization.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/utils/Hdf5Utils.hpp>


namespace wmtk {

    class AbstractAttributeCollection;
// Interface for adding an attribute to a logged hdf5 file
class AttributeCollectionRecorder
{
public:

    template <typename T>
    AttributeCollectionRecorder(
        HighFive::File& file,
        const std::string& name,
        AttributeCollection<T>& attr_);
    ~AttributeCollectionRecorder();


    // number of updates (chunks of changes) recorded
    size_t updates_size() const;
    // number of individual value changes recorded
    size_t changes_size() const;

    AttributeCollectionUpdate  update(size_t index) const;

    friend class AbstractAttributeCollection;
protected:
    size_t record();
    size_t record_initial_state();
    // the file being serialized to, the name of the attribute, and information on how the data
    // should be serialized
    AttributeCollectionRecorder(
        std::unique_ptr<AttributeCollectionSerializationBase>&& serialization);
    // friend class OperationSerialization;



protected:
    std::unique_ptr<AttributeCollectionSerializationBase> m_serialization;
};


// Class capable of recording updates to a single attribute
template <typename T>
AttributeCollectionRecorder::AttributeCollectionRecorder(
    HighFive::File& file,
    const std::string& name,
    AttributeCollection<T>& attr_)
    : AttributeCollectionRecorder(
          std::make_unique<AttributeCollectionSerialization<T>>(file, name, attr_))
{}


} // namespace wmtk

