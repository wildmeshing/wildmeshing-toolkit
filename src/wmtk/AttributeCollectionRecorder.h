#pragma once
#include <wmtk/AttributeCollectionSerialization.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/utils/Hdf5Utils.hpp>


namespace wmtk {

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

    size_t record();
    AttributeCollectionUpdate  update(size_t index) const;

protected:
    // the file being serialized to, the name of the attribute, and information on how the data
    // should be serialized
    AttributeCollectionRecorder(
        std::unique_ptr<AttributeCollectionSerializationBase>&& serialization);
    // friend class OperationSerialization;


    // load a particular set of attribute changes from a particular dataset
    void load(const AttributeCollectionUpdate& update, const HighFive::DataSet& data_set);

    // undoes a particular change to an attribute
    void unload(const AttributeCollectionUpdate& update, const HighFive::DataSet& data_set);

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

