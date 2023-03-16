
namespace wmtk {


// Interface for adding an attribute to a logged hdf5 file
class AttributeCollectionReplayer
{
public:
    template <typename T>
    create(HighFive::File& file, const std::string& name, AttributeCollection<T>& attr_);

protected:
    // the file being serialized to, the name of the attribute, and information on how the data
    // should be serialized
    AttributeCollectionReplayer(
        std::unique_ptr<AttributeCollectionSerializationBase>&& serialization);
    ~AttributeCollectionReplayer();


    bool step_forward();
    bool step_backward();

protected:
    // friend class OperationSerialization;

    // returns the range of values used and size
    // {start_index, end_index, new size of data}
    std::array<size_t, 3> record(HighFive::DataSet& data_set);

    // load a particular set of attribute changes from a particular dataset
    void load(const AttributeChanges& changes, const HighFive::DataSet& data_set);

    // undoes a particular change to an attribute
    void unload(const AttributeChanges& changes, const HighFive::DataSet& data_set);


protected:
    std::unique_ptr<AttributeCollectionSerializationBase> m_serialization;
    size_t current_index = 0;
};


// Class capable of recording updates to a single attribute
template <typename T>
AttributeCollectionReplayer::create(
    HighFive::File& file,
    const std::string& name,
    AttributeCollection<T>& attr_)
{
    return AttributeCollectionReplayer(
        std::make_unique<AttributeCollectionSerialization<T>>(file, name, attr_));
}

} // namespace wmtk
