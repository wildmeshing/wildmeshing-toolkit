#pragma once
#include <spdlog/spdlog.h>
#include <wmtk/AttributeCollection.hpp>
#include "Hdf5Utils.hpp"

namespace HighFive {
class File;
}

namespace wmtk {
class AttributeCollectionSerializerBase
{
protected:
    AttributeCollectionSerializerBase(
        HighFive::File& file,
        const std::string& name,
        const HighFive::DataType& data_type);

public:
    virtual ~AttributeCollectionSerializerBase();

    virtual void serialize() = 0;
    virtual void deserialize() = 0;

protected:
    HighFive::DataSet m_dataset;
};

class AbstractAttributeCollection;
// Interface for adding an attribute to a logged hdf5 file
template <typename AttributeType>
class AttributeCollectionSerializer : public AttributeCollectionSerializerBase
{
public:
    //using SerializedVectorType = utils::detail::TypeErasedVectorType<AttributeType>;
    using TypeErasedType = utils::detail::TypeErasedType<AttributeType>;
    AttributeCollectionSerializer(
        HighFive::File& file,
        const std::string& name,
        AttributeCollection<AttributeType>& attr_);


    void serialize() override;
    void deserialize() override;

private:
    AttributeCollection<AttributeType>& m_collection;
};


// Class capable of recording updates to a single attribute
template <typename T>
AttributeCollectionSerializer<T>::AttributeCollectionSerializer(
    HighFive::File& file,
    const std::string& name,
    AttributeCollection<T>& attr_)
    : AttributeCollectionSerializerBase(file, name, HighFive::create_datatype<SerializedVectorType>())
    , m_collection(attr_)
//: AttributeCollectionSerializerBase(file, name, HighFive::create_datatype<T>()),
//: m_collection(attr_)
{}

template <typename T>
void AttributeCollectionSerializer<T>::serialize()
{
    static_assert(sizeof(data::value_type) == sizeof(m_collection::value_type));
    spdlog::error("Write");
    ;
    m_dataset.resize({m_collection.size()});
    spdlog::warn("{}", fmt::join(m_dataset.getDimensions(), ","));

    SerializedVectorType data;
    data.resize(m_collection.size());
    spdlog::info("{} {}", sizeof(SerializedVectorType), sizeof(T));
    for (size_t j = 0; j < m_collection.size(); ++j) {
        memcpy((void*)data[j].data(), (const void*)&m_collection[j], sizeof(T));
    }
    m_dataset.write(data);
    // m_dataset.write_raw(m_collection.data());
    spdlog::error("Write done");
    ;
}
template <typename T>
void AttributeCollectionSerializer<T>::deserialize()
{
    // spdlog::error("Read");;
    // spdlog::warn("{}",fmt::join(m_dataset.getDimensions(),","));
    // m_collection.resize(m_dataset.getDimensions()[0]);
    std::vector<TypeErasedType> data(m_collection.size());
    m_dataset.read(data);
    static_assert(sizeof(data::value_type) == sizeof(m_collection::value_type));
    for (size_t j = 0; j < m_collection.size(); ++j) {
        memcpy((void*)&m_collection[j], (void*)data[j].data(), sizeof(T));
    }
    ////m_dataset.read(m_collection.data());
    // m_collection.m_attributes.assign(data.begin(),data.end());;
    // spdlog::error("REad done");;
}

} // namespace wmtk


namespace HighFive::details {
// adaptation of HighFive's std::vector inspector for AttributeCollection
template <typename T>
struct inspector<wmtk::AttributeCollection<T>>
{
    using type = wmtk::AttributeCollection<T>;
    using value_type = unqualified_t<T>;
    using base_type = typename inspector<value_type>::base_type;
    using hdf5_type = typename inspector<value_type>::hdf5_type;

    static constexpr size_t ndim = 1;
    static constexpr size_t recursive_ndim = ndim + inspector<value_type>::recursive_ndim;
    static constexpr bool is_trivially_copyable = std::is_trivially_copyable<value_type>::value &&
                                                  inspector<value_type>::is_trivially_copyable;

    static std::vector<size_t> getDimensions(const type& val)
    {
        std::vector<size_t> sizes{val.size()};
        if (val.size() > 0) {
            auto s = inspector<value_type>::getDimensions(val[0]);
            sizes.insert(sizes.end(), s.begin(), s.end());
        }
        return sizes;
    }

    static size_t getSizeVal(const type& val) { return compute_total_size(getDimensions(val)); }

    static size_t getSize(const std::vector<size_t>& dims) { return compute_total_size(dims); }

    static void prepare(type& val, const std::vector<size_t>& dims)
    {
        val.resize(dims[0]);
        std::vector<size_t> next_dims(dims.begin() + 1, dims.end());
        for (auto&& e : val) {
            inspector<value_type>::prepare(e, next_dims);
        }
    }

    static hdf5_type* data(type& val) { return inspector<value_type>::data(val[0]); }

    static const hdf5_type* data(const type& val) { return inspector<value_type>::data(val[0]); }

    static void serialize(const type& val, hdf5_type* m)
    {
        size_t subsize = inspector<value_type>::getSizeVal(val[0]);
        for (auto&& e : val) {
            inspector<value_type>::serialize(e, m);
            m += subsize;
        }
    }

    static void unserialize(const hdf5_type* vec_align, const std::vector<size_t>& dims, type& val)
    {
        std::vector<size_t> next_dims(dims.begin() + 1, dims.end());
        size_t next_size = compute_total_size(next_dims);
        for (size_t i = 0; i < dims[0]; ++i) {
            inspector<value_type>::unserialize(vec_align + i * next_size, next_dims, val[i]);
        }
    }
};

} // namespace HighFive::details
