#pragma once
#include <Eigen/Core>
#include <map>
#include <memory>
#include "AttributeCacheData.hpp"
#include "MapTypes.hpp"

#if defined(WMTK_USE_MONOTONIC_ATTRIBUTE_CACHE)
#include <memory_resource>
#endif


namespace wmtk::attribute {
template <typename T>
class Attribute;
template <typename T, int Dim>
class AccessorBase;
namespace internal {
template <typename T>
class AttributeMapCache
{
public:
    using Data = AttributeCacheData<T>;
    using DataStorage = std::map<
        int64_t,
        Data,
        std::less<int64_t>
#if defined(WMTK_USE_MONOTONIC_ATTRIBUTE_CACHE)
        ,
        std::pmr::polymorphic_allocator<std::pair<const int64_t, Data>>
#endif
        >;

    using MapResult = internal::MapResult<T>;
    using ConstMapResult = internal::ConstMapResult<T>;


    AttributeMapCache();
    ~AttributeMapCache();
    AttributeMapCache(const AttributeMapCache&) = delete;
    AttributeMapCache& operator=(const AttributeMapCache&) = delete;
    AttributeMapCache(AttributeMapCache&&) = default;
    AttributeMapCache& operator=(AttributeMapCache&&) = default;


    template <typename Derived>
    void try_caching(int64_t index, const Eigen::MatrixBase<Derived>& value);
    void try_caching(int64_t index, const T& value);

    typename DataStorage::const_iterator find_value(int64_t index) const;
    bool is_value(const typename DataStorage::const_iterator& it) const;

    void clear();
    size_t size() const { return m_data.size(); }

    void apply_to(Attribute<T>& attribute) const;
    void apply_to(AttributeMapCache<T>& other) const;

    // applyes to some other buffer that was passed in
    void apply_to(const Attribute<T>& attribute, std::vector<T>& other) const;

    const DataStorage& data() const { return m_data; }

protected:
#if defined(WMTK_USE_MONOTONIC_ATTRIBUTE_CACHE)
    std::vector<std::int8_t> m_buffer;
    std::pmr::monotonic_buffer_resource m_resource;
#endif
    DataStorage m_data;
};

} // namespace internal
} // namespace wmtk::attribute

#include "AttributeMapCache.hxx"
