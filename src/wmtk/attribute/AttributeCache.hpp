#pragma once
#include <Eigen/Core>
#include <map>
#include <memory>
#include "AttributeCacheData.hpp"
#include "internal/MapTypes.hpp"

#if defined(WMTK_USE_MONOTONIC_ATTRIBUTE_CACHE)
#include <memory_resource>
#endif


namespace wmtk::attribute {
template <typename T>
class Attribute;
template <typename T>
class AccessorBase;
template <typename T>
class AttributeCache
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


    AttributeCache();
    ~AttributeCache();
    AttributeCache(const AttributeCache&) = delete;
    AttributeCache& operator=(const AttributeCache&) = delete;
    AttributeCache(AttributeCache&&) = default;
    AttributeCache& operator=(AttributeCache&&) = default;


    void try_caching(int64_t index, const MapResult& value);

    typename DataStorage::const_iterator find_value(int64_t index) const;
    bool is_value(const typename DataStorage::const_iterator& it) const;

    void clear();

    void apply_to(Attribute<T>& attribute) const;
    void apply_to(AttributeCache<T>& other) const;

    // applyes to some other buffer that was passed in
    void apply_to(const Attribute<T>& attribute, std::vector<T>& other) const;


protected:
#if defined(WMTK_USE_MONOTONIC_ATTRIBUTE_CACHE)
    std::vector<std::int8_t> m_buffer;
    std::pmr::monotonic_buffer_resource m_resource;
#endif
    DataStorage m_data;
};

} // namespace wmtk::attribute

#include "AttributeCache.hxx"
