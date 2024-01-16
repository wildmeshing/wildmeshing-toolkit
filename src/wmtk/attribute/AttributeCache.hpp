#pragma once
#include <Eigen/Core>
#include <map>
#include <memory>
#include "AccessorBase.hpp"
#include "AttributeCacheData.hpp"

#if defined(WMTK_USE_MONOTONIC_ATTRIBUTE_CACHE)
#include <memory_resource>
#endif


namespace wmtk::attribute {
template <typename T>
class AttributeCache
{
public:
    using Data = AttributeCacheData<T>;
    using DataStorage = std::map<int64_t, Data, std::less<int64_t>
#if defined(WMTK_USE_MONOTONIC_ATTRIBUTE_CACHE)
        , std::pmr::polymorphic_allocator<std::pair<const int64_t, Data>>
#endif
        >;

    using MapResult = typename AccessorBase<T>::MapResult;
    using ConstMapResult = typename AccessorBase<T>::ConstMapResult;


    AttributeCache();
    ~AttributeCache();
    AttributeCache(const AttributeCache&) = delete;
    AttributeCache& operator=(const AttributeCache&) = delete;

    // returns an iterator and if the value was inserted
    // the returned value may have undetermined state if new oen was inserted
    std::pair<typename DataStorage::iterator, bool> load_it(int64_t index) const;


    void clear();

    void flush_to(Attribute<T>& attribute);
    void flush_to(AttributeCache<T>& other);

    // flushes to some other buffer that was passed in
    void flush_to(const Attribute<T>& attribute, std::vector<T>& other) const;


protected:
#if defined(WMTK_USE_MONOTONIC_ATTRIBUTE_CACHE)
    mutable std::vector<std::int8_t> m_buffer;
    mutable std::pmr::monotonic_buffer_resource m_resource;
#endif
    mutable DataStorage m_data;
};
} // namespace wmtk::attribute
