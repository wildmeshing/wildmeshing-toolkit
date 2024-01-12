#pragma once
#include <Eigen/Core>
#include <map>
#include <memory>
#include "AccessorBase.hpp"
#include "AttributeCacheData.hpp"
#include <memory_resource>


namespace wmtk::attribute {
template <typename T>
class AttributeCache
{
public:
    using Data = AttributeCacheData<T>;
    using DataStorage = std::map<int64_t, Data, std::less<int64_t>, std::pmr::polymorphic_allocator<std::pair<const int64_t, Data>>>;

    using MapResult = typename AccessorBase<T>::MapResult;
    using ConstMapResult = typename AccessorBase<T>::ConstMapResult;


    AttributeCache();
    ~AttributeCache();

    // returns an iterator and if the value was inserted
    // the returned value may have undetermined state if new oen was inserted
    std::pair<typename DataStorage::iterator, bool> load_it(int64_t index) const;


    void clear();

    void flush_to(Attribute<T>& attribute);
    void flush_to(AttributeCache<T>& other);

    // flushes to some other buffer that was passed in
    void flush_to(const Attribute<T>& attribute, std::vector<T>& other) const;


protected:
    mutable std::vector<std::int8_t> m_buffer;
    mutable std::pmr::monotonic_buffer_resource m_resource;
    mutable DataStorage m_data;
};
} // namespace wmtk::attribute
