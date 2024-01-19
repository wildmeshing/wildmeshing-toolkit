#include "AttributeCache.hpp"
#include <wmtk/utils/Rational.hpp>

namespace wmtk::attribute {


template <typename T>
AttributeCache<T>::AttributeCache()
#if defined(WMTK_USE_MONOTONIC_ATTRIBUTE_CACHE)
    : m_buffer(32 * sizeof(typename DataStorage::value_type))
    , m_resource(m_buffer.data(), m_buffer.size())
    , m_data(std::pmr::polymorphic_allocator<std::pair<const int64_t, Data>>{&m_resource})
#endif
{} //: m_data({m_resource}) {}
template <typename T>
AttributeCache<T>::~AttributeCache() = default;
template <typename T>
auto AttributeCache<T>::load_it(int64_t index) const
    -> std::pair<typename DataStorage::iterator, bool>
{
    if (const auto& it = m_data.find(index); it != m_data.end()) {
        return {it, false};
    } else {
#if defined(WMTK_ONLY_CACHE_WRITES)
        return   m_data.try_emplace(index,AttributeCacheData<T>{});
#else
        return m_data.try_emplace(index, false);
#endif
    }
}


template <typename T>
void AttributeCache<T>::clear()
{
    m_data.clear();
}

template <typename T>
void AttributeCache<T>::flush_to(Attribute<T>& attribute)
{
    for (auto& [index, data] : m_data) {
#if !defined(WMTK_ONLY_CACHE_WRITES)
        if (data.dirty)
#endif
        {
            auto a = attribute.vector_attribute(index);
            auto b = data.data;
            a = b;
        }
#if !defined(WMTK_ONLY_CACHE_WRITES)
        data.dirty = false;
#endif
    }
}
template <typename T>
void AttributeCache<T>::flush_to(AttributeCache<T>& other)
{
    auto& o_data = other.m_data;

    for (auto& [index, data] : m_data) {
#if !defined(WMTK_ONLY_CACHE_WRITES)
        if (data.dirty)
#endif
        {
#if defined(WMTK_FLUSH_ON_FAIL)
            if(o_data.find(index) == o_data.end()) {
                o_data[index] = data;
            }

#else
            o_data[index] = data;
#endif
        }
#if !defined(WMTK_ONLY_CACHE_WRITES)
        data.dirty = false;
#endif
    }
}

template <typename T>
void AttributeCache<T>::flush_to(const Attribute<T>& attribute, std::vector<T>& other) const
{
    for (auto& [index, data] : m_data) {
#if !defined(WMTK_ONLY_CACHE_WRITES)
        if (data.dirty)
#endif
        {
            attribute.vector_attribute(index, other) = data.data;
        }
    }
}
template class AttributeCache<int64_t>;
template class AttributeCache<double>;
template class AttributeCache<char>;
template class AttributeCache<Rational>;
} // namespace wmtk::attribute
