#include <wmtk/utils/Rational.hpp>
#include <wmtk/Types.hpp>
#include "AccessorBase.hpp"
#include "AttributeCache.hpp"

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
auto AttributeCache<T>::find_value(int64_t index) const -> typename DataStorage::const_iterator
{
    return m_data.find(index);
}
template <typename T>
bool AttributeCache<T>::is_value(const typename DataStorage::const_iterator& it) const
{
    return it != m_data.end();
}

template <typename T>
void AttributeCache<T>::clear()
{
    m_data.clear();
}


template <typename T>
template <typename Derived>
void AttributeCache<T>::try_caching(int64_t index, const Eigen::MatrixBase<Derived>& value)
{
    // basically try_emplace but optimizes to avoid accessing the pointed-to value
    auto [it, did_insert] = m_data.try_emplace(index, AttributeCacheData<T>{});
    if (did_insert) {
        it->second.data = value;
    }
}

template <typename T>
void AttributeCache<T>::try_caching(int64_t index, const T& value)
{
    // basically try_emplace but optimizes to avoid accessing the pointed-to value
    auto [it, did_insert] = m_data.try_emplace(index, AttributeCacheData<T>{});
    if (did_insert) {
        it->second.data =  VectorX<T>::Constant(1,value);
    }
}


template <typename T>
void AttributeCache<T>::apply_to(Attribute<T>& attribute) const
{
    for (const auto& [index, data] : m_data) {
        {
            auto a = attribute.vector_attribute(index);
            auto b = data.data;
            a = b;
        }
    }
}
template <typename T>
void AttributeCache<T>::apply_to(AttributeCache<T>& other) const
{
    auto& o_data = other.m_data;

    for (const auto& [index, data] : m_data) {
        {
            if (o_data.find(index) == o_data.end()) {
                o_data[index] = data;
            }
        }
    }
}

template <typename T>
void AttributeCache<T>::apply_to(const Attribute<T>& attribute, std::vector<T>& other) const
{
    for (auto& [index, data] : m_data) {
        attribute.vector_attribute(index, other) = data.data;
    }
}
} // namespace wmtk::attribute
