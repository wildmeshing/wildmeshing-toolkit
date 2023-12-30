#include "AttributeCache.hpp"
#include <wmtk/utils/Rational.hpp>

namespace wmtk::attribute {


template <typename T>
AttributeCache<T>::AttributeCache() = default;
template <typename T>
AttributeCache<T>::~AttributeCache() = default;
template <typename T>
auto AttributeCache<T>::load_it(long index) const -> std::pair<typename DataStorage::iterator, bool>
{
    if (const auto& it = m_data.find(index); it != m_data.end()) {
        return {it, false};
    } else {
        return m_data.try_emplace(index, false);
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
        if (data.dirty) {
            attribute.vector_attribute(index) = data.data;
        }
        data.dirty = false;
    }
}
template <typename T>
void AttributeCache<T>::flush_to(AttributeCache<T>& other)
{
    auto& o_data = other.m_data;

    for (auto& [index, data] : m_data) {
        if (data.dirty) {
            o_data[index] = data;
        }
        data.dirty = false;
    }
}

template <typename T>
void AttributeCache<T>::flush_to(const Attribute<T>& attribute, std::vector<T>& other) const
{
    for (auto& [index, data] : m_data) {
        if (data.dirty) {
            attribute.vector_attribute(index, other) = data.data;
        }
    }
}
template class AttributeCache<long>;
template class AttributeCache<double>;
template class AttributeCache<char>;
template class AttributeCache<Rational>;
} // namespace wmtk::attribute
