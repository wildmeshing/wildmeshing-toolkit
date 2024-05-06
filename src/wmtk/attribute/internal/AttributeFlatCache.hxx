#include <wmtk/Types.hpp>
#include <wmtk/attribute/AccessorBase.hpp>
#include <wmtk/utils/Rational.hpp>
#include "AttributeFlatCache.hpp"

namespace wmtk::attribute::internal {


template <typename T>
inline AttributeFlatCache<T>::AttributeFlatCache()
    : m_buffer(64)
    , m_indices(32)
{}
template <typename T>
inline AttributeFlatCache<T>::~AttributeFlatCache() = default;


template <typename T>
inline void AttributeFlatCache<T>::clear()
{
    m_buffer.clear();
    m_indices.clear();
}


template <typename T>
template <typename Derived>
inline void AttributeFlatCache<T>::try_caching(
    int64_t index,
    const Eigen::MatrixBase<Derived>& value)
{
    // basically try_emplace but optimizes to avoid accessing the pointed-to value

    size_t dim = value.size();
    size_t old_size = m_buffer.size();

    //assert(old_size % dim == 0);
    //assert(old_size / value.size() == m_indices.size());
    // m_indices.emplace_back(index, m_indices.size());
    m_indices.emplace_back(index, old_size);


    m_buffer.resize(old_size + dim);
    std::copy(value.begin(), value.end(), m_buffer.begin() + old_size);
    // assert(dim * m_buffer.size() == m_indices.size());
}

template <typename T>
inline void AttributeFlatCache<T>::try_caching(int64_t index, const T& value)
{
    //assert(m_buffer.size() == m_indices.size());
    m_indices.emplace_back(index, m_buffer.size());
    m_buffer.emplace_back(value);
    //assert(m_buffer.size() == m_indices.size());
}


template <typename T>
inline void AttributeFlatCache<T>::apply_to(Attribute<T>& attribute) const
{
    for (auto it = m_indices.crbegin(); it != m_indices.crend(); ++it) {
        const auto& [global, local] = *it;
        auto a = attribute.vector_attribute(global);
        auto b = attribute.const_vector_attribute_from_start(local, m_buffer);
        a = b;
    }
}
template <typename T>
inline void AttributeFlatCache<T>::apply_to(AttributeFlatCache<T>& other) const
{
    size_t offset = other.m_buffer.size();
    std::copy(m_buffer.begin(), m_buffer.end(), std::back_inserter(other.m_buffer));
    std::transform(
        m_indices.begin(),
        m_indices.end(),
        std::back_inserter(other.m_indices),
        [offset](const auto& pr) {
            const auto& [a, b] = pr;
            return std::make_pair(a, b + offset);
        });
}

template <typename T>
inline void AttributeFlatCache<T>::apply_to(const Attribute<T>& attribute, std::vector<T>& other)
    const
{
    for (auto it = m_indices.crbegin(); it != m_indices.crend(); ++it) {
        const auto& [global, local] = *it;
        auto a = attribute.vector_attribute(global, other);
        auto b = attribute.const_vector_attribute_from_start(local, m_buffer);
        a = b;
    }
}

template <typename T>
inline auto AttributeFlatCache<T>::get_value(int64_t index, size_t dim) const -> const T*
{
    for (auto iit = m_indices.cbegin(); iit != m_indices.cend(); ++iit) {
        const auto& [global, local] = *iit;
        if (global == index) {
            return m_buffer.data() + local;
        }
    }
    return nullptr;
}
} // namespace wmtk::attribute::internal
