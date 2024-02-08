#pragma once

#include <wmtk/utils/Rational.hpp>
#include "AttributeScope.hpp"
#include "AttributeScopeStack.hpp"
#include "CachingAccessor.hpp"

namespace wmtk::attribute {

template <typename T>
inline CachingAccessor<T>::CachingAccessor(Mesh& mesh_in, const TypedAttributeHandle<T>& handle)
    : BaseType(mesh_in, handle)
    , m_cache_stack(attribute().get_local_scope_stack())
{}

template <typename T>
inline CachingAccessor<T>::~CachingAccessor() = default;

template <typename T>
inline bool CachingAccessor<T>::has_stack() const
{
    return !m_cache_stack.empty();
}

template <typename T>
inline bool CachingAccessor<T>::writing_enabled() const
{
    return m_cache_stack.writing_enabled();
}

template <typename T>
inline int64_t CachingAccessor<T>::stack_depth() const
{
    return m_cache_stack.size();
}

template <typename T>
inline auto CachingAccessor<T>::vector_attribute(const int64_t index) -> MapResult
{
    return m_cache_stack.vector_attribute(*this, index);
}


template <typename T>
inline auto CachingAccessor<T>::scalar_attribute(const int64_t index) -> T&
{
    return m_cache_stack.scalar_attribute(*this, index);
}

template <typename T>
inline auto CachingAccessor<T>::const_vector_attribute(const int64_t index) const -> ConstMapResult
{
    return m_cache_stack.const_vector_attribute(*this, index);
}


template <typename T>
inline auto CachingAccessor<T>::const_scalar_attribute(const int64_t index) const -> T
{
    return m_cache_stack.const_scalar_attribute(*this, index);
}

template <typename T>
inline auto CachingAccessor<T>::vector_attribute(const int64_t index) const -> ConstMapResult
{
    return const_vector_attribute(index);
}
template <typename T>
inline T CachingAccessor<T>::scalar_attribute(const int64_t index) const
{
    return const_scalar_attribute(index);
}

template <typename T>
inline auto CachingAccessor<T>::scalar_attribute(const int64_t index, int8_t offset) -> T&
{
    return m_cache_stack.scalar_attribute(*this, index, offset);
}


template <typename T>
inline auto CachingAccessor<T>::const_scalar_attribute(const int64_t index, int8_t offset) const
    -> T
{
    return m_cache_stack.const_scalar_attribute(*this, index, offset);
}

// template class CachingAccessor<char>;
// template class CachingAccessor<int64_t>;
// template class CachingAccessor<double>;
// template class CachingAccessor<Rational>;
} // namespace wmtk::attribute
