
#include "CachingAccessor.hpp"
#include <wmtk/utils/Rational.hpp>
#include "AttributeScope.hpp"
#include "AttributeScopeStack.hpp"

namespace wmtk::attribute {

template <typename T>
CachingAccessor<T>::CachingAccessor(
    Mesh& mesh_in,
    const MeshAttributeHandle<T>& handle,
    AttributeAccessMode mode)
    : BaseType(mesh_in, handle)
    , m_mode(mode)
{
    m_cache_stack = attribute().get_local_scope_stack_ptr();
}

template <typename T>
CachingAccessor<T>::~CachingAccessor() = default;
template <typename T>
bool CachingAccessor<T>::has_stack() const
{
    return m_cache_stack && !m_cache_stack->empty();
}

template <typename T>
std::optional<long> CachingAccessor<T>::stack_depth() const
{
    if (m_cache_stack != nullptr) {
        return m_cache_stack->depth();
    } else {
        return {};
    }
}

template <typename T>
auto CachingAccessor<T>::vector_attribute(const long index) -> MapResult
{
    if (has_stack()) {
        return m_cache_stack->current_scope_ptr()->vector_attribute(*this, m_mode, index);
    } else {
        return BaseType::vector_attribute(index);
    }
}


template <typename T>
auto CachingAccessor<T>::scalar_attribute(const long index) -> T&
{
    if (has_stack()) {
        return m_cache_stack->current_scope_ptr()->scalar_attribute(*this, m_mode, index);
    } else {
        return BaseType::scalar_attribute(index);
    }
}

template <typename T>
auto CachingAccessor<T>::const_vector_attribute(const long index) const -> ConstMapResult
{
    if (has_stack()) {
        return m_cache_stack->current_scope_ptr()->const_vector_attribute(*this, m_mode, index);
    } else {
        return BaseType::const_vector_attribute(index);
    }
}


template <typename T>
auto CachingAccessor<T>::const_scalar_attribute(const long index) const -> T
{
    if (has_stack()) {
        return m_cache_stack->current_scope_ptr()->const_scalar_attribute(*this, m_mode, index);
    } else {
        return BaseType::const_scalar_attribute(index);
    }
}

template <typename T>
auto CachingAccessor<T>::vector_attribute(const long index) const -> ConstMapResult
{
    return const_vector_attribute(index);
}
template <typename T>
T CachingAccessor<T>::scalar_attribute(const long index) const
{
    return const_scalar_attribute(index);
}
template class CachingAccessor<char>;
template class CachingAccessor<long>;
template class CachingAccessor<double>;
template class CachingAccessor<Rational>;
} // namespace wmtk::attribute
