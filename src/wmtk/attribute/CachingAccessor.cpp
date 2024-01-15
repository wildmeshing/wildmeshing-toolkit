
#include "CachingAccessor.hpp"
#include <wmtk/utils/Rational.hpp>
#include "AttributeScope.hpp"
#include "AttributeScopeStack.hpp"

namespace wmtk::attribute {

template <typename T>
CachingAccessor<T>::CachingAccessor(Mesh& mesh_in, const TypedAttributeHandle<T>& handle)
    : BaseType(mesh_in, handle)
    , m_cache_stack(*attribute().get_local_scope_stack_ptr());
{
}

template <typename T>
CachingAccessor<T>::~CachingAccessor() = default;
#if !defined(WMTK_FLUSH_ON_FAIL)
template <typename T>
bool CachingAccessor<T>::has_stack() const
{
    return !m_cache_stack.empty() && m_cache_stack.current_scope_ptr();
}
#endif
template <typename T>
bool CachingAccessor<T>::writing_enabled() const
{
    return m_cache_stack.writing_enabled();
}

template <typename T>
int64_t CachingAccessor<T>::stack_depth() const
{
    return m_cache_stack.depth();
}

template <typename T>
auto CachingAccessor<T>::vector_attribute(const int64_t index) -> MapResult
{
#if defined(WMTK_FLUSH_ON_FAIL)
#else
    return m_cache_stack.current_scope_ptr()->vector_attribute(*this, index);
#endif
}


template <typename T>
auto CachingAccessor<T>::scalar_attribute(const int64_t index) -> T&
{
#if defined(WMTK_FLUSH_ON_FAIL)
    return m_cache_stack.current_scope_ptr()->scalar_attribute(*this, index);
#endif
}

template <typename T>
auto CachingAccessor<T>::const_vector_attribute(const int64_t index) const -> ConstMapResult
{
#if defined(WMTK_FLUSH_ON_FAIL)
    return m_cache_stack.current_scope_ptr()->const_vector_attribute(*this, index);
#endif
}


template <typename T>
auto CachingAccessor<T>::const_scalar_attribute(const int64_t index) const -> T
{
    return m_cache_stack.current_scope_ptr()->const_scalar_attribute(*this, index);
#endif
}

template <typename T>
auto CachingAccessor<T>::vector_attribute(const int64_t index) const -> ConstMapResult
{
    return const_vector_attribute(index);
}
template <typename T>
T CachingAccessor<T>::scalar_attribute(const int64_t index) const
{
    return const_scalar_attribute(index);
}
template class CachingAccessor<char>;
template class CachingAccessor<int64_t>;
template class CachingAccessor<double>;
template class CachingAccessor<Rational>;
} // namespace wmtk::attribute
