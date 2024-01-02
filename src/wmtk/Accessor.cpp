#include "Accessor.hpp"
#include "Mesh.hpp"
#include "attribute/AttributeManager.hpp"
#include "attribute/AttributeScope.hpp"
#include "attribute/AttributeScopeStack.hpp"
#include "attribute/MeshAttributes.hpp"

namespace wmtk {
namespace {
constexpr static bool accessor_requires_caching(AttributeAccessMode mode)
{
    if (mode == AttributeAccessMode::Buffered) {
        return true;
    } else {
        return false;
    }
}
} // namespace

template <typename T, bool IsConst>
Accessor<T, IsConst>::Accessor(
    MeshType& mesh,
    const MeshAttributeHandle<T>& handle,
    AttributeAccessMode mode)
    : BaseType(const_cast<Mesh&>(mesh), handle)
    , m_mesh(mesh)
    , m_mode(mode)
{
    m_cache_stack = attribute().get_local_scope_stack_ptr();
}
template <typename T, bool IsConst>
int64_t Accessor<T, IsConst>::index(const Tuple& t) const
{
    return BaseType::index(m_mesh, t);
}

template <typename T, bool IsConst>
Accessor<T, IsConst>::~Accessor()
{
    // if constexpr (!IsConst) {
    //     if (m_cache) {
    //         m_cache->flush(*this);
    //     }
    // }
}

template <typename T, bool IsConst>
AttributeAccessMode Accessor<T, IsConst>::access_mode() const
{
    return m_mode;
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::cacheable_const_vector_attribute(const int64_t index) const
    -> ConstMapResult
{
    if (m_cache_stack && !m_cache_stack->empty()) {
        return m_cache_stack->current_scope_ptr()->const_vector_attribute(*this, m_mode, index);
    } else {
        return BaseType::const_vector_attribute(index);
    }
}


template <typename T, bool IsConst>
auto Accessor<T, IsConst>::cacheable_vector_attribute(const int64_t index) -> MapResultT
{
    if (m_cache_stack && !m_cache_stack->empty()) {
        if constexpr (IsConst) {
            return m_cache_stack->current_scope_ptr()->const_vector_attribute(*this, m_mode, index);
        } else {
            return m_cache_stack->current_scope_ptr()->vector_attribute(*this, m_mode, index);
        }
    }
    if constexpr (IsConst) {
        return BaseType::const_vector_attribute(index);
    } else {
        return BaseType::vector_attribute(index);
    }
}

template <typename T, bool IsConst>
T Accessor<T, IsConst>::cacheable_const_scalar_attribute(const int64_t index) const
{
    if (m_cache_stack && !m_cache_stack->empty()) {
        return m_cache_stack->current_scope_ptr()->const_scalar_attribute(*this, m_mode, index);
    } else {
        return BaseType::const_scalar_attribute(index);
    }
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::cacheable_scalar_attribute(const int64_t index) -> TT
{
    if (m_cache_stack && !m_cache_stack->empty()) {
        if constexpr (IsConst) {
            return m_cache_stack->current_scope_ptr()->const_scalar_attribute(*this, m_mode, index);
        } else {
            return m_cache_stack->current_scope_ptr()->scalar_attribute(*this, m_mode, index);
        }
    } else {
        if constexpr (IsConst) {
            return BaseType::const_scalar_attribute(index);
        } else {
            return BaseType::scalar_attribute(index);
        }
    }
}


//===================================================
// These following methods just forward to to const names
template <typename T, bool IsConst>
auto Accessor<T, IsConst>::vector_attribute(const Tuple& t) const -> ConstMapResult
{
    return const_vector_attribute(t);
}
template <typename T, bool IsConst>
T Accessor<T, IsConst>::scalar_attribute(const Tuple& t) const
{
    return const_scalar_attribute(t);
}
//===================================================


//===================================================
// These methods just compute the index and forward it
template <typename T, bool IsConst>
auto Accessor<T, IsConst>::const_vector_attribute(const Tuple& t) const -> ConstMapResult
{
    const int64_t idx = index(t);
    return cacheable_const_vector_attribute(idx);
}
template <typename T, bool IsConst>
auto Accessor<T, IsConst>::vector_attribute(const Tuple& t) -> MapResultT
{
    const int64_t idx = index(t);
    return cacheable_vector_attribute(idx);
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::const_vector_attribute(const int64_t index) const -> ConstMapResult
{
    return cacheable_const_vector_attribute(index);
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::vector_attribute(const int64_t index) const -> ConstMapResult
{
    return cacheable_const_vector_attribute(index);
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::vector_attribute(const int64_t index) -> MapResultT
{
    return cacheable_vector_attribute(index);
}


template <typename T, bool IsConst>
T Accessor<T, IsConst>::const_scalar_attribute(const Tuple& t) const
{
    const int64_t idx = index(t);
    return cacheable_const_scalar_attribute(idx);
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::scalar_attribute(const Tuple& t) -> TT
{
    const int64_t idx = index(t);
    return cacheable_scalar_attribute(idx);
}

template <typename T, bool IsConst>
T Accessor<T, IsConst>::const_scalar_attribute(const int64_t index) const
{
    return cacheable_const_scalar_attribute(index);
}

template <typename T, bool IsConst>
T Accessor<T, IsConst>::scalar_attribute(const int64_t index) const
{
    return cacheable_const_scalar_attribute(index);
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::scalar_attribute(const int64_t index) -> TT
{
    return cacheable_scalar_attribute(index);
}


template <typename T, bool IsConst>
std::optional<int64_t> Accessor<T, IsConst>::stack_depth() const
{
    if (m_cache_stack != nullptr) {
        return m_cache_stack->depth();
    } else {
        return {};
    }
}


template <typename T>
auto MutableAccessor<T>::vector_attribute(const int64_t index) -> MapResultT
{
    return cacheable_vector_attribute(index);
}


template <typename T>
auto MutableAccessor<T>::scalar_attribute(const int64_t index) -> TT
{
    return cacheable_scalar_attribute(index);
}
template class Accessor<char, true>;
template class Accessor<int64_t, true>;
template class Accessor<double, true>;
template class Accessor<char, false>;
template class Accessor<int64_t, false>;
template class Accessor<double, false>;
} // namespace wmtk
