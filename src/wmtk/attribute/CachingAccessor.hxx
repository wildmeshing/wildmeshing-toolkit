#pragma once

#include <wmtk/utils/Rational.hpp>
#include "CachingAccessor.hpp"

#if defined(WMTK_ENABLED_DEV_MODE)
#define WMTK_CACHING_ACCESSOR_INLINE
#else
#define WMTK_CACHING_ACCESSOR_INLINE inline
#endif
namespace wmtk::attribute {

template <typename T, int Dim>
CachingAttribute<T>& CachingAccessor<T, Dim>::get_cache_stack()
{
    return static_cast<CachingAttribute<T>&>(attribute());
}
template <typename T, int Dim>
const CachingAttribute<T>& CachingAccessor<T, Dim>::get_cache_stack() const
{
    //
    return static_cast<const CachingAttribute<T>&>(attribute());
}
template <typename T, int Dim>
WMTK_CACHING_ACCESSOR_INLINE CachingAccessor<T, Dim>::CachingAccessor(
    Mesh& mesh_in,
    const TypedAttributeHandle<T>& handle)
    : BaseType(mesh_in, handle)
{}
template <typename T, int Dim>
CachingAccessor<T, Dim>::CachingAccessor(const Mesh& mesh_in, const TypedAttributeHandle<T>& handle)
    : BaseType(mesh_in, handle)
{}

template <typename T, int Dim>
WMTK_CACHING_ACCESSOR_INLINE CachingAccessor<T, Dim>::~CachingAccessor() = default;

template <typename T, int Dim>
WMTK_CACHING_ACCESSOR_INLINE bool CachingAccessor<T, Dim>::has_stack() const
{
    return get_cache_stack().has_transactions();
}

template <typename T, int Dim>
WMTK_CACHING_ACCESSOR_INLINE bool CachingAccessor<T, Dim>::writing_enabled() const
{
    return get_cache_stack().writing_enabled();
}

template <typename T, int Dim>
WMTK_CACHING_ACCESSOR_INLINE int64_t CachingAccessor<T, Dim>::stack_depth() const
{
    return get_cache_stack().size();
}

template <typename T, int Dim>
template <int D>
WMTK_CACHING_ACCESSOR_INLINE auto CachingAccessor<T, Dim>::vector_attribute(const int64_t index)
    -> MapResult<std::max(D, Dim)>
{
    return get_cache_stack().template vector_attribute<std::max(D, Dim)>(index);
    // return BaseType::template vector_attribute<D>( index);
}


template <typename T, int Dim>
WMTK_CACHING_ACCESSOR_INLINE auto CachingAccessor<T, Dim>::scalar_attribute(const int64_t index)
    -> T&
{
    assert(Dim == Eigen::Dynamic || Dim == 1);
    return get_cache_stack().scalar_attribute(index);
}

template <typename T, int Dim>
template <int D>
WMTK_CACHING_ACCESSOR_INLINE auto CachingAccessor<T, Dim>::const_vector_attribute(
    const int64_t index) const -> ConstMapResult<std::max(D, Dim)>
{
    return get_cache_stack().template const_vector_attribute<std::max(D, Dim)>(index);
}


template <typename T, int Dim>
WMTK_CACHING_ACCESSOR_INLINE auto CachingAccessor<T, Dim>::const_scalar_attribute(
    const int64_t index) const -> const T&
{
    assert(Dim == Eigen::Dynamic || Dim == 1);
    return get_cache_stack().const_scalar_attribute(index);
}


template <typename T, int Dim>
WMTK_CACHING_ACCESSOR_INLINE auto CachingAccessor<T, Dim>::vector_single_value(
    const int64_t index,
    int8_t vector_index) -> T&
{
    return get_cache_stack().template vector_single_value<Dim>(index, vector_index);
}


template <typename T, int Dim>
WMTK_CACHING_ACCESSOR_INLINE auto CachingAccessor<T, Dim>::const_vector_single_value(
    const int64_t index,
    int8_t vector_index) const -> const T&
{
    return get_cache_stack().template const_vector_single_value<Dim>(index, vector_index);
}

} // namespace wmtk::attribute
#undef WMTK_CACHING_ACCESSOR_INLINE
