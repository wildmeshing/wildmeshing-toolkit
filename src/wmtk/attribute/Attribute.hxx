#pragma once
#include "Attribute.hpp"
#if defined(WMTK_ENABLED_DEV_MODE)
#define WMTK_ATTRIBUTE_INLINE
#else
#define WMTK_ATTRIBUTE_INLINE inline
#endif
namespace wmtk::attribute {

//=======================================================
// Scalar Attribute Access from arbitrary data
//=======================================================
template <typename T>
WMTK_ATTRIBUTE_INLINE const T& Attribute<T>::const_scalar_attribute(
    int64_t index,
    const std::vector<T>& data) const
{
    assert(index < reserved_size(data));
    assert(m_dimension == 1);
    return data[index];
}

template <typename T>
WMTK_ATTRIBUTE_INLINE T& Attribute<T>::scalar_attribute(int64_t index, std::vector<T>& data) const
{
    assert(index < reserved_size(data));
    assert(m_dimension == 1);
    return data[index];
}

//=======================================================
// Vector Attribute Access from arbitrary data without offset
//=======================================================
template <typename T>
template <int D>
WMTK_ATTRIBUTE_INLINE auto Attribute<T>::const_vector_attribute_without_stride(
    int64_t start,
    const std::vector<T>& data) const -> ConstMapResult<D>
{
    assert(m_dimension > 0);
    assert(D == Eigen::Dynamic || D == m_dimension);
    int64_t dim = D == Eigen::Dynamic ? m_dimension : D;
    ConstMapResult<D> R(data.data() + start, dim);

    assert(R.size() == m_dimension);

    return R;
}
template <typename T>
template <int D>
WMTK_ATTRIBUTE_INLINE auto Attribute<T>::vector_attribute_without_stride(
    int64_t start,
    std::vector<T>& data) const -> MapResult<D>
{
    assert(m_dimension > 0);
    assert(D == Eigen::Dynamic || D == m_dimension);
    int64_t dim = D == Eigen::Dynamic ? m_dimension : D;
    MapResult<D> R(data.data() + start, dim);
    assert(R.size() == m_dimension);
    return R;
}

//=======================================================
// Vector Attribute Access from arbitrary data
//=======================================================
template <typename T>
template <int D>
WMTK_ATTRIBUTE_INLINE auto Attribute<T>::const_vector_attribute(
    int64_t index,
    const std::vector<T>& data) const -> ConstMapResult<D>
{
    int64_t dim = D == Eigen::Dynamic ? m_dimension : D;
    assert(D == Eigen::Dynamic || D == m_dimension);
    assert(index < reserved_size(data));
    assert(data.size() % m_dimension == 0);
    const int64_t start = index * dim;
    return const_vector_attribute_without_stride<D>(start, data);
}
template <typename T>
template <int D>
WMTK_ATTRIBUTE_INLINE auto Attribute<T>::vector_attribute(int64_t index, std::vector<T>& data) const
    -> MapResult<D>
{
    assert(index < reserved_size(data));
    assert(data.size() % m_dimension == 0);
    assert(D == Eigen::Dynamic || D == m_dimension);
    int64_t dim = D == Eigen::Dynamic ? m_dimension : D;
    const int64_t start = index * dim;
    return vector_attribute_without_stride<D>(start, data);
}


//=======================================================
// Vector Attribute Access of single element from arbitrary data
//=======================================================
template <typename T>
template <int D>
WMTK_ATTRIBUTE_INLINE const T& Attribute<T>::const_vector_single_value(
    int64_t index,
    int8_t vector_index,
    const std::vector<T>& data) const
{
    assert(D == Eigen::Dynamic || D == m_dimension);
    int64_t dim = D == Eigen::Dynamic ? m_dimension : D;
    const int64_t idx = index * dim + vector_index;
    assert(index < reserved_size(data));
    return data[idx];
}
template <typename T>
template <int D>
WMTK_ATTRIBUTE_INLINE T&
Attribute<T>::vector_single_value(int64_t index, int8_t vector_index, std::vector<T>& data) const
{
    assert(D == Eigen::Dynamic || D == m_dimension);
    int64_t dim = D == Eigen::Dynamic ? m_dimension : D;
    const int64_t idx = index * dim + vector_index;
    assert(index < reserved_size(data));
    return data[idx];
}

//=======================================================
// Standard Scalar access
//=======================================================

template <typename T>
WMTK_ATTRIBUTE_INLINE const T& Attribute<T>::const_scalar_attribute(int64_t index) const
{
    return const_scalar_attribute(index, m_data);
}
template <typename T>
WMTK_ATTRIBUTE_INLINE T& Attribute<T>::scalar_attribute(int64_t index)
{
    return scalar_attribute(index, m_data);
}


//=======================================================
// Standard vector access
//=======================================================
template <typename T>
template <int D>
WMTK_ATTRIBUTE_INLINE auto Attribute<T>::const_vector_attribute(int64_t index) const
    -> ConstMapResult<D>
{
    return const_vector_attribute<D>(index, m_data);
}


template <typename T>
template <int D>
WMTK_ATTRIBUTE_INLINE auto Attribute<T>::vector_attribute(int64_t index) -> MapResult<D>
{
    return vector_attribute<D>(index, m_data);
}


//=======================================================
// Standard vector single value access
//=======================================================
template <typename T>
template <int D>
WMTK_ATTRIBUTE_INLINE const T& Attribute<T>::const_vector_single_value(
    int64_t index,
    int8_t vector_index) const
{
    return const_vector_single_value<D>(index, vector_index, m_data);
}

template <typename T>
template <int D>
WMTK_ATTRIBUTE_INLINE T& Attribute<T>::vector_single_value(int64_t index, int8_t vector_index)
{
    return vector_single_value<D>(index, vector_index, m_data);
}


//=======================================================
// Simple getters
//=======================================================
template <typename T>
WMTK_ATTRIBUTE_INLINE int64_t Attribute<T>::dimension() const
{
    return m_dimension;
}

template <typename T>
WMTK_ATTRIBUTE_INLINE const T& Attribute<T>::default_value() const
{
    return m_default_value;
}


} // namespace wmtk::attribute
#undef WMTK_ATTRIBUTE_INLINE
