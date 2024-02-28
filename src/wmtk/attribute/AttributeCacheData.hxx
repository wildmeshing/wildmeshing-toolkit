#pragma once
#include <wmtk/utils/Rational.hpp>
#include "AttributeCacheData.hpp"
namespace wmtk::attribute {
template <typename T>
template <int D>
inline auto AttributeCacheData<T>::data_as_map() -> typename VectorD<D>::MapType
{
    return typename VectorD<D>::MapType(data.data(), data.size());
}
template <typename T>
template <int D>
inline auto AttributeCacheData<T>::data_as_const_map() const -> typename VectorD<D>::ConstMapType
{
    return typename VectorD<D>::ConstMapType(data.data(), data.size());
}

} // namespace wmtk::attribute
