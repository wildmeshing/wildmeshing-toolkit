#pragma once
#include <wmtk/utils/Rational.hpp>
#include "AttributeCacheData.hpp"
namespace wmtk::attribute {
template <typename T>
inline auto AttributeCacheData<T>::data_as_map() -> typename Vector::MapType
{
    return typename Vector::MapType(data.data(), data.size());
}
template <typename T>
inline auto AttributeCacheData<T>::data_as_const_map() const -> typename Vector::ConstMapType
{
    return typename Vector::ConstMapType(data.data(), data.size());
}

} // namespace wmtk::attribute
