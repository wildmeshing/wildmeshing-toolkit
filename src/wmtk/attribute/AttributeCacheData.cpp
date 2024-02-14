#include "AttributeCacheData.hpp"
#include <wmtk/utils/Rational.hpp>
namespace wmtk {
template <typename T>
auto AttributeCacheData<T>::data_as_map() -> typename Vector::MapType
{
    return typename Vector::MapType(data.data(), data.size());
}
template <typename T>
auto AttributeCacheData<T>::data_as_const_map() const -> typename Vector::ConstMapType
{
    return typename Vector::ConstMapType(data.data(), data.size());
}

template class AttributeCacheData<int64_t>;
template class AttributeCacheData<double>;
template class AttributeCacheData<char>;
template class AttributeCacheData<Rational>;
} // namespace wmtk
