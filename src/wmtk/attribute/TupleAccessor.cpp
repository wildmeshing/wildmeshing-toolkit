#include "TupleAccessor.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::attribute {

template <typename T>
auto TupleAccessor<T>::const_vector_attribute(const Tuple& t) const -> ConstMapResult
{
    const int64_t idx = index(t);
    return CachingBaseType::const_vector_attribute(idx);
}

template <typename T>
auto TupleAccessor<T>::vector_attribute(const Tuple& t) -> MapResult
{
    const int64_t idx = index(t);
    return CachingBaseType::vector_attribute(idx);
}

template <typename T>
auto TupleAccessor<T>::scalar_attribute(const Tuple& t) -> T&
{
    const int64_t idx = index(t);
    return CachingBaseType::scalar_attribute(idx);
}

template <typename T>
T TupleAccessor<T>::const_scalar_attribute(const Tuple& t) const
{
    const int64_t idx = index(t);
    return CachingBaseType::const_scalar_attribute(idx);
}
template <typename T>
int64_t TupleAccessor<T>::index(const Tuple& t) const
{
    assert(mesh().is_valid_slow(t));
    return mesh().id(t, BaseType::primitive_type());
}
template class TupleAccessor<char>;
template class TupleAccessor<int64_t>;
template class TupleAccessor<double>;
template class TupleAccessor<Rational>;
} // namespace wmtk::attribute
